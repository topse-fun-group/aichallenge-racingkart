# ADR-029: 寄せ側判定のデッドバンド + dwell + ラッチ（ステアリングチャタリング対策）

## ステータス
承認済み (Accepted)

## コンテキスト

FollowState 走行中にステアリングが左右へ大きく振れ、壁に衝突するケースが発生していた。

### 原因: 寄せ側の判定がデッドバンドなしの厳密比較

寄せ側を決めているのは以下の 2 箇所で、どちらも**毎 tick 再評価**される。

```python
# mpc_controller.py  _compute_v2x_overtake_corridor
if avail_left >= avail_right:  target_offset = clip(offset_left,  0.8,  2.5)
else:                          target_offset = clip(offset_right, -2.5, -0.8)

# mpc_controller.py  _compute_waypoint_shift_pure_pursuit_control
is_left = (ctx.overtake_width_left >= ctx.overtake_width_right)
target_offset = ((left - 0.5)/2 + 0.95) if is_left else -((right - 0.5)/2 + 0.95)
```

左右の空き幅の差は、先行車の横偏差の **2 倍**で効く。

```text
avail_left  = ub - 1.3 - e_y_leader
avail_right = e_y_leader - 1.3 - lb
------------------------------------------------
avail_left - avail_right = (ub + lb) - 2 * e_y_leader
```

対称路（`max_width: 6.0` → `ub ≈ +3.0`, `lb ≈ -3.0`）で先行車がセンターライン付近にいると
`avail_left ≈ avail_right ≈ 1.7` の同点になり、V2X の位置ノイズ（σ ≈ 0.1 m）だけで符号が反転する。

```text
            e_y_leader ≈ 0 のとき
  左端                  中央                  右端
   |----------------------|----------------------|
              +1.55m   <--X-->   -1.55m
              ^^^^^^^^^^^^^^^^^^^^^^^^
              毎 tick この 3.1m をジャンプ
```

反転 1 回あたりの操舵指令の変化:

| 項目 | 値 |
|---|---|
| 横目標のジャンプ | 3.10 m (±1.55 m) |
| 注視点距離 `lookahead` | 6.217 m（速度によらず固定） |
| 注視点方位の変化 | `2 * atan(1.55 / 6.217)` ≈ 27.7 deg |
| `steering_tire_angle` | ≈ ±0.125 rad |
| publish 値（`steering_tire_angle_gain_var = 1.639` 乗算後） | **±0.205 rad** |
| 発生周期 | 40 Hz（毎 tick 交互） |

合計スイング約 0.41 rad（23.5 deg）が 40 Hz で出続ける。

### 悪化要因: 通常経路にステアの時間平滑化が一切ない

`steer_low_pass_gain: 0.4` は `override is not None` ブロック（RecoveryState 専用）でしか
適用されていない。通常経路（follow_path / follow / overtake）では `_last_u[1]` は
書き込まれるだけで読まれず、Pure Pursuit の生値がそのまま publish される。
ADR-022/023 の PD センタリング（左右切り返しのダンピング）も削除済みで、現在ダンピングは不在。

FollowState は `min_forward_overtake_width >= MIN_OVERTAKE_WIDTH_M` のとき車間を `D0_M` まで
詰める（ADR-028）ため、**最も接近した状態でこの操舵反転が入る**のが壁衝突の最有力経路である。

## 決定事項

### 1. 二段閾値 + ラッチ + dwell による寄せ側の決定

`states.LateralShiftSideFilter` を新設し、左右の空き幅差から `"left" / "right" / "none"` を返す。

```text
diff = overtake_width_left - overtake_width_right

  diff <-0.4        -0.4 .. -0.2   -0.2 .. +0.2   +0.2 .. +0.4        +0.4< diff
 ---------------+----------------+--------------+----------------+---------------
   候補 "right" |   現状維持     |  候補 "none" |    現状維持    |  候補 "left"
                |   (ラッチ)     |              |    (ラッチ)    |

候補が LATERAL_SHIFT_DWELL_SEC (0.1s) 継続して初めて確定する。
1 tick でも候補が変われば計時をやり直す。
```

| 定数 | 既定値 | 意味 |
|---|---|---|
| `LATERAL_SHIFT_ENTER_DIFF_M` | 0.4 m | 寄せを開始する左右空き幅の差 |
| `LATERAL_SHIFT_EXIT_DIFF_M` | 0.2 m | センターラインへ戻す左右空き幅の差 |
| `LATERAL_SHIFT_DWELL_SEC` | 0.1 s | 判定が継続すべき時間（40 Hz で 4 tick） |

- **デッドバンド内（`|diff| < 0.2`）は `target_offset = 0.0`**、つまりセンターラインを走る。
- `0.2 〜 0.4` の帯は現状維持（ラッチ）。閾値ちょうど付近での ON/OFF チャタリングを防ぐ。
- 反対側が `0.4` を超えたときは `"none"` を経由せず直接反転する。
- 先行車も側方車も検知されないとき `_compute_v2x_overtake_corridor` は `(0, 0, 0)` を返すので
  `diff = 0` → `"none"` となり、自然にセンターラインへ戻る。

閾値 0.4 m の根拠: 幅差は横偏差の 2 倍で効くので、V2X の位置ノイズ σ ≈ 0.1 m は
幅差 σ ≈ 0.2 m 相当になる。開始閾値はその 2σ を取った。

### 2. 適用範囲は FollowState のみ

`ctx.lateral_shift_side` は常に計算するが、`_compute_waypoint_shift_pure_pursuit_control` に
渡すのは FollowState のときだけ。OvertakeState は従来どおり毎 tick の幅比較で決める。

```python
shift_side = ctx.lateral_shift_side if isinstance(current_state, FollowState) else None
v_target, steer_target = self._compute_waypoint_shift_pure_pursuit_control(pose, v, ctx, shift_side)
```

### 3. デッドバンド帯は追い越し機会を潰さない

デッドバンド帯（`|diff| < 0.4 m` ⟺ 対称路で `|e_y_leader| < 0.2 m`）は、
**追い越しが幾何的に成立しない領域と一致する**。

```text
avail_left >= MIN_OVERTAKE_WIDTH_M (3.2)
  <=> ub - 1.3 - e_y_leader >= 3.2
  <=> ub - e_y_leader >= 4.5
  ub ≈ 3.0 のとき e_y_leader <= -1.5 m
  このとき diff = (ub + lb) - 2*e_y_leader ≈ +3.0 m  >>  0.4 m
```

つまり追い越しが可能な場面では幅差は常にデッドバンドの遥か外側にあり、
センターライン走行に落とす帯で追い越しの機会を失うことはない。

### 4. 側の決定だけを共通化し、オフセットの大きさの式は温存

`_compute_waypoint_shift_pure_pursuit_control` のオフセット絶対値
`((w - 0.5) / 2.0) + 0.725 + 0.225` は現行チューニングを維持するためそのまま残し、
「左 / 右 / なし」の判定だけを差し替えた。

「車がいなければセンターライン」を担っていた `abs(ctx.target_overtake_offset) > 0.1` ガードは
OvertakeState 経路に移植して残す。`_compute_v2x_overtake_corridor` は `±0.8` にクリップするので
車を検知している限りこのガードは成立せず、従来どおりの挙動になる。
**このガードを落とすと幅 0 のとき `((0 - 0.5)/2) + 0.95 = 0.7 m` のオフセットが出てしまう。**

### 5. 横オフセットのレートリミットは入れない

判定ロジックだけで実走し、効果を見てから判断する（下記「未解決事項」参照）。

## 実装

| ファイル | 変更 |
|---|---|
| `states.py` | 定数 3 つ、`StateContext.lateral_shift_side`、`LateralShiftSideFilter` クラス |
| `mpc_controller.py` | `self._shift_side_filter` の保持、`_build_state_context` での更新、`_compute_waypoint_shift_pure_pursuit_control` の `shift_side` 引数、`STATE_PARAM_MAP` に 3 行 |

判定ロジックを `MPCController` のメソッドではなく `states.py` のクラスに置いたのは、
`mpc_controller.py` が `autoware_auto_control_msgs` を import するため単体テストできないから。
`states.py` は ROS メッセージの ImportError フォールバックを持つ純 Python で、rclpy なしにテストできる。

閾値は `LateralShiftSideFilter.update` 内で**モジュールグローバルとして参照する**こと。
ROS パラメータは `setattr(states, ...)` でモジュール属性を差し替えるため、
ローカル変数やデフォルト引数に取り込むと動的更新が効かなくなる。

## 関連 ADR

| ADR | 関係 |
|---|---|
| ADR-024 | `overtake_width_left >= overtake_width_right` による左右判定の出典。本 ADR はこの比較にデッドバンドと時間ヒステリシスを追加する |
| ADR-022 / ADR-023 | 左右切り返しのオーバーシュートを PD ダンピングで抑えた先例。**コードは削除済み**（`ControlMode.MPC` ゲート内にあり一度も実行されていなかった） |
| ADR-010 | `steer_low_pass_gain` を機敏化のため意図的に弱めた経緯。単純にゲインを戻すと追い越しの回避速度が落ちる |
| ADR-008 | 「1 フレームでも V2X が外れると即遷移」を時間ヒステリシスで解決した先例。`FOLLOW_CLEAR_HYSTERESIS_SEC` の出典であり、本 ADR の dwell も同じ型 |
| ADR-028 | FollowState の車間 PD。`D0_M` まで詰める場面でこの操舵反転が最も危険になる |

## 未解決事項

1. **通常経路のステアに時間平滑化が一切ない。**
   `_last_u[1]` は書き込まれるだけで読まれておらず、`u = [v_target, steer_target]` の直後に
   1 行足せばローパスを挿せる下地はある。今回はレートリミットを入れない判断をしたため、
   効果不足なら次の一手として残す。ADR-010 の機敏性要求とのトレードオフに注意。
   なお `config.yaml` の `steer_rate_max: 0.35` は `MPCConfig` にフィールドが無く読み込まれていない。

2. **`states.py` の遷移条件は独立に幅比較しており、演算子が食い違っている。**
   `is_left`（`FollowPathState` / `FollowState` の `check_transition`）は `>`（同点 = 右）、
   `mpc_controller` 側は `>=`（同点 = 左）。さらに `has_future_width` は `if / elif` の
   どちらも成立せず `False` 固定になり、完全同幅のとき追い越しを恒久的にブロックする。
   今回は遷移挙動を変えないため触らない。`ctx.lateral_shift_side` に一本化するのが本筋。

3. **`OvertakeState.on_enter` の `_calculated_offset` / `_overtake_side` はデッドコード。**
   MPC 撤去に伴い `_apply_state_params` が `lateral_offset` を読み捨てているため、
   「入場時に側をラッチしている」ように見えて実際の走行には一切効かない。

## 検証

- ユニットテスト `test/test_lateral_shift_side_hysteresis.py`（純 Python、rclpy 不要）
  - 初期状態は `"none"` / dwell 未満は確定しない / dwell 到達で確定
  - `0.2 〜 0.4` の帯でのラッチ / 解除閾値でのセンターライン復帰
  - `"none"` を経由しない直接反転
  - 1 tick おきに符号が反転するノイズで一度も確定しないこと
  - 先行車消失（幅 0/0）でセンターラインへ戻ること
  - 閾値がモジュールグローバルから呼び出し時に読まれること
- 実走: 先行車がセンターライン付近にいる場面で `lateral.steering_tire_angle` が
  左右にフルスイングしないこと。OvertakeState の挙動が変わっていないこと。
