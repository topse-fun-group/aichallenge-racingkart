# ADR-028: FollowState における車間距離 PD 制御と縦方向指令への配線

## ステータス
承認済み (Accepted)

## コンテキスト

FollowState（先行車追従）で自車が先行車に追突する事象が発生していた。原因は 2 つある。

### 1. PD のギャップ項が無効化されていた

`states.py` の `FollowState.get_adjusted_v_max_mps()` は constant-time-headway 方式
（ADR-004 系）を意図していたが、ギャップ誤差項がコメントアウトされ、相対速度ダンピング
項だけが残った縮退状態だった。

```text
v_cmd = v_lead + FOLLOW_K_V * (v_lead - v_ego)
```

この式は「先行車と同じ速度に収束する」だけで、車間距離を目標値へ引き戻す力を持たない。
一度詰まった車間はそのまま維持され、先行車が減速すると追突する。

### 2. そもそも速度指令に反映されていなかった（実害の本体）

`get_adjusted_v_max_mps()` の返り値は `set_v_ref()` 経由で `waypoint.v_ref` に書き込まれる。
ところが FollowState の `control_mode` は ADR-024 で `WAYPOINT_SHIFT_PURE_PURSUIT` に変更されて
おり、`_compute_waypoint_shift_pure_pursuit_control()` は `wp.v_ref` を一切読まず
`kmh_to_m_per_sec(35.0)` を固定で返す。`v_ref` を読むのは `_compute_pure_pursuit_control()`
（= FollowPathState 専用経路）だけである。

```text
get_adjusted_v_max_mps()
  -> set_v_ref()  -> waypoint.v_ref
                        |
                        +--> _compute_pure_pursuit_control()          (FollowPath) 読む
                        +--> _compute_waypoint_shift_pure_pursuit_...  (Follow/Overtake) 読まない
                                                                        └ 35 km/h 固定
```

結果として FollowState 中の `longitudinal.speed` は常に 35 km/h であり、車間制御は一度も
効いていなかった。加速度は `acc = KP * (u[0] - v)`（`KP = 100.0`）でほぼバンバン制御なので、
`u[0]` を正しく落とせばそのまま最大減速が出る構造になっている。

## 決定事項

### 1. 車間距離 PD 制御

`FollowState.get_adjusted_v_max_mps()` を単一の PD 則に統一する。

```text
gap   = max(0, forward_vehicle_distance - VEHICLE_LENGTH)
d_des = D0_M                      if min_forward_overtake_width >= MIN_OVERTAKE_WIDTH_M
        FOLLOW_TARGET_DISTANCE_M  otherwise
v_cmd = clip(v_lead + FOLLOW_K_GAP * (gap - d_des) + FOLLOW_K_V * (v_lead - v_ego),
             0, VEHICLE_V_MAX / 3.6)
```

| 記号 | 定数 | 既定値 | 意味 |
|---|---|---|---|
| `gap` | — | — | バンパー間ギャップ [m] |
| `d_des` | `FOLLOW_TARGET_DISTANCE_M` | 3.0 m | 追い越し不可時の目標車間 |
| `d_des` | `D0_M` | 1.0 m | 追い越し可能時の目標車間 |
| P ゲイン | `FOLLOW_K_GAP` | 1.4 [1/s] | 車間誤差 → 速度 |
| D ゲイン | `FOLLOW_K_V` | 0.5 [-] | 相対速度ダンピング |
| 幅閾値 | `MIN_OVERTAKE_WIDTH_M` | 3.2 m | 目標車間を切り替える追い越し幅 |

- **距離の基準はバンパー間ギャップ**。`forward_vehicle_distance` は ADR-027 の
  参照経路沿い累積アーク長差（中心間距離）なので、車体全長 `VEHICLE_LENGTH` を引く。
- **追い越し幅の判定は `min_forward_overtake_width`**（前方複数車にわたる最小の追い越し幅）。
  幅が確保できている場面では `D0_M` まで車間を詰めて追い越しの助走を作り、幅が無い場面では
  `FOLLOW_TARGET_DISTANCE_M` の安全車間を保つ。
- **time headway 項は加算しない**。`d_des` は固定値のみとし、パラメータの意味を明確にして
  チューニングを容易にする。
- 停止判定は別途設けない。詰まりすぎると `v_cmd` が負になり `clip` により 0（フルブレーキ）
  に落ちるため、PD 則だけで停止まで連続的に扱える。
- `gap == d_des` かつ `v_ego == v_lead` のとき `v_cmd == v_lead` となり、定常偏差は残らない。

### 2. 縦方向指令への配線

`mpc_controller._control()` で、FollowState のときだけ `v_target` を PD 出力で上書きする。

```text
v_target, steer_target = _compute_waypoint_shift_pure_pursuit_control(...)   # 35 km/h 固定
if follow_target_speed_mps is not None:      # FollowState のときのみ非 None
    v_target = follow_target_speed_mps
u = [v_target, steer_target]
```

- **ステア計算には手を入れない。** `_compute_waypoint_shift_pure_pursuit_control` の
  `lookahead_distance` は 35 km/h ベースのままとし、ADR-024 以降の操舵チューニングを維持する。
  減速時に lookahead を縮めると操舵が鋭くなり、既存の走行挙動が変わってしまう。
- OvertakeState も `WAYPOINT_SHIFT_PURE_PURSUIT` だが `follow_target_speed_mps` は None のままな
  ので 35 km/h 固定を維持する（追い越し中は減速させない）。
- `set_v_ref()` の呼び出しは残す。FollowPath へ戻った直後の 1 tick 分の整合のためで、害はない。

### 3. 単位バグの修正

`forward_vehicle_distance is None` の分岐が `VEHICLE_V_MAX`（35.0, km/h）を生で返していた。
他の分岐がすべて m/s を返す中でこれだけ km/h であり、そのまま m/s として消費されると
126 km/h 相当になる。呼び出し側のガードにより到達不能なデッドコードだったが、
`VEHICLE_V_MAX / 3.6` に統一し、docstring の `[km/h]` も `[m/s]` に訂正した。

## ADR-004 / 005 / 006 との関係

ADR-004・005・006 は距離依存の 3 段ガバナー（停止域 / 線形制限域 / 追従域）を定めていたが、
実装されないまま定数だけが縮小されて残っていた。本 ADR はこれを **単一の PD 則に置き換える**。

| | 3 段ガバナー (ADR-004/005/006) | 本 ADR |
|---|---|---|
| 構造 | 距離域ごとに別式へ切り替え | 全域で 1 本の PD 則 |
| 切り替え | 距離による 3 分岐 | 目標車間 `d_des` の二値切り替えのみ |
| 停止 | 明示的な停止域 | `clip(..., 0, ...)` により連続的に到達 |
| チューニング対象 | 域ごとの境界距離と係数 | `FOLLOW_K_GAP` / `FOLLOW_K_V` / `d_des` の 3 つ |

境界をまたぐたびに指令が不連続に飛ぶ 3 段方式より、単一の PD 則のほうが挙動が読みやすく、
かつ全定数が ROS パラメータ化済み（`follow_k_gap` / `follow_k_v` / `follow_target_distance_m` /
`d0_m` / `min_overtake_width_m`）で再ビルド無しにチューニングできる。

## 実装上の制約

定数は必ず**モジュールグローバルとして関数内で参照する**こと。
`mpc_controller._setup_parameters_callback` が `setattr(states, "FOLLOW_K_GAP", ...)` で
モジュール属性を書き換えるため、デフォルト引数やクラス属性にキャプチャすると
ROS パラメータによる動的更新が効かなくなる。ユニットテストでこの性質を担保している。

## 未解決事項

`StateContext.forward_vehicle_gap` は `_build_state_context` でどこにも代入されておらず、
**常に 0.0** である。これに依存する `check_transition` 内の TTC 判定および
`is_settled_behind` 判定（`states.py` の follow_path→overtake / follow→overtake 条件）は
gap = 0 前提で動作している（例: `is_settled_behind = 0.0 <= D0_M + 0.35*v + 1.0` は常に真）。

本 ADR ではこのフィールドを埋めず、`get_adjusted_v_max_mps()` 内でギャップをローカル計算した。
埋めると follow ↔ overtake の遷移挙動が全面的に変わり、本 ADR の追突対策と切り分けが
できなくなるためである。遷移条件の見直しは別 ADR で扱う。

## 検証

- ユニットテスト `test/test_follow_state_gap_pd.py`（純 Python、rclpy 不要）
  - 目標車間での整定（定常偏差なし）／接近時の 0 指令／離間時の加速
  - バンパー重なり時に負の指令を出さないこと
  - 相対速度ダンピングが接近時に効くこと／上限クリップ
  - 追い越し幅ありで目標が `D0_M` に切り替わること
  - 先行車なしで `VEHICLE_V_MAX / 3.6` を返すこと（単位バグの回帰防止）
  - ゲインがモジュールグローバルから呼び出し時に読まれること
- 実走: FollowState 中の `/control/command/control_cmd` の `longitudinal.speed` が
  35 km/h 固定から離れ、先行車速度付近に追従すること。
