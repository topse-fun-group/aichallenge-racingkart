# ADR-030: 整列判定による復帰の早期離脱と復帰後 boost

## ステータス
承認済み (Accepted)

## コンテキスト

衝突で停止したあと通常走行に戻るまでが遅い。`RecoveryState` は**時間だけ**で進行し、
姿勢や経路偏差を一切見ていないため、姿勢がほとんど崩れていない軽い接触でも
シーケンスを最後まで走り切っていた。

```text
wait (0.5s) --> back (1.0s) --> forward_turn (1.0s) --> follow_path
                                                        合計 2.5s 固定
```

左右の切り返し方向（左にいれば左へ切って後退 → 右へ切って前進、右は逆 / ADR-015 の符号規則）は
すでに正しく実装されており、変更しない。足すのは「もう向きが整っているなら次へ進む」だけ。

### 早期離脱を入れるだけでは動かない — 2 つの障害

#### 1. `MIN_DWELL_TIME` が recovery から抜ける方向にも効いていた

```python
# state_manager.py (変更前)
if next_name != "recovery" and self._last_transition_time is not None:
    if (ctx.current_time_sec - self._last_transition_time) < self.MIN_DWELL_TIME:  # 1.0s
        return None
```

recovery は**入る方向だけ**が例外扱いだった。抜ける方向は 1.0 s の dwell を必ず食らうため、
早期離脱条件が 50 ms で成立しても状態機械が遷移を握り潰す。その間 `forward_turn` の指令
（+7 m/s、±0.55 rad の大舵角前進）が出続け、コースを外れる。

#### 2. `is_colliding` の 2.5 s ラッチによる再突入ループ

```text
condition トピックの差分 > 30  -->  _last_colliding_time = now
                                     is_colliding = True が 2.5s ラッチ
```

`FollowPathState` / `FollowState` / `OvertakeState` はいずれも
`if ctx.is_colliding: return "recovery"` を**ノーガードで**返す。`is_in_recovery_cooldown` は
スタック検知側にしか掛かっていない（ADR-026 に明文化された意図的な設計）。
さらに `StateManager` は recovery への遷移を dwell から除外している。

つまり**現状ループしていなかったのは、復帰の総時間 2.5 s と衝突ラッチ 2.5 s が
たまたま一致していたから**にすぎず、余裕は 1 制御 tick（25 ms）しかなかった。
早期離脱を入れた瞬間に無限ループになる。

```text
        変更前の危うい均衡
  t=0    衝突                 ラッチ ON  ================== 2.5s ================== OFF
  t=0+   recovery 突入        wait 0.5 | back 1.0 | forward 1.0 |
  t=2.5  recovery 離脱                                          ^ ここが 1 tick でもズレると再突入
```

## 決定事項

### 1. 整列判定によるフェーズの早期終了

`back` / `forward_turn` はいずれも次の条件が成立した時点で次へ進む。

```text
|経路との角度差| < RECOVERY_ALIGNED_HEADING_DEG (10 deg)
                かつ
|path_e_y|      < RECOVERY_ALIGNED_E_Y_M       (1.0 m)
```

| 定数 | 既定値 | 意味 |
|---|---|---|
| `RECOVERY_ALIGNED_HEADING_DEG` | 10.0 deg | 復帰完了とみなす経路との角度差 |
| `RECOVERY_ALIGNED_E_Y_M` | 1.0 m | 復帰完了とみなすセンターラインからの距離 |
| `RECOVERY_BOOST_VALUE` | 1.5 | 復帰後の boost 値（`OvertakeState` と同値） |
| `RECOVERY_BOOST_DURATION_SEC` | 2.0 s | 復帰後に boost を維持する時間 |

- 角度差は `ctx.pose_theta` と `ctx.path_psi` から出す。
  `ctx.path_deviation` は `_build_state_context` で代入されておらず**常に 0.0** なので使えない。
  横偏差は `ctx.path_e_y` を使う。
- 整列済みなら `back → forward_turn → follow_path` を各 1 tick で通過し、
  **約 50 ms で復帰**する（決定 7 で wait フェーズを削除した後の値）。
  整列しなければ `BACK_DURATION_TIME_SEC + FORWARD_DURATION_TIME_SEC` で抜ける。
- `WAIT_DURATION_TIME_SEC` はまず 0.5 → 0.0 にしたが、それだけではフェーズ機構が
  1 tick を消費する。**決定 7 で wait フェーズごと削除**した。

### 2. フェーズごとの経過時間

変更前は `elapsed` が recovery 入場からの絶対時間で、閾値も累積値（`WAIT + BACK` など）だった。
早期離脱で `back` を早く抜けると `forward_turn` に本来より長い時間が割り当てられてしまうため、
`_phase_start_time` を持たせ、各フェーズの経過をフェーズ開始基準で測る。

### 3. `MIN_DWELL_TIME` を recovery の**両方向**で例外にする

```python
if (next_name != "recovery"
        and self._current.name != "recovery"
        and self._last_transition_time is not None):
```

recovery への再突入は下記のラッチクリアとスタック検知のクールダウンで守られているため、
この緩和でチャタリングは増えない。

### 4. 退出時に衝突ラッチをクリアする

```python
if prev_state_name == "recovery" and current_state_name != "recovery":
    self._last_recovery_exit_time = ...
    self._last_colliding_time = None   # <-- 追加
```

復帰後に本当に再衝突すれば `condition` トピックが再びラッチを立てるので、検知能力は落ちない。
「衝突からの復帰を終えた」という事実で衝突イベントを消費する、という意味づけになる。

### 5. 加速度を config 準拠にする（**決定 9 で撤回済み**）

> この決定は実走の結果「復帰の立ち上がりが遅い」と判断され、**決定 9 で撤回**された。
> 現在の値は 500.0 m/s²。以下は経緯の記録として残す。

```python
RECOVERY_FORWARD_ACCEL_MPSS = 3.0  # = config.yaml mpc.a_max
RECOVERY_BACK_ACCEL_MPSS    = 3.5  # = |config.yaml mpc.a_min| (override 側で abs())
```

override 経路には `np.clip(acc, a_min, a_max)` が**掛からない**（override ブロックが
早期 return するため）ので、ここの値がそのまま publish される。変更前の 6.0 は
`a_max = 3.0` の 2 倍だった。

**トレードオフ**: 「加速度を最大にしたい」という当初の要望に対し、この値は変更前の 6.0 より
**小さい**。復帰の立ち上がりは変更前より遅くなる。ROS パラメータ化していないので、
調整はこの定数の直接編集で行う。

### 6. 退出時に boost を ON、時間で OFF

ON は `RecoveryState.on_exit` が `ctx.publish_boost(RECOVERY_BOOST_VALUE)` で出す
（`OvertakeState` と同じパターン）。OFF は **mpc_controller 側のデッドライン**が出す。

```python
# recovery 退出時
self._recovery_boost_off_time = ctx.current_time_sec + states.RECOVERY_BOOST_DURATION_SEC

# 毎 tick
if self._recovery_boost_off_time is not None and now >= self._recovery_boost_off_time:
    if not isinstance(current_state, states.OvertakeState):
        self._publish_boost(0.0)
    self._recovery_boost_off_time = None
```

**OFF を `FollowPathState` 側に置かない理由**: recovery は `follow_path` へ抜けるが、その直後に
`follow` / `overtake` へ移ると `FollowPathState.check_transition` が呼ばれなくなり、boost が
入りっぱなしになる。制御ループ側のデッドラインなら状態に依存せず必ず切れる。

`OvertakeState` 中はスキップする。追い越しの boost を復帰タイマーが横から消さないためで、
`OvertakeState.on_exit` が 0.0 を出して閉じる。

### 7. wait フェーズの削除（追記・2 段階復帰へ）

早期離脱を入れたあとも、**突入した tick に 1 tick 分の全停止指令が漏れていた**。
`StateManager.update()` は遷移した tick では新しい状態の `check_transition` を呼ばないため、
`on_enter` が `_phase = "wait"` を置くと、その tick の `get_control_override` は
`(0.0, 0.0, 0.0)` を返し、`gear` も `GEAR_DRIVE` のままになる。

```text
変更前
  tick N   : 衝突検知 -> recovery 突入, on_enter -> phase="wait"
             publish: speed=0.0, steer=0.0, acc=0.0, gear=DRIVE   <-- 25ms 無駄
  tick N+1 : check_transition -> phase="back"
             publish: speed<0, steer=±LOCK, acc>0, gear=REVERSE

変更後
  tick N   : 衝突検知 -> recovery 突入, on_enter -> phase="back"
             publish: speed<0, steer=±LOCK, acc>0, gear=REVERSE
```

`WAIT_DURATION_TIME_SEC` を 0.0 にするだけでは足りない（フェーズ機構が 1 tick を消費する）
ため、**wait フェーズごと削除**した。フェーズは `back` / `forward_turn` の 2 つだけになる。

### 8. 突入レイテンシには削り代が無い（調査結果）

同じ検討を繰り返さないための記録。`_control()` は先頭で `now` を取り、直後に
`self._control_rate.sleep()` で 1 周期ブロックしてから状態機械を回す。
`is_colliding` はこの 25 ms 古い `now` で判定されるが、sleep 中に来た衝突は
`elapsed` が負になって `< 2.5` を満たすため取りこぼしはない。
`MIN_DWELL_TIME` も recovery への遷移を除外している。

**したがって衝突メッセージ受信から recovery 突入までは最悪 1 制御 tick（25 ms）で、
状態機械側に削れる遅延は無い。**

残っていたのは `now` の古さが `_publish_control_command(now, ...)` のヘッダスタンプに
乗る点だけなので、`now` の取得を `sleep()` の後ろへ移した。**突入レイテンシを縮める
変更ではなく、指令スタンプの 25 ms の古さを直すもの**である。

### 9. 速度・加速度を車両側の上限まで出し切る

| 定数 | 値 | 根拠 |
|---|---|---|
| `RECOVERY_FORWARD_TURN_SPEED_MPS` | +30.0 m/s | override 経路は速度をそのまま `longitudinal.speed` に載せる |
| `RECOVERY_BACK_TURN_SPEED_MPS` | -30.0 m/s | 同上 |
| `RECOVERY_FORWARD_ACCEL_MPSS` | 500.0 m/s² | `USE_BUG_ACC` と同値。override は `np.clip(acc, a_min, a_max)` を通らない |
| `RECOVERY_BACK_ACCEL_MPSS` | 500.0 m/s² | 同上（override 側で `abs()` を取る） |
| `RECOVERY_STEER_LOCK_RAD` | 1.0 rad | 下流のクランプに当てにいく |

いずれも物理的に妥当な値ではなく、**「クリップされない経路に上限を超える値を入れて
車両側の限界に当てる」という意図的な指令**である。決定 5（config 準拠の 3.0 / 3.5）は
復帰が遅すぎたため撤回した。

**舵角の未確認事項**: `aichallenge_awsim_adapter/src/actuation_cmd_converter.cpp` に
`std::clamp(steer_cmd, -0.61, 0.61)` があるが、これは `/control/command/actuation_cmd` を
受ける経路で、本ノードは `AckermannControlCommand` を `/control/command/control_cmd` へ
直接出しているため同じクランプを通るとは限らない。`steering_tire_angle_gain_var = 1.639`
が乗るので publish 値は 1.639 rad になる。実走で頭打ちの有無を確認すること。

**後退距離の上限**: 整列せずタイムアウトまで走ると 30 m/s × 1.5 s = 45 m 後退する計算になる。
実際は整列判定で早く抜けるが、過大なら `BACK_DURATION_TIME_SEC` を縮めること。

### 10. 操舵を「経路からずれた角度分」に切り替え（追記）

ADR-014 / ADR-015 以来、復帰時の操舵は `_collision_side`（衝突時に `path_e_y` の符号で
決めた左右）による **±固定舵**だった。これを **経路との角度差に比例した舵角**に置き換える。

```python
e_psi = wrap(pose_theta - path_psi)          # 左が正
lock  = RECOVERY_STEER_LOCK_RAD

back        : delta = clip(+RECOVERY_STEER_K * e_psi, -lock, lock)
forward_turn: delta = clip(-RECOVERY_STEER_K * e_psi, -lock, lock)
```

**符号が前後で反転する根拠**（自転車モデル `psi_dot = (v/L) * tan(delta)`）:

```text
  e_psi を減らすには psi_dot = -k * e_psi にしたい
    => tan(delta) = -k * e_psi * L / v

  後退 (v < 0): delta は e_psi と同符号  -> +K * e_psi
  前進 (v > 0): delta は e_psi と逆符号  -> -K * e_psi
```

これは `_collision_side` 方式が持っていた「back と forward で符号が反転する」という
パターンと同じで、キーが「衝突時の左右」から「現在の角度差」に変わったことになる。

**副次的な性質**: 後退中は `y_dot ~ v * sin(e_psi)` なので、`e_psi > 0`（経路より左を向いている）
のとき `v < 0` により `y_dot < 0` となり、**角度を消す過程で横偏差も自然に減る**。
角度だけを見ていても横方向が付いてくるため、横偏差項（Stanley 風）は入れていない。

| 定数 | 既定値 | 意味 |
|---|---|---|
| `RECOVERY_STEER_K` | 1.0 | 角度差 → 舵角のゲイン。1.0 = ずれた角度分そのまま |

`_collision_side` は舵角が唯一の用途だったため**削除**した。経路と平行に壁へ刺さった
（`e_psi ≈ 0` かつ横に大きくずれている）場合は舵角 0 の**まっすぐ後退**になるが、
脱出動作としてはこれが妥当なので側方フォールバックは設けていない。

**飽和点**: publish 時に `steering_tire_angle_gain_var = 1.639` が掛かり、AWSIM の
`physics.maxSteerAngle` は 30°(0.524 rad)。よって **`e_psi` が 18.3° を超えると実効フルロック**、
それ以下では比例する。加えて `physics.maxSteerRate = 60 °/s` があるので 0→30° に 0.5 s かかる。

**既知のリスク**: スピンして `e_psi` が ±180° 付近にあると符号がノイズで反転し、
フルロックで左右にばたつきうる。クリップで振幅は抑えられ、後退で対称性は崩れるはずだが、
実走で確認すること。

## 関連 ADR

| ADR | 関係 |
|---|---|
| ADR-014 / ADR-015 | 3 段階復帰と方向指定リバースの出典。左右の操舵符号規則は本 ADR でも維持する。**ただし記載されている時間・速度はいずれも現コードと乖離済み**（back 3.0s / forward 2.0s / ∓4.0 m/s など） |
| ADR-026 | 「衝突検知は最優先・クールダウンはスタック側だけ」という設計の明文化。本 ADR はその前提を保ったまま、ラッチのクリアで再突入を断つ |
| ADR-001 | 復帰直後の即時発進シード（`_last_u[0] = 1.5` / `_last_acc = 1.0`）の出典 |
| ADR-002 | 前車待ち停止中の誤 recovery を `is_waiting_behind_leader` で防ぐ話。本 ADR の対象外 |

## 未解決事項

1. **`/awsim/cmd` の boost に ADR が 1 本も無く、値の意味が未定義。**
   `teleop_manager` は `[1.0] → [0.0]` を続けて送る**モーメンタリ**（押すたびに発火）として使い、
   `OvertakeState` は `[1.5]` を送りっぱなしにして `[0.0]` で戻す**ホールド**として使っている。
   2 つのモデルが食い違っており、1.5 が倍率なのか継続時間なのかを規定した記述が無い。
   本 ADR は既に走行実績のある `OvertakeState` 側（ホールド、1.5）に揃えた。

2. **`OvertakeState` に最大存続時間が無い**（タイムアウト判定がコメントアウトされている）。
   ノードが overtake 中に異常終了すると boost が ON のまま残る。`stop()` も
   `run()` のループ脱出も 0.0 を publish しない。

3. **即時発進シードはほぼデッドコード。**
   `_last_u[0]` / `_last_acc` を読むのは override 経路のローパスと制御無効時のブレーキ初期値だけで、
   MPC 撤去で通常経路のローパスが消えたため、復帰直後の効果はほぼ無い。

4. **衝突検知は `use_sim_time` のときしか subscribe していない。**
   `/aichallenge/pitstop/condition` の購読が `if self.use_sim_time:` ブロック内にあるため、
   実機モードでは `is_colliding` が立たず RecoveryState に入らない。

## 検証

- ユニットテスト `test/test_recovery_state.py`（純 Python、rclpy 不要、13 ケース）
  - 衝突側の記録（`path_e_y` の符号）
  - `back` / `forward_turn` の操舵符号・速度・ギア・加速度
  - 整列済みなら 5 tick 以内に `follow_path` へ戻ること
  - 角度だけ / 偏差だけでは早期離脱しないこと（AND 条件の担保）
  - 一度も整列しない場合のタイムアウト離脱
  - `back` を早く抜けても `forward_turn` が自分の持ち時間を保つこと
  - `on_exit` が `RECOVERY_BOOST_VALUE` で boost を 1 回だけ publish すること
  - 閾値がモジュールグローバルから呼び出し時に読まれること
- 実走: recovery の滞在時間が短くなること、**recovery ↔ follow_path のピンポンが
  起きないこと**（ラッチクリアの検証。最大の回帰リスク）、`/awsim/cmd` に `[1.5]` の後
  必ず `[0.0]` が出ること。
