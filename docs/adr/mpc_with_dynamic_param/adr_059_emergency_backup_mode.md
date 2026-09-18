# ADR-059: 緊急回避モード（手動トピックで 3m 後退して復帰する独立した脱出経路）

## ステータス
承認済み (Accepted)

`EmergencyState` を新設し、**`/control/final/emergency_request` の手動発行でのみ**
どの状態からでも最優先で遷移して、**まっすぐ 3m 後退してから `follow_path` へ返す**。
**既存 4 状態の遷移条件・制御式は 1 行も変更しない。**

## コンテキスト

決勝大会の現場での事前走行で、**前方車両が近距離で完全停止したとき、
`RecoveryState` が無限ループに入り前方車両への衝突を繰り返す**事象が見つかった。

`naito_v1.3.6b` が最終バージョンであり、**この時点で通常制御を変更すると
動作が不安定になるリスクがある**。そのため、自動判定には一切手を入れず、
**人の判断で発行するトピックだけをトリガーにした脱出経路**を追加する。

なお同種の事象は本番ログでも観測済みで、ADR-058 でクールダウンを 2.0 → 4.0 秒に
戻して緩和したが、**完全停止した前方車両が至近距離にある場合の脱出は保証できない**。
本 ADR はその最後の逃げ道を人の手に渡すもの。

### 要件

1. トリガーは **ROS トピックの手動発行のみ**（自動で立つ経路を作らない）
2. **どの状態からでも最優先**で遷移する
3. **3m 後退 → 通常走行（`follow_path`）へ遷移**
4. **`RecoveryState` とは完全に独立**（状態・動作・定数を共有しない）

## 決定事項

### 1. `EmergencyState`（`states.py`）

`RecoveryState` を継承も流用もしない独立クラス。

| 項目 | 値 |
|---|---|
| `name` | `"emergency"` |
| `control_mode` | `ControlMode.OVERRIDE` |
| `gear` | `GEAR_REVERSE` |
| `compute_control_override` | `(EMERGENCY_BACK_SPEED_MPS, EMERGENCY_STEER_RAD, EMERGENCY_BACK_ACCEL_MPSS)` |
| 離脱 | **実際に後退した積算距離** ≥ `EMERGENCY_BACK_DISTANCE_M`、または `EMERGENCY_TIMEOUT_SEC` |

```python
EMERGENCY_BACK_DISTANCE_M = 3.0    # [m]
EMERGENCY_BACK_SPEED_MPS  = -32.0  # [m/s]   override 経路は np.clip を通らない
EMERGENCY_BACK_ACCEL_MPSS = 3.0    # [m/s^2]
EMERGENCY_STEER_RAD       = 0.0    # [rad]   直進バック
EMERGENCY_TIMEOUT_SEC     = 8.0    # [s]     噛んで動けないときの保険 (減速時間を見て 6.0 -> 8.0)
```

**距離は「実際に後退した分」だけを積む**（`ctx.velocity < 0` の間だけ `|v| * dt`）。
姿勢にも経路にも依存しない。

> **初版は突入位置からの直線距離で測っており、これが誤りだった。**
> 走行中に発行されると減速までに惰性で前へ進み、**後退を始める前に
> 「3m 動いた」が成立して抜けてしまう。** 実測 (`output/20260919-030622`):
>
> ```text
> [StateManager] follow_path → emergency (manual request)
> [emergency] exit reason=backed to=follow_path elapsed=0.48s | ego_v=22.03
> ```
>
> 22 km/h (6.1 m/s) で発行し、**0.48 秒・前進したまま**離脱していた。
> 遷移自体は起きていたが後退が始まる前に終わるため、外からは
> 「発火しない」ようにしか見えなかった。
> 「どの向きへ 3m 動いても離脱する」という単体テストを書いていたことが、
> この誤りをそのまま固定してしまっていた。

### 2. 後退しながら経路の中心・姿勢へ戻す

要件どおり、3m の後退中に**横偏差と姿勢の両方**を経路に合わせる。
横偏差から目標姿勢を作り、その姿勢誤差から舵角を出す二段の比例制御。

```python
EMERGENCY_HEADING_FROM_EY = 1.2    # [rad/m] 横偏差 -> 目標姿勢
EMERGENCY_HEADING_MAX_RAD = 0.9    # [rad]   目標姿勢の上限 (52 度)
EMERGENCY_STEER_K         = 3.0    # [-]     姿勢誤差 -> 舵角
EMERGENCY_STEER_MAX_RAD   = 0.9    # [rad]   舵角の上限 (52 度、出版時 85 度)
EMERGENCY_ALIGN_START     = 0.5    # [-]     進捗がこれを超えたら姿勢合わせへ

    target_psi = clip(K_ey * e_y, ±HEADING_MAX) * (1 - align)
    delta      = clip(K_psi * (e_psi - target_psi), ±STEER_MAX)
```

#### 後半は「経路と平行」を優先する（初版からの修正）

初版は目標姿勢を横偏差に比例させるだけだったため、**横偏差が残る限り
車体が斜めのまま終わり、経路と平行にならなかった**（実走で報告された）。

**単にゲインや舵角上限を上げても解決しない。** シミュレーションでは
横偏差ゲインを上げるほど終端の姿勢誤差が**悪化**した。

| `K_ey` | 3m 後の最大 \|e_psi\| |
|---|---|
| 0.5 | 24 deg |
| 0.8 | 27 deg |
| 1.2 | **33 deg** |

舵角上限を 29 → 52 度に上げても 24 → 24 度でほぼ変化なし。
構造的な原因なので、**進捗が `EMERGENCY_ALIGN_START` を超えたら目標姿勢を
線形に 0 へ落とす**ようにした。これで終端の姿勢誤差が **24 度 → 4〜6 度**になる。

#### 符号の導出（破ってはいけない契約）

**後退では舵角の効き方が前進と逆になる。** 自転車モデル (後軸基準) で

```text
e_y_dot = v * sin(e_psi)
psi_dot = (v / L) * tan(delta)
```

後退は `v < 0` なので、

- `e_y > 0` (経路の左にいる) を減らすには `e_y_dot < 0`、すなわち
  `sin(e_psi) > 0` → **`e_psi > 0` (機首を左) を作る**
- `e_psi` を増やすには `psi_dot > 0`、すなわち `tan(delta) < 0`
  → **`delta < 0` (右に切る)**

まとめると **「経路の左にいるほど右へ切る」** という、前進とは逆の関係になる。

実装の出力（`v = -2.0 m/s`）:

| `e_y` | `e_psi` | 舵角 | 向き |
|---|---|---|---|
| **+1.0 m** | 0 deg | **−28.6 deg** | **右** |
| −1.0 m | 0 deg | +28.6 deg | 左 |
| 0 m | +17 deg | +25.8 deg | 左 |
| 0 m | −17 deg | −25.8 deg | 右 |
| 0 m | 0 deg | 0 deg | 直進 |

この符号は `RecoveryState` が ADR-044 で踏んだ「後退の舵角が反転する」罠と
同じ性質のものなので、**単体テストで各象限を固定した**（12 件）。
なお `RecoveryState` は目標姿勢が違う（センターラインを**またぐ**向き）ため
舵角の向きも逆になるが、どちらも同じ運動方程式から導いた結果で矛盾しない。

#### 前進の惰性が残っている間は切らない

`ctx.velocity >= 0` の間は舵角 0。進行方向が逆だと同じ舵角が逆に効くため、
減速しきるまで待つ。

#### 収束（40Hz の離散制御、L=1.087、v=−3 m/s、3m 後退）

距離基準の制御なので**後退速度にほぼ依存しない**（v=−1 / −3 / −6 m/s で同等、
舵角の符号反転も 2 回以内で発振なし）。

| 初期 `e_y` | 初期 `e_psi` | 3m 後 `e_y` | 3m 後 `e_psi` |
|---|---|---|---|
| **+1.0 m** | 0 deg | **−0.01 m** | **+1 deg** |
| −1.0 m | 0 deg | +0.01 m | −1 deg |
| 0 m | +17 deg | +0.00 m | 0 deg |
| +1.5 m | −11 deg | +0.15 m | +5 deg |
| +2.0 m | +17 deg | +0.24 m | +6 deg |

`e_y = +1.0 m` の軌跡（前半で横偏差を詰め、後半で姿勢を戻す）:

```text
後退[m]  e_y[m]  姿勢[deg]  目標姿勢[deg]  舵角[deg]
   0.0    1.00       0.0         51.6      -51.6
   1.0    0.51      43.8         35.2      +25.6
   2.0    0.06      10.4          2.6      +23.1
   3.0   -0.01       0.6            -          -
```

横偏差・姿勢ともほぼ経路に一致した状態で `follow_path` へ返せる。

### 3. 最優先分岐は `StateManager.update` の入口に置く

```python
        if ctx.emergency_request and self._current.name != "emergency":
            ...
            return self._current.get_params()

        next_name = self._current.check_transition(ctx)
```

**既存の `check_transition` をそもそも呼ばない**ので、通常の遷移条件には一切影響しない。
各状態に条件を足して回る実装（4 か所の変更）を避けられる。

`MIN_DWELL_TIME` の免除に `emergency` を加えた。免除しないと 3m 到達後も
最大 1 秒は後退指令が出続けて下がりすぎる。
**これが既存コードへの唯一の変更（1 行）。**

### 4. ラッチの消費と退出時の後始末（`mpc_controller.py`）

```python
        # 緊急モードに入ったらラッチを消費する（残すと抜けた瞬間に再突入して永久に後退）
        if self._state_manager.current_state_name == "emergency":
            self._emergency_requested = False

        if prev_state_name == "emergency" and ... != "emergency":
            self._last_u[0] = 1.5                     # override の負値を前進側へ戻す
            self._last_acc = 1.0
            self._last_colliding_time = None          # 衝突ラッチを消す
            self._last_recovery_exit_time = now_sec   # recovery 再突入を抑止
```

**衝突ラッチの消去は本機能の目的そのもの。** これが無いと、3m 下がって戻った直後に
ラッチが残っていて即座に `recovery` へ落ち、報告された無限ループがそのまま再開する。
`RecoveryState` の退出処理とは別ブロックにしてある（完全に独立させる要件のため）。

## 既知の性質

**override 経路は `if not self._enable_control:` の判定（`mpc_controller.py`）より
手前で早期 return する。** したがって緊急回避は制御無効中でも車体を動かす。
これは `RecoveryState` も同じ既存の挙動であり、手動トリガーである本機能では
むしろ望ましい（スタート前でも係員が下げられる）。**仕様として明記しておく。**

## 未解決事項

1. **無限ループの根本原因は未解明のまま。** 本 ADR は人の手による脱出経路であって
   原因の修正ではない。ADR-058 の未解決事項 2（停止車の脇を 0.387 m/s でしか
   抜けられない）に帰着する可能性が高い。
2. `StateManager` の最優先分岐は `Node` を要するため単体テストできない。**実機で確認する。**
3. 後退中に後方の障害物は一切見ていない。3m 後方の安全確認は発行する人の責任。

## 検証

**198 件パス**（`test/test_emergency_state.py` を新設し 29 件追加。既存 169 件は不変）。

| ケース | 期待 |
|---|---|
| `control_mode` / `gear` / `name` | `OVERRIDE` / `GEAR_REVERSE` / `"emergency"` |
| `compute_control_override` | 速度が負・加速度が正 |
| **経路の左にいる (`e_y>0`)** | **右へ切る**（符号契約・最重要） |
| 経路の右にいる | 左へ切る |
| 機首が左を向いている | 左へ切って直す |
| 中心かつ平行 | 舵角 0 |
| 左右対称性 | 符号を反転して一致 |
| 舵角・目標姿勢の上限 | それぞれ頭打ちになる |
| **前進の惰性が残っている間** | **舵角 0（切らない）** |
| **後退が進むほど目標姿勢が 0 へ寄る** | **進捗 100% では姿勢合わせのみ** |
| 姿勢合わせ区間でも姿勢は直す | 傾いていれば戻す |
| **3m 後退後（5 通りの初期条件）** | **`e_y` < 0.3m かつ姿勢 < 8 度** |
| 2m だけ後退 | `None`（後退を続ける） |
| **3m 後退** | **`"follow_path"`** |
| **18m 前進しても** | **`None`**（前進の惰性は数えない・回帰防止） |
| 前進 18m のあと 3m 後退 | `"follow_path"` |
| 停止したまま | `None` |
| 複数 tick にまたがる積算 | 合計で判定する |
| 2 回目の発行 | 前回の後退分が残っていない |
| 噛んで動けないまま 8s | `"follow_path"`（保険） |
| `on_enter` 前 / `on_exit` 後 | `None`（不活性） |
| 定数の書き換え | 呼び出し時に読む（ROS パラメータが効く） |
| `StateContext.emergency_request` の既定 | `False` |

`naito_v1.3.6b` からの差分は **3 ファイル / +175 行・−1 行**。
**削除行は `state_manager.py` の `MIN_DWELL_TIME` 免除条件 1 行のみ**で、
他はすべて追加。既存 4 状態のロジックには一切触れていない。

### 実機での確認手順

```bash
# 走行中に、トピックを 1 回発行するだけ（他の操作は不要）
ros2 topic pub -1 /control/final/emergency_request std_msgs/msg/Empty '{}'
```

**発行は必ず「車両の Autoware と同じ ROS_DOMAIN_ID」から行う。**
dev 環境では `.env` の `ROS_DOMAIN_ID=1` で Autoware が動くのに対し、
ホストのシェルは既定の domain 0 (AWSIM 側) になるため**何も起きない**
（現場で「発行しても遷移しない」事象として実際に出た。コード側の不具合ではない）。

```bash
# dev のホストから叩く場合は domain を明示する
ROS_DOMAIN_ID=1 ros2 topic pub -1 /control/final/emergency_request std_msgs/msg/Empty '{}'
```

**発行前の疎通確認**: 同じ端末で `ros2 topic echo /mpc/driving_state` が
状態名を出せば、domain は合っている。出なければ発行しても届かない。

発行側の QoS は問わない。RELIABLE / BEST_EFFORT の両方で購読しているため、
どちらの publisher でも必ず繋がる（片方だけだと QoS 非互換で**無言のまま
一度も届かない**。緊急用の経路でこの失敗は許容できない）。

| # | 確認内容 |
|---|---|
| 1 | 発行した瞬間に `/mpc/driving_state` が `emergency` になる（どの状態からでも） |
| 2 | ギアが REVERSE になり、まっすぐ後退する |
| 3 | **約 3m 下がって停止し、`follow_path` に戻って通常走行を再開する** |
| 4 | ログに `[StateManager] <前の状態> → emergency (manual request)` と `Exited EmergencyState: latch cleared` が出る |
| 5 | 復帰後に `recovery` へ即落ちしない |
| 6 | **トピックを発行しない限り一度も `emergency` にならない**（1 周して確認） |
| 7 | 報告された事象の再現（前方に停止車）で、手動発行により脱出できる |

**切り戻し**: `git checkout naito_v1.3.6b -- <3 ファイル>` で完全に戻る。
