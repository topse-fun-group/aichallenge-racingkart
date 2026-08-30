# ADR-035: 復帰の目標姿勢を「センターラインをまたぐ向き」にする

## ステータス
一部のみ承認 (Partially accepted)

決定 1・2・5（復帰）のみ適用。**決定 3（速度上限のゲート除去）と
決定 4（コーナーの要求幅マージン）は実走で挙動が怪しく、差し戻した。**
したがって ADR-033 の決定 2（速度上限を回頭角 120° 未満に限定）は**有効なまま**。

差し戻し後の実走で「後退と旋回はよくなったが、その後の前進が短くセンターラインへ
戻りきらない」という報告があり、決定 2a を追記した。

## コンテキスト

ADR-034 適用後の `output/20260830-071818` で、直線の追い越しは 9% → 59% に回復した。
残った 3 症状を同じログで裏取りした。

```text
追い越し 204 件 出口: passed 66 (32%) / hard_narrow 50 / lost 43 / narrow 37 / stuck 8

|kappa| 別        n   passed
  0.00-0.05 直線  22   13 (59%)   <- ADR-034 で回復
  0.05-0.10 緩    15    1 ( 7%)   <- うち narrow 中断が 10
  0.10-0.16 中    95   27 (28%)
  0.16-0.40 急    72   25 (35%)

recovery 66 件: 理由は全て stuck (ego_v = 0 が 100%)、|kappa| 中央値 0.190
  |kappa| 分布: <0.05 が 7 / 0.10-0.16 が 20 / 0.16-0.40 が 39  -> 89% が急コーナー
  元状態: follow_path 48 / follow 10 / overtake 8
recovery 滞在: d1 med 3.53s / d2 med 3.55s / d3 med 1.87s
  BACK 2.0 + FORWARD 1.5 = 3.5s なので、d1/d2 は毎回タイムアウトしている
```

### 問題 1: ヘアピンに速度上限が掛かっていない

ADR-033 の決定 2 は速度上限を `in_tight_corner`（回頭角 120° 未満）に限定した。
その結果、**回頭角 149〜184°、R=2.9〜4.8 m のヘアピン 5 区間には上限が掛からず**、
42 km/h（`v = sqrt(3.0/0.15) = 4.5 m/s = 16 km/h` が上限のところ）で進入していた。

recovery の元状態が `follow_path` 48 件（通常走行、追い越しではない）で、
かつ 89% が |kappa| >= 0.10 であることと整合する。曲がりきれずに壁で止まり、
`stuck` 検知が発動している。この制限は ADR-033 の未解決事項 1 に自分で書いていた。

なお `is_colliding` は 66 件中 1 件も立っていない。`/aichallenge/pitstop/condition` に
publisher が無いという既知の事実（ADR-030）と一致する。

### 問題 2: 復帰の舵角が消え、元の位置に戻ってしまう

ADR-030 の操舵は「経路と平行 (`e_psi = 0`) を目標」とする比例制御だった。

```python
back:    delta = clip(+K * e_psi + 0.3 * e_y, -lock, lock)
forward: delta = clip(-K * e_psi - 0.3 * e_y, -lock, lock)
```

**経路とほぼ平行に刺さる（`e_psi ~ 0` かつ `e_y ~ 0`）と舵角がほぼ 0 になり、
まっすぐ後退してまっすぐ前進し、位置も姿勢も復帰前と変わらない。**
「最後の直線上で停止すると抜け出せない」という報告と一致する。

離脱条件（`|e_psi| < 5°` かつ `|e_y| < 0.5 m`）も、急コーナーでは経路が曲がり続ける
ため成立せず、実測で滞在時間が毎回タイムアウト値 3.5 s に張り付いていた。

### 問題 3: 緩〜中コーナーの突入幅に余裕が無い

`|kappa| 0.05-0.10` は 15 件中 10 件が `narrow` 中断。突入時は
`MIN_OVERTAKE_WIDTH_M = 2.6 m` を満たしていても、幅の縮小速度は実測 0.95 m/s あり、
コーナーを抜ける前に `OVERTAKE_ABORT_WIDTH_M = 2.3 m` を割る。

## 決定事項

### 1. 復帰の目標姿勢を「停止位置から見てセンターラインをまたぐ向き」にする

`states.py` の `RecoveryState`。目標を 0（経路と平行）ではなく
`-sign(path_e_y) * RECOVERY_CROSS_ANGLE_DEG` に置く。突入時に符号を確定させ、
復帰中に `e_y` の符号が変わっても目標は動かさない。

```python
RECOVERY_CROSS_ANGLE_DEG = 30.0  # [deg]

# on_enter
self._cross_sign = -1.0 if ctx.path_e_y >= 0.0 else 1.0

# compute_control_override
err = self._heading_error(ctx) - self._cross_sign * np.deg2rad(RECOVERY_CROSS_ANGLE_DEG)
back:    delta = clip(+K * err + 0.3 * e_y, -lock, lock)
forward: delta = clip(-K * err - 0.3 * e_y, -lock, lock)
```

`e_y > 0`（経路の左）で `e_psi = 0` のまま刺さった場合:

```text
cross_sign = -1  ->  target = -30deg
err = 0 - (-30deg) = +30deg
back:    delta = +K*30deg > 0  左に切って後退  -> 機首は右へ振れる (e_psi が負へ)
forward: delta = -K*30deg < 0  右に切って前進  -> 機首は同じく右へ振れる
```

前進・後退で符号が反転するのは自転車モデル `psi_dot = (v/L)·tan(delta)` によるもので
ADR-015 から変わらない。`0.3 * path_e_y` の横偏差項も同じ向きに働くので温存した。
舵角が最低でも `K * CROSS_ANGLE = 1.8 * 0.524 = 0.94 rad` 残るため、
「動かずに元の位置へ戻る」パターンが構造的に起きない。

離脱条件も `e_psi * cross_sign >= CROSS_ANGLE` に置き換える
（`_is_aligned` と `RECOVERY_ALIGNED_HEADING_DEG` / `RECOVERY_ALIGNED_E_Y_M` は削除）。
**復帰後の姿勢が必ずセンターラインをまたぐ向きになる**ので、
follow_path に戻った直後にコースへ復帰していく向きが担保される。

### 2. 最低フェーズ時間 `RECOVERY_MIN_PHASE_SEC = 0.5 s`

決定 1 の副作用の打ち消し。刺さった時点で既にまたぐ向き（例 `e_y > 0` かつ
`e_psi = -40°`）だと離脱条件が突入直後に成立し、**1 tick も後退せずに復帰を抜けて
再び stuck する**。従来の `_is_aligned` は 40° では成立しなかったので、放置すると退行になる。
各フェーズは最低この時間だけ動く。

### 2a. 前進フェーズの離脱条件を「センターラインへ戻ったか」にする（追記）

決定 1・2 を実走した結果、「後退と旋回はよくなったが、その後の前進が短く
センターラインに近づけないまま復帰が終わる」という報告があった。

**原因は、2 つのフェーズが同じ離脱条件（`_phase_done` = 最低時間 + 交差達成）を
共有していたこと。** 後退フェーズの役割がまさに交差姿勢を作ることなので、
後退が成功した瞬間に前進フェーズの条件は既に成立しており、
**前進は必ず `RECOVERY_MIN_PHASE_SEC = 0.5 s` ちょうどで終わる**。

AWSIM の加速度上限 1.37 m/s² では、0.5 s の走行距離は **0.17 m** しかない。

```text
 0.5s: v= 2.5km/h  走行 0.17m  横成分(30deg) 0.09m   <- 実質動いていない
 1.5s: v= 7.4km/h  走行 1.54m  横成分 0.77m
 2.5s: v=12.3km/h  走行 4.28m  横成分 2.14m
```

フェーズごとに役割に応じた条件を与える。

| フェーズ | 役割 | 離脱条件 |
|---|---|---|
| `back` | またぐ向きを**作る** | `e_psi * cross_sign >= RECOVERY_CROSS_ANGLE_DEG` |
| `forward_turn` | その向きのまま**戻る** | `abs(path_e_y) <= RECOVERY_RETURN_E_Y_M` |

```python
RECOVERY_RETURN_E_Y_M = 0.8       # [m]
FORWARD_DURATION_TIME_SEC: 1.5 -> 2.5  # [s]
```

`FORWARD_DURATION_TIME_SEC` を戻したのは、1.5 s では 1.54 m しか進めず
2〜3 m 横にずれた位置から戻りきれないため。ADR-033 の決定 3 は 3.0 → 1.5 に
下げたが、あれは整列条件（`5° / 0.5 m`）を満たせず**毎回**タイムアウトまで
全開加速していた頃の話で、現在は横偏差で早期離脱できる。
2.5 s まで使い切った場合の到達速度は 12.3 km/h。

### 3. 速度上限のゲートを外し、全コーナーに適用する

> **差し戻し済み**。実走で挙動が怪しく、`if ctx.in_tight_corner:` を戻した。
> 問題 1 自体は未解決のまま残る。加えて「コーナーでも 28 km/h 以上を保ちたい」
> という要求が新たに出ており、単に上限を掛けるだけでは足りない。

`mpc_controller.py` の `_control`。ADR-033 決定 2 の `if ctx.in_tight_corner:` を外す。

```python
v_corner = np.sqrt(self._mpc_cfg.ay_max / (abs(ctx.path_kappa) + 1e-6))
v_target = min(v_target, v_corner)
```

直線では `|path_kappa| ~ 0` で `v_corner` が巨大になり `min()` が効かないため、
ゲート無しでも直線速度には影響しない。ヘアピンにだけ新しく上限が掛かる。

`ctx.in_tight_corner` 自体は残す。将来幅予測のゲート（ADR-034）と決定 4 で使う。

### 4. コーナーでは突入に要求する幅を上乗せする

> **差し戻し済み**。実走で挙動が怪しく、`min_overtake_width` と
> `OVERTAKE_CORNER_WIDTH_MARGIN_M` を削除した。問題 3 は未解決のまま残る。

`states.py` にモジュール関数を 1 つ追加し、`FollowPathState` /
`FollowState` の `check_transition` 冒頭で 1 回だけ束縛して、
**突入条件の比較値だけ**を差し替える（条件式の構造は変えない）。

```python
OVERTAKE_CORNER_WIDTH_MARGIN_M = 0.6  # [m]


def min_overtake_width(ctx: StateContext) -> float:
    if ctx.in_tight_corner:
        return MIN_OVERTAKE_WIDTH_M + OVERTAKE_CORNER_WIDTH_MARGIN_M
    return MIN_OVERTAKE_WIDTH_M
```

中断側（`OVERTAKE_ABORT_WIDTH_M = 2.3` / `OVERTAKE_HARD_ABORT_WIDTH_M = 1.5`）は据え置く。
コーナーでは突入 3.2 m / 中断 2.3 m とヒステリシス帯が広がる方向に働く。

`FollowState.get_adjusted_v_max_mps` の `MIN_OVERTAKE_WIDTH_M`（追従車間 `D0_M` と
`FOLLOW_TARGET_DISTANCE_M` の切り替え、ADR-028）は突入判定ではないので触っていない。
`resolve_overtake_side` の側選択も同様。

`_exit` の遷移ログに `req_w=` を追加した。既存の `min_w=`（`min_forward_overtake_width`）
とは別物で、そのとき実際に要求していた幅を出す。ログ解析でコーナー判定と
マージンの効きを直接確認できる。

### 2b. スタート前のグリッド待機を stuck と誤検知しない（追記）

決定 1・2a を実走したところ、**開始直後に後退する車両**が出た。

`_build_state_context` の stuck 検知は、制御ノードが回り始めた時点から
「速度がゼロの継続時間」を積算する。`_enable_control` の初期値は `True` で
青信号を待つゲートになっていないため、グリッド待機中に `time_stopped` が伸び、
**`STUCK_DURATION = 0.7 s` を青信号の前に超える**。結果、発進すべき瞬間に
`follow_path -> recovery (stuck)` が成立して後退を始める。

これは本 ADR 以前からある誤検知だが、従来はスタート位置がライン上
（`e_psi ~ 0`、`e_y ~ 0`）で `_is_aligned` が即成立し数 tick で抜けていたため
見えていなかった。決定 1 で交差姿勢を目標にしたことで、大きく舵を切って
0.5 s 以上後退するようになり表面化した。

発生源で塞ぐ。一度も走り出していないうちは積算しない。

```python
moving = abs(v) >= STOPPED_THRESHOLD
self._has_ever_moved = self._has_ever_moved or moving

if moving or not self._has_ever_moved:
    self._stopped_since = None
    time_stopped = 0.0
else:
    ...
```

走り出した後の挙動は一切変わらない（`_has_ever_moved` は一方向のラッチ）。
代償として、スポーン位置で完全に嵌まって一度も動けない場合は復帰が発動しない。
グリッド上は前方が開いているので、そこで後退を始めるより待つ方が望ましい。

### 5. ROS パラメータ

`STATE_PARAM_MAP` から `recovery_aligned_heading_deg` / `recovery_aligned_e_y_m` を削除し、
`recovery_cross_angle_deg` / `recovery_min_phase_sec` / `recovery_return_e_y_m`
を追加。`config/` と `launch/` に旧名の上書きが無いことは確認済み。
（差し戻した決定 4 の `overtake_corner_width_margin_m` は含まない。）

## 未解決事項

1. `forward_vehicle_gap` が常に 0.0 のままで `is_ttc_close` が恒真（ADR-027 から継続）。
2. `has_future_width` の左右符号非対称（ADR-032 から継続）。
3. V2X 速度が 2 点差分・無平滑（σ≈2.8 m/s、heading 換算で σ≈22° (訂正: 実測は中央 1.3〜2.0°、ADR-042)）。
   `V2XVehicleTracker` のバッファが `maxlen=2` なので、平滑化するには先にそこを広げる。
4. **実行時 waypoint の平滑化で、最小 R のコーナーの `|kappa|` が CSV の 0.39 に対し
   0.24 と 4 割過小評価される**（上限 19 km/h のところ 23 km/h を許す）。
5. `is_colliding` が一度も立たない（`/aichallenge/pitstop/condition` に publisher が無い）。
   復帰は全て `stuck` 経由で、`STUCK_DURATION` 分だけ発動が遅れる。
6. 差し戻した決定 3・4 が狙っていた問題 1（ヘアピンに速度上限が無い）と
   問題 3（コーナーの突入幅に余裕が無い）は未解決。別 ADR で扱う。
   問題 1 には「コーナーでも 28 km/h 以上を保つ」という追加要求がある。
7. `_has_ever_moved` はノード起動から一度も 0.3 m/s に達しない場合に
   stuck 検知を無効化し続ける。レース中に完全停止から始まる状況は
   グリッド以外に無い想定だが、リスタート系の機能が入ると再考が要る。
8. **後退中に横偏差が `RECOVERY_RETURN_E_Y_M` を超えなかった場合、
   前進フェーズは再び最低時間で終わる。** ただしその場合は実際に
   センターライン近傍にいるので、症状としては現れないはず。実走で確認する。

## 検証

`test/test_recovery_state.py` を交差姿勢とフェーズ分離向けに書き換えた。**106 件パス**。

主なケース:

| ケース | 期待 |
|---|---|
| `path_e_y > 0` で突入 | `cross_sign == -1` |
| `path_e_y` が復帰中に反転 | 目標は突入時のまま |
| **`e_psi = 0`, `e_y = 0` で停止** | **舵角が `±K * CROSS_ANGLE`（0 にならない）** |
| 目標姿勢に到達 | 舵角の角度項が 0 |
| 突入時点で既に交差 | `RECOVERY_MIN_PHASE_SEC` 未満では離脱しない |
| `back` が交差を達成 | `forward_turn` へ進む |
| **交差済みだが横偏差が大きい** | **`forward_turn` は離脱しない**（2a の回帰防止） |
| 交差済みかつ横偏差が小さい | `follow_path` へ戻る |
| どちらも成立しない | `BACK + FORWARD` でタイムアウト離脱 |
| `back` を早く抜けた場合 | `forward_turn` の持ち時間は満額 |

実走（`make dev3`）は `output/20260830-071818` と同じ指標で比較する。

| 指標 | 20260830-071818 | 期待 |
|---|---|---|
| recovery 離脱後の再 recovery 率 | — | 下がる（戻りきってから返すため） |
| recovery 滞在時間 | med 3.53s | 伸びる（前進が実際に走るため。1.0〜4.5s） |
| 目視: 復帰後の位置 | センターラインへ戻りきらない | 戻りきる |
| 目視: 復帰後の姿勢 | 不十分 | センターラインをまたぐ向き |

差し戻した決定 3・4 の指標（追い越し成功率、recovery 件数）は本 ADR の
対象外になったので、比較表から外した。
