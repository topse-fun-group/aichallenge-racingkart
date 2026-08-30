# ADR-031: コーナーでの追い越しを成立させる（速度・寄せ側・発火条件・コミット）

## ステータス
承認済み (Accepted) — ただし**決定 2（コーナーは内側優先）は ADR-032 で撤回**。
実走ログで内側 18% / 外側 31% (|kappa| 0.08-0.15) と逆効果であることが判明した。
決定 1・4・5 は維持。

## コンテキスト

コーナーでの追い越し試行回数と成功率が低い。「相手がインにいるとき追い越しをかけるが、
速度差がなく外側にハンドルを切って減速して失敗する」という走行観察に対しコードを調べた結果、
**観察どおりの現象を作る構造的な原因が 4 つ**見つかった。

### 原因 1: 追い越しに入った瞬間フルブレーキを指令していた

2 つの Pure Pursuit で目標速度の出し方が違っていた。

| 状態 | 関数 | 目標速度 |
|---|---|---|
| FollowPath | `_compute_pure_pursuit_control` | `wp.v_ref × speed_scale_factor(1.2)` = **11.67 m/s = 42.0 km/h** |
| Overtake | `_compute_waypoint_shift_pure_pursuit_control` | `kmh_to_m_per_sec(35.0)` = **9.72 m/s = 35.0 km/h**（`speed_scale` なし） |

`acc = KP(=100) × (u[0] − v)` を `[a_min, a_max] = [-3.5, +3.0]` でクリップ。
AWSIM の drive-fade 平衡速度は約 9.92 m/s（`vehicle.yaml` の `maxSpeed: 36.0` /
`driveFadeExponent: 40.0` から解析的に 35.7 km/h）。

```text
FollowPath 走行中    : err = 11.67 - 9.92 = +1.75 -> acc = +175 -> +3.0 (フルスロットル)
Overtake に入った瞬間: err =  9.72 - 9.92 = -0.20 -> acc =  -20 -> -3.5 (フルブレーキ)
```

**追い越し中だけ目標速度が下がり、P 制御が最大制動を出していた。**
「ハンドルを切ったから遅くなる」のではなく「追い越し状態に入ったから速度指令が下がり、
たまたま同時にハンドルも切っている」という構図だった。

AWSIM 側に操舵と縦速度を結び付ける機構は無い（`gripSteerFactor` は舵角をスケールするだけ）。

### 原因 2: コーナーでは構造的に「外側（弧長が長い側）」を選んでいた

参照経路は `traj_mincurv.csv`（**最小曲率レーシングライン**であってコース中心線ではない）。
このラインはコーナーでインにつくので、`wp.ub` / `wp.lb` が表す残り空間は**外側に偏る**。

```text
      外側 (空き幅 大 / 弧長 長)
   ┌──────────────────────────
   │        ← ここを選んでいた
   │   ●先行車
   │ ~~~~~~ traj_mincurv (インにつく)
   └──────────
      内側 (空き幅 小 / 弧長 短)
```

351 waypoint の再現計算で **コーナーの約 72% で「広い側 = 外側」**。
`is_left = overtake_width_left > overtake_width_right`（広い方）で寄せ側を決めていたため、
コーナーでは体系的に外側を選び、弧長が長い分だけ追い越しに必要な速度差が増えていた。

### 原因 3: follow → overtake が「幅だけ」になり、下の条件が全部デッドコードだった

`states.py` の
```python
if (ctx.min_forward_overtake_width >= MIN_OVERTAKE_WIDTH_M):
    return "overtake"
```
が無条件 return で、その下の「停止車」「安全な追い越し条件」ブロックが**到達不能**だった。

### 原因 4: 追い越しの半数が最低 dwell の 1.0 秒ちょうどで終わっていた

`output/*/d*/autoware.log` の `[StateManager]` 行 1583 ラン分の実測：

```text
overtake エピソード 9286 件
  出口: follow_path 7531 (81%) / follow 962 / recovery 793
  継続時間: median 1.06 s / mean 1.67 s / p90 2.54 s
  1.05 s 以内で終了: 48.4%
```

継続を保証していたのは `StateManager.MIN_DWELL_TIME = 1.0 s` **だけ**。
35 km/h で 1.0 s は 9.7 m で、R≈5 m のコーナーを抜けるには足りない。
`OvertakeState._enter_time` は設定済みだが未使用（タイムアウト判定がコメントアウト）だった。

## 決定事項

### 1. 追い越し中の目標速度を車両上限より上に置く

```python
OVERTAKE_TARGET_SPEED_KMH = 50.0  # [km/h]
```

`_control` で FollowState の上書きと並べて `v_target` を差し替える。
drive-fade 平衡（35.7 km/h）を超える値なので `acc` は常に `a_max` へ飽和し、
追い越し中はフルスロットルになる。

**加速度のクリップ（`a_max = 3.0`）は変更しない。** AWSIM には
`AIChallenge2026.Penalty.AccelInputAnomalyDetector` / `VehicleAccelInputGuard` が存在し、
過大な加速度指令はペナルティ判定の対象になる（バイナリにクラスの存在を確認済み）。

`lookahead_distance` は `_compute_waypoint_shift_pure_pursuit_control` 内で
35 km/h ベースの固定値のまま（既存の操舵チューニングを崩さないため）。

### 2. コーナーでは内側（弧長が短い側）を優先する

`StateContext.path_kappa` を新設。`_build_state_context` で
`OVERTAKE_CORNER_LOOKAHEAD_M` 先までを見て、**絶対値が最大の符号付き曲率**を取る。
コーナーの強さと向きを 1 値で表せる（正 = 左コーナー = 内側が左）。

```text
kappa = wrap(psi_ahead - psi_behind) / dist_ahead     (core/reference_path.py)
  kappa > 0 -> 左コーナー (内側 = 左)
  kappa < 0 -> 右コーナー (内側 = 右)
```

`_compute_waypoint_shift_pure_pursuit_control` の寄せ側決定を、
`|kappa| > OVERTAKE_CORNER_KAPPA` かつ内側が `MIN_OVERTAKE_WIDTH_M` を満たすときは
**内側を選ぶ**ように変更（それ以外は従来どおり広い側）。

オフセットの絶対値の式は現行チューニング維持のため触らない。

### 3. コーナー専用の積極的な発火条件

無条件 return だったプレースホルダ枠を置き換える。これにより下の 2 ブロックが
**到達可能に戻る**。

```text
is_corner               : |path_kappa| > OVERTAKE_CORNER_KAPPA
inside_width            : kappa の符号で選んだ内側の空き幅
is_near_corner          : forward_vehicle_distance <= OVERTAKE_CORNER_MAX_DIST_M
has_corner_speed_margin : speed_diff >= OVERTAKE_CORNER_SPEED_MARGIN_MPS
                          + 左右に側方車がいないこと
```

| 定数 | 既定値 | 意味 |
|---|---|---|
| `OVERTAKE_CORNER_KAPPA` | 0.05 [1/m] | コーナー判定閾値（R=20 m 相当） |
| `OVERTAKE_CORNER_LOOKAHEAD_M` | 15.0 m | コーナー判定の先読み距離 |
| `OVERTAKE_CORNER_MAX_DIST_M` | 6.0 m | 仕掛ける最大車間（中心間） |
| `OVERTAKE_CORNER_SPEED_MARGIN_MPS` | 0.0 m/s | 必要な相対速度の下限 |

- `is_same_lane` は使わない。自車は Pure Pursuit でインをカットするので `path_e_y` は
  内側符号になる一方、`is_left` は 72% の確率で外側を指す。両者は**構造的に反対符号**で、
  コーナーでは系統的に False になる（`path_e_y == 0.0` でも必ず False）
- `is_slow_leader`（絶対速度 ≤ 25 km/h）ではなく**相対速度**で見る。内側は弧長が短いので、
  速度差が小さくても詰められる
- 近さは `forward_vehicle_distance`（中心間）で見る。`ctx.forward_vehicle_gap` は
  未代入で常に 0.0 なので使えない

**既存の 2 ブロック（停止車・安全な追い越し条件）はそのまま残す。**

### 4. コミット期間

`OVERTAKE_COMMIT_SEC = 1.5 s`。`_enter_time` からの経過でガードする。

| 経路 | 変更前 | 変更後 |
|---|---|---|
| 衝突 → recovery | 即時 | **即時**（変更なし） |
| スタック → recovery | 即時 | **即時**（変更なし） |
| 幅不足 → follow | 即時 | コミット中は維持 |
| 追い越し完了 → follow_path | 即時 | **即時**（変更なし） |
| 前方・側方ロスト → follow_path | 即時 | コミット中は維持 |

**ロスト経路もコミット対象にした理由**: 実測で出口の 81%（7531/9286）がここで、
追い越しで横に 1.5〜2 m シフトすると前方検知（±45°/10 m）と側方検知
（`0.6 <= |y_rel| <= 3.5`）の**両方の窓から一瞬抜け落ちる**。「抜けた」ではなく
「見失った」で戻っている疑いが強く、48.4% が dwell 1.0 s ちょうどで終わる実測とも整合する。

### 5. `has_future_width` の穴を塞ぐ

`if/elif` の入れ子で、**(a) 左右同幅** と **(b) `heading_diff == 0`** のどちらでも
どの枝にも入らず `has_future_width` が `False` 固定になり、追い越しを恒久的に
ブロックしていた。(b) は先行車の heading が取れないとき（停止中・サンプル不足）に常に起きる。

符号の扱いは**既存挙動のまま**にして、抜けていたケースだけを塞ぐ形に整理した。

```python
width_shift = abs((0.5 * 0.4**2 + 0.4 * lead_speed) * np.sin(heading_diff))
if left > right:  has_future_width = (left  - width_shift >= MIN_OVERTAKE_WIDTH_M)
else:             has_future_width = (right + width_shift >= MIN_OVERTAKE_WIDTH_M)
```

3 か所（`FollowPathState` ×2、`FollowState` ×1）に同じ整理を適用。

### 6. 追い越しの離脱理由をログに残す

`StateContext.log_event` コールバックを新設（`publish_boost` と同じパターン）。
`OvertakeState` の各離脱点で `reason=passed|narrow|lost|collision|stuck` と経過時間を出す。

`[StateManager] overtake → follow_path` は「本当に抜けた」と「見失った」が**同じ文字列**で、
成功率が測定できなかった。

## 実走ログによる検証（`output/20260830-005522`、d1/d2/d3 各 49 周）

上記の決定を入れた状態で走らせたログを集計した。

```text
追い越しエピソード 208 件
  passed 99 (48%) / narrow 64 (31%) / lost 36 (17%) / stuck 9 (4%)

突入トリガ別                      n    passed
  corner overtake               118    44%   <- 最多だが最低
  safe overtake                  51    49%
  safe with enough distance      18    61%
  safe between Vf/Ve             10    70%
  stopping with width w/ enough   4     0%   <- 全件 stuck
```

**突入時の変数はどれも成否を予測しなかった。** 幅マージン別の成功率は
`0〜0.2m: 49%` / `0.2〜0.5m: 52%` / `0.5〜1.0m: 40%` / `1.0m〜: 51%` と平坦で、
車間も `passed 3.65m / narrow 3.41m / lost 3.92m` で差が無い。
**突入条件の増減では動かない。問題は突入後にあった。**

### 判明した根因: 幅の量が 4 種類混在していた

| 用途 | 実際に見ていた量 | 対象 |
|---|---|---|
| 突入 `corner overtake` | `inside_width`（kappa 側） | **最近傍 1 台のみ** |
| 突入 `safe` / `stopping` | `min_forward_overtake_width` | 前方**全車**の最小 |
| 中断 `narrow` | `max(overtake_width_left, overtake_width_right)` | **最近傍 1 台のみ** |
| ログ出力 | `min_forward_overtake_width` | 前方全車の最小 |

**突入は「内側」で判断し、中断は「広い側」で判断していた。**
内側が塞がっても外側さえ広ければ `max >= 2.6` で中断が効かず、塞がった側へ突っ込み続ける。
実測で `overtake → X → recovery` 21 件、`overtake → recovery` 直行 9 件、
**全 recovery 59 件のうち 30 件（51%）が追い越しの 2 遷移以内**に発生していた。

### 判明した根因: 閾値にヒステリシスが無かった

`narrow` 中断時の幅は突入 2.79m → 離脱 2.38m と **0.58m しか動いていない**。
突入も中断も同じ 2.6 だったため、V2X ノイズと waypoint index の飛びだけで往復する。
`corner overtake` は突入時マージン中央値 **+0.17m**（51% が +0.2m 未満）と特に際どい。

さらに **`narrow` の 41% がコミット期間（1.5s）明け 0.1 秒以内に発火**していた
（最小 elapsed = 1.50s、p10 = 1.51s）。中断条件はコミット中ずっと真で、明けるのを
待っていただけ。**コミット期間の延長では解決しない。**

### 追加の決定（実走ログを受けて）

1. **寄せ側の決定を `resolve_overtake_side(ctx)` に集約。**
   Pure Pursuit と状態機械が同じ関数を共有し、「内側へ寄せているのに広い側の幅で
   中断判定する」食い違いを構造的に無くす。
2. **中断は突入時にラッチした寄せ側の幅だけを見る**（`OvertakeState._overtake_side`）。
3. **二段閾値**: 突入 `MIN_OVERTAKE_WIDTH_M = 2.6` / 中断 `OVERTAKE_ABORT_WIDTH_M = 2.2`。
4. **`corner overtake` に `min_forward_overtake_width >= 2.6` を追加。**
   実測で 5 件（全て `corner overtake`）が奥の車に塞がれたまま突入していた。
   この枝だけが前方全車を見ていなかった。
5. **`_exit` ログを `DrivingState` の共通メソッドに集約**し、下記のバグを修正した。

### ログのバグ（分析の前提を壊していた）

3 状態にコピペされていた `_exit` に 2 つのバグがあった。

- **`forward_v` は速度ではなく距離だった**。`forward_vehicle_speed` で None 判定しながら
  `forward_vehicle_distance * 3.6` を出力していた（実測で `forward_v == gap × 3.6` を確認）。
  **このログを根拠にした速度側の判断はすべて無効。**
- **`overtake_side` は実際の寄せ側ではなかった**。その場で `left >= right` を再計算した
  「広い側」を出していた。

修正後は `side` / `side_w`（判定に使った幅）/ `min_w`（全車最小）/ `kappa` を出力し、
コーナー別・寄せ側別の成否を後から集計できるようにした。

### 「幅が十分でも追い越さない」について

`MIN_DWELL_TIME` は主因ではなかった。連続 tick をまとめた判断 257 回のうち
**208 回（81%）が実際に遷移し、遅れの中央値は 0.00s**（最大 1.0s は 19 件）。
一方 `follow` に 5 秒以上留まった episode が 40 件・計 328 秒（follow 滞在時間の 45%）ある。
判断が捨てられているのではなく、条件成立を待っている状態だった。

## 未解決事項

1. **`ctx.forward_vehicle_gap` が常に 0.0**（`_build_state_context` で未代入）。
   その結果 `is_closing` は事実上常に False（Δv ≥ 6.5 m/s が必要になる）、
   `is_settled_behind` は常に True で、`is_ttc_close` が意味を失っている。
2. **`has_future_width` の左右符号非対称**。左が広いときは必ず幅が縮む向き、
   右が広いときは必ず広がる向きに評価される。再現計算では右コーナーで pass 率を
   92% → 30% に落としていた。今回は穴を塞ぐだけに留めた。
3. **`heading_diff` がコーナー曲率で汚染されている**。
   「先行車の速度方位 − **自車**最近傍 waypoint の psi」なので、先行車が横に
   切り込んでいなくてもコーナーにいるだけで大きくなる。先行車 waypoint の psi との
   差を引けばコーナー由来の成分を除去できる。
4. **`has_left/right_side_cutin_hazard` は計算済みだが `states.py` で一度も読まれていない**
   （ADR-026 が定めたカットイン予測が死んでいる）。
5. **`ref_vel.yaml` が 1 セクションも効いていない**。`min(km/h の数値, m/s の v_max)` の
   単位不一致で常に 9.72 に飽和し、セクション別のコーナー減速が無効。
6. **boost の予算は 1 レース 2 回**（全 launch スクリプトが `--boosts 2`）。
   `OvertakeState.on_enter` と `RecoveryState.on_exit` の両方が消費するため、
   序盤で使い切る。値 1.5 は倍率でも継続時間でもなく「1.0 以上への立ち上がりエッジ」
   でしかない（1.0 と等価）。
7. **`ctx.forward_vehicle_gap` が常に 0.0 で `is_ttc_close` が全経路で恒真**（最優先の残課題）。
   `_build_state_context` で一度も代入されないため、
   `is_settled_behind = (0.0 <= D0_M + TIME_HEADWAY_SEC*v + ... + 1.0)` が恒真になり、
   `is_ttc_close = is_closing or is_settled_behind` も恒真。
   **TTC 由来の条件は 1 つも効いていない**（`is_closing` も同時に無効化されている）。
8. **`FollowState.control_mode` が `PURE_PURSUIT` に変更されたため、ADR-029 の
   `LateralShiftSideFilter` と `shift_side` 引数がデッドコードになっている**
   （車間 PD 自体は `_control` 側の分岐で生きている）。
9. **`stopping with width with enough distance` は 4 件中 4 件が `stuck`**（成功率 0%）。
10. `OVERTAKE_PASSED_CLEARANCE_TIME_SEC` / `OVERTAKE_TTC_SEC` / `SIDE_VEHICLE_ANGLE_*` /
    `FOLLOW_MIN_SPEED_KMH` / `FOLLOW_LEADER_MOVING_MPS` は未使用のデッド定数。
11. `states.py` の `follow_path → follow`（前方車あり かつ 幅不足）は `_exit` を通らず
    ログが出ないため、この経路だけ後から集計できない。
12. **Hann 窓シフトが操舵指令に反映されていない**。`_compute_waypoint_shift_pure_pursuit_control`
   はループ内で lookahead 点に達したら `break` するため、窓が形作る手前の点は使われない。

## 検証

- ユニットテスト `test/test_overtake_corner.py`（純 Python、rclpy 不要、16 ケース）
  - 左右コーナーで内側が空いていれば発火／外側だけ空いていても発火しない
  - 直線・遠い・相対速度不足・側方車ありで発火しない／衝突が優先される
  - コミット期間中は幅不足・ロストで中断せず、経過後は中断する／衝突は即時
  - `has_future_width` の同幅ケース
  - 各定数がモジュールグローバルから呼び出し時に読まれること
- 実走: `autoware.log` の `[StateManager]` 行と新設の `[Overtake] exit reason=...` 行を
  集計し、試行回数・出口内訳・継続時間分布を変更前後で比較する。
  あわせて `result-summary.json` の `penalty_by_kind`（crash / wall / over）が
  悪化していないことを必ず確認する。
