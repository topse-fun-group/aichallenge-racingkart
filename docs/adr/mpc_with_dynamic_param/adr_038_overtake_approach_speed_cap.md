# ADR-038: 追い越し中も前方車が見えている間は追突しない速度で頭打ちにする

## ステータス
**撤回 (Superseded by ADR-039)**。ただし**式そのものは ADR-041 で復活**した
（適用先を「追い越し中」から「中断してラインへ戻る間」に変えれば正しく働く）。

dev 実測で追い越し成功率 46% -> 25%、overtake の `stuck` 出口 1 -> 11 件と明確に退行した。
機序は「追い越し中に先行車が減速 -> 上限が連動して崩壊 -> 自車も減速 -> 並んで停止」
という正帰還。横に出て抜けるべき場面で自車が先行車に速度を合わせに行っていた。
「横に出れば前方コーンから外れて上限は外れる」という本 ADR の想定が誤りだった。

ADR-032 の「追い越し中は常時フルスロットル」に**前方車が見えている間だけの上限**を足す。
`OVERTAKE_TARGET_SPEED_KMH` とその意図（AWSIM の drive-fade 平衡を超える目標を出して
`acc = KP*(u[0]-v)` を飽和させる）は維持する。

## コンテキスト

ADR-037（追従の目標車間を速度依存に）適用後も「Follow 時もしくは Follow への遷移直後に
制御が間に合わず先行車に追突する」という報告が残った。

### 誤った見立てとその棄却

最初は「`FollowPathState` に車間制御が無く、全速で近づいてから遷移が遅れる」と考えた。
これは**誤り**だった。`FollowPathState.check_transition` は
`forward_vehicle_distance <= FORWARD_FOLLOW_DISTANCE_M (5.0)` に入った時点で
必ず `overtake` か `follow` のどちらかを返す（`not_safe_width_with_enough_distance`）。
つまり 5 m 以内で `follow_path` に留まり続けることはない。

なお本件について、`FollowPathState` に車間制御を入れない方針は維持する
（近づいたら Follow に遷移することを条件側で保証する設計）。

### 実際の機序: 追い越しの中断が全開のまま起きる

`follow_path -> follow` の遷移 152 件について、直前の遷移を数えた
（`output/20260831-002141`）。

```text
follow_path -> follow の直前の遷移
  102  overtake -> follow_path
   21  follow_path -> follow
   13  follow -> follow_path
   11  follow_path -> overtake

中心間 4.5m 未満で follow に入ったもの: 16 件 / 152
   10  overtake -> follow_path      <- 危険な遷移の 6 割
    2  follow_path -> overtake
  gap: 1.18, 1.36, 1.45, 1.69, 1.76, 1.82, 1.83, 1.84, 2.03, 2.25, ...
```

`OvertakeState` の目標速度は `OVERTAKE_TARGET_SPEED_KMH = 50 km/h` 固定で、
車間を一切見ない（ADR-032 の意図的な設計）。コリドーが閉じて
`narrow` / `hard_narrow` で中断すると、**全開のまま先行車の 1〜2 m 後ろで
`follow_path` → `follow` へ渡される**。中心間 1.18 m はバンパーが既に重なっている。

`FollowState` の PD はそこから全制動を掛けるが（`KP = 100` で実質バンバン制御、
`a_min = -3.5`、AWSIM 側は -3.0）、残り距離が無い。
これが「遷移直後に制御が間に合わない」の正体である。

## 決定事項

### 1. 制動力から逆算した接近速度上限

`states.py` に定数 1 つと関数 1 つ。

```python
SAFE_APPROACH_BRAKE_MPSS = 2.0  # [m/s^2]


def safe_approach_speed_mps(distance_m: float, lead_speed_mps: float) -> float:
    """先行車に追突せずに済む速度上限 [m/s]。"""
    margin = max(0.0, (distance_m - VEHICLE_LENGTH) - FOLLOW_STOP_DISTANCE_M)
    return max(0.0, lead_speed_mps) + np.sqrt(2.0 * SAFE_APPROACH_BRAKE_MPSS * margin)
```

`v <= v_lead + sqrt(2 a (gap - d_stop))` を満たしていれば、先行車の速度まで
落としきってなお `FOLLOW_STOP_DISTANCE_M` が残る。

`2.0 m/s²` は物理上限ではなく実効値。`config` の `a_min` は -3.5 だが AWSIM 側が
-3.0 に丸め、そこから V2X 20 Hz の遅延と速度推定ノイズ（σ≈2.8 m/s）分を引いてある。

### 2. `OvertakeState` にだけ適用する

`mpc_controller.py` の `_control`、`OvertakeState` の `v_target` を決めた直後。

```python
                if (ctx.forward_vehicle_distance is not None
                        and ctx.forward_vehicle_speed is not None):
                    v_target = min(v_target, states.safe_approach_speed_mps(
                        ctx.forward_vehicle_distance, ctx.forward_vehicle_speed))
```

横に出て前方コーン（`FORWARD_CONE_DEG = 45`）から外れれば
`forward_vehicle_distance` は `None` になり、上限は自動的に外れる。
**追い越しの完遂側には手を入れず、まだ真後ろにいる間だけ絞る。**

先行車 24.7 km/h（実測中央値）のときの上限:

| 中心間 | バンパー間 | 上限 | 効くか |
|---|---|---|---|
| 6.00 m | 4.40 m | 38.4 km/h | なし |
| 5.00 m | 3.40 m | 36.3 km/h | なし |
| 4.24 m | 2.64 m | 34.5 km/h | わずかに制限（突入距離の中央値） |
| 3.00 m | 1.40 m | 30.3 km/h | 制限 |
| 2.50 m | 0.90 m | 27.0 km/h | 制限 |
| 2.00 m | 0.40 m | 24.7 km/h | 先行車と同速 |

突入距離の中央値 4.24 m では `v_max = 35 km/h` に対し 34.5 km/h と、
ほとんど制限しない。危険帯（中断が集中する 1〜2.5 m）でだけ効く。

### 3. `FollowState` / `FollowPathState` には入れない

- `FollowPathState`: 方針として車間制御を持たせない。
- `FollowState`: PD が既にこの上限より厳しい。
  例（車間 2.0 m バンパー、自車 30 km/h、先行車 20 km/h）:
  PD は 2.80 m/s を指令するのに対し、本 ADR の上限は 7.75 m/s。
  上限が binding にならないので足す意味がない。

### 4. ROS パラメータ

`STATE_PARAM_MAP` に `safe_approach_brake_mpss` を追加。

## 未解決事項

1. `is_colliding` が一度も立たない（`/aichallenge/pitstop/condition` に publisher が無い）。
   追突は状態ログに現れず、遷移距離と目視でしか測れない。**追突回数の直接計測手段が無い**
   ことが、この一連の調査で一番効いている制約。
2. コーナー（120° 以下）内側の追い越し幅の件は保留中（ユーザ指示）。
   `narrow` 中断 89 件の原因は現行ログ項目では特定できておらず、
   `_exit` に将来幅と要求幅を足す計測追加が先（ADR-037 未解決事項 1）。
3. 幅が足りているのに追い越さない件（ADR-037 未解決事項 2）。
4. `forward_vehicle_gap` が常に 0.0 で `is_ttc_close` が恒真（ADR-027 から継続）。
5. `corner_overtake` の `inside_width` 判定が有名無実（ADR-036 未解決事項 1）。

## 検証

`test/test_follow_state_gap_pd.py` に 9 件追加。**131 件パス**。

| ケース | 期待 |
|---|---|
| 中心間 6 m | 上限が `VEHICLE_V_MAX` を超える（制限しない） |
| 中心間 2.5 m / 2.0 m | 上限が `VEHICLE_V_MAX` 未満、かつ距離に対し単調 |
| 停止距離ちょうど | 上限 == 先行車速 |
| 停止距離の内側・接触 | 上限 == 先行車速（それ以下には絞らない） |
| 停止した先行車に接触 | 上限 0.0 |
| 後退する先行車 | 上限が負にならない |
| 式の一致 | `v_lead + sqrt(2 a (gap - d_stop))` |
| `SAFE_APPROACH_BRAKE_MPSS` の書き換え | 呼び出し時に読む |

実走（`make dev3`）は `output/20260831-002141` と比較する。

| 指標 | 20260831-002141 | 期待 |
|---|---|---|
| 目視: 追い越し中断後の追突 | 数回 | 減る（本命） |
| 中心間 4.5 m 未満での `-> follow` | 16 件（うち 10 件が overtake 由来） | 大きく減る |
| `follow` 遷移時の中心間 最小値 | 1.18 m | 2.5 m 以上へ |
| 追い越し成功率 | — | 維持（上限は突入距離ではほぼ効かない） |
| 追い越し試行の総数 | — | 変わらないこと |
| ラップタイム | — | 悪化しないこと |

**主なリスク**: 中断間際に減速することで `narrow` からの復帰が遅くなり、
`lost` が増える可能性。上限が効き始めるのは中心間 4.5 m 以下なので影響は限定的と見るが、
`lost` の件数を確認する。
