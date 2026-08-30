# ADR-036: コーナーの内側へは追い越しを仕掛けない

## ステータス
承認済み (Accepted)

ADR-031 の決定 2（コーナーは内側優先）は ADR-032 で既に撤回されていたが、
**「内側を避ける」という積極的な条件は入っていなかった**。本 ADR でそれを入れる。

## コンテキスト

「120° 以下のコーナーで、十分な幅がない、もしくは幅がなくなるのに追い越しをして
衝突する」という報告が継続していた。`output/20260830-221428`（d1/d2/d3）を解析した。

追い越しの突入 158 件（dwell 中の再出力を潰し、直後の離脱と対応付けた 125 件を集計）。

### まず否定された仮説

**(a) 突入時の幅が足りていない** — 違った。出口別の突入時 `side_w` 中央値は
`hard_narrow` 2.96 / `narrow` 2.92 / `passed` 3.10 / `lost` 3.15 と、
**成功例と失敗例で区別がつかない**。ADR-035 の決定 4（コーナーで要求幅に
一律 0.6 m のマージンを上乗せ）が効かず差し戻されたのは、これが理由である。

幅は突入後に崩れている。

```text
出口            n |  突入sw  離脱sw  崩壊速度[m/s] | 突入gap  速度差  |k|
hard_narrow    35 |   2.96    1.27      0.96      |   5.40   3.56  0.140
narrow         21 |   2.92    2.20      0.23      |   4.00   3.48  0.132
passed         26 |   3.10    0.00      2.07      |   3.90   1.87  0.143
```

**(b) `forward_vehicle_gap` が常に 0.0 で距離ゲートが効いていない** — 事実では
あるが、これだけでは説明にならない。`is_settled_behind` の閾値を正しく計算しても
ego 25 km/h で中心間 10.1 m まで許容され、観測された突入（gap 中央値 5.1 m）は
すべて通過する。修正しても弁別しない。

### 実際の弁別要因: 寄せ側がコーナーの内側か外側か

`FollowState._log_side` は `resolve_overtake_side(ctx)`、つまり
`OvertakeState.on_enter` がラッチする**実際に寄せる側**を出している。
これを `path_kappa` の符号（`kappa > 0` = 左コーナー）と突き合わせた。

```text
|kappa| 帯       内側                                外側
0.00-0.05  n= 10 成功  9 (90%) 幅中断  1 (10%) | n=  2 成功  0 ( 0%) 幅中断  2 (100%)
0.05-0.10  n= 10 成功  1 (10%) 幅中断  9 (90%) | n=  3 成功  0 ( 0%) 幅中断  3 (100%)
0.10-0.16  n= 32 成功  4 (12%) 幅中断 22 (69%) | n= 29 成功 12 (41%) 幅中断  6 ( 21%)
0.16-0.25  n= 14 成功  0 ( 0%) 幅中断 11 (79%) | n= 55 成功 15 (27%) 幅中断 15 ( 27%)

|kappa| >= 0.05 合計   内側 46 件 成功  3 (7%)  幅中断 35 (76%)
                       外側 69 件 成功 16 (23%) 幅中断 18 (26%)
```

**コーナー内側への突入は 76% が幅で中断する。** 参照経路が `traj_mincurv`
（最小曲率ライン）で既に内側を舐めているため、内側には構造的に幅が無い。
突入時点では空いて見えても、コーナーが進むにつれて閉じる。
これが「幅がなくなるのに追い越しをする」の正体である。

**直線（|kappa| < 0.05）では逆に内側が 10 件中 9 件成功している。**
直線では `kappa` の符号はノイズで「内外」に意味が無い。
既存の `OVERTAKE_CORNER_KAPPA = 0.05` がちょうど境界になっている。

### どの突入条件が通していたか

```text
突入条件                 n  passed  hard_narrow  narrow  lost  stuck  成功率
safe_width             109      23           30      19    32      5    21%
future_width_overtake   16       3            5       2     2      4    19%
corner_overtake          0  (一度も発火せず)
forward_vehicle_stop     0  (一度も発火せず)
```

`safe_width` が突入の 87% を占める。この条件が見ているのは
`min_forward_overtake_width`（全車の `max(l,r)` の最小）だけで、
**実際に寄せる側の幅も、コーナーの内外も一切見ていない**。
`is_ttc_close` は `forward_vehicle_gap == 0.0` により恒真なので距離ゲートも無い。

## 決定事項

### コーナーでは、寄せ側が内側になる追い越しを見送る

`states.py` にモジュール関数を 1 つ追加する。

```python
def is_inside_corner_overtake(ctx: StateContext) -> bool:
    if abs(ctx.path_kappa) <= OVERTAKE_CORNER_KAPPA:
        return False                      # 直線では内外に意味が無い
    side = resolve_overtake_side(ctx)
    if side == "none":
        return False                      # 前方車を検知していない
    inside_is_left = ctx.path_kappa > 0.0  # kappa > 0 = 左コーナー
    return (side == "left") == inside_is_left
```

`side == "none"` のガードは必須。これが無いと `"none" != "left"` が「右寄せ」と
読まれ、右コーナーを常に見送ってしまう。

`FollowPathState` と `FollowState` の `check_transition` 冒頭で 1 回だけ束縛し、
**追い越し突入ブロック 8 か所の条件末尾に `and not is_inside_corner` を足す**。

```python
        is_inside_corner = is_inside_corner_overtake(ctx)
```

| 状態 | 突入ブロック |
|---|---|
| `FollowPathState` | `forward_vehicle_stop_with_width_...` / `safe_width_between_...` / `stop_with_width_in_enough_distance` / `safe_width_with_enough_distance` |
| `FollowState` | `future_width_overtake` / `corner_overtake` / `forward_vehicle_stop` / `safe_width` |

**既存の条件式は 1 つも変更・削除していない。** 判定を 1 つ足しただけ。
突入元は実測で follow 127 件 / follow_path 31 件だったので両方に入れる。

中断側（`OVERTAKE_ABORT_WIDTH_M` / `OVERTAKE_HARD_ABORT_WIDTH_M`）は据え置き。

### 期待される効果（実測ログへの当てはめ）

```text
|kappa| >= 0.05 の突入について
  失う成功    :  5 件
  防ぐ幅中断  : 42 件
  コーナー成功率: 22% -> 31%
```

## 未解決事項

1. **`corner_overtake` ブロックの前提が実測と矛盾している。**
   ADR-031 決定 2 で「内側が空いているコーナーに限って積極的に仕掛ける」として
   `inside_width >= MIN_OVERTAKE_WIDTH_M` を要求しているが、本 ADR の veto と
   組み合わさると「内側も広いが外側の方がもっと広い」ときだけ発火する条件になる。
   実測では 0 回発火しており、`inside_width` の判定は既に有名無実。
   整理は別の変更で行う（1 度に 1 つずつ、という進め方のため今回は触らない）。
2. **`forward_vehicle_gap` が常に 0.0 で `is_ttc_close` が恒真**（ADR-027 から継続）。
   本 ADR では弁別しないと確認したので手を付けていないが、
   `safe_width` に距離ゲートが無い状態は残る。gap 別の成功率は
   3-4 m で 46%、4 m 以上で 8% と差が大きく、次の候補になる。
3. `has_future_width` の左右符号非対称（ADR-032 から継続）。
4. V2X 速度が 2 点差分・無平滑（σ≈2.8 m/s）。`predict_overtake_widths` が使う
   `heading_diff` の σ は約 22° あり、将来幅の予測方向はかなりの割合で反転しうる。
   `V2XVehicleTracker` のバッファが `maxlen=2` なので、平滑化には先にそこを広げる。
5. 実行時 waypoint の平滑化で最小 R のコーナーの `|kappa|` が
   CSV の 0.39 に対し 0.24 と 4 割過小評価される。本 ADR は `OVERTAKE_CORNER_KAPPA`
   （0.05）との比較にしか使っておらず、その桁では影響しない。

## 検証

`test/test_overtake_corner.py` に 7 件追加、既存 3 件を新しい契約に反転。**115 件パス**。

| ケース | 期待 |
|---|---|
| 左コーナー・左寄せ / 右コーナー・右寄せ | 突入しない |
| 左コーナー・右寄せ / 右コーナー・左寄せ | 突入する |
| 直線（両符号・両側） | 突入する（veto しない） |
| 前方車なし（`offset ~ 0`） | veto しない（`"none"` ガード） |
| 今は左が広いが閉じる → 実際は右寄せ | 右コーナーなら veto（実際に寄せる側で判定） |
| `OVERTAKE_CORNER_KAPPA` を書き換え | 呼び出し時に読む |
| `FollowPathState` | 内側は見送り、外側は突入 |
| 外側でも速度差が無ければ | 突入しない（既存条件は生きている） |

実走（`make dev3`）は `output/20260830-221428` と同じ指標で比較する。

| 指標 | 20260830-221428 | 期待 |
|---|---|---|
| コーナー突入のうち内側 | 46 件 (40%) | ほぼ 0 |
| `hard_narrow` + `narrow` | 56 件 | 大きく減る |
| コーナー成功率 | 22% | 上がる（当てはめでは 31%） |
| 直線の成功率 | 90% | 維持（veto は掛からない） |
| 追い越し試行の総数 | 125 | 減る（内側 46 件分） |
