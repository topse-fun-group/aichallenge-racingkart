# ADR-032: 追い越しを「1 秒後の幅」で判断する（ADR-031 決定 2 の撤回）

## ステータス
**「将来幅で寄せ側を採点する」路線は ADR-048 で終了。** 実測で予測ありの方が 3 倍悪かった。

`predict_overtake_widths` の左右符号非対称は未解決のまま。ただし
**「物理的に入れない側を選んでしまう」最悪ケースは ADR-044 で塞いだ**
(max(0.0, ...) の飽和で広い側の将来幅が 0 に潰れ、0.98m の側を選んでいた)。

**追い越し中の常時フルスロットルには ADR-038 で上限を追加**
（前方車が見えている間だけ、制動力から逆算した速度で頭打ちにする）。
承認済み (Accepted) — **ADR-031 の決定 2（コーナーは内側優先）を supersede する**。
ただし**本 ADR の決定 1（予測位置での関数再呼び出し）は ADR-034 で撤回**。
コーナーで予測点が外側へ 2.2〜5.2m ずれ、「内側が広い」と誤認させていた。決定 2〜5 は維持。

## コンテキスト

`make dev3` の目視で「回頭角 120° 以下のコーナーで、明らかに幅の無いインコースへ入って
壁や先行車に衝突する」現象が、ほぼ全ての該当コーナーで、ほぼ 100% の失敗率で観測された。
`output/20260830-033858`（d1/d2/d3、追い越し 319 件）で裏を取った結果、
**原因は 2 つとも ADR-031 で入れた変更**だった。

```text
追い越し 319 件: passed 112 (35%) / narrow 158 (50%) / lost 41 (13%) / stuck 8 (2%)
```

### 原因 1: 内側優先が逆効果だった

|kappa| 別・寄せ側別の成功率:

| \|kappa\| | 内側 | 外側 |
|---|---|---|
| 0.03–0.08（緩） | n=30 50% | n=2 0% |
| **0.08–0.15（中）** | **n=79 18%** | **n=88 31%** |
| **0.15–0.25（急）** | **n=43 40%** | **n=67 52%** |

参照経路は最小曲率ライン（`traj_mincurv.csv`）でコーナーでは既にインについている。

```text
      外側 (残り空間 大 / 弧長 長)
   ┌──────────────────────────
   │   ●先行車
   │ ~~~~~~ traj_mincurv (インにつく)
   └──────────
      内側 (残り空間 ほぼ無い / 弧長 短)   <- ADR-031 はここを選んでいた
```

**弧長より先に幅が尽きる。** 内側突入時の幅は中央値 2.83 m、外側は 3.90 m と 1 m 以上の差。

### 原因 2: コミット期間が「幅が無いと分かった後」の 0.84 秒を強制していた

```text
narrow 中断の 66% がコミット期間 1.5s 明けの 0.1 秒以内に発火
幅の縮小速度       中央値 0.95 m/s (p90 2.06 m/s)
内側突入時の幅     2.83 m  ->  中断閾値 2.2 m を約 0.66 秒で下回る計算

  t=0.00  突入 (幅 2.83)
  t=0.66  幅 2.2 を割る … が、コミット期間中なので中断できない
  t=1.50  コミット明け。ここで初めて中断
          ← この 0.84 秒 = 約 8 m を「無いと分かっている隙間」へ突っ込み続ける
```

中断時の幅は **17% が 1.0 m 未満**（実質ゼロクリアランス）、47% が 1.5 m 未満。
ADR-031 で入れたコミット期間そのものが、観測された突っ込みの直接原因だった。

### 原因 3: `corner_overtake` が実質「速度差ゼロで可」だった

実速度差を要求する条件だけが成功している（|kappa| ≥ 0.08 の突入のみ集計）:

| 突入トリガ | n | 成功率 |
|---|---|---|
| `safe_width_between_Vf<=25km/h_and_Ve>=29km/h` | 14 | **93%** |
| `forward_vehicle_stop_with_width_between…` | 4 | **100%** |
| `safe_width` | 130 | 35% |
| `safe_width_with_enough_distance` | 42 | 26% |
| **`corner_overtake`**（`speed_diff >= 0.0`） | **83** | **23%** |

### 認識・作動レイテンシの実体

前回 ADR-031 では「突入レイテンシに削り代は無い」と結論したが、
それは**状態機械の判断→遷移の遅れ**（中央値 0.00 s）を測ったもので、
**認識→作動の遅れ**は別物だった。実体は次のとおり。

| 要素 | 平均 | 出典 |
|---|---|---|
| V2X サンプル age（20 Hz） | 25 ms | AWSIM `PublishIntervalSec = 0.05` |
| 2 点差分の速度推定の中心時刻ずれ | +25 ms | `v2x_vehicle_tracker.py` |
| 制御 tick 待ち（40 Hz） | 12.5 ms | `config.yaml control_rate` |
| `steerDelayTime`（むだ時間） | **70 ms** | `vehicle.yaml` |
| `steerTimeConstant`（一次遅れ） | 20 ms | `vehicle.yaml` |
| `maxSteerRate = 60 deg/s` で 10° まで | **167 ms** | `vehicle.yaml` |
| 横変位 0.3 m が積み上がるまで | +390 ms | 推定 |
| **合計: 判断 → 目に見える横移動** | **≈ 0.7 s** | |

現行 `has_future_width` の予測ホライズンは 0.4〜0.8 s で、**この実レイテンシより短い**。
「数秒先を見るべき」というご指摘はレイテンシ実測からも支持される。

### 原因 4: 現行 `has_future_width` が今回の失敗を拾えない

現行は「現在の幅」から `abs((0.5a² + a·v_lead)·sin(heading_diff))` を引く近似で、
**先行車の横移動しか見ていない**。今回の幅縮小の支配要因は
**コーナーで `wp.ub` / `wp.lb`（コース幅）自体が変わること**で、この項では拾えない。

## 決定事項

### 1. 将来幅を「予測位置での関数再呼び出し」で得る

`_compute_v2x_overtake_corridor(pos)` は任意の座標を取り `self` に何も書かない純関数で、
既に `_scan_surrounding_vehicles` が全前方車に対して呼んでいる実績がある。
**同じ関数を先行車の予測位置でもう一度呼ぶ**だけでよい。

```text
【既存】1 回目
  先行車の現在位置 -> _compute_v2x_overtake_corridor() -> overtake_width_left / _right

【追加】2 回目（同じ関数・引数だけ違う）
  予測位置 = v2x_tracker.predict_positions(vid, [T])[0]
           -> _compute_v2x_overtake_corridor() -> overtake_width_left_future / _right_future
```

予測には `V2XVehicleTracker.predict_positions()` を使う。**既に実装済みで未配線**だった
（等速直線モデル）。自前で外挿を書かない。

| 定数 | 値 | 意味 |
|---|---|---|
| `OVERTAKE_PREDICT_HORIZON_SEC` | 1.0 s | 予測ホライズン |

コリドーの基準にした車の id を `_detect_forward_and_side_vehicles` の返り値に追加する
（`target_corridor_pos` と対になる `target_corridor_vid`）。

### 2. 寄せ側は「T 秒後に幅が大きい側」（ADR-031 決定 2 を撤回）

```python
def resolve_overtake_side(ctx):
    if abs(ctx.target_overtake_offset) <= 0.1:
        return "none"
    return ("left" if ctx.overtake_width_left_future >= ctx.overtake_width_right_future
            else "right")
```

レーシングラインがインについている以上、コーナーでは自然に外側が選ばれる。
**回頭角の算出もコーナー区間の切り出しも不要**で、「120° 以下はアウト既定」を包含する。
「先行車がアウトを向いていればインを許可」も、先行車の予測位置がアウトへ動く＝
インの将来幅が広がる、として同じ式で表現される。

### 3. ハード中断（コミット期間を貫通）

```python
if side_w < OVERTAKE_HARD_ABORT_WIDTH_M:
    return "follow"                      # コミット無視・即時
if side_w < OVERTAKE_ABORT_WIDTH_M and not committed:
    return "follow"                      # 従来どおり
```

| 定数 | 値 | 意味 |
|---|---|---|
| `OVERTAKE_HARD_ABORT_WIDTH_M` | 1.5 m | 幅が崩壊。コミットを無視して即中断 |
| `OVERTAKE_ABORT_WIDTH_M` | 2.2 m | ソフト中断（コミット中は維持） |

コミット期間は「シフトしかけの一瞬の幅不足で引き返さない」ためのもので、
「幅がもう無いと分かっている隙間へ突っ込み続ける」ためのものではない。
離脱理由を `hard_narrow` として分け、次回のログで件数を直接数えられるようにした。

### 4. FollowState に将来幅ブロックを追加（既存条件は全て温存）

```text
future_side = 将来幅が大きい側
条件: 将来幅 >= MIN_OVERTAKE_WIDTH_M          (T 秒後も幅が残る)
   かつ 現在幅 >= MIN_OVERTAKE_WIDTH_M         (今も入れる)
   かつ min_forward_overtake_width >= MIN      (他の前方車で塞がっていない)
   かつ speed_diff >= OVERTAKE_CORNER_SPEED_MARGIN_MPS
   かつ 左右に側方車がいない
-> reason="future_width_overtake"
```

既存の `corner_overtake` / `forward_vehicle_stop` / `safe_width` はそのまま残す。

### 5. コーナー追い越しに実速度差を要求

`OVERTAKE_CORNER_SPEED_MARGIN_MPS` を **0.0 → 2.0 m/s**。
0.0 は実質無条件で成功率 23%、実速度差を要求する条件は 93〜100% だった。

## 関連 ADR

| ADR | 関係 |
|---|---|
| **ADR-031** | **決定 2（コーナーは内側優先）を本 ADR が撤回する。** 決定 1（追い越し中の目標速度）、決定 4（コミット期間の枠組み）、決定 5（`has_future_width` の穴埋め）は維持 |
| ADR-029 | 寄せ側ヒステリシス。`FollowState.control_mode` が `PURE_PURSUIT` に変わり `LateralShiftSideFilter` はデッドコード |
| ADR-024 | Waypoint-Shift Pure Pursuit の出典 |

## 未解決事項

1. **`ctx.forward_vehicle_gap` が常に 0.0**（`_build_state_context` で未代入）。
   `is_settled_behind` が恒真になり **`is_ttc_close` が全経路で恒真**。
   TTC 由来の条件は 1 つも効いていない。**最優先の残課題**。
   修正は 1 行だが、入れると追い越し頻度が下がる方向に効くので閾値の再調整とセットで行うこと。
2. **`heading_diff` の基準フレーム不整合**。`heading_diff` は**自車**最近傍 waypoint の psi 基準、
   `e_y_leader` は**先行車**最近傍 waypoint の psi 基準。10 m 離れると中央値曲率 0.083 で
   **約 47° の系統誤差**になる。先行車が完璧にラインに乗っていても `heading_diff` が数十度になり、
   `width_shift` が 2〜3.6 m に達して `MIN_OVERTAKE_WIDTH_M = 2.6` を食い潰す。
   `_compute_v2x_overtake_corridor` が既に持っている先行車 waypoint の psi を返せば直せる。
3. **`has_future_width` の左右符号非対称**。`abs()` で符号を捨てて左に `−`・右に `+` を固定適用
   している。正しくは符号付き `Δe_y` を両方に適用（左は `−Δe_y`、右は `+Δe_y`）。
4. **`OVERTAKE_CORNER_KAPPA = 0.05` が緩すぎる**。実行時 waypoint の **66%** がこれを超えるため、
   `is_corner` は事実上常に True。「コーナー限定」が「常時適用」になっている。
5. `V2XVehicleTracker` に age / タイムアウトが無く、V2X が途絶えても
   `active_vehicle_ids()` が最後の内容のまま固まる。
6. V2X の速度は 2 点差分（dt=0.05 s）で無平滑。位置ノイズ σ=0.1 m なら
   速度ノイズ σ ≈ 2.8 m/s。T=1.0 s の外挿誤差は実走で確認が必要。
7. `V2XVehiclePosition.covariance` が誰にも読まれていない（予測の信頼区間に使える）。

## 検証

- ユニットテスト `test/test_overtake_corner.py`（33 ケース）
  - 将来幅で側が決まること／コーナーでも内側を選ばないこと
  - ハード中断がコミット期間中に発火し、ソフト中断は待つこと／衝突が優先されること
  - 将来幅ブロック: 今は入れるが 1 秒後に閉じるケースで発火しないこと（本命の回帰防止）
  - 各定数がモジュールグローバルから呼び出し時に読まれること
- 実走: `output/20260830-033858` と同じ集計で比較する。
  特に `hard_narrow` の件数（突っ込みを止めた証拠）と、
  |kappa| 0.08–0.15 帯の成功率（今回 25%）、内側を選んだ突入の割合。
