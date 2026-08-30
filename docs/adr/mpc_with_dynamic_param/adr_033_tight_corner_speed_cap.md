# ADR-033: 急カーブでの速度上限と、寄せ側の「今と将来の狭い方」採点

## ステータス
承認済み (Accepted)

## コンテキスト

ADR-032 適用後も `make dev3` の目視で、回頭角 120° 以下のコーナーで幅の無いラインへ
入って衝突・停止する現象が残り、加えて**低速時と復帰直後に急カーブを曲がりきれない**
という退行が報告された。`output/20260830-054134` のログで裏を取った。

### 問題 1: 寄せ側を「将来幅だけ」で選んでいた（ADR-032 の不整合）

```text
追い越し 34 件: hard_narrow 18 / stuck 6 / narrow 5 / passed 4 / lost 1
hard_narrow のログ行は 380 行（dwell 中の再出力を含む）

hard_narrow の突入時 side_w 中央値 2.53 -> 離脱時 1.21   (min_w は 3.24 -> 3.45 で変わらない)
突入時点で既に side_w < 1.5m だったもの: 34 件中 9 件
```

`resolve_overtake_side` が **T 秒後の幅だけ**で側を選ぶ一方、中断判定は**現在幅**を見る。
そのため「今は塞がっているが 1 秒後に開く側」を掴み、突入直後に `hard_narrow` で弾かれる。
`safe_width` 系の突入条件が見るのは `min_forward_overtake_width`（全車の `max(l,r)` の最小）で、
これは実際に寄せる側の幅を反映しない（実測で 3.24 のまま side_w だけ 1.21 へ落ちる）。

### 問題 2: 曲率と無関係な速度指令

recovery は **|kappa| 中央値 0.13〜0.23（急コーナー）で速度ほぼ 0** のときに発生し、
その 25〜38% が overtake から直接来ていた。原因は 2 つ。

- **追い越し中**: ADR-032 の `OVERTAKE_TARGET_SPEED_KMH = 50.0` は曲率を見ない常時フルスロットル。
- **通常走行**: `ref_vel.yaml` の区間速度（22〜30 km/h）が**単位不整合で 1 つも効いていない**。
  `min(get_ref_vel(), self._mpc_cfg.v_max)` は km/h の数値と m/s の `v_max`(9.72) を比較するため
  常に 9.72 に飽和し、`_compute_pure_pursuit_control` が `× speed_scale_factor(1.2)` して
  **全コーナーで 42 km/h を目標**にしていた。これは本 ADR 以前からの既存バグ。

### 問題 3: 復帰の前進フェーズが 3 秒の全開加速になっていた

離脱条件が `|角度差| < 5°` かつ `|e_y| < 0.5 m` と厳しく、R=4 m 級のコーナーでは
満たせずに毎回タイムアウトする。`FORWARD_DURATION_TIME_SEC = 3.0` を
AWSIM 上限 1.37 m/s² で走り切ると **+15 km/h** ほど乗せた状態でコーナーへ復帰していた。

## 決定事項

### 1. 寄せ側は「今と T 秒後の狭い方」で採点する

```python
left_w  = min(ctx.overtake_width_left,  ctx.overtake_width_left_future)
right_w = min(ctx.overtake_width_right, ctx.overtake_width_right_future)
return "left" if left_w >= right_w else "right"
```

将来幅は「閉じる側を選ばない」拒否権として残しつつ、現在幅も見ることで
中断判定（現在幅）と整合させる。ADR-032 の「側は将来幅、中断は現在幅」という
役割分担は不整合だった。

### 2. 回頭角 120° 未満のコーナーで曲率から速度上限を掛ける

`_update_waypoint_cache` で一度だけコーナー区間を切り出し、
`|∫kappa ds| < TIGHT_CORNER_MAX_TURN_DEG` の区間に印を付ける
（`kappa = wrap(psi_ahead − psi_behind)/ds` なので `Σ kappa·ds` は方位差そのもの）。

走行中は先読み窓に印があれば、速度プロファイルと同じ式で頭打ちにする。

```python
if ctx.in_tight_corner:
    v_corner = sqrt(ay_max / (|path_kappa| + eps))
    v_target = min(v_target, v_corner)
```

**この 1 箇所で全状態（follow_path / follow / overtake）に効く。**
ADR-032 で追い越し専用に入れた cap は、これに統合して削除した。

適用結果（`env/final_ver3`、350 waypoint、110 点 = 31% に印）:

```text
  s=  1.0-  8.0m   50.8deg  R= 7.0m  適用
  s= 11.9- 21.0m   50.8deg  R= 8.9m  適用
  s= 63.9- 86.9m  170.0deg  R= 4.4m  据置(ヘアピン)
  s=108.9-131.8m  184.2deg  R= 4.1m  据置(ヘアピン)
  s=166.8-189.8m  179.4deg  R= 4.8m  据置(ヘアピン)
  s=199.8-209.7m   70.2deg  R= 6.6m  適用
  s=222.7-238.7m  149.3deg  R= 2.9m  据置(ヘアピン)
  s=246.7-256.6m  103.8deg  R= 3.2m  適用
  s=280.6-304.6m  154.5deg  R= 4.6m  据置(ヘアピン)
  ...  計 13 コーナーに適用 / ヘアピン 5 区間は据置
```

速度上限: R=10m → 35 km/h、R=6.7m → 29 km/h、R=5m → 25 km/h、R=4.2m → 23 km/h。
`ref_vel.yaml` が意図していた 22〜30 km/h と一致する。

| 定数 | 値 | 意味 |
|---|---|---|
| `TIGHT_CORNER_MAX_TURN_DEG` | 120.0 deg | これ未満の回頭角のコーナーに速度上限を掛ける |

### 3. 復帰の前進フェーズを 3.0 → 1.5 s

前進フェーズの役割は姿勢を戻すことで、加速ではない。定数のみの変更。

## 未解決事項

1. **ヘアピン 5 区間（回頭角 149〜184°、R=2.9〜4.8 m）には速度上限が掛からない。**
   決定 2 の「120° 未満」という指定に従った結果だが、R=2.9 m は上限 20 km/h 相当で、
   42 km/h では物理的に曲がれない。`TIGHT_CORNER_MAX_TURN_DEG` を上げるか
   ゲートを外せば 1 行で全コーナーに適用できる。
2. **`ref_vel.yaml` の単位不整合そのものは未修正。**
   決定 2 は曲率から直接上限を掛ける迂回策で、`min(get_ref_vel(), v_max)` の
   km/h と m/s の比較は残っている。根治するなら `kmh_to_m_per_sec()` を挟む 1 行。
3. **復帰の離脱条件 `5° / 0.5 m` は急コーナーでは満たせず、常にタイムアウトする。**
   緩めれば早く抜けられるが、整列が甘くなり再スタックが増える可能性がある。今回は据え置いた。
4. ADR-032 からの引き継ぎ: `forward_vehicle_gap` が常に 0.0 で `is_ttc_close` が恒真、
   `heading_diff` の基準フレーム不整合、`OVERTAKE_CORNER_KAPPA = 0.05` を
   waypoint の 66% が超えること。

## 検証

- ユニットテスト 89 件（`test/test_overtake_corner.py` に 5 件追加）
  - 閉じる側を選ばない／今塞がっている側も選ばない／コーナーで内側を優先しない
  - 速度上限の式が速度プロファイルと一致すること
- コーナー切り出しは実データで検証済み（上表）。
- 実走: `hard_narrow` の件数、|kappa| 0.08–0.15 帯の成功率、
  復帰直後の再 recovery 率（前回 d3 で 36%）を比較する。
