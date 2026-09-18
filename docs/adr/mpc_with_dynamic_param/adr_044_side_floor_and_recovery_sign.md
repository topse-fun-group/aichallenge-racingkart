# ADR-044: 寄せ側に物理的下限を入れ、復帰の舵角の向きを停止側で固定する

## ステータス
承認済み (Accepted)

ADR-032 の未解決事項「`has_future_width` の左右符号非対称」の最悪ケースを塞ぐ。
ADR-035 の復帰操舵（角度差に比例）の**符号の決め方**を変更する。

## コンテキスト

本番 4 走行（`production-result/analysis_v135_priorities.md`）と動画の観察から、
機序が確定していて変更範囲が小さい 2 件を扱う。

| 観察された挙動 | 分類 | 本 ADR |
|---|---|---|
| Overtake → Follow で前車と衝突 | P2（ADR-041 / ADR-043 で修正済み・本番未投入） | 対象外 |
| 停止車を避けずに突っ込んで停止 | **P0**（横シフトが自車基準・先読み固定） | 対象外（次の最優先） |
| 直線で**狭い方（左が多い）に寄せて壁に衝突** | P1 | **決定 1** |
| 復帰の舵角が契約と逆符号 | 契約違反 | **決定 2** |

### 問題 1: 寄せ側の選択に物理的な下限が無い

本番ログで「寄せ側が広い方でない」ケースが
v1.3.4 で 2/30・7/44、v1.3.5 run1 で 6/42 出ている。

```text
side=left  side_w=0.98  min_w=2.61  kappa=-0.155  -> hard_narrow
side=left  side_w=1.39  min_w=2.99  kappa=-0.105  -> narrow
side=left  side_w=1.83  min_w=2.75  kappa=-0.105  -> hard_narrow
```

**`side_w = 0.98 m` は車幅 1.45 m を下回っており物理的に入れない。**
それでも左を選び、反対側には 2.61 m 空いている。

`resolve_overtake_side` は下限を持たなかった。

```python
left_w  = min(ctx.overtake_width_left,  ctx.overtake_width_left_future)
right_w = min(ctx.overtake_width_right, ctx.overtake_width_right_future)
return "left" if left_w >= right_w else "right"
```

`predict_overtake_widths` は `max(0.0, right + d)` で飽和するので、`d` が大きく負だと
**広い側の将来幅が 0 に潰れる**。すると `right_w = 0` となり、左が 0.98 m でも勝つ。

さらに**タイブレークが `>=` で常に左**。両側の将来幅が 0 に潰れた場合は必ず左が
選ばれる。「左が多い」という観察はこれで説明できる。

### 問題 2: ゲイン項が復帰の舵角の符号を反転させる

契約（ユーザ指定）:

- センターラインの**左**で停止 → 後退は **steer > 0**、その後の前進は **steer < 0**
- センターラインの**右**で停止 → 後退は **steer < 0**、その後の前進は **steer > 0**

ADR-035 の実装は符号もゲイン項に委ねていた。

```python
err = e_psi - cross_sign * CROSS_ANGLE      # e_y >= 0 なら cross_sign = -1
back: steer = clip(+K * err + 0.3 * e_y, -lock, lock)
```

`e_y > 0`（左で停止）のときの実際の値:

```text
   e_psi      err    K*err  +0.3*e_y    steer   判定
      0°      30°     0.94      0.30     1.24   OK
    -20°      10°     0.31      0.30     0.61   OK
    -40°     -10°    -0.31      0.30    -0.01   契約違反
    -60°     -30°    -0.94      0.30    -0.64   契約違反
    -80°     -50°    -1.57      0.30    -1.27   契約違反
```

**`e_psi < -40°` で符号が反転する。** 壁に刺さった直後には普通に起こる角度で、
「ゲインの影響で反転しているのでは」という指摘のとおりだった。

## 決定事項

### 1. 寄せ側の採点の前に、物理的に入れない側を失格にする

```python
    floor = OVERTAKE_HARD_ABORT_WIDTH_M   # 1.5m。ここを割れば入った瞬間に中断する幅
    left_ok = ctx.overtake_width_left >= floor
    right_ok = ctx.overtake_width_right >= floor
    if left_ok != right_ok:
        return "left" if left_ok else "right"

    left_w = min(ctx.overtake_width_left, ctx.overtake_width_left_future)
    right_w = min(ctx.overtake_width_right, ctx.overtake_width_right_future)
    if left_w == right_w:
        return "left" if ctx.overtake_width_left >= ctx.overtake_width_right else "right"
    return "left" if left_w > right_w else "right"
```

`OVERTAKE_HARD_ABORT_WIDTH_M = 1.5` を再利用する（新規定数を増やさない）。
意味づけは「入った瞬間に `hard_narrow` で弾かれる幅の側は、そもそも選ばない」。

**片側だけが下限を満たすときにしか結果が変わらない。** 両側とも満たす／両側とも
満たさない場合は従来と同じ採点なので、試行回数は減らない（ADR-040 の教訓）。

同点時のタイブレークも現在幅基準にして、左バイアスを消した。

### 2. 復帰の舵角は向きを停止側で固定し、大きさだけを角度差に比例させる

```python
    # on_enter
    self._steer_sign = -self._cross_sign     # 左で停止 -> +1 (左に切る)

    # compute_control_override
    err = self._heading_error(ctx) - self._target_heading()
    mag = min(abs(RECOVERY_STEER_K * err) + 0.3 * abs(ctx.path_e_y), lock)
    back:    steer = +self._steer_sign * mag
    forward: steer = -self._steer_sign * mag
```

自転車モデル `psi_dot = (v/L)·tan(delta)` より、同じ向きに機首を回すには後退と前進で
舵角の符号が反転するので、前進側は裏返す。これは ADR-015 から変わらない。

修正後の実測（`RECOVERY_STEER_LOCK_RAD = 1.55`）:

```text
  左で停止 (path_e_y = +1.0)          右で停止 (path_e_y = -1.0)
    e_psi= +80°  後退 +1.55 前進 -1.55    後退 -1.55 前進 +1.55
    e_psi= +40°  後退 +1.55 前進 -1.55    後退 -0.61 前進 +0.61
    e_psi=   0°  後退 +1.24 前進 -1.24    後退 -1.24 前進 +1.24
    e_psi= -40°  後退 +0.61 前進 -0.61    後退 -1.55 前進 +1.55
    e_psi= -80°  後退 +1.55 前進 -1.55    後退 -1.55 前進 +1.55
```

**全域で契約が成立する。**

既に交差姿勢を越えて刺さっている場合は契約優先で回し過ぎる向きになるが、
その状況では `_has_crossed` が既に真で、後退は `RECOVERY_MIN_PHASE_SEC = 0.5 s` で
切り上がるため回転量は限定される。

`_has_crossed` / `_forward_done` / 各フェーズの持ち時間は変更していない。

## 未解決事項

1. **P0（停止車を避けられない）は未着手。次の最優先。**
   横シフトの Hann 窓が自車基準（`closest_idx` 起点）なので、自車が進むと山も逃げ、
   近傍は常に無シフト。追い越し時の先読みも 35 km/h 固定で 6.22 m。
   1.5 m 前の停止車は最大舵角でも避けられない（必要 2.25 m）。
   v1.3.5 run2 ではこれで約 190 s（走行の 45%）を空費した。
2. **`predict_overtake_widths` の左右符号非対称そのものは残る**（ADR-032 から継続）。
   決定 1 は「物理的に入れない側を選ぶ」という最悪ケースを塞いだだけで、
   `left - d` / `right + d` の構造には手を付けていない。
3. ADR-042（V2X 速度平滑化）／ADR-043（横に離れた車を追従対象外）は本番未検証。
4. コーナー |kappa| 0.10-0.16 の幅中断は決定 1 でも残る可能性が高い。
   切り分けには `_exit` ログへの将来幅・要求幅の追加が要る。
5. `is_colliding` が立たず、接触回数を直接計測する手段が無い。

## 検証

`test/test_overtake_corner.py` に 6 件、`test/test_recovery_state.py` に 5 件追加。
**148 件パス**。

| ケース | 期待 |
|---|---|
| **本番の再現**（左 0.98 / 右 2.61、右の将来幅 0） | **`"right"`**（下限で左を失格） |
| 両側とも下限以上 | 従来どおり `min(now, future)` で採点 |
| 両側とも下限未満 | 従来どおり採点 |
| 将来幅が両側 0 で同点 | 現在幅の広い方（左バイアスの解消） |
| 前方車なし | `"none"`（変更なし） |
| `OVERTAKE_HARD_ABORT_WIDTH_M` の書き換え | 呼び出し時に読む |
| **左で停止・`e_psi` を ±80° まで振る** | **後退 > 0 / 前進 < 0 が全域で成立** |
| **右で停止・同上** | **後退 < 0 / 前進 > 0 が全域で成立** |
| 復帰中に `e_y` の符号が反転 | 舵角の向きは突入時のまま |
| 舵角の大きさ | 角度差に対し単調・`RECOVERY_STEER_LOCK_RAD` でクリップ |

実走（`make dev3`）は ADR-040 の基準（成功件数 → 試行回数 → 低速時間 → 成功率）で見る。

| 指標 | v1.3.5 本番 run1 | 期待 |
|---|---|---|
| 試行回数 | 42 | 維持（下限は片側のみ失格のときしか効かない） |
| 成功件数 | 15 | 増える |
| 寄せ側が「広い方でない」件数 | 6 / 42 | ほぼ 0 |
| 目視: 直線で狭い側へ寄せて壁 | 数回 | 0 |
| 目視: 復帰の舵角 | 契約違反あり | 左停止 → 後退左 / 右停止 → 後退右 が常に成立 |
| recovery 件数 | 2 | 増えないこと |

**主なリスク**: 決定 1 で片側だけが下限を満たすとき、従来は将来幅の広い側を
選んでいたのが現在幅基準に変わる。将来幅の予測が正しかったケースでは不利に
なりうるが、`side_w < 1.5 m` は入った瞬間に `hard_narrow` で弾かれる幅なので、
損失より利得が大きいと判断する。
