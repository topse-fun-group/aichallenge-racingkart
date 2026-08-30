# ADR-041: 追い越しを中断してラインへ戻る間だけ速度を絞る

## ステータス
承認済み (Accepted)

ADR-038（追い越し**中**の接近速度上限、ADR-039 で撤回）と同じ式を使うが、
**適用範囲を「追い越しを抜けた後の復帰中」だけに限定**する点が異なる。

## コンテキスト

「追い越しから途中でキャンセルされて Follow 状態になり、元の軌道に戻るときに
先行車両と衝突する」という報告。`output/20260831-021425`（試行 148 / 成功 60 = 41%）を解析した。

### 機序: 横オフセットが一瞬でゼロに戻る

状態ごとの制御モード:

| 状態 | `control_mode` | 横オフセット |
|---|---|---|
| `OvertakeState` | `WAYPOINT_SHIFT_PURE_PURSUIT` | ±2.5 m 程度 |
| `FollowState` / `FollowPathState` | `PURE_PURSUIT` | **0（レーシングライン）** |

`_control` の寄せ側は
`shift_side = current_state._overtake_side if isinstance(current_state, OvertakeState) else None`
なので、**中断した tick で横目標が ±2.5 m から 0 へ階段状に戻る**。

そのとき先行車はまだ前にいる。

```text
中断時の中心間距離
  narrow       n=45  中央 4.25m  最小 1.65m  3m 未満が 29%
  hard_narrow  n=21  中央 3.38m  最小 1.36m  3m 未満が 38%
  (参考) passed / lost は前方車を検知しておらず gap=None)

中断 56 件はすべて follow へ遷移。
中断から 4s 以内に recovery(stuck) に入ったのは 2 件 (4%) —
つまり多くは「停止」ではなく「接触」で終わっている。
```

**まだ 1.4〜3.4 m 前にいる相手の車線へ切り返す**形になるのが衝突の正体。

### なぜ ADR-038 のやり方では駄目だったか

ADR-038 は同じ式を**追い越し中**に掛けた。追い越し中に先行車が減速すると
上限も連動して崩れ、「抜くべき場面で自車が相手に速度を合わせに行く」正帰還になり、
成功率 46% → 25%、`stuck` 出口 1 → 11 件と退行した（ADR-039 で撤回）。

**適用先が違っただけで、式そのものは正しかった。**
復帰中に限れば、抜くべき局面は既に終わっているのでその正帰還は起きない。

## 決定事項

### 1. 追い越しを抜けた時刻を持つ

`mpc_controller.py`。既存の `_last_recovery_exit_time` と同じ遷移検出パターン。

```python
            if prev_state_name == "overtake" and self._state_manager.current_state_name != "overtake":
                self._overtake_exit_time = (now.nanoseconds / 1e9)
```

### 2. 復帰中だけ制動力から逆算した速度上限を掛ける

```python
OVERTAKE_RETURN_SEC        = 1.0   # [s]
OVERTAKE_RETURN_BRAKE_MPSS = 2.0   # [m/s^2]


def overtake_return_speed_mps(distance_m: float, lead_speed_mps: float) -> float:
    margin = max(0.0, (distance_m - VEHICLE_LENGTH) - FOLLOW_STOP_DISTANCE_M)
    return max(0.0, lead_speed_mps) + np.sqrt(2.0 * OVERTAKE_RETURN_BRAKE_MPSS * margin)
```

`_control` で、`OvertakeState` **以外**かつ抜けてから `OVERTAKE_RETURN_SEC` 以内、
かつ前方車を検知しているときだけ `v_target` に `min()` を掛ける。

先行車 24 km/h のときの上限:

| 中心間 | 上限 | |
|---|---|---|
| 5.00 m | 36.3 km/h | 制限なし |
| 4.25 m | 34.6 km/h | `narrow` の中央値。ほぼ効かない |
| 3.38 m | 32.1 km/h | `hard_narrow` の中央値。わずか |
| 2.50 m | 28.6 km/h | 制限 |
| 1.65 m | 24.0 km/h | 先行車と同速まで落とす |

**中断の中央値ではほとんど効かず、危険な尾（3 m 未満、中断の 29〜38%）でだけ効く。**

### 制約への適合

| 要件 | 適合 |
|---|---|
| 現状の車間距離を短くしない | 目標車間（`D0_M` / `FOLLOW_TARGET_DISTANCE_M`）は不変 |
| 追い越し試行回数を減らさない | 突入条件を一切変更していない |
| 確実に追い越しできる成功率を保つ | `OvertakeState` 中は無制限。成功例（`passed`）は前方車を検知していないので上限自体が発動しない |
| 無能な追い越しを許容しない | 中断の判定条件は不変 |

## 未解決事項

1. **横オフセットの戻しは依然として階段状。** 本 ADR は縦方向（速度）だけで対処した。
   横方向をランプさせるのが本筋だが、`_compute_waypoint_shift_pure_pursuit_control` は
   通常走行の急カーブ追従に直結しており、触ると退行のリスクが高い。
   本 ADR の効果を実走で見てから判断する。
2. **突入の 37〜41% は自車より速い相手**（本番実測、成功率 17〜18%）。
   `safe_width` に相対速度ゲートが無い。試行を減らす方向なので、
   ADR-040 の教訓どおり**成功件数**で検証してから入れる。
3. コーナーの `narrow` 中断は突入時の観測量では弁別できていない。
   `_exit` ログへの将来幅・要求幅の追加が先。
4. `is_colliding` が一度も立たず、**接触回数を直接計測する手段が無い**。
   本 ADR も中断時の車間分布と目視でしか検証できない。
5. ヘアピンに速度上限が掛かっていない（ADR-033 決定 2）。

## 検証

`test/test_overtake_corner.py` に 7 件追加。**132 件パス**。

| ケース | 期待 |
|---|---|
| 中心間 4.25 m（`narrow` の中央値） | 上限が `VEHICLE_V_MAX` の 95% 超（性能を削らない） |
| 中心間 5.0 m 以上 | 上限が `VEHICLE_V_MAX` 超（完全に無制限） |
| 中心間 2.5 m / 1.65 m | 制限され、1.65 m では先行車と同速 |
| 停止距離の内側 | 先行車速。それ以下には絞らない |
| 距離に対し単調 | `ret(5.0) > ret(3.0) > ret(2.0)` |
| 後退する先行車 | 上限が負にならない |
| 式の一致 | `v_lead + sqrt(2 a (gap - d_stop))` |
| 定数の書き換え | 呼び出し時に読む |

窓（`OVERTAKE_RETURN_SEC` の判定）は `_control` 内で rclpy に依存するため
ユニットテストできていない。実走で確認する。

実走（`make dev3`）は `output/20260831-021425` と比較する。**ADR-040 の基準どおり
成功件数と試行回数を先に見る。**

| 指標 | 20260831-021425 | 期待 |
|---|---|---|
| **試行回数** | **148** | **維持（突入条件は不変）** |
| **成功件数** | **60** | **維持〜微増** |
| 中断 (`narrow` + `hard_narrow`) | 66 件 | 維持（中断条件は不変） |
| 中断時の中心間距離 3 m 未満 | 29〜38% | 維持（中断のタイミングは変えていない） |
| 目視: 中断からラインへ戻る際の接触 | 数回 | 減る（本命） |
| recovery 件数 | 4 件 | 増えないこと |
| ラップタイム | — | 悪化しないこと |

**主なリスク**: 復帰中の減速で `follow` に戻った直後の再加速が鈍り、
次の追い越しの助走が遅れること。`OVERTAKE_RETURN_SEC = 1.0` と短くしてあるが、
試行回数が減っていないかを必ず確認する。
