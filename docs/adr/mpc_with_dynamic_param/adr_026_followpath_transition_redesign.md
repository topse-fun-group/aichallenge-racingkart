# ADR-026: FollowPath 状態遷移条件の再設計（幾何・速度・予測統合判定）

## ステータス
承認済み (Accepted)

## コンテキスト
レースシミュレーションにおいて、`FollowPathState`（通常レーシング走行状態）からの遷移を正確に行い、安全な追従（`Follow`）と積極的かつ安全な追い越し（`Overtake`）を両立するため、幾何学的な角度・距離範囲、3秒後の相対速度予測位置、およびコース道幅クリアランスに基づく包括的な状態遷移条件を再設計した。

## 決定事項

### 1. 遷移条件の定義（遷移元: FollowPath）

#### ① Recovery 状態への遷移（最優先）
- 衝突検知（`ctx.is_colliding`）
- または スタック検知（`time_stopped_sec >= STUCK_DURATION` かつ クールダウン外）

#### ② Follow 状態への遷移（以下のいずれかを満たした場合）
- **条件1（正面・道幅不足）**:
  - 前方 $-15^\circ \sim +15^\circ$ かつ 半径 $8.0\text{ m}$ 以内に他車が存在。
  - 前方車両群の左右のどちらかの道幅（最大空き幅）が $2.2\text{ m}$（車幅 $1.4\text{ m} + 0.8\text{ m}$）以下。（※複数車の場合は最も小さい道幅を基準）
- **条件2（左側方・3秒後割り込み予測）**:
  - 現在位置が「前方 $+30^\circ \sim +150^\circ$」かつ「車体横 $y$ 軸 $0\text{ m} \sim 3\text{ m}$」に他車が存在。
  - 相対速度 $\times 3\text{ s}$ 後の予測位置が「前方 $+30^\circ \sim +90^\circ$」かつ「車体横 $y$ 軸 $0\text{ m} \sim 3\text{ m}$」に入る。
- **条件3（右側方・3秒後割り込み予測）**:
  - 現在位置が「前方 $-150^\circ \sim -30^\circ$」かつ「車体横 $y$ 軸 $-3\text{ m} \sim 0\text{ m}$」に他車が存在。
  - 相対速度 $\times 3\text{ s}$ 後の予測位置が「前方 $-90^\circ \sim -30^\circ$」かつ「車体横 $y$ 軸 $-3\text{ m} \sim 0\text{ m}$」に入る。

#### ③ Overtake 状態への遷移（以下のすべてを満たした場合）
1. 前方 $-15^\circ \sim +15^\circ$ かつ 半径 $8.0\text{ m}$ 以内に他車が存在。
2. 左側方（$+30^\circ \sim +150^\circ$, $y \in [0, 3]\text{m}$）に他車が**存在しない**。
3. 右側方（$-150^\circ \sim -30^\circ$, $y \in [-3, 0]\text{m}$）に他車が**存在しない**。
4. 前方車両群の道幅（最大空き幅）が $2.2\text{ m}$ より**大きい**（※複数車の場合は最も小さい道幅 $> 2.2\text{ m}$）。
5. 前方車両の速度 $\le 24\text{ km/h}$（$6.67\text{ m/s}$）かつ $\text{TTC} = \frac{2.2\text{ m}}{v_{\text{ego}} - v_{\text{other}}} \le 1.0\text{ s}$（すなわち $v_{\text{ego}} - v_{\text{other}} \ge 2.2\text{ m/s}$）。

#### ④ FollowPath 状態の維持
- 上記のいずれにも該当しない場合、`FollowPath` 状態を維持。

---

### 2. 幾何計算と予測処理の実装
- `mpc_controller.py` に `_scan_surrounding_vehicles(pose, v)` を追加。
- 各他車について自車ローカル座標 $(x_{\text{rel}}, y_{\text{rel}})$、方位角 $\alpha_{\text{deg}}$、距離 $r$、相対速度 $\Delta \vec{v}_{\text{rel}}$、3秒後予測位置 $(x_{\text{rel},3\text{s}}, y_{\text{rel},3\text{s}}, \alpha_{3\text{s}})$、および Waypoint 限界に基づく道幅を網羅的に計算し、`StateContext` に格納。

## 影響と効果
- 前方に十分な道幅があり安全な条件が揃ったときのみ確実に `Overtake` に遷移し、側方に他車がいる場合や道幅が狭い場合は安全に `Follow` に遷移する、極めて安全で無駄のない挙動が実現された。
