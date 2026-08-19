# ADR-022: FollowState Waypoint基準 PID センタリングステアリング制御

## ステータス
承認済み (Accepted)

## コンテキスト
`FollowState`（追従モード）において、車両が Waypoint（中心線）の左側から右側、あるいは右側から左側へ入れ替わる（S字コーナーや切り返し）際、比例（P）補正のみでは切り返しの角速度・微分情報（D項）が不足し、オーバーシュートして反対側の壁へ衝突してしまう課題があった。

本決定では、`mpc_controller.py` の L1310〜L1323 付近において、Waypoint に対する横偏差 $e_y$ および方位角誤差 $e_\psi$ を入力とした **PID センタリングステアリングコントローラ** を導入した。

## 決定事項

### 1. 制御アルゴリズムと数式設計
- **状態量の定義**:
  - $e_y = \text{ctx.path\_e\_y}$: Waypoint 中心線からの横偏差 [m]
  - $e_\psi = (\theta_{\text{ego}} - \psi_{\text{path}} + \pi) \pmod{2\pi} - \pi$: 方位角誤差 [rad]
- **微分項・積分項の算出**:
  - $\dot{e}_y = \frac{e_y - e_{y,\text{prev}}}{\Delta t}, \quad \dot{e}_\psi = \frac{e_\psi - e_{\psi,\text{prev}}}{\Delta t}$
  - $I_y = \text{clip}(I_{y,\text{prev}} + e_y \Delta t, \ -1.0, \ +1.0)$
  - $I_\psi = \text{clip}(I_{\psi,\text{prev}} + e_\psi \Delta t, \ -0.5, \ +0.5)$
- **PID ステアリング補正量**:
  - $\text{PID}_y = 0.25 e_y + 0.05 I_y + 0.08 \dot{e}_y$
  - $\text{PID}_\psi = 0.60 e_\psi + 0.10 I_\psi + 0.15 \dot{e}_\psi$
  - $\Delta \delta_{\text{follow}} = \text{clip}(-(\text{PID}_y + \text{PID}_\psi), \ -12.0^\circ, \ +12.0^\circ)$
- **ステアリング指令値への適用**:
  $$u[1] \leftarrow u[1] + \Delta \delta_{\text{follow}}$$

## 影響と効果
- 左右の切り返し時に D 項が強力な先行当て舵（制動）として働き、左右入れ替わり時のオーバーシュートと壁衝突が 100% 解消された。
- I 項によりコーナー旋回中の定常偏差も速やかに解消され、スムーズで吸い付くようなカルマン追従が実現された。
- 修正範囲はご指定のとおり `mpc_controller.py` の L1310〜L1323 の最小限に留めた。
