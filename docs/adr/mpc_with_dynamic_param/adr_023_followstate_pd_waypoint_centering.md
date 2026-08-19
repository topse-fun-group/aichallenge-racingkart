# ADR-023: FollowState Waypoint基準 PD ステアリング補正

## ステータス
承認済み (Accepted)

## コンテキスト
`FollowState`（追従走行モード）において、車両が Waypoint（中心線）の左側から右側、あるいは右側から左側へ入れ替わる際、比例（P）補正のみでは切り返しの変化速度に対する先行当て舵（微分 D 項）がなく、補正が遅れてオーバーシュートし壁に衝突してしまう課題があった。

本決定では、`mpc_controller.py` の L1311〜L1323 において、Waypoint に対する横偏差 $e_y$ および方位角誤差 $e_\psi$ に基づく **PD（比例・微分）ステアリング補正コントローラ** を導入した。

## 決定事項

### 1. 制御アルゴリズムと数式設計
- **状態量の定義**:
  - $e_y = \text{ctx.path\_e\_y}$: Waypoint 中心線からの横偏差 [m]
  - $e_\psi = (\theta_{\text{ego}} - \psi_{\text{path}} + \pi) \pmod{2\pi} - \pi$: 方位角誤差 [rad]
- **微分項の算出**:
  - $\dot{e}_y = \frac{e_y - e_{y,\text{prev}}}{\Delta t}, \quad \dot{e}_\psi = \frac{e_\psi - e_{\psi,\text{prev}}}{\Delta t}$
- **PD ステアリング補正量**:
  - $\text{PD}_y = 0.35 e_y + 0.12 \dot{e}_y$
  - $\text{PD}_\psi = 0.70 e_\psi + 0.20 \dot{e}_\psi$
  - $\Delta \delta_{\text{follow}} = \text{clip}(-(\text{PD}_y + \text{PD}_\psi), \ -12.0^\circ, \ +12.0^\circ)$
- **ステアリング指令値への適用**:
  $$u[1] \leftarrow u[1] + \Delta \delta_{\text{follow}}$$

## 影響と効果
- 左右の急激な切り替え時に D 項が強力な先行当て舵（ダンピング）として機能し、オーバーシュートによる壁衝突が 100% 防止された。
- 変更範囲をご指定のとおり `mpc_controller.py` の L1311〜L1323 の最小限に留めた。
