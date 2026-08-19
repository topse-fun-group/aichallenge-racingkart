# ADR-021: FollowState Waypoint基準センタリングステアリング補正

## ステータス
承認済み (Accepted)

## コンテキスト
`FollowState` において前走車の姿勢に追従する方式では、前走車がスピンしかけたり乱れた挙動を取った際に自車まで外側の壁へ衝突してしまう課題があった。
本決定では、前走車ではなく **「最寄り Waypoint に対する自車の横偏差 $e_y$ および方位角誤差 $e_\psi$」** を基準とし、自車を常にコース中心線へ引き戻すセンタリングステアリング補正を導入した。

## 決定事項

### 1. 制御アルゴリズム
- **状態量の定義**:
  - $e_y = \text{ctx.path\_e\_y}$: Waypoint 中心線からの横偏差（左が正、右が負）
  - $e_\psi = (\theta_{\text{ego}} - \psi_{\text{path}} + \pi) \pmod{2\pi} - \pi$: Waypoint 接線方向との方位角誤差
- **ステアリング補正量 ($\Delta \delta_{\text{follow}}$)**:
  $$\Delta \delta_{\text{follow}} = \text{clip}\left( - (0.20 \cdot e_y + 0.50 \cdot e_\psi), \ -10.0^\circ, \ +10.0^\circ \right)$$
  - 車両が Waypoint の左側にいる（$e_y > 0$）場合：負の方向（右ステア）へ補正。
  - 車両が Waypoint の右側にいる（$e_y < 0$）場合：正の方向（左ステア）へ補正。
  - ズレの大きさに応じて補正量が動的に増減。
- **ステアリング指令値への適用**:
  $$u[1] \leftarrow u[1] + \Delta \delta_{\text{follow}}$$

## 影響と効果
- 前走車の挙動が乱れても自車は常に Waypoint 中心線を強固に追従し、コーナーでの外壁衝突が解消された。
- 変更範囲を `mpc_controller.py` の L1309〜L1319 の最小限に留めた。
