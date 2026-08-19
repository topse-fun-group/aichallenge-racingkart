# ADR-020: FollowState 前走車姿勢連動ステアリング補正

## ステータス
承認済み (Accepted)

## コンテキスト
`FollowState`（追従走行モード）において、特に急コーナー進入時に前走車が旋回姿勢に入った際、自車の MPC 出力が前走車の曲がり角に追従しきれず、ステアリング舵角が不足して外側の壁へ衝突してしまう事象が発生していた。

本決定では、`mpc_controller.py` の L1309〜L1318 において、前走車の方位角（heading $\psi_{\text{leader}}$）と自車の方位角（$\theta_{\text{ego}}$）の差分 $\Delta \psi$ を計算し、前走車が向いている方向へステアリング補正値を直接加算する適応型ステアリングアシスト制御を導入した。

## 決定事項

### 1. 制御アルゴリズム
- **姿勢角誤差 ($\Delta \psi$)**:
  $$\Delta \psi = (\psi_{\text{leader}} - \theta_{\text{ego}} + \pi) \pmod{2\pi} - \pi$$
- **距離スケーリング係数 ($K_{\text{dist}}$)**:
  $$K_{\text{dist}} = \text{clip}\left( \frac{10.0 - d_{\text{fwd}}}{10.0 - 2.5}, 0.2, 1.0 \right)$$
- **ステアリングアシスト補正角 ($\Delta \delta_{\text{follow}}$)**:
  $$\Delta \delta_{\text{follow}} = \text{clip}\left( 0.8 \times \Delta \psi \times K_{\text{dist}}, \ -12.0^\circ, \ +12.0^\circ \right)$$
- **ステアリング指令値への適用**:
  $$u[1] \leftarrow u[1] + \Delta \delta_{\text{follow}}$$

## 影響と効果
- コーナー進入時に前走車の旋回姿勢に合わせて自車も即座に旋回ステアが加算されるため、ステアリング不足による外壁衝突が解消された。
- 車間距離に応じた適応スケーリングにより、遠方では緩やかに、接近時はしっかり追従する自然なカルマン追従動作が実現された。
