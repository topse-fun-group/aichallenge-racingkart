# ADR-018: 追い越し時の動的適応ステアリング加算角制御

## ステータス
承認済み (Accepted)

## コンテキスト
`OvertakeState` において直接ステアリング角 $u[1]$ に加算角（例: $\pm 3^\circ$）を与えることで追い越しの成功率が大きく向上することが実験で確認された。
本決定では、この加算制御をさらに高度化し、「検出された空き道幅」と「前走車との距離」を動的に評価して最大加算角 $\alpha_{\max}$（$8.0^\circ$）のスケーリング率 $P_{\text{steer}} \in [0.0, 1.0]$ を決定する適応型制御を導入した。

## 決定事項

### 1. 動的適応加算角の数式設計
- **最大加算角**: $\alpha_{\max} = 8.0^\circ \approx 0.1396\text{ rad}$
- **空き道幅比率係数 ($K_{\text{width}}$)**:
  $$K_{\text{width}} = \text{clip}\left( \frac{W_{\text{avail}} - 1.0}{2.5 - 1.0}, 0.0, 1.0 \right)$$
  - 道幅 $2.5\text{m}$ 以上で $100\%$ 加算、道幅 $1.0\text{m}$ 未満では $0\%$ に絞り壁接触を防止。
- **距離近接係数 ($K_{\text{dist}}$)**:
  $$K_{\text{dist}} = \text{clip}\left( \frac{15.0 - d_{\text{fwd}}}{15.0 - 4.0}, 0.0, 1.0 \right)$$
  - $15\text{m}$ 手前から滑らかに立ち上がり、最も回避が必要な $4\text{m} \sim 6\text{m}$ で $100\%$ 加算。
- **ステアリング加算角 $\Delta \delta$**:
  $$\Delta \delta = \alpha_{\max} \times (K_{\text{width}} \times K_{\text{dist}})$$
  - 左追い越し時（`overtake_width_left >= overtake_width_right`）: $u[1] \leftarrow u[1] + \Delta \delta$
  - 右追い越し時（`overtake_width_left < overtake_width_right`）: $u[1] \leftarrow u[1] - \Delta \delta$

## 影響と効果
- 道幅が広い場所ではしっかり大舵角で素早く横へ逃げ、道幅が狭い場所では加算角を絞ることで反対側の壁への接触を自動防止。
- 遠方から滑らかに車線変更を開始し、安定したクリーンな追い越しが実現された。
