# ADR-009: ReferencePath & V2X による LiDAR レス追い越し・追従制御アーキテクチャ

## ステータス
承認済み (Accepted)

## コンテキスト
本番のシミュレーション環境では LiDAR センサが提供されないことが判明した。
従来の実装では障害物検知や追い越し幅の検出（`LidarProcessor`）を LiDAR に依存していたため、本番環境において以下の改修が必要となった：
1. **LiDAR 依存の完全撤廃**: センサ入力を V2X（`/v2x/vehicle_positions`）およびマップ定義（`ReferencePath`）のみに統一。
2. **幾何学的道幅計算**: V2X で取得した前走車座標 $(x_v, y_v)$ を `ReferencePath` の最近傍ウェイポイントへ投影し、コース境界（`wp.ub`, `wp.lb`）と前走車横偏差 $e_y$ から左右の空き幅（`overtake_width_left`, `overtake_width_right`）を高精度に算出。
3. **空き道幅の中央トレース追い越し**: 検出した空き領域の中央線を目標横オフセット（`target_overtake_offset`）として算出し、`OvertakeState` で MPC に動的設定。

## 決定事項

### 1. 前走車周囲の空き道幅および動的オフセット計算
`mpc_controller.py` に `_compute_v2x_overtake_corridor(fwd_pos)` メソッドを新設：
- 前走車の最近傍ウェイポイント $wp$ を探索。
- ウェイポイント中心線に対する前走車の横偏差 $e_{y,\text{vehicle}} = -dx \sin(wp.\psi) + dy \cos(wp.\psi)$ を計算。
- 車両マージン $w_{\text{margin}} = 0.9\text{m}$、壁マージン $w_{\text{wall}} = 0.6\text{m}$ を考慮：
  $$\text{width}_{\text{left}} = \max(0.0, (wp.ub - w_{\text{wall}}) - (e_{y,\text{vehicle}} + w_{\text{margin}}))$$
  $$\text{width}_{\text{right}} = \max(0.0, (e_{y,\text{vehicle}} - w_{\text{margin}}) - (wp.lb + w_{\text{wall}}))$$
- 広い側の空き領域の中央線を動的横オフセットとして算出：
  $$\text{offset}_{\text{left}} = \frac{(e_{y,\text{vehicle}} + w_{\text{margin}}) + (wp.ub - w_{\text{wall}})}{2.0}$$
  $$\text{offset}_{\text{right}} = \frac{(wp.lb + w_{\text{wall}}) + (e_{y,\text{vehicle}} - w_{\text{margin}})}{2.0}$$

### 2. 状態遷移ロジックの適正化
- **追い越し条件**:
  $\max(\text{width}_{\text{left}}, \text{width}_{\text{right}}) \ge 1.6\text{m}$ かつ（前走車停止中 $v < 0.3\text{m/s}$ または 方位角不一致 $\Delta\psi > 45^\circ$）
  $\to$ **`OvertakeState` へ遷移**
- **追従条件**:
  前方に車両が存在し、追い越し条件を満たさない場合
  $\to$ **`FollowState`（安全車間 8m 保持、停止 4m）**
- **通常走行復帰**:
  前方に車両が存在しない状態が 1.5 秒以上継続
  $\to$ **`FollowPathState` へ復帰**

### 3. OvertakeState の動的オフセット追従
`OvertakeState.on_enter()` において、`ctx.target_overtake_offset` を自動採用し、空いている道幅の真ん中を確実にトレースする。

## 影響と効果
- LiDAR が一切利用できない本番環境においても、V2X とコース幾何マップのみで 100% 高精度に追い越し・追従・復帰のステートマシンが機能する。
- 固定オフセット（例: ±1.8m）ではなく、前走車の実際の位置と道幅に応じた「最適な中央オフセット」を走行するため、壁や前走車への接触リスクが劇的に低減される。
