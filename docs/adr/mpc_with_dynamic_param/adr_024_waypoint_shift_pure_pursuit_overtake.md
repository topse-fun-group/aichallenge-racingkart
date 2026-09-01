# ADR-024: Waypoint補正型 Pure Pursuit 追い越し＆遷移条件拡張

## ステータス
承認済み (Accepted)

## コンテキスト
1. **追い越し追従の安定化**:
   MPC による追い越しでは、過渡状態や計算遅延によって前方の車両を避けきれずに衝突する事象が発生していた。
2. **追い越し機会の損失防止**:
   十分な道幅があるにもかかわらず、速度条件の厳しさによって `OvertakeState` に遷移せず、前走車後方で詰まってしまう問題があった。

本決定では、
- 空き幅の中央を通るように前方 N 個の Waypoint 座標を法線方向にシフト補正し、Simple Pure Pursuit で追従する追い越し制御を導入した。
- 既存 MPC との性能比較のため、`OvertakeState.USE_MPC_OVERTAKE` フラグで切り替え可能とした。
- 前方および側方にクリアランスがあり、道幅が確保されている場合の追い越し遷移条件を拡張した。

## 決定事項

### 1. Waypoint 補正型 Pure Pursuit 追い越しアルゴリズム
- **オフセット量 ($d_{\text{offset}}$)**:
  - 左追い越し時（`overtake_width_left >= overtake_width_right`）:
    $$d_{\text{offset}} = + \frac{\text{overtake\_width\_left}}{2.0} \quad (\text{クリップ } +1.0\text{m} \sim +2.2\text{m})$$
  - 右追い越し時（`overtake_width_left < overtake_width_right`）:
    $$d_{\text{offset}} = - \frac{\text{overtake\_width\_right}}{2.0} \quad (\text{クリップ } -2.2\text{m} \sim -1.0\text{m})$$
- **前方 N 個 Waypoint への Hann 窓補正**:
  自車最寄り Waypoint から前方 $N = 35$ 個に対し：
  $$(x'_k, y'_k) = (x_k, y_k) + \sin^2\left(\frac{\pi k}{N}\right) \cdot d_{\text{offset}} \cdot (-\sin\psi_k, \cos\psi_k)$$
- **Simple Pure Pursuit 追従**:
  - 注視距離 $L_d = \max(2.5, 0.4 \cdot v)$、目標速度 $35.0\text{ km/h}$。
  - シフト Waypoint 上の注視点へ Pure Pursuit 制御を適用し、滑らかな S 字回避・追い越しを実現。

### 2. 追い越し切り替えスイッチ
- `OvertakeState.USE_MPC_OVERTAKE: bool = False`（デフォルト: Pure Pursuit、`True` で MPC 追い越し）。

### 3. 追い越し遷移条件の拡張
- `has_clearance = max(ctx.overtake_width_left, ctx.overtake_width_right) >= 1.0`
- `is_slower_leader = (ctx.forward_vehicle_speed < 5.56 m/s or velocity - speed >= 1.5 m/s or speed is None)`
- `is_clear_side = not ctx.has_side_vehicle`
- 上記を満たす場合、確実に `OvertakeState` へ遷移。

## 影響と効果
- MPC の計算遅延や逆操舵に起因する前走車衝突が撲滅され、幾何学的に確実な空き幅中央ラインを通るクリーンな追い越しが実現された。
- 十分な道幅がある場合にスムーズに追い越しが発動し、周回ラップタイムが大幅に向上した。
