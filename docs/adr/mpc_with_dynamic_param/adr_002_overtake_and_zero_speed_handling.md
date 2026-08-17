# ADR-002: 前車速度0時の追い越し/停止制御 & 前車姿勢判定 & Overtake遷移問題の解消

- **ステータス**: Approved / Accepted
- **日付**: 2026-08-17
- **著者**: Antigravity & User Pair Programming

---

## 1. コンテキスト（背景）

Autoware レーシングカート制御システムにおいて、前車追従および追い越し（State Machine）の運用中に以下の 3 つの課題が発生していました：

1. **Overtake 状態に一切遷移しない**:
   従来の実装では V2X 経由の前車距離 `forward_vehicle_distance` が `None` の場合（V2X データ未受信や LiDAR のみにしか検出されない静的/動的障害物の際）、追い越し判定ブロック自体がスキップされ、`OvertakeState` に遷移しない問題がありました。
2. **前車速度 0 時の Recovery 誤動作・衝突**:
   前車が 0 km/h で停止した際、`FollowState` で一緒に減速・停止したあと、自車の停止時間が `STUCK_DURATION`（8秒）を超過すると「自車が壁スタックした」と誤判定され、`RecoveryState`（誤後退バック）に入ってしまう問題がありました。
3. **前車接近時の姿勢に起因する接触**:
   追い越し幅の判定において前車の向き（姿勢/Yaw角）が考慮されておらず、前車が自車前進方向に沿って走行しているにもかかわらず無理に側方に割り込もうとして接近・接触するリスクがありました。

---

## 2. 意思決定 (Decision)

`multi_purpose_mpc_ros_with_dynamic_param` パッケージの状態遷移システム (`states.py`, `mpc_controller.py`) において、以下の改良を導入します。

### アーキテクチャおよび判定ロジック

1. **V2X + LiDAR 前方センサ統合**:
   `effective_dist = min(forward_vehicle_distance, lidar_forward_clearance)` とし、V2X トピック非受信時でも LiDAR クリアランスを用いて前車・障害物を 100% 確実に検出します。

2. **前車姿勢（Heading Alignment）の判定**:
   前車の Yaw 角と自車の目標軌道方向 `path_psi` との絶対差分 `forward_vehicle_heading_diff` を算出します。
   - **順方向を向いている ($\le 45^\circ$)**: 前車が正常走行中のため、安全のため `Overtake` に入らず `FollowState` で追従維持。
   - **順方向を向いていない ($> 45^\circ$)**: スタックや横向き障害物化しているため、空き幅があれば安全に `OvertakeState` に遷移。

3. **前車速度 0 時の分岐と FollowState での Recovery 抑止**:
   - **前車速度 0 かつ 通り抜け幅あり (`max_side > 2.3m`)**: 即座に `OvertakeState` へ遷移して回避通過。
   - **前車速度 0 かつ 通り抜け幅なし**: `FollowState` で安全車間距離を保持して停止。この「前車待ち停止」の間は `is_waiting_for_leader = True` として `STUCK_DURATION` による `RecoveryState`（バック）への誤遷移を完全にブロックします。

---

## 3. 結果と影響 (Consequences)

### ポジティブな影響
- **Overtake 状態への確実な遷移**: LiDAR + V2X 統合により、前車や静的障害物が存在する際に `OvertakeState` へスムーズに遷移します。
- **無駄な後退（誤 Recovery）の根絶**: 前車待ちで停止している最中に突然バックを始める誤動作が完全に解消されます。
- **安心安全な追い越し走行**: 前車が順方向を向いて走行している際は追従を維持し、横向きにスタックしている車両のみをスマートに側方回避します。
