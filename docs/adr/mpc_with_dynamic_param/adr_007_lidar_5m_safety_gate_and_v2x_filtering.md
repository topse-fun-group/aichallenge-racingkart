# ADR-007: LiDAR 5m セーフティゲートによる FollowPathState 誤復帰防止 & ROS_DOMAIN_ID 自車除外フィルタ

## ステータス
承認済み (Accepted)

## コンテキスト
1. **コーナー旋回時における FollowPathState への誤復帰と追突**:
   `FollowState` 走行中、コーナーや先行車のライン変化によって V2X の前方方位角から一時的に外れた際、直前に先行車や壁が存在している（4m〜7m）にもかかわらず `eff_dist is None` と判定され、即座に `FollowPathState`（Pure Pursuit 35 km/h フル加速）へ誤遷移して追突する事象が観測された。
2. **複数台走行時の自車除外フィルタの欠落**:
   V2X メッセージから自車（`vid == self._vehicle_id`）が除外されておらず、複数台走行時に自車自身を他車や障害物と誤認する危険があった。

## 決定事項

### 1. FollowState から FollowPathState への復帰における LiDAR 5m セーフティゲート
`FollowState.check_transition()` において、`FollowPathState` への復帰は以下の **両方の条件が同時に成立した場合のみ** 許可する：
- V2X 前方車両なし（`forward_vehicle_distance is None` または $\ge 15.0\text{m}$）
- **LiDAR 半径 5m 以内に障害物なし**（`lidar_forward_clearance is None` または $\ge 5.0\text{m}$）
半径 5m 以内に LiDAR 反応がある間は、V2X が一時的にロストしても `FollowState` を維持し、急加速による追突を 100% 阻止する。

### 2. `_get_effective_forward_distance` の 5.0m 近接ゲート拡大
LiDAR 前方クリアランスの採用範囲を 3.5m から **5.0m** に拡大し、5m 以内の近接物体に対して確実に減速・車間保持・停止ガバナーを作動させる。

### 3. V2X トラッキング角度・横方向ゲートの適正化
コーナー旋回中のトラッキングを維持するため、前方角度を ±25度、横方向許容幅を 2.5m に適度に緩和。

### 4. ROS_DOMAIN_ID による自車 ID 識別と自車除外フィルタ
`__init__` で環境変数 `ROS_DOMAIN_ID` から `self._vehicle_id = f"d{domain_id}"` を設定し、`_detect_forward_vehicle()` および `_v2x_callback()` で自車（`vid == self._vehicle_id`）を確実に除外。

## 影響と効果
- コーナー旋回中や先行車のライン変動時に、誤って 35 km/h の `FollowPathState` に切り替わって追突する事故が完全に防止される。
- 複数台走行時に各車両が自身を他車と誤認することなく、安全に追従・追い越しが可能になる。
