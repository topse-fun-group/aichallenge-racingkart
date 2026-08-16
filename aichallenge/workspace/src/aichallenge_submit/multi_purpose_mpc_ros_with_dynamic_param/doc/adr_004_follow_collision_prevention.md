# ADR-004: FollowStateにおける発進時追突防止と車間距離ベース速度ガバナー (FollowState Collision Prevention & Distance Governor)

## ステータス
承認待ち (Proposed)

## コンテキスト
先行車の後ろで停止した状態から、先行車が発進した際に、自車が過剰に加速して先行車に追突する事象が観測された。

### 原因の分析
1. **`ref_vel_configulator` による追従速度の強制上書き**:
   `mpc_controller.py` 内で、`FollowState` が先行車の速度に合わせて算出した目標速度 `v_ref`（低速〜停止）が、直後の `self._ref_vel_configulator` によってコース規定速度（35 km/h 等）で無条件に上書きされていた。
2. **車間距離不足時の速度上限キャップ（ガバナー）の欠如**:
   停止状態からの発進直後は車間距離が目標（10m）より狭い（例: 3〜5m）状態にあるが、先行車速度以下に厳格に制限する安全ガバナーが存在せず、MPC が急加速していた。

## 決定事項

### 1. `FollowState` 実行時の `ref_vel_configulator` 上書き遮断
`FollowState` がアクティブな際は、`ref_vel_configulator` による目標速度の上書きをスキップし、`FollowState` が算出した安全追従速度 `dynamic_v_max` を最優先で MPC ソルバーおよび `v_ref` に適用する。

### 2. 車間距離に応じた適応型速度ガバナー (Adaptive Distance Governor)
先行車との距離 $d$ に基づき、目標速度 $v_{\text{target}}$ を厳格に制限する：
- $d \le 3.5\text{m}$: $v_{\text{target}} = 0.0\text{ m/s}$（完全停止を維持）
- $3.5\text{m} < d < 10.0\text{m}$: $v_{\text{target}} \le v_{\text{leader}} \times \left(\frac{d - 3.5}{6.5}\right)$（先行車の速度以下に滑らかに制限）
- $d \ge 10.0\text{m}$: $v_{\text{target}} = v_{\text{leader}} + K_p (d - 10.0)$（通常追従）

## 影響と効果
- 先行車が停止から動き出した直後、先行車の速度を超えて急加速することが物理的に阻止される。
- 安全な車間距離（10m）を確保しながら、先行車の加速に合わせて滑らかに追従発進する。
