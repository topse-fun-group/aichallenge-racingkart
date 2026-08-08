# multi_purpose_mpc_ros_with_dynamic_param 設計仕様書 (design.md)

本ドキュメントは、Autoware Universe をベースとしたレーシングカート向け自動運転モジュール `multi_purpose_mpc_ros_with_dynamic_param` の全体設計、内部コンポーネント構成、制御ロジック、状態遷移マシンの仕様をまとめた技術資料です。

---

## 1. データフロー図 (Data Flow)

本モジュールは ROS 2 ノード `mpc_controller` を中心として動作し、センサーデータ（Odometry, V2X, LiDAR scan）、軌道制約、動作モード切替などのトピックを受信して MPC 最適化計算を行い、車両制御指令（`AckermannControlCommand`）およびギア切替指令（`GearCommand`）をパブリッシュします。

### PlantUML データフロー図

```plantuml
@startuml data_flow
!theme vivid
skinparam componentStyle uml2

package "Localization & Sensing" {
  [Odometry Provider] --> [ /localization/kinematic_state ] : nav_msgs/Odometry
  [V2X Receiver] --> [ /v2x/vehicle_positions ] : v2x_msgs/V2XVehiclePositionArray
  [laserscan_generator] --> [ /sensing/lidar/scan ] : sensor_msgs/LaserScan
}

package "Planning & Environment" {
  [Trajectory Planner] --> [ /planning/scenario_planning/trajectory ] : autoware_auto_planning_msgs/Trajectory
  [Path Constraints Provider] --> [ /path_constraints_provider/path_constraints ] : multi_purpose_mpc_ros_msgs/PathConstraints
  [Border Cells Provider] --> [ /path_constraints_provider/border_cells ] : multi_purpose_mpc_ros_msgs/BorderCells
}

package "System & Simulator" {
  [Autoware System/UI] --> [ /control/control_mode_request_topic ] : std_msgs/Bool
  [Autoware System/UI] --> [ /control/mpc/stop_request ] : std_msgs/Empty
  [AWSIM Simulator] --> [ /awsim/status ] : std_msgs/Float32MultiArray
  [AWSIM Pitstop] --> [ /aichallenge/pitstop/condition ] : std_msgs/Int32
}

node "multi_purpose_mpc_ros_with_dynamic_param" {
  component "MPCController" as NodeMain {
    portin p_odom
    portin p_v2x
    portin p_scan
    portin p_traj
    portin p_mode
    portin p_stop
    
    portout p_cmd
    portout p_cmd_raw
    portout p_boost
    portout p_gear
    portout p_state
    portout p_viz
  }
}

[ /localization/kinematic_state ] --> p_odom
[ /v2x/vehicle_positions ] --> p_v2x
[ /sensing/lidar/scan ] --> p_scan
[ /planning/scenario_planning/trajectory ] --> p_traj
[ /control/control_mode_request_topic ] --> p_mode
[ /control/mpc/stop_request ] --> p_stop

p_cmd --> [ /control/command/control_cmd ] : autoware_auto_control_msgs/AckermannControlCommand
p_cmd_raw --> [ /control/command/control_cmd_raw ] : autoware_auto_control_msgs/AckermannControlCommand
p_boost --> [ /boost_commander/command ] : multi_purpose_mpc_ros_msgs/AckermannControlBoostCommand
p_gear --> [ /control/command/gear_cmd ] : autoware_auto_vehicle_msgs/GearCommand
p_state --> [ /mpc/driving_state ] : std_msgs/String
p_viz --> [ /mpc/prediction ] : visualization_msgs/MarkerArray
p_viz --> [ /mpc/ref_path ] : visualization_msgs/MarkerArray

@enduml
```

### 入出力トピック詳細

| トピック名 | 型 | 種別 | 説明 |
|---|---|---|---|
| `/localization/kinematic_state` | `nav_msgs/msg/Odometry` | Sub | 自車の現在位置 $(x, y)$、姿勢 (yaw $\theta$)、縦速度 $v$ |
| `/v2x/vehicle_positions` | `v2x_msgs/msg/V2XVehiclePositionArray` | Sub | V2X通信による周囲他車の位置配列（相対/絶対座標） |
| `/sensing/lidar/scan` | `sensor_msgs/msg/LaserScan` | Sub | 2D LiDARスキャンデータ（障害物判定・追越可能幅計算） |
| `planning/scenario_planning/trajectory` | `autoware_auto_planning_msgs/msg/Trajectory` | Sub | 外部から供給される動的参照軌道（QoS: BEST_EFFORT） |
| `/path_constraints_provider/path_constraints` | `multi_purpose_mpc_ros_msgs/msg/PathConstraints` | Sub | コース壁面・境界に基づくパス制約 |
| `/control/command/control_cmd` | `autoware_auto_control_msgs/msg/AckermannControlCommand` | Pub | 制御指令（ステア角、目標速度、加速度） |
| `/control/command/gear_cmd` | `autoware_auto_vehicle_msgs/msg/GearCommand` | Pub | ギア切替指令 (`DRIVE`=2, `REVERSE`=20) |
| `/mpc/driving_state` | `std_msgs/msg/String` | Pub | 現在の走行状態 (`follow_path`, `follow`, `overtake`, `recovery`) |

---

## 2. クラス図 (Class Diagram)

本モジュールは、ROS 2 ノードクラス `MPCController` を統括オブジェクトとし、ステートパターン (`DrivingState`, `StateManager`)、センサ処理 (`LidarProcessor`, `V2XVehicleTracker`)、空間キネマティックモデル (`BicycleModel`, `SimpleSpatialState`)、および OSQP二次計画法ソルバー (`MPC`) が疎結合に連携するオブジェクト指向設計となっています。

### PlantUML クラス図

```plantuml
@startuml class_diagram
!theme vivid
skinparam classAttributeIconSize 0

class Node {
}

class MPCController {
  - _config_path: str
  - _cfg: NamedTuple
  - _odom: Odometry
  - _enable_control: bool
  - _current_lateral_offset: float
  - _target_lateral_offset: float
  - _state_manager: StateManager
  - _lidar_processor: LidarProcessor
  - _v2x_tracker: V2XVehicleTracker
  - _mpc: MPC
  - _car: BicycleModel
  - _reference_path: ReferencePath
  - _map: Map
  + __init__(config_path: str, ref_vel_config_path: str)
  + _control(): void
  - _build_state_context(dt: float, is_colliding: bool): StateContext
  - _apply_state_params(params: MPCStateParams): void
}

class StateManager {
  - _node: Node
  - _states: Dict[str, DrivingState]
  - _current: DrivingState
  - _last_transition_time: float
  - _state_pub: Publisher
  + update(ctx: StateContext): Optional[MPCStateParams]
  + get_control_override(ctx: StateContext): Optional[Tuple]
  + current_state_name: str
  + current_gear: int
}

abstract class DrivingState {
  + name: str {abstract}
  + gear: int
  + get_params(): MPCStateParams {abstract}
  + check_transition(ctx: StateContext): Optional[str] {abstract}
  + compute_control_override(ctx: StateContext): Optional[Tuple]
  + on_enter(ctx: StateContext): void
  + on_exit(ctx: StateContext): void
}

class FollowPathState {
  + V_MAX: float = 35.0
  + AY_MAX: float = 9.5
  + check_transition(ctx: StateContext): Optional[str]
}

class FollowState {
  + TARGET_FOLLOWING_DISTANCE: float = 10.0
  + get_adjusted_v_max_kmh(ctx: StateContext): float
  + check_transition(ctx: StateContext): Optional[str]
}

class OvertakeState {
  + MAX_OVERTAKE_DURATION: float = 2.5
  - _calculated_offset: float
  + check_transition(ctx: StateContext): Optional[str]
}

class RecoveryState {
  + WAIT_DURATION: float = 2.0
  + BACK_SPEED: float = -2.5
  - _phase: str
  + compute_control_override(ctx: StateContext): Tuple
  + check_transition(ctx: StateContext): Optional[str]
}

class StateContext <<dataclass>> {
  + current_time_sec: float
  + dt: float
  + pose_x: float
  + pose_y: float
  + pose_theta: float
  + velocity: float
  + is_colliding: bool
  + path_deviation: float
  + forward_vehicle_distance: Optional[float]
  + forward_vehicle_speed: Optional[float]
  + overtake_width_left: float
  + overtake_width_right: float
  + time_stopped_sec: float
  + is_in_recovery_cooldown: bool
}

class MPCStateParams <<dataclass>> {
  + v_max: float
  + ay_max: float
  + Q: List[float]
  + R: List[float]
  + QN: List[float]
  + lateral_offset: float
}

class LidarProcessor {
  - _scan: LaserScan
  + get_forward_clearance(half_angle_deg: float): Optional[float]
  + get_overtake_widths(): Tuple[float, float]
}

class V2XVehicleTracker {
  - _samples: Dict
  - _velocities: Dict
  + update(msg: V2XVehiclePositionArray): void
  + velocity(vehicle_id: str): Tuple[float, float]
  + predict_all(t_samples): Dict
}

class MPC {
  - model: BicycleModel
  - N: int
  - Q: dia_matrix
  - R: dia_matrix
  - QN: dia_matrix
  + get_control(): Tuple[np.ndarray, float]
  + update_v_max(v_max: float): void
  + update_Q(Q: dia_matrix): void
  + update_R(R: dia_matrix): void
}

class BicycleModel {
  + reference_path: ReferencePath
  + length: float
  + width: float
  + temporal_state: TemporalState
  + spatial_state: SimpleSpatialState
  + update_states(x: float, y: float, psi: float): void
}

Node <|-- MPCController
MPCController *-- StateManager
MPCController *-- LidarProcessor
MPCController *-- V2XVehicleTracker
MPCController *-- MPC
MPCController *-- BicycleModel

StateManager *-- "4" DrivingState
DrivingState <|-- FollowPathState
DrivingState <|-- FollowState
DrivingState <|-- OvertakeState
DrivingState <|-- RecoveryState

StateManager ..> StateContext : Consumes
StateManager ..> MPCStateParams : Produces
MPC *-- BicycleModel

@enduml
```

---

## 3. シーケンス図 (Sequence Diagram)

40Hz (25ms周期) のタイマー制御ループ `_control()` におけるデータ収集、状態遷移評価、MPC動的パラメータ適用、OSQPによる制御入力算出、パブリッシュの流れを示します。

### PlantUML シーケンス図

```plantuml
@startuml sequence_diagram
!theme vivid
autonumber

participant "ROS2 Clock / Timer" as Timer
participant "MPCController" as Main
participant "LidarProcessor" as Lidar
participant "V2XVehicleTracker" as V2X
participant "StateManager" as StateMgr
participant "DrivingState\n(Current)" as CurrState
participant "BicycleModel" as Vehicle
participant "MPC Solver\n(OSQP)" as Solver
participant "ROS2 Topics" as Topics

Timer -> Main : _control() [40Hz]
activate Main

Main -> Lidar : get_overtake_widths()
activate Lidar
Lidar --> Main : (width_left, width_right)
deactivate Lidar

Main -> V2X : active_vehicle_ids() / velocity()
activate V2X
V2X --> Main : (fwd_distance, fwd_speed)
deactivate V2X

Main -> Main : _build_state_context(dt, is_colliding) -> ctx

Main -> StateMgr : update(ctx)
activate StateMgr
StateMgr -> CurrState : check_transition(ctx)
activate CurrState
CurrState --> StateMgr : next_state_name (or None)
deactivate CurrState

alt State Transition Occurred (e.g. follow_path -> overtake)
  StateMgr -> CurrState : on_exit(ctx)
  StateMgr -> StateMgr : switch state & call on_enter(ctx)
  StateMgr --> Main : new_params (MPCStateParams)
  Main -> Main : _apply_state_params(new_params)\n[Update v_max, ay_max, Q, R, QN, speed_profile]
else No Transition
  StateMgr --> Main : None
end
deactivate StateMgr

alt Recovery State Active (Control Override)
  Main -> StateMgr : get_control_override(ctx)
  StateMgr -> CurrState : compute_control_override(ctx)
  CurrState --> StateMgr : (speed, steer, accel)
  StateMgr --> Main : override_cmd
  Main -> Topics : publish /control/command/control_cmd & gear_cmd (REVERSE/DRIVE)
else Normal Driving State
  alt FollowState Active
    Main -> CurrState : get_adjusted_v_max_kmh(ctx)
    CurrState --> Main : dynamic_v_max
    Main -> Solver : update_v_max(dynamic_v_max)
  end

  Main -> Main : Smooth interpolate lateral offset\n(x_shifted, y_shifted)
  Main -> Vehicle : update_states(x_shifted, y_shifted, theta)
  
  Main -> Solver : get_control()
  activate Solver
  Solver -> Solver : Solve OSQP QP Problem
  Solver --> Main : u = [v_cmd, steer_cmd], max_delta
  deactivate Solver

  Main -> Main : Apply low-pass filter & steering tire gain
  Main -> Topics : publish /control/command/control_cmd
  Main -> Topics : publish /control/command/gear_cmd (DRIVE)
  Main -> Topics : publish /mpc/driving_state & visualization markers
end

deactivate Main
@enduml
```

---

## 4. 状態遷移図 (State Transition Diagram)

本システムは 4 つの走行状態 (`follow_path`, `follow`, `overtake`, `recovery`) を持ち、周囲の車両状況・道路幅・衝突/スタック検知に応じて自律的にモードを遷移します。

### PlantUML 状態遷移図

```plantuml
@startuml state_transition
!theme vivid

[*] --> follow_path : 初期化完了 (35 km/h)

state follow_path {
  follow_path : 目的: 最速レーシングライン追従
  follow_path : v_max = 35 km/h, ay_max = 9.5 m/s^2
}

state follow {
  follow : 目的: 先行車追従 (安全距離 10m 保持)
  follow : 動的速度制御 (P制御: v_max 調整)
}

state overtake {
  follow : 目的: 横オフセット追従による追い越し
  follow : 左右空間幅に応じた動的横シフト (±1.2~2.2m)
  follow : 最大存続時間: 2.5s
}

state recovery {
  recovery : 目的: 衝突/スタック脱出
  state "1. Wait Phase (2.0s 停止)" as WaitPhase
  state "2. Back Phase (1.5~3.5s 後退)" as BackPhase
  WaitPhase --> BackPhase : 2.0s 経過
}

follow_path --> follow : 前方15m以内にV2X車両検知\nAND 左右空き幅 ≤ 2.3m
follow_path --> overtake : 前方15m以内にV2X車両検知\nAND 左右空き幅 > 2.3m
follow_path --> recovery : 衝突検知 OR スタック検知 (速度<0.3m/sが8s継続)

follow --> overtake : 左右空き幅 > 2.3m を検知
follow --> follow_path : 前方車両が消脱 (距離 ≥ 15m)
follow --> recovery : 衝突検知 OR スタック検知

overtake --> follow_path : 前方車両通過完了 OR 2.5s経過 (タイムアウト)
overtake --> recovery : 衝突検知 OR スタック検知

BackPhase --> follow_path : パス偏差 < 2.0m OR 最大後退時間3.5s経過\n(復帰後 5s 間はクールダウン)

@enduml
```

---

## 5. 状態推定・状態遷移・MPCパラメータ選定のロジック詳細

### 5.1 状態推定ロジック (State Estimation)

1. **空間キネマティック状態量 ($e_y, e_\psi, t$)**:
   - 自車位置 $(x, y, \theta)$ から、参照軌道上の最寄ウェイポイント $(x_{ref}, y_{ref}, \psi_{ref})$ を探索します。
   - 横偏差 $e_y$: 参照軌道の中心線からの直交離脱距離。
     $$e_y = -(x - x_{ref})\sin\psi_{ref} + (y - y_{ref})\cos\psi_{ref}$$
   - 方位偏差 $e_\psi$: 参照軌道接線ベクトルに対する自車姿勢角の誤差。
     $$e_\psi = \theta - \psi_{ref}$$
2. **V2X 車両追跡 (`V2XVehicleTracker`)**:
   - V2Xトピックから受信した各車両 ID の位置時系列から有限差分法により速度ベクトル $(v_x, v_y)$ を算出します。
   - 位置情報の急変（マルチパス・通信エラー等）を検出するため、位置ジャンプ閾値 (`position_jump_threshold = 5.0m`) を設け、閾値超過時は速度推定量のリセットを行います。
3. **LiDAR 追越可能幅推定 (`LidarProcessor`)**:
   - 前方スキャンを左右アジマス角で分割し、障害物の左右に存在する最大スキャン距離を空間空き幅 (`overtake_width_left`, `overtake_width_right`) として推定します。

### 5.2 状態遷移ロジック (State Transition Logic)

- **チャタリング防止 (Minimum Dwell-Time)**:
  - `StateManager` は非 Recovery 遷移において、同一状態に最低 `MIN_DWELL_TIME = 1.0s` 滞留することを保証します。
- **Recovery クールダウンメカニズム**:
  - `RecoveryState` 脱出直後は自車姿勢や速度が過渡状態にあるため、脱出後 5.0秒間（および起動後 10.0秒間）は `is_in_recovery_cooldown = True` とし、スタック判定による再度の誤 Recovery 遷移を抑制します。
- **スタック判定 (Stuck Detection)**:
  - 自車速度 $v < 0.3\text{ m/s}$ の状態が $8.0\text{ 秒}$ 継続した場合、自動的に Recovery モードへ移行します。

### 5.3 MPC パラメータ選定ロジック (MPC Parameter Selection Rationale)

1. **横加速度制限 $a_{y,max}$ とスピードプロファイル**:
   - カーブ通過時の許容限界横加速度を設定し、曲率 $\kappa$ に対する限界速度 $v_{curve} = \sqrt{a_{y,max} / |\kappa|}$ を算出して速度プロファイルを作成します。
2. **コスト行列 $Q, R, QN$ の役割**:
   - $Q_0$ (横偏差 $e_y$ の重み): レーシングライン追従精度を高めるため $1,000,000.0$ などの高い値を設定。
   - $Q_1$ (方位偏差 $e_\psi$ の重み): 車首方位の乱れを抑えるため $100,000,000.0$ を設定。
   - $R_1$ (ステア入力重み): ステアリング微振動を抑止するため $100.0 \sim 1000.0$ に設定。

---

## 6. 各MPCパラメータの意味 (MPC Parameters Rationale)

| パラメータ名 | 設定値（標準） | 単位 | 意味と役割 |
|---|---|---|---|
| `N` | `20` | ステップ | MPCの予測ホライズンステップ数 (40Hz制御時: 0.5秒先まで予測) |
| `Q0` (または `Q[0]`) | `1,000,000.0` | - | 状態量 $e_y$ (横偏差) に対するコスト重み。直線・カーブでのライン維持力 |
| `Q1` (または `Q[1]`) | `100,000,000.0` | - | 状態量 $e_\psi$ (方位偏差) に対するコスト重み。スピン・ふらつき防止 |
| `Q2` (または `Q[2]`) | `850,000.0` | - | 状態量 $t$ (時間偏差) に対するコスト重み。目標速度への追従性 |
| `R0` (または `R[0]`) | `100,000.0` | - | 制御入力 $v$ (速度指令) の変化に対するコスト重み |
| `R1` (または `R[1]`) | `100.0` / `1000.0` | - | 制御入力 $\delta$ (ステア指令) に対するコスト重み。ハンチング抑制 |
| `QN0..QN2` | `[1M, 1k, 10k]` | - | 終端状態 (Terminal state) に対する各誤差のコスト重み |
| `v_max` | `35.0` | km/h | 直線区間での最高目標速度上限 |
| `a_min` | `-3.0` | m/s^2 | 最大減速加速度 (ブレーキ性能限界) |
| `a_max` | `2.0` | m/s^2 | 最大加速加速度 (エンジン/モータ出力限界) |
| `ay_max` | `9.5` | m/s^2 | コーナリング時の最大許容横加速度 (グリップ限界基準) |
| `delta_max_deg` | `32.0` | deg | 最大ステアリングタイア角制限 |
| `steer_rate_max` | `0.35` | rad/s | 1秒あたりの最大操舵角速度制限 |
| `steering_tire_angle_gain_var` | `1.639` | - | カート実機・AWSIMのステアリングギヤ比/入力変換ゲイン補正値 |
| `accel_low_pass_gain` | `0.8` | - | 加速度出力一次遅れフィルタのゲイン (1.0でフィルタ無効) |
| `steer_low_pass_gain` | `0.4` | - | ステア指令のローパスフィルタゲイン (高周波ノイズ・振動カット) |

---

## 7. 各状態の制御詳細 (Per-State Control Details)

### 7.1 `follow_path` 状態 (通常レーシングライン追従)
- **制御目標**: 最速タイムアタックを達成するため、設計された参照軌道（Min-Curvature Raceline）を最高 35 km/h で高精度に追従します。
- **制御ロジック**: 通常の空間MPCがアクティブとなり、$x_{shifted} = x, y_{shifted} = y$ の基準位置で OSQP 最適化問題を計算します。

### 7.2 `follow` 状態 (先行車追従)
- **制御目標**: 前方に低速車が存在し、かつ追い越し不可能な狭い区間において、安全な車間距離 (10.0m) を保ち追突を回避します。
- **動的速度調整**:
  $$v_{target} = \text{clip}(v_{leader} + 0.5 \times (d_{fwd} - 10.0), 20.0, 35.0) \text{ [km/h]}$$
  先行車の速度 $v_{leader}$ と距離誤差に応じて MPC の $v_{max}$ を動的に書き換え、最低速度 20 km/h を維持しながらスムーズに追従します。

### 7.3 `overtake` 状態 (動的横オフセット追越)
- **制御目標**: 先行車の左右に十分な幅 (> 2.3m) を検出した際、レーシングラインから横方向にシフトして安全かつ速やかに追い越しを行います。
- **動的横シフト座標変換**:
  LiDARから得られた空き領域幅に応じて、左追越時 $\text{offset} \in [+1.2, +2.2]\text{m}$、右追越時 $\text{offset} \in [-1.2, -2.2]\text{m}$ を動的に計算します。
  自車座標を以下のようにシフトして MPC に入力することで、レーシングラインそのものを変更することなくスムーズな車線変更・追い越しを実現します。
  $$x_{shifted} = x - \text{offset} \times \sin\theta$$
  $$y_{shifted} = y + \text{offset} \times \cos\theta$$
- **復帰条件**: 追い越し開始から `MAX_OVERTAKE_DURATION = 2.5s` 経過後、自動的に `follow_path` へ復帰し、オフセットをスムーズに 0m に戻します。

### 7.4 `recovery` 状態 (衝突・スタック脱出)
- **制御目標**: 壁への接触やスタック状態から自律的にバック・再方向修正を行って脱出し、走行を継続します。
- **フェーズ1 (`wait`)**:
  衝突検知後 2.0 秒間は速度 $0.0\text{ m/s}$ で完全停止し、車両慣性を収束させます。
- **フェーズ2 (`back`)**:
  ギアを `GEAR_REVERSE` (20) に切り替え、後退速度 $-2.5\text{ m/s}$、加速度 $2.5\text{ m/s}^2$ を直接指令（MPCをバイパス）します。
  ステアリングは参照軌道の接線方位に対し自車ノーズを正しく向ける P 制御を実施します:
  $$\psi_{target} = \psi_{ref} \mp 10^\circ$$
  $$\delta_{cmd} = \text{clip}(1.2 \times (\theta - \psi_{target}), -0.55, 0.55)$$
- **復帰条件**: パス離脱距離 $e_y < 2.0\text{m}$ に復帰、または最大後退時間 3.5秒 経過後に `follow_path` へ自動復帰します。
