#!/usr/bin/env python3

import yaml
from typing import List, Tuple, Optional, NamedTuple
import dataclasses
import numpy as np
import os
import shutil
from datetime import datetime

# ROS 2
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Empty, Bool, Float32MultiArray, Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Pose2D

from rcl_interfaces.msg import SetParametersResult

# autoware
from autoware_auto_control_msgs.msg import AckermannControlCommand
try:
    from autoware_auto_vehicle_msgs.msg import GearCommand
    HAS_GEAR_MSG = True
except ImportError:
    GearCommand = None
    HAS_GEAR_MSG = False
from autoware_auto_planning_msgs.msg import Trajectory
from v2x_msgs.msg import V2XVehiclePositionArray
from multi_purpose_mpc_ros_with_dynamic_param.v2x_vehicle_tracker import V2XVehicleTracker

# Multi_Purpose_MPC (reference path / vehicle model のみ利用。MPC ソルバは撤去済み)
from multi_purpose_mpc_ros_with_dynamic_param.core.map import Map
from multi_purpose_mpc_ros_with_dynamic_param.core.reference_path import ReferencePath
from multi_purpose_mpc_ros_with_dynamic_param.core.spatial_bicycle_models import BicycleModel
from multi_purpose_mpc_ros_with_dynamic_param.core.utils import load_waypoints, kmh_to_m_per_sec, load_ref_path

# Project
from multi_purpose_mpc_ros_with_dynamic_param.common import convert_to_namedtuple, file_exists
from multi_purpose_mpc_ros_with_dynamic_param.simulation_logger import SimulationLogger
from multi_purpose_mpc_ros_with_dynamic_param.exexution_stats import ExecutionStats
from multi_purpose_mpc_ros_msgs.msg import AckermannControlBoostCommand
from multi_purpose_mpc_ros_with_dynamic_param.tools.reference_velocity_configulator import ReferenceVelocityConfigulator

# State machine
import multi_purpose_mpc_ros_with_dynamic_param.states as states
from multi_purpose_mpc_ros_with_dynamic_param.states import (
    StateContext,
    MPCStateParams,
    FollowState,
    ControlMode,
)
from multi_purpose_mpc_ros_with_dynamic_param.state_manager import StateManager


def array_to_ackermann_control_command(stamp, u: np.ndarray, acc: float) -> AckermannControlCommand:
    msg = AckermannControlCommand()
    msg.stamp = stamp
    msg.lateral.stamp = stamp
    msg.lateral.steering_tire_angle = u[1]
    msg.lateral.steering_tire_rotation_rate = 2.0
    msg.longitudinal.stamp = stamp
    msg.longitudinal.speed = u[0]
    msg.longitudinal.acceleration = acc
    return msg

def yaw_from_quaternion(q: Quaternion):
    sqx = q.x * q.x
    sqy = q.y * q.y
    sqz = q.z * q.z
    sqw = q.w * q.w

    # Cases derived from https://orbitalstation.wordpress.com/tag/quaternion/
    sarg = -2 * (q.x*q.z - q.w*q.y) / (sqx + sqy + sqz + sqw) # normalization added from urdfom_headers

    if sarg <= -0.99999:
        yaw = -2. * np.arctan2(q.y, q.x)
    elif sarg >= 0.99999:
        yaw = 2. * np.arctan2(q.y, q.x)
    else:
        yaw = np.arctan2(2. * (q.x*q.y + q.w*q.z), sqw + sqx - sqy - sqz)

    return yaw

def odom_to_pose_2d(odom: Odometry) -> Pose2D:
    pose = Pose2D()
    pose.x = odom.pose.pose.position.x
    pose.y = odom.pose.pose.position.y
    pose.theta = yaw_from_quaternion(odom.pose.pose.orientation)

    return pose

@dataclasses.dataclass
class MPCConfig:
    """制御に実際に効く設定値のみ。

    MPC ソルバは撤去済みなので、重み行列 (Q/R/QN) やホライズン長 N、
    ステアレート制限といったソルバ専用のフィールドは持たない。
    クラス名と ``_mpc_cfg`` 属性名は ``mpc_simulation.py`` が参照するため据え置く。

    - ``v_max``  : Pure Pursuit の目標速度キャップ [m/s]
    - ``a_min`` / ``a_max`` : 加速度指令のクランプ [m/s^2]
    - ``ay_max`` : 起動時の ``compute_speed_profile`` で使う横加速度上限 [m/s^2]
    """
    v_max: float
    a_min: float
    a_max: float
    ay_max: float
    control_rate: float
    steering_tire_angle_gain_var: float
    accel_low_pass_gain: float
    steer_low_pass_gain: float


class MPCController(Node):

    PKG_PATH: str = get_package_share_directory('multi_purpose_mpc_ros_with_dynamic_param') + "/"
    # MAX_LAPS = 6
    MAX_LAPS = 10000
    BUG_VEL = 40.0 # km/h

    SHOW_PLOT_ANIMATION = False
    PLOT_RESULTS = False
    ANIMATION_INTERVAL = 20

    KP = 100.0

    def __init__(self, config_path: str, ref_vel_config_path: Optional[str]) -> None:
        super().__init__("mpc_controller") # type: ignore

        # declare parameters
        self.declare_parameter("use_boost_acceleration", False)
        # NOTE: 障害物回避は MPC のコリドー制約専用だったため撤去済み。
        #       launch から <param name="use_obstacle_avoidance"> が渡され続けるので宣言だけ残す。
        self.declare_parameter("use_obstacle_avoidance", False)
        self.declare_parameter("use_stats", False)
        self.declare_parameter("vehicle_id", os.environ.get("VEHICLE_ID", "default"))

        # get parameters
        self.use_sim_time = self.get_parameter("use_sim_time").get_parameter_value().bool_value
        self.USE_BUG_ACC = self.get_parameter("use_boost_acceleration").get_parameter_value().bool_value
        self.use_stats = self.get_parameter("use_stats").get_parameter_value().bool_value
        self._vehicle_id = self.get_parameter("vehicle_id").get_parameter_value().string_value
        if self._vehicle_id in ["default", "A0", ""]:
            domain_id = int(os.environ.get("ROS_DOMAIN_ID", "1"))
            self._vehicle_id = f"d{domain_id}"
        self.get_logger().info(f"VEHICLE ID INITIALIZED AS: {self._vehicle_id} (ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '1')})")

        self._config_path = config_path
        self._ref_vel_config_path: Optional[str] = ref_vel_config_path
        self._cfg = self._load_config()
        self._odom: Optional[Odometry] = None
        self._enable_control = True
        self._initialize()
        self._setup_parameters_callback()
        self._setup_pub_sub()

        if self.use_sim_time:
            self.get_logger().warn("------------------------------------")
            self.get_logger().warn("use_sim_time is enabled!")
            self.get_logger().warn("------------------------------------")
        if self.USE_BUG_ACC:
            self.get_logger().warn("------------------------------------")
            self.get_logger().warn("USE_BUG_ACC is enabled!")
            self.get_logger().warn("------------------------------------")

    def _load_config(self) -> NamedTuple:

        # logging content
        with open(self._config_path, "r") as f:
            config_content = f.read()
            self.get_logger().info(
                "\n" +
                "----- config.yaml -----\n"+
                config_content + "\n" +
                "-----------------------")

        if self._ref_vel_config_path is not None:
            with open(self._ref_vel_config_path, "r") as f:
                ref_vel_config_content = f.read()
                self.get_logger().info(
                    "\n" +
                    "----- ref_vel.yaml -----\n"+
                    ref_vel_config_content + "\n" +
                    "-----------------------")

        with open(self._config_path, "r") as f:
            cfg: NamedTuple = convert_to_namedtuple(yaml.safe_load(f)) # type: ignore

        # Check if the files exist
        mandatory_files = [cfg.map.yaml_path, cfg.waypoints.csv_path] # type: ignore
        for file_path in mandatory_files:
            file_exists(self.in_pkg_share(file_path))
        return cfg

    def _create_reference_path_from_autoware_trajectory(self, trajectory: Trajectory) -> Optional[ReferencePath]:
        wp_x = [0] * len(trajectory.points)
        wp_y = [0] * len(trajectory.points)
        for i, p in enumerate(trajectory.points):
            wp_x[i] = p.pose.position.x
            wp_y[i] = p.pose.position.y

        cfg_ref_path = self._cfg.reference_path # type: ignore
        reference_path = ReferencePath(
            self._map,
            wp_x,
            wp_y,
            cfg_ref_path.resolution,
            cfg_ref_path.smoothing_distance,
            cfg_ref_path.max_width,
            cfg_ref_path.circular)

        mpc_config = self._mpc_cfg
        speed_profile_constraints = {
            "a_min": mpc_config.a_min, "a_max": mpc_config.a_max,
            "v_min": 0.0, "v_max": mpc_config.v_max, "ay_max": mpc_config.ay_max}

        if not reference_path.compute_speed_profile(speed_profile_constraints):
            return None

        return reference_path

    def _setup_parameters_callback(self) -> None:
        STATE_PARAM_MAP = {
            "stuck_velocity_threshold": ("STUCK_VELOCITY_THRESHOLD", float(states.STUCK_VELOCITY_THRESHOLD)),
            "stuck_duration": ("STUCK_DURATION", float(states.STUCK_DURATION)),
            "forward_cone_deg": ("FORWARD_CONE_DEG", float(states.FORWARD_CONE_DEG)),
            "forward_lateral_max": ("FORWARD_LATERAL_MAX", float(states.FORWARD_LATERAL_MAX)),
            "forward_vehicle_detection": ("FORWARD_VEHICLE_DETECTION", float(states.FORWARD_VEHICLE_DETECTION)),
            "side_vehicle_angle_min_deg": ("SIDE_VEHICLE_ANGLE_MIN_DEG", float(states.SIDE_VEHICLE_ANGLE_MIN_DEG)),
            "side_vehicle_angle_max_deg": ("SIDE_VEHICLE_ANGLE_MAX_DEG", float(states.SIDE_VEHICLE_ANGLE_MAX_DEG)),
            "d0_m": ("D0_M", float(states.D0_M)),
            "time_headway_sec": ("TIME_HEADWAY_SEC", float(states.TIME_HEADWAY_SEC)),
            "forward_follow_distance_m": ("FORWARD_FOLLOW_DISTANCE_M", float(states.FORWARD_FOLLOW_DISTANCE_M)),
            "follow_clear_hysteresis_sec": ("FOLLOW_CLEAR_HYSTERESIS_SEC", float(states.FOLLOW_CLEAR_HYSTERESIS_SEC)),
            "follow_stop_distance_m": ("FOLLOW_STOP_DISTANCE_M", float(states.FOLLOW_STOP_DISTANCE_M)),
            "follow_k_gap": ("FOLLOW_K_GAP", float(states.FOLLOW_K_GAP)),
            "follow_k_v": ("FOLLOW_K_V", float(states.FOLLOW_K_V)),
            "follow_min_speed_kmh": ("FOLLOW_MIN_SPEED_KMH", float(states.FOLLOW_MIN_SPEED_KMH)),
            "follow_leader_moving_mps": ("FOLLOW_LEADER_MOVING_MPS", float(states.FOLLOW_LEADER_MOVING_MPS)),
            "follow_target_distance_m": ("FOLLOW_TARGET_DISTANCE_M", float(states.FOLLOW_TARGET_DISTANCE_M)),
            "lateral_shift_enter_diff_m": ("LATERAL_SHIFT_ENTER_DIFF_M", float(states.LATERAL_SHIFT_ENTER_DIFF_M)),
            "lateral_shift_exit_diff_m": ("LATERAL_SHIFT_EXIT_DIFF_M", float(states.LATERAL_SHIFT_EXIT_DIFF_M)),
            "lateral_shift_dwell_sec": ("LATERAL_SHIFT_DWELL_SEC", float(states.LATERAL_SHIFT_DWELL_SEC)),
            "recovery_aligned_heading_deg": ("RECOVERY_ALIGNED_HEADING_DEG", float(states.RECOVERY_ALIGNED_HEADING_DEG)),
            "recovery_aligned_e_y_m": ("RECOVERY_ALIGNED_E_Y_M", float(states.RECOVERY_ALIGNED_E_Y_M)),
            "recovery_steer_k": ("RECOVERY_STEER_K", float(states.RECOVERY_STEER_K)),
            "recovery_boost_value": ("RECOVERY_BOOST_VALUE", float(states.RECOVERY_BOOST_VALUE)),
            "recovery_boost_duration_sec": ("RECOVERY_BOOST_DURATION_SEC", float(states.RECOVERY_BOOST_DURATION_SEC)),
            "min_overtake_width_m": ("MIN_OVERTAKE_WIDTH_M", float(states.MIN_OVERTAKE_WIDTH_M)),
            "min_overtake_lead_speed": ("MIN_OVERTAKE_LEAD_SPEED", float(states.MIN_OVERTAKE_LEAD_SPEED)),
            "overtake_target_speed_kmh": ("OVERTAKE_TARGET_SPEED_KMH", float(states.OVERTAKE_TARGET_SPEED_KMH)),
            "overtake_corner_kappa": ("OVERTAKE_CORNER_KAPPA", float(states.OVERTAKE_CORNER_KAPPA)),
            "overtake_corner_lookahead_m": ("OVERTAKE_CORNER_LOOKAHEAD_M", float(states.OVERTAKE_CORNER_LOOKAHEAD_M)),
            "overtake_corner_max_dist_m": ("OVERTAKE_CORNER_MAX_DIST_M", float(states.OVERTAKE_CORNER_MAX_DIST_M)),
            "overtake_corner_speed_margin_mps": ("OVERTAKE_CORNER_SPEED_MARGIN_MPS", float(states.OVERTAKE_CORNER_SPEED_MARGIN_MPS)),
            "overtake_commit_sec": ("OVERTAKE_COMMIT_SEC", float(states.OVERTAKE_COMMIT_SEC)),
            "overtake_closing_margin_m": ("OVERTAKE_CLOSING_MARGIN_M", float(states.OVERTAKE_CLOSING_MARGIN_M)),
            "overtake_ttc_sec": ("OVERTAKE_TTC_SEC", float(states.OVERTAKE_TTC_SEC)),
            "overtake_passed_clearance_m": ("OVERTAKE_PASSED_CLEARANCE_M", float(states.OVERTAKE_PASSED_CLEARANCE_M)),
            "overtake_passed_clearance_time_sec": ("OVERTAKE_PASSED_CLEARANCE_TIME_SEC", float(states.OVERTAKE_PASSED_CLEARANCE_TIME_SEC)),
            "vehicle_length": ("VEHICLE_LENGTH", float(states.VEHICLE_LENGTH)),
            "vehicle_v_max": ("VEHICLE_V_MAX", float(states.VEHICLE_V_MAX)),
        }

        def declatre_parameters():
            cfg_mpc = self._cfg.mpc
            self.declare_parameter("v_max", cfg_mpc.v_max)
            self.declare_parameter("steering_tire_angle_gain_var", cfg_mpc.steering_tire_angle_gain_var)

            mpc_cfg = self._mpc_cfg
            self.declare_parameter("ay_max", mpc_cfg.ay_max)
            self.declare_parameter("accel_low_pass_gain", mpc_cfg.accel_low_pass_gain)
            self.declare_parameter("steer_low_pass_gain", mpc_cfg.steer_low_pass_gain)

            # states.py 定数の ROS パラメータ宣言
            for param_name, (_, default_val) in STATE_PARAM_MAP.items():
                self.declare_parameter(param_name, default_val)

        def param_cb(parameters):
            mpc_cfg = self._mpc_cfg

            for param in parameters:
                if param.name == "v_max" and param.type_ == Parameter.Type.DOUBLE:
                    # NOTE: mpc_cfg.v_max は m/s だが、ここは km/h をそのまま代入している
                    #       (既存挙動。_control の ref_vel キャップ側と単位が食い違う)
                    mpc_cfg.v_max = param.value
                    v_ref: List[float] = [kmh_to_m_per_sec(param.value)] * len(self._reference_path.waypoints)
                    self._reference_path.set_v_ref(v_ref)

                    self.get_logger().warn(f"v_max was updated to '{param.value}' [km/h]")

                elif param.name == "steering_tire_angle_gain_var" and param.type_ == Parameter.Type.DOUBLE:
                    mpc_cfg.steering_tire_angle_gain_var = param.value
                    self.get_logger().warn(f"steering_tire_angle_gain_var was updated to '{param.value}'")

                elif param.name == "ay_max" and param.type_ == Parameter.Type.DOUBLE:
                    mpc_cfg.ay_max = param.value
                    self.get_logger().warn(f"ay_max was updated to '{param.value}'")

                elif param.name == "accel_low_pass_gain" and param.type_ == Parameter.Type.DOUBLE:
                    mpc_cfg.accel_low_pass_gain = param.value
                    self.get_logger().warn(f"accel_low_pass_gain was updated to '{param.value}'")

                elif param.name == "steer_low_pass_gain" and param.type_ == Parameter.Type.DOUBLE:
                    mpc_cfg.steer_low_pass_gain = param.value
                    self.get_logger().warn(f"steer_low_pass_gain was updated to '{param.value}'")

                elif param.name in STATE_PARAM_MAP:
                    attr_name, _ = STATE_PARAM_MAP[param.name]
                    val = float(param.value) if param.type_ in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER) else param.value
                    setattr(states, attr_name, val)
                    self.get_logger().info(f"[StateParam] Dynamic update: states.{attr_name} = {val}")

            return SetParametersResult(successful=True)

        declatre_parameters()
        self.add_on_set_parameters_callback(param_cb)

    def _initialize(self) -> None:
        def create_map() -> Map:
            return Map(self.in_pkg_share(self._cfg.map.yaml_path)) # type: ignore

        def create_ref_path(map: Map) -> ReferencePath:
            cfg_ref_path = self._cfg.reference_path # type: ignore

            is_ref_path_given = cfg_ref_path.csv_path != "" # type: ignore
            if is_ref_path_given:
                print("Using given reference path")
                wp_x, wp_y, _, _ = load_ref_path(self.in_pkg_share(self._cfg.reference_path.csv_path)) # type: ignore
                return ReferencePath(
                    map,
                    wp_x,
                    wp_y,
                    cfg_ref_path.resolution,
                    cfg_ref_path.smoothing_distance,
                    cfg_ref_path.max_width,
                    cfg_ref_path.circular)

            else:
                print("Using waypoints to create reference path")
                wp_x, wp_y = load_waypoints(self.in_pkg_share(self._cfg.waypoints.csv_path)) # type: ignore

                return ReferencePath(
                    map,
                    wp_x,
                    wp_y,
                    cfg_ref_path.resolution,
                    cfg_ref_path.smoothing_distance,
                    cfg_ref_path.max_width,
                    cfg_ref_path.circular)


        def create_car(ref_path: ReferencePath) -> BicycleModel:
            cfg_model = self._cfg.bicycle_model # type: ignore
            return BicycleModel(
                ref_path,
                cfg_model.length,
                cfg_model.width,
                1.0 / self._cfg.mpc.control_rate) # type: ignore

        def create_controller_config() -> MPCConfig:
            cfg_mpc = self._cfg.mpc # type: ignore

            return MPCConfig(
                kmh_to_m_per_sec(self.BUG_VEL if self.USE_BUG_ACC else cfg_mpc.v_max),
                cfg_mpc.a_min,
                cfg_mpc.a_max,
                cfg_mpc.ay_max,
                cfg_mpc.control_rate,
                cfg_mpc.steering_tire_angle_gain_var,
                cfg_mpc.accel_low_pass_gain,
                cfg_mpc.steer_low_pass_gain)

        def compute_speed_profile(car: BicycleModel, mpc_config: MPCConfig) -> None:
            speed_profile_constraints = {
                "a_min": mpc_config.a_min, "a_max": mpc_config.a_max,
                "v_min": 0.0, "v_max": mpc_config.v_max, "ay_max": mpc_config.ay_max}
            car.reference_path.compute_speed_profile(speed_profile_constraints)

        def create_ref_vel_configulator() -> Optional[ReferenceVelocityConfigulator]:
            if self._ref_vel_config_path is None:
                return None
            return ReferenceVelocityConfigulator(
                self, self._config_path, self._ref_vel_config_path
            )

        self._map = create_map()
        self._reference_path = create_ref_path(self._map)
        self._car = create_car(self._reference_path)
        self._mpc_cfg = create_controller_config()
        compute_speed_profile(self._car, self._mpc_cfg)

        self._ref_vel_configulator: Optional[ReferenceVelocityConfigulator] = create_ref_vel_configulator()

        self._trajectory: Optional[Trajectory] = None

        # Laps
        self._current_laps = 1
        self._last_lap_time = 0.0
        self._lap_times = [None] * (self.MAX_LAPS + 1) # +1 means include lap 0

        # condition
        self._last_condition = None
        self._last_colliding_time = None

        # --- Stuck detection ---
        self._stopped_since = None  # time when velocity first dropped near zero
        self._start_time = None
        self._last_recovery_exit_time = None

        # --- Lateral shift side (寄せ側) hysteresis ---
        self._shift_side_filter = states.LateralShiftSideFilter()

        # --- Recovery boost ---
        self._recovery_boost_off_time: Optional[float] = None  # boost を切る時刻 [s]

        # stats
        self._stats = ExecutionStats(self.get_logger(), window_size=50, record_count_threshold=1000)

        # --- State machine ---------------------------------------------------
        self._state_manager = StateManager(self)

        # V2X tracker for state context — the state machine needs it to detect
        # forward / side vehicles.
        v2x_cfg = self._cfg.v2x_obstacle_avoidance  # type: ignore
        self._v2x_tracker = V2XVehicleTracker(
            v_max_safety=float(v2x_cfg.v_max_safety),
            position_jump_threshold=float(v2x_cfg.position_jump_threshold),
            warn_callback=self.get_logger().warn,
        )

        # Precompute waypoint array and cumulative arc lengths
        self._update_waypoint_cache()

        self.get_logger().info("[StateManager] Initialised — starting in 'follow_path'")

        # save config
        if self._cfg.common.save_config:
            self._save_config()

    def _save_config(self) -> None:
        now = datetime.now().strftime("%Y%m%d_%H%M%S")
        dst_dir = self.PKG_PATH + f"log/{now}"
        os.makedirs(dst_dir, exist_ok=True)
        shutil.copy(self._config_path, os.path.join(dst_dir, "config.yaml"))

    def _setup_pub_sub(self) -> None:
        # Publishers
        if self.USE_BUG_ACC:
          self._command_pub = self.create_publisher(
            AckermannControlBoostCommand, "/boost_commander/command", 1)
          self._command_raw_pub = self.create_publisher(
            AckermannControlBoostCommand, "/boost_commander/command_raw", 1)
        else:
          self._command_pub = self.create_publisher(
            AckermannControlCommand, "/control/command/control_cmd", 1)
          self._command_raw_pub = self.create_publisher(
            AckermannControlCommand, "/control/command/control_cmd_raw", 1)
          print("use normal ackermann control command")

        # Gear command publisher (if supported in environment)
        if HAS_GEAR_MSG:
            self._gear_cmd_pub = self.create_publisher(
                GearCommand, "/control/command/gear_cmd", 1)
        else:
            self._gear_cmd_pub = None

        # Subscribers
        self._odom_sub = self.create_subscription(
            Odometry, "/localization/kinematic_state", self._odom_callback, 1)
        self._control_mode_request_sub = self.create_subscription(
            Bool, "control/control_mode_request_topic", self._control_mode_request_callback, 1)
        # simple_trajectory_generator publishes with BEST_EFFORT/KEEP_LAST(1) — match it
        # so the subscription is QoS-compatible (rclpy default is RELIABLE).
        trajectory_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._trajectory_sub = self.create_subscription(
            Trajectory, "planning/scenario_planning/trajectory", self._trajectory_callback, trajectory_qos)
        self._stop_request_sub = self.create_subscription(
            Empty, "/control/mpc/stop_request", self._stop_request_callback, 1)

        if self.use_sim_time:
            self._awsim_status_sub = self.create_subscription(
                Float32MultiArray, "/awsim/status", self._awsim_status_callback, 1)
            self._condition_sub = self.create_subscription(
                Int32, "/aichallenge/pitstop/condition", self._condition_callback, 1)

        # V2X subscriber — the state machine needs it for forward / side vehicle detection.
        self._v2x_sub = self.create_subscription(
            V2XVehiclePositionArray,
            "/v2x/vehicle_positions",
            self._v2x_callback,
            1)

        # AWSIM boostコマンド用パブリッシャ (/awsim/cmd)
        self._awsim_boost_pub = self.create_publisher(
            Float32MultiArray, "/awsim/cmd", 10
        )

    def _publish_boost(self, boost_value: float) -> None:
        """AWSIM ブーストコマンド (/awsim/cmd) を送信する."""
        msg = Float32MultiArray()
        msg.data = [float(boost_value)]
        self._awsim_boost_pub.publish(msg)
        # self.get_logger().info(f"[AWSIM Boost] Published boost command: {boost_value}")

    def _create_ackerman_control_command(self, stamp, u, acc, bug_acc_enabled):
        v_cmd = u[0]
        steer_cmd = u[1]

        ackerman_cmd = array_to_ackermann_control_command(stamp.to_msg(), [v_cmd, steer_cmd], acc)

        if not self.USE_BUG_ACC:
            return ackerman_cmd

        ackerman_boost_cmd = AckermannControlBoostCommand()
        ackerman_boost_cmd.command = ackerman_cmd
        ackerman_boost_cmd.boost_mode = bug_acc_enabled
        return ackerman_boost_cmd

    def _publish_control_command(self, stamp, u, acc, bug_acc_enabled):
        cmd = self._create_ackerman_control_command(stamp, u, acc, bug_acc_enabled)

        # compensate steering angle for the real vehicle (support both message formats)
        if isinstance(cmd, AckermannControlCommand):
            cmd.lateral.steering_tire_angle *= self._mpc_cfg.steering_tire_angle_gain_var
        elif hasattr(cmd, "command") and hasattr(cmd.command, "lateral"):
            cmd.command.lateral.steering_tire_angle *= self._mpc_cfg.steering_tire_angle_gain_var

        # publish raw control command
        if hasattr(self, "_command_raw_pub") and self._command_raw_pub is not None:
            self._command_raw_pub.publish(cmd)

        self._command_pub.publish(cmd)

    def _publish_gear_command(self, stamp, gear_val: int) -> None:
        if not HAS_GEAR_MSG or self._gear_cmd_pub is None:
            return
        gear_msg = GearCommand()
        gear_msg.stamp = stamp
        gear_msg.command = gear_val
        self._gear_cmd_pub.publish(gear_msg)


    def _odom_callback(self, msg: Odometry) -> None:
        self._odom = msg

    def _control_mode_request_callback(self, msg):
        if msg.data and not self._enable_control:
            self.get_logger().info("Control mode request received")
            self._enable_control = True

    def _v2x_callback(self, msg: V2XVehiclePositionArray) -> None:
        self._v2x_tracker.update(msg)
        active_ids = self._v2x_tracker.active_vehicle_ids()
        self.get_logger().info(f"V2X active ids: {active_ids}, ego id: {self._vehicle_id}", throttle_duration_sec=2.0)

    def _trajectory_callback(self, msg):
        self._trajectory = msg

    def _awsim_status_callback(self, msg):
        laps = int(msg.data[1])
        lap_time = msg.data[2]
        # section = int(msg.data[3])

        if self._current_laps is None:
            self._current_laps = 1 if laps == 0 else laps

        if laps > self._current_laps:
            self.get_logger().info(f'\033[32mLap {self._current_laps} completed! Lap time: {self._last_lap_time} s\033[0m')
            self._lap_times[self._current_laps] = self._last_lap_time
            self._current_laps = laps

        self._last_lap_time = lap_time

    def _condition_callback(self, msg: Int32):
        if self._last_condition is None:
            self._last_condition = msg.data

        diff_condition = msg.data - self._last_condition
        if diff_condition > 30.0:
            self._last_colliding_time = self.get_clock().now()
            self.get_logger().warning(f"Collision detected!")
        self._last_condition = msg.data

    def _stop_request_callback(self, msg: Empty) -> None:
        if self._enable_control:
            self.get_logger().warn(f"Stop request received {self._enable_control}")
            self._enable_control = False

    def _wait_until_clock_received(self) -> None:
        if self.use_sim_time:
            self.get_logger().info(f"wait until clock received...")
            rate = self.create_rate(10)
            rate.sleep()
            self.get_logger().info(f">> OK!")

    def _wait_until_message_received(self, message_getter, message_name: str, timeout: float, rate_hz: int = 30) -> None:

        t_start = self.get_clock().now()
        rate = self.create_rate(rate_hz)

        self.get_logger().info(f"wait until {message_name} received...")

        while message_getter() is None:
            now = self.get_clock().now()
            if (now - t_start).nanoseconds > timeout * 1e9:
                self.get_logger().info(f"now: {now}, t_start: {t_start}")
                raise TimeoutError(f"Timeout while waiting for {message_name} message")
            rate.sleep()

        self.get_logger().info(f">> OK!")

    def _wait_until_odom_received(self, timeout: float = 30.) -> None:
        self._wait_until_message_received(lambda: self._odom, 'odometry', timeout)

    def _wait_until_trajectory_received(self, timeout: float = 30.) -> None:
        if self._cfg.reference_path.update_by_topic:
            self._wait_until_message_received(lambda: self._trajectory, 'trajectory', timeout)

    # ------------------------------------------------------------------
    # State-machine helpers
    # ------------------------------------------------------------------

    def _compute_pure_pursuit_control(self, pose, v_current: float) -> Tuple[float, float]:
        """Compute (target_speed_mps, steer_cmd) mirroring simple_pure_pursuit.cpp algorithm with safety guard."""
        default_speed = kmh_to_m_per_sec(35.0)
        if pose is None or self._waypoint_xy is None or len(self._waypoint_xy) == 0:
            return default_speed, 0.0

        try:
            pp = getattr(self._cfg, "pure_pursuit", None)
            wheel_base = float(getattr(pp, "wheel_base", 1.087)) if pp else 1.087
            lookahead_gain = float(getattr(pp, "lookahead_gain", 0.25)) if pp else 0.25
            lookahead_min = float(getattr(pp, "lookahead_min_distance", 2.0)) if pp else 2.0
            speed_scale = float(getattr(pp, "speed_scale_factor", 1.0)) if pp else 1.0
            steer_gain = float(getattr(pp, "steering_tire_angle_gain", 1.0)) if pp else 1.0
            use_ext_v = bool(getattr(pp, "use_external_target_vel", False)) if pp else False
            ext_v = float(getattr(pp, "external_target_vel", 0.0)) if pp else 0.0

            car_xy = np.array([pose.x, pose.y], dtype=np.float64)
            diffs = self._waypoint_xy - car_xy
            dists_sq = np.einsum("ij,ij->i", diffs, diffs)
            closest_idx = int(np.argmin(dists_sq))
            closest_wp = self._reference_path.waypoints[closest_idx]

            # Sync wp_id for velocity configulator compatibility
            # (旧 self._mpc.model は self._car と同一オブジェクト)
            self._car.wp_id = closest_idx

            base_v_mps = (
                ext_v if use_ext_v
                else (float(closest_wp.v_ref) if closest_wp.v_ref is not None else default_speed)
            )
            target_longitudinal_vel = base_v_mps * speed_scale

            lookahead_distance = lookahead_gain * target_longitudinal_vel + lookahead_min
            rear_x = pose.x - (wheel_base / 2.0) * np.cos(pose.theta)
            rear_y = pose.y - (wheel_base / 2.0) * np.sin(pose.theta)

            n_wps = len(self._reference_path.waypoints)
            target_idx = closest_idx
            for i in range(n_wps):
                idx = (closest_idx + i) % n_wps
                wp = self._reference_path.waypoints[idx]
                if np.hypot(wp.x - rear_x, wp.y - rear_y) >= lookahead_distance:
                    target_idx = idx
                    break

            lookahead_wp = self._reference_path.waypoints[target_idx]
            alpha = np.arctan2(lookahead_wp.y - rear_y, lookahead_wp.x - rear_x) - pose.theta
            alpha = (alpha + np.pi) % (2 * np.pi) - np.pi

            steering_tire_angle = steer_gain * np.arctan2(
                2.0 * wheel_base * np.sin(alpha), lookahead_distance
            )
            steer_cmd = float(np.clip(steering_tire_angle, -0.55, 0.55))

            return target_longitudinal_vel, steer_cmd
        except Exception as e:
            self.get_logger().error(f"Error in pure pursuit calculation (fallback to default speed): {e}")
            return default_speed, 0.0

    def _compute_waypoint_shift_pure_pursuit_control(
        self, pose, v_current: float, ctx: StateContext, shift_side: Optional[str] = None
    ) -> Tuple[float, float]:
        """Compute (target_speed_mps, steer_cmd) using Pure Pursuit on shifted waypoints for overtaking."""
        target_speed_mps = kmh_to_m_per_sec(35.0)
        if pose is None or self._waypoint_xy is None or len(self._waypoint_xy) == 0:
            return target_speed_mps, 0.0

        try:
            pp = getattr(self._cfg, "pure_pursuit", None)
            wheel_base = float(getattr(pp, "wheel_base", 1.087)) if pp else 1.087
            lookahead_gain = float(getattr(pp, "lookahead_gain", 0.25)) if pp else 0.25
            lookahead_min = float(getattr(pp, "lookahead_min_distance", 2.0)) if pp else 2.0
            steer_gain = float(getattr(pp, "steering_tire_angle_gain", 1.0)) if pp else 1.0

            car_xy = np.array([pose.x, pose.y], dtype=np.float64)
            diffs = self._waypoint_xy - car_xy
            dists_sq = np.einsum("ij,ij->i", diffs, diffs)
            closest_idx = int(np.argmin(dists_sq))

            # Sync wp_id for velocity configulator compatibility
            # (旧 self._mpc.model は self._car と同一オブジェクト)
            self._car.wp_id = closest_idx

            # センターライン基準の目標横オフセット（空き空間の中心）を取得
            # 壁からのマージン0.5mを先行車両の左右の空き幅から引いて半分にした値を
            # 先行車両の横端からの追い越しラインとする。実際の座標はセンターラインから
            # 先行車両の車幅の半分の最大0.725m未満+先行車両との横マージン0.225mだけ
            # ずらした位置が実際の追い越し時の座標になる
            # 寄せ側の決定。shift_side が渡されたとき (FollowState) はデッドバンド +
            # dwell 済みのラッチ判定を使い、"none" ならセンターラインを走る。
            # 渡されないとき (OvertakeState) は従来どおり毎 tick の幅比較で決める。
            if shift_side is not None:
                side = shift_side
            elif abs(ctx.target_overtake_offset) <= 0.1:
                # 前方にも側方にも車がいない (_compute_v2x_overtake_corridor が 0 を返す)。
                # このガードを落とすと幅 0 のとき 0.7m のオフセットが出てしまう。
                side = "none"
            else:
                # 参照経路は traj_mincurv (最小曲率ライン) でコーナーではインにつくため、
                # 残った空き幅は外側に偏る (コーナーの約 72%)。そのまま「広い側」を選ぶと
                # 弧長の長い外側を通ることになり、同じ速度では追い越せない。
                # コーナーでは内側が最低幅を満たす限り内側 (弧長が短い側) を優先する。
                inside = "left" if ctx.path_kappa > 0.0 else "right"
                inside_w = (ctx.overtake_width_left if inside == "left"
                            else ctx.overtake_width_right)
                if (abs(ctx.path_kappa) > states.OVERTAKE_CORNER_KAPPA
                        and inside_w >= states.MIN_OVERTAKE_WIDTH_M):
                    side = inside
                else:
                    side = "left" if ctx.overtake_width_left >= ctx.overtake_width_right else "right"

            if side == "none":
                target_offset = 0.0
            elif side == "left":
                target_offset = ((ctx.overtake_width_left - 0.5) / 2.0) + 0.725 + 0.17
            else:
                target_offset = -(((ctx.overtake_width_right - 0.5) / 2.0) + 0.725 + 0.17)

            # S字カーブ生成のための Hann 窓シフト処理
            # N_SHIFT = 35 # 35pointでだいたい35m
            N_SHIFT = 7 # int(ctx.forward_vehicle_gap) (default: 8)
            wps = self._reference_path.waypoints
            n_wps = len(wps)

            lookahead_distance = lookahead_gain * target_speed_mps + lookahead_min
            rear_x = pose.x - (wheel_base / 2.0) * np.cos(pose.theta)
            rear_y = pose.y - (wheel_base / 2.0) * np.sin(pose.theta)

            target_shifted_x = rear_x
            target_shifted_y = rear_y
            found_lookahead = False

            for i in range(n_wps):
                idx = (closest_idx + i) % n_wps
                wp = wps[idx]
                psi = float(wp.psi)
                ub = float(wp.ub) if wp.ub is not None else 3.25  # 左端
                lb = float(wp.lb) if wp.lb is not None else -3.25 # 右端

                # 壁マージン0.5m＋自車半幅0.725m（計1.225m）を確保する安全クランプ
                d_offset = float(np.clip(target_offset, lb + 1.225, ub - 1.225))

                # 最初の N_SHIFT 点に Hann 窓を適用
                if i < N_SHIFT:
                    weight = float(np.sin((np.pi / 2.0) * i / N_SHIFT) ** 2)
                else:
                    weight = float(np.sin((np.pi / 2.0)))

                # センターライン Waypoint からの法線方向オフセット計算
                shift_x = wp.x + weight * d_offset * (-np.sin(psi))
                shift_y = wp.y + weight * d_offset * np.cos(psi)

                if np.hypot(shift_x - rear_x, shift_y - rear_y) >= lookahead_distance:
                    target_shifted_x = shift_x
                    target_shifted_y = shift_y
                    found_lookahead = True
                    break

            if not found_lookahead:
                # Fallback to closest shifted waypoint
                target_shifted_x = wps[closest_idx].x
                target_shifted_y = wps[closest_idx].y

            alpha = np.arctan2(target_shifted_y - rear_y, target_shifted_x - rear_x) - pose.theta
            alpha = (alpha + np.pi) % (2 * np.pi) - np.pi

            steering_tire_angle = steer_gain * np.arctan2(
                2.0 * wheel_base * np.sin(alpha), lookahead_distance
            )
            steer_cmd = float(np.clip(steering_tire_angle, -0.55, 0.55))

            return target_speed_mps, steer_cmd
        except Exception as e:
            self.get_logger().error(f"Error in waypoint-shift pure pursuit overtake: {e}")
            return target_speed_mps, 0.0

    def _update_waypoint_cache(self) -> None:
        """Precompute waypoint coordinates, cumulative arc lengths, and track length."""
        wps = self._reference_path.waypoints
        n_wps = len(wps)
        self._waypoint_xy = np.asarray([(wp.x, wp.y) for wp in wps], dtype=np.float64)
        self._waypoint_kappa = np.asarray([wp.kappa for wp in wps], dtype=np.float64)

        seg_lens = [
            float(np.hypot(wps[(i + 1) % n_wps].x - wps[i].x, wps[(i + 1) % n_wps].y - wps[i].y))
            for i in range(n_wps)
        ]
        self._waypoint_s = np.zeros(n_wps, dtype=np.float64)
        cum_s = 0.0
        for i in range(1, n_wps):
            cum_s += seg_lens[i - 1]
            self._waypoint_s[i] = cum_s
        self._track_length = float(cum_s + seg_lens[-1])
        # Mean waypoint spacing, used to convert a lookahead distance into a waypoint count
        self._wp_spacing = max(self._track_length / max(n_wps, 1), 1e-3)

    def _path_s_of(self, x: float, y: float) -> Tuple[float, int]:
        """Arc-length position along the reference path [m], interpolated within the segment.

        Returns (s, closest_waypoint_idx).

        Snapping to the nearest waypoint alone quantises the result to the ~1 m waypoint
        spacing. The following distance is a difference of two such values, so it comes out
        as a ~1 m staircase; a proportional speed controller on that staircase produces
        ~0.8 m/s command steps, and since `acc = KP*(u[0]-v)` saturates at a 0.03 m/s error
        every step became a full-throttle or full-brake burst.
        """
        wxy = self._waypoint_xy
        n = len(wxy)
        p = np.array([x, y], dtype=np.float64)
        diffs = wxy - p
        i = int(np.argmin(np.einsum("ij,ij->i", diffs, diffs)))

        best_s = float(self._waypoint_s[i])
        best_d2 = float(diffs[i] @ diffs[i])
        for j in (i - 1, i):  # the two segments touching waypoint i
            a = wxy[j % n]
            ab = wxy[(j + 1) % n] - a
            L2 = float(ab @ ab)
            if L2 <= 1e-9:
                continue
            t = float(np.clip((p - a) @ ab / L2, 0.0, 1.0))
            d = p - (a + t * ab)
            d2 = float(d @ d)
            if d2 < best_d2:
                best_d2 = d2
                best_s = float(self._waypoint_s[j % n]) + t * float(np.sqrt(L2))
        return best_s, i

    def _compute_path_distance(self, ego_s: float, target_pos: Tuple[float, float]) -> Tuple[float, int]:
        """Compute longitudinal distance along reference path [m] between ego and target.

        Returns
        -------
        (s_rel, closest_target_idx)
        where s_rel > 0 means target is ahead of ego along the path, and s_rel < 0 means behind.
        """
        if self._waypoint_xy is None or len(self._waypoint_xy) == 0:
            return 0.0, 0

        target_s, target_idx = self._path_s_of(target_pos[0], target_pos[1])

        d_fwd = (target_s - ego_s) % self._track_length
        if d_fwd <= self._track_length / 2.0:
            s_rel = d_fwd
        else:
            s_rel = d_fwd - self._track_length
        return float(s_rel), target_idx

    def _detect_forward_and_side_vehicles(
        self
    ) -> Tuple[Optional[float], Optional[float], Optional[float], Optional[Tuple[float, float]], bool, Optional[float], Optional[float]]:
        """Return (fwd_dist, fwd_speed, fwd_heading, fwd_pos, has_side_vehicle, side_speed,
        nearest_s_rel)."""
        # Forward criteria must match _scan_surrounding_vehicles exactly — see the
        # FORWARD_CONE_DEG comment in states.py for why.
        HALF_ANGLE = np.deg2rad(states.FORWARD_CONE_DEG)

        pose = odom_to_pose_2d(self._odom)  # type: ignore
        best_dist: Optional[float] = None
        best_speed: Optional[float] = None
        best_heading: Optional[float] = None
        best_pos: Optional[Tuple[float, float]] = None

        has_side_vehicle = False
        side_vehicle_speed: Optional[float] = None
        side_vehicle_pos: Optional[Tuple[float, float]] = None

        # Nearest vehicle in our lane corridor, sign preserved (negative = behind).
        nearest_s_rel: Optional[float] = None

        cos_t = np.cos(pose.theta)
        sin_t = np.sin(pose.theta)

        # Ego arc-length position (interpolated, not snapped to a waypoint)
        ego_s, _ = self._path_s_of(pose.x, pose.y)

        for vid in self._v2x_tracker.active_vehicle_ids():
            if vid == self._vehicle_id:
                continue  # Exclude self vehicle

            buf = self._v2x_tracker._samples.get(vid)
            if not buf:
                continue
            _, vx_pos, vy_pos = buf[-1]
            dx = vx_pos - pose.x
            dy = vy_pos - pose.y

            # Transform to ego-vehicle local coordinates
            x_rel = cos_t * dx + sin_t * dy
            y_rel = -sin_t * dx + cos_t * dy

            vx_vel, vy_vel = self._v2x_tracker.velocity(vid)
            v_speed = float(np.hypot(vx_vel, vy_vel))

            # 1. Side-by-side vehicle detection (-2.5m <= x_rel <= 2.5m, alongside on track)
            if -2.5 <= x_rel <= 2.5 and 0.6 <= abs(y_rel) <= states.FORWARD_LATERAL_MAX:
                has_side_vehicle = True
                side_vehicle_speed = v_speed
                side_vehicle_pos = (vx_pos, vy_pos)

            # 2. Forward vehicle detection (longitudinal path distance ahead along reference path)
            s_rel, _ = self._compute_path_distance(ego_s, (vx_pos, vy_pos))

            # Nearest vehicle in our lane corridor, keeping the SIGN (negative = behind).
            # The forward filter below drops s_rel <= 0 and applies a +-45 deg cone, so a
            # vehicle we have just passed becomes invisible to every other field in
            # StateContext. OvertakeState needs this to tell "the leader is now behind me"
            # from "the tracker lost it". No angle gate here: a vehicle directly behind
            # sits at ~180 deg.
            if abs(s_rel) <= states.FORWARD_VEHICLE_DETECTION and abs(y_rel) <= states.FORWARD_LATERAL_MAX:
                if nearest_s_rel is None or abs(s_rel) < abs(nearest_s_rel):
                    nearest_s_rel = float(s_rel)

            if s_rel <= 0.0 or s_rel > states.FORWARD_VEHICLE_DETECTION:
                continue

            if abs(y_rel) > states.FORWARD_LATERAL_MAX:
                continue

            angle = np.arctan2(dy, dx) - pose.theta
            angle = (angle + np.pi) % (2 * np.pi) - np.pi
            if abs(angle) > HALF_ANGLE:
                continue

            if best_dist is None or s_rel < best_dist:
                best_dist = s_rel
                best_pos = (vx_pos, vy_pos)
                best_speed = v_speed
                if best_speed > 0.3:
                    best_heading = float(np.arctan2(vy_vel, vx_vel))
                elif len(buf) >= 2:
                    dx_v = buf[-1][1] - buf[0][1]
                    dy_v = buf[-1][2] - buf[0][2]
                    if np.hypot(dx_v, dy_v) > 0.05:
                        best_heading = float(np.arctan2(dy_v, dx_v))

        # If no vehicle directly ahead but a vehicle is alongside, use side vehicle pos for corridor calc
        target_corridor_pos = best_pos if best_pos is not None else side_vehicle_pos
        return (best_dist, best_speed, best_heading, target_corridor_pos,
                has_side_vehicle, side_vehicle_speed, nearest_s_rel)

    def _compute_v2x_overtake_corridor(
        self, fwd_pos: Optional[Tuple[float, float]]
    ) -> Tuple[float, float, float]:
        """Compute available road widths (left, right) and optimal target overtake offset using ReferencePath.

        Returns
        -------
        (overtake_width_left, overtake_width_right, target_overtake_offset)
        """
        if fwd_pos is None or self._waypoint_xy is None or len(self._waypoint_xy) == 0:
            return 0.0, 0.0, 0.0

        vx, vy = fwd_pos
        fwd_xy = np.array([vx, vy], dtype=np.float64)
        diffs = self._waypoint_xy - fwd_xy
        dists_sq = np.einsum("ij,ij->i", diffs, diffs)
        closest_idx = int(np.argmin(dists_sq))
        wp = self._reference_path.waypoints[closest_idx]

        # Waypoint bounds (left is positive, right is negative)
        ub = float(wp.ub) if wp.ub is not None else 3.0  # 3.0
        lb = float(wp.lb) if wp.lb is not None else -3.0 # 3.0

        # Leader lateral position e_y relative to waypoint centerline
        path_psi = float(wp.psi)
        dx = vx - wp.x
        dy = vy - wp.y
        e_y_leader = float(-dx * np.sin(path_psi) + dy * np.cos(path_psi))

        # Margins: vehicle half-width + clearance
        W_MARGIN = 0.8   # vehicle half-width (0.725m) + safety margin
        W_WALL = 0.5     # course wall buffer

        # Available widths on left and right
        left_edge = ub - W_WALL
        left_obstacle_inner = e_y_leader + W_MARGIN
        avail_left = max(0.0, left_edge - left_obstacle_inner)

        right_edge = lb + W_WALL
        right_obstacle_inner = e_y_leader - W_MARGIN
        avail_right = max(0.0, right_obstacle_inner - right_edge)

        # Target centerlines for the corridors
        offset_left = (left_obstacle_inner + left_edge) / 2.0
        offset_right = (right_edge + right_obstacle_inner) / 2.0

        if avail_left >= avail_right:
            target_offset = float(np.clip(offset_left, 0.8, 2.5))
        else:
            target_offset = float(np.clip(offset_right, -2.5, -0.8))

        return float(avail_left), float(avail_right), float(target_offset)

    def _scan_surrounding_vehicles(
        self,
        pose,
        v_ego: float,
        fwd_detect_distance: float,
        fwd_angle_min_deg: float,
        fwd_angle_max_deg: float,
    ):
        """Perform comprehensive scan of surrounding vehicles for FollowPath transition logic."""
        cos_t = np.cos(pose.theta)
        sin_t = np.sin(pose.theta)

        v_ego_x = v_ego * cos_t
        v_ego_y = v_ego * sin_t

        # Ego arc-length position (interpolated, not snapped to a waypoint)
        ego_s, _ = self._path_s_of(pose.x, pose.y)

        fwd_vehicles = []         # list of (s_rel, x_rel, y_rel, speed, pos, max_width)
        left_side_vehicles = []   # list of (x_rel, y_rel, is_cutin)
        right_side_vehicles = []  # list of (x_rel, y_rel, is_cutin)

        for vid in self._v2x_tracker.active_vehicle_ids():
            if vid == self._vehicle_id:
                continue

            buf = self._v2x_tracker._samples.get(vid)
            if not buf:
                continue
            _, vx_pos, vy_pos = buf[-1]
            dx = vx_pos - pose.x
            dy = vy_pos - pose.y

            # Ego local coordinates
            x_rel = cos_t * dx + sin_t * dy
            y_rel = -sin_t * dx + cos_t * dy

            # Angle relative to ego heading in degrees [-180, 180]
            angle_rad = np.arctan2(y_rel, x_rel)
            angle_deg = float(np.rad2deg(angle_rad))

            vx_vel, vy_vel = self._v2x_tracker.velocity(vid)
            v_speed = float(np.hypot(vx_vel, vy_vel))

            # Relative velocity in ego local frame
            dv_x_world = vx_vel - v_ego_x
            dv_y_world = vy_vel - v_ego_y
            dv_x_rel = cos_t * dv_x_world + sin_t * dv_y_world
            dv_y_rel = -sin_t * dv_x_world + cos_t * dv_y_world

            # 3-second projected relative position
            x_rel_3s = x_rel + 1.0 * dv_x_rel
            y_rel_3s = y_rel + 1.0 * dv_y_rel
            angle_3s_deg = float(np.rad2deg(np.arctan2(y_rel_3s, x_rel_3s)))

            # Path distance along reference path
            s_rel, _ = self._compute_path_distance(ego_s, (vx_pos, vy_pos))

            # 1. Forward vehicle. The cone, the lateral gate and the distance metric must
            #    match _detect_forward_and_side_vehicles exactly, otherwise follow_path and
            #    follow disagree about whether a leader exists and the state machine
            #    oscillates — flipping the controller between pure pursuit and MPC each time.
            if (fwd_angle_min_deg <= angle_deg <= fwd_angle_max_deg
                    and 0.0 < s_rel <= fwd_detect_distance
                    and x_rel > 0.0
                    and abs(y_rel) <= states.FORWARD_LATERAL_MAX):
                    # and abs(y_rel) <= 7.0):
                left_w, right_w, _ = self._compute_v2x_overtake_corridor((vx_pos, vy_pos))
                max_avail_w = max(left_w, right_w)
                fwd_vehicles.append((s_rel, x_rel, y_rel, v_speed, (vx_pos, vy_pos), max_avail_w))

            # 2. Left side vehicle: angle in [+30°, +150°] and 0.0 <= y_rel <= 3.0
            if fwd_angle_max_deg <= angle_deg <= (180.0 - fwd_angle_max_deg) and 0.0 <= y_rel <= 4.0:
                is_cutin = (45.0 <= angle_3s_deg <= 100.0 and 0.0 <= y_rel_3s <= 4.0)
                left_side_vehicles.append((x_rel, y_rel, is_cutin))

            # 3. Right side vehicle: angle in [-150°, -30°] and -3.0 <= y_rel <= 0.0
            if (-180.0 - fwd_angle_min_deg) <= angle_deg <= fwd_angle_min_deg and -4.0 <= y_rel <= 0.0:
                is_cutin = (-100.0 <= angle_3s_deg <= -45.0 and -4.0 <= y_rel_3s <= 0.0)
                right_side_vehicles.append((x_rel, y_rel, is_cutin))

        has_forward_vehicle = len(fwd_vehicles) > 0
        min_fwd_width = min([v[5] for v in fwd_vehicles]) if fwd_vehicles else 0.0
        fwd_vehicles.sort(key=lambda x: x[0])
        closest_fwd_speed = fwd_vehicles[0][3] if fwd_vehicles else None

        has_left_side = len(left_side_vehicles) > 0
        has_left_cutin = any(v[2] for v in left_side_vehicles)

        has_right_side = len(right_side_vehicles) > 0
        has_right_cutin = any(v[2] for v in right_side_vehicles)

        return (
            has_forward_vehicle,
            min_fwd_width,
            closest_fwd_speed,
            has_left_side,
            has_left_cutin,
            has_right_side,
            has_right_cutin,
        )

    def _build_state_context(self, dt: float, is_colliding: bool) -> StateContext:
        """Assemble a StateContext snapshot for the current tick."""
        now_sec = self.get_clock().now().nanoseconds / 1e9
        pose = odom_to_pose_2d(self._odom)  # type: ignore
        v = self._odom.twist.twist.linear.x  # type: ignore

        # --- Stuck detection: track how long velocity has been near zero ---
        STOPPED_THRESHOLD = 0.3  # [m/s]
        if abs(v) < STOPPED_THRESHOLD:
            if self._stopped_since is None:
                self._stopped_since = now_sec
            time_stopped = now_sec - self._stopped_since
        else:
            self._stopped_since = None
            time_stopped = 0.0

        (
            fwd_dist,
            fwd_speed,
            fwd_heading,
            corridor_pos,
            has_side,
            side_speed,
            nearest_s_rel,
        ) = self._detect_forward_and_side_vehicles()
        left_w, right_w, target_offset = self._compute_v2x_overtake_corridor(corridor_pos)

        (
            has_fwd,
            min_fwd_w,
            closest_fwd_spd,
            has_l_side,
            has_l_cutin,
            has_r_side,
            has_r_cutin,
        ) = self._scan_surrounding_vehicles(
            pose,
            v,
            fwd_detect_distance=states.FORWARD_VEHICLE_DETECTION,
            fwd_angle_min_deg=-states.FORWARD_CONE_DEG,
            fwd_angle_max_deg=states.FORWARD_CONE_DEG,
        )

        if self._start_time is None:
            self._start_time = now_sec

        # Cooldown check: 10.0s after startup or 5.0s after exiting RecoveryState
        is_cooldown = False
        if (now_sec - self._start_time) < 2.0: # 10.0
            is_cooldown = True
        if self._last_recovery_exit_time is not None:
            if (now_sec - self._last_recovery_exit_time) < 2.0: # 5.0
                is_cooldown = True

        # Closest waypoint orientation & signed lateral distance
        car_xy = np.array([pose.x, pose.y], dtype=np.float64)
        diffs = self._waypoint_xy - car_xy
        dists_sq = np.einsum("ij,ij->i", diffs, diffs)
        closest_idx = int(np.argmin(dists_sq))
        closest_wp = self._reference_path.waypoints[closest_idx]

        path_psi = float(closest_wp.psi)
        dx = pose.x - closest_wp.x
        dy = pose.y - closest_wp.y
        path_e_y = float(-dx * np.sin(path_psi) + dy * np.cos(path_psi))

        # 追い越しに必要な距離だけ先読みし、その区間で最も曲率が大きい点の
        # 符号付き曲率を取る。コーナーの強さと向き (正 = 左) を 1 値で表す。
        n_look = max(1, int(states.OVERTAKE_CORNER_LOOKAHEAD_M / self._wp_spacing))
        look_idxs = (closest_idx + np.arange(n_look)) % len(self._waypoint_kappa)
        kappa_window = self._waypoint_kappa[look_idxs]
        path_kappa = float(kappa_window[np.argmax(np.abs(kappa_window))])

        fwd_heading_diff = 0.0
        if fwd_heading is not None:
            diff = (fwd_heading - path_psi + np.pi) % (2 * np.pi) - np.pi
            fwd_heading_diff = float(diff)

        return StateContext(
            current_time_sec=now_sec,
            dt=dt,
            pose_x=pose.x,
            pose_y=pose.y,
            pose_theta=pose.theta,
            velocity=v,
            is_colliding=is_colliding,
            path_psi=path_psi,
            path_e_y=path_e_y,
            path_kappa=path_kappa,
            forward_vehicle_distance=fwd_dist,
            forward_vehicle_speed=fwd_speed,
            forward_vehicle_heading_diff=fwd_heading_diff,
            nearest_vehicle_s_rel=nearest_s_rel,
            overtake_width_left=left_w,
            overtake_width_right=right_w,
            target_overtake_offset=target_offset,
            lateral_shift_side=self._shift_side_filter.update(left_w, right_w, now_sec),
            has_side_vehicle=has_side,
            side_vehicle_speed=side_speed,
            has_forward_vehicle=has_fwd,
            min_forward_overtake_width=min_fwd_w,
            closest_forward_vehicle_speed=closest_fwd_spd,
            has_left_side_vehicle=has_l_side,
            has_left_side_cutin_hazard=has_l_cutin,
            has_right_side_vehicle=has_r_side,
            has_right_side_cutin_hazard=has_r_cutin,
            time_stopped_sec=time_stopped,
            is_in_recovery_cooldown=is_cooldown,
            # boost使用時に以下をコメントアウト。
            publish_boost=self._publish_boost,
            log_event=self.get_logger().info,
        )

    def _apply_state_params(self, params: MPCStateParams) -> None:
        """Push state-specific parameters into the speed cap used by Pure Pursuit.

        params.Q / R / QN / lateral_offset は MPC ソルバ専用だったため、
        MPC 撤去に伴い読み捨てている (states.py 側の整理待ち)。
        """
        self._mpc_cfg.v_max = kmh_to_m_per_sec(params.v_max)
        self._mpc_cfg.ay_max = params.ay_max

    # ------------------------------------------------------------------
    # Main control loop
    # ------------------------------------------------------------------

    def _control(self):
        if self._odom is None or self._waypoint_xy is None or len(self._waypoint_xy) == 0:
            return

        try:
            self._loop += 1

            # record and print execution stats
            if self.use_stats:
                self._stats.record()

            # self.get_logger().info("loop")
            self._control_rate.sleep()

            # 時刻は sleep の後に取る。先に取ると now が 1 制御周期 (25ms) 古くなり、
            # 衝突ラッチの判定と publish するコマンドのヘッダスタンプがその分ずれる。
            # dt は前後どちらで測っても 1 周期のままで変わらない。
            now = self.get_clock().now()
            t = (now - self._t_start).nanoseconds / 1e9
            dt = (now - self._last_t).nanoseconds / 1e9
            self._last_t = now

            if self._loop % 100 == 0:
                # update reference path
                if self._cfg.reference_path.update_by_topic: # type: ignore
                    new_referece_path = self._create_reference_path_from_autoware_trajectory(self._trajectory)
                    if new_referece_path is not None:
                        self._car.reference_path = new_referece_path
                        self._car.update_reference_path(self._car.reference_path)

            is_colliding = False
            if self._last_colliding_time is not None:
                elapsed_from_last_colliding = (now - self._last_colliding_time).nanoseconds / 1e9
                if elapsed_from_last_colliding < 2.5: # 2.5
                    is_colliding = True

            pose = odom_to_pose_2d(self._odom) # type: ignore
            v = self._odom.twist.twist.linear.x

            # ---- State-machine tick -------------------------------------------
            prev_state_name = self._state_manager.current_state_name
            ctx = self._build_state_context(dt, is_colliding)
            new_params = self._state_manager.update(ctx)
            if new_params is not None:
                self._apply_state_params(new_params)

            current_state = self._state_manager.current_state

            if prev_state_name == "recovery" and self._state_manager.current_state_name != "recovery":
                self._last_recovery_exit_time = (now.nanoseconds / 1e9)
                # 衝突ラッチ (2.5s) が残っていると follow_path / follow / overtake が
                # 即座に "recovery" を返し、MIN_DWELL_TIME の recovery 例外により
                # 無条件で再突入してループする。復帰後に本当に再衝突すれば
                # condition トピックが再びラッチを立てるので検知能力は落ちない。
                self._last_colliding_time = None
                # Instantly reset last control memory to forward motion for zero-lag launch
                self._last_u[0] = 1.5
                self._last_acc = 1.0
                # boost の ON は RecoveryState.on_exit が publish 済み。ここでは切る時刻だけ持つ。
                self._recovery_boost_off_time = (
                    ctx.current_time_sec + states.RECOVERY_BOOST_DURATION_SEC)
                self.get_logger().info("Exited RecoveryState: instant launch & boost on")

            # 復帰 boost を時間で切る。OvertakeState が boost を使っている間は
            # その on_exit が 0.0 を出すので、ここで上書きしない。
            if (self._recovery_boost_off_time is not None
                    and ctx.current_time_sec >= self._recovery_boost_off_time):
                if not isinstance(current_state, states.OvertakeState):
                    self._publish_boost(0.0)
                self._recovery_boost_off_time = None

            # 横オフセットによる車両状態のシフトは MPC の参照点をずらすためのものだった。
            # Waypoint-shift Pure Pursuit は ctx.target_overtake_offset から独自に
            # オフセットを計算するため、ここでは実測姿勢をそのまま反映する。
            self._car.update_states(pose.x, pose.y, pose.theta)

            # Follow state: dynamically adjust v_max to match leader speed
            follow_target_speed_mps: Optional[float] = None
            if isinstance(current_state, FollowState):
                if ctx.forward_vehicle_speed is not None:
                    follow_target_speed_mps = current_state.get_adjusted_v_max_mps(ctx)
                    v_ref_list: List[float] = [follow_target_speed_mps] * len(self._reference_path.waypoints)
                    self._reference_path.set_v_ref(v_ref_list)

            # Check for control override (e.g., Recovery wait/back)
            override = self._state_manager.get_control_override(ctx)
            if override is not None:
                override_speed, override_steer, override_acc = override
                u = [override_speed, override_steer]
                acc = override_acc

                # If reversing (negative speed), bypass low-pass filter and set positive acceleration magnitude
                if override_speed < 0:
                    acc = abs(override_acc)
                    self._last_acc = acc
                    self._last_u[0] = override_speed
                    self._last_u[1] = override_steer
                else:
                    acc = self._last_acc + (acc - self._last_acc) * self._mpc_cfg.accel_low_pass_gain
                    u[1] = self._last_u[1] + (u[1] - self._last_u[1]) * self._mpc_cfg.steer_low_pass_gain
                    self._last_acc = acc
                    self._last_u[0] = u[0]
                    self._last_u[1] = u[1]

                self._car.drive([v, u[1]])
                self._publish_control_command(now, u, acc, False)
                self._publish_gear_command(now.to_msg(), self._state_manager.current_gear)
                self._sim_logger.log(self._car, u, t)
                # NOTE: 第7引数は旧 MPC インスタンス。sim_logger 側は
                #       animation_enabled が True のときしか参照しない。
                self._sim_logger.plot_animation(t, self._loop, self._current_laps, self._lap_times, is_colliding, u, None, self._car)
                return

            # ---- Control Selection (Waypoint-Shift Pure Pursuit for Follow/Overtake, Pure Pursuit otherwise) ----
            if current_state.control_mode == ControlMode.WAYPOINT_SHIFT_PURE_PURSUIT:
                # FollowState だけデッドバンド + dwell 付きの寄せ側を渡す。
                # Overtake は幅の差が大きい場面に限られるため従来どおり毎 tick 判定。
                shift_side = ctx.lateral_shift_side if isinstance(current_state, FollowState) else None
                v_target, steer_target = self._compute_waypoint_shift_pure_pursuit_control(
                    pose, v, ctx, shift_side)
            else:
                v_target, steer_target = self._compute_pure_pursuit_control(pose, v)

            # FollowState: 車間 PD の出力を縦方向指令に反映する。
            # _compute_waypoint_shift_pure_pursuit_control は wp.v_ref を読まず
            # 35 km/h 固定を返すため、ここで上書きしないと車間制御が効かない。
            # ステア (lookahead は 35 km/h ベース) は既存チューニングを崩さないよう触らない。
            if follow_target_speed_mps is not None:
                v_target = follow_target_speed_mps
            elif isinstance(current_state, states.OvertakeState):
                # OvertakeState: 車両上限を超える目標速度を出して常時フルスロットルにする。
                # 35 km/h 固定は AWSIM の drive-fade 平衡 (約 35.7 km/h) を下回るため、
                # 追い越しに入った瞬間 acc = KP*(u[0]-v) が a_min (フルブレーキ) になっていた。
                v_target = kmh_to_m_per_sec(states.OVERTAKE_TARGET_SPEED_KMH)

            u = [v_target, steer_target]
            max_delta = steer_target

            # FollowState uses dynamic following speed; skip static ref_vel overwrite
            if self._ref_vel_configulator is not None and not isinstance(current_state, FollowState):
                # NOTE: get_ref_vel() は ref_vel.yaml の値 (km/h) をそのまま返すが
                #       self._mpc_cfg.v_max は m/s。既存挙動を保つため単位はそのまま。
                ref_vel_mps = self._ref_vel_configulator.get_ref_vel(self._car.wp_id)
                ref_vel_mps_capped = min(ref_vel_mps, self._mpc_cfg.v_max)
                v_ref: List[float] = [ref_vel_mps_capped] * len(self._reference_path.waypoints)
                self._reference_path.set_v_ref(v_ref)

            # override by brake command if control is disabled
            if not self._enable_control:
                last_v_cmd = self._last_u[0]
                if last_v_cmd < 0.5:
                    u[0] = 0.0
                else:
                    decel_v = last_v_cmd + self._mpc_cfg.a_min * dt
                    u[0] = np.clip(decel_v, 0.0, self._mpc_cfg.v_max)

            if len(u) == 0:
                self.get_logger().error("No control signal", throttle_duration_sec=1)
                u = [0.0, 0.0]

            acc = 0.
            bug_acc_enabled = False
            if self.USE_BUG_ACC:
                def deg2rad(deg):
                    return deg * np.pi / 180.0

                if abs(v) > kmh_to_m_per_sec(44.0) or \
                 (abs(v) > kmh_to_m_per_sec(38.0) and abs(max_delta) > deg2rad(12.0)):
                    bug_acc_enabled = False
                    acc = self._mpc_cfg.a_min / 3.0 * 2.0
                elif abs(v) > kmh_to_m_per_sec(41.0) or abs(u[1]) > deg2rad(10.0):
                    bug_acc_enabled = False
                    acc = self._mpc_cfg.a_max
                else:
                    bug_acc_enabled = True
                    acc = 500.0
            else:
                acc = self.KP * (u[0] - v)
                # print(f"v: {v}, u[0]: {u[0]}, acc: {acc}")
                acc = np.clip(acc, self._mpc_cfg.a_min, self._mpc_cfg.a_max)

            # NOTE: ここには FollowState 用の加速度制限 + PD ステア補正があったが、
            #       `control_mode == ControlMode.MPC` で囲まれており、FollowState は
            #       WAYPOINT_SHIFT_PURE_PURSUIT を返すため一度も実行されていなかった。
            #       Waypoint-shift Pure Pursuit で有効化したい場合は git 履歴から復元し、
            #       ゲートを外したうえで走行性能を再計測すること。

            self._last_acc = acc
            self._last_u[0] = u[0]
            self._last_u[1] = u[1]

            # update car state (use v for feedback actual speed)
            self._car.drive([v, u[1]])

            # Publish control command & gear command
            self._publish_control_command(now, u, acc, bug_acc_enabled)
            self._publish_gear_command(now.to_msg(), self._state_manager.current_gear)

            # Log states
            self._sim_logger.log(self._car, u, t)
            self._sim_logger.plot_animation(t, self._loop, self._current_laps, self._lap_times, is_colliding, u, None, self._car)
        except Exception as e:
            self.get_logger().error(f"Error in _control loop (continuing execution): {e}")

    def run(self) -> None:
        try:
            self._wait_until_clock_received()
            self._wait_until_odom_received()
            self._wait_until_trajectory_received(timeout=5.0)
        except Exception as e:
            self.get_logger().warn(f"Topic wait warning (continuing node execution): {e}")

        # wait until odom is received to avoid NoneType AttributeError
        while rclpy.ok() and self._odom is None:
            self.get_logger().info("Waiting for odometry message before starting control loop...")
            self.get_clock().sleep_for(rclpy.time.Duration(seconds=0.1))

        # initialize car states
        pose = odom_to_pose_2d(self._odom) # type: ignore
        self._car.update_states(pose.x, pose.y, pose.theta)
        self._car.update_reference_path(self._car.reference_path)

        # initialize control states
        self._control_rate = self.create_rate(self._mpc_cfg.control_rate)
        self._sim_logger = SimulationLogger(
            self.get_logger(),
            self._car.temporal_state.x, self._car.temporal_state.y, self._cfg.sim_logger.animation_enabled, self.SHOW_PLOT_ANIMATION, self.PLOT_RESULTS, self.ANIMATION_INTERVAL) # type: ignore

        self._loop = 0
        self._last_acc = 0.0
        self._last_u = np.array([0.0, 0.0])
        self._t_start = self.get_clock().now()
        self._last_t = self._t_start

        self.get_logger().info("----------------------")
        self.get_logger().info("START!")
        self.get_logger().info("----------------------")

        while rclpy.ok() and (not self._sim_logger.stop_requested()):
            self._control()

    def stop(self):
        # Wait for stopping
        self.get_logger().warn("----------------------")
        self.get_logger().warn("Stopping...")
        self.get_logger().warn("----------------------")
        timeout_time = self.get_clock().now() + rclpy.time.Duration(seconds=5)
        while self._odom.twist.twist.linear.x > 0.1 and self.get_clock().now() < timeout_time:
            self._enable_control = False
            self._control()

        # Publish zero command to stop the car completely
        zero_cmd = self._create_ackerman_control_command(self.get_clock().now(), [0.0, 0.0], 0.0, False)
        self._command_pub.publish(zero_cmd)

        self.get_logger().warn(">> Stop Completed!")

        # show results
        self._sim_logger.show_results(self._current_laps, self._lap_times, self._car)

    @classmethod
    def in_pkg_share(cls, file_path: str) -> str:
        return cls.PKG_PATH + file_path
