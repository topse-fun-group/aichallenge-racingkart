#!/usr/bin/env python3

import yaml
from typing import List, Tuple, Optional, NamedTuple
import dataclasses
from scipy import sparse
from scipy.sparse import dia_matrix
import numpy as np
import copy
import os
import shutil
from datetime import datetime

# ROS 2
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.parameter import Parameter
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Empty, Bool, Float32MultiArray, Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Pose2D, Point, Vector3
from std_msgs.msg import ColorRGBA, Float32MultiArray

from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

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
from multi_purpose_mpc_ros_with_dynamic_param.v2x_vehicle_tracker import (
    V2XVehicleTracker,
    predictions_to_obstacles,
)

# Multi_Purpose_MPC
from multi_purpose_mpc_ros_with_dynamic_param.core.map import Map, Obstacle
from multi_purpose_mpc_ros_with_dynamic_param.core.reference_path import ReferencePath
from multi_purpose_mpc_ros_with_dynamic_param.core.spatial_bicycle_models import BicycleModel
from multi_purpose_mpc_ros_with_dynamic_param.core.MPC import MPC
from multi_purpose_mpc_ros_with_dynamic_param.core.utils import load_waypoints, kmh_to_m_per_sec, load_ref_path, load_ref_path_speed_profile

# Project
from multi_purpose_mpc_ros_with_dynamic_param.common import convert_to_namedtuple, file_exists
from multi_purpose_mpc_ros_with_dynamic_param.simulation_logger import SimulationLogger
from multi_purpose_mpc_ros_with_dynamic_param.obstacle_manager import ObstacleManager
from multi_purpose_mpc_ros_with_dynamic_param.exexution_stats import ExecutionStats
from multi_purpose_mpc_ros_msgs.msg import AckermannControlBoostCommand, PathConstraints, BorderCells
from multi_purpose_mpc_ros_with_dynamic_param.tools.reference_velocity_configulator import ReferenceVelocityConfigulator

# State machine
import multi_purpose_mpc_ros_with_dynamic_param.states as states
from multi_purpose_mpc_ros_with_dynamic_param.states import (
    StateContext,
    MPCStateParams,
    FollowState,
    OvertakeState,
    ControlMode,
)
from multi_purpose_mpc_ros_with_dynamic_param.state_manager import StateManager
# from multi_purpose_mpc_ros_with_dynamic_param.lidar_processor import LidarProcessor


RED = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
YELLOW = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
CYAN = ColorRGBA(r=0.0, g=156.0 / 255.0, b=209.0 / 255.0, a=1.0)

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
    N: int
    Q: dia_matrix
    R: dia_matrix
    QN: dia_matrix
    v_max: float
    a_min: float
    a_max: float
    ay_max: float
    delta_max: float
    steer_rate_max: float
    control_rate: float
    steering_tire_angle_gain_var: float
    accel_low_pass_gain: float
    steer_low_pass_gain: float
    wp_id_offset: int
    use_max_kappa_pred: bool


class MPCController(Node):

    PKG_PATH: str = get_package_share_directory('multi_purpose_mpc_ros_with_dynamic_param') + "/"
    # MAX_LAPS = 6
    MAX_LAPS = 10000
    BUG_VEL = 40.0 # km/h
    BUG_ACC = 400.0

    SHOW_PLOT_ANIMATION = False
    PLOT_RESULTS = False
    ANIMATION_INTERVAL = 20

    KP = 100.0

    def __init__(self, config_path: str, ref_vel_config_path: Optional[str]) -> None:
        super().__init__("mpc_controller") # type: ignore

        # declare parameters
        self.declare_parameter("use_boost_acceleration", False)
        self.declare_parameter("use_obstacle_avoidance", False)
        self.declare_parameter("use_stats", False)
        self.declare_parameter("vehicle_id", os.environ.get("VEHICLE_ID", "default"))

        # get parameters
        self.use_sim_time = self.get_parameter("use_sim_time").get_parameter_value().bool_value
        self.USE_BUG_ACC = self.get_parameter("use_boost_acceleration").get_parameter_value().bool_value
        self.USE_OBSTACLE_AVOIDANCE = self.get_parameter("use_obstacle_avoidance").get_parameter_value().bool_value
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
        if self.USE_OBSTACLE_AVOIDANCE:
            self.get_logger().warn("------------------------------------")
            self.get_logger().warn("USE_OBSTACLE_AVOIDANCE is enabled!")
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
            "min_overtake_width_m": ("MIN_OVERTAKE_WIDTH_M", float(states.MIN_OVERTAKE_WIDTH_M)),
            "min_overtake_lead_speed": ("MIN_OVERTAKE_LEAD_SPEED", float(states.MIN_OVERTAKE_LEAD_SPEED)),
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
            self.declare_parameter("Q0", cfg_mpc.Q[0])
            self.declare_parameter("Q1", cfg_mpc.Q[1])
            self.declare_parameter("Q2", cfg_mpc.Q[2])
            self.declare_parameter("R0", cfg_mpc.R[0])
            self.declare_parameter("R1", cfg_mpc.R[1])
            self.declare_parameter("QN0", cfg_mpc.QN[0])
            self.declare_parameter("QN1", cfg_mpc.QN[1])
            self.declare_parameter("QN2", cfg_mpc.QN[2])

            mpc_cfg = self._mpc_cfg
            self.declare_parameter("ay_max", mpc_cfg.ay_max)
            self.declare_parameter("accel_low_pass_gain", mpc_cfg.accel_low_pass_gain)
            self.declare_parameter("steer_low_pass_gain", mpc_cfg.steer_low_pass_gain)
            self.declare_parameter("wp_id_offset", mpc_cfg.wp_id_offset)

            # states.py 定数の ROS パラメータ宣言
            for param_name, (_, default_val) in STATE_PARAM_MAP.items():
                self.declare_parameter(param_name, default_val)

        def param_cb(parameters):
            cfg_mpc = self._cfg.mpc # type: ignore
            mpc_cfg = self._mpc_cfg

            def update_Q(index: int, value: float):
                cfg_mpc.Q[index] = value
                mpc_cfg.Q = sparse.diags(cfg_mpc.Q)
                self._mpc.update_Q(mpc_cfg.Q)
                self.get_logger().warn(f"Q[{index}] was updated to '{value}'")

            def update_R(index: int, value: float):
                cfg_mpc.R[index] = value
                mpc_cfg.R = sparse.diags(cfg_mpc.R)
                self._mpc.update_R(mpc_cfg.R)
                self.get_logger().warn(f"R[{index}] was updated to '{value}'")

            def update_QN(index: int, value: float):
                cfg_mpc.QN[index] = value
                mpc_cfg.QN = sparse.diags(cfg_mpc.QN)
                self._mpc.update_QN(mpc_cfg.QN)
                self.get_logger().warn(f"QN[{index}] was updated to '{value}'")

            for param in parameters:
                if param.name == "v_max" and param.type_ == Parameter.Type.DOUBLE:
                    mpc_cfg.v_max = param.value
                    self._mpc.update_v_max(kmh_to_m_per_sec(param.value))
                    v_ref: List[float] = [kmh_to_m_per_sec(param.value)] * len(self._reference_path.waypoints)
                    self._reference_path.set_v_ref(v_ref)

                    self.get_logger().warn(f"v_max was updated to '{param.value}' [km/h]")

                elif param.name == "steering_tire_angle_gain_var" and param.type_ == Parameter.Type.DOUBLE:
                    mpc_cfg.steering_tire_angle_gain_var = param.value
                    self.get_logger().warn(f"steering_tire_angle_gain_var was updated to '{param.value}'")

                elif param.name == "Q0" and param.type_ == Parameter.Type.DOUBLE:
                    update_Q(0, param.value)
                elif param.name == "Q1" and param.type_ == Parameter.Type.DOUBLE:
                    update_Q(1, param.value)
                elif param.name == "Q2" and param.type_ == Parameter.Type.DOUBLE:
                    update_Q(2, param.value)


                elif param.name == "R0" and param.type_ == Parameter.Type.DOUBLE:
                    update_R(0, param.value)
                elif param.name == "R1" and param.type_ == Parameter.Type.DOUBLE:
                    update_R(1, param.value)

                elif param.name == "QN0" and param.type_ == Parameter.Type.DOUBLE:
                    update_QN(0, param.value)
                elif param.name == "QN1" and param.type_ == Parameter.Type.DOUBLE:
                    update_QN(1, param.value)
                elif param.name == "QN2" and param.type_ == Parameter.Type.DOUBLE:
                    update_QN(2, param.value)

                elif param.name == "ay_max" and param.type_ == Parameter.Type.DOUBLE:
                    mpc_cfg.ay_max = param.value
                    self._mpc.update_ay_max(param.value)
                    self.get_logger().warn(f"ay_max was updated to '{param.value}'")

                elif param.name == "accel_low_pass_gain" and param.type_ == Parameter.Type.DOUBLE:
                    mpc_cfg.accel_low_pass_gain = param.value
                    self.get_logger().warn(f"accel_low_pass_gain was updated to '{param.value}'")

                elif param.name == "steer_low_pass_gain" and param.type_ == Parameter.Type.DOUBLE:
                    mpc_cfg.steer_low_pass_gain = param.value
                    self.get_logger().warn(f"steer_low_pass_gain was updated to '{param.value}'")

                elif param.name == "wp_id_offset" and param.type_ == Parameter.Type.INTEGER:
                    mpc_cfg.wp_id_offset = param.value
                    self._mpc.update_wp_id_offset(param.value)
                    self.get_logger().warn(f"wp_id_offset was updated to '{param.value}'")

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


        def create_obstacles() -> List[Obstacle]:
            use_csv_obstacles = self._cfg.obstacles.csv_path != "" # type: ignore
            if use_csv_obstacles:
                obstacles_file_path = self.in_pkg_share(self._cfg.obstacles.csv_path) # type: ignore
                obs_x, obs_y = load_waypoints(obstacles_file_path)
                obstacles = []
                for cx, cy in zip(obs_x, obs_y):
                    obstacles.append(Obstacle(cx=cx, cy=cy, radius=self._cfg.obstacles.radius)) # type: ignore
                self._obstacle_manager = ObstacleManager(self._map, obstacles)
                return obstacles
            else:
                return []

        def create_car(ref_path: ReferencePath) -> BicycleModel:
            cfg_model = self._cfg.bicycle_model # type: ignore
            return BicycleModel(
                ref_path,
                cfg_model.length,
                cfg_model.width,
                1.0 / self._cfg.mpc.control_rate) # type: ignore

        def create_mpc(car: BicycleModel) -> Tuple[MPCConfig, MPC]:
            cfg_mpc = self._cfg.mpc # type: ignore

            mpc_cfg = MPCConfig(
                cfg_mpc.N,
                sparse.diags(cfg_mpc.Q),
                sparse.diags(cfg_mpc.R),
                sparse.diags(cfg_mpc.QN),
                kmh_to_m_per_sec(self.BUG_VEL if self.USE_BUG_ACC else cfg_mpc.v_max),
                cfg_mpc.a_min,
                cfg_mpc.a_max,
                cfg_mpc.ay_max,
                np.deg2rad(cfg_mpc.delta_max_deg),
                cfg_mpc.steer_rate_max,
                cfg_mpc.control_rate,
                cfg_mpc.steering_tire_angle_gain_var,
                cfg_mpc.accel_low_pass_gain,
                cfg_mpc.steer_low_pass_gain,
                cfg_mpc.wp_id_offset,
                cfg_mpc.use_max_kappa_pred)

            state_constraints = {
                "xmin": np.array([-np.inf, -np.inf, -np.inf]),
                "xmax": np.array([np.inf, np.inf, np.inf])}
            input_constraints = {
                "umin": np.array([0.0, -np.tan(mpc_cfg.delta_max) / car.length]),
                "umax": np.array([mpc_cfg.v_max, np.tan(mpc_cfg.delta_max) / car.length])}

            # mpcからのsteer指令出力は、gainを掛けて出力され、その状態で車体のsteer rate limit が適用されるため、
            # mpcの制御計算におけるsteer_rate_maxは、実際のsteer_rate_maxをgainで除した値で設定する
            scaled_steer_rate_max = mpc_cfg.steer_rate_max / mpc_cfg.steering_tire_angle_gain_var

            mpc = MPC(
                car,
                mpc_cfg.N,
                mpc_cfg.Q,
                mpc_cfg.R,
                mpc_cfg.QN,
                state_constraints,
                input_constraints,
                mpc_cfg.ay_max,
                scaled_steer_rate_max,
                mpc_cfg.wp_id_offset,
                self.USE_OBSTACLE_AVOIDANCE,
                self._cfg.reference_path.use_path_constraints_topic,
                mpc_cfg.use_max_kappa_pred)

            return mpc_cfg, mpc

        def compute_speed_profile(car: BicycleModel, mpc_config: MPCConfig) -> None:
            speed_profile_constraints = {
                "a_min": mpc_config.a_min, "a_max": mpc_config.a_max,
                "v_min": 0.0, "v_max": mpc_config.v_max, "ay_max": mpc_config.ay_max}
            car.reference_path.compute_speed_profile(speed_profile_constraints)

        def create_ref_vel_configulator() -> Optional[ReferenceVelocityConfigulator]:
            if self._ref_vel_config_path is None:
                return None
            return ReferenceVelocityConfigulator(
                self, self._config_path, self._ref_vel_config_path,
                on_change=self._rebuild_ref_vel_profile
            )

        self._map = create_map()
        self._reference_path = create_ref_path(self._map)
        self._car = create_car(self._reference_path)
        self._mpc_cfg, self._mpc = create_mpc(self._car)
        compute_speed_profile(self._car, self._mpc_cfg)

        self._ref_vel_configulator: Optional[ReferenceVelocityConfigulator] = create_ref_vel_configulator()

        self._trajectory: Optional[Trajectory] = None
        self._path_constraints = None

        # Obstacles
        if self.USE_OBSTACLE_AVOIDANCE:
            self._static_obstacles: List[Obstacle] = create_obstacles()
            self._dynamic_obstacles: List[Obstacle] = []
            self._obstacles_updated = bool(self._static_obstacles)
            v2x_cfg = self._cfg.v2x_obstacle_avoidance  # type: ignore
            self._v2x_tracker = V2XVehicleTracker(
                v_max_safety=float(v2x_cfg.v_max_safety),
                position_jump_threshold=float(v2x_cfg.position_jump_threshold),
                warn_callback=self.get_logger().warn,
            )
            self._v2x_vehicle_radius = float(v2x_cfg.vehicle_radius)
            mpc_N = int(self._cfg.mpc.N)  # type: ignore
            t_horizon = mpc_N / float(self._cfg.mpc.control_rate)  # type: ignore
            self._v2x_t_samples = [
                k * t_horizon / max(mpc_N - 1, 1) for k in range(mpc_N)
            ]
            # コリドー外の V2X 障害物で MPC のコリドー狭窄/反転が起きないよう、
            # ref-path 近傍のみに絞り込む。閾値 = max_width/2 + vehicle_radius + 余白。
            ref_max_width = float(self._cfg.reference_path.max_width)  # type: ignore
            self._v2x_corridor_threshold_sq = (
                ref_max_width / 2.0 + self._v2x_vehicle_radius + 0.5
            ) ** 2
            wps = self._reference_path.waypoints
            self._waypoint_xy = np.asarray(
                [(wp.x, wp.y) for wp in wps], dtype=np.float64)

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

        # stats
        self._stats = ExecutionStats(self.get_logger(), window_size=50, record_count_threshold=1000)

        # --- State machine ---------------------------------------------------
        self._target_lateral_offset = 0.0
        self._current_lateral_offset = 0.0
        self._state_manager = StateManager(self)
        self._follow_prev_e_y = 0.0
        self._follow_prev_e_psi = 0.0

        # V2X tracker for state context — always initialise so the state
        # machine can detect forward vehicles regardless of obstacle-avoidance
        # mode.  When USE_OBSTACLE_AVOIDANCE is True the tracker is already
        # created above (line ~459); only create it here if it was skipped.
        if not hasattr(self, '_v2x_tracker'):
            v2x_cfg = self._cfg.v2x_obstacle_avoidance  # type: ignore
            self._v2x_tracker = V2XVehicleTracker(
                v_max_safety=float(v2x_cfg.v_max_safety),
                position_jump_threshold=float(v2x_cfg.position_jump_threshold),
                warn_callback=self.get_logger().warn,
            )

        # Precompute waypoint array for path-deviation calculation — always
        # needed by the state machine even when obstacle avoidance is active.
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

    def _build_ref_vel_profile(self) -> Optional[List[float]]:
        """Build the per-waypoint reference velocity [m/s], once, at startup.

        Shape comes from the raceline CSV's own ``vx_mps`` column (the min-curvature
        optimizer solved it for ay <= 12 m/s^2), so the car keeps straight-line speed and
        only slows where the curvature demands it. On top of that:

          * ``mpc.v_ref_scale``   — global multiplier for lap-time tuning.
          * ``mpc.v_ref_min_kmh`` — floor, so tight apexes are not crawled through.
          * ``ref_vel.yaml``      — per-section cap in km/h, live-tunable at runtime.
          * ``mpc.v_max``         — global cap.

        Returns ``None`` when no CSV profile is available, in which case the curvature
        based profile from ``compute_speed_profile()`` is left untouched.
        """
        cfg_mpc = self._cfg.mpc  # type: ignore
        csv_path = str(self._cfg.reference_path.csv_path)  # type: ignore
        if csv_path == "":
            return None

        csv_x, csv_y, csv_v = load_ref_path_speed_profile(self.in_pkg_share(csv_path))
        if not csv_v:
            self.get_logger().warn(
                "Reference path CSV has no 'vx_mps' column; keeping the curvature-based speed profile")
            return None

        csv_xy = np.asarray(list(zip(csv_x, csv_y)), dtype=np.float64)
        csv_v_arr = np.asarray(csv_v, dtype=np.float64)

        scale = float(getattr(cfg_mpc, "v_ref_scale", 1.0))
        v_min = kmh_to_m_per_sec(float(getattr(cfg_mpc, "v_ref_min_kmh", 28.0)))

        profile: List[float] = []
        for i, wp in enumerate(self._reference_path.waypoints):
            diffs = csv_xy - np.array([wp.x, wp.y], dtype=np.float64)
            j = int(np.argmin(np.einsum("ij,ij->i", diffs, diffs)))

            cap = float(self._mpc_cfg.v_max)
            if self._ref_vel_configulator is not None:
                try:
                    cap = min(cap, kmh_to_m_per_sec(float(self._ref_vel_configulator.get_ref_vel(i))))
                except ValueError:
                    pass  # waypoint outside every configured section: fall back to v_max
            # The floor must never override the cap, otherwise a deliberately slow
            # section would be silently sped back up.
            profile.append(float(np.clip(csv_v_arr[j] * scale, min(v_min, cap), cap)))

        return profile

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

        # NOTE:評価環境での可視化のためにダミーのトピック名を使用
        self._mpc_pred_pub = self.create_publisher(
            MarkerArray, "/mpc/prediction", 1)
        self._mpc_pred_pub_dummy = self.create_publisher(
            MarkerArray, "/planning/scenario_planning/lane_driving/motion_planning/obstacle_stop_planner/virtual_wall", 1)

        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        # NOTE:評価環境での可視化のためにダミーのトピック名を使用
        self._ref_path_pub = self.create_publisher(
            MarkerArray, "/mpc/ref_path", latching_qos)
        self._ref_path_pub_dummy = self.create_publisher(
            MarkerArray, "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/bound", latching_qos)

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

        if self.USE_OBSTACLE_AVOIDANCE:
            if self._cfg.reference_path.use_path_constraints_topic: # type: ignore
                self._path_constraints_sub = self.create_subscription(
                    PathConstraints, "/path_constraints_provider/path_constraints", self._path_constraints_callback, 1)

            if self._cfg.reference_path.use_border_cells_topic: # type: ignore
                self._border_cells_sub = self.create_subscription(
                    BorderCells, "/path_constraints_provider/border_cells", self._border_cells_callback, 1)

        # V2X subscriber — always subscribe for state-machine context,
        # even when obstacle avoidance is disabled.
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

    def _path_constraints_callback(self, msg: PathConstraints):
        self._reference_path.set_path_constraints(
            msg.upper_bounds, msg.lower_bounds, msg.rows, msg.cols)

    def _v2x_callback(self, msg: V2XVehiclePositionArray) -> None:
        self._v2x_tracker.update(msg)
        active_ids = self._v2x_tracker.active_vehicle_ids()
        self.get_logger().info(f"V2X active ids: {active_ids}, ego id: {self._vehicle_id}", throttle_duration_sec=2.0)

        if self.USE_OBSTACLE_AVOIDANCE:
            predictions = {}
            for vid in active_ids:
                if vid == self._vehicle_id:
                    continue  # Exclude self vehicle
                predictions[vid] = self._v2x_tracker.predict_positions(vid, self._v2x_t_samples)
            self._dynamic_obstacles = predictions_to_obstacles(
                predictions, self._v2x_vehicle_radius)
            self._obstacles_updated = True

    def _filter_obstacles_to_corridor(self, obstacles: List[Obstacle]) -> List[Obstacle]:
        if not obstacles or self._waypoint_xy.size == 0:
            return obstacles
        thr_sq = self._v2x_corridor_threshold_sq
        wps = self._waypoint_xy
        kept: List[Obstacle] = []
        for ob in obstacles:
            dxy = wps - np.array([ob.cx, ob.cy], dtype=np.float64)
            if np.min(np.einsum('ij,ij->i', dxy, dxy)) <= thr_sq:
                kept.append(ob)
        return kept

    def _border_cells_callback(self, msg: BorderCells):
        self._reference_path.set_border_cells(
            msg.dynamic_upper_bounds, msg.dynamic_lower_bounds, msg.rows, msg.cols)

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

    def _wait_until_path_constraints_received(self, timeout: float = 30.) -> None:
        if self.USE_OBSTACLE_AVOIDANCE and self._cfg.reference_path.use_path_constraints_topic: # type: ignore
            self._wait_until_message_received(lambda: self._reference_path.path_constraints, 'path constraints', timeout)

    def _publish_mpc_pred_marker(self, x_pred, y_pred):
        pred_marker_array = MarkerArray()
        m_base = Marker()
        m_base.header.frame_id = "map"
        m_base.ns = "mpc_pred"
        m_base.type = Marker.SPHERE
        m_base.action = Marker.ADD
        m_base.pose.position.z = 0.0
        m_base.scale = Vector3(x=0.5, y=0.5, z=0.5)
        m_base.color = self._pred_marker_color
        for i in range(len(x_pred)):
            m = copy.deepcopy(m_base)
            m.id = i
            m.pose.position.x = x_pred[i]
            m.pose.position.y = y_pred[i]
            pred_marker_array.markers.append(m) # type: ignore
        self._mpc_pred_pub.publish(pred_marker_array)
        self._mpc_pred_pub_dummy.publish(pred_marker_array)

    def _publish_ref_path_marker(self, ref_path: ReferencePath):
        WP_SPHERE_ENABLED = False

        ref_path_marker_array = MarkerArray()

        m_base = Marker()
        m_base.header.frame_id = "map"
        m_base.ns = "ref_path"
        m_base.type = Marker.LINE_STRIP
        m_base.action = Marker.ADD
        m_base.pose.position.z = 0.0
        m_base.scale.x = 0.2
        m_base.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.7)

        for i in range(len(ref_path.waypoints) - 1):
            m = copy.deepcopy(m_base)
            m.id = i
            start = Point()
            start.x = ref_path.waypoints[i].x
            start.y = ref_path.waypoints[i].y
            end = Point()
            end.x = ref_path.waypoints[i + 1].x
            end.y = ref_path.waypoints[i + 1].y
            m.points.append(start) # type: ignore
            m.points.append(end) # type: ignore
            ref_path_marker_array.markers.append(m) # type: ignore

        if WP_SPHERE_ENABLED:
            spheres = Marker()
            spheres.header.frame_id = "map"
            spheres.ns = "ref_path_point"
            spheres.type = Marker.SPHERE_LIST
            spheres.action = Marker.ADD
            radius = 0.2
            spheres.scale = Vector3(x=radius, y=radius, z=radius)
            spheres.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.7)
            for i in range(len(ref_path.waypoints) - 1):
                p = Point()
                p.x = ref_path.waypoints[i].x
                p.y = ref_path.waypoints[i].y
                p.z = 0.
                spheres.points.append(p) #type: ignore
            ref_path_marker_array.markers.append(spheres) # type: ignore

        self._ref_path_pub.publish(ref_path_marker_array)
        self._ref_path_pub_dummy.publish(ref_path_marker_array)

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
            if hasattr(self._mpc, "model") and hasattr(self._mpc.model, "wp_id"):
                self._mpc.model.wp_id = closest_idx

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
        self, pose, v_current: float, ctx: StateContext
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
            if hasattr(self._mpc, "model") and hasattr(self._mpc.model, "wp_id"):
                self._mpc.model.wp_id = closest_idx

            # センターライン基準の目標横オフセット（空き空間の中心）を取得
            # 壁からのマージン0.5mを先行車両の左右の空き幅から引いて半分にした値を
            # 先行車両の横端からの追い越しラインとする。実際の座標はセンターラインから
            # 先行車両の車幅の半分の最大0.725m未満+先行車両との横マージン0.225mだけ
            # ずらした位置が実際の追い越し時の座標になる
            target_offset = ctx.target_overtake_offset
            if abs(target_offset) > 0.1:
                is_left = (ctx.overtake_width_left >= ctx.overtake_width_right)
                # target_offset = 1.6 if is_left else -1.6
                target_offset = (
                    ((ctx.overtake_width_left - 0.5) / 2.0) + 0.725 + 0.225
                    if is_left else -(((ctx.overtake_width_right - 0.5) / 2.0) + 0.725 + 0.225)
                )

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
                ub = float(wp.ub) if wp.ub is not None else 3.5  # 左端
                lb = float(wp.lb) if wp.lb is not None else -3.5 # 右端

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

    def _compute_path_deviation(self) -> float:
        """Approximate lateral deviation from the reference path [m]."""
        car_xy = np.array(
            [self._car.temporal_state.x, self._car.temporal_state.y],
            dtype=np.float64,
        )
        diffs = self._waypoint_xy - car_xy
        dists_sq = np.einsum("ij,ij->i", diffs, diffs)
        return float(np.sqrt(np.min(dists_sq)))

    def _rebuild_ref_vel_profile(self) -> None:
        """Rebuild the reference velocity profile after a ref_vel.yaml parameter change.

        Called from the ReferenceVelocityConfigulator parameter callback (executor thread)
        so that live tuning of the per-section caps still takes effect. The new list is
        built first and swapped in with a single assignment.
        """
        if getattr(self, "_ref_vel_profile", None) is None:
            return  # still initialising, or no CSV profile in use
        try:
            profile = self._build_ref_vel_profile()
        except Exception as e:  # never let a tuning mistake kill the parameter callback
            self.get_logger().error(f"Failed to rebuild reference velocity profile: {e}")
            return
        if profile is None:
            return
        self._ref_vel_profile = profile
        # FollowState rewrites v_ref every tick with its own leader-capped list,
        # so leave it alone and let the next tick pick the new profile up.
        state_manager = getattr(self, "_state_manager", None)
        if state_manager is None or not isinstance(state_manager.current_state, FollowState):
            self._reference_path.set_v_ref(profile)

    def _update_waypoint_cache(self) -> None:
        """Precompute waypoint coordinates, cumulative arc lengths, and track length."""
        wps = self._reference_path.waypoints
        n_wps = len(wps)
        self._waypoint_xy = np.asarray([(wp.x, wp.y) for wp in wps], dtype=np.float64)

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
        fwd_detect_distance: float = 8.0,
        fwd_angle_min_deg: float = -15.0,
        fwd_angle_max_deg: float = 15.0,
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

        time_since_collision: Optional[float] = None
        if self._last_colliding_time is not None:
            time_since_collision = (
                self.get_clock().now() - self._last_colliding_time
            ).nanoseconds / 1e9

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
            time_since_collision=time_since_collision,
            path_deviation=self._compute_path_deviation(),
            path_psi=path_psi,
            path_e_y=path_e_y,
            forward_vehicle_distance=fwd_dist,
            forward_vehicle_speed=fwd_speed,
            forward_vehicle_heading_diff=fwd_heading_diff,
            nearest_vehicle_s_rel=nearest_s_rel,
            overtake_width_left=left_w,
            overtake_width_right=right_w,
            target_overtake_offset=target_offset,
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
        )

    def _apply_state_params(self, params: MPCStateParams) -> None:
        """Push state-specific MPC parameters into the solver and speed profile."""
        v_max_mps = kmh_to_m_per_sec(params.v_max)

        self._mpc_cfg.v_max = v_max_mps
        self._mpc.update_v_max(v_max_mps)

        self._mpc_cfg.ay_max = params.ay_max
        self._mpc.update_ay_max(params.ay_max)

        self._mpc_cfg.Q = sparse.diags(params.Q)
        self._mpc.update_Q(self._mpc_cfg.Q)

        self._mpc_cfg.R = sparse.diags(params.R)
        self._mpc.update_R(self._mpc_cfg.R)
        self._mpc_cfg.QN = sparse.diags(params.QN)
        self._mpc.update_QN(self._mpc_cfg.QN)

        self._target_lateral_offset = params.lateral_offset
        # Avoid heavy synchronous compute_speed_profile OSQP re-computation during state transitions to eliminate freezes

    # ------------------------------------------------------------------
    # Main control loop
    # ------------------------------------------------------------------

    def _control(self):
        if self._odom is None or self._waypoint_xy is None or len(self._waypoint_xy) == 0:
            return

        try:
            now = self.get_clock().now()
            t = (now - self._t_start).nanoseconds / 1e9
            dt = (now - self._last_t).nanoseconds / 1e9

            self._last_t = now
            self._loop += 1

            # record and print execution stats
            if self.use_stats:
                self._stats.record()

            # self.get_logger().info("loop")
            self._control_rate.sleep()

            if self._loop % 100 == 0:
                # update reference path
                if self._cfg.reference_path.update_by_topic: # type: ignore
                    new_referece_path = self._create_reference_path_from_autoware_trajectory(self._trajectory)
                    if new_referece_path is not None:
                        self._car.reference_path = new_referece_path
                        self._car.update_reference_path(self._car.reference_path)

                def plot_reference_path(car):
                    import matplotlib.pyplot as plt
                    import sys
                    fig, ax = plt.subplots(1, 1)
                    car.reference_path.show(ax)
                    plt.show()
                    sys.exit(1)
                # plot_reference_path(self._car)

            if self.USE_OBSTACLE_AVOIDANCE and self._obstacles_updated:
                self._obstacles_updated = False
                self._map.reset_map()
                filtered_dynamic = self._filter_obstacles_to_corridor(self._dynamic_obstacles)
                self._map.add_obstacles(self._static_obstacles + filtered_dynamic)
                self._reference_path.reset_dynamic_constraints()

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
                # Instantly reset last control memory to forward motion for zero-lag launch
                self._last_u[0] = 1.5
                self._last_acc = 1.0
                self.get_logger().info("Exited RecoveryState: instant launch & recovery cooldown started")

            # Smoothly interpolate lateral offset: fast rate (0.22) for agile avoidance in OvertakeState, smooth (0.08) for return
            rate = 0.0
            if isinstance(current_state, OvertakeState):
                rate = 1.3
            elif isinstance(current_state, FollowState):
                rate = 0.6
            self._current_lateral_offset += (self._target_lateral_offset - self._current_lateral_offset) * rate

            offset = self._current_lateral_offset
            x_shifted = pose.x - offset * np.sin(pose.theta)
            y_shifted = pose.y + offset * np.cos(pose.theta)
            self._car.update_states(x_shifted, y_shifted, pose.theta)

            # Follow state: dynamically adjust v_max to match leader speed
            if isinstance(current_state, FollowState):
                if ctx.forward_vehicle_speed is not None:
                    dynamic_v_max = current_state.get_adjusted_v_max_mps(ctx)
                    self._mpc.update_v_max(dynamic_v_max)
                    v_ref_list: List[float] = [dynamic_v_max] * len(self._reference_path.waypoints)
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
                self._sim_logger.plot_animation(t, self._loop, self._current_laps, self._lap_times, is_colliding, u, self._mpc, self._car)
                return

            # ---- Control Selection (Hybrid: Pure Pursuit for FollowPathState, Waypoint-Shift Pure Pursuit for Overtake, MPC for Follow/Recovery) ----
            if current_state.control_mode == ControlMode.PURE_PURSUIT:
                v_target, steer_target = self._compute_pure_pursuit_control(pose, v)
                u = [v_target, steer_target]
                max_delta = steer_target
            elif current_state.control_mode == ControlMode.WAYPOINT_SHIFT_PURE_PURSUIT:
                v_target, steer_target = self._compute_waypoint_shift_pure_pursuit_control(pose, v, ctx)
                u = [v_target, steer_target]
                max_delta = steer_target
            else:
                with self._stats.time_block("control"):
                    u, max_delta = self._mpc.get_control()

            # FollowState uses dynamic following speed; skip static ref_vel overwrite
            if self._ref_vel_configulator is not None and not isinstance(current_state, FollowState):
                ref_vel_mps = self._ref_vel_configulator.get_ref_vel(self._mpc.model.wp_id)
                ref_vel_mps_capped = min(ref_vel_mps, self._mpc_cfg.v_max)
                self._mpc.update_v_max(ref_vel_mps_capped)
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
                    self._pred_marker_color = RED
                elif abs(v) > kmh_to_m_per_sec(41.0) or abs(u[1]) > deg2rad(10.0):
                    bug_acc_enabled = False
                    acc = self._mpc_cfg.a_max
                    self._pred_marker_color = YELLOW
                else:
                    bug_acc_enabled = True
                    acc = 500.0
                    self._pred_marker_color = CYAN
            else:
                acc = self.KP * (u[0] - v)
                # print(f"v: {v}, u[0]: {u[0]}, acc: {acc}")
                acc = np.clip(acc, self._mpc_cfg.a_min, self._mpc_cfg.a_max)

            # FollowState: Limit positive acceleration to prevent rear-end collisions due to latency
            # if isinstance(current_state, FollowState):
            #     acc = np.clip(acc, self._mpc_cfg.a_min, 1.0)  # max 1.0 m/s² (half of a_max)
            if isinstance(current_state, FollowState):
                if current_state.control_mode == ControlMode.MPC:
                    acc = np.clip(acc, self._mpc_cfg.a_min, 1.0)  # max 1.0 m/s² (half of a_max)
                    # Waypoint-relative heading error [-pi, pi] and lateral error [m]
                    e_psi = (pose.theta - ctx.path_psi + np.pi) % (2 * np.pi) - np.pi
                    e_y = ctx.path_e_y  # Left > 0, Right < 0
                    dt_safe = max(dt, 0.001)

                    # Derivative terms (rate of error change)
                    d_e_y = (e_y - self._follow_prev_e_y) / dt_safe
                    d_e_psi = (e_psi - self._follow_prev_e_psi) / dt_safe

                    # Store previous errors
                    self._follow_prev_e_y = e_y
                    self._follow_prev_e_psi = e_psi

                    # PD Gains:
                    # P: restores position & heading to waypoint center
                    # D: provides proactive counter-steering & damping when crossing center line (prevents wall overshoot)
                    # KP_Y, KD_Y = 0.35, 0.12
                    # KP_PSI, KD_PSI = 0.70, 0.20

                    # case 1: 衝突することはあるが、すこし安定
                    KP_Y, KD_Y = 0.30, 0.12
                    KP_PSI, KD_PSI = 0.80, 0.20

                    pd_y = KP_Y * e_y + KD_Y * d_e_y
                    pd_psi = KP_PSI * e_psi + KD_PSI * d_e_psi

                    steer_correction = float(np.clip(-(pd_y + pd_psi), -np.deg2rad(12.0), np.deg2rad(12.0)))
                    u[1] += steer_correction
            else:
                # Reset memory when outside FollowState
                self._follow_prev_e_y = 0.0
                self._follow_prev_e_psi = 0.0

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
            self._sim_logger.plot_animation(t, self._loop, self._current_laps, self._lap_times, is_colliding, u, self._mpc, self._car)

            # 約 0.25 秒ごとに予測結果を表示
            if (self._mpc.current_prediction is not None) and (self._loop % (self._mpc_cfg.control_rate // 4) == 0):
                self._publish_mpc_pred_marker(self._mpc.current_prediction[0], self._mpc.current_prediction[1]) # type: ignore
        except Exception as e:
            self.get_logger().error(f"Error in _control loop (continuing execution): {e}")

    def run(self) -> None:
        try:
            self._wait_until_clock_received()
            self._wait_until_odom_received()
            self._wait_until_trajectory_received(timeout=5.0)
            self._wait_until_path_constraints_received(timeout=5.0)
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

        if self._ref_vel_configulator is None:
            self._publish_ref_path_marker(self._car.reference_path)

        self._pred_marker_color = CYAN

        # for i in range(10):
        #     self._obstacle_manager.push_next_obstacle()

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
