#!/usr/bin/env python3

import yaml
import math
from typing import List, Dict, Tuple, Optional, NamedTuple
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
from std_msgs.msg import ColorRGBA

from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

# autoware
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import GearCommand
from autoware_auto_planning_msgs.msg import Trajectory
from v2x_msgs.msg import V2XVehiclePositionArray
from multi_purpose_mpc_ros.v2x_vehicle_tracker import (
    V2XVehicleTracker,
    predictions_to_obstacles,
)

# Multi_Purpose_MPC
from multi_purpose_mpc_ros.core.map import Map, Obstacle
from multi_purpose_mpc_ros.core.reference_path import ReferencePath
from multi_purpose_mpc_ros.core.spatial_bicycle_models import BicycleModel
from multi_purpose_mpc_ros.core.MPC import MPC
from multi_purpose_mpc_ros.core.utils import load_waypoints, kmh_to_m_per_sec, load_ref_path

# Project
from multi_purpose_mpc_ros.common import convert_to_namedtuple, file_exists
from multi_purpose_mpc_ros.simulation_logger import SimulationLogger
from multi_purpose_mpc_ros.obstacle_manager import ObstacleManager
from multi_purpose_mpc_ros.exexution_stats import ExecutionStats
from multi_purpose_mpc_ros_msgs.msg import AckermannControlBoostCommand, PathConstraints, BorderCells
from multi_purpose_mpc_ros.tools.reference_velocity_configulator import ReferenceVelocityConfigulator


RED = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
YELLOW = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
CYAN = ColorRGBA(r=0.0, g=156.0 / 255.0, b=209.0 / 255.0, a=1.0)

def array_to_ackermann_control_command(stamp, u: np.ndarray, acc: float) -> AckermannControlCommand:
    msg = AckermannControlCommand()
    msg.stamp = stamp
    msg.lateral.stamp = stamp
    # u[1] is already tire angle delta (rad) — converted from curvature kappa by MPC.py L356
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
    delay_compensation_sec: float
    enable_dynamic_delay_compensation: bool


class MPCController(Node):

    PKG_PATH: str = get_package_share_directory('multi_purpose_mpc_ros') + "/"
    # MAX_LAPS = 6
    MAX_LAPS = 10000
    BUG_VEL = 40.0 # km/h
    BUG_ACC = 400.0

    SHOW_PLOT_ANIMATION = False
    PLOT_RESULTS = False
    ANIMATION_INTERVAL = 20

    KP = 2.5  # Smooth continuous acceleration P gain eliminating 40Hz Bang-Bang pitching oscillation

    def __init__(self, config_path: str, ref_vel_config_path: Optional[str]) -> None:
        super().__init__("mpc_controller") # type: ignore

        # declare parameters
        self.declare_parameter("use_boost_acceleration", False)
        self.declare_parameter("use_obstacle_avoidance", True)
        self.declare_parameter("use_stats", False)
        self.declare_parameter("vehicle_id", os.environ.get("VEHICLE_ID", "d1"))

        # get parameters
        self.use_sim_time = self.get_parameter("use_sim_time").get_parameter_value().bool_value
        self.USE_BUG_ACC = self.get_parameter("use_boost_acceleration").get_parameter_value().bool_value
        self.USE_OBSTACLE_AVOIDANCE = self.get_parameter("use_obstacle_avoidance").get_parameter_value().bool_value
        self.use_stats = self.get_parameter("use_stats").get_parameter_value().bool_value
        self._vehicle_id = self.get_parameter("vehicle_id").get_parameter_value().string_value
        if self._vehicle_id in ["A0", "default"]:
            domain_id = int(os.environ.get("ROS_DOMAIN_ID", "1"))
            self._vehicle_id = f"d{domain_id}"
        self.get_logger().info(f"VEHICLE ID INITIALIZED AS: {self._vehicle_id}")

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
            self.declare_parameter("delay_compensation_sec", mpc_cfg.delay_compensation_sec)
            self.declare_parameter("enable_dynamic_delay_compensation", mpc_cfg.enable_dynamic_delay_compensation)

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

                elif param.name == "delay_compensation_sec" and param.type_ == Parameter.Type.DOUBLE:
                    mpc_cfg.delay_compensation_sec = param.value
                    self.get_logger().warn(f"delay_compensation_sec was updated to '{param.value}'")

                elif param.name == "enable_dynamic_delay_compensation" and param.type_ == Parameter.Type.BOOL:
                    mpc_cfg.enable_dynamic_delay_compensation = param.value
                    self.get_logger().warn(f"enable_dynamic_delay_compensation was updated to '{param.value}'")


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
                cfg_mpc.use_max_kappa_pred,
                float(getattr(cfg_mpc, 'delay_compensation_sec', 0.10)),
                bool(getattr(cfg_mpc, 'enable_dynamic_delay_compensation', True)))

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
            return ReferenceVelocityConfigulator(self, self._config_path, self._ref_vel_config_path)

        self._map = create_map()
        self._reference_path = create_ref_path(self._map)
        self._car = create_car(self._reference_path)
        self._mpc_cfg, self._mpc = create_mpc(self._car)
        compute_speed_profile(self._car, self._mpc_cfg)

        # If not using topic-based path constraints, initialize static path_constraints now
        # so MPC always has valid bounds even with use_obstacle_avoidance: true
        if not self._cfg.reference_path.use_path_constraints_topic:  # type: ignore
            mpc_N = int(self._cfg.mpc.N)  # type: ignore
            safety_margin = float(self._car.safety_margin)
            self._reference_path.update_simple_path_constraints(mpc_N, safety_margin)

        self._ref_vel_configulator: Optional[ReferenceVelocityConfigulator] = create_ref_vel_configulator()

        self._trajectory: Optional[Trajectory] = None
        self._path_constraints = None

        # Obstacles
        if self.USE_OBSTACLE_AVOIDANCE:
            self._static_obstacles: List[Obstacle] = create_obstacles()
            self._dynamic_obstacles: List[Obstacle] = []
            self._obstacles_updated = bool(self._static_obstacles)
            self._had_obstacles = False
            v2x_cfg = self._cfg.v2x_obstacle_avoidance  # type: ignore
            self._v2x_tracker = V2XVehicleTracker(
                v_max_safety=float(v2x_cfg.v_max_safety),
                position_jump_threshold=float(v2x_cfg.position_jump_threshold),
                warn_callback=self.get_logger().warn,
            )
            self._v2x_vehicle_radius = float(v2x_cfg.vehicle_radius)
            self._v2x_vehicle_radius_normal = float(v2x_cfg.vehicle_radius)
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

            # V2X 近接制御・追い越しモード状態
            self._v2x_speed_limit: float = float('inf')   # 動的速度上限 [m/s]
            self._v2x_mode: str = "NORMAL"                # NORMAL / FOLLOWING / OVERTAKING / EMERGENCY_BRAKE
            self._v2x_following_since: Optional[float] = None  # FOLLOWING 開始時刻 [s]
            self._v2x_motion_start_time: Optional[float] = None  # 車両が動き出した時刻 [s]

        # Stuck Recovery 状態管理
        self._stuck_state: str = "NORMAL"  # NORMAL / REVERSING / STOP_BEFORE_FORWARD
        self._stuck_timer_start: Optional[float] = None
        self._stuck_phase_start: Optional[float] = None
        self._has_launched: bool = False  # 発進完了フラグ（初期発進時の誤リバース発動を予防）
        self._stuck_evasive_steer: float = 0.0  # バック時回避切返し操舵角

        # Laps
        self._current_laps = 1
        self._last_lap_time = 0.0
        self._lap_times = [None] * (self.MAX_LAPS + 1) # +1 means include lap 0

        # condition
        self._last_condition = None
        self._last_colliding_time = None

        # stats
        self._stats = ExecutionStats(self.get_logger(), window_size=50, record_count_threshold=1000)

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
        self._gear_pub = self.create_publisher(
            GearCommand, "/control/command/gear_cmd", 1)

        if self.USE_BUG_ACC:
          self._command_pub = self.create_publisher(
            AckermannControlBoostCommand, "/boost_commander/command", 1)
        else:
          self._command_pub = self.create_publisher(
            AckermannControlCommand, "/control/command/control_cmd", 1)
          self._command_raw_pub = self.create_publisher(
            AckermannControlCommand, "/control/command/control_cmd_raw", 1)
          print("use normal ackermann control command")

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

            self._v2x_sub = self.create_subscription(
                V2XVehiclePositionArray,
                "/v2x/vehicle_positions",
                self._v2x_callback,
                1)

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

        # publish raw control command
        if not self.USE_BUG_ACC:
            self._command_raw_pub.publish(cmd)

        # Pure 1:1 control command output without redundant steering gain multiplication
        # actuation_cmd_converter in downstream pipeline handles gains natively.
        self._command_pub.publish(cmd)

    def _publish_gear_command(self, gear_val: int) -> None:
        gear_msg = GearCommand()
        gear_msg.stamp = self.get_clock().now().to_msg()
        gear_msg.command = gear_val
        self._gear_pub.publish(gear_msg)


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

        ego_x, ego_y = 0.0, 0.0
        # 近接制御・追い越しモードの更新
        if self._odom is not None:
            pose = odom_to_pose_2d(self._odom)
            ego_x, ego_y = pose.x, pose.y
            ego_speed = self._odom.twist.twist.linear.x
            self._update_v2x_mode(pose.x, pose.y, pose.theta, ego_speed)

        predictions = {}
        for vid in active_ids:
            if vid == self._vehicle_id:
                continue
            pred_pts = self._v2x_tracker.predict_positions(vid, self._v2x_t_samples)
            if self._odom is not None and pred_pts and len(pred_pts) > 0:
                # 自車位置から1.2m以内の障害物は自分自身とみなして完全除外
                first_pt = pred_pts[0]
                if math.hypot(first_pt[0] - ego_x, first_pt[1] - ego_y) < 1.2:
                    continue
            predictions[vid] = pred_pts

        self._dynamic_obstacles = predictions_to_obstacles(
            predictions, self._v2x_vehicle_radius)
        self._obstacles_updated = True

    def _update_v2x_mode(
        self, ego_x: float, ego_y: float, ego_yaw: float, ego_speed_mps: float
    ) -> None:
        """前方車両との距離・TTC を評価して走行モードと速度制限を更新する。

        モード遷移:
          NORMAL ──[前方15m以内]──▶ FOLLOWING ──[5s追走後]──▶ OVERTAKING
          FOLLOWING / OVERTAKING ──[TTC<3s or 距離<6m]──▶ EMERGENCY_BRAKE
          OVERTAKING ──[前方10m以上クリア]──▶ NORMAL
        """
        v2x_cfg = self._cfg.v2x_obstacle_avoidance  # type: ignore
        follow_start = float(v2x_cfg.follow_distance_start)
        follow_brake = float(v2x_cfg.follow_distance_brake)
        v_min_safe_mps = kmh_to_m_per_sec(float(v2x_cfg.v_min_safe))
        ttc_thresh = float(v2x_cfg.ttc_threshold)
        fwd_cos_thresh = float(getattr(v2x_cfg, 'forward_cos_threshold', 0.5))
        overtake_patience = float(v2x_cfg.overtake_patience)
        overtake_gap_min = float(v2x_cfg.overtake_gap_min)
        overtake_clearance = float(v2x_cfg.overtake_clearance)
        vehicle_radius_overtake = float(v2x_cfg.vehicle_radius_overtake)
        v_max_normal = self._mpc_cfg.v_max

        min_d = float('inf')
        min_ttc = float('inf')
        lead_speed = 0.0
        is_leading_ahead = False
        min_rel_fwd = 0.0
        min_rel_lat = 0.0

        fwd_cos = math.cos(ego_yaw)
        fwd_sin = math.sin(ego_yaw)

        for vid in self._v2x_tracker.active_vehicle_ids():
            if vid == self._vehicle_id:
                continue
            buf = self._v2x_tracker._samples.get(vid)
            if not buf:
                continue
            _, ox, oy = buf[-1]
            vx, vy = self._v2x_tracker.velocity(vid)

            dx = ox - ego_x
            dy = oy - ego_y
            d = math.hypot(dx, dy)

            # 自車位置そのもの（0.5m未満）は自分自身の可能性が高いため絶対除外
            if d < 0.5:
                continue

            # 前方判定：進行方向の内積をユークリッド距離で割ったcos値が閾値以上
            # （0.5 = 60度以内を「前方」と判定。横並び車両を除外）
            dot = dx * fwd_cos + dy * fwd_sin
            cos_angle = dot / max(d, 0.001)  # = cos(相対角度)
            is_ahead = cos_angle >= fwd_cos_thresh

            if is_ahead and d < min_d:
                min_d = d
                is_leading_ahead = True
                lead_speed = math.hypot(vx, vy)
                # 相対接近速度（自車進行方向成分: 縦速度）
                ego_vx = ego_speed_mps * fwd_cos
                ego_vy = ego_speed_mps * fwd_sin
                rel_approach = (ego_vx - vx) * fwd_cos + (ego_vy - vy) * fwd_sin
                # 横方向相対移動速度（自車横方向成分: クロス速度）
                rel_cross = abs((ego_vx - vx) * (-fwd_sin) + (ego_vy - vy) * fwd_cos)

                min_rel_fwd = rel_approach
                min_rel_lat = rel_cross

                if rel_approach > 0.0:
                    min_ttc = d / rel_approach
                else:
                    min_ttc = float('inf')  # 離れていく・同速

        now_sec = self.get_clock().now().nanoseconds / 1e9

        # ---- スタート抑制：車両が動き出してから N 秒間は V2X 制御を無効化 ----
        # グリッド並走スタートで隣の車が近いため EMERGENCY_BRAKE が誤発動するのを防ぐ
        startup_suppress_sec = float(getattr(v2x_cfg, 'startup_suppress_sec', 15.0))
        MOTION_THRESHOLD_MPS = 1.0  # [m/s] この速度を超えた時点を「動き出し」と定義
        if ego_speed_mps >= MOTION_THRESHOLD_MPS and self._v2x_motion_start_time is None:
            self._v2x_motion_start_time = now_sec
            self.get_logger().info(
                f"[V2X] Vehicle started moving (v={ego_speed_mps:.1f}m/s). "
                f"V2X suppressed for {startup_suppress_sec:.0f}s.")
        if (self._v2x_motion_start_time is None or
                now_sec - self._v2x_motion_start_time < startup_suppress_sec):
            self._v2x_mode = "NORMAL"
            self._v2x_speed_limit = float('inf')
            return

        overtake_speed_diff_thresh = float(getattr(v2x_cfg, 'overtake_speed_diff_threshold', 3.0))
        cross_velocity_thresh = float(getattr(v2x_cfg, 'cross_velocity_threshold', 2.5))

        # ダイレクト OVERTAKING 判定:
        # 1. 前方に車両が存在し、距離がアプローチ範囲内 (min_d < follow_start)
        # 2. 縦方向の接近速度が閾値以上 (min_rel_fwd >= overtake_speed_diff_thresh)
        # 3. 前車が自車前方を高速度で横切っていないこと (min_rel_lat <= cross_velocity_thresh)
        is_large_speed_gap = (is_leading_ahead and (min_d < follow_start) and
                              (min_rel_fwd >= overtake_speed_diff_thresh) and
                              (min_rel_lat <= cross_velocity_thresh))

        # ---- モード遷移ロジック ----

        # ダイレクト OVERTAKING: 大きな速度差（低速・スタック車への接近）時は FOLLOWING/BRAKE をバイパスして直接 OVERTAKING へ
        if is_large_speed_gap and self._v2x_mode in ("NORMAL", "FOLLOWING"):
            self._v2x_mode = "OVERTAKING"
            self._v2x_vehicle_radius = vehicle_radius_overtake
            self._v2x_speed_limit = float('inf')  # 減速せず最高速度を維持して回避
            self.get_logger().info(
                f"[V2X] Direct OVERTAKING (Speed Gap Detected): d={min_d:.1f}m rel_v={rel_speed_diff*3.6:.1f}km/h (ego_v={ego_speed_mps*3.6:.1f}km/h, lead_v={lead_speed*3.6:.1f}km/h)",
                throttle_duration_sec=1.0)

        # EMERGENCY_BRAKE: TTC 危険 or 距離が非常に近い
        elif min_ttc < ttc_thresh or (is_leading_ahead and min_d < follow_brake):
            self._v2x_mode = "EMERGENCY_BRAKE"
            self._v2x_speed_limit = v_min_safe_mps
            self.get_logger().warn(
                f"[V2X] EMERGENCY_BRAKE: d={min_d:.1f}m ttc={min_ttc:.1f}s",
                throttle_duration_sec=1.0)

        # NORMAL → FOLLOWING: 前方に車両が近づいてきた
        elif is_leading_ahead and min_d < follow_start and self._v2x_mode == "NORMAL":
            self._v2x_mode = "FOLLOWING"
            self._v2x_following_since = now_sec
            target_follow_speed = max(lead_speed, v_min_safe_mps)
            ratio = max(0.0, (min_d - follow_brake) / (follow_start - follow_brake))
            acc_speed = target_follow_speed + ratio * (v_max_normal - target_follow_speed)
            self._v2x_speed_limit = min(v_max_normal, acc_speed)
            self.get_logger().info(
                f"[V2X] FOLLOWING (ACC): d={min_d:.1f}m lead_v={lead_speed*3.6:.1f}km/h v_lim={self._v2x_speed_limit*3.6:.1f}km/h",
                throttle_duration_sec=1.0)

        # FOLLOWING: 速度制限を距離と前車速度に応じて動的に更新 (Adaptive Cruise Control)
        elif self._v2x_mode == "FOLLOWING":
            if not is_leading_ahead or min_d >= follow_start:
                # 前方車がいなくなった → NORMAL に戻る
                self._v2x_mode = "NORMAL"
                self._v2x_speed_limit = float('inf')
                self._v2x_following_since = None
                self.get_logger().info("[V2X] Back to NORMAL (vehicle left front zone)")
            else:
                # 継続 FOLLOWING: 前車速度に応じた適応型スロットル調整
                target_follow_speed = max(lead_speed, v_min_safe_mps)
                ratio = max(0.0, (min_d - follow_brake) / (follow_start - follow_brake))
                acc_speed = target_follow_speed + ratio * (v_max_normal - target_follow_speed)
                self._v2x_speed_limit = min(v_max_normal, acc_speed)

                # FOLLOWING → OVERTAKING 判断
                following_duration = now_sec - (self._v2x_following_since or now_sec)
                if (following_duration >= overtake_patience
                        and min_d <= overtake_gap_min):
                    self._v2x_mode = "OVERTAKING"
                    self._v2x_vehicle_radius = vehicle_radius_overtake
                    self._v2x_speed_limit = float('inf')  # 速度制限解除
                    self.get_logger().info(
                        f"[V2X] OVERTAKING: following={following_duration:.1f}s d={min_d:.1f}m")

        # OVERTAKING: 追い越し継続 or 完了チェック
        elif self._v2x_mode == "OVERTAKING":
            # 完了判定: 前方車が後方かつ十分な距離
            if not is_leading_ahead and min_d >= overtake_clearance:
                self._v2x_mode = "NORMAL"
                self._v2x_vehicle_radius = self._v2x_vehicle_radius_normal
                self._v2x_speed_limit = float('inf')
                self._v2x_following_since = None
                self.get_logger().info(
                    f"[V2X] Overtaking COMPLETE! d={min_d:.1f}m. Back to NORMAL.")
            else:
                # 追い越し中は速度制限しない
                self._v2x_speed_limit = float('inf')

        # NORMAL: 制限なし
        else:
            self._v2x_mode = "NORMAL"
            self._v2x_speed_limit = float('inf')
            self._v2x_following_since = None

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

    def _control(self):
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
            filtered_dynamic = self._filter_obstacles_to_corridor(self._dynamic_obstacles)
            active_obs = self._static_obstacles + filtered_dynamic

            # 有効な障害物が存在する場合のみマップ再構築を行い、障害物ゼロ時は不要なリセットを抑止して完全固定・平滑走行を維持
            if len(active_obs) > 0:
                self._map.reset_map()
                self._map.add_obstacles(active_obs)
                self._reference_path.reset_dynamic_constraints()
                self._had_obstacles = True
            elif self._had_obstacles:
                # 障害物がクリアされた瞬間のみ1回リセット
                self._map.reset_map()
                self._reference_path.reset_dynamic_constraints()
                self._had_obstacles = False

        is_colliding = False
        if self._last_colliding_time is not None:
            elapsed_from_last_colliding = (now - self._last_colliding_time).nanoseconds / 1e9
            if elapsed_from_last_colliding < 5.0:
                is_colliding = True

        pose = odom_to_pose_2d(self._odom) # type: ignore
        v = self._odom.twist.twist.linear.x

        # 車両状態は実オドメトリ位置で更新（コリドー境界・マップ参照のズレを防止）
        self._car.update_states(pose.x, pose.y, pose.theta)

        # 速度に応じたマイルドな動的先読みオフセット調整
        if self._mpc_cfg.enable_dynamic_delay_compensation:
            delay_sec = self._mpc_cfg.delay_compensation_sec
            resolution = self._reference_path.resolution
            dynamic_offset = int((v * delay_sec) / max(resolution, 1e-3))
            total_offset = max(1, self._mpc_cfg.wp_id_offset + dynamic_offset)
            self._mpc.update_wp_id_offset(total_offset)
        else:
            self._mpc.update_wp_id_offset(self._mpc_cfg.wp_id_offset)
        # print(f"car x: {self._car.temporal_state.x}, y: {self._car.temporal_state.y}, psi: {self._car.temporal_state.psi}")
        # print(f"mpc x: {self._mpc.model.temporal_state.x}, y: {self._mpc.model.temporal_state.y}, psi: {self._mpc.model.temporal_state.psi}")

        with self._stats.time_block("control"):
            # Skip MPC computation during STUCK RECOVERY: vehicle may be reversing (e_psi ~-140deg)
            # which would corrupt infeasibility_counter and current_control buffer with invalid state.
            if self._stuck_state == "NORMAL":
                u, max_delta = self._mpc.get_control()
                mpc_raw_steer = u[1]  # Save raw MPC output for diagnostics
                # initial_smooth_blend: Start-up protection preventing initial straight steering chatter
                if self._loop < 120:
                    blend_factor = min(1.0, max(0.0, (self._loop - 20) / 100.0))
                    u[1] = u[1] * blend_factor
            else:
                u, max_delta = np.array([0.0, 0.0]), 0.0
                mpc_raw_steer = 0.0
            # self.get_logger().info(f"u: {u}")

        # ベースの制限速度（通常速度プロファイルまたはリファレンス速度）を決定
        if self._ref_vel_configulator is not None:
            ref_vel_kmh = self._ref_vel_configulator.get_ref_vel(self._mpc.model.wp_id)
            base_v_max_mps = min(
                kmh_to_m_per_sec(ref_vel_kmh),
                self._mpc_cfg.v_max)
        else:
            base_v_max_mps = self._mpc_cfg.v_max

        # V2X障害物回避が有効かつ動的速度制限がベース速度を下回る場合、速度を制限する
        v_max_effective = base_v_max_mps
        if self.USE_OBSTACLE_AVOIDANCE and self._v2x_speed_limit < base_v_max_mps:
            v_max_effective = max(self._v2x_speed_limit, 0.0)
            self.get_logger().info(
                f"[V2X] Applying speed limit: mode={self._v2x_mode} v_lim={v_max_effective:.2f} m/s (base={base_v_max_mps:.2f} m/s)",
                throttle_duration_sec=1.0)

        # MPCとウェイポイントの目標速度を更新
        self._mpc.update_v_max(v_max_effective)
        v_ref: List[float] = [v_max_effective] * len(self._reference_path.waypoints)
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
            # continue

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
            acc =  self.KP * (u[0] - v)
            # print(f"v: {v}, u[0]: {u[0]}, acc: {acc}")
            acc = np.clip(acc, self._mpc_cfg.a_min, self._mpc_cfg.a_max)
        # u[0] = np.clip(last_u[0] + acc * dt, 0.0, self._mpc_cfg.v_max)

        # apply low pass filter to control signal
        steer_before_lpf = u[1]
        acc = self._last_acc + (acc - self._last_acc) * self._mpc_cfg.accel_low_pass_gain
        u[1] = self._last_u[1] + (u[1] - self._last_u[1]) * self._mpc_cfg.steer_low_pass_gain

        # [DIAG] Detailed steering pipeline trace for oscillation debugging
        e_y0 = self._mpc.model.spatial_state.e_y if self._mpc.model.spatial_state else 0.0
        e_psi0 = self._mpc.model.spatial_state.e_psi if self._mpc.model.spatial_state else 0.0
        print(f'[STEER_DIAG] wp={self._mpc.model.wp_id} e_y={e_y0:.4f} e_psi={e_psi0:.4f} '
              f'mpc_raw={mpc_raw_steer:.4f} pre_lpf={steer_before_lpf:.4f} post_lpf={u[1]:.4f} '
              f'prev_steer={self._mpc.previous_steering:.4f} v={v:.2f} acc={acc:.2f}')

        self._last_acc = acc
        self._last_u[0] = u[0]
        self._last_u[1] = u[1]

        # Sync actual steering command to MPC internal previous_steering
        # u[1] is tire angle delta (rad) after MPC.py L356 arctan conversion
        # MPC internally works in curvature kappa (1/m) space, so convert back: kappa = tan(delta) / length
        kappa_for_mpc = np.tan(u[1]) / self._car.length
        self._mpc.set_previous_steering(kappa_for_mpc)

        # --- Stuck Recovery Logic ---
        stuck_cfg = getattr(self._cfg, 'stuck_recovery', None)
        enable_stuck_rec = getattr(stuck_cfg, 'enable_stuck_recovery', True) if stuck_cfg else True

        if enable_stuck_rec and self._enable_control:
            stuck_vel_thresh = float(getattr(stuck_cfg, 'stuck_velocity_threshold', 0.15)) if stuck_cfg else 0.15
            stuck_time_thresh = float(getattr(stuck_cfg, 'stuck_time_threshold', 3.0)) if stuck_cfg else 3.0
            rev_speed = float(getattr(stuck_cfg, 'reverse_speed', -2.5)) if stuck_cfg else -2.5
            rev_duration = float(getattr(stuck_cfg, 'reverse_duration', 2.0)) if stuck_cfg else 2.0
            stop_duration = float(getattr(stuck_cfg, 'stop_duration', 0.4)) if stuck_cfg else 0.4

            now_sec = now.nanoseconds / 1e9

            # 発進完了判定: 一度でも車速 0.5 m/s を超えたら発進完了とみなす
            if abs(v) > 0.5:
                self._has_launched = True

            # 制御無効時または未発進（かつ非衝突）状態ではスタックタイマーをリセット
            if not self._enable_control or (not self._has_launched and not is_colliding):
                self._stuck_timer_start = None

            elif self._stuck_state == "NORMAL":
                # 壁衝突時: abs(v) <= 0.8 m/s かつ 0.5秒継続で即座にリカバリー発動
                # 非衝突時: 発進完了後 (self._has_launched) かつ前進指示中 (u[0] > 1.0) で abs(v) <= 0.05 m/s が 3.0秒継続で発動
                is_stuck_candidate = (
                    (is_colliding and abs(v) <= 0.8) or
                    (self._has_launched and abs(v) <= 0.05 and u[0] > 1.0)
                )

                req_time = 0.5 if is_colliding else stuck_time_thresh

                if is_stuck_candidate:
                    if self._stuck_timer_start is None:
                        self._stuck_timer_start = now_sec
                    elif (now_sec - self._stuck_timer_start) >= req_time:
                        self._stuck_state = "BRAKE_BEFORE_REVERSE"
                        self._stuck_phase_start = now_sec

                        # バック時回避操舵角の計算（前車の相対位置またはコース偏位に基づく切返し方向決定）
                        rev_steer_angle = float(getattr(stuck_cfg, 'reverse_steer_angle', 0.35)) if stuck_cfg else 0.35
                        lead_rel_y = 0.0
                        found_lead = False
                        if self._odom is not None and hasattr(self, '_v2x_tracker'):
                            pose = odom_to_pose_2d(self._odom)
                            ego_x, ego_y, ego_yaw = pose.x, pose.y, pose.theta
                            fwd_sin, fwd_cos = math.sin(ego_yaw), math.cos(ego_yaw)
                            left_cos, left_sin = -fwd_sin, fwd_cos

                            min_d = float('inf')
                            for vid in self._v2x_tracker.active_vehicle_ids():
                                if vid == self._vehicle_id:
                                    continue
                                buf = self._v2x_tracker._samples.get(vid)
                                if not buf:
                                    continue
                                _, ox, oy = buf[-1]
                                dx, dy = ox - ego_x, oy - ego_y
                                d = math.hypot(dx, dy)
                                if 0.5 <= d < 15.0 and d < min_d:
                                    lead_rel_y = dx * left_cos + dy * left_sin
                                    min_d = d
                                    found_lead = True

                        if not found_lead:
                            e_y_curr = self._mpc.model.spatial_state.e_y if self._mpc.model.spatial_state else 0.0
                            lead_rel_y = e_y_curr

                        # 前車が左側 (lead_rel_y > 0) -> バック時「左ステア (+rev_steer_angle)」でノーズを右へ振る
                        # 前車が右側 (lead_rel_y <= 0) -> バック時「右ステア (-rev_steer_angle)」でノーズを左へ振る
                        self._stuck_evasive_steer = rev_steer_angle if lead_rel_y > 0 else -rev_steer_angle

                        self.get_logger().warn(
                            f"[STUCK RECOVERY] Stuck detected! (v={v:.2f} m/s, colliding={is_colliding}, duration={now_sec - self._stuck_timer_start:.1f}s). "
                            f"Evasive reverse steer set to {self._stuck_evasive_steer:.2f} rad. Initiating reverse sequence...")
                else:
                    self._stuck_timer_start = None

            elif self._stuck_state == "BRAKE_BEFORE_REVERSE":
                elapsed = now_sec - (self._stuck_phase_start or now_sec)
                if elapsed < stop_duration:
                    u[0] = 0.0
                    acc = -3.0
                    u[1] = 0.0
                    bug_acc_enabled = False
                    self.get_logger().warn(f"[STUCK RECOVERY] Braking for reverse gear shift... ({elapsed:.1f}/{stop_duration:.1f}s)", throttle_duration_sec=0.3)
                else:
                    self._stuck_state = "REVERSING"
                    self._stuck_phase_start = now_sec
                    self.get_logger().warn(f"[STUCK RECOVERY] Shifted to REVERSE. Reversing with evasive steer {self._stuck_evasive_steer:.2f} rad...")

            elif self._stuck_state == "REVERSING":
                elapsed = now_sec - (self._stuck_phase_start or now_sec)
                if elapsed < rev_duration:
                    u[0] = abs(rev_speed)
                    acc = 1.5
                    u[1] = self._stuck_evasive_steer
                    bug_acc_enabled = False
                    self.get_logger().warn(f"[STUCK RECOVERY] Reversing evasively... v_cmd={u[0]:.1f} m/s, steer={u[1]:.2f} rad ({elapsed:.1f}/{rev_duration:.1f}s)", throttle_duration_sec=0.3)
                else:
                    self._stuck_state = "STOP_BEFORE_FORWARD"
                    self._stuck_phase_start = now_sec
                    self.get_logger().info("[STUCK RECOVERY] Reverse complete. Stopping before shifting forward...")

            elif self._stuck_state == "STOP_BEFORE_FORWARD":
                elapsed = now_sec - (self._stuck_phase_start or now_sec)
                if elapsed < stop_duration:
                    u[0] = 0.0
                    acc = -1.0
                    u[1] = 0.0
                    bug_acc_enabled = False
                else:
                    self._car.update_reference_path(self._car.reference_path)
                    self._stuck_state = "NORMAL"
                    self._stuck_timer_start = None
                    # Reset MPC internal state: during REVERSING, e_psi can be ~-140deg
                    # which corrupts infeasibility_counter and current_control buffer.
                    self._mpc.infeasibility_counter = 0
                    self._mpc.current_control = np.zeros(self._mpc.current_control.shape)
                    self._mpc.set_previous_steering(0.0)

                    # Post-recovery: Force V2X mode to OVERTAKING with reduced radius to pass the obstacle smoothly
                    v2x_cfg = getattr(self._cfg, 'v2x_obstacle_avoidance', None)
                    v_rad_ot = float(getattr(v2x_cfg, 'vehicle_radius_overtake', 0.65)) if v2x_cfg else 0.65
                    self._v2x_mode = "OVERTAKING"
                    self._v2x_vehicle_radius = v_rad_ot
                    self._v2x_speed_limit = float('inf')

                    self.get_logger().warn("[STUCK RECOVERY] Resuming forward MPC control in OVERTAKING mode! (MPC state reset & evasive radius active)")

        # update car state (use v for feedback actual speed)
        self._car.drive([v, u[1]])

        # Publish GearCommand (DRIVE = 2 / REVERSE = 20)
        target_gear = GearCommand.REVERSE if self._stuck_state in ["BRAKE_BEFORE_REVERSE", "REVERSING"] else GearCommand.DRIVE
        self._publish_gear_command(target_gear)

        # Publish control command
        self._publish_control_command(now, u, acc, bug_acc_enabled)

        # Log states
        self._sim_logger.log(self._car, u, t)
        self._sim_logger.plot_animation(t, self._loop, self._current_laps, self._lap_times, is_colliding, u, self._mpc, self._car)

        # 約 0.25 秒ごとに予測結果を表示
        if (self._mpc.current_prediction is not None) and (self._loop % (self._mpc_cfg.control_rate // 4) == 0):
            self._publish_mpc_pred_marker(self._mpc.current_prediction[0], self._mpc.current_prediction[1]) # type: ignore

    def run(self) -> None:
        self._wait_until_clock_received()
        self._wait_until_odom_received()
        self._wait_until_trajectory_received()
        self._wait_until_path_constraints_received()

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
