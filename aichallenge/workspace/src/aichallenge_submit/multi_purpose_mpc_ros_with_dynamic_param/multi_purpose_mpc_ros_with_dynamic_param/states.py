#!/usr/bin/env python3
"""Driving state definitions for the State pattern.

Each concrete state defines:
  - MPC parameters (v_max, ay_max, Q, R, QN, etc.)
  - Transition conditions to other states
  - Optional control override (e.g., Recovery bypasses MPC)

Phase 1: FollowPath + Recovery
Phase 2: Follow + Overtake
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Callable, Optional, List, Tuple
import numpy as np

class ControlMode(Enum):
    PURE_PURSUIT = auto()
    MPC = auto()
    WAYPOINT_SHIFT_PURE_PURSUIT = auto()
    OVERRIDE = auto()

try:
    from autoware_auto_vehicle_msgs.msg import GearCommand
    GEAR_DRIVE = GearCommand.DRIVE
    GEAR_REVERSE = GearCommand.REVERSE
except ImportError:
    GEAR_DRIVE = 2
    GEAR_REVERSE = 20

# ---------------------------------------------------------------------------
# Stuck detection constants (shared by all states)
# ---------------------------------------------------------------------------
STUCK_VELOCITY_THRESHOLD = 0.3  # [m/s] — below this is considered "stopped"
STUCK_DURATION = 3.0            # [s] — stopped 8s triggers recovery (prevents startup false-alarm)

# ---------------------------------------------------------------------------
# Forward-vehicle detection (shared by every state and by both detectors)
# ---------------------------------------------------------------------------
# follow_path と follow が別々の検知器・別々の視野角を見ていると、
# 「片方だけが見える」角度に他車がいるとき 1.5s ヒステリシス + 1.0s dwell の
# 周期で状態が往復し、そのたびに制御モードが Pure Pursuit ↔ MPC で切り替わって
# 大きくふらつく。両方の検知器がこの同じ値を使うこと。
FORWARD_CONE_DEG           = 45.0  # [deg] 前方検知角度
FORWARD_LATERAL_MAX        = 3.5   # [m] 前方検知の横方向ゲート
FORWARD_VEHICLE_DETECTION  = 10.0  # [m] 前方検知距離
SIDE_VEHICLE_ANGLE_MIN_DEG = 45.0  # [deg] 横最小検知角度
SIDE_VEHICLE_ANGLE_MAX_DEG = 90.0  # [deg] 横最大検知角度

# ---------------------------------------------------------------------------
# follow state parameter
# ---------------------------------------------------------------------------
D0_M                        = 1.0   # [m] 追従時の停止目標車間距離 (default: 1.5)
TIME_HEADWAY_SEC            = 0.35  # [s] 追従時に車間距離を縮める期待時間 (default: 0.35)
FORWARD_FOLLOW_DISTANCE_M   = 5.0   # [m] 追従を行う前方車両との車間距離 (default: 4.0)
FOLLOW_CLEAR_HYSTERESIS_SEC = 1.0   # [s] 追従状態を維持する最低時間 (チャタリング防止)
FOLLOW_STOP_DISTANCE_M      = 0.8   # [m] (完全停止・ブレーキ閾値、遅延を考慮)
FOLLOW_K_GAP                = 1.4   # [1/s] ギャップ誤差 → 速度
FOLLOW_K_V                  = 0.5   # [-] 相対速度ダンピング (default: 0.7)
FOLLOW_MIN_SPEED_KMH        = 10.0  # [km/h] 最低追従速度
FOLLOW_LEADER_MOVING_MPS    = 0.5   # [m/s] 0.5m/s = 1.8km/s
FOLLOW_TARGET_DISTANCE_M    = 3.0

# ---------------------------------------------------------------------------
# overtake state parameter
# ---------------------------------------------------------------------------
MIN_OVERTAKE_WIDTH_M      = 3.2     # [m] 最低追い越し幅 (default 2.5)
MIN_OVERTAKE_LEAD_SPEED   = 25.0    # [km/s] 前方車両の最低追い越し速度
OVERTAKE_CLOSING_MARGIN_M = 1.0     # [m] 追い越し時の車間距離の余裕距離
OVERTAKE_TTC_SEC          = 0.3     # [s] TTCの時間
OVERTAKE_PASSED_CLEARANCE_M = 1.7   # [m] 追い越し完了とみなす中心間の後方距離
                                    # 自車の後端が前方車両の前端を抜けるのに
                                    # 全長(VEHICLE_LENGTH) + ラインへ戻り始める
                                    # オフセットが必要 (最低1.6より大きい値)
OVERTAKE_PASSED_CLEARANCE_TIME_SEC = 0.35 # [s] 追い越し状態のクリア最大時間

# ---------------------------------------------------------------------------
# Vehicle configuration
# ---------------------------------------------------------------------------
# カート全長。bicycle_model.length (1.087) はホイールベースであって全長ではない。
# 中心間距離からバンパー間ギャップを出すために引く。
VEHICLE_LENGTH = 1.6   # [m] カート全長. bicycle_model.length (1.087) は
                       # ホイールベースであって全長ではない,横は1.45
VEHICLE_V_MAX  = 35.0  # [km/s] 最大速度


# ---------------------------------------------------------------------------
# Data transfer objects
# ---------------------------------------------------------------------------
@dataclass
class MPCStateParams:
    """MPC parameters that each state provides."""
    v_max: float          # [km/h]
    ay_max: float         # [m/s^2]
    Q: List[float]        # [e_y, e_psi, t] — 3 elements
    R: List[float]        # [v, delta] — 2 elements
    QN: List[float]       # [e_y, e_psi, t] — 3 elements
    lateral_offset: float = 0.0  # [m] for overtake maneuver


@dataclass
class StateContext:
    """Snapshot of sensor / vehicle data consumed by state transition logic."""

    # --- Time ---------------------------------------------------------------
    current_time_sec: float   # ROS clock time [s]
    dt: float                 # time since last control loop [s]

    # --- Vehicle state ------------------------------------------------------
    pose_x: float             # [m]
    pose_y: float             # [m]
    pose_theta: float         # yaw [rad]
    velocity: float           # longitudinal speed [m/s]

    # --- Collision ----------------------------------------------------------
    is_colliding: bool
    time_since_collision: Optional[float] = None  # [s], None if no collision

    # --- Path ---------------------------------------------------------------
    path_deviation: float = 0.0  # lateral distance from reference path [m]
    path_psi: float = 0.0        # closest waypoint orientation [rad]
    path_e_y: float = 0.0        # signed lateral offset from path centerline [m]

    # --- V2X (Phase 2) ------------------------------------------------------
    forward_vehicle_distance: Optional[float] = None # [m]
    forward_vehicle_gap: float = 0.0                 # [m] bumper-to-bumper (distance - VEHICLE_LENGTH)
    forward_vehicle_speed: Optional[float] = None    # [m/s]
    forward_vehicle_heading_diff: float = 0.0        # absolute heading diff relative to path_psi [rad]
    nearest_vehicle_s_rel: Optional[float] = None    # [m] signed; negative = behind ego

    # --- ReferencePath Clearance & Overtake ----------------------------------
    overtake_width_left: float = 0.0      # [m] 車両の左の外端から道路端（壁マージン考慮）までの空き幅
    overtake_width_right: float = 0.0     # [m] 車両の右の外端から道路端（壁マージン考慮）までの空き幅
    target_overtake_offset: float = 0.0   # dynamic lateral offset [m] for centerline of free space
    has_side_vehicle: bool = False        # True if another vehicle is alongside (-2.5m <= x_rel <= 2.5m)
    side_vehicle_speed: Optional[float] = None  # speed of side vehicle [m/s]

    # --- Detailed Multi-Vehicle Scan for FollowPath transitions ---
    has_forward_vehicle: bool = False          # - FORWARD_CONE_DEG / 2  <= angle <= FORWARD_CONE_DEG / 2, r <= VEHICLE_DETECT_DISTANCE
    min_forward_overtake_width: float = 0.0    # Minimum of max(left_w, right_w) among fwd vehicles [m]
    closest_forward_vehicle_speed: Optional[float] = None  # Speed of nearest fwd vehicle [m/s]
    has_left_side_vehicle: bool = False        # 30° <= angle <= 150°, 0 <= y <= 3m
    has_left_side_cutin_hazard: bool = False   # 3s projected: 30° <= angle <= 90°, 0 <= y <= 3m
    has_right_side_vehicle: bool = False       # -150° <= angle <= -30°, -3m <= y <= 0
    has_right_side_cutin_hazard: bool = False  # 3s projected: -90° <= angle <= -30°, -3m <= y <= 0

    # --- Stuck detection ----------------------------------------------------
    time_stopped_sec: float = 0.0  # duration velocity ≈ 0 [s]
    is_in_recovery_cooldown: bool = False  # True during cooldown after recovery / startup

    # --- Boost ----------------------------------------------------
    # boost送信コールバック (1.0: ON, 0.0: OFF)
    # boost使用時に以下をコメントアウト。
    # 現状は不安定もしくは効果が薄いのでコメントアウト
    publish_boost: Optional[Callable[[float], None]] = None

class DrivingState(ABC):
    """Base class for all driving states (State pattern)."""

    @property
    @abstractmethod
    def name(self) -> str:
        """Unique name identifying this state."""
        ...

    @property
    def gear(self) -> int:
        """Gear command for this state (GEAR_DRIVE or GEAR_REVERSE)."""
        return GEAR_DRIVE

    @property
    def control_mode(self) -> ControlMode:
        """Control mode required by this state (PURE_PURSUIT, MPC, or OVERRIDE)."""
        return ControlMode.MPC

    @abstractmethod
    def get_params(self) -> MPCStateParams:
        """Return the MPC parameters for this state."""
        ...

    @abstractmethod
    def check_transition(self, ctx: StateContext) -> Optional[str]:
        """Return the name of the next state, or ``None`` to stay."""
        ...

    def compute_control_override(
        self, ctx: StateContext
    ) -> Optional[Tuple[float, float, float]]:
        """Override MPC control output if this state needs direct actuation.

        Returns
        -------
        (speed [m/s], steer [rad], acceleration [m/s^2]) or ``None``
        to use normal MPC output.
        """
        return None

    def on_enter(self, ctx: StateContext) -> None:
        """Called once when entering this state."""
        pass

    def on_exit(self, ctx: StateContext) -> None:
        """Called once when leaving this state."""
        pass


class FollowPathState(DrivingState):
    """Normal path-following — no obstacles or vehicles ahead.

    Target speed: 35 km/h (~9.7 m/s)
    """

    # TODO: FollowPathではMPCを使用しないため要削除
    # ------ mpc parameter ------
    V_MAX = 35.0              # [km/h]
    AY_MAX = 9.5              # [m/s^2]
    Q = [1_000_000.0, 100_000_000.0, 850_000.0]
    R = [100_000.0, 100.0]  # R[1]=1000.0 を追加してステアリング微振動を抑止
    QN = [1_000_000.0, 1_000.0, 10_000.0]

    @property
    def name(self) -> str:
        return "follow_path"

    @property
    def control_mode(self) -> ControlMode:
        return ControlMode.PURE_PURSUIT

    def get_params(self) -> MPCStateParams:
        return MPCStateParams(
            v_max=self.V_MAX,
            ay_max=self.AY_MAX,
            Q=list(self.Q),
            R=list(self.R),
            QN=list(self.QN),
        )

    def check_transition(self, ctx: StateContext) -> Optional[str]:

        ###################################################
        # follow path -> recovery (always immediate)
        ###################################################

        if ctx.is_colliding:
            return "recovery"

        # Stuck detection: velocity near zero for too long → Recovery (unless in cooldown)
        if (not ctx.is_in_recovery_cooldown
                and ctx.time_stopped_sec >= STUCK_DURATION
                and ctx.velocity < STUCK_VELOCITY_THRESHOLD):
            return "recovery"


        ###################################################
        # follow path -> follow
        ###################################################

        # 前方に車両があり、追い越し幅が足りない → follow
        has_overtake_wide = ctx.min_forward_overtake_width > MIN_OVERTAKE_WIDTH_M
        if ctx.has_forward_vehicle and not has_overtake_wide:
            return "follow"


        ###################################################
        # follow path -> overtake
        ###################################################

        if (
            ctx.forward_vehicle_speed is not None
            and ctx.forward_vehicle_speed <= 25.0 / 3.6
            and ctx.velocity >= 29.0 / 3.6
        ):
            #-------------- version 1 --------------
            # 比較的に安定した条件も一時的にメモで残す
            #---------------------------------------
            # if (
            #     (
            #         (ctx.forward_vehicle_gap + VEHICLE_LENGTH * 1.5)
            #         / (ctx.velocity - ctx.forward_vehicle_speed)
            #     ) <= 2 # 1.5
            # ):
            #     return "overtake"
            #-------------- version 1 --------------


            #--------------------------------- version 2 ---------------------------------
            is_left = ctx.overtake_width_left > ctx.overtake_width_right
            is_same_lane = (ctx.path_e_y > 0 and is_left) or (ctx.path_e_y < 0 and not is_left)
            lead_speed = ctx.forward_vehicle_speed if ctx.forward_vehicle_speed is not None else 0.0
            is_overtake_gap = (
                (ctx.forward_vehicle_gap + VEHICLE_LENGTH * 1.5 + OVERTAKE_CLOSING_MARGIN_M)
                <= 0.4 * (ctx.velocity - lead_speed)
            )
            heading_diff = (
                ctx.forward_vehicle_heading_diff
                if ctx.forward_vehicle_heading_diff is not None
                else 0.0
            )
            has_future_width = False
            if ctx.overtake_width_left > ctx.overtake_width_right:
                if heading_diff > 0.0:
                    has_future_width = (
                        ctx.overtake_width_left - (
                            (0.5 * 0.4**2 + 0.4 * lead_speed) * np.sin(heading_diff)
                        ) >= MIN_OVERTAKE_WIDTH_M)
                elif heading_diff < 0.0:
                    has_future_width = (
                        ctx.overtake_width_left + (
                            (0.5 * 0.4**2 + 0.4 * lead_speed) * np.sin(heading_diff)
                        ) >= MIN_OVERTAKE_WIDTH_M)
            elif ctx.overtake_width_left < ctx.overtake_width_right:
                if heading_diff > 0.0:
                    has_future_width = (
                        ctx.overtake_width_right + (
                            (0.5 * 0.4**2 + 0.4 * lead_speed) * np.sin(heading_diff)
                        ) >= MIN_OVERTAKE_WIDTH_M)
                if heading_diff < 0.0:
                    has_future_width = (
                        ctx.overtake_width_right - (
                            (0.5 * 0.4**2 + 0.4 * lead_speed) * np.sin(heading_diff)
                        ) >= MIN_OVERTAKE_WIDTH_M)

            if (
                is_overtake_gap
                and ctx.min_forward_overtake_width >= MIN_OVERTAKE_WIDTH_M
                and has_future_width
                and is_same_lane
            ):
                return "overtake"
            #--------------------------------- version 2 ---------------------------------


        # Check Overtake condition
        #-------------------------- version 1 --------------------------
        # 比較的に安定した条件も一時的にメモで残す
        #---------------------------------------------------------------
        # 1. 前方 FORWARD_CONE_DEG 度の範囲かつ前方検知距離内に車両がある
        # ② 前方30度から150度、かつ、y方向0mから3m以内に他の車両がない
        # ③ 前方-30度から-150度、かつ、y方向0mから-3m以内に他の車両がない
        # ④ 前方の車両の左右のどちらかの道幅が MIN_OVERTAKE_WIDTH より大きい (複数車は最小道幅)
        # ⑤ 前の車両の速度が OVERTAKE_LEAD_SPEED_KMH 以下 かつ 接近中 (速度差 >= 1.47 m/s)
        # if (
        #     ctx.forward_vehicle_distance is not None
        #     and ctx.forward_vehicle_distance < FORWARD_VEHICLE_DETECTION
        # ):
        #     if ctx.forward_vehicle_distance <= FORWARD_FOLLOW_DISTANCE_M:
        #         # OVERTAKE_LEAD_SPEED_KMHちょうどだと、OVERTAKE_LEAD_SPEED_KMH前後で走る先行車が
        #         # 境界に乗り、V2X の2点差分速度のノイズで毎tick反転する。
        #         # マージン + OVERTAKE_LEAD_SPEED_KMH以上にすること。
        #         lead_speed = ctx.closest_forward_vehicle_speed if ctx.closest_forward_vehicle_speed is not None else 0.0
        #         is_slow_leader = (lead_speed <= (MIN_OVERTAKE_LEAD_SPEED / 3.6))
        #         speed_diff = ctx.velocity - lead_speed
        #         # 追従が整定すると speed_diff -> 0 になるので、接近判定だけでは
        #         # 原理的に追い越しに入れない（永遠に follow のまま）。
        #         # 「詰めきって遅い先行車の直後に張り付いている」状態も追い越し可とする。
        #         # 除算をやめたのは speed_diff < 0 のとき 2.2/負数 が符号で通ってしまい、
        #         # speed_diff == 0.0 では ZeroDivisionError になるため。
        #         speed_diff = ctx.velocity - lead_speed
        #         is_closing = speed_diff >= (
        #             (
        #                 ctx.forward_vehicle_gap + VEHICLE_LENGTH + OVERTAKE_CLOSING_MARGIN_M
        #             ) / OVERTAKE_TTC_SEC
        #         )
        #         is_settled_behind = (
        #             ctx.forward_vehicle_gap
        #             <= D0_M + TIME_HEADWAY_SEC * ctx.velocity + OVERTAKE_CLOSING_MARGIN_M)
        #         is_ttc_close = is_closing or is_settled_behind
        #         if (
        #             not ctx.has_left_side_vehicle
        #             and not ctx.has_right_side_vehicle
        #             and ctx.min_forward_overtake_width >= MIN_OVERTAKE_WIDTH_M
        #             and is_slow_leader
        #             and is_ttc_close
        #         ):
        #             return "overtake"
        #         return "follow"
        #-------------------------- version 1 --------------------------


        #-------------------------- version 2 --------------------------

        ###################################################
        #               /--> follow
        # follow path --
        #               \--> overtake
        ###################################################
        if (
            ctx.forward_vehicle_distance is not None
            and ctx.forward_vehicle_distance < FORWARD_VEHICLE_DETECTION
        ):
            if ctx.forward_vehicle_distance <= FORWARD_FOLLOW_DISTANCE_M:
                is_left = ctx.overtake_width_left > ctx.overtake_width_right
                is_same_lane = (ctx.path_e_y > 0 and is_left) or (ctx.path_e_y < 0 and not is_left)

                # 先行車両の速度
                lead_speed = (
                    ctx.closest_forward_vehicle_speed
                    if ctx.closest_forward_vehicle_speed is not None
                    else 0.0
                )

                # 先行車両の速度が MIN_OVERTAKE_LEAD_SPEED 以下かどうか
                is_slow_leader = (lead_speed <= (MIN_OVERTAKE_LEAD_SPEED / 3.6))

                # 先行車両との速度差
                speed_diff = ctx.velocity - lead_speed

                # (先行車両との車間距離 + 車体全長 + 車体半分長 + オフセット距離)[m]
                # の距離を 1.5s で移動できるだけの速度差があるかどうか
                is_closing = speed_diff >= (
                    (
                        ctx.forward_vehicle_gap + VEHICLE_LENGTH + OVERTAKE_CLOSING_MARGIN_M
                    ) / 0.75)

                # 車間距離が(停止車間距離 + 瞬間詰め距離 + オフセット距離)以下であるかどうか
                is_settled_behind = (
                    ctx.forward_vehicle_gap
                    <= D0_M + TIME_HEADWAY_SEC * ctx.velocity + OVERTAKE_CLOSING_MARGIN_M)

                is_ttc_close = is_closing or is_settled_behind

                # 横偏差の予測
                heading_diff = (
                    ctx.forward_vehicle_heading_diff
                    if ctx.forward_vehicle_heading_diff is not None
                    else 0.0
                )
                has_future_width = False
                if ctx.overtake_width_left > ctx.overtake_width_right:
                    if heading_diff > 0.0:
                        has_future_width = (
                            ctx.overtake_width_left - (
                                (0.5 * 0.4**2 + 0.4 * lead_speed) * np.sin(heading_diff)
                            ) >= MIN_OVERTAKE_WIDTH_M)
                    elif heading_diff < 0.0:
                        has_future_width = (
                            ctx.overtake_width_left + (
                                (0.5 * 0.4**2 + 0.4 * lead_speed) * np.sin(heading_diff)
                            ) >= MIN_OVERTAKE_WIDTH_M)
                elif ctx.overtake_width_left < ctx.overtake_width_right:
                    if heading_diff > 0.0:
                        has_future_width = (
                            ctx.overtake_width_right + (
                                (0.5 * 0.4**2 + 0.4 * lead_speed) * np.sin(heading_diff)
                            ) >= MIN_OVERTAKE_WIDTH_M)
                    if heading_diff < 0.0:
                        has_future_width = (
                            ctx.overtake_width_right - (
                                (0.5 *0.4**2 + 0.4 * lead_speed) * np.sin(heading_diff)
                            ) >= MIN_OVERTAKE_WIDTH_M)

                if (
                    not ctx.has_left_side_vehicle
                    and not ctx.has_right_side_vehicle
                    and ctx.min_forward_overtake_width >= MIN_OVERTAKE_WIDTH_M
                    and is_slow_leader
                    and is_ttc_close
                    and has_future_width
                    and is_same_lane
                ):
                    return "overtake"
                return "follow"
        #-------------------------- version 2 --------------------------

        return None


class RecoveryState(DrivingState):
    """3-Phase Recovery: wait (0.5s) -> directional back (3.0s) -> directional forward turn (2.0s) -> follow_path.

    Sequence
    --------
    1. **wait** (``WAIT_DURATION`` = 0.5s): full stop, records collision side (left ey>=0 or right ey<0).
    2. **back** (``BACK_DURATION`` = 3.0s): reverse (gear=REVERSE) with directional steering:
       - Left collision (left on track): steer LEFT (+0.55 rad) while reversing.
       - Right collision (right on track): steer RIGHT (-0.55 rad) while reversing.
    3. **forward_turn** (``FORWARD_TURN_DURATION`` = 2.0s): forward drive (gear=DRIVE) with directional steering:
       - Left collision (left on track): steer RIGHT (-0.55 rad) to point nose towards track right.
       - Right collision (right on track): steer LEFT (+0.55 rad) to point nose towards track left.
    4. Transition to ``follow_path``.
    """

    WAIT_DURATION_TIME_SEC = 0.5     # [s] 停止待機時間
    BACK_DURATION_TIME_SEC = 1.0     # [s] 最大後退時間
    FORWARD_DURATION_TIME_SEC = 1.0  # [s] 最大前進時間

    RECOVERY_FORWARD_TURN_SPEED_MPS = 7.0   # [m/s] 前進旋回速度
    RECOVERY_BACK_TURN_SPEED_MPS    = -7.0  # [m/s] 後退旋回速度

    RECOVERY_FORWARD_ACCEL_MPSS = 6.0  # [m/s^2] 前進旋回加速度
    RECOVERY_BACK_ACCEL_MPSS = 6.0     # [m/s^2] 後退旋回加速度

    RECOVERY_STEER_LOCK_RAD = 0.55     # [rad] 大舵角ステアリング角度

    def __init__(self) -> None:
        self._phase: str = "wait"
        self._enter_time: Optional[float] = None  # "wait" | "back" | "forward_turn"
        self._collision_side: str = "left"        # "left" | "right"

    @property
    def name(self) -> str:
        return "recovery"

    @property
    def control_mode(self) -> ControlMode:
        return ControlMode.OVERRIDE

    @property
    def gear(self) -> int:
        if self._phase == "back":
            return GEAR_REVERSE
        return GEAR_DRIVE

    def get_params(self) -> MPCStateParams:
        # Use a slow / conservative preset while recovering
        return MPCStateParams(
            v_max=18.0,  # 5 m/s = 18 km/h
            ay_max=3.0,
            Q=[5_000_000.0, 100_000_000.0, 200_000.0],
            R=[100_000.0, 0.0],
            QN=[1_000_000.0, 1_000.0, 10_000.0],
        )

    def on_enter(self, ctx: StateContext) -> None:
        self._enter_time = ctx.current_time_sec
        self._phase = "wait"
        # 衝突時の軌道横偏差左側: 左側(ey >= 0)か右側(ey < 0)かを記録
        self._collision_side = "left" if ctx.path_e_y >= 0 else "right"

    def on_exit(self, ctx: StateContext) -> None:
        self._enter_time = None
        self._phase = "wait"

    def check_transition(self, ctx: StateContext) -> Optional[str]:
        if self._enter_time is None:
            return None

        elapsed = ctx.current_time_sec - self._enter_time

        if self._phase == "wait":
            if elapsed >= self.WAIT_DURATION_TIME_SEC:
                self._phase = "back"
            return None  # stay in recovery while waiting

        if self._phase == "back":
            if elapsed >= (self.WAIT_DURATION_TIME_SEC + self.BACK_DURATION_TIME_SEC):
                self._phase = "forward_turn"
            return None


        ###################################################
        # recovery -> follow path
        ###################################################

        total_recovery_duration = (
            self.WAIT_DURATION_TIME_SEC + self.BACK_DURATION_TIME_SEC + self.FORWARD_DURATION_TIME_SEC
        )
        if elapsed >= total_recovery_duration:
            return "follow_path"

        return None

    def compute_control_override(
        self, ctx: StateContext
    ) -> Optional[Tuple[float, float, float]]:
        if self._phase == "wait":
            return (0.0, 0.0, 0.0)  # full stop

        if self._phase == "back":
            # 1. 軌道の左側にいるとき: ステアリングを左に切ってバック (+0.55 rad)
            # 2. 軌道の右側にいるとき: ステアリングを右に切ってバック (-0.55 rad)
            steer_cmd = self.RECOVERY_STEER_LOCK_RAD if self._collision_side == "left" else -self.RECOVERY_STEER_LOCK_RAD
            return (self.RECOVERY_BACK_TURN_SPEED_MPS, steer_cmd, self.RECOVERY_BACK_ACCEL_MPSS)

        # phase == "forward_turn" (前進旋回)
        # 1. 軌道の左側にいたとき: 姿勢が右側に向いて前進 (-0.55 rad)
        # 2. 軌道の右側にいたとき: 姿勢が左側に向いて前進 (+0.55 rad)
        steer_cmd = -self.RECOVERY_STEER_LOCK_RAD if self._collision_side == "left" else self.RECOVERY_STEER_LOCK_RAD
        return (self.RECOVERY_FORWARD_TURN_SPEED_MPS, steer_cmd, self.RECOVERY_FORWARD_ACCEL_MPSS)


class FollowState(DrivingState):
    """Follow a leading vehicle — maintain safe distance, match speed."""

    # ---- MPC parameters (same cornering capability as FollowPathState) ------
    _V_MAX_DEFAULT = 35.0   # [km/h] — ceiling, actual v_max is dynamic
    AY_MAX = 6.5
    Q  = [2_000_000.0, 100_000_000.0, 100_000.0]
    R  = [100_000.0, 100_000_000.0]
    QN = [2_000_000.0, 10_000.0, 3_000.0]

    def __init__(self) -> None:
        self._clear_start_time: Optional[float] = None

    @property
    def name(self) -> str:
        return "follow"

    @property
    def control_mode(self) -> ControlMode:
        return ControlMode.WAYPOINT_SHIFT_PURE_PURSUIT

    def get_params(self) -> MPCStateParams:
        return MPCStateParams(
            v_max=self._V_MAX_DEFAULT,
            ay_max=self.AY_MAX,
            Q=list(self.Q),
            R=list(self.R),
            QN=list(self.QN),
        )

    def check_transition(self, ctx: StateContext) -> Optional[str]:

        ##########################################
        # follow -> recovery
        ##########################################

        if ctx.is_colliding:
            return "recovery"


        ##########################################
        # follow -> follow_path
        ##########################################

        has_v2x_leader = (
            ctx.forward_vehicle_distance is not None
            and ctx.forward_vehicle_distance < FORWARD_VEHICLE_DETECTION
        )
        if not (has_v2x_leader or ctx.has_side_vehicle):

            if self._clear_start_time is None:
                self._clear_start_time = ctx.current_time_sec

            elapsed_clear = ctx.current_time_sec - self._clear_start_time
            if elapsed_clear >= FOLLOW_CLEAR_HYSTERESIS_SEC:
                self._clear_start_time = None
                return "follow_path"
        else:
            self._clear_start_time = None  # Instantly reset timer if vehicle is detected


        ##########################################
        # follow -> overtake
        ##########################################

        #--------------------------------- version 1 ---------------------------------
        # 比較的に安定した条件も一時的にメモで残す
        #-----------------------------------------------------------------------------
        # lead_speed = ctx.closest_forward_vehicle_speed if ctx.closest_forward_vehicle_speed is not None else 0.0
        # speed_diff = ctx.velocity - lead_speed
        # max_side = max(ctx.overtake_width_left, ctx.overtake_width_right)
        # has_clearance = max_side >= MIN_OVERTAKE_WIDTH_M
        # is_slow_leader = (lead_speed <= (MIN_OVERTAKE_LEAD_SPEED / 3.6))
        # is_closing = speed_diff >= (
        #     (
        #         ctx.forward_vehicle_gap + VEHICLE_LENGTH + OVERTAKE_CLOSING_MARGIN_M
        #     ) / OVERTAKE_TTC_SEC
        # )
        # is_settled_behind = (
        #     ctx.forward_vehicle_gap
        #     <= D0_M + TIME_HEADWAY_SEC * ctx.velocity + OVERTAKE_CLOSING_MARGIN_M)
        # is_ttc_close = is_closing or is_settled_behind
        # if (
        #     not ctx.has_left_side_vehicle
        #     and not ctx.has_right_side_vehicle
        #     and ctx.min_forward_overtake_width >= MIN_OVERTAKE_WIDTH_M
        #     and is_slow_leader
        #     and is_ttc_close
        # ):
        #     return "overtake"
        #--------------------------------- version 1 ---------------------------------

        #--------------------------------- version 2 ---------------------------------
        is_left = ctx.overtake_width_left > ctx.overtake_width_right
        is_same_lane = (ctx.path_e_y > 0 and is_left) or (ctx.path_e_y < 0 and not is_left)
        has_long_gap = ctx.forward_vehicle_distance > 3.5 if ctx.forward_vehicle_distance is not None else False

        # 先行車両の速度
        lead_speed = ctx.closest_forward_vehicle_speed if ctx.closest_forward_vehicle_speed is not None else 0.0

        # 先行車両との速度差
        speed_diff = ctx.velocity - lead_speed

        # 先行車両の左右の幅が最低追い越し幅以上あるかどうか
        max_side = max(ctx.overtake_width_left, ctx.overtake_width_right)
        has_clearance = max_side >= MIN_OVERTAKE_WIDTH_M

        # 先行車両の速度が MIN_OVERTAKE_LEAD_SPEED 以下かどうか
        is_slow_leader = (lead_speed <= (MIN_OVERTAKE_LEAD_SPEED / 3.6))

        # (先行車両との車間距離 + 車体全長 + 車体半分長 + オフセット距離)[m]
        # の距離を 1.5s で移動できるだけの速度差があるかどうか
        is_closing = speed_diff >= (
            (
                ctx.forward_vehicle_gap + VEHICLE_LENGTH + OVERTAKE_CLOSING_MARGIN_M
            ) / 0.5
        )

        # 車間距離が(停止車間距離 + 瞬間詰め距離 + オフセット距離)以下であるかどうか
        is_settled_behind = (
            ctx.forward_vehicle_gap
            <= D0_M + TIME_HEADWAY_SEC * ctx.velocity + OVERTAKE_CLOSING_MARGIN_M)

        is_ttc_close = is_closing or is_settled_behind

        # 2s後の先行車両の左右の幅が、最低追い越し幅以上あるかどうか
        heading_diff = (
            ctx.forward_vehicle_heading_diff
            if ctx.forward_vehicle_heading_diff is not None
            else 0.0
        )
        has_future_width = False
        if ctx.overtake_width_left > ctx.overtake_width_right:
            if heading_diff > 0.0:
                has_future_width = (
                    ctx.overtake_width_left - (
                        (0.5 * 0.75**2 + 0.75 * lead_speed) * np.sin(heading_diff)
                    ) >= MIN_OVERTAKE_WIDTH_M)
            elif heading_diff < 0.0:
                has_future_width = (
                    ctx.overtake_width_left + (
                        (0.5 * 0.75**2 + 0.75 * lead_speed) * np.sin(heading_diff)
                    ) >= MIN_OVERTAKE_WIDTH_M)
        elif ctx.overtake_width_left < ctx.overtake_width_right:
            if heading_diff > 0.0:
                has_future_width = (
                    ctx.overtake_width_right + (
                        (0.5 * 0.75**2 + 0.75 * lead_speed) * np.sin(heading_diff)
                    ) >= MIN_OVERTAKE_WIDTH_M)
            if heading_diff < 0.0:
                has_future_width = (
                    ctx.overtake_width_right - (
                        (0.5 * 0.75**2 + 0.75 * lead_speed) * np.sin(heading_diff)
                    ) >= MIN_OVERTAKE_WIDTH_M)

        if (
            not ctx.has_left_side_vehicle
            and not ctx.has_right_side_vehicle
            and ctx.min_forward_overtake_width >= MIN_OVERTAKE_WIDTH_M
            and is_slow_leader
            and is_ttc_close
            and has_future_width
            and is_same_lane
            and has_long_gap
        ):
            return "overtake"
        #--------------------------------- version 2 ---------------------------------


        ##########################################
        # follow -> recovery
        ##########################################

        # Stuck detection: ONLY trigger Recovery if NOT intentionally waiting behind a leader/obstacle
        is_waiting_behind_leader = (
            ctx.forward_vehicle_distance is not None
            and ctx.forward_vehicle_distance < FORWARD_VEHICLE_DETECTION
            and not has_clearance
        )
        if not is_waiting_behind_leader:
            if (not ctx.is_in_recovery_cooldown
                    and ctx.time_stopped_sec >= STUCK_DURATION
                    and ctx.velocity < STUCK_VELOCITY_THRESHOLD):
                return "recovery"

        return None

    def get_adjusted_v_max_kmh(self, ctx: StateContext) -> float:
        """Compute dynamic v_max [km/h] with strict distance governor and side-vehicle yielding."""
        # If a side-vehicle is alongside and clearance is insufficient, yield by reducing speed
        max_side = max(ctx.overtake_width_left, ctx.overtake_width_right)
        if ctx.has_side_vehicle and (
            ctx.forward_vehicle_distance is not None
            and ctx.forward_vehicle_distance < FOLLOW_STOP_DISTANCE_M
        ):
            side_speed = ctx.side_vehicle_speed if ctx.side_vehicle_speed is not None else 0.0
            return float(np.clip(side_speed / 2.0, 0.0, (VEHICLE_V_MAX / 3.6)))
        elif ctx.has_side_vehicle and ctx.forward_vehicle_distance is None:
            return VEHICLE_V_MAX / 3.6
            # side_speed = ctx.side_vehicle_speed * 3.6 if ctx.side_vehicle_speed is not None else VEHICLE_V_MAX
            # return float(np.clip(VEHICLE_V_MAX, 0.0, (VEHICLE_V_MAX)))

        if ctx.forward_vehicle_distance is None:
            return VEHICLE_V_MAX

        v_ego = ctx.velocity
        d_des = D0_M + TIME_HEADWAY_SEC * v_ego # 一定時間の停止距離を含めた車間制御
        # v_cmd = ctx.forward_vehicle_speed + FOLLOW_K_GAP * (ctx.forward_vehicle_gap - d_des) + FOLLOW_K_V * (ctx.forward_vehicle_speed - v_ego)
        v_cmd = ctx.forward_vehicle_speed + FOLLOW_K_V * (ctx.forward_vehicle_speed - v_ego)
        # if ctx.forward_vehicle_speed > FOLLOW_LEADER_MOVING_MPS:
        #     v_cmd = max(v_cmd, FOLLOW_MIN_SPEED_KMH)

        return float(np.clip(v_cmd, 0.0, (VEHICLE_V_MAX / 3.6)))


class OvertakeState(DrivingState):
    """Overtake a slower vehicle by inducing a lateral offset."""

    # ---- hardcoded parameters (35 km/h) ------------------------------------
    V_MAX = 35.0              # [km/h]
    AY_MAX = 9.5
    Q = [8_000_000.0, 20_000_000.0, 300_000.0]
    R = [30_000.0, 0.0]
    QN = [4_000_000.0, 1_000.0, 10_000.0]

    USE_MPC_OVERTAKE: bool = False  # False: Waypoint-Shift Pure Pursuit (Default), True: MPC

    def __init__(self) -> None:
        self._overtake_side: str = "left"  # "left" or "right"
        self._enter_time: Optional[float] = None
        self._calculated_offset: float = 1.8
        self._was_alongside: bool = False
        self._passed: bool = False

    @property
    def name(self) -> str:
        return "overtake"

    @property
    def control_mode(self) -> ControlMode:
        """Control mode: Waypoint-shifted Pure Pursuit by default, switchable to MPC."""
        return ControlMode.MPC if self.USE_MPC_OVERTAKE else ControlMode.WAYPOINT_SHIFT_PURE_PURSUIT

    def get_params(self) -> MPCStateParams:
        return MPCStateParams(
            v_max=self.V_MAX,
            ay_max=self.AY_MAX,
            Q=list(self.Q),
            R=list(self.R),
            QN=list(self.QN),
            lateral_offset=self._calculated_offset,
        )

    def on_enter(self, ctx: StateContext) -> None:
        self._enter_time = ctx.current_time_sec
        self._was_alongside = False
        self._passed = False

        # boost開始 (1.0 をパブリッシュ)
        # boost使用時に以下をコメントアウト。
        #----------------------------------
        if ctx.publish_boost is not None:
            ctx.publish_boost(1.5)

        # ReferencePath から計算された空き領域の真ん中を通る動的オフセットを採用
        if abs(ctx.target_overtake_offset) > 0.1:
            self._calculated_offset = ctx.target_overtake_offset
            self._overtake_side = "left" if ctx.target_overtake_offset > 0 else "right"
        elif ctx.overtake_width_left >= ctx.overtake_width_right:
            self._overtake_side = "left"
            half_w = ctx.overtake_width_left / 2.0
            self._calculated_offset = float(np.clip(half_w, 1.2, 2.2))
        else:
            self._overtake_side = "right"
            half_w = ctx.overtake_width_right / 2.0
            self._calculated_offset = -float(np.clip(half_w, 1.2, 2.2))

    def on_exit(self, ctx: StateContext) -> None:
        self._enter_time = None
        self._was_alongside = False
        self._passed = False

        # boost解除 (0.0 をパブリッシュ)
        # boost使用時に以下をコメントアウト。
        #----------------------------------
        if ctx.publish_boost is not None:
            ctx.publish_boost(0.0)

    def _update_passed(self, ctx: StateContext) -> bool:
        """前車が自車より後ろに出たか。一度 True になったらラッチする。"""
        if self._passed:
            return True

        if ctx.has_side_vehicle:
            self._was_alongside = True

        # まだ前方に車がいる間は完了ではない。このガードがないと、自車を追ってくる
        # 別の車が nearest_vehicle_s_rel の最小値を取ってしまい、進入直後に
        # 「もう抜けた」と誤判定する。
        if ctx.forward_vehicle_distance is not None:
            return False

        s_rel = ctx.nearest_vehicle_s_rel
        if s_rel is not None and s_rel <= -OVERTAKE_PASSED_CLEARANCE_M:
            self._passed = True      # 実測: 相手の中心が自車の PASSED_CLEARANCE 後方
        elif s_rel is None and self._was_alongside:
            self._passed = True      # 真横に並んだ後に検知範囲から消えた（ロスト対策）
        return self._passed

    def check_transition(self, ctx: StateContext) -> Optional[str]:

        ###################################################
        # overtake -> recovery
        ###################################################

        # 1. 衝突検知
        if ctx.is_colliding:
            return "recovery"

        # 2. スタック検知
        if (not ctx.is_in_recovery_cooldown
                and ctx.time_stopped_sec >= STUCK_DURATION
                and ctx.velocity < STUCK_VELOCITY_THRESHOLD):
            return "recovery"


        ###################################################
        # overtake -> follow
        ###################################################

        # 3. 追い越し中の即時安全中断（先行車が寄せてきて道幅が狭くなった場合は即座に Follow に戻る）
        has_v2x_leader = (
            ctx.forward_vehicle_distance is not None
            and ctx.forward_vehicle_distance < FORWARD_VEHICLE_DETECTION
        )
        if has_v2x_leader:
            max_side = max(ctx.overtake_width_left, ctx.overtake_width_right)
            if max_side < MIN_OVERTAKE_WIDTH_M:
                return "follow"


        ###################################################
        # overtake -> follow path
        ###################################################

        # 4. 追い越し完了判定またはタイムアウト判定（相手が後方に抜けたか、最大存続時間が経過した場合は FollowPath に復帰）
        passed = self._update_passed(ctx)
        # timed_out = (
        #     self._enter_time is not None
        #     and (ctx.current_time_sec - self._enter_time) >= OVERTAKE_PASSED_CLEARANCE_TIME_SEC
        # )
        if passed:
            return "follow_path"

        # 5. 並走中の保護（真横に並んでいる間は相手側面への切り込みを防ぐため Overtake を維持）
        if ctx.has_side_vehicle:
            return None

        # 6. 前方・側方に障害車両が検知されない場合の安全復帰
        if not has_v2x_leader and not ctx.has_side_vehicle:
            return "follow_path"

        return None
