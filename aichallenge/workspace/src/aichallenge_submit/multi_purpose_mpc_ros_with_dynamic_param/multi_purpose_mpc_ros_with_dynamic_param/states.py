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
from typing import Optional, List, Tuple
import numpy as np

class ControlMode(Enum):
    PURE_PURSUIT = auto()
    MPC = auto()
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
STUCK_DURATION = 2.0            # [s] — stopped 8s triggers recovery (prevents startup false-alarm)


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
    forward_vehicle_distance: Optional[float] = None   # [m]
    forward_vehicle_speed: Optional[float] = None       # [m/s]
    forward_vehicle_heading_diff: float = 0.0          # absolute heading diff relative to path_psi [rad]

    # --- ReferencePath Clearance & Overtake ----------------------------------
    overtake_width_left: float = 0.0      # available width on left [m]
    overtake_width_right: float = 0.0     # available width on right [m]
    target_overtake_offset: float = 0.0   # dynamic lateral offset [m] for centerline of free space
    has_side_vehicle: bool = False        # True if another vehicle is alongside (-2.5m <= x_rel <= 2.5m)
    side_vehicle_speed: Optional[float] = None  # speed of side vehicle [m/s]
    lidar_forward_clearance: Optional[float] = None   # retained for compatibility
    lidar_range_clearance: Optional[float] = None

    # --- Stuck detection ----------------------------------------------------
    time_stopped_sec: float = 0.0  # duration velocity ≈ 0 [s]
    is_in_recovery_cooldown: bool = False  # True during cooldown after recovery / startup


def _get_effective_forward_distance(ctx: StateContext) -> Optional[float]:
    """Get the forward vehicle distance from V2X tracker."""
    return ctx.forward_vehicle_distance


# ---------------------------------------------------------------------------
# Abstract base
# ---------------------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# Phase 1 states
# ---------------------------------------------------------------------------

class FollowPathState(DrivingState):
    """Normal path-following — no obstacles or vehicles ahead.

    Target speed: 35 km/h (~9.7 m/s)
    """

    # ---- hardcoded parameters -----------------------------------------------
    V_MAX = 35.0              # [km/h]
    AY_MAX = 9.5              # [m/s^2]
    Q = [1_000_000.0, 100_000_000.0, 850_000.0]
    R = [100_000.0, 100.0]  # R[1]=1000.0 を追加してステアリング微振動を抑止
    QN = [1_000_000.0, 1_000.0, 10_000.0]

    # ---- forward-vehicle detection thresholds -------------------------------
    VEHICLE_DETECT_DISTANCE = 8.0   # [m]
    MIN_OVERTAKE_WIDTH = 1.4         # minimum available width to execute overtake [m]

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
        # Collision → Recovery (always immediate)
        if ctx.is_colliding:
            return "recovery"

        # Stuck detection: velocity near zero for too long → Recovery (unless in cooldown)
        if (not ctx.is_in_recovery_cooldown
                and ctx.time_stopped_sec >= STUCK_DURATION
                and ctx.velocity < STUCK_VELOCITY_THRESHOLD):
            return "recovery"

        # Phase 2: forward / side vehicle detection (V2X)
        eff_dist = _get_effective_forward_distance(ctx)
        has_forward_vehicle = (eff_dist is not None and eff_dist < self.VEHICLE_DETECT_DISTANCE)
        has_any_obstacle = has_forward_vehicle or ctx.has_side_vehicle

        if has_any_obstacle:
            max_side = max(ctx.overtake_width_left, ctx.overtake_width_right)
            has_clearance = max_side >= self.MIN_OVERTAKE_WIDTH
            # is_zero_speed = (ctx.forward_vehicle_speed is not None and ctx.forward_vehicle_speed < 0.3)
            # is_not_aligned = (ctx.forward_vehicle_heading_diff > np.deg2rad(45.0))
            is_slower_leader = (
                ctx.forward_vehicle_speed is None #
                or (
                    ctx.forward_vehicle_speed < 2.77 # 10 km/h = 2.77 m/s
                        # and ctx.velocity - ctx.forward_vehicle_speed >= 4.17 # 4.17 m/s = 15 km/h speed difference threshold for overtaking
                )
            )
            # is_aligned = (ctx.forward_vehicle_heading_diff <= np.deg2rad(45.0))
            is_clear_side = not ctx.has_side_vehicle

            # (1) Overtake if clearance exists AND (leader is stopped OR leader heading is NOT aligned OR side-by-side)
            # (2) Follow if no clearance OR leader is aligned and moving
            # if has_clearance and (is_zero_speed or is_not_aligned or ctx.has_side_vehicle):
            if has_clearance and is_slower_leader and is_clear_side:
                return "overtake"
            else:
                return "follow"

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

    WAIT_DURATION = 0.5          # [s] (停止待機)
    BACK_DURATION = 2.0          # [s] (最大2秒バック)
    FORWARD_TURN_DURATION = 2.0  # [s] (最大2秒前進)

    BACK_SPEED = -4.0            # [m/s] (後退速度)
    FORWARD_TURN_SPEED = 3.5     # [m/s] (前進旋回速度)
    BACK_ACCEL = 3.5             # [m/s^2]
    FORWARD_ACCEL = 3.0          # [m/s^2]

    STEER_LOCK = 0.55            # [rad] (大舵角ステアリング)

    def __init__(self) -> None:
        self._enter_time: Optional[float] = None
        self._phase: str = "wait"          # "wait" | "back" | "forward_turn"
        self._collision_side: str = "left"  # "left" | "right"

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

    # -- lifecycle -----------------------------------------------------------

    def on_enter(self, ctx: StateContext) -> None:
        self._enter_time = ctx.current_time_sec
        self._phase = "wait"
        # 衝突時の軌道横偏差: 左側(ey >= 0)か右側(ey < 0)かを記録
        self._collision_side = "left" if ctx.path_e_y >= 0 else "right"

    def on_exit(self, ctx: StateContext) -> None:
        self._enter_time = None
        self._phase = "wait"

    # -- transitions ---------------------------------------------------------

    def check_transition(self, ctx: StateContext) -> Optional[str]:
        if self._enter_time is None:
            return None

        elapsed = ctx.current_time_sec - self._enter_time

        if self._phase == "wait":
            if elapsed >= self.WAIT_DURATION:
                self._phase = "back"
            return None  # stay in recovery while waiting

        if self._phase == "back":
            if elapsed >= (self.WAIT_DURATION + self.BACK_DURATION):
                self._phase = "forward_turn"
            return None

        # phase == "forward_turn"
        total_recovery_duration = self.WAIT_DURATION + self.BACK_DURATION + self.FORWARD_TURN_DURATION
        if elapsed >= total_recovery_duration:
            return "follow_path"

        return None

    # -- control override ----------------------------------------------------

    def compute_control_override(
        self, ctx: StateContext
    ) -> Optional[Tuple[float, float, float]]:
        if self._phase == "wait":
            return (0.0, 0.0, 0.0)  # full stop

        if self._phase == "back":
            # 1. 軌道の左側にいるとき: ステアリングを左に切ってバック (+0.55 rad)
            # 2. 軌道の右側にいるとき: ステアリングを右に切ってバック (-0.55 rad)
            steer_cmd = self.STEER_LOCK if self._collision_side == "left" else -self.STEER_LOCK
            return (self.BACK_SPEED, steer_cmd, self.BACK_ACCEL)

        # phase == "forward_turn" (前進旋回)
        # 1. 軌道の左側にいたとき: 姿勢が右側に向いて前進 (-0.55 rad)
        # 2. 軌道の右側にいたとき: 姿勢が左側に向いて前進 (+0.55 rad)
        steer_cmd = -self.STEER_LOCK if self._collision_side == "left" else self.STEER_LOCK
        return (self.FORWARD_TURN_SPEED, steer_cmd, self.FORWARD_ACCEL)


# ---------------------------------------------------------------------------
# Phase 2 states
# ---------------------------------------------------------------------------

class FollowState(DrivingState):
    """Follow a leading vehicle — maintain safe distance (8m), match speed."""

    TARGET_FOLLOWING_DISTANCE = 3.0   # [m] (相対距離保持目標)
    STOP_DISTANCE = 1.0               # [m] (完全停止・ブレーキ閾値、遅延を考慮)
    FOLLOWING_KP = 1.1                # speed adjustment gain

    # ---- MPC parameters (same cornering capability as FollowPathState) ------
    _V_MAX_DEFAULT = 35.0   # [km/h] — ceiling, actual v_max is dynamic
    AY_MAX = 9.5
    # Q[0]=e_y (lateral), Q[1]=e_psi (heading), Q[2]=t (speed tracking)
    # Same as FollowPathState to maintain identical corner-tracking ability
    # Q = [1_000_000.0, 100_000_000.0, 850_000.0]
    # R[0]=v, R[1]=delta (steering) — R[1]=100 to allow full steering in corners
    # R = [100_000.0, 100.0]
    # QN = [1_000_000.0, 1_000.0, 10_000.0]
    # ---
    Q  = [1_000_000_000.0, 500_000_000.0, 100_000.0]
    R  = [1_000_000.0, 500_000_000.0]
    QN = [1_000_000.0, 5_000.0, 10_000.0]

    VEHICLE_DETECT_DISTANCE = 2.5
    MIN_OVERTAKE_WIDTH = 1.4  # minimum available width to execute overtake [m]

    CLEAR_HYSTERESIS_SEC = 1.5  # Must remain clear for 1.5 seconds continuously before returning to follow_path

    def __init__(self) -> None:
        self._clear_start_time: Optional[float] = None

    @property
    def name(self) -> str:
        return "follow"

    def get_params(self) -> MPCStateParams:
        return MPCStateParams(
            v_max=self._V_MAX_DEFAULT,
            ay_max=self.AY_MAX,
            Q=list(self.Q),
            R=list(self.R),
            QN=list(self.QN),
        )

    def check_transition(self, ctx: StateContext) -> Optional[str]:
        if ctx.is_colliding:
            return "recovery"

        eff_dist = _get_effective_forward_distance(ctx)

        # 1.5s Hysteresis: Only return to follow_path if path remains clear for >= 1.5s continuously
        has_v2x_leader = (ctx.forward_vehicle_distance is not None and ctx.forward_vehicle_distance < self.VEHICLE_DETECT_DISTANCE)
        has_any_obstacle = (has_v2x_leader or ctx.has_side_vehicle)
        is_forward_clear = not has_any_obstacle
        if is_forward_clear:
            if self._clear_start_time is None:
                self._clear_start_time = ctx.current_time_sec
            elapsed_clear = ctx.current_time_sec - self._clear_start_time
            if elapsed_clear >= self.CLEAR_HYSTERESIS_SEC:
                self._clear_start_time = None
                return "follow_path"
        else:
            self._clear_start_time = None  # Instantly reset timer if vehicle is detected

        max_side = max(ctx.overtake_width_left, ctx.overtake_width_right)
        has_clearance = max_side >= self.MIN_OVERTAKE_WIDTH
        # is_zero_speed = (ctx.forward_vehicle_speed is not None and ctx.forward_vehicle_speed < 0.3)
        # is_not_aligned = (ctx.forward_vehicle_heading_diff > np.deg2rad(45.0))
        is_slower_leader = (
            ctx.forward_vehicle_speed is None #
            or (
                ctx.forward_vehicle_speed < 2.77 # 10 km/h = 2.77 m/s
                # and ctx.velocity - ctx.forward_vehicle_speed >= 4.17 # 4.17 m/s = 15 km/h speed difference threshold for overtaking
            )
        )
        # is_aligned = (ctx.forward_vehicle_heading_diff <= np.deg2rad(45.0))
        is_clear_side = not ctx.has_side_vehicle

        # Switch to Overtake if clearance exists AND (leader is stopped OR leader heading is NOT aligned OR side-by-side)
        # if has_clearance and (is_zero_speed or is_not_aligned or ctx.has_side_vehicle):
        if has_clearance and is_slower_leader and is_clear_side:
            return "overtake"

        # Stuck detection: ONLY trigger Recovery if NOT intentionally waiting behind a leader/obstacle
        is_waiting_behind_leader = (eff_dist is not None and eff_dist < 10.0 and not has_clearance)
        if not is_waiting_behind_leader:
            if (not ctx.is_in_recovery_cooldown
                    and ctx.time_stopped_sec >= STUCK_DURATION
                    and ctx.velocity < STUCK_VELOCITY_THRESHOLD):
                return "recovery"

        return None

    # -- dynamic v_max -------------------------------------------------------

    def get_adjusted_v_max_kmh(self, ctx: StateContext) -> float:
        """Compute dynamic v_max [km/h] with strict distance governor and side-vehicle yielding."""
        # If a side-vehicle is alongside and clearance is insufficient, yield by reducing speed
        max_side = max(ctx.overtake_width_left, ctx.overtake_width_right)
        if ctx.has_side_vehicle and max_side < self.MIN_OVERTAKE_WIDTH:
            side_speed = ctx.side_vehicle_speed if ctx.side_vehicle_speed is not None else 5.0
            target_mps = max(0.0, side_speed - 1.5)  # Yield by slowing down 1.5 m/s to open longitudinal gap
            return float(np.clip(target_mps * 3.6, 0.0, self._V_MAX_DEFAULT))

        eff_dist = _get_effective_forward_distance(ctx)
        if eff_dist is None:
            return self._V_MAX_DEFAULT

        fwd_speed = ctx.forward_vehicle_speed if ctx.forward_vehicle_speed is not None else 0.0

        if eff_dist <= self.STOP_DISTANCE:
            target_mps = 0.0
        elif eff_dist < self.TARGET_FOLLOWING_DISTANCE:
            # Distance governor: ego speed must never exceed leader speed when closer than 8.0m
            ratio = (eff_dist - self.STOP_DISTANCE) / (self.TARGET_FOLLOWING_DISTANCE - self.STOP_DISTANCE)
            target_mps = fwd_speed * ratio
        else:
            # Normal following: match speed + proportional distance error
            distance_error = eff_dist - self.TARGET_FOLLOWING_DISTANCE
            target_mps = fwd_speed + self.FOLLOWING_KP * distance_error

        target_kmh = target_mps * 3.6
        return float(np.clip(target_kmh, 0.0, self._V_MAX_DEFAULT))


class OvertakeState(DrivingState):
    """Overtake a slower vehicle by inducing a lateral offset."""

    MAX_OVERTAKE_DURATION = 2.5  # [s] (追い越し動作の最大存続時間)

    # ---- hardcoded parameters (35 km/h) ------------------------------------
    V_MAX = 35.0              # [km/h]
    AY_MAX = 9.5
    # Q = [1_000_000.0, 100_000_000.0, 850_000.0]
    # R = [100_000.0, 0.0]
    # QN = [1_000_000.0, 1_000.0, 10_000.0]
    Q = [8_000_000.0, 20_000_000.0, 300_000.0]
    R = [30_000.0, 0.0]
    QN = [4_000_000.0, 1_000.0, 10_000.0]

    VEHICLE_DETECT_DISTANCE = 6.0

    def __init__(self) -> None:
        self._overtake_side: str = "left"  # "left" or "right"
        self._enter_time: Optional[float] = None
        self._calculated_offset: float = 1.8

    @property
    def name(self) -> str:
        return "overtake"

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

    def check_transition(self, ctx: StateContext) -> Optional[str]:
        if ctx.is_colliding:
            return "recovery"

        # Stuck detection
        if (not ctx.is_in_recovery_cooldown
                and ctx.time_stopped_sec >= STUCK_DURATION
                and ctx.velocity < STUCK_VELOCITY_THRESHOLD):
            return "recovery"

        # Overtake timeout: return to follow_path after 2.5s for smooth raceline return
        if self._enter_time is not None:
            elapsed = ctx.current_time_sec - self._enter_time
            if elapsed >= self.MAX_OVERTAKE_DURATION:
                return "follow_path"

        # Vehicle cleared
        if ctx.forward_vehicle_distance is None:
            return "follow_path"
        if ctx.forward_vehicle_distance >= self.VEHICLE_DETECT_DISTANCE:
            return "follow_path"

        return None
