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
STUCK_DURATION = 8.0            # [s] — stopped 8s triggers recovery (prevents startup false-alarm)


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

    # --- LiDAR (Phase 2) ----------------------------------------------------
    overtake_width_left: float = 0.0   # available width on left [m]
    overtake_width_right: float = 0.0  # available width on right [m]
    lidar_forward_clearance: Optional[float] = None   # [m] from LiDAR forward cone
    lidar_range_clearance: Optional[float] = None     # [m] from LiDAR full range

    # --- Stuck detection ----------------------------------------------------
    time_stopped_sec: float = 0.0  # duration velocity ≈ 0 [s]
    is_in_recovery_cooldown: bool = False  # True during cooldown after recovery / startup


def _get_effective_forward_distance(ctx: StateContext) -> Optional[float]:
    """Get the effective forward distance combining V2X and close-proximity LiDAR (<= 5.0m)."""
    dists = []
    if ctx.forward_vehicle_distance is not None:
        dists.append(ctx.forward_vehicle_distance)
    # Gated LiDAR: Consider LiDAR if obstacle is within 5.0m radius to guarantee collision prevention
    if ctx.lidar_forward_clearance is not None and ctx.lidar_forward_clearance <= 5.0:
        dists.append(ctx.lidar_forward_clearance)
    return min(dists) if dists else None


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
    VEHICLE_DETECT_DISTANCE = 15.0   # [m]
    VEHICLE_WIDTH_WITH_MARGIN = 2.30 # vehicle width + safety margin [m]

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

        # Phase 2: forward-vehicle / obstacle detection (V2X + LiDAR fusion)
        eff_dist = _get_effective_forward_distance(ctx)
        if eff_dist is not None and eff_dist < self.VEHICLE_DETECT_DISTANCE:
            max_side = max(ctx.overtake_width_left, ctx.overtake_width_right)
            has_clearance = max_side > self.VEHICLE_WIDTH_WITH_MARGIN
            is_zero_speed = (ctx.forward_vehicle_speed is not None and ctx.forward_vehicle_speed < 0.3)
            is_aligned = (ctx.forward_vehicle_heading_diff <= np.deg2rad(45.0))

            # (1) Overtake if clearance exists AND (leader is stopped OR leader heading is NOT aligned with path)
            # (2) Follow if no clearance OR leader is aligned and moving
            if has_clearance and (is_zero_speed or not is_aligned):
                return "overtake"
            else:
                return "follow"

        return None


class RecoveryState(DrivingState):
    """Recovery after collision: wait → back up → rejoin path.

    Sequence
    --------
    1. **wait** (``WAIT_DURATION`` seconds): full stop.
    2. **back**: reverse at gentle speed (gear=REVERSE) until path deviation is small enough
       or ``MAX_BACK_DURATION`` elapses.
    3. Transition to ``follow_path``.
    """

    WAIT_DURATION = 2.0          # [s] (停止待機)
    BACK_SPEED = -2.5            # [m/s] (後退速度)
    BACK_ACCEL = 2.5             # [m/s^2] (正の絶対値でスロットルを要求)
    MIN_BACK_DURATION = 1.5      # [s] (最低1.5秒はバックを継続してチャタリング防止)
    MAX_BACK_DURATION = 3.5      # [s] (最大バック時間)
    PATH_DEVIATION_THRESHOLD = 2.0  # [m] — threshold to rejoin

    def __init__(self) -> None:
        self._enter_time: Optional[float] = None
        self._phase: str = "wait"          # "wait" | "back"

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

        # phase == "back"
        back_elapsed = elapsed - self.WAIT_DURATION

        # Enforce minimum back duration to prevent instant exit / chattering
        if back_elapsed < self.MIN_BACK_DURATION:
            return None

        if ctx.path_deviation < self.PATH_DEVIATION_THRESHOLD:
            return "follow_path"
        if back_elapsed >= self.MAX_BACK_DURATION:
            return "follow_path"

        return None

    # -- control override ----------------------------------------------------

    def compute_control_override(
        self, ctx: StateContext
    ) -> Optional[Tuple[float, float, float]]:
        if self._phase == "wait":
            return (0.0, 0.0, 0.0)  # full stop

        # Steering while reversing: turn nose to face parallel + 10 deg towards the raceline
        TARGET_ANGLE_OFFSET = np.deg2rad(10.0)  # 10 degrees in radians
        if ctx.path_e_y >= 0:
            # Vehicle is to the left of the path -> point nose right (-10 deg relative to path)
            target_psi = ctx.path_psi - TARGET_ANGLE_OFFSET
        else:
            # Vehicle is to the right of the path -> point nose left (+10 deg relative to path)
            target_psi = ctx.path_psi + TARGET_ANGLE_OFFSET

        # Normalized yaw error [-pi, pi]
        psi_err = (ctx.pose_theta - target_psi + np.pi) % (2 * np.pi) - np.pi

        # P-control for reverse steering
        # psi_err > 0 (nose too far left) -> turn right (steer > 0) to rotate nose right
        K_P = 1.2
        steer_cmd = float(np.clip(K_P * psi_err, -0.55, 0.55))

        return (self.BACK_SPEED, steer_cmd, self.BACK_ACCEL)


# ---------------------------------------------------------------------------
# Phase 2 states
# ---------------------------------------------------------------------------

class FollowState(DrivingState):
    """Follow a leading vehicle — maintain safe distance (8m), match speed."""

    TARGET_FOLLOWING_DISTANCE = 7.0   # [m] (相対距離保持目標)
    STOP_DISTANCE = 3.5               # [m] (完全停止・ブレーキ閾値、遅延を考慮)
    FOLLOWING_KP = 0.5                # speed adjustment gain

    # ---- MPC parameters (same cornering capability as FollowPathState) ------
    _V_MAX_DEFAULT = 35.0   # [km/h] — ceiling, actual v_max is dynamic
    AY_MAX = 9.5
    # Q[0]=e_y (lateral), Q[1]=e_psi (heading), Q[2]=t (speed tracking)
    # Same as FollowPathState to maintain identical corner-tracking ability
    Q = [1_000_000.0, 100_000_000.0, 850_000.0]
    # R[0]=v, R[1]=delta (steering) — R[1]=100 to allow full steering in corners
    R = [100_000.0, 100.0]
    QN = [1_000_000.0, 1_000.0, 10_000.0]

    VEHICLE_DETECT_DISTANCE = 12.0
    VEHICLE_WIDTH_WITH_MARGIN = 2.80

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

        # Check if clearance is completely clear ahead (no V2X leader AND no LiDAR obstacle within 5m)
        has_v2x_leader = (ctx.forward_vehicle_distance is not None and ctx.forward_vehicle_distance < self.VEHICLE_DETECT_DISTANCE)
        has_lidar_close_obstacle = (ctx.lidar_forward_clearance is not None and ctx.lidar_forward_clearance < 5.0)

        # 1.5s Hysteresis: Only return to follow_path if path remains clear for >= 1.5s continuously
        is_forward_clear = (not has_v2x_leader and not has_lidar_close_obstacle)
        if is_forward_clear:
            if self._clear_start_time is None:
                self._clear_start_time = ctx.current_time_sec
            elapsed_clear = ctx.current_time_sec - self._clear_start_time
            if elapsed_clear >= self.CLEAR_HYSTERESIS_SEC:
                self._clear_start_time = None
                return "follow_path"
        else:
            self._clear_start_time = None  # Instantly reset timer if vehicle/obstacle is detected

        max_side = max(ctx.overtake_width_left, ctx.overtake_width_right)
        has_clearance = max_side > self.VEHICLE_WIDTH_WITH_MARGIN
        is_zero_speed = (ctx.forward_vehicle_speed is not None and ctx.forward_vehicle_speed < 0.3)
        is_aligned = (ctx.forward_vehicle_heading_diff <= np.deg2rad(45.0))

        # Switch to Overtake if clearance exists AND (leader is stopped OR leader heading is NOT aligned)
        if has_clearance and (is_zero_speed or not is_aligned):
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
        """Compute dynamic v_max [km/h] with strict distance governor to maintain 8m relative distance."""
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

    LATERAL_OFFSET = 1.8  # [m] (壁に接触しない安全な横オフセット幅)
    MAX_OVERTAKE_DURATION = 2.5  # [s] (追い越し動作の最大存続時間)

    # ---- hardcoded parameters (35 km/h) ------------------------------------
    V_MAX = 35.0              # [km/h]
    AY_MAX = 9.5
    Q = [1_000_000.0, 100_000_000.0, 850_000.0]
    R = [100_000.0, 0.0]
    QN = [1_000_000.0, 1_000.0, 10_000.0]

    VEHICLE_DETECT_DISTANCE = 15.0

    def __init__(self) -> None:
        self._overtake_side: str = "left"  # "left" or "right"
        self._enter_time: Optional[float] = None
        self._calculated_offset: float = 2.5

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
        # 空きスペースの中央 (幅 / 2.0) を通る動的オフセット算出
        if ctx.overtake_width_left >= ctx.overtake_width_right:
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
