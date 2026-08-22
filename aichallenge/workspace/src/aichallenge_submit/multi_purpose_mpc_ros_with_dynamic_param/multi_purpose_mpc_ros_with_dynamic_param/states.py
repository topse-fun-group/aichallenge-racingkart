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
STUCK_DURATION = 2.0            # [s] — stopped 2.0s triggers recovery (prevents false trigger during launch)


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
    path_kappa: float = 0.0      # closest waypoint curvature [1/m]
    future_max_kappa: float = 0.0 # max curvature over next 15m [1/m]
    is_approaching_straight: bool = False # True if exiting corner into upcoming straight

    # --- V2X (Phase 2) ------------------------------------------------------
    forward_vehicle_distance: Optional[float] = None   # [m]
    forward_vehicle_speed: Optional[float] = None       # [m/s]
    forward_vehicle_heading_diff: float = 0.0          # absolute heading diff relative to path_psi [rad]
    forward_vehicle_x_rel: Optional[float] = None      # relative longitudinal position (+ is ahead, - is behind) [m]
    forward_vehicle_y_rel: Optional[float] = None      # relative lateral position (+ is left, - is right) [m]
    forward_vehicle_pred_y_rel: Optional[float] = None # predicted leader lateral position at time of passing [m]

    # --- LiDAR (Phase 2) ----------------------------------------------------
    overtake_width_left: float = 0.0   # available width on left [m]
    overtake_width_right: float = 0.0  # available width on right [m]
    lidar_forward_clearance: Optional[float] = None   # [m] from LiDAR forward cone
    lidar_range_clearance: Optional[float] = None     # [m] from LiDAR full range
    current_laps: int = 1                             # current race lap count

    # --- Stuck detection ----------------------------------------------------
    time_stopped_sec: float = 0.0  # duration velocity ≈ 0 [s]
    is_in_recovery_cooldown: bool = False  # True during cooldown after recovery / startup


def _get_effective_forward_distance(ctx: StateContext) -> Optional[float]:
    """Get the effective forward distance using V2X."""
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
    VEHICLE_DETECT_DISTANCE = 8.5    # [m] (先行車直後まで一気に詰める)
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

        # Phase 2: forward-vehicle / obstacle detection (V2X)
        eff_dist = _get_effective_forward_distance(ctx)
        if eff_dist is not None and 1.0 < eff_dist < 9.5:
            max_side = max(ctx.overtake_width_left, ctx.overtake_width_right)
            has_basic_clearance = max_side > self.VEHICLE_WIDTH_WITH_MARGIN  # > 2.20m
            has_wide_clearance = max_side >= 2.60                             # >= 2.60m (明確なワイド空間)

            is_hairpin = (abs(ctx.path_kappa) >= 0.070 or ctx.future_max_kappa >= 0.075)
            is_braking_zone = (ctx.future_max_kappa >= 0.055 and abs(ctx.path_kappa) < 0.040)
            is_straight = abs(ctx.path_kappa) < 0.040 and ctx.future_max_kappa < 0.045

            # Leader speed status:
            # - Completely stopped / crashed obstacle: speed < 1.0 m/s (3.6 km/h)
            # - Recovering from wall / very slow: speed < 5.0 m/s (18 km/h)
            is_leader_stopped = (ctx.forward_vehicle_speed is not None and ctx.forward_vehicle_speed < 1.0)
            is_leader_recovering = (ctx.forward_vehicle_speed is not None and ctx.forward_vehicle_speed < 5.0)

            if is_leader_stopped:
                # True stopped obstacle: bypass anywhere on track
                can_overtake = has_basic_clearance
                min_speed_req = 0.5
            elif is_leader_recovering:
                # Recovering / slow vehicle: pass if wide space (even in hairpins!) or basic space in normal zones
                can_overtake = has_wide_clearance or (has_basic_clearance and not is_hairpin)
                min_speed_req = 0.5
            else:
                # Dynamic racing overtake mode: pass if wide space (even in hairpins!) or straights
                can_overtake = has_wide_clearance or (has_basic_clearance and not is_hairpin and not is_braking_zone and is_straight)
                min_speed_req = 3.5

            if can_overtake and ctx.velocity >= min_speed_req:
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

    WAIT_DURATION = 0.1          # [s] (停止待機 0.1秒)
    BACK_SPEED = -4.0            # [m/s] (力強い後退速度で確実に角から脱出)
    BACK_ACCEL = 5.0             # [m/s^2] (後退加速度)
    MIN_BACK_DURATION = 1.8      # [s] (確実に約4mバックして障害物・角から完全離脱)
    MAX_BACK_DURATION = 2.4      # [s] (最大後退時間)
    PATH_DEVIATION_THRESHOLD = 1.5  # [m] — threshold to rejoin

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

        # Must reverse for at least MIN_BACK_DURATION (1.8s) to clear corner obstruction
        if back_elapsed < self.MIN_BACK_DURATION:
            return None

        # 1. Reverse stuck detection: if hit a rear wall after full reverse attempt, switch to forward
        if back_elapsed >= 1.8 and abs(ctx.velocity) < 0.12:
            return "follow_path"

        # 2. Rejoin path once min back duration completed or max reached
        if back_elapsed >= self.MAX_BACK_DURATION or (ctx.path_deviation < self.PATH_DEVIATION_THRESHOLD and back_elapsed >= self.MIN_BACK_DURATION):
            return "follow_path"

        return None

    # -- control override ----------------------------------------------------

    def compute_control_override(
        self, ctx: StateContext
    ) -> Optional[Tuple[float, float, float]]:
        if self._phase == "wait":
            return (0.0, 0.0, 0.0)  # full stop

        # Steering while reversing: turn nose towards raceline (+12 deg bias towards center)
        TARGET_ANGLE_OFFSET = np.deg2rad(12.0)
        if ctx.path_e_y >= 0:
            # Vehicle is to the left of the path -> point nose right (-12 deg relative to path)
            target_psi = ctx.path_psi - TARGET_ANGLE_OFFSET
        else:
            # Vehicle is to the right of the path -> point nose left (+12 deg relative to path)
            target_psi = ctx.path_psi + TARGET_ANGLE_OFFSET

        # Normalized yaw error [-pi, pi]
        psi_err = (ctx.pose_theta - target_psi + np.pi) % (2 * np.pi) - np.pi

        # P-control for reverse steering:
        # In Autoware, steer > 0 is CCW (Left). While reversing, Left steer rotates nose CW (Right).
        # Therefore, psi_err > 0 (nose too far CCW/Left) requires steer > 0 (+K_P * psi_err).
        K_P = 1.2
        steer_cmd = float(np.clip(K_P * psi_err, -0.50, 0.50))

        return (self.BACK_SPEED, steer_cmd, self.BACK_ACCEL)


# ---------------------------------------------------------------------------
# Phase 2 states
# ---------------------------------------------------------------------------

class FollowState(DrivingState):
    """Follow a leading vehicle — maintain safe distance (5.5m), match speed."""

    TARGET_FOLLOWING_DISTANCE = 5.5   # [m] (安全な追従車間目標)
    STOP_DISTANCE = 2.5               # [m] (完全停止・ブレーキ閾値)
    FOLLOWING_KP = 0.6                # speed adjustment gain

    # ---- MPC parameters (same cornering capability as FollowPathState) ------
    _V_MAX_DEFAULT = 35.0   # [km/h] — ceiling, actual v_max is dynamic
    AY_MAX = 9.5
    # Q[0]=e_y (lateral), Q[1]=e_psi (heading), Q[2]=t (speed tracking)
    # Same as FollowPathState to maintain identical corner-tracking ability
    Q = [1_000_000.0, 100_000_000.0, 850_000.0]
    # R[0]=v, R[1]=delta (steering) — R[1]=100 to allow full steering in corners
    R = [100_000.0, 100.0]
    QN = [1_000_000.0, 1_000.0, 10_000.0]

    VEHICLE_DETECT_DISTANCE = 9.5
    VEHICLE_WIDTH_WITH_MARGIN = 2.20

    CLEAR_HYSTERESIS_SEC = 1.5  # Must remain clear for 1.5 seconds continuously before returning to follow_path

    def __init__(self) -> None:
        self._clear_start_time: Optional[float] = None

    @property
    def name(self) -> str:
        return "follow"

    @property
    def control_mode(self) -> ControlMode:
        return ControlMode.PURE_PURSUIT

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

        max_side = max(ctx.overtake_width_left, ctx.overtake_width_right)
        has_basic_clearance = max_side > self.VEHICLE_WIDTH_WITH_MARGIN  # > 2.20m
        has_wide_clearance = max_side >= 2.60                             # >= 2.60m (明確なワイド空間)

        is_hairpin = (abs(ctx.path_kappa) >= 0.070 or ctx.future_max_kappa >= 0.075)
        is_braking_zone = (ctx.future_max_kappa >= 0.055 and abs(ctx.path_kappa) < 0.040)
        is_straight = abs(ctx.path_kappa) < 0.040 and ctx.future_max_kappa < 0.045

        # Leader speed status:
        # - Completely stopped / crashed obstacle: speed < 1.0 m/s (3.6 km/h)
        # - Recovering from wall / very slow: speed < 5.0 m/s (18 km/h)
        is_leader_stopped = (ctx.forward_vehicle_speed is not None and ctx.forward_vehicle_speed < 1.0)
        is_leader_recovering = (ctx.forward_vehicle_speed is not None and ctx.forward_vehicle_speed < 5.0)

        if is_leader_stopped:
            # True stopped obstacle: bypass anywhere on track
            can_overtake = has_basic_clearance
            min_speed_req = 0.5
        elif is_leader_recovering:
            # Recovering / slow vehicle: pass if wide space (even in hairpins!) or basic space in normal zones
            can_overtake = has_wide_clearance or (has_basic_clearance and not is_hairpin)
            min_speed_req = 0.5
        else:
            # Dynamic racing overtake mode: pass if wide space (even in hairpins!) or straights
            can_overtake = has_wide_clearance or (has_basic_clearance and not is_hairpin and not is_braking_zone and is_straight)
            min_speed_req = 3.5

        # 1. Switch to Overtake when clearance is open
        if can_overtake and ctx.velocity >= min_speed_req:
            self._clear_start_time = None
            return "overtake"

        # 2. Check if clearance is completely clear ahead (no leader within 10m)
        has_v2x_leader = (ctx.forward_vehicle_distance is not None and ctx.forward_vehicle_distance < self.VEHICLE_DETECT_DISTANCE)
        is_forward_clear = (not has_v2x_leader and (ctx.lidar_forward_clearance is None or ctx.lidar_forward_clearance >= 5.0))
        if is_forward_clear:
            if self._clear_start_time is None:
                self._clear_start_time = ctx.current_time_sec
            elapsed_clear = ctx.current_time_sec - self._clear_start_time
            if elapsed_clear >= 1.5:
                self._clear_start_time = None
                return "follow_path"
        else:
            self._clear_start_time = None

        # Stuck detection: ONLY trigger Recovery if NOT intentionally waiting behind a leader/obstacle
        is_waiting_behind_leader = (eff_dist is not None and eff_dist < 10.0 and not has_basic_clearance)
        if not is_waiting_behind_leader:
            if (not ctx.is_in_recovery_cooldown
                    and ctx.time_stopped_sec >= STUCK_DURATION
                    and ctx.velocity < STUCK_VELOCITY_THRESHOLD):
                return "recovery"

        return None

    # -- dynamic v_max -------------------------------------------------------

    def get_adjusted_v_max_kmh(self, ctx: StateContext) -> float:
        """Compute dynamic v_max [km/h] to maintain steady following without excessive braking."""
        eff_dist = _get_effective_forward_distance(ctx)
        if eff_dist is None:
            return self._V_MAX_DEFAULT

        fwd_speed = ctx.forward_vehicle_speed if ctx.forward_vehicle_speed is not None else 6.0
        ego_speed = ctx.velocity
        rel_speed = ego_speed - fwd_speed  # > 0 means ego is closing in on leader

        if eff_dist <= self.STOP_DISTANCE:
            # Emergency stop only when critically close (< 1.8m)
            target_mps = 0.0
        elif eff_dist < self.TARGET_FOLLOWING_DISTANCE:
            # Close following zone (1.8m - 5.5m):
            # If relative speed is small (already matched speed), gently brake/coast to preserve momentum!
            # Only apply stronger brake if relative closing speed is large (rel_speed > 0.8 m/s).
            dist_factor = float(np.clip((eff_dist - self.STOP_DISTANCE) / (self.TARGET_FOLLOWING_DISTANCE - self.STOP_DISTANCE), 0.0, 1.0))
            
            if rel_speed <= 0.8:  # Speeds well-matched: maintain 90%-100% of leader speed without dropping anchor
                target_mps = fwd_speed * (0.90 + 0.10 * dist_factor)
            else:  # Closing in too fast: progressively brake to prevent ramming
                target_mps = fwd_speed * (0.65 + 0.35 * dist_factor) - 0.4 * (rel_speed - 0.8)

            # Minimum speed floor: never drop more than 1.0 m/s below moving leader
            if fwd_speed > 3.5:
                target_mps = max(target_mps, fwd_speed - 1.0)
        else:
            # Normal following zone (>= 5.5m): smoothly approach leader
            distance_error = eff_dist - self.TARGET_FOLLOWING_DISTANCE
            target_mps = fwd_speed + self.FOLLOWING_KP * distance_error

        target_kmh = target_mps * 3.6
        return float(np.clip(target_kmh, 0.0, self._V_MAX_DEFAULT))


class OvertakeState(DrivingState):
    """Constant offset parallel attack mode: maintain high exit speed and rocket-pass on straight."""

    LATERAL_OFFSET = 1.30        # [m] (安全な並走オフセット幅)
    MIN_OVERTAKE_DURATION = 2.0  # [s] (最低2.0秒間はレーンをキープしチャタリング離脱を防止)
    MAX_OVERTAKE_DURATION = 6.0  # [s] (最大6.0秒で安全に通常ラインへ復帰・リセット)

    # ---- hardcoded parameters (38 km/h: フル加速で抜き去る) -----------------
    V_MAX_NORMAL = 35.0       # [km/h] normal overtake speed
    V_MAX_BOOST = 40.0        # [km/h] Push-to-Pass / DRS speed (unlocked at lap >= 5)
    AY_MAX = 9.5
    Q = [1_000_000.0, 100_000_000.0, 850_000.0]
    R = [100_000.0, 100.0]
    QN = [1_000_000.0, 1_000.0, 10_000.0]

    VEHICLE_DETECT_DISTANCE = 10.0

    def __init__(self) -> None:
        self._overtake_side: str = "right"  # Default to open right side
        self._enter_time: Optional[float] = None
        self._calculated_offset: float = -0.95
        self._is_boost: bool = False

    @property
    def name(self) -> str:
        return "overtake"

    @property
    def control_mode(self) -> ControlMode:
        return ControlMode.PURE_PURSUIT

    def get_params(self) -> MPCStateParams:
        v_max = self.V_MAX_BOOST if self._is_boost else self.V_MAX_NORMAL
        return MPCStateParams(
            v_max=v_max,
            ay_max=self.AY_MAX,
            Q=list(self.Q),
            R=list(self.R),
            QN=list(self.QN),
            lateral_offset=self._calculated_offset,
        )

    def on_enter(self, ctx: StateContext) -> None:
        self._enter_time = ctx.current_time_sec
        left_space = ctx.overtake_width_left
        right_space = ctx.overtake_width_right

        # Push-to-Pass (DRS): unlocked only when current_laps >= 5
        self._is_boost = (ctx.current_laps >= 5)
        v_max = self.V_MAX_BOOST if self._is_boost else self.V_MAX_NORMAL

        # 100% Truth-based side selection using REAL measured open clearances:
        # Choose whichever side has MORE actual open space between leader and track border!
        if right_space >= left_space:
            self._overtake_side = "right"
            chosen_space = right_space
        else:
            self._overtake_side = "left"
            chosen_space = left_space

        # Dynamic max offset clipping:
        # Offset is carefully scaled, guaranteeing >= 1.35m wall buffer!
        abs_k = abs(ctx.path_kappa)
        is_straight_zone = (abs_k < 0.035) or ctx.is_approaching_straight

        if is_straight_zone:
            max_allowable_offset = 1.40
            min_allowable_offset = 1.25
            ratio = 0.45
        elif abs_k < 0.055:
            max_allowable_offset = 1.35
            min_allowable_offset = 1.20
            ratio = 0.42
        else:
            max_allowable_offset = 1.30
            min_allowable_offset = 1.15
            ratio = 0.40

        # Strictly clip to avoid exceeding wall boundary
        safe_offset = float(np.clip(chosen_space * ratio, min_allowable_offset, max_allowable_offset))
        # Ensure offset leaves at least 1.35m margin to the wall
        safe_offset = min(safe_offset, chosen_space - 1.35)
        safe_offset = max(safe_offset, 1.15)

        if self._overtake_side == "right":
            self._calculated_offset = -safe_offset
        else:
            self._calculated_offset = safe_offset

        print(f"[Overtake Dynamic] Lap {ctx.current_laps}: kappa={ctx.path_kappa:+.3f}, Left={left_space:.2f}m, Right={right_space:.2f}m -> Chose: {self._overtake_side} (offset: {self._calculated_offset:+.2f}m, space={chosen_space:.2f}m)", flush=True)

    def on_exit(self, ctx: StateContext) -> None:
        # Full reset on exit (whether successfully overtaken or failed/aborted)
        self._enter_time = None
        self._is_boost = False

    def check_transition(self, ctx: StateContext) -> Optional[str]:
        if ctx.is_colliding:
            return "recovery"

        # Stuck detection
        if (not ctx.is_in_recovery_cooldown
                and ctx.time_stopped_sec >= STUCK_DURATION
                and ctx.velocity < STUCK_VELOCITY_THRESHOLD):
            return "recovery"

        # Check longitudinal relative position using V2X (x_rel < 0 means ego is ahead of other car)
        x_rel = ctx.forward_vehicle_x_rel

        # 1. Minimum duration lock: stay committed to overtake lane for at least MIN_OVERTAKE_DURATION (2.0s)
        if self._enter_time is not None:
            elapsed = ctx.current_time_sec - self._enter_time
            if elapsed < self.MIN_OVERTAKE_DURATION:
                return None

        # Check if approaching a high-speed braking zone before a sharp corner
        # Avoid cutting across raceline directly in pre-corner braking zones
        is_braking_zone = (ctx.future_max_kappa >= 0.055 and abs(ctx.path_kappa) < 0.040)

        # 2. Successfully overtaken: Leader is comfortably behind us (x_rel < -4.5m) -> smoothly return to raceline
        # In braking zones, hold the shifted line through turn-in to avoid clipping leader's nose
        if x_rel is not None and x_rel < -4.5:
            if not is_braking_zone:
                return "follow_path"

        # 3. If leader has pulled far ahead (> 15.0m)
        if x_rel is not None and x_rel > 15.0:
            return "follow_path"

        # 4. Timeout handling:
        if self._enter_time is not None:
            elapsed = ctx.current_time_sec - self._enter_time
            # While currently side-by-side / overlapping (-4.0m <= x_rel <= 4.0m), NEVER abort! Maintain full acceleration!
            is_side_by_side = (x_rel is not None and -4.0 <= x_rel <= 4.0)
            if elapsed >= self.MAX_OVERTAKE_DURATION and not is_side_by_side and not is_braking_zone:
                return "follow_path"
            if elapsed >= 9.0:  # Absolute failsafe timeout
                return "follow_path"

        return None
