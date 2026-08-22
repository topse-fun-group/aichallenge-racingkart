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
STUCK_DURATION = 2.0            # [s] — stopped 8s triggers recovery (prevents startup false-alarm)


# ---------------------------------------------------------------------------
# Forward-vehicle detection (shared by every state and by both detectors)
# ---------------------------------------------------------------------------
# follow_path と follow が別々の検知器・別々の視野角を見ていると、
# 「片方だけが見える」角度に他車がいるとき 1.5s ヒステリシス + 1.0s dwell の
# 周期で状態が往復し、そのたびに制御モードが Pure Pursuit ↔ MPC で切り替わって
# 大きくふらつく。両方の検知器がこの同じ値を使うこと。
FORWARD_CONE_DEG = 45.0        # [deg] 前方検知の半角
FORWARD_LATERAL_MAX = 3.5      # [m] 前方検知の横方向ゲート

# 進入 < 離脱 とすることで距離方向にもヒステリシスを持たせる
ENTER_FOLLOW_DISTANCE = 10.0    # [m] 中心間の経路弧長。これ以下で follow へ
EXIT_FOLLOW_DISTANCE = 9.0     # [m] これを超えて初めて follow_path へ戻る

# カート全長。bicycle_model.length (1.087) はホイールベースであって全長ではない。
# 中心間距離からバンパー間ギャップを出すために引く。
VEHICLE_LENGTH = 1.6           # [m]


# ---------------------------------------------------------------------------
# Overtake gating (shared by FollowPathState and FollowState)
# ---------------------------------------------------------------------------
# 先行車がこれ以下の速度なら追い越し対象。24.0 ちょうどだと「24km/h前後で走る
# 先行車」が境界に乗り、V2X の2点差分速度のノイズで判定が毎tick反転する。
OVERTAKE_LEAD_SPEED_KMH = 25.5   # [km/h] was 24.0

# 接近中判定: 相対距離 OVERTAKE_CLOSING_MARGIN_M を OVERTAKE_TTC_SEC 以内に
# 詰められる速度差があるか。== 速度差 >= 1.47 m/s。
OVERTAKE_CLOSING_MARGIN_M = 1.0  # [m] default 2.2
OVERTAKE_TTC_SEC = 2.5           # [s] default 1.5

# 「もう詰めきっている」と見なす、目標車間からの許容超過幅。
# 追従制御は v_ego -> v_lead に収束させるのが仕事なので、整定すると速度差が 0 に
# なり接近中判定は原理的に成立しなくなる。目標車間まで詰めたこと自体を
# 追い越しのトリガにしないと、遅い先行車の後ろで永久に follow から出られない。
SETTLED_GAP_TOLERANCE = 1.0      # [m]


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
    forward_vehicle_distance: Optional[float] = None   # [m] centre-to-centre along the path
    forward_vehicle_gap: float = 0.0                   # [m] bumper-to-bumper (distance - VEHICLE_LENGTH)
    forward_vehicle_speed: Optional[float] = None       # [m/s]
    forward_vehicle_heading_diff: float = 0.0          # absolute heading diff relative to path_psi [rad]
    # 自車と同一レーン帯 (|y_rel| <= FORWARD_LATERAL_MAX) にいる最寄り車両の符号付き経路弧長 [m]。
    # 正 = 前方 / 負 = 後方。前方コーン (±45°) は掛けていないので、真横に並んだ相手も
    # 抜き終わった相手もここには残る。OvertakeState の完了判定用。
    nearest_vehicle_s_rel: Optional[float] = None       # [m] signed; negative = behind ego

    # --- ReferencePath Clearance & Overtake ----------------------------------
    overtake_width_left: float = 0.0      # available width on left [m]
    overtake_width_right: float = 0.0     # available width on right [m]
    target_overtake_offset: float = 0.0   # dynamic lateral offset [m] for centerline of free space
    has_side_vehicle: bool = False        # True if another vehicle is alongside (-2.5m <= x_rel <= 2.5m)
    side_vehicle_speed: Optional[float] = None  # speed of side vehicle [m/s]
    lidar_forward_clearance: Optional[float] = None   # retained for compatibility
    lidar_range_clearance: Optional[float] = None
    # --- Detailed Multi-Vehicle Scan for FollowPath transitions ---
    has_forward_vehicle: bool = False                # -15° <= angle <= 15°, r <= VEHICLE_DETECT_DISTANCE (default 8m)
    min_forward_overtake_width: float = 0.0          # Minimum of max(left_w, right_w) among fwd vehicles [m]
    closest_forward_vehicle_speed: Optional[float] = None  # Speed of nearest fwd vehicle [m/s]
    has_left_side_vehicle: bool = False              # 30° <= angle <= 150°, 0 <= y <= 3m
    has_left_side_cutin_hazard: bool = False         # 3s projected: 30° <= angle <= 90°, 0 <= y <= 3m
    has_right_side_vehicle: bool = False             # -150° <= angle <= -30°, -3m <= y <= 0
    has_right_side_cutin_hazard: bool = False        # 3s projected: -90° <= angle <= -30°, -3m <= y <= 0

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
        """Called once when transitioning into this state."""
        pass

    def on_exit(self, ctx: StateContext) -> None:
        """Called once when transitioning out of this state."""
        pass

    @abstractmethod
    def check_transition(self, ctx: StateContext) -> Optional[str]:
        """Evaluate sensor snapshot and return next state name, or None."""
        ...


# ---------------------------------------------------------------------------
# Concrete States
# ---------------------------------------------------------------------------

class FollowPathState(DrivingState):
    """Normal racing state: tracks reference path using pure pursuit / MPC without obstacles."""

    # ---- hardcoded parameters (45 km/h) ------------------------------------
    V_MAX = 45.0              # [km/h] (default target speed)
    AY_MAX = 9.5              # [m/s^2]
    Q = [1_000_000.0, 100_000_000.0, 850_000.0]
    R = [100_000.0, 100.0]  # R[1]=1000.0 を追加してステアリング微振動を抑止
    QN = [1_000_000.0, 1_000.0, 10_000.0]

    # ---- forward-vehicle detection thresholds ------------------------------
    # モジュール定数から取ること。ここだけ変えると follow 側の判定と食い違い、
    # 状態がチャタリングする（FORWARD_CONE_DEG のコメント参照）。
    VEHICLE_DETECT_DISTANCE = ENTER_FOLLOW_DISTANCE
    VEHICLE_DETECT_ANGLE_MIN = -FORWARD_CONE_DEG
    VEHICLE_DETECT_ANGLE_MAX = FORWARD_CONE_DEG
    MIN_OVERTAKE_WIDTH = 2.9          # [m] (車幅 1.4m + オフセット 1.2m = 3.0)

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
        # 1. Recovery (always immediate)
        if ctx.is_colliding:
            return "recovery"

        # Stuck detection: velocity near zero for too long → Recovery (unless in cooldown)
        if (not ctx.is_in_recovery_cooldown
                and ctx.time_stopped_sec >= STUCK_DURATION
                and ctx.velocity < STUCK_VELOCITY_THRESHOLD):
            return "recovery"

        # 2. Check Follow condition (any of 3 conditions)
        # 条件1: 前方15度から-15度の範囲かつ前方検知距離内に車両がある
        #        かつ 前方の車両の左右のどちらかの道幅が MIN_OVERTAKE_WIDTH 以下 (複数車は最小道幅)
        follow_cond1 = (
            ctx.has_forward_vehicle
            and ctx.min_forward_overtake_width <= self.MIN_OVERTAKE_WIDTH
        )

        # 条件2: 前方30度から150度かつy軸0mから3m以内に他車があり、
        #        相対速度×3秒後の位置が前方30度から90度かつy軸0mから3m以内
        follow_cond2 = ctx.has_left_side_cutin_hazard

        # 条件3: 前方-30度から-150度かつy軸0mから-3m以内に他車があり、
        #        相対速度×3秒後の位置が前方-30度から-90度かつy軸0mから-3m以内
        follow_cond3 = ctx.has_right_side_cutin_hazard

        # if follow_cond1 or follow_cond2 or follow_cond3:
        #     return "follow"
        if follow_cond1 and (follow_cond2 or follow_cond3):
            return "follow"

        # 3. Check Overtake condition (all 5 conditions must be met)
        # ① 前方15度から-15度の範囲かつ前方検知距離内に車両がある
        # ② 前方30度から150度、かつ、y方向0mから3m以内に他の車両がない
        # ③ 前方-30度から-150度、かつ、y方向0mから-3m以内に他の車両がない
        # ④ 前方の車両の左右のどちらかの道幅が MIN_OVERTAKE_WIDTH より大きい (複数車は最小道幅)
        # ⑤ 前の車両の速度が OVERTAKE_LEAD_SPEED_KMH 以下 かつ 接近中 (速度差 >= 1.47 m/s)
        if ctx.has_forward_vehicle:
            no_left_vehicle = not ctx.has_left_side_vehicle
            no_right_vehicle = not ctx.has_right_side_vehicle
            has_sufficient_width = ctx.min_forward_overtake_width > self.MIN_OVERTAKE_WIDTH

            lead_speed = ctx.closest_forward_vehicle_speed if ctx.closest_forward_vehicle_speed is not None else 0.0
            is_slow_leader = (lead_speed <= (OVERTAKE_LEAD_SPEED_KMH / 3.6))
            speed_diff = ctx.velocity - lead_speed
            # follow_path はまだ接近中のフェーズなので、接近判定のみで妥当。
            # 割り算をやめたのは speed_diff < 0（自車の方が遅い）のとき 2.2/負数 が
            # 符号で <= 1.5 を通ってしまい、speed_diff == 0.0 では
            # ZeroDivisionError で制御ループが落ちるため（両車停止時に起こり得る）。
            is_ttc_close = speed_diff >= (
                (1.5 * VEHICLE_LENGTH + OVERTAKE_CLOSING_MARGIN_M) / OVERTAKE_TTC_SEC
            )
            # has_diff = ctx.forward_vehicle_gap < 3.0
            
            if (no_left_vehicle
                    and no_right_vehicle
                    and has_sufficient_width
                    and is_slow_leader
                    and is_ttc_close):
                return "overtake"

            return "follow"

        # 4. Stay in FollowPath if none matched
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
    FORWARD_TURN_SPEED = 4.0     # [m/s] (前進旋回速度)
    BACK_ACCEL = 3.5             # [m/s^2]
    FORWARD_ACCEL = 3.0          # [m/s^2]

    STEER_LOCK = 0.58            # [rad] (大舵角ステアリング)

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
    """Follow a leading vehicle — keep a constant time headway, match speed."""

    # ---- spacing controller (constant time headway) -------------------------
    # 目標車間を自車速に比例させるので、時間車間そのものが速度フィードバックとして
    # 働く。距離だけの純P制御はアクチュエータ遅れと組み合わさると必ず
    # オーバーシュートする（string 不安定）ため使わない。
    # 整定時のバンパー間ギャップは厳密に D0 + TIME_HEADWAY * v_ego になる。
    # K_GAP / K_V は到達の速さを決めるだけで整定値には影響しない（整定点では
    # (gap - d_des) == 0 なので係数が何であれ指令は同じ）。
    # 中心間距離 (= forward_vehicle_distance) はこれに VEHICLE_LENGTH (1.6) を足した値。
    D0 = 1.5                     # [m] 停止時のバンパー間ギャップ default 1.5
    # 0.35 だと 24km/h の先行車に対して中心間 4.93m で整定し、has_forward_vehicle の
    # 検知距離 ENTER_FOLLOW_DISTANCE (5.0m) の境界に張り付いて追い越し判定が立たない。
    # 0.20 なら中心間 3.93m。一次系の時定数は 2.85s -> 2.70s とほぼ変わらず、
    # オーバーシュートなしの性質も保たれる。
    TIME_HEADWAY = 0.35          # [s] default 0.35
    K_GAP = 0.6                  # [1/s] ギャップ誤差 → 速度
    K_V = 0.5                    # [-]  相対速度ダンピング
    EMERGENCY_GAP = 0.5          # [m] これ未満でのみ 0 指令
    # 前方車が動いている限り這わない。低速は MPC の線形化も痛めるため
    # (core/MPC.py の v_lin_min 参照)、縦横どちらにとっても効く。
    MIN_FOLLOW_SPEED_KMH = 10.0
    LEADER_MOVING_MPS = 0.5

    # ---- MPC parameters (same cornering capability as FollowPathState) ------
    _V_MAX_DEFAULT = 35.0   # [km/h] — ceiling, actual v_max is dynamic
    AY_MAX = 6.5
    # Q[0]=e_y (lateral), Q[1]=e_psi (heading), Q[2]=t (speed tracking)
    # Same as FollowPathState to maintain identical corner-tracking ability
    # Q = [1_000_000.0, 100_000_000.0, 850_000.0]
    # R[0]=v, R[1]=delta (steering) — R[1]=100 to allow full steering in corners
    # R = [100_000.0, 100.0]
    # QN = [1_000_000.0, 1_000.0, 10_000.0]
    # ---
    # Q  = [2_000_000.0, 100_000_000.0, 100_000.0]
    # R  = [100_000.0, 100_000_000.0]
    # QN = [2_000_000.0, 10_000.0, 3_000.0]
    # ---
    Q = [200.0, 1500.0, 100.0]
    R = [0.1, 1200.0]
    QN = [200.0, 1500.0, 100.0]



    # 離脱しきい値は進入 (ENTER_FOLLOW_DISTANCE) より遠くする
    VEHICLE_DETECT_DISTANCE = EXIT_FOLLOW_DISTANCE
    MIN_OVERTAKE_WIDTH = 2.9  # minimum available width to execute overtake [m]

    CLEAR_HYSTERESIS_SEC = 1.5  # Must remain clear for 1.5 seconds continuously before returning to follow_path

    # 横制御の選択。False にすると follow_path と同一の Pure Pursuit で走る。
    # mpc_controller 側の PD 補正ブロックと pose シフトは既に ControlMode.MPC で
    # ゲートされているので、このフラグ一つで両方とも一緒に切れる。
    # 縦制御（車間保持）は横のディスパッチ後に u[0] を上書きする構造なので、
    # どちらを選んでも影響を受けない。MPC と PP の実走比較用。
    USE_MPC_FOLLOW: bool = False

    def __init__(self) -> None:
        self._clear_start_time: Optional[float] = None

    @property
    def name(self) -> str:
        return "follow"

    @property
    def control_mode(self) -> ControlMode:
        return ControlMode.MPC if self.USE_MPC_FOLLOW else ControlMode.PURE_PURSUIT

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

        # Stuck detection: velocity near zero for too long → Recovery (unless in cooldown)
        if (not ctx.is_in_recovery_cooldown
                and ctx.time_stopped_sec >= STUCK_DURATION
                and ctx.velocity < STUCK_VELOCITY_THRESHOLD):
            return "recovery"

        # 2. Check Follow condition (any of 3 conditions)
        # 条件1: 前方15度から-15度の範囲かつ前方検知距離内に車両がある
        #        かつ 前方の車両の左右のどちらかの道幅が MIN_OVERTAKE_WIDTH 以下 (複数車は最小道幅)
        follow_cond1 = (
            ctx.has_forward_vehicle
            and ctx.min_forward_overtake_width <= self.MIN_OVERTAKE_WIDTH
        )

        # 条件2: 前方30度から150度かつy軸0mから3m以内に他車があり、
        #        相対速度×3秒後の位置が前方30度から90度かつy軸0mから3m以内
        follow_cond2 = ctx.has_left_side_cutin_hazard

        # 条件3: 前方-30度から-150度かつy軸0mから-3m以内に他車があり、
        #        相対速度×3秒後の位置が前方-30度から-90度かつy軸0mから-3m以内
        follow_cond3 = ctx.has_right_side_cutin_hazard

        # if follow_cond1 or follow_cond2 or follow_cond3:
        #     return "follow"
        if follow_cond1 and (follow_cond2 or follow_cond3):
                    return "follow"

        # 3. Check Overtake condition (all 5 conditions must be met)
        # ① 前方15度から-15度の範囲かつ前方検知距離内に車両がある
        # ② 前方30度から150度、かつ、y方向0mから3m以内に他の車両がない
        # ③ 前方-30度から-150度、かつ、y方向0mから-3m以内に他の車両がない
        # ④ 前方の車両の左右のどちらかの道幅が MIN_OVERTAKE_WIDTH より大きい (複数車は最小道幅)
        # ⑤ 前の車両の速度が OVERTAKE_LEAD_SPEED_KMH 以下 かつ 接近中 (速度差 >= 1.47 m/s)
        if ctx.has_forward_vehicle:
            no_left_vehicle = not ctx.has_left_side_vehicle
            no_right_vehicle = not ctx.has_right_side_vehicle
            has_sufficient_width = ctx.min_forward_overtake_width > self.MIN_OVERTAKE_WIDTH

            lead_speed = ctx.closest_forward_vehicle_speed if ctx.closest_forward_vehicle_speed is not None else 0.0
            # 24.0 ちょうどだと「24km/h前後で走る先行車」が境界に乗り、V2X の
            # 2点差分速度のノイズで毎tick反転する。マージンを取って 27.0。
            is_slow_leader = (lead_speed <= (OVERTAKE_LEAD_SPEED_KMH / 3.6))
            speed_diff = ctx.velocity - lead_speed
            # 追従が整定すると speed_diff -> 0 になるので、接近判定だけでは
            # 原理的に追い越しに入れない（永遠に follow のまま）。
            # 「詰めきって遅い先行車の直後に張り付いている」状態も追い越し可とする。
            # 除算をやめたのは speed_diff < 0 のとき 2.2/負数 が符号で通ってしまい、
            # speed_diff == 0.0 では ZeroDivisionError になるため。
            is_closing = speed_diff >= (
                (1.5 * VEHICLE_LENGTH + OVERTAKE_CLOSING_MARGIN_M) / OVERTAKE_TTC_SEC
            )
            is_settled_behind = (
                ctx.forward_vehicle_gap
                <= self.D0 + self.TIME_HEADWAY * ctx.velocity + SETTLED_GAP_TOLERANCE)
            is_ttc_close = is_closing or is_settled_behind
            # has_diff = ctx.forward_vehicle_gap < 3.0

            if (no_left_vehicle
                    and no_right_vehicle
                    and has_sufficient_width
                    and is_slow_leader
                    and is_ttc_close):
                return "overtake"

            return "follow"

        
        if (not ctx.has_forward_vehicle 
            and not ctx.has_left_side_vehicle 
            and not ctx.has_right_side_vehicle
        ):
            return "follow_path"

        # 4. Stay in Follow if none matched
        return None


    # -- spacing controller --------------------------------------------------

    def get_target_speed_mps(self, ctx: StateContext, v_lead: float) -> float:
        """Target speed [m/s] for following the leader — the single longitudinal law.

        Constant time headway: the desired gap grows with ego speed, so the headway term
        itself acts as velocity feedback. ``K_V`` adds explicit relative-velocity damping
        on top. Treating the whole loop as first order gives
        ``d(gap)/dt = 0.123*v_lead - 0.351*(gap - D0)`` — a stable 2.85 s time constant
        with no overshoot, settling at ``gap = D0 + TIME_HEADWAY * v_lead``.

        ``v_lead`` is passed in already low-pass filtered; the V2X tracker derives it from
        a two-sample finite difference and is far too noisy to use as a feedforward term raw.
        """
        v_max_mps = self._V_MAX_DEFAULT / 3.6

        # Side vehicle alongside with nowhere to go: yield to open a longitudinal gap.
        # 横に車両がいる、かつ、左右のどちらの幅も追い越し幅がない場合、(横の車両の速度-5.0m/s)で追従する
        max_side = max(ctx.overtake_width_left, ctx.overtake_width_right)
        if ctx.has_side_vehicle and max_side < self.MIN_OVERTAKE_WIDTH:
            side_speed = ctx.side_vehicle_speed if ctx.side_vehicle_speed is not None else 5.0
            return float(np.clip(side_speed - 1.5, 0.0, v_max_mps))

        if ctx.forward_vehicle_distance is None:
            return v_max_mps

        # 緊急停止: 前方車両とのバンパー間ギャップが 0.5m 以下なら 0 指令
        gap = ctx.forward_vehicle_gap
        if gap <= self.EMERGENCY_GAP:
            return 0.0

        # 一定時間の車間制御
        v_ego = ctx.velocity
        d_des = self.D0 + self.TIME_HEADWAY * v_ego


        # 前方車両の道幅の大きさに応じて、停止時の距離を短くする=速度を補正する
        # if ctx.min_forward_overtake_width is not None and ctx.min_forward_overtake_width > 1.0:
        #     d_des -= 0.7 * np.clip(
        #         ctx.min_forward_overtake_width, 0.0, self.MIN_OVERTAKE_WIDTH
        #     ) / self.MIN_OVERTAKE_WIDTH  

        v_cmd = v_lead + self.K_GAP * (gap - d_des) + self.K_V * (v_lead - v_ego)

        if v_lead > self.LEADER_MOVING_MPS:
            v_cmd = max(v_cmd, self.MIN_FOLLOW_SPEED_KMH / 3.6)

        return float(np.clip(v_cmd, 0.0, v_max_mps))


class OvertakeState(DrivingState):
    """Overtake a slower vehicle by inducing a lateral offset."""

    # [s] 追い越し動作の最大存続時間。
    # StateManager.MIN_DWELL_TIME (1.0s) より短い値にすると、タイムアウトが dwell に
    # 阻まれて毎tick黙って握り潰され、結局 1.0s 継続することになるため、必ず長くする。
    # 24 km/h の先行車を抜き切るには計算上 2.9 s 必要（進入 中心間 4.9m → 完了 -2.5m の
    # 相対変位 7.4m を、Overtake の V_MAX 35km/h との速度差 3.05 m/s で詰める）。
    # 一方この横オフセットは進入時に決めたきり固定で、レースラインから壁までは
    # 中央値 3.45m・最小 1.75m しかないため、長く保持するほど壁に寄るリスクが上がる。
    MAX_OVERTAKE_DURATION = OVERTAKE_TTC_SEC

    # [m] 追い越し完了とみなす中心間の後方距離。自車の後端が相手の前端を抜けるのに
    # VEHICLE_LENGTH (1.6) が要り、レースラインへ戻り始める余裕として +0.9。
    PASSED_CLEARANCE = 2.5

    MIN_OVERTAKE_WIDTH = 2.9

    # ---- hardcoded parameters (35 km/h) ------------------------------------
    V_MAX = 35.0              # [km/h]
    AY_MAX = 9.5
    # Q = [1_000_000.0, 100_000_000.0, 850_000.0]
    # R = [100_000.0, 0.0]
    # QN = [1_000_000.0, 1_000.0, 10_000.0]
    Q = [8_000_000.0, 20_000_000.0, 300_000.0]
    R = [30_000.0, 0.0]
    QN = [4_000_000.0, 1_000.0, 10_000.0]

    USE_MPC_OVERTAKE: bool = False  # False: Waypoint-Shift Pure Pursuit (Default), True: MPC

    def __init__(self) -> None:
        self._overtake_side: str = "left"  # "left" or "right"
        self._enter_time: Optional[float] = None
        self._calculated_offset: float = 1.8
        self._was_alongside: bool = False  # 一度でも真横に並んだか
        self._passed: bool = False         # 抜き切ったか（一度立てたらラッチ）

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
        # StateManager はこのインスタンスを使い回すので、ラッチは必ず入場時に落とす
        self._was_alongside = False
        self._passed = False
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

    # -- overtake completion --------------------------------------------------

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
        if s_rel is not None and s_rel <= -self.PASSED_CLEARANCE:
            self._passed = True      # 実測: 相手の中心が自車の PASSED_CLEARANCE 後方
        elif s_rel is None and self._was_alongside:
            self._passed = True      # 真横に並んだ後に検知範囲から消えた（ロスト対策）
        return self._passed

    def check_transition(self, ctx: StateContext) -> Optional[str]:
        if ctx.is_colliding:
            return "recovery"

        # Stuck detection
        if (not ctx.is_in_recovery_cooldown
                and ctx.time_stopped_sec >= STUCK_DURATION
                and ctx.velocity < STUCK_VELOCITY_THRESHOLD):
            return "recovery"

        passed = self._update_passed(ctx)
        timed_out = (
            self._enter_time is not None
            and (ctx.current_time_sec - self._enter_time) >= self.MAX_OVERTAKE_DURATION
        )

        # 抜き切るかタイムアウトするまでは他の遷移を一切許さない。
        # 従来はここで forward_vehicle_distance が None になった時点で follow_path へ
        # 戻っていたが、真横に並ぶと相手が前方コーン (±45°) から外れて None になるため、
        # 抜き切る前に必ずレースラインへ戻っていた。停止車はライン外に居ることが多く
        # 「前方から消えた = 抜けた」が実質成立していたので気付かれなかった。
        if not (passed or timed_out):
            return None

        # 2. Check Follow condition (any of 3 conditions)
        # 条件1: 前方15度から-15度の範囲かつ前方検知距離内に車両がある
        #        かつ 前方の車両の左右のどちらかの道幅が MIN_OVERTAKE_WIDTH 以下 (複数車は最小道幅)
        follow_cond1 = (
            ctx.has_forward_vehicle
            and ctx.min_forward_overtake_width <= self.MIN_OVERTAKE_WIDTH
        )

        # follow → state
        # 条件2: 前方30度から150度かつy軸0mから3m以内に他車があり、
        #        相対速度×3秒後の位置が前方30度から90度かつy軸0mから3m以内
        follow_cond2 = ctx.has_left_side_cutin_hazard

        # 条件3: 前方-30度から-150度かつy軸0mから-3m以内に他車があり、
        #        相対速度×3秒後の位置が前方-30度から-90度かつy軸0mから-3m以内
        follow_cond3 = ctx.has_right_side_cutin_hazard

        if follow_cond1 or follow_cond2 or follow_cond3:
            return "follow"

        if ctx.has_forward_vehicle:
            return "follow"

        if (not ctx.has_forward_vehicle
            and not ctx.has_left_side_vehicle
            and not ctx.has_right_side_vehicle
        ):
            return "follow_path"

        return "follow_path"
