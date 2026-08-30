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
STUCK_DURATION = 0.7            # [s] — stopped 8s triggers recovery (prevents startup false-alarm)

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
D0_M                        = 0.7   # [m] 追従時の停止目標車間距離 (default: 1.5)
TIME_HEADWAY_SEC            = 0.8  # [s] 追従時に車間距離を縮める期待時間 (default: 0.35)
FORWARD_FOLLOW_DISTANCE_M   = 5.0   # [m] 追従を行う前方車両との車間距離 (default: 4.0)
FOLLOW_CLEAR_HYSTERESIS_SEC = 1.0   # [s] 追従状態を維持する最低時間 (チャタリング防止)
FOLLOW_STOP_DISTANCE_M      = 0.8   # [m] (完全停止・ブレーキ閾値、遅延を考慮)
FOLLOW_K_GAP                = 1.4   # [1/s] ギャップ誤差 → 速度
FOLLOW_K_V                  = 0.5   # [-] 相対速度ダンピング (default: 0.7)
FOLLOW_MIN_SPEED_KMH        = 10.0  # [km/h] 最低追従速度
FOLLOW_LEADER_MOVING_MPS    = 0.5   # [m/s] 0.5m/s = 1.8km/s
FOLLOW_TARGET_DISTANCE_M    = 1.5

# ---------------------------------------------------------------------------
# overtake state parameter
# ---------------------------------------------------------------------------
MIN_OVERTAKE_WIDTH_M      = 2.6     # [m] 最低追い越し幅 (default 2.5)
MIN_OVERTAKE_LEAD_SPEED   = 25.0    # [km/s] 前方車両の最低追い越し速度
OVERTAKE_CLOSING_MARGIN_M = 1.0     # [m] 追い越し時の車間距離の余裕距離
OVERTAKE_TTC_SEC          = 0.3     # [s] TTCの時間
OVERTAKE_PASSED_CLEARANCE_M = 1.7   # [m] 追い越し完了とみなす中心間の後方距離
                                    # 自車の後端が前方車両の前端を抜けるのに
                                    # 全長(VEHICLE_LENGTH) + ラインへ戻り始める
                                    # オフセットが必要 (最低1.6より大きい値)
OVERTAKE_PASSED_CLEARANCE_TIME_SEC = 0.35 # [s] 追い越し状態のクリア最大時間

# 追い越し中の目標速度。AWSIM の drive-fade 平衡 (約 35.7 km/h) を超える値を入れて
# acc = KP*(u[0]-v) を常に a_max へ飽和させ、追い越し中はフルスロットルにする。
# 35.0 のままだと平衡速度を下回り、追い越しに入った瞬間フルブレーキになる。
OVERTAKE_TARGET_SPEED_KMH        = 50.0  # [km/h]

# コーナー追い越し。参照経路が traj_mincurv (最小曲率ライン) なので空き幅は
# 構造的に外側へ偏る。外側は弧長が長く同じ速度では抜けないため、
# 「内側が空いているコーナー」に限って積極的に仕掛ける。
OVERTAKE_CORNER_KAPPA            = 0.05  # [1/m] これを超えたらコーナー扱い (R=20m)
# 回頭角がこれ未満のコーナーでは、曲率から速度上限を掛ける。
# ref_vel.yaml の区間速度は km/h と m/s の単位不整合で常に v_max に飽和し、
# 全コーナーで 42km/h を目標にしてしまっているため (ADR-033)。
# ヘアピン (>= この角度) は従来どおり据え置く。
TIGHT_CORNER_MAX_TURN_DEG        = 120.0  # [deg]
OVERTAKE_CORNER_LOOKAHEAD_M      = 15.0  # [m] コーナー判定の先読み距離
OVERTAKE_CORNER_MAX_DIST_M       = 6.0   # [m] 仕掛ける最大車間 (中心間)
OVERTAKE_CORNER_SPEED_MARGIN_MPS = 2.0   # [m/s] 必要な相対速度の下限。
                                         # 0.0 は実質無条件で、実測の成功率は 23%。
                                         # 実速度差を要求する条件は 93〜100% だった。
OVERTAKE_COMMIT_SEC              = 1.5   # [s] 幅不足・ロストでも中断しない最低継続時間
OVERTAKE_HARD_ABORT_WIDTH_M      = 1.5   # [m] 幅が崩壊。コミット期間を無視して即中断
OVERTAKE_PREDICT_HORIZON_SEC     = 2.0   # [s] 先行車位置の予測ホライズン
OVERTAKE_PREDICT_LAT_ACCEL_MPSS  = 0.6   # [m/s^2] 先行車の横方向加速度の見積り。
                                         # 0.5*A*T^2 がコース幅 (半幅 約3m) を超えると
                                         # 片側の将来幅が必ず 0 に飽和し、寄せ側が
                                         # sign(heading_diff) だけで決まってしまう。
OVERTAKE_ABORT_WIDTH_M           = 2.3   # [m] 寄せ側がこれを下回ったら中断
                                         #     突入は MIN_OVERTAKE_WIDTH_M (2.6) で、
                                         #     二段閾値にしてノイズでの往復を防ぐ

# ---------------------------------------------------------------------------
# recovery state parameter
# ---------------------------------------------------------------------------
RECOVERY_ALIGNED_HEADING_DEG = 5.0  # [deg] 復帰完了とみなす経路との角度差
RECOVERY_ALIGNED_E_Y_M       = 0.5   # [m]   復帰完了とみなすセンターラインからの距離
RECOVERY_STEER_K             = 1.8   # [-]   経路との角度差 → 舵角のゲイン
                                     #       1.0 = ずれた角度分そのまま切る
RECOVERY_BOOST_VALUE         = 0.0   # [-]   復帰後の boost 値 (OvertakeState と同値)
RECOVERY_BOOST_DURATION_SEC  = 2.0   # [s]   復帰後に boost を維持する時間

# ---------------------------------------------------------------------------
# lateral shift (寄せ側) hysteresis parameter
# ---------------------------------------------------------------------------
# 左右の空き幅の差は先行車の横偏差の 2 倍で効く (diff = ub + lb - 2*e_y_leader)。
# V2X の位置ノイズ σ≈0.1m は幅差 σ≈0.2m 相当なので、開始閾値は 2σ を取る。
LATERAL_SHIFT_ENTER_DIFF_M = 0.4  # [m] 寄せを開始する左右空き幅の差
LATERAL_SHIFT_EXIT_DIFF_M  = 0.2  # [m] センターラインへ戻す左右空き幅の差
LATERAL_SHIFT_DWELL_SEC    = 0.2  # [s] 判定が継続すべき時間 (40Hz で 4 tick) default 0.1

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
    state_id: str
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
    path_kappa: float = 0.0      # [1/m] 先読み区間で最も曲率が大きい点の符号付き曲率
    in_tight_corner: bool = False  # 先読み区間に回頭角 120 度未満のコーナーがあるか
                                 #       正 = 左コーナー (内側が左)

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
    lateral_shift_side: str = "none"      # "left" | "right" | "none" — デッドバンド + dwell 適用後の寄せ側
    # T 秒後の先行車位置でコリドーを引き直した幅。先行車の横移動だけでなく、
    # コーナーでコース幅 (wp.ub / wp.lb) 自体が変わる効果も入る。
    overtake_width_left_future: float = 0.0   # [m]
    overtake_width_right_future: float = 0.0  # [m]
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

    # --- Logging ------------------------------------------------------------
    # 状態側から 1 行ログを出すためのコールバック (mpc_controller の logger に配線)。
    # 追い越しの離脱理由など、[StateManager] の遷移ログだけでは区別できない情報を残す。
    log_event: Optional[Callable[[str], None]] = None


# TODO: 調整中である。左右のステアリング切り返しのチャタリングが十分に抑えられてないため、
# FollowStateはsimple pure pursuitを使用中である。しかし、以下の調整でチャタリングが
# の抑制が確認でき次第、FollowStateにwaypoint shift pure pursuitを適用する
def resolve_overtake_side(ctx: StateContext) -> str:
    """追い越しで実際に寄せる側 ("left" | "right" | "none")。

    Pure Pursuit (mpc_controller) と状態機械がこの 1 つの関数を共有することで、
    「内側へ寄せているのに広い側の幅で中断判定する」食い違いを無くす (ADR-031)。

    側は **今と T 秒後の狭い方**で採点して広い側を選ぶ。将来幅だけで選ぶと、
    「今は塞がっているが 1 秒後に開く側」を掴んでしまい、中断判定は現在幅を見るので
    突入直後に hard_narrow で弾かれる (実測 34 件中 9 件が突入時点で 1.5m 未満)。
    ADR-031 の「コーナーは弧長が短い内側」も撤回済み。参照経路が最小曲率ラインで
    既にインについているため内側には壁までの余地が無く、実測で内側 18% / 外側 31%
    (|kappa| 0.08-0.15) と逆効果だった (ADR-032)。
    """
    if abs(ctx.target_overtake_offset) <= 0.1:
        return "none"          # 前方にも側方にも車がいない
    left_w = min(ctx.overtake_width_left, ctx.overtake_width_left_future)
    right_w = min(ctx.overtake_width_right, ctx.overtake_width_right_future)
    return "left" if left_w >= right_w else "right"


def predict_overtake_widths(left: float, right: float,
                            lead_speed: float, heading_diff: float):
    """T 秒後の左右の空き幅 (left_future, right_future) を返す。

    先行車の速度をセンターライン法線方向に分解し、横加速度
    OVERTAKE_PREDICT_LAT_ACCEL_MPSS で T 秒進んだ移動量 d を左右に加減する。
    先行車が左へ寄る (heading_diff > 0) なら左が狭まり右が広がる。

    経路相対で計算するので、先行車の位置をグローバル座標で直線外挿する方式のように
    コーナーで予測点が外側の壁へ飛び出し「内側が広い」と誤認する破綻が起きない
    (R=6m / 25km/h で外側へ 3.2m ずれていた。ADR-034)。
    """
    T = OVERTAKE_PREDICT_HORIZON_SEC
    sin_hd = np.sin(heading_diff)
    # 向きは heading_diff の符号で決める。v_lat の符号から取ると、先行車が
    # 停止しているときに加速度項まで消えてしまう。
    d = (lead_speed * sin_hd * T
         + 0.5 * OVERTAKE_PREDICT_LAT_ACCEL_MPSS * T * T * np.sign(sin_hd))
    return max(0.0, left - d), max(0.0, right + d)


def overtake_width_of(ctx: StateContext, side: str) -> float:
    """指定した側の空き幅 [m]。"none" は 0.0。"""
    if side == "left":
        return ctx.overtake_width_left
    if side == "right":
        return ctx.overtake_width_right
    return 0.0


def overtake_width_future_of(ctx: StateContext, side: str) -> float:
    """指定した側の T 秒後の空き幅 [m]。"none" は 0.0。"""
    if side == "left":
        return ctx.overtake_width_left_future
    if side == "right":
        return ctx.overtake_width_right_future
    return 0.0


class LateralShiftSideFilter:
    """左右の空き幅差から「どちら側へ寄せるか」を決める。二段閾値 + ラッチ + dwell 付き。

    幅差は先行車の横偏差の 2 倍で効く (diff = ub + lb - 2*e_y_leader) ため、
    デッドバンド無しの単純比較では先行車がセンターライン付近にいるだけで
    V2X の位置ノイズだけで毎 tick 符号が反転し、横目標が 3m 以上ジャンプして
    ステアリングが左右に振れる。

    判定
    ----
    - 差 >  LATERAL_SHIFT_ENTER_DIFF_M : 左へ寄せる候補
    - 差 < -LATERAL_SHIFT_ENTER_DIFF_M : 右へ寄せる候補
    - |差| < LATERAL_SHIFT_EXIT_DIFF_M : センターライン ("none") の候補
    - その間の帯                       : 現状維持 (ラッチ)

    候補が LATERAL_SHIFT_DWELL_SEC 継続して初めて確定する。1 tick でも
    候補が変われば計時をやり直す (FollowState._clear_start_time と同じ型)。
    """

    def __init__(self) -> None:
        self._side: str = "none"
        self._pending: Optional[str] = None
        self._pending_since: float = 0.0

    @property
    def side(self) -> str:
        """現在確定している寄せ側。"""
        return self._side

    def update(self, left_width: float, right_width: float, now_sec: float) -> str:
        diff = left_width - right_width

        # 閾値はモジュールグローバルとして参照する。ROS パラメータは
        # setattr(states, ...) でモジュール属性を差し替えるため、
        # ローカルやデフォルト引数に取り込むと動的更新が効かなくなる。
        if diff > LATERAL_SHIFT_ENTER_DIFF_M:
            candidate = "left"
        elif diff < -LATERAL_SHIFT_ENTER_DIFF_M:
            candidate = "right"
        elif abs(diff) < LATERAL_SHIFT_EXIT_DIFF_M:
            candidate = "none"
        else:
            candidate = self._side  # 解除〜開始閾値の帯は現状維持

        if candidate == self._side:
            self._pending = None
            return self._side

        if candidate != self._pending:
            self._pending = candidate
            self._pending_since = now_sec
        elif (now_sec - self._pending_since) >= LATERAL_SHIFT_DWELL_SEC:
            self._side = candidate
            self._pending = None

        return self._side


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

    def _log_side(self, ctx: StateContext) -> str:
        """ログに出す寄せ側。Overtake は突入時にラッチした側を上書きする。"""
        return resolve_overtake_side(ctx)

    def _exit(self, ctx: StateContext, next_state: str, reason: str) -> str:
        """離脱理由と、その判断に使った値を 1 行残してから遷移先を返す。

        [StateManager] の遷移ログだけでは overtake -> follow_path が「本当に抜けた」のか
        「見失った」のか区別できず、成功率が測定できない。
        3 状態で同じ実装をコピペしていたため同じバグを 3 か所に抱えていたので、
        ここに集約する。
        """
        if ctx.log_event is None:
            return next_state

        def num(x, scale=1.0, fmt="{:.2f}"):
            return "None" if x is None else fmt.format(x * scale)

        enter_time = getattr(self, "_enter_time", None)
        elapsed = ctx.current_time_sec - enter_time if enter_time is not None else 0.0
        side = self._log_side(ctx)
        ctx.log_event(
            f"[{self.name}] exit reason={reason} to={next_state} elapsed={elapsed:.2f}s"
            f" | vehicleID={ctx.state_id}"
            f" ego_v={num(ctx.velocity, 3.6)}"
            f" forward_v={num(ctx.forward_vehicle_speed, 3.6)}"
            f" gap={num(ctx.forward_vehicle_distance)}"
            f" side={side} side_w={overtake_width_of(ctx, side):.2f}"
            f" min_w={ctx.min_forward_overtake_width:.2f}"
            f" kappa={ctx.path_kappa:+.3f}"
            f" offset={num(ctx.target_overtake_offset)}")
        return next_state


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

    def __init__(self) -> None:
        self._enter_time: Optional[float] = None

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

    def on_enter(self, ctx: StateContext) -> None:
        self._enter_time = ctx.current_time_sec


    def check_transition(self, ctx: StateContext) -> Optional[str]:

        ###################################################
        # follow path -> recovery (always immediate)
        ###################################################

        if ctx.is_colliding:
            return self._exit(ctx, "recovery", "collision")

        # Stuck detection: velocity near zero for too long → Recovery (unless in cooldown)
        if (not ctx.is_in_recovery_cooldown
                and ctx.time_stopped_sec >= STUCK_DURATION
                and ctx.velocity < STUCK_VELOCITY_THRESHOLD):
            return self._exit(ctx, "recovery", "stuck")


        ###################################################
        # follow path -> follow
        ###################################################

        # 前方に車両があり、追い越し幅が足りない → follow
        has_overtake_wide = ctx.min_forward_overtake_width > MIN_OVERTAKE_WIDTH_M
        if ctx.has_forward_vehicle and not has_overtake_wide:
            return self._exit(ctx, "follow", "no_forward_vehicle_and_no_width")


        ###################################################
        # follow path -> overtake
        ###################################################

        if (
            ctx.forward_vehicle_speed is not None
            and ctx.forward_vehicle_speed <= 25.0 / 3.6
            and ctx.velocity >= 29.0 / 3.6
        ):
            #---------------------------------------
            # 比較的に安定した条件も一時的にメモで残す
            #---------------------------------------
            # if (
            #     (
            #         (ctx.forward_vehicle_gap + VEHICLE_LENGTH * 1.5)
            #         / (ctx.velocity - ctx.forward_vehicle_speed)
            #     ) <= 2 # 1.5
            # ):
            #     return "overtake"
            #---------------------------------------

            is_left = ctx.overtake_width_left > ctx.overtake_width_right
            is_same_lane = (ctx.path_e_y > 0 and is_left) or (ctx.path_e_y < 0 and not is_left)
            lead_speed = ctx.forward_vehicle_speed if ctx.forward_vehicle_speed is not None else 0.0
            is_overtake_gap = (
                (ctx.forward_vehicle_gap + VEHICLE_LENGTH * 1.5 + OVERTAKE_CLOSING_MARGIN_M)
                <= 0.4 * (ctx.velocity - lead_speed) + 0.5 * 2.5 * 0.4**2
            )
            heading_diff = (
                ctx.forward_vehicle_heading_diff
                if ctx.forward_vehicle_heading_diff is not None
                else 0.0
            )
            # 元は if/elif の入れ子で、(a) 左右同幅 (b) heading_diff == 0 のどちらでも
            # どの枝にも入らず has_future_width が False 固定になり、追い越しを恒久的に
            # ブロックしていた。符号の扱いは既存挙動のまま (左が広いときは必ず縮む向き、
            # 右が広いときは必ず広がる向き) で、抜けていたケースだけを塞ぐ。
            # NOTE: この左右非対称そのものは未解決。ADR-031 参照。
            width_shift = abs((0.5 * 0.4**2 + 0.4 * lead_speed) * np.sin(heading_diff))
            if ctx.overtake_width_left > ctx.overtake_width_right:
                has_future_width = (
                    ctx.overtake_width_left - width_shift >= MIN_OVERTAKE_WIDTH_M)
            else:
                has_future_width = (
                    ctx.overtake_width_right + width_shift >= MIN_OVERTAKE_WIDTH_M)

            if (
                lead_speed < 1.0 / 3.6
                and ctx.min_forward_overtake_width >= MIN_OVERTAKE_WIDTH_M
            ):
                return self._exit(ctx, "overtake", "forward_vehicle_stop_with_width_between_Vf<=25km/h_and_Ve>=29km/h")

            if (
                is_overtake_gap
                and ctx.min_forward_overtake_width >= MIN_OVERTAKE_WIDTH_M
                and has_future_width
                # and is_same_lane # shift waypoint pure pursuit
            ):
                return self._exit(ctx, "overtake", "safe_width_between_Vf<=25km/h_and_Ve>=29km/h")

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
                # の距離を 0.4s で移動できるだけの速度差があるかどうか
                is_closing = speed_diff >= (
                    (
                        ctx.forward_vehicle_gap + VEHICLE_LENGTH + OVERTAKE_CLOSING_MARGIN_M
                    ) / 0.4)

                # 車間距離が(停止車間距離 + 瞬間詰め距離 + オフセット距離)以下であるかどうか
                is_settled_behind = (
                    ctx.forward_vehicle_gap
                    <= D0_M + TIME_HEADWAY_SEC * ctx.velocity + 0.5 * 2.5 * TIME_HEADWAY_SEC**2 + OVERTAKE_CLOSING_MARGIN_M)

                is_ttc_close = is_closing or is_settled_behind

                # 横偏差の予測
                heading_diff = (
                    ctx.forward_vehicle_heading_diff
                    if ctx.forward_vehicle_heading_diff is not None
                    else 0.0
                )
                # 元は if/elif の入れ子で、(a) 左右同幅 (b) heading_diff == 0 のどちらでも
                # どの枝にも入らず has_future_width が False 固定になり、追い越しを恒久的に
                # ブロックしていた。符号の扱いは既存挙動のまま (左が広いときは必ず縮む向き、
                # 右が広いときは必ず広がる向き) で、抜けていたケースだけを塞ぐ。
                # NOTE: この左右非対称そのものは未解決。ADR-031 参照。
                width_shift = abs((0.5 * 0.4**2 + 0.4 * lead_speed) * np.sin(heading_diff))
                if ctx.overtake_width_left > ctx.overtake_width_right:
                    has_future_width = (
                        ctx.overtake_width_left - width_shift >= MIN_OVERTAKE_WIDTH_M)
                else:
                    has_future_width = (
                        ctx.overtake_width_right + width_shift >= MIN_OVERTAKE_WIDTH_M)

                if (
                    lead_speed < 1.0 / 3.6
                    and ctx.min_forward_overtake_width >= MIN_OVERTAKE_WIDTH_M
                ):
                    return self._exit(ctx, "overtake", "stop_with_width_in_enough_distance")

                if (
                    not ctx.has_left_side_vehicle
                    and not ctx.has_right_side_vehicle
                    and ctx.min_forward_overtake_width >= MIN_OVERTAKE_WIDTH_M
                    and is_slow_leader
                    and is_ttc_close
                    and has_future_width
                    # and is_same_lane # shift waypoint pure pursuit
                ):
                    return self._exit(ctx, "overtake", "safe_width_with_enough_distance")
                return self._exit(ctx, "follow", "not_safe_width_with_enough_distance")
        #-------------------------- version 3 --------------------------
        return None


class RecoveryState(DrivingState):
    """2-Phase Recovery: directional back -> directional forward turn -> follow_path.

    Sequence
    --------
    1. **back** (``BACK_DURATION_TIME_SEC``): reverse (gear=REVERSE).
    2. **forward_turn** (``FORWARD_DURATION_TIME_SEC``): forward drive (gear=DRIVE).
    3. Transition to ``follow_path``.

    操舵
    ----
    どちらのフェーズも**経路からずれた角度分だけ舵を切る**
    (``delta = ±RECOVERY_STEER_K * e_psi``、``±RECOVERY_STEER_LOCK_RAD`` でクリップ)。
    自転車モデルの ``psi_dot = (v/L) * tan(delta)`` より e_psi を減らす舵角の符号は
    進行方向で反転するので、後退では ``+``、前進では ``-`` を取る。
    後退中は角度を消す過程で ``e_y`` も自然に減る (``y_dot ~ v * sin(e_psi)``)。
    経路と平行に刺さった (``e_psi ~ 0``) ときは舵角も 0 になり、まっすぐ後退する。

    ``on_enter`` が直接 ``back`` に入るため、**衝突を検知した tick から後退指令と
    ギア REVERSE が出る**。停止待機フェーズを挟むと、StateManager が遷移した tick では
    新しい状態の ``check_transition`` を呼ばない都合で (0,0,0) + GEAR_DRIVE が
    1 tick 漏れて後退の立ち上がりが遅れる。

    早期離脱
    --------
    ``back`` / ``forward_turn`` はいずれも「経路と平行 (角度差 < ``RECOVERY_ALIGNED_HEADING_DEG``)
    かつセンターライン近傍 (|e_y| < ``RECOVERY_ALIGNED_E_Y_M``)」が成立した時点で
    次へ進む。姿勢がほとんど崩れていない軽い接触なら数 tick で ``follow_path`` に戻る。
    各フェーズの時間はフェーズ開始からで測るので、``back`` を早く抜けても
    ``forward_turn`` の持ち時間は変わらない。

    退出時に ``ctx.publish_boost`` で boost を入れる。OFF は mpc_controller 側の
    デッドラインが出す (follow_path 以外へ直行しても確実に切るため)。
    """

    BACK_DURATION_TIME_SEC = 2.0     # [s] 最大後退時間
    FORWARD_DURATION_TIME_SEC = 1.5  # [s] 最大前進時間
                                     # 前進フェーズの役割は姿勢を戻すことで、加速ではない。
                                     # 3.0s だと整列条件を満たせない急コーナーで毎回
                                     # タイムアウトまで全開加速し、+15km/h ほど乗せた状態で
                                     # コーナーへ復帰していた (ADR-033)。

    # 速度・加速度はいずれも「車両側の上限まで出し切る」ことを狙った値。
    # override 経路には np.clip(acc, a_min, a_max) が掛からない (mpc_controller が
    # override ブロックで早期 return するため) ので、ここの値がそのまま
    # longitudinal.speed / .acceleration に載る。実効上限は AWSIM の車両モデル次第。
    RECOVERY_FORWARD_TURN_SPEED_MPS = 32.0   # [m/s] 前進旋回速度
    RECOVERY_BACK_TURN_SPEED_MPS    = -32.0  # [m/s] 後退旋回速度

    RECOVERY_FORWARD_ACCEL_MPSS = 3.0  # [m/s^2] USE_BUG_ACC と同値
    RECOVERY_BACK_ACCEL_MPSS = 3.0     # [m/s^2] (override 側で abs() を取る)

    RECOVERY_STEER_LOCK_RAD = 1.48     # [rad] 大舵角ステアリング角度

    def __init__(self) -> None:
        self._phase: str = "back"                     # "back" | "forward_turn"
        self._enter_time: Optional[float] = None
        self._phase_start_time: float = 0.0           # 現フェーズの開始時刻 [s]

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
        # 衝突を検知した tick から後退を始める。停止待機フェーズを挟むと
        # (0,0,0) + GEAR_DRIVE が 1 tick 漏れる (クラス docstring 参照)。
        self._enter_phase("back", ctx)

    def on_exit(self, ctx: StateContext) -> None:
        self._enter_time = None
        self._phase = "back"

        # 復帰直後の立ち上がりを稼ぐため boost を入れる。
        # OFF は mpc_controller 側のデッドラインが出す。follow_path 以外へ
        # 直行した場合でも確実に切るため、状態側では時間を持たない。
        if ctx.publish_boost is not None:
            ctx.publish_boost(RECOVERY_BOOST_VALUE)

    def _enter_phase(self, phase: str, ctx: StateContext) -> None:
        self._phase = phase
        self._phase_start_time = ctx.current_time_sec

    def _heading_error(self, ctx: StateContext) -> float:
        """経路方位に対する自車 heading のずれ [rad]。左が正、[-pi, pi) に正規化。"""
        return (ctx.pose_theta - ctx.path_psi + np.pi) % (2 * np.pi) - np.pi

    def _is_aligned(self, ctx: StateContext) -> bool:
        """経路と平行かつセンターライン近傍か（＝もう復帰動作は要らないか）。"""
        # ctx.path_deviation は _build_state_context で代入されておらず常に 0.0 なので
        # 使わない。横偏差は path_e_y、角度差は pose_theta と path_psi から出す。
        return (
            abs(self._heading_error(ctx)) < np.deg2rad(RECOVERY_ALIGNED_HEADING_DEG)
            and abs(ctx.path_e_y) < RECOVERY_ALIGNED_E_Y_M
        )

    def check_transition(self, ctx: StateContext) -> Optional[str]:
        if self._enter_time is None:
            return None

        # フェーズごとの経過時間。早期離脱で back を短く切り上げても
        # forward_turn の持ち時間が変わらないよう、累積ではなくフェーズ基準で測る。
        phase_elapsed = ctx.current_time_sec - self._phase_start_time

        if self._phase == "back":
            # 向きが整ったら後退を打ち切って前進へ
            if self._is_aligned(ctx) or phase_elapsed >= self.BACK_DURATION_TIME_SEC:
                self._enter_phase("forward_turn", ctx)
            return None


        ###################################################
        # recovery -> follow path
        ###################################################

        # forward_turn: 向きが整ったら即座に通常走行へ戻る
        if self._is_aligned(ctx) or phase_elapsed >= self.FORWARD_DURATION_TIME_SEC:
            return self._exit(ctx, "follow_path", "aligned")

        return None

    def compute_control_override(
        self, ctx: StateContext
    ) -> Optional[Tuple[float, float, float]]:
        # 経路からずれた角度分だけ舵を切る。自転車モデルの psi_dot = (v/L)*tan(delta)
        # より、e_psi を減らす舵角の符号は進行方向で反転する。
        #   後退 (v < 0): delta と同符号   -> +K * e_psi
        #   前進 (v > 0): delta と逆符号   -> -K * e_psi
        # 経路と平行に刺さった (e_psi ~ 0) ときは舵角も 0 になり、まっすぐ後退する。
        e_psi = self._heading_error(ctx)
        lock = self.RECOVERY_STEER_LOCK_RAD

        if self._phase == "back":
            steer_cmd = float(np.clip(RECOVERY_STEER_K * e_psi + 0.3 * ctx.path_e_y, -lock, lock))
            return (self.RECOVERY_BACK_TURN_SPEED_MPS, steer_cmd, self.RECOVERY_BACK_ACCEL_MPSS)

        # phase == "forward_turn" (前進旋回)
        steer_cmd = float(np.clip(-RECOVERY_STEER_K * e_psi - 0.3 * ctx.path_e_y, -lock, lock))
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
        self._enter_time: Optional[float] = None

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

    def on_enter(self, ctx: StateContext) -> None:
        self._enter_time = ctx.current_time_sec


    def check_transition(self, ctx: StateContext) -> Optional[str]:

        ##########################################
        # follow -> recovery
        ##########################################

        if ctx.is_colliding:
            return self._exit(ctx, "recovery", "collision")


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
                return self._exit(ctx, "follow_path", "no_surrounding_vehicle")
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

        is_left = ctx.overtake_width_left > ctx.overtake_width_right
        is_same_lane = (ctx.path_e_y > 0 and is_left) or (ctx.path_e_y < 0 and not is_left)
        # has_long_gap = ctx.forward_vehicle_distance > 3.5 if ctx.forward_vehicle_distance is not None else False

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
            ) / 0.8 #default: 0.4
        )

        # 車間距離が(停止車間距離 + 瞬間詰め距離 + オフセット距離)以下であるかどうか
        is_settled_behind = (
            ctx.forward_vehicle_gap
            <= D0_M + TIME_HEADWAY_SEC * ctx.velocity + 0.8 * 2.5 * TIME_HEADWAY_SEC**2 + OVERTAKE_CLOSING_MARGIN_M)

        is_ttc_close = is_closing or is_settled_behind

        # 2s後の先行車両の左右の幅が、最低追い越し幅以上あるかどうか
        heading_diff = (
            ctx.forward_vehicle_heading_diff
            if ctx.forward_vehicle_heading_diff is not None
            else 0.0
        )
        # 元は if/elif の入れ子で、(a) 左右同幅 (b) heading_diff == 0 のどちらでも
        # どの枝にも入らず has_future_width が False 固定になり、追い越しを恒久的に
        # ブロックしていた。符号の扱いは既存挙動のまま (左が広いときは必ず縮む向き、
        # 右が広いときは必ず広がる向き) で、抜けていたケースだけを塞ぐ。
        # NOTE: この左右非対称そのものは未解決。ADR-031 参照。
        width_shift = abs((0.5 * 0.8**2 + 0.8 * lead_speed) * np.sin(heading_diff)) # default: 0.4s
        if ctx.overtake_width_left > ctx.overtake_width_right:
            has_future_width = (
                ctx.overtake_width_left - width_shift >= MIN_OVERTAKE_WIDTH_M)
        else:
            has_future_width = (
                ctx.overtake_width_right + width_shift >= MIN_OVERTAKE_WIDTH_M)

        # --- T 秒後に幅が大きい側へ仕掛ける ------------------------------------
        # 上の has_future_width は「現在の幅から先行車の横移動分を引く」近似で、
        # コーナーで wp.ub / wp.lb (コース幅) 自体が変わる効果を拾えない。
        # 実測ではその変化が幅縮小 0.95 m/s の支配要因だった。ここでは先行車の
        # 予測位置でコリドーを引き直した将来幅を直接使う (ADR-032)。
        future_side = ("left" if ctx.overtake_width_left_future >= ctx.overtake_width_right_future
                       else "right")
        future_w = overtake_width_future_of(ctx, future_side)
        now_w = overtake_width_of(ctx, future_side)

        if (
            future_w >= MIN_OVERTAKE_WIDTH_M          # T 秒後も幅が残る
            and now_w >= MIN_OVERTAKE_WIDTH_M         # 今も入れる
            and ctx.min_forward_overtake_width >= MIN_OVERTAKE_WIDTH_M
            and speed_diff >= OVERTAKE_CORNER_SPEED_MARGIN_MPS
            and not ctx.has_left_side_vehicle
            and not ctx.has_right_side_vehicle
        ):
            return self._exit(ctx, "overtake", "future_width_overtake")

        # ここにコーナーなどで積極的に追い越しを試行する条件を書く
        # 参照経路は traj_mincurv (最小曲率ライン) なので、空き幅は構造的に外側へ偏る
        # (コーナーの約 72%)。外側は弧長が長く同じ速度では抜けないため、
        # 「内側が空いているコーナー」に限って積極的に仕掛ける。
        is_corner = abs(ctx.path_kappa) > OVERTAKE_CORNER_KAPPA
        inside_is_left = ctx.path_kappa > 0.0          # kappa > 0 = 左コーナー
        inside_width = (
            ctx.overtake_width_left if inside_is_left else ctx.overtake_width_right)

        # 近さは中心間距離で見る。ctx.forward_vehicle_gap は _build_state_context で
        # 代入されておらず常に 0.0 なので使わない。
        is_near_corner = (
            ctx.forward_vehicle_distance is not None
            and ctx.forward_vehicle_distance <= OVERTAKE_CORNER_MAX_DIST_M)

        # 絶対速度ゲート (is_slow_leader) ではなく相対速度で見る。
        # 内側は弧長が短いので、速度差が小さくても詰められる。
        has_corner_speed_margin = speed_diff >= OVERTAKE_CORNER_SPEED_MARGIN_MPS

        if (
            is_corner
            and inside_width >= MIN_OVERTAKE_WIDTH_M
            # 他の前方車で塞がっていないか。この枝だけ最近傍 1 台のコリドーしか
            # 見ておらず、実走ログで 5 件が奥の車に塞がれたまま突入していた。
            and ctx.min_forward_overtake_width >= MIN_OVERTAKE_WIDTH_M
            and is_near_corner
            and has_corner_speed_margin
            and not ctx.has_left_side_vehicle
            and not ctx.has_right_side_vehicle
        ):
            return self._exit(ctx, "overtake", "corner_overtake")

        # 先行車両が停止時に追い越し
        if (
            lead_speed < 1.0 / 3.6
            and ctx.min_forward_overtake_width >= MIN_OVERTAKE_WIDTH_M
        ):
            return self._exit(ctx, "overtake", "forward_vehicle_stop")

        # 安全な追い越し条件
        if (
            not ctx.has_left_side_vehicle
            and not ctx.has_right_side_vehicle
            and ctx.min_forward_overtake_width >= MIN_OVERTAKE_WIDTH_M
            and is_slow_leader
            and is_ttc_close
            and has_future_width
            # and is_same_lane # shift waypoint pure pursuit
            # and has_long_gap
        ):
            return self._exit(ctx, "overtake", "safe_width")

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
                return self._exit(ctx, "recovery", "stuck")

        return None

    def get_adjusted_v_max_mps(self, ctx: StateContext) -> float:
        """Compute dynamic v_max [m/s] with a gap PD controller and side-vehicle yielding."""
        # If a side-vehicle is alongside and clearance is insufficient, yield by reducing speed
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

        if ctx.forward_vehicle_distance is None or ctx.forward_vehicle_speed is None:
            return VEHICLE_V_MAX / 3.6

        v_ego = ctx.velocity
        v_lead = ctx.forward_vehicle_speed

        # バンパー間ギャップ。forward_vehicle_distance は参照経路沿いの中心間距離
        # (ADR-027) なので全長を引く。ctx.forward_vehicle_gap は
        # _build_state_context で代入されておらず常に 0.0 なので使わない。
        gap = max(0.0, ctx.forward_vehicle_distance - VEHICLE_LENGTH)

        # 追い越せる幅があるうちは D0_M まで詰めて追い越しの助走を作る。
        # 幅が無いときは FOLLOW_TARGET_DISTANCE_M の安全車間を保つ。
        d_des = (
            D0_M
            if ctx.min_forward_overtake_width >= MIN_OVERTAKE_WIDTH_M
            else FOLLOW_TARGET_DISTANCE_M
        )

        # P項: 車間誤差 → 速度 / D項: 相対速度ダンピング
        # gap == d_des かつ v_ego == v_lead で v_cmd == v_lead となり定常偏差は残らない。
        # 詰まりすぎたときは v_cmd が負になり、下の clip で 0 (フルブレーキ) に落ちる。
        v_cmd = (
            v_lead
            + FOLLOW_K_GAP * (gap - d_des)
            + FOLLOW_K_V * (v_lead - v_ego)
        )

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

        # 実際に寄せる側をラッチする。中断判定はこの側の幅だけを見る。
        # max(left, right) で見ていると、内側が塞がっても外側が広ければ中断が効かず、
        # 塞がった側へ突っ込み続ける (ADR-031 の実走ログで確認)。
        self._overtake_side = resolve_overtake_side(ctx)

        # ReferencePath から計算された空き領域の真ん中を通る動的オフセットを採用
        if abs(ctx.target_overtake_offset) > 0.1:
            self._calculated_offset = ctx.target_overtake_offset
        elif ctx.overtake_width_left >= ctx.overtake_width_right:
            half_w = ctx.overtake_width_left / 2.0
            self._calculated_offset = float(np.clip(half_w, 1.2, 2.2))
        else:
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

    def _log_side(self, ctx: StateContext) -> str:
        """突入時にラッチした側。追い越し中は側が固定されるため。"""
        return self._overtake_side

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

    def _is_committed(self, ctx: StateContext) -> bool:
        """入場から OVERTAKE_COMMIT_SEC 未満か（＝まだ引き返さない期間か）。

        実測 (autoware.log の [StateManager] 行 9286 エピソード) では追い越しの 48.4% が
        StateManager.MIN_DWELL_TIME = 1.0s ちょうどで終了していた。35km/h で 1.0s は
        9.7m しかなく、コーナーを抜けきる前に引き返している。
        """
        if self._enter_time is None:
            return False
        return (ctx.current_time_sec - self._enter_time) < OVERTAKE_COMMIT_SEC


    def check_transition(self, ctx: StateContext) -> Optional[str]:

        ###################################################
        # overtake -> recovery
        ###################################################

        # 1. 衝突検知（コミット期間中でも即時）
        if ctx.is_colliding:
            return self._exit(ctx, "recovery", "collision")

        # 2. スタック検知（コミット期間中でも即時）
        if (not ctx.is_in_recovery_cooldown
                and ctx.time_stopped_sec >= STUCK_DURATION
                and ctx.velocity < STUCK_VELOCITY_THRESHOLD):
            return self._exit(ctx, "recovery", "stuck")


        ###################################################
        # overtake -> follow
        ###################################################

        # 3. 道幅が狭くなったら Follow に戻る。ただしコミット期間中は維持する
        #    （シフトしかけの一瞬の幅不足で引き返さないため）。
        has_v2x_leader = (
            ctx.forward_vehicle_distance is not None
            and ctx.forward_vehicle_distance < FORWARD_VEHICLE_DETECTION
        )
        if has_v2x_leader:
            # 実際に寄せている側の幅だけを見る。突入 (MIN_OVERTAKE_WIDTH_M) より
            # 低い OVERTAKE_ABORT_WIDTH_M で判定し、ノイズでの往復を防ぐ。
            side_w = overtake_width_of(ctx, self._overtake_side)

            # ハード中断: 幅が崩壊したらコミット期間を無視して即座に引き返す。
            # コミット期間は「シフトしかけの一瞬の幅不足で引き返さない」ためのもので、
            # 「幅がもう無いと分かっている隙間へ突っ込み続ける」ためのものではない。
            # 実測では中断の 66% がコミット明け直後に発火し、その時点の幅の 17% は
            # 1.0m 未満だった。幅は 0.95 m/s で縮むので、突入 2.83m なら約 0.66 秒で
            # 中断閾値を割る。残り 0.84 秒 (約 8m) を突っ込み続けていた (ADR-032)。
            if side_w < OVERTAKE_HARD_ABORT_WIDTH_M:
                return self._exit(ctx, "follow", "hard_narrow")

            if side_w < OVERTAKE_ABORT_WIDTH_M and not self._is_committed(ctx):
                return self._exit(ctx, "follow", "narrow")


        ###################################################
        # overtake -> follow path
        ###################################################

        # 4. 追い越し完了判定（相手が後方に抜けた）。コミット期間に関係なく即時。
        passed = self._update_passed(ctx)
        if passed:
            return self._exit(ctx, "follow_path", "passed")

        # 5. 並走中の保護（真横に並んでいる間は相手側面への切り込みを防ぐため Overtake を維持）
        if ctx.has_side_vehicle:
            return None

        # 6. 前方にも側方にも車が見えなくなったら復帰。ただしコミット期間中は維持する。
        #    横に 1.5〜2m シフトすると前方検知 (±45deg / 10m) と側方検知
        #    (0.6 <= |y_rel| <= 3.5) の両方の窓から一瞬抜け落ちるため、実際には
        #    「抜けた」のではなく「見失った」だけでここへ来るケースが多い。
        #    実測ではこの経路が出口の 81% (7531/9286) を占めていた。
        if not has_v2x_leader and not ctx.has_side_vehicle and not self._is_committed(ctx):
            return self._exit(ctx, "follow_path", "lost")

        return None
