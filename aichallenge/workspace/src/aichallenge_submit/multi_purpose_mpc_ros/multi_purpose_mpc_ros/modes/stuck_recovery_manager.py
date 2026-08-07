import math
from typing import Optional, Tuple
from dataclasses import dataclass
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, Quaternion
from autoware_auto_vehicle_msgs.msg import GearCommand


def yaw_from_quaternion(q: Quaternion) -> float:
    sqx = q.x * q.x
    sqy = q.y * q.y
    sqz = q.z * q.z
    sqw = q.w * q.w

    sarg = -2 * (q.x * q.z - q.w * q.y) / (sqx + sqy + sqz + sqw)

    if sarg <= -0.99999:
        yaw = -2. * np.arctan2(q.y, q.x)
    elif sarg >= 0.99999:
        yaw = 2. * np.arctan2(q.y, q.x)
    else:
        yaw = np.arctan2(2. * (q.x * q.y + q.w * q.z), sqw + sqx - sqy - sqz)

    return float(yaw)


def odom_to_pose_2d(odom: Odometry) -> Pose2D:
    pose = Pose2D()
    pose.x = odom.pose.pose.position.x
    pose.y = odom.pose.pose.position.y
    pose.theta = yaw_from_quaternion(odom.pose.pose.orientation)
    return pose


@dataclass
class StuckRecoveryOutput:
    state: str  # "NORMAL", "BRAKE_BEFORE_REVERSE", "REVERSING", "STOP_BEFORE_FORWARD"
    override_control: bool = False
    u_override: Optional[Tuple[float, float]] = None  # (v_cmd, steer_cmd)
    acc_override: float = 0.0
    bug_acc_enabled: bool = False
    gear_cmd: int = GearCommand.DRIVE
    post_recovery_lock_requested: bool = False


class StuckRecoveryManager:
    """
    Manages Stuck Recovery States:
    - NORMAL: Normal forward driving mode
    - BRAKE_BEFORE_REVERSE: Stopping vehicle & shifting gear to REVERSE
    - REVERSING: Evasive reverse sequence (backwards with evasive steering)
    - STOP_BEFORE_FORWARD: Stopping vehicle & shifting gear back to DRIVE
    """
    def __init__(self, vehicle_id: str, logger=None):
        self.vehicle_id = vehicle_id
        self.logger = logger

        self.state = "NORMAL"
        self.timer_start: Optional[float] = None
        self.phase_start: Optional[float] = None
        self.last_recovery_time: Optional[float] = None
        self.retry_count: int = 0

        self.reverse_duration: float = 2.2
        self.evasive_steer: float = 0.35
        self.stuck_target_radius: float = 0.65
        self.post_recovery_immunity_until: Optional[float] = None

    def _log(self, level: str, msg: str, throttle_duration_sec: Optional[float] = None) -> None:
        if not self.logger:
            return
        log_fn = getattr(self.logger, level, None)
        if not log_fn:
            return
        try:
            log_fn(msg)
        except Exception:
            pass

    def update(
        self,
        now_sec: float,
        v_curr: float,
        u_cmd: Tuple[float, float],
        is_colliding: bool,
        enable_control: bool,
        has_launched: bool,
        odom,
        tracker,
        car,
        mpc,
        stuck_cfg
    ) -> StuckRecoveryOutput:

        if not stuck_cfg or not getattr(stuck_cfg, 'enable_stuck_recovery', True):
            return StuckRecoveryOutput(state="NORMAL", override_control=False, gear_cmd=GearCommand.DRIVE)

        stuck_vel_thresh = float(getattr(stuck_cfg, 'stuck_velocity_threshold', 0.25))
        stuck_time_thresh = float(getattr(stuck_cfg, 'stuck_time_threshold', 1.0))
        rev_duration = float(getattr(stuck_cfg, 'reverse_duration', 2.2))
        stop_duration = float(getattr(stuck_cfg, 'stop_duration', 0.4))
        rev_speed = float(getattr(stuck_cfg, 'reverse_speed', 2.5))

        post_recovery_lock = False

        if not enable_control or (not has_launched and not is_colliding):
            self.timer_start = None

        elif self.state == "NORMAL":
            # 復帰完了後の3.0秒間クーリング期間（発進加速中の低速による誤再判定防止）
            if self.post_recovery_immunity_until is not None:
                if now_sec < self.post_recovery_immunity_until and not is_colliding:
                    self.timer_start = None
                    return StuckRecoveryOutput(state="NORMAL", override_control=False, gear_cmd=GearCommand.DRIVE)
                elif now_sec >= self.post_recovery_immunity_until:
                    self.post_recovery_immunity_until = None

            is_stuck_candidate = (
                (is_colliding and abs(v_curr) <= 0.8) or
                (has_launched and abs(v_curr) <= max(stuck_vel_thresh, 0.35))
            )

            req_time = 0.5 if is_colliding else stuck_time_thresh

            if is_stuck_candidate:
                if self.timer_start is None:
                    self.timer_start = now_sec
                elif (now_sec - self.timer_start) >= req_time:
                    self.state = "BRAKE_BEFORE_REVERSE"
                    self.phase_start = now_sec

                    if self.last_recovery_time is not None and (now_sec - self.last_recovery_time) < 15.0:
                        self.retry_count += 1
                    else:
                        self.retry_count = 1
                    self.last_recovery_time = now_sec

                    if self.retry_count >= 2:
                        self.reverse_duration = 3.2
                        rev_steer_angle = 0.50
                        self.stuck_target_radius = 0.65
                    else:
                        self.reverse_duration = max(2.2, rev_duration)
                        rev_steer_angle = max(0.40, float(getattr(stuck_cfg, 'reverse_steer_angle', 0.35)))
                        self.stuck_target_radius = 0.65

                    lead_rel_y = 0.0
                    found_lead = False
                    if odom is not None and tracker is not None and hasattr(tracker, 'active_vehicle_ids'):
                        pose = odom_to_pose_2d(odom)
                        ego_x, ego_y, ego_yaw = pose.x, pose.y, pose.theta
                        fwd_sin, fwd_cos = math.sin(ego_yaw), math.cos(ego_yaw)
                        left_cos, left_sin = -fwd_sin, fwd_cos

                        min_d = float('inf')
                        for vid in tracker.active_vehicle_ids():
                            if vid == self.vehicle_id:
                                continue
                            buf = tracker._samples.get(vid) if hasattr(tracker, '_samples') else None
                            if not buf:
                                continue
                            _, ox, oy = buf[-1]
                            dx, dy = ox - ego_x, oy - ego_y
                            d = math.hypot(dx, dy)
                            if 0.5 <= d < 15.0 and d < min_d:
                                lead_rel_y = dx * left_cos + dy * left_sin
                                min_d = d
                                found_lead = True

                    if found_lead:
                        # 他車が左側 (lead_rel_y > 0) -> バック時「左ステア (+rev_steer_angle)」でノーズを右へ振る
                        # 他車が右側 (lead_rel_y <= 0) -> バック時「右ステア (-rev_steer_angle)」でノーズを左へ振る
                        self.evasive_steer = rev_steer_angle if lead_rel_y > 0 else -rev_steer_angle
                    else:
                        e_y_curr = 0.0
                        if mpc is not None and hasattr(mpc, 'model') and mpc.model is not None and hasattr(mpc.model, 'spatial_state'):
                            if mpc.model.spatial_state is not None:
                                e_y_curr = getattr(mpc.model.spatial_state, 'e_y', 0.0)

                        # 左壁衝突 (e_y_curr > 0): バック時「左ステア (+rev_steer_angle)」によりフロントノーズを右（コース中央 e_y=0）へ振る
                        # 右壁衝突 (e_y_curr <= 0): バック時「右ステア (-rev_steer_angle)」によりフロントノーズを左（コース中央 e_y=0）へ振る
                        self.evasive_steer = rev_steer_angle if e_y_curr > 0 else -rev_steer_angle

                    self._log('warn',
                        f"[STUCK RECOVERY] Stuck detected (try #{self.retry_count})! (v={v_curr:.2f} m/s, e_y={e_y_curr if not found_lead else 0.0:.2f}m, duration={now_sec - self.timer_start:.1f}s). "
                        f"Reverse dur={self.reverse_duration:.1f}s steer={self.evasive_steer:.2f} rad rad_target={self.stuck_target_radius:.2f}m. Initiating reverse sequence...")
            else:
                self.timer_start = None

        if self.state == "BRAKE_BEFORE_REVERSE":
            elapsed = now_sec - (self.phase_start or now_sec)
            if elapsed < stop_duration:
                self._log('warn', f"[STUCK RECOVERY] Braking for reverse gear shift... ({elapsed:.1f}/{stop_duration:.1f}s)")
                return StuckRecoveryOutput(
                    state=self.state,
                    override_control=True,
                    u_override=(0.0, 0.0),
                    acc_override=-3.0,
                    bug_acc_enabled=False,
                    gear_cmd=GearCommand.REVERSE
                )
            else:
                self.state = "REVERSING"
                self.phase_start = now_sec
                self._log('warn', f"[STUCK RECOVERY] Shifted to REVERSE. Reversing with evasive steer {self.evasive_steer:.2f} rad...")

        if self.state == "REVERSING":
            elapsed = now_sec - (self.phase_start or now_sec)
            if elapsed < self.reverse_duration:
                self._log('warn', f"[STUCK RECOVERY] Reversing evasively... v_cmd={abs(rev_speed):.1f} m/s, steer={self.evasive_steer:.2f} rad ({elapsed:.1f}/{self.reverse_duration:.1f}s)")
                return StuckRecoveryOutput(
                    state=self.state,
                    override_control=True,
                    u_override=(abs(rev_speed), self.evasive_steer),
                    acc_override=1.5,
                    bug_acc_enabled=False,
                    gear_cmd=GearCommand.REVERSE
                )
            else:
                self.state = "STOP_BEFORE_FORWARD"
                self.phase_start = now_sec
                self._log('info', "[STUCK RECOVERY] Reverse complete. Stopping before shifting forward...")

        if self.state == "STOP_BEFORE_FORWARD":
            elapsed = now_sec - (self.phase_start or now_sec)
            if elapsed < stop_duration:
                return StuckRecoveryOutput(
                    state=self.state,
                    override_control=True,
                    u_override=(0.0, 0.0),
                    acc_override=-1.0,
                    bug_acc_enabled=False,
                    gear_cmd=GearCommand.DRIVE
                )
            else:
                if car is not None and hasattr(car, 'update_reference_path') and car.reference_path is not None:
                    car.update_reference_path(car.reference_path)
                self.state = "NORMAL"
                self.timer_start = None
                self.post_recovery_immunity_until = now_sec + 3.0

                if mpc is not None:
                    mpc.infeasibility_counter = 0
                    if hasattr(mpc, 'current_control') and mpc.current_control is not None:
                        mpc.current_control = mpc.current_control * 0.0
                    if hasattr(mpc, 'set_previous_steering'):
                        mpc.set_previous_steering(0.0)

                post_recovery_lock = True
                self._log('warn', "[STUCK RECOVERY] Resuming forward MPC control in OVERTAKING mode (LOCKED 10s)! (MPC state reset & evasive radius active)")

        return StuckRecoveryOutput(
            state=self.state,
            override_control=False,
            gear_cmd=GearCommand.DRIVE,
            post_recovery_lock_requested=post_recovery_lock
        )
