import math
from typing import Optional
from dataclasses import dataclass

from multi_purpose_mpc_ros.core.utils import kmh_to_m_per_sec


@dataclass
class V2XStateOutput:
    mode: str
    speed_limit: float
    vehicle_radius: float
    is_emergency_brake: bool = False


class V2XModeManager:
    """
    Manages V2X Obstacle Avoidance Modes:
    - NORMAL: Default race speed and normal obstacle radius
    - FOLLOWING: Adaptive Cruise Control (ACC) behind moving lead car
    - EMERGENCY_BRAKE: Safety deceleration when TTC < 1.5s or distance < 5m
    - OVERTAKING: Bypass / evasion mode with reduced obstacle radius & unlimited speed limit
    """
    def __init__(self, vehicle_id: str, logger=None):
        self.vehicle_id = vehicle_id
        self.logger = logger

        self.mode = "NORMAL"
        self.vehicle_radius = 1.0  # Normal default radius
        self.speed_limit = float('inf')

        self.following_since: Optional[float] = None
        self.emergency_brake_since: Optional[float] = None
        self.overtake_lock_until: Optional[float] = None
        self.stuck_target_radius: float = 0.65
        self.motion_start_time: Optional[float] = None

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

    def set_motion_start_time(self, t_sec: float) -> None:
        if self.motion_start_time is None:
            self.motion_start_time = t_sec

    def lock_overtaking(self, lock_until_sec: float, radius: float = 0.65) -> None:
        self.mode = "OVERTAKING"
        self.vehicle_radius = radius
        self.speed_limit = float('inf')
        self.overtake_lock_until = lock_until_sec

    def update(
        self,
        now_sec: float,
        ego_x: float,
        ego_y: float,
        ego_yaw: float,
        ego_speed_mps: float,
        ego_wp_id: int,
        tracker,
        reference_path,
        car,
        v2x_cfg,
        mpc_v_max: float,
        vehicle_radius_normal: float
    ) -> V2XStateOutput:

        if not v2x_cfg:
            return V2XStateOutput(self.mode, self.speed_limit, self.vehicle_radius)

        follow_start = float(getattr(v2x_cfg, 'follow_distance_start', 15.0))
        follow_brake = float(getattr(v2x_cfg, 'follow_distance_brake', 5.0))
        v_min_safe_mps = kmh_to_m_per_sec(float(getattr(v2x_cfg, 'v_min_safe', 8.0)))
        ttc_thresh = float(getattr(v2x_cfg, 'ttc_threshold', 1.5))
        fwd_cos_thresh = float(getattr(v2x_cfg, 'forward_cos_threshold', 0.5))
        overtake_patience = float(getattr(v2x_cfg, 'overtake_patience', 3.0))
        overtake_gap_min = float(getattr(v2x_cfg, 'overtake_gap_min', 10.0))
        overtake_clearance = float(getattr(v2x_cfg, 'overtake_clearance', 8.0))
        vehicle_radius_overtake = float(getattr(v2x_cfg, 'vehicle_radius_overtake', 0.65))
        v_max_normal = mpc_v_max

        min_d = float('inf')
        min_ttc = float('inf')
        lead_speed = 0.0
        is_leading_ahead = False
        min_rel_fwd = 0.0
        min_rel_lat = 0.0
        min_lead_rel_fwd = 0.0

        fwd_cos = math.cos(ego_yaw)
        fwd_sin = math.sin(ego_yaw)

        wp_lookahead_max = int(getattr(v2x_cfg, 'wp_lookahead_max', 30))
        wp_dist_max = float(getattr(v2x_cfg, 'wp_dist_max', 2.5))

        n_wps = len(reference_path.waypoints) if (reference_path and hasattr(reference_path, 'waypoints')) else 0
        n_base = getattr(reference_path, 'n_base_waypoints', n_wps) if (reference_path and getattr(reference_path, 'circular', False)) else n_wps

        if tracker is not None and hasattr(tracker, 'active_vehicle_ids'):
            for vid in tracker.active_vehicle_ids():
                if vid == self.vehicle_id:
                    continue
                buf = tracker._samples.get(vid) if hasattr(tracker, '_samples') else None
                if not buf:
                    continue
                _, ox, oy = buf[-1]
                vx, vy = tracker.velocity(vid)

                dx = ox - ego_x
                dy = oy - ego_y
                d = math.hypot(dx, dy)

                if d < 0.5:
                    continue

                if n_base > 0 and car is not None and reference_path is not None:
                    lead_wp_id = car.get_closest_waypoint_global(ox, oy)
                    wp_diff = (lead_wp_id - (ego_wp_id % n_base)) % n_base

                    if not (0 <= wp_diff <= wp_lookahead_max):
                        continue

                    lead_wp = reference_path.waypoints[lead_wp_id % len(reference_path.waypoints)]
                    dist_to_wp = math.hypot(ox - lead_wp.x, oy - lead_wp.y)
                    if dist_to_wp > wp_dist_max:
                        continue

                dot = dx * fwd_cos + dy * fwd_sin
                cos_angle = dot / max(d, 0.001)
                is_ahead = cos_angle >= fwd_cos_thresh

                if is_ahead and d < min_d:
                    min_d = d
                    min_lead_rel_fwd = dot
                    is_leading_ahead = True
                    lead_speed = math.hypot(vx, vy)
                    ego_vx = ego_speed_mps * fwd_cos
                    ego_vy = ego_speed_mps * fwd_sin
                    rel_approach = (ego_vx - vx) * fwd_cos + (ego_vy - vy) * fwd_sin
                    rel_cross = abs((ego_vx - vx) * (-fwd_sin) + (ego_vy - vy) * fwd_cos)

                    min_rel_fwd = rel_approach
                    min_rel_lat = rel_cross

                    if rel_approach > 0.0:
                        min_ttc = d / rel_approach
                    else:
                        min_ttc = float('inf')

        # OVERTAKING Lock check
        if self.overtake_lock_until is not None:
            if now_sec < self.overtake_lock_until:
                is_passed = (is_leading_ahead and min_lead_rel_fwd < -2.0)
                if not is_leading_ahead or is_passed or min_d >= follow_start:
                    self.mode = "NORMAL"
                    self.vehicle_radius = vehicle_radius_normal
                    self.speed_limit = float('inf')
                    self.following_since = None
                    self.emergency_brake_since = None
                    self.overtake_lock_until = None
                    self._log('info', f"[V2X] Overtaking COMPLETE during lock! d={min_d:.1f}m. Back to NORMAL.")
                else:
                    self.mode = "OVERTAKING"
                    self.vehicle_radius = self.stuck_target_radius
                    self.speed_limit = min(v_max_normal, max(kmh_to_m_per_sec(15.0), lead_speed + kmh_to_m_per_sec(10.0)))
                return V2XStateOutput(self.mode, self.speed_limit, self.vehicle_radius)
            else:
                self.overtake_lock_until = None

        overtake_speed_diff_thresh = float(getattr(v2x_cfg, 'overtake_speed_diff_threshold', 3.0))
        cross_velocity_thresh = float(getattr(v2x_cfg, 'cross_velocity_threshold', 2.5))

        is_stationary_lead = (is_leading_ahead and lead_speed < 1.5 and min_d < follow_start)

        is_large_speed_gap = (is_leading_ahead and (min_d < follow_start) and
                              (min_rel_fwd >= overtake_speed_diff_thresh) and
                              (min_rel_lat <= cross_velocity_thresh))
        should_direct_overtake = (is_leading_ahead and (min_d < follow_start) and
                                  (is_large_speed_gap or is_stationary_lead) and
                                  (min_rel_lat <= cross_velocity_thresh))

        denom_follow = max(0.001, follow_start - follow_brake)

        # Mode Transition Logic
        if self.mode == "OVERTAKING":
            self.emergency_brake_since = None
            self.vehicle_radius = vehicle_radius_overtake

            # 先行車が自車前方に存在中（rel_fwd > 0.0）は過剰高速（45km/h）突入を防ぎ、コントロールされた速度（lead + 10km/h）に抑える
            if is_leading_ahead and min_lead_rel_fwd > 0.0:
                controlled_v = max(kmh_to_m_per_sec(15.0), lead_speed + kmh_to_m_per_sec(10.0))
                self.speed_limit = min(v_max_normal, controlled_v)
            else:
                self.speed_limit = float('inf')

            # 追い越し完了条件：
            # 1. 自車が先行車の前方へ抜け出た場合 (min_lead_rel_fwd < -2.0m)
            # 2. 前方に車が存在しない、または遠方 (min_d >= 15.0m) へ離脱した場合
            is_passed = (is_leading_ahead and min_lead_rel_fwd < -2.0)
            is_far_away = (not is_leading_ahead or min_d >= follow_start)

            if is_passed or is_far_away:
                self.mode = "NORMAL"
                self.vehicle_radius = vehicle_radius_normal
                self.speed_limit = float('inf')
                self.following_since = None
                self._log('info', f"[V2X] Overtaking COMPLETE! (passed={is_passed}, d={min_d:.1f}m). Back to NORMAL.")

        elif (is_stationary_lead or should_direct_overtake) and self.mode in ("NORMAL", "FOLLOWING", "EMERGENCY_BRAKE"):
            self.mode = "OVERTAKING"
            self.vehicle_radius = vehicle_radius_overtake
            controlled_v = max(kmh_to_m_per_sec(15.0), lead_speed + kmh_to_m_per_sec(10.0))
            self.speed_limit = min(v_max_normal, controlled_v)
            self.emergency_brake_since = None
            self._log('info',
                f"[V2X] Instant Direct OVERTAKING (Stationary/Stuck Lead Avoidance): d={min_d:.1f}m lead_v={lead_speed*3.6:.1f}km/h rad={self.vehicle_radius:.2f}m",
                throttle_duration_sec=1.0)

        elif min_ttc < ttc_thresh or (is_leading_ahead and min_d < follow_brake):
            if self.mode != "EMERGENCY_BRAKE":
                self.mode = "EMERGENCY_BRAKE"
                self.emergency_brake_since = now_sec

            eb_duration = now_sec - (self.emergency_brake_since or now_sec)

            is_startup_acceleration_phase = (
                ego_wp_id < 40 and
                self.motion_start_time is not None and
                (now_sec - self.motion_start_time < 12.0)
            )

            if is_startup_acceleration_phase and lead_speed >= 1.5:
                self.speed_limit = max(lead_speed, 5.0)
                self._log('info',
                    f"[V2X] Startup Acceleration Phase Protection: Maintaining lead speed {lead_speed*3.6:.1f} km/h, no eb stall.",
                    throttle_duration_sec=1.0)
            else:
                self.speed_limit = v_min_safe_mps

            if is_stationary_lead or eb_duration >= overtake_patience:
                self.mode = "OVERTAKING"
                self.vehicle_radius = vehicle_radius_overtake
                controlled_v = max(kmh_to_m_per_sec(15.0), lead_speed + kmh_to_m_per_sec(10.0))
                self.speed_limit = min(v_max_normal, controlled_v)
                self.emergency_brake_since = None
                self._log('info',
                    f"[V2X] EMERGENCY_BRAKE -> OVERTAKING (Immediate Stationary bypass): dur={eb_duration:.1f}s d={min_d:.1f}m rad={self.vehicle_radius:.2f}m")
            else:
                self._log('warn',
                    f"[V2X] EMERGENCY_BRAKE: d={min_d:.1f}m ttc={min_ttc:.1f}s (dur={eb_duration:.1f}s)",
                    throttle_duration_sec=1.0)

        elif is_leading_ahead and min_d < follow_start and self.mode == "NORMAL":
            self.mode = "FOLLOWING"
            self.following_since = now_sec
            self.emergency_brake_since = None
            target_follow_speed = max(lead_speed, v_min_safe_mps)
            ratio = max(0.0, min(1.0, (min_d - follow_brake) / denom_follow))
            max_allowable_speed = target_follow_speed + ratio * kmh_to_m_per_sec(12.0)
            self.speed_limit = min(v_max_normal, max_allowable_speed)
            self._log('info',
                f"[V2X] FOLLOWING (ACC): d={min_d:.1f}m lead_v={lead_speed*3.6:.1f}km/h v_lim={self.speed_limit*3.6:.1f}km/h",
                throttle_duration_sec=1.0)

        elif self.mode == "FOLLOWING":
            self.emergency_brake_since = None
            if not is_leading_ahead or min_d >= follow_start:
                self.mode = "NORMAL"
                self.speed_limit = float('inf')
                self.following_since = None
                self._log('info', "[V2X] Back to NORMAL (vehicle left front zone)")
            else:
                target_follow_speed = max(lead_speed, v_min_safe_mps)
                ratio = max(0.0, min(1.0, (min_d - follow_brake) / denom_follow))
                max_allowable_speed = target_follow_speed + ratio * kmh_to_m_per_sec(12.0)
                self.speed_limit = min(v_max_normal, max_allowable_speed)

                following_duration = now_sec - (self.following_since or now_sec)
                if (following_duration >= overtake_patience and min_d <= overtake_gap_min):
                    self.mode = "OVERTAKING"
                    self.vehicle_radius = vehicle_radius_overtake
                    controlled_v = max(kmh_to_m_per_sec(15.0), lead_speed + kmh_to_m_per_sec(10.0))
                    self.speed_limit = min(v_max_normal, controlled_v)
                    self._log('info', f"[V2X] OVERTAKING: following={following_duration:.1f}s d={min_d:.1f}m")

        else:
            self.mode = "NORMAL"
            self.speed_limit = float('inf')
            self.following_since = None
            self.emergency_brake_since = None

        return V2XStateOutput(
            mode=self.mode,
            speed_limit=self.speed_limit,
            vehicle_radius=self.vehicle_radius,
            is_emergency_brake=(self.mode == "EMERGENCY_BRAKE")
        )
