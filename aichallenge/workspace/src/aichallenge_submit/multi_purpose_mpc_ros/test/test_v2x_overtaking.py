import unittest
import math

class StubV2XConfig:
    follow_distance_start = 15.0
    follow_distance_brake = 5.0
    v_min_safe = 8.0  # km/h
    ttc_threshold = 1.5
    forward_cos_threshold = 0.65
    startup_suppress_sec = 3.0
    overtake_patience = 3.0
    overtake_gap_min = 10.0
    overtake_clearance = 8.0
    vehicle_radius = 1.0
    vehicle_radius_overtake = 0.65
    overtake_speed_diff_threshold = 3.0  # m/s (10.8 km/h)
    cross_velocity_threshold = 2.5        # m/s (9.0 km/h)

def evaluate_v2x_mode(ego_speed_mps, ego_x, ego_y, ego_yaw, lead_vx_mps, lead_vy_mps, lead_x, lead_y, current_mode="NORMAL"):
    v2x_cfg = StubV2XConfig()
    follow_start = v2x_cfg.follow_distance_start
    follow_brake = v2x_cfg.follow_distance_brake
    v_min_safe_mps = v2x_cfg.v_min_safe / 3.6
    ttc_thresh = v2x_cfg.ttc_threshold
    fwd_cos_thresh = v2x_cfg.forward_cos_threshold
    vehicle_radius_overtake = v2x_cfg.vehicle_radius_overtake
    overtake_speed_diff_thresh = v2x_cfg.overtake_speed_diff_threshold
    cross_velocity_thresh = v2x_cfg.cross_velocity_threshold

    dx = lead_x - ego_x
    dy = lead_y - ego_y
    d = math.hypot(dx, dy)

    fwd_cos = math.cos(ego_yaw)
    fwd_sin = math.sin(ego_yaw)

    dot = dx * fwd_cos + dy * fwd_sin
    cos_angle = dot / max(d, 0.001)
    is_leading_ahead = (d >= 0.5) and (cos_angle >= fwd_cos_thresh)

    ego_vx = ego_speed_mps * fwd_cos
    ego_vy = ego_speed_mps * fwd_sin

    rel_approach = (ego_vx - lead_vx_mps) * fwd_cos + (ego_vy - lead_vy_mps) * fwd_sin
    rel_cross = abs((ego_vx - lead_vx_mps) * (-fwd_sin) + (ego_vy - lead_vy_mps) * fwd_cos)

    min_ttc = (d / rel_approach) if rel_approach > 0.0 else float('inf')

    is_large_speed_gap = (
        is_leading_ahead and
        (d < follow_start) and
        (rel_approach >= overtake_speed_diff_thresh) and
        (rel_cross <= cross_velocity_thresh)
    )

    mode = current_mode
    speed_limit = float('inf')
    radius = v2x_cfg.vehicle_radius

    if is_large_speed_gap and mode in ("NORMAL", "FOLLOWING"):
        mode = "OVERTAKING"
        radius = vehicle_radius_overtake
        speed_limit = float('inf')
    elif min_ttc < ttc_thresh or (is_leading_ahead and d < follow_brake):
        mode = "EMERGENCY_BRAKE"
        speed_limit = v_min_safe_mps
    elif is_leading_ahead and d < follow_start and mode == "NORMAL":
        mode = "FOLLOWING"
        speed_limit = 10.0
    return mode, speed_limit, radius

class TestV2XOvertaking(unittest.TestCase):
    def test_stopped_vehicle_direct_overtaking(self):
        # Ego: 10.0 m/s (36 km/h), Lead: 0.0 m/s (0 km/h) at 12m ahead -> rel_fwd = 10.0 >= 3.0, rel_lat = 0.0 <= 2.5
        mode, speed_limit, radius = evaluate_v2x_mode(
            ego_speed_mps=10.0, ego_x=0.0, ego_y=0.0, ego_yaw=0.0,
            lead_vx_mps=0.0, lead_vy_mps=0.0, lead_x=12.0, lead_y=0.0, current_mode="NORMAL")
        self.assertEqual(mode, "OVERTAKING")
        self.assertEqual(speed_limit, float('inf'))
        self.assertEqual(radius, 0.65)

    def test_slow_vehicle_direct_overtaking(self):
        # Ego: 10.0 m/s, Lead: vx=3.0 m/s, vy=0.0 m/s at 12m ahead -> rel_fwd = 7.0 >= 3.0, rel_lat = 0.0 <= 2.5
        mode, speed_limit, radius = evaluate_v2x_mode(
            ego_speed_mps=10.0, ego_x=0.0, ego_y=0.0, ego_yaw=0.0,
            lead_vx_mps=3.0, lead_vy_mps=0.0, lead_x=12.0, lead_y=0.0, current_mode="NORMAL")
        self.assertEqual(mode, "OVERTAKING")
        self.assertEqual(speed_limit, float('inf'))
        self.assertEqual(radius, 0.65)

    def test_cross_moving_vehicle_blocks_direct_overtaking(self):
        # Ego: 10.0 m/s (facing +X), Lead at (12, 0) moving sideways with vy = -4.0 m/s (crossing from left to right)
        # OVERTAKING is blocked by cross_velocity_threshold check, resulting in non-OVERTAKING mode (EMERGENCY_BRAKE / FOLLOWING)
        mode, speed_limit, radius = evaluate_v2x_mode(
            ego_speed_mps=10.0, ego_x=0.0, ego_y=0.0, ego_yaw=0.0,
            lead_vx_mps=0.0, lead_vy_mps=-4.0, lead_x=12.0, lead_y=0.0, current_mode="NORMAL")
        self.assertNotEqual(mode, "OVERTAKING")
        self.assertEqual(radius, 1.0)  # Safe radius preserved

    def test_similar_speed_following(self):
        # Ego: 10.0 m/s, Lead: vx=9.0 m/s, vy=0.0 m/s at 12m ahead -> rel_fwd = 1.0 < 3.0
        mode, speed_limit, radius = evaluate_v2x_mode(
            ego_speed_mps=10.0, ego_x=0.0, ego_y=0.0, ego_yaw=0.0,
            lead_vx_mps=9.0, lead_vy_mps=0.0, lead_x=12.0, lead_y=0.0, current_mode="NORMAL")
        self.assertEqual(mode, "FOLLOWING")
        self.assertEqual(radius, 1.0)

if __name__ == '__main__':
    unittest.main()
