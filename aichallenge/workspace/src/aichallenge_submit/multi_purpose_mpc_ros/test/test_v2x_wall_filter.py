import unittest
import math

class StubWaypoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class StubReferencePath:
    def __init__(self, waypoints):
        self.waypoints = waypoints
        self.circular = True
        self.n_base_waypoints = len(waypoints)

class StubCar:
    def __init__(self, wp_id, reference_path):
        self.wp_id = wp_id
        self.reference_path = reference_path

    def get_closest_waypoint(self, x, y):
        # Linear search for closest waypoint
        best_id = 0
        min_d = float('inf')
        for i, wp in enumerate(self.reference_path.waypoints):
            d = math.hypot(wp.x - x, wp.y - y)
            if d < min_d:
                min_d = d
                best_id = i
        return best_id

class StubV2XConfig:
    follow_distance_start = 15.0
    follow_distance_brake = 5.0
    v_min_safe = 8.0
    ttc_threshold = 1.5
    forward_cos_threshold = 0.65
    overtake_speed_diff_threshold = 3.0
    cross_velocity_threshold = 2.5
    vehicle_radius = 1.0
    vehicle_radius_overtake = 0.65
    wp_lookahead_max = 30
    wp_dist_max = 2.5

def filter_and_evaluate(car, ego_speed_mps, ego_x, ego_y, ego_yaw, lead_x, lead_y, lead_vx=0.0, lead_vy=0.0):
    v2x_cfg = StubV2XConfig()
    wp_lookahead_max = v2x_cfg.wp_lookahead_max
    wp_dist_max = v2x_cfg.wp_dist_max

    ego_wp_id = car.wp_id
    n_base = car.reference_path.n_base_waypoints

    dx = lead_x - ego_x
    dy = lead_y - ego_y
    d = math.hypot(dx, dy)

    if d < 0.5:
        return "NORMAL", float('inf')

    # WP_ID Filter
    lead_wp_id = car.get_closest_waypoint(lead_x, lead_y)
    wp_diff = (lead_wp_id - (ego_wp_id % n_base)) % n_base

    if not (1 <= wp_diff <= wp_lookahead_max):
        return "NORMAL", float('inf')  # Filtered out (Wall / Opposite Lane)

    lead_wp = car.reference_path.waypoints[lead_wp_id % n_base]
    if math.hypot(lead_x - lead_wp.x, lead_y - lead_wp.y) > wp_dist_max:
        return "NORMAL", float('inf')  # Filtered out (Off-track)

    fwd_cos = math.cos(ego_yaw)
    fwd_sin = math.sin(ego_yaw)
    dot = dx * fwd_cos + dy * fwd_sin
    cos_angle = dot / max(d, 0.001)

    if cos_angle < v2x_cfg.forward_cos_threshold:
        return "NORMAL", float('inf')

    rel_approach = (ego_speed_mps * fwd_cos - lead_vx) * fwd_cos + (ego_speed_mps * fwd_sin - lead_vy) * fwd_sin
    rel_cross = abs((ego_speed_mps * fwd_cos - lead_vx) * (-fwd_sin) + (ego_speed_mps * fwd_sin - lead_vy) * fwd_cos)

    if rel_approach >= v2x_cfg.overtake_speed_diff_threshold and rel_cross <= v2x_cfg.cross_velocity_threshold:
        return "OVERTAKING", float('inf')
    elif d < v2x_cfg.follow_distance_start:
        return "FOLLOWING", 10.0
    return "NORMAL", float('inf')

class TestV2XWallFilter(unittest.TestCase):
    def setUp(self):
        # Create a loop of 100 waypoints (0 to 99) in a oval shape
        waypoints = []
        for i in range(50):  # Straight 1 (0 to 49)
            waypoints.append(StubWaypoint(x=float(i), y=0.0))
        for i in range(50):  # Straight 2 parallel at y=10.0 (50 to 99) in opposite direction
            waypoints.append(StubWaypoint(x=float(49 - i), y=10.0))
        self.ref_path = StubReferencePath(waypoints)

    def test_same_lane_lead_vehicle_detected(self):
        # Ego at WP 10 (x=10, y=0, yaw=0). Lead at WP 20 (x=20, y=0) -> wp_diff = 10 <= 30 -> Detected!
        car = StubCar(wp_id=10, reference_path=self.ref_path)
        mode, _ = filter_and_evaluate(car, ego_speed_mps=10.0, ego_x=10.0, ego_y=0.0, ego_yaw=0.0, lead_x=20.0, lead_y=0.0)
        self.assertEqual(mode, "OVERTAKING")

    def test_wall_behind_vehicle_filtered_out(self):
        # Ego at WP 10 (x=10, y=0, yaw=0). Wall-behind vehicle at WP 89 (x=10, y=10)
        # Direct Euclidean distance = 10m (close!), but WP_ID = 89 -> wp_diff = (89 - 10) = 79 > 30 -> Filtered OUT!
        car = StubCar(wp_id=10, reference_path=self.ref_path)
        mode, _ = filter_and_evaluate(car, ego_speed_mps=10.0, ego_x=10.0, ego_y=0.0, ego_yaw=0.0, lead_x=10.0, lead_y=10.0)
        self.assertEqual(mode, "NORMAL")  # Successfully ignored!

if __name__ == '__main__':
    unittest.main()
