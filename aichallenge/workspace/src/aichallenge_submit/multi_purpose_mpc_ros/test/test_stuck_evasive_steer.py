import unittest
import math

def compute_evasive_steer(ego_x, ego_y, ego_yaw, lead_x, lead_y, rev_steer_angle=0.35):
    dx = lead_x - ego_x
    dy = lead_y - ego_y

    fwd_sin = math.sin(ego_yaw)
    fwd_cos = math.cos(ego_yaw)
    left_cos = -fwd_sin
    left_sin = fwd_cos

    # Projection onto ego left-axis
    lead_rel_y = dx * left_cos + dy * left_sin

    # If lead vehicle is on left (lead_rel_y > 0), positive steer (+angle) turns nose right in reverse
    # If lead vehicle is on right (lead_rel_y <= 0), negative steer (-angle) turns nose left in reverse
    if lead_rel_y > 0:
        return rev_steer_angle
    else:
        return -rev_steer_angle

class TestStuckEvasiveSteer(unittest.TestCase):
    def test_lead_vehicle_on_left(self):
        # Ego heading 0 (facing +X). Lead at X=2, Y=1 (left side) -> lead_rel_y = +1.0
        steer = compute_evasive_steer(ego_x=0.0, ego_y=0.0, ego_yaw=0.0, lead_x=2.0, lead_y=1.0)
        self.assertAlmostEqual(steer, 0.35)

    def test_lead_vehicle_on_right(self):
        # Ego heading 0 (facing +X). Lead at X=2, Y=-1 (right side) -> lead_rel_y = -1.0
        steer = compute_evasive_steer(ego_x=0.0, ego_y=0.0, ego_yaw=0.0, lead_x=2.0, lead_y=-1.0)
        self.assertAlmostEqual(steer, -0.35)

if __name__ == '__main__':
    unittest.main()
