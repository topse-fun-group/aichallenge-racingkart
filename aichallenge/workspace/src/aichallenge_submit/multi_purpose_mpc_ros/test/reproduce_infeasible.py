import sys
import os
import numpy as np
from scipy import sparse

pkg_path = "/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros"
sys.path.insert(0, pkg_path)

from multi_purpose_mpc_ros.core.map import Map
from multi_purpose_mpc_ros.core.reference_path import ReferencePath
from multi_purpose_mpc_ros.core.spatial_bicycle_models import BicycleModel
from multi_purpose_mpc_ros.core.MPC import MPC
from multi_purpose_mpc_ros.core.utils import load_ref_path

map_path = os.path.join(pkg_path, "env/final_ver3/occupancy_grid_map.yaml")
csv_path = os.path.join(pkg_path, "env/final_ver3/traj_out_in_middle.csv")

m = Map(map_path)
wp_x, wp_y, _, _ = load_ref_path(csv_path)

ref_path = ReferencePath(
    map=m,
    wp_x=wp_x,
    wp_y=wp_y,
    resolution=0.4,
    smoothing_distance=3,
    max_width=3.0,
    circular=True
)

car = BicycleModel(ref_path, length=1.087, width=1.0, Ts=0.025)

state_constraints = {
    "xmin": np.array([-np.inf, -np.inf, -np.inf]),
    "xmax": np.array([np.inf, np.inf, np.inf])
}
delta_max = np.deg2rad(32.0)
input_constraints = {
    "umin": np.array([0.0, -np.tan(delta_max) / car.length]),
    "umax": np.array([45.0, np.tan(delta_max) / car.length])
}

mpc = MPC(
    model=car,
    N=20,
    Q=sparse.diags([300.0, 600.0, 100.0]),
    R=sparse.diags([0.1, 1200.0]),
    QN=sparse.diags([300.0, 600.0, 100.0]),
    StateConstraints=state_constraints,
    InputConstraints=input_constraints,
    ay_max=9.5,
    max_steering_rate=2.5,
    wp_id_offset=1,
    use_obstacle_avoidance=True,
    use_path_constraints_topic=False,
    use_max_kappa_pred=False
)

speed_profile_constraints = {
    "a_min": -1.6, "a_max": 2.5,
    "v_min": 0.0, "v_max": 45.0, "ay_max": 9.5}
ref_path.compute_speed_profile(speed_profile_constraints)
ref_path.update_simple_path_constraints(20, car.safety_margin)

# Test solving for WP 691 to 695 with conditions from autoware.log:
for wp_id, e_y0, e_psi0, prev_steer in [
    (691, 0.5709, 0.0765, -0.1341),
    (692, 0.6061, 0.0847, -0.1388),
    (693, 0.6625, 0.0893, -0.2324),
    (694, 0.7083, 0.0920, -0.3421),
    (695, 0.7564, 0.0678, -0.5484),
]:
    target_wp = ref_path.waypoints[wp_id]
    # Set temporal pose near target_wp with lateral offset e_y0
    x_car = target_wp.x - e_y0 * np.sin(target_wp.psi)
    y_car = target_wp.y + e_y0 * np.cos(target_wp.psi)
    psi_car = target_wp.psi + e_psi0

    car.temporal_state.x = x_car
    car.temporal_state.y = y_car
    car.temporal_state.psi = psi_car
    car._wp_id_initialized = True
    car.wp_id = wp_id
    car.s = car.get_s_at_waypoint(wp_id)
    car.current_waypoint = target_wp

    mpc.previous_steering = prev_steer

    print(f"\n--- Testing WP {wp_id} (e_y0={e_y0:.4f}, e_psi0={e_psi0:.4f}, prev_steer={prev_steer:.4f}) ---")
    try:
        u, max_d = mpc.get_control()
        print(f"Result: Solved! u={u}, max_delta={max_d:.4f}")
    except Exception as e:
        print(f"Result: Exception! {type(e).__name__}: {e}")
