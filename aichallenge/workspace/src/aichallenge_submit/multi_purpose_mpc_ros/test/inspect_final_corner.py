import sys
import os
import numpy as np

pkg_path = "/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros"
sys.path.insert(0, pkg_path)

from multi_purpose_mpc_ros.core.map import Map
from multi_purpose_mpc_ros.core.reference_path import ReferencePath
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

N = 20
safety_margin = 0.6
ref_path.update_simple_path_constraints(N, safety_margin)

print(f"Path constraints shape: ub={ref_path.path_constraints[0].shape}, lb={ref_path.path_constraints[1].shape}")

for wp_id in range(688, 697):
    ub = ref_path.path_constraints[0][wp_id]
    lb = ref_path.path_constraints[1][wp_id]
    print(f"\n--- WP {wp_id} ---")
    print("ub horizon:", [round(x, 3) for x in ub])
    print("lb horizon:", [round(x, 3) for x in lb])
