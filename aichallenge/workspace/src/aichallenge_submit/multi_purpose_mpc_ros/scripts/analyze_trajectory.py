#!/usr/bin/env python3
import os
import sys
import time
import math
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend for saving PNGs
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.collections import LineCollection
import matplotlib.image as mpimg

# Ensure robust font rendering across Docker containers
matplotlib.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Liberation Sans', 'sans-serif']
matplotlib.rcParams['axes.unicode_minus'] = False

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from autoware_auto_control_msgs.msg import AckermannControlCommand
from visualization_msgs.msg import MarkerArray


class TrajectoryAnalyzer(Node):
    def __init__(self, output_dir: str = None):
        super().__init__("trajectory_analyzer")
        self.declare_parameter("output_dir", os.environ.get("LOG_DIR", "/output"))
        if output_dir is None:
            output_dir = self.get_parameter("output_dir").get_parameter_value().string_value
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)

        self.get_logger().info(f"Initializing TrajectoryAnalyzer node... Saving plots to {self.output_dir}")

        # Data storage
        self.timestamps = []
        self.x_data = []
        self.y_data = []
        self.v_data = []  # m/s
        self.a_cmd_data = []  # m/s^2
        self.steer_cmd_data = []  # rad

        self.ref_x = []
        self.ref_y = []
        self.ref_s = []

        # Course map data
        self.map_data = None
        self.map_extent = None
        self.track_boundaries = []  # list of (x_arr, y_arr) for inner/outer walls

        # Load Full Reference Path & Map Data from package files
        self.load_static_course_data()

        self.lap_start_times = []
        self.lap_times = []
        # Exact Main Straight Start/Finish position for Kart course
        self.start_x = 89633.29
        self.start_y = 43127.57
        self.last_cross_time = 0.0

        # ROS 2 Subscriptions
        self.sub_odom = self.create_subscription(
            Odometry,
            "/localization/kinematic_state",
            self.odom_callback,
            10
        )
        self.sub_cmd = self.create_subscription(
            AckermannControlCommand,
            "/control/command/control_cmd",
            self.cmd_callback,
            10
        )
        self.sub_ref = self.create_subscription(
            MarkerArray,
            "/mpc/ref_path",
            self.ref_callback,
            1
        )

        self.current_cmd_acc = 0.0
        self.current_cmd_steer = 0.0

        # Periodic status logger & plot saver every 10 seconds
        self.timer = self.create_timer(10.0, self.save_analysis_plots)

    def load_static_course_data(self):
        """Loads static full reference raceline and occupancy grid course map."""
        possible_dirs = [
            "/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros",
            "/home/ci008043/workspace/aichallenge-racingkart/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros",
            os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        ]

        # 1. Load Reference Raceline (traj_mincurv.csv)
        ref_loaded = False
        for pdir in possible_dirs:
            for ver in ["final_ver3", "final_ver2", "final", "final_ver4"]:
                csv_file = os.path.join(pdir, "env", ver, "traj_mincurv.csv")
                if os.path.isfile(csv_file):
                    try:
                        data = np.genfromtxt(csv_file, delimiter=',', names=True)
                        if 'x_m' in data.dtype.names and 'y_m' in data.dtype.names:
                            self.ref_x = list(data['x_m'])
                            self.ref_y = list(data['y_m'])
                            if 's_m' in data.dtype.names:
                                self.ref_s = list(data['s_m'])
                            self.get_logger().info(f"Loaded full reference path ({len(self.ref_x)} points) from {csv_file}")
                            ref_loaded = True
                            break
                    except Exception as e:
                        self.get_logger().warn(f"Failed to parse {csv_file}: {e}")
            if ref_loaded:
                break

        # 2. Load Occupancy Grid Map (occupancy_grid_map.yaml & .pgm)
        for pdir in possible_dirs:
            for ver in ["final_ver3", "final_ver2", "final", "final_ver4"]:
                yaml_file = os.path.join(pdir, "env", ver, "occupancy_grid_map.yaml")
                if os.path.isfile(yaml_file):
                    try:
                        with open(yaml_file, 'r') as f:
                            meta = yaml.safe_load(f)
                        pgm_file = os.path.join(os.path.dirname(yaml_file), meta['image'])
                        if os.path.isfile(pgm_file):
                            img = mpimg.imread(pgm_file)
                            if img.ndim == 3:
                                img = img[:, :, 0]
                            res = float(meta['resolution'])
                            origin = meta['origin']
                            h, w = img.shape
                            self.map_extent = [
                                origin[0],
                                origin[0] + w * res,
                                origin[1],
                                origin[1] + h * res
                            ]
                            self.map_data = np.flipud(img)
                            self.get_logger().info(f"Loaded course map ({w}x{h}, res={res}m) from {pgm_file}")
                            return
                    except Exception as e:
                        self.get_logger().warn(f"Failed to load course map from {yaml_file}: {e}")

    def cmd_callback(self, msg: AckermannControlCommand):
        self.current_cmd_acc = msg.longitudinal.acceleration
        self.current_cmd_steer = msg.lateral.steering_tire_angle

    def ref_callback(self, msg: MarkerArray):
        ref_x = []
        ref_y = []
        for marker in msg.markers:
            if len(marker.points) >= 2:
                for pt in marker.points:
                    ref_x.append(pt.x)
                    ref_y.append(pt.y)
        if len(ref_x) > len(self.ref_x):
            self.ref_x = ref_x
            self.ref_y = ref_y
            self.get_logger().info(f"Captured updated reference path with {len(self.ref_x)} points.")

    def odom_callback(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        v = math.hypot(vx, vy)

        # Ignore initial uninitialized zero coordinates or NaN
        if abs(x) < 1000.0 or abs(y) < 1000.0 or math.isnan(x) or math.isnan(y):
            return

        if self.last_cross_time == 0.0:
            self.last_cross_time = t

        self.timestamps.append(t)
        self.x_data.append(x)
        self.y_data.append(y)
        self.v_data.append(v)
        self.a_cmd_data.append(self.current_cmd_acc)
        self.steer_cmd_data.append(self.current_cmd_steer)

        # Lap completion detection (distance to start point < 5.0m after minimum 15s)
        dist_to_start = math.hypot(x - self.start_x, y - self.start_y)
        if len(self.timestamps) > 100 and (t - self.last_cross_time) > 15.0:
            if dist_to_start < 5.0:
                lap_time = t - self.last_cross_time
                self.lap_times.append(lap_time)
                self.last_cross_time = t
                self.get_logger().info(f"=== LAP COMPLETED: Lap {len(self.lap_times)} Time = {lap_time:.2f} s ===")
                self.save_analysis_plots()

    def detect_key_points(self, s_arr, v_arr, a_arr, steer_arr, dt_arr, ey_arr):
        """
        Detects Top-K prominent points with Non-Maximum Suppression (NMS) to avoid clutter:
        1. Time-Loss Opportunities (T1-T4, Cyan Star [*]):
           - Lowest apex speeds in corners (over-braking)
           - Sluggish exit acceleration
           - Maximum lateral path deviation
        2. Control Instabilities (U1-U4, Red Triangle [^]):
           - Highest steering oscillation / rapid hunting peaks
           - Acceleration command chattering
        """
        n = len(s_arr)
        raw_time_loss = []
        raw_unstable = []

        if n < 30:
            return [], []

        dt_safe = np.clip(dt_arr, 0.01, 1.0)
        d_steer = np.diff(steer_arr, prepend=steer_arr[0]) / dt_safe
        d_accel = np.diff(a_arr, prepend=a_arr[0]) / dt_safe

        # 1. Control Instabilities
        window = 10
        steer_osc_mask = np.zeros(n, dtype=bool)
        for i in range(window, n - window):
            local_rates = d_steer[i - window:i + window]
            sign_changes = np.count_nonzero(np.diff(np.sign(local_rates)))
            rate_mag = np.max(np.abs(local_rates))
            if rate_mag > 0.6 and sign_changes >= 4:
                steer_osc_mask[i] = True

        in_osc = False
        osc_start = 0
        for i in range(n):
            if steer_osc_mask[i] and not in_osc:
                in_osc = True
                osc_start = i
            elif not steer_osc_mask[i] and in_osc:
                in_osc = False
                peak_idx = osc_start + np.argmax(np.abs(d_steer[osc_start:i]))
                sev = float(np.abs(d_steer[peak_idx]))
                t_start_val = self.timestamps[osc_start] - self.timestamps[0]
                t_end_val = self.timestamps[min(n-1, i)] - self.timestamps[0]
                dur_sec = max(0.05, t_end_val - t_start_val)
                # Count zero-crossings to estimate oscillation frequency (Hz)
                segment_rates = d_steer[osc_start:i]
                sign_changes = np.count_nonzero(np.diff(np.sign(segment_rates)))
                freq_hz = max(1.0, (sign_changes / 2.0) / dur_sec)
                raw_unstable.append({
                    'idx': peak_idx,
                    's': s_arr[peak_idx],
                    'x': self.x_data[peak_idx],
                    'y': self.y_data[peak_idx],
                    'type': 'Steer Hunting',
                    'desc': f"Steer Hunting ({sev:.1f} rad/s, freq: {freq_hz:.1f}Hz, dur: {dur_sec:.2f}s)",
                    'severity': sev,
                    'freq_hz': freq_hz,
                    'duration': dur_sec,
                    's_start': s_arr[osc_start],
                    's_end': s_arr[min(n-1, i)],
                    't_start': t_start_val,
                    't_end': t_end_val
                })

        for i in range(window, n - window, window):
            local_da = d_accel[i - window:i + window]
            std_val = float(np.std(local_da))
            if std_val > 6.5:
                t_start_val = max(0.0, self.timestamps[i - window] - self.timestamps[0])
                t_end_val = self.timestamps[min(n-1, i + window)] - self.timestamps[0]
                dur_sec = max(0.05, t_end_val - t_start_val)
                raw_unstable.append({
                    'idx': i,
                    's': s_arr[i],
                    'x': self.x_data[i],
                    'y': self.y_data[i],
                    'type': 'Throttle Jitter',
                    'desc': f"Throttle Jitter (std: {std_val:.1f}, dur: {dur_sec:.2f}s)",
                    'severity': std_val * 0.2,
                    'duration': dur_sec,
                    's_start': s_arr[max(0, i - window)],
                    's_end': s_arr[min(n-1, i + window)],
                    't_start': t_start_val,
                    't_end': t_end_val
                })

        # 2. Time-Loss Opportunities
        for i in range(15, n - 15):
            if v_arr[i] < v_arr[i - 1] and v_arr[i] < v_arr[i + 1]:
                if v_arr[i] == np.min(v_arr[max(0, i - 15):min(n, i + 15)]):
                    if v_arr[i] < 35.0:
                        raw_time_loss.append({
                            'idx': i,
                            's': s_arr[i],
                            'x': self.x_data[i],
                            'y': self.y_data[i],
                            'type': 'Slow Corner',
                            'desc': f"Low Apex Speed ({v_arr[i]:.1f} km/h)",
                            'severity': float(35.0 - v_arr[i])
                        })

        for i in range(15, n - 15):
            if v_arr[i] < 45.0 and abs(steer_arr[i]) < 0.08 and 0.0 <= a_arr[i] < 0.4:
                raw_time_loss.append({
                    'idx': i,
                    's': s_arr[i],
                    'x': self.x_data[i],
                    'y': self.y_data[i],
                    'type': 'Sluggish Accel',
                    'desc': f"Slow Exit Accel ({a_arr[i]:.2f} m/s^2)",
                    'severity': float(1.0 - a_arr[i]) * 10.0
                })

        if len(ey_arr) == n:
            for i in range(15, n - 15):
                if ey_arr[i] > 0.45 and ey_arr[i] == np.max(ey_arr[max(0, i - 15):min(n, i + 15)]):
                    raw_time_loss.append({
                        'idx': i,
                        's': s_arr[i],
                        'x': self.x_data[i],
                        'y': self.y_data[i],
                        'type': 'Wide Line',
                        'desc': f"Path Deviation ({ey_arr[i]:.2f} m)",
                        'severity': float(ey_arr[i]) * 15.0
                    })

        def filter_top_k(points_list, min_dist=40.0, k=4):
            if not points_list:
                return []
            sorted_pts = sorted(points_list, key=lambda p: p['severity'], reverse=True)
            selected = []
            for p in sorted_pts:
                if not any(abs(p['s'] - s['s']) < min_dist for s in selected):
                    selected.append(p)
                if len(selected) >= k:
                    break
            return sorted(selected, key=lambda p: p['s'])

        time_loss_points = filter_top_k(raw_time_loss, min_dist=40.0, k=4)
        unstable_points = filter_top_k(raw_unstable, min_dist=35.0, k=4)

        for idx, p in enumerate(time_loss_points, 1):
            p['tag'] = f"T{idx}"
        for idx, p in enumerate(unstable_points, 1):
            p['tag'] = f"U{idx}"

        return time_loss_points, unstable_points

    def save_analysis_plots(self):
        if len(self.x_data) < 20:
            return

        fig, axes = plt.subplots(2, 2, figsize=(18, 13))
        t_arr = np.array(self.timestamps) - self.timestamps[0]
        x_arr = np.array(self.x_data)
        y_arr = np.array(self.y_data)
        v_arr = np.array(self.v_data) * 3.6  # km/h
        a_arr = np.array(self.a_cmd_data)
        steer_arr = np.array(self.steer_cmd_data)
        dt_arr = np.diff(t_arr, prepend=0.05)

        # Calculate distance s
        ds = np.hypot(np.diff(x_arr, prepend=x_arr[0]), np.diff(y_arr, prepend=y_arr[0]))
        s_arr = np.cumsum(ds)

        # Calculate lateral deviation ey
        ey_arr = np.zeros(len(x_arr))
        if len(self.ref_x) > 0:
            ref_pts = np.column_stack((self.ref_x, self.ref_y))
            for i in range(len(x_arr)):
                pt = np.array([x_arr[i], y_arr[i]])
                dists = np.hypot(ref_pts[:, 0] - pt[0], ref_pts[:, 1] - pt[1])
                ey_arr[i] = np.min(dists)

        # Detect Clean Top-K Points
        time_loss_pts, unstable_pts = self.detect_key_points(s_arr, v_arr, a_arr, steer_arr, dt_arr, ey_arr)

        # =========================================================
        # Plot 1: 2D Course Trajectory with High-Contrast Map & Reference
        # =========================================================
        ax1 = axes[0, 0]

        # A. Draw Course Outlines (Track Boundaries / Walls)
        if self.map_data is not None and self.map_extent is not None:
            h, w = self.map_data.shape
            x_coords = np.linspace(self.map_extent[0], self.map_extent[1], w)
            y_coords = np.linspace(self.map_extent[2], self.map_extent[3], h)
            is_drivable = (self.map_data > 128).astype(float)
            # Track Surface Fill (Soft slate gray)
            ax1.imshow(is_drivable, extent=self.map_extent, origin='lower', cmap='gray', alpha=0.15, zorder=1)
            # Track Boundary Walls (Solid dark outline)
            ax1.contour(x_coords, y_coords, is_drivable, levels=[0.5], colors=['#334155'], linewidths=1.6, alpha=0.85, zorder=2)
        elif len(self.ref_x) > 10:
            # Fallback boundary walls
            ref_arr = np.column_stack((self.ref_x, self.ref_y))
            dx = np.gradient(ref_arr[:, 0])
            dy = np.gradient(ref_arr[:, 1])
            norms = np.hypot(dx, dy) + 1e-6
            nx = -dy / norms
            ny = dx / norms
            track_half_width = 2.0
            outer_x, outer_y = ref_arr[:, 0] + nx * track_half_width, ref_arr[:, 1] + ny * track_half_width
            inner_x, inner_y = ref_arr[:, 0] - nx * track_half_width, ref_arr[:, 1] - ny * track_half_width
            ax1.plot(outer_x, outer_y, color='#334155', linewidth=1.5, linestyle='-', alpha=0.8, zorder=2)
            ax1.plot(inner_x, inner_y, color='#334155', linewidth=1.5, linestyle='-', alpha=0.8, zorder=2)

        # B. Draw High-Contrast Full Reference Raceline (Vivid Purple/Indigo Dashed Line)
        if len(self.ref_x) > 0:
            ax1.plot(self.ref_x, self.ref_y, color='#6366f1', linestyle='--', linewidth=2.0, alpha=0.9, label='Reference Raceline', zorder=3)

        # C. Draw Exact Start / Finish Line (Main Straight: 89633.29, 43127.57)
        ax1.scatter([self.start_x], [self.start_y], marker='s', s=100, color='#10b981', edgecolors='black', linewidth=1.5, label='Start/Finish Line', zorder=6)
        ax1.annotate("Start/Finish", (self.start_x, self.start_y), textcoords="offset points", xytext=(-10, 10),
                     fontsize=8, fontweight='bold', color='#065f46',
                     bbox=dict(boxstyle='round,pad=0.2', facecolor='#a7f3d0', edgecolor='black', alpha=0.9), zorder=7)

        # D. Draw Actual Logged Driven Path (Only connect physically valid continuous driving segments)
        if len(x_arr) >= 2:
            valid_segments = []
            valid_speeds = []
            for i in range(len(x_arr) - 1):
                seg_dist = math.hypot(x_arr[i+1] - x_arr[i], y_arr[i+1] - y_arr[i])
                # Only link points from continuous driving (< 1.5m step at ~40Hz), avoiding teleport/lap jumps
                if seg_dist < 1.5:
                    valid_segments.append([(x_arr[i], y_arr[i]), (x_arr[i+1], y_arr[i+1])])
                    valid_speeds.append(v_arr[i])

            if len(valid_segments) > 0:
                lc = LineCollection(valid_segments, cmap='jet', norm=plt.Normalize(vmin=0, vmax=45), linewidths=2.8, alpha=0.95, zorder=4)
                lc.set_array(np.array(valid_speeds))
                line_art = ax1.add_collection(lc)
                cbar = fig.colorbar(line_art, ax=ax1, fraction=0.046, pad=0.04)
                cbar.set_label('Speed [km/h]', fontsize=9)
            # Current vehicle position
            ax1.scatter([x_arr[-1]], [y_arr[-1]], marker='o', s=80, color='#f59e0b', edgecolors='black', linewidth=1.5, label='Current Pos', zorder=8)

        # E. Plot Time-Loss Points with clean compact badges [T1..T4]
        for p in time_loss_pts:
            ax1.scatter(p['x'], p['y'], marker='*', s=180, c='cyan', edgecolors='black', linewidth=1.2, zorder=9)
            ax1.annotate(p['tag'], (p['x'], p['y']), textcoords="offset points", xytext=(6, 6),
                         fontsize=8, fontweight='bold', color='black',
                         bbox=dict(boxstyle='round,pad=0.2', facecolor='#00ffff', edgecolor='black', alpha=0.9),
                         zorder=10)

        # F. Plot Control Instability Points with clean compact badges [U1..U4]
        for p in unstable_pts:
            ax1.scatter(p['x'], p['y'], marker='^', s=140, c='#ff2222', edgecolors='black', linewidth=1.2, zorder=9)
            ax1.annotate(p['tag'], (p['x'], p['y']), textcoords="offset points", xytext=(6, -10),
                         fontsize=8, fontweight='bold', color='white',
                         bbox=dict(boxstyle='round,pad=0.2', facecolor='#dc2626', edgecolor='black', alpha=0.9),
                         zorder=10)

        # Legend with all key components (100% English, no Unicode box glitches)
        custom_legend = [
            Line2D([0], [0], color='#334155', linewidth=1.6, label='Track Boundaries'),
            Line2D([0], [0], color='#6366f1', linestyle='--', linewidth=2.0, label='Reference Raceline'),
            Line2D([0], [0], color='#ea580c', linewidth=2.8, label='Driven Trajectory (Speed)'),
            Line2D([0], [0], marker='s', color='w', label='Start/Finish Line',
                   markerfacecolor='#10b981', markeredgecolor='black', markersize=9),
            Line2D([0], [0], marker='*', color='w', label='[T1..T4] Time-Loss Points',
                   markerfacecolor='cyan', markeredgecolor='black', markersize=11),
            Line2D([0], [0], marker='^', color='w', label='[U1..U4] Instability Points',
                   markerfacecolor='#ff2222', markeredgecolor='black', markersize=10)
        ]
        ax1.legend(handles=custom_legend, loc='upper right', fontsize=7.8, framealpha=0.9)
        ax1.set_title('1. 2D Course Track & Driving Line vs Reference\n(Purple Dash: Reference | Colormap Line: Driven Path)', fontsize=11, fontweight='bold')
        ax1.set_xlabel('X [m]', fontsize=9)
        ax1.set_ylabel('Y [m]', fontsize=9)
        ax1.axis('equal')
        ax1.grid(True, linestyle=':', alpha=0.4)

        if len(self.ref_x) > 0:
            margin = 6.0
            ax1.set_xlim(min(self.ref_x) - margin, max(self.ref_x) + margin)
            ax1.set_ylim(min(self.ref_y) - margin, max(self.ref_y) + margin)

        # =========================================================
        # Plot 2: Velocity & Lateral Deviation vs Distance
        # =========================================================
        ax2 = axes[0, 1]
        line_v, = ax2.plot(s_arr, v_arr, 'b-', label='Speed (km/h)', linewidth=2.0, zorder=3)
        ax2.set_xlabel('Distance s [m]', fontweight='bold', fontsize=9)
        ax2.set_ylabel('Speed [km/h]', color='b', fontweight='bold', fontsize=9)
        ax2.tick_params(axis='y', labelcolor='b')
        ax2.grid(True, linestyle=':', alpha=0.5)

        ax2_twin = ax2.twinx()
        line_ey, = ax2_twin.plot(s_arr, ey_arr, color='purple', linestyle=':', label='Lateral Dev ey (m)', alpha=0.65, linewidth=1.5)
        ax2_twin.set_ylabel('Lateral Deviation [m]', color='purple', fontweight='bold', fontsize=9)
        ax2_twin.tick_params(axis='y', labelcolor='purple')

        for p in time_loss_pts:
            v_val = v_arr[p['idx']]
            ax2.scatter(p['s'], v_val, marker='*', s=140, c='cyan', edgecolors='black', linewidth=1.0, zorder=5)
            ax2.annotate(p['tag'], (p['s'], v_val), textcoords="offset points", xytext=(0, 7),
                         ha='center', fontsize=7.5, fontweight='bold', color='black',
                         bbox=dict(boxstyle='round,pad=0.15', facecolor='#00ffff', edgecolor='black', alpha=0.85))

        for p in unstable_pts:
            v_val = v_arr[p['idx']]
            ax2.scatter(p['s'], v_val, marker='^', s=110, c='#ff2222', edgecolors='black', linewidth=1.0, zorder=5)
            ax2.annotate(p['tag'], (p['s'], v_val), textcoords="offset points", xytext=(0, -12),
                         ha='center', fontsize=7.5, fontweight='bold', color='white',
                         bbox=dict(boxstyle='round,pad=0.15', facecolor='#dc2626', edgecolor='black', alpha=0.85))

        ax2.set_title('2. Velocity & Lateral Deviation (Bottom: Distance s | Top: Time t)', fontsize=10.5, fontweight='bold')
        ax2.legend(handles=[line_v, line_ey], loc='lower right', fontsize=8, framealpha=0.85)

        # Secondary top x-axis for Elapsed Time [s]
        if len(s_arr) > 1 and s_arr[-1] > s_arr[0]:
            try:
                def s_to_t(s_val):
                    return np.interp(s_val, s_arr, t_arr)
                def t_to_s(t_val):
                    return np.interp(t_val, t_arr, s_arr)
                ax2_top = ax2.secondary_xaxis('top', functions=(s_to_t, t_to_s))
                ax2_top.set_xlabel('Elapsed Time t [s]', fontsize=8.5, color='#475569')
                ax2_top.tick_params(axis='x', labelcolor='#475569', labelsize=8)
            except Exception as e:
                self.get_logger().warn(f"ax2 secondary_xaxis failed: {e}")

        # =========================================================
        # Plot 3: Control Commands & Stability Profiles vs Distance
        # =========================================================
        ax3 = axes[1, 0]
        d_steer = np.diff(steer_arr, prepend=steer_arr[0]) / np.clip(dt_arr, 0.01, 1.0)

        # Highlight oscillation zones with subtle red shade along track distance s
        for p in unstable_pts:
            if 's_start' in p and 's_end' in p:
                ax3.axvspan(p['s_start'], p['s_end'], color='red', alpha=0.08, zorder=1)

        line_acc, = ax3.plot(s_arr, a_arr, 'r-', label='Cmd Accel (m/s^2)', linewidth=1.8, zorder=3)
        ax3.axhline(1.0, color='green', linestyle='--', label='SW Limit (+1.0 m/s^2)', alpha=0.8, linewidth=1.2)
        ax3.axhline(-1.6, color='orange', linestyle='--', label='Brake Limit (-1.6 m/s^2)', alpha=0.8, linewidth=1.2)
        ax3.set_xlabel('Distance s [m]', fontweight='bold', fontsize=9)
        ax3.set_ylabel('Acceleration [m/s^2]', color='darkred', fontweight='bold', fontsize=9)
        ax3.tick_params(axis='y', labelcolor='darkred')
        ax3.grid(True, linestyle=':', alpha=0.5)

        ax3_twin = ax3.twinx()
        line_steer_rate, = ax3_twin.plot(s_arr, np.abs(d_steer), color='teal', linestyle='-.', label='|Steer Rate| (rad/s)', alpha=0.6, linewidth=1.3)
        ax3_twin.axhline(0.6, color='red', linestyle=':', label='Hunting Threshold (0.6 rad/s)', alpha=0.7, linewidth=1.2)
        ax3_twin.set_ylabel('|Steering Rate| [rad/s]', color='teal', fontweight='bold', fontsize=9)
        ax3_twin.tick_params(axis='y', labelcolor='teal')

        for p in unstable_pts:
            s_val = p['s']
            a_val = a_arr[p['idx']]
            ax3.scatter(s_val, a_val, marker='^', s=130, c='#ff2222', edgecolors='black', linewidth=1.0, zorder=5)
            ax3.annotate(p['tag'], (s_val, a_val), textcoords="offset points", xytext=(0, -12),
                         ha='center', fontsize=7.5, fontweight='bold', color='white',
                         bbox=dict(boxstyle='round,pad=0.15', facecolor='#dc2626', edgecolor='black', alpha=0.85))

        ax3.set_title('3. Control Stability & Rule Compliance (Bottom: Distance s | Top: Time t)', fontsize=10.5, fontweight='bold')
        ax3.legend(handles=[line_acc, line_steer_rate], loc='upper right', fontsize=8, framealpha=0.85)

        # Secondary top x-axis for Elapsed Time [s] using interpolation
        if len(s_arr) > 1 and s_arr[-1] > s_arr[0]:
            try:
                def s_to_t(s_val):
                    return np.interp(s_val, s_arr, t_arr)
                def t_to_s(t_val):
                    return np.interp(t_val, t_arr, s_arr)
                ax3_top = ax3.secondary_xaxis('top', functions=(s_to_t, t_to_s))
                ax3_top.set_xlabel('Elapsed Time t [s]', fontsize=8.5, color='#475569')
                ax3_top.tick_params(axis='x', labelcolor='#475569', labelsize=8)
            except Exception as e:
                self.get_logger().warn(f"secondary_xaxis failed: {e}")

        # =========================================================
        # Plot 4: Comprehensive Diagnostics & Optimization Report
        # =========================================================
        ax4 = axes[1, 1]
        ax4.axis('off')

        # Calculate exact full lap distance of reference path
        ref_lap_dist = 0.0
        if len(self.ref_x) > 1:
            ref_ds = np.hypot(np.diff(self.ref_x), np.diff(self.ref_y))
            ref_lap_dist = np.sum(ref_ds)
            # Add closing segment for loop
            ref_lap_dist += math.hypot(self.ref_x[-1] - self.ref_x[0], self.ref_y[-1] - self.ref_y[0])

        report = "=========================================================\n"
        report += "      RACING KART PERFORMANCE & STABILITY DIAGNOSTICS   \n"
        report += "=========================================================\n\n"

        report += "[COURSE & REFERENCE METRICS]\n"
        if ref_lap_dist > 0.0:
            report += f"  - Total Reference Lap Distance: {ref_lap_dist:.1f} m ({len(self.ref_x)} waypoints)\n"
        report += f"  - Start/Finish Line Position  : X={self.start_x:.2f}, Y={self.start_y:.2f}\n\n"

        report += "[BADGE REFERENCE & LEGEND]\n"
        report += " [T1..T4] CYAN STAR    : Lap-Time Loss Points (Apex / Accel / Line)\n"
        report += " [U1..U4] RED TRIANGLE : Control Instability Points (Hunting / Jitter)\n\n"

        report += "[LAP TIME SUMMARY]\n"
        if len(self.lap_times) == 0:
            report += "  - In Progress (Recording Lap 1...)\n"
        else:
            for idx, ltime in enumerate(self.lap_times, 1):
                report += f"  - Lap {idx}: {ltime:.2f} s\n"
            report += f"  >> Fastest Lap: {min(self.lap_times):.2f} s | Average: {np.mean(self.lap_times):.2f} s\n"
        report += "\n"

        report += "[*] TOP LAP-TIME IMPROVEMENT AREAS (T1-T4 Details)\n"
        if len(time_loss_pts) == 0:
            report += "  - No prominent bottleneck detected.\n"
        else:
            for p in time_loss_pts:
                report += f"  [{p['tag']}] s={p['s']:.1f}m : {p['type']} -> {p['desc']}\n"
            report += "  >> Recommended Action: Raise corner entry speed / tune MPC weight\n"
        report += "\n"

        report += "[^] CONTROL INSTABILITY DIAGNOSTICS (U1-U4 Details)\n"
        if len(unstable_pts) == 0:
            report += "  - Good: No significant steering hunting detected.\n"
        else:
            for p in unstable_pts:
                report += f"  [{p['tag']}] s={p['s']:.1f}m : {p['desc']}\n"
            report += "  >> Recommended Action: Increase steering rate penalty (d_steer weight)\n"
        report += "\n"

        max_a = np.max(a_arr) if len(a_arr) > 0 else 0
        min_a = np.min(a_arr) if len(a_arr) > 0 else 0
        report += "[SW RULE COMPLIANCE CHECK]\n"
        report += f"  - Max Accel: {max_a:.2f} m/s^2 (Limit: 1.0 m/s^2) -> "
        report += "PASS (Compliant)\n" if max_a <= 1.05 else "VIOLATION (> 1.0 m/s^2!)\n"
        report += f"  - Max Brake: {min_a:.2f} m/s^2 (Limit: -1.6 m/s^2)\n"

        ax4.text(0.02, 0.98, report, fontsize=9.2, family='monospace', verticalalignment='top',
                 bbox=dict(boxstyle='round,pad=0.6', facecolor='#f8f9fa', edgecolor='#cccccc', alpha=0.95))

        plt.tight_layout()
        plot_filepath = os.path.join(self.output_dir, "trajectory_analysis.png")
        plt.savefig(plot_filepath, dpi=150)
        plt.close(fig)
        self.get_logger().info(f"Saved clean high-visibility trajectory analysis plot to {plot_filepath}")


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryAnalyzer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            node.save_analysis_plots()
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
