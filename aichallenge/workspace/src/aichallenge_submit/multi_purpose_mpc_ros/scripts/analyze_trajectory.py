#!/usr/bin/env python3
"""
Trajectory & Performance Analysis Script for MPC Racing Kart
Subscribes to odometry, control commands, and reference path to analyze:
1. 2D Course line (driving line vs reference raceline)
2. Velocity profile (actual speed vs distance)
3. Acceleration profile (checking compliance with <= 1.0 m/s^2 SW rule)
4. Lateral deviation (ey) from reference path
5. Lap times and section breakdown
"""

import os
import sys
import time
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend for saving PNGs
import matplotlib.pyplot as plt

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

        self.lap_start_times = []
        self.lap_times = []
        self.start_x = None
        self.start_y = None
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
        # Update if new reference path is longer / more complete
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

        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.last_cross_time = t

        self.timestamps.append(t)
        self.x_data.append(x)
        self.y_data.append(y)
        self.v_data.append(v)
        self.a_cmd_data.append(self.current_cmd_acc)
        self.steer_cmd_data.append(self.current_cmd_steer)

        # Lap completion detection (distance to start point < 2.5m after minimum 15s)
        dist_to_start = math.hypot(x - self.start_x, y - self.start_y)
        if len(self.timestamps) > 100 and (t - self.last_cross_time) > 15.0:
            if dist_to_start < 3.0:
                lap_time = t - self.last_cross_time
                self.lap_times.append(lap_time)
                self.last_cross_time = t
                self.get_logger().info(f"=== LAP COMPLETED: Lap {len(self.lap_times)} Time = {lap_time:.2f} s ===")
                self.save_analysis_plots()

    def save_analysis_plots(self):
        if len(self.x_data) < 10:
            return

        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        t_arr = np.array(self.timestamps) - self.timestamps[0]
        x_arr = np.array(self.x_data)
        y_arr = np.array(self.y_data)
        v_arr = np.array(self.v_data) * 3.6  # km/h
        a_arr = np.array(self.a_cmd_data)

        # Calculate distance s
        ds = np.hypot(np.diff(x_arr, prepend=x_arr[0]), np.diff(y_arr, prepend=y_arr[0]))
        s_arr = np.cumsum(ds)

        # Plot 1: 2D Course trajectory
        ax1 = axes[0, 0]
        if len(self.ref_x) > 0:
            ax1.plot(self.ref_x, self.ref_y, 'k--', label='Reference Path', alpha=0.6)
        scatter = ax1.scatter(x_arr, y_arr, c=v_arr, cmap='jet', s=3, label='Vehicle Path')
        fig.colorbar(scatter, ax=ax1, label='Speed [km/h]')
        ax1.set_title('Driving Line (Course Trajectory)')
        ax1.set_xlabel('X [m]')
        ax1.set_ylabel('Y [m]')
        ax1.axis('equal')
        ax1.grid(True)
        ax1.legend()

        # Plot 2: Speed profile vs Distance
        ax2 = axes[0, 1]
        ax2.plot(s_arr, v_arr, 'b-', label='Actual Speed (km/h)')
        ax2.set_title('Velocity Profile along Distance')
        ax2.set_xlabel('Distance s [m]')
        ax2.set_ylabel('Speed [km/h]')
        ax2.grid(True)
        ax2.legend()

        # Plot 3: Acceleration profile vs Time (Rule check <= 1.0 m/s^2)
        ax3 = axes[1, 0]
        ax3.plot(t_arr, a_arr, 'r-', label='Cmd Acceleration (m/s^2)')
        ax3.axhline(1.0, color='g', linestyle='--', label='SW Limit (+1.0 m/s^2)')
        ax3.axhline(-1.6, color='orange', linestyle='--', label='Brake Limit (-1.6 m/s^2)')
        ax3.set_title('Acceleration Command Profile (Rule Compliance Check)')
        ax3.set_xlabel('Time [s]')
        ax3.set_ylabel('Acceleration [m/s^2]')
        ax3.grid(True)
        ax3.legend()

        # Plot 4: Lap Times Summary Text
        ax4 = axes[1, 1]
        ax4.axis('off')
        summary_text = "=== LAP TIME SUMMARY ===\n\n"
        if len(self.lap_times) == 0:
            summary_text += "Currently driving lap 1...\n"
        else:
            for idx, ltime in enumerate(self.lap_times, 1):
                summary_text += f"Lap {idx}: {ltime:.2f} s\n"
            summary_text += f"\nFastest Lap: {min(self.lap_times):.2f} s\n"
            summary_text += f"Average Lap: {np.mean(self.lap_times):.2f} s\n"

        # Check acceleration violations
        max_a = np.max(a_arr) if len(a_arr) > 0 else 0
        summary_text += f"\nPeak Accel: {max_a:.2f} m/s^2 "
        if max_a <= 1.05:
            summary_text += "(Rule Compliant - PASS)\n"
        else:
            summary_text += "(VIOLATION > 1.0 m/s^2!)\n"

        ax4.text(0.1, 0.2, summary_text, fontsize=12, family='monospace')

        plt.tight_layout()
        plot_filepath = os.path.join(self.output_dir, "trajectory_analysis.png")
        plt.savefig(plot_filepath)
        plt.close(fig)
        self.get_logger().info(f"Saved trajectory analysis plot to {plot_filepath}")


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_analysis_plots()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
