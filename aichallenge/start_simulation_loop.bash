#!/bin/bash
source /opt/ros/humble/setup.bash
source /autoware/install/setup.bash

echo "Publishing AWSIM start signal for 10 seconds..."
ros2 topic pub -r 2 /admin/awsim/start std_msgs/msg/Bool "{data: true}" &
PID_START=$!

sleep 3
echo "Publishing Autoware control mode request for 10 seconds..."
ros2 topic pub -r 2 /awsim/control_mode_request_topic std_msgs/msg/Bool "{data: true}" &
PID_CONTROL=$!

sleep 7
kill $PID_START 2>/dev/null || true
kill $PID_CONTROL 2>/dev/null || true
echo "Simulation start signals completed successfully."
