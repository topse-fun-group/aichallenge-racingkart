#!/bin/bash
# Wrapper to launch trajectory analyzer directly from source or install path
SCRIPT_PATH="/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/scripts/analyze_trajectory.py"
if [ ! -f "${SCRIPT_PATH}" ]; then
  SCRIPT_PATH="$(ros2 pkg prefix multi_purpose_mpc_ros)/lib/multi_purpose_mpc_ros/analyze_trajectory.py"
fi
exec /usr/bin/python3 -u "${SCRIPT_PATH}" "$@"
