#!/bin/bash
# Script to run trajectory analysis inside autoware container
source /opt/ros/humble/setup.bash 2>/dev/null || true
source /aichallenge/workspace/install/setup.bash 2>/dev/null || true

export PYTHONUNBUFFERED=1
exec python3 /aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/scripts/analyze_trajectory.py "${1:-/output}"
