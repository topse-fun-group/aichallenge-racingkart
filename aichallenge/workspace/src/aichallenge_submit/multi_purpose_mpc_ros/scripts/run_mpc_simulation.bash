#!/bin/bash
# shellcheck disable=SC1091
exec /usr/bin/python3 -u "$(ros2 pkg prefix multi_purpose_mpc_ros)/lib/multi_purpose_mpc_ros/mpc_simulation" "$@"
