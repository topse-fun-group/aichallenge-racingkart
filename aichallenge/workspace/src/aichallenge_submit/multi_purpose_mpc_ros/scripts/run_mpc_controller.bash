#!/bin/bash
# shellcheck disable=SC1091
if ! /usr/bin/python3 -c "import skimage" 2>/dev/null; then
    /usr/bin/python3 -m pip install --no-cache-dir scikit-image pandas >/dev/null 2>&1 || true
fi
exec /usr/bin/python3 -u "$(ros2 pkg prefix multi_purpose_mpc_ros)/lib/multi_purpose_mpc_ros/mpc_controller" "$@"
