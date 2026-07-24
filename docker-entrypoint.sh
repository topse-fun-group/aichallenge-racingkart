#!/bin/bash
# Common container initialization: ROS workspace setup.

# --- Source Autoware & ROS workspace ---
export PYTHONPATH="/tmp/.local/lib/python3.10/site-packages:${PYTHONPATH}"
if [ -f /opt/ros/humble/setup.bash ]; then
    set +u && source /opt/ros/humble/setup.bash
fi
if [ -f /autoware/install/setup.bash ]; then
    set +u && source /autoware/install/setup.bash
fi
if [ -f /aichallenge/workspace/install/setup.bash ]; then
    set +u && source /aichallenge/workspace/install/setup.bash
elif [ -f /aichallenge/workspace/install/local_setup.bash ]; then
    set +u && source /aichallenge/workspace/install/local_setup.bash
fi

# When used as ENTRYPOINT, hand off to the CMD / command.
# When sourced from .bashrc, exec is a no-op (no positional args).
if [ $# -gt 0 ]; then
    exec "$@"
fi
