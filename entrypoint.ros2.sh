#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/${ROS_DISTRO}/setup.bash"
[[ -f "${ROS_WS}/install/setup.bash" ]] && source "${ROS_WS}/install/setup.bash"

# ROS information
echo 'ROS distro:     ' $ROS_DISTRO

if [ -n "$RMW_IMPLEMENTATION" ]; then
    echo 'DDS middleware: ' $RMW_IMPLEMENTATION;
fi

if [ -n "$ROS_DOMAIN_ID" ]; then
    echo 'ROS Domain ID:  ' $ROS_DOMAIN_ID;
fi

exec "$@"

