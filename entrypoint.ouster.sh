#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/${ROS_DISTRO}/setup.bash"
[[ -f "${ROS_WS}/install/setup.bash" ]] && source "${ROS_WS}/install/setup.bash"

# ROS information
echo 'ROS distro: ' $ROS_DISTRO

if [ -n "$RMW_IMPLEMENTATION" ]; then
    echo 'DDS middleware: ' $RMW_IMPLEMENTATION;
fi

if [ -n "$ROS_DOMAIN_ID" ]; then
    echo 'ROS Domain ID: ' $ROS_DOMAIN_ID;
fi

echo "---------------------"
echo "Checking network settings"

# Define the expected values
EXPECTED_RMEM_MAX=2147483647
EXPECTED_IPFRAG_TIME=3
EXPECTED_IPFRAG_HIGH_THRESH=1342177280

# Initialize a flag to track if any parameter is incorrectly set
ALL_VALID=true

# Function to check and report parameter
check_parameter() {
    PARAM_NAME=$1
    EXPECTED_VALUE=$2
    CURRENT_VALUE=$(sysctl -n $PARAM_NAME)

    if [ "$CURRENT_VALUE" -eq "$EXPECTED_VALUE" ]; then
        echo "✔ $PARAM_NAME = $EXPECTED_VALUE"
    else
        echo "✘ $PARAM_NAME is incorrectly set to $CURRENT_VALUE (expected $EXPECTED_VALUE)"
        echo "To set it correctly, run:"
        echo "    sudo sysctl -w $PARAM_NAME=$EXPECTED_VALUE"
        echo "To make this change permanent, add the following line to /etc/sysctl.conf or a file in /etc/sysctl.d/:"
        echo "    $PARAM_NAME=$EXPECTED_VALUE"
        echo
        ALL_VALID=false
    fi
}

# Check each parameter
check_parameter net.core.rmem_max $EXPECTED_RMEM_MAX
check_parameter net.ipv4.ipfrag_time $EXPECTED_IPFRAG_TIME
check_parameter net.ipv4.ipfrag_high_thresh $EXPECTED_IPFRAG_HIGH_THRESH

# Exit with an error code if any parameter is incorrect
if [ "$ALL_VALID" = false ]; then
    echo "Error: One or more network parameters are incorrectly set."
    exit 1
fi

echo "---------------------"


exec "$@"

