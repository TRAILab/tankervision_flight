#!/bin/bash
# Stop raw image recording on arena_camera_node.
# Usage: ./test_scripts/stop_record.sh
set -e
source /opt/ros/humble/setup.bash
source "$(dirname "$(dirname "$(realpath "$0")")")/install/setup.bash"
ros2 topic pub --once \
    --qos-reliability reliable \
    --qos-durability transient_local \
    /camera/record_mode std_msgs/msg/String "{data: standby}"
