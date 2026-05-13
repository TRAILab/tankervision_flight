#!/bin/bash
# Start raw image recording on arena_camera_node and publish a save trigger.
# Usage: ./test_scripts/start_record.sh
set -e
source /opt/ros/humble/setup.bash
source "$(dirname "$(dirname "$(realpath "$0")")")/install/setup.bash"
ros2 topic pub --times 3 --rate 1 \
    --wait-matching-subscriptions 1 \
    --qos-reliability reliable \
    --qos-durability transient_local \
    /camera/record_mode std_msgs/msg/String "{data: record}"
