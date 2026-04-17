#!/bin/bash
# start_camera.sh
# Sets up the DFG/USB2pro composite NTSC input and launches the v4l2_camera ROS node.

DEVICE=${1:-/dev/video0}

echo "[setup] Configuring $DEVICE for composite NTSC..."
v4l2-ctl --device="$DEVICE" --set-input=0          # Composite1
v4l2-ctl --device="$DEVICE" --set-standard=NTSC
v4l2-ctl --device="$DEVICE" --set-fmt-video=width=720,height=480,pixelformat=YUYV

echo "[setup] Current format:"
v4l2-ctl --device="$DEVICE" --get-fmt-video
v4l2-ctl --device="$DEVICE" --get-standard
v4l2-ctl --device="$DEVICE" --get-input

echo "[setup] Launching v4l2_camera node..."
source /opt/ros/$ROS_DISTRO/setup.bash

ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:="$DEVICE" \
  -p pixel_format:=YUYV \
  -p output_encoding:=rgb8
