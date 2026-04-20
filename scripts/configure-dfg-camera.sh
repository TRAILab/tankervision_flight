#!/bin/bash
DEVICE=/dev/video0
sleep 1 #give the device a second to settle
v4l2-ctl --device="$DEVICE" --set-input=0
v4l2-ctl --device="$DEVICE" --set-standard=NTSC
v4l2-ctl --device="$DEVICE" --set-fmt-video=width=720,height=576,pixelformat=YUYV
