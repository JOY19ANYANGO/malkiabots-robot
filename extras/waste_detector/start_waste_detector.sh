#!/usr/bin/env bash
# Starts the camera stream and the Hailo waste detector together.
# Run by the waste-detector service at boot; can also be run by hand.

# ---- settings (change these here only) ----
HEF=/home/malkia/waste/waste.hef
CONF=0.25            # confidence threshold
STABLE_FRAMES=5      # frames in a row before a class is decided
export ROS_DOMAIN_ID=7
# -------------------------------------------

source /opt/ros/jazzy/setup.bash

# Camera: serve MJPEG on the Pi only, restarting automatically if it stops
(
  while true; do
    rpicam-vid -t 0 -n --width 640 --height 480 --framerate 30 \
      --codec mjpeg --quality 80 --listen -o tcp://127.0.0.1:8888 >/dev/null 2>&1
    sleep 1
  done
) &
CAMERA_LOOP=$!

cleanup() {
  kill "$CAMERA_LOOP" 2>/dev/null
  pkill -f "rpicam-vid.*127.0.0.1:8888" 2>/dev/null
}
trap cleanup EXIT

exec_detector() {
  /home/malkia/hailo_venv/bin/python3 /home/malkia/waste/hailo_waste_detector.py \
    --hef "$HEF" --conf "$CONF" --stable-frames "$STABLE_FRAMES" --start-disabled
}
exec_detector
