#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

if [ -f "$HOME/unitree_ros2/setup.sh" ]; then
  # shellcheck disable=SC1091
  source "$HOME/unitree_ros2/setup.sh"
fi

if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$HOME/ros2_ws/install/setup.bash"
fi
set -u

# shellcheck disable=SC1091
source "$SCRIPT_DIR/dds_interface.sh"
configure_cyclonedds_interface

COLOR_PROFILE="${REALSENSE_COLOR_PROFILE:-640x480x15}"
DEPTH_PROFILE="${REALSENSE_DEPTH_PROFILE:-640x480x15}"

ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=camera \
  camera_name:=camera \
  enable_color:=true \
  enable_depth:=true \
  align_depth.enable:=true \
  enable_sync:=true \
  rgb_camera.color_profile:="$COLOR_PROFILE" \
  depth_module.depth_profile:="$DEPTH_PROFILE" \
  "$@"
