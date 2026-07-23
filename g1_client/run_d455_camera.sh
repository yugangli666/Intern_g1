#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Load Foxy environment (handles nounset internally) ─────────────────
# shellcheck disable=SC1091
source "$SCRIPT_DIR/ros_foxy_env.sh"
source_g1_ros_foxy

# ── CycloneDDS network-interface selection ─────────────────────────────
# shellcheck disable=SC1091
source "$SCRIPT_DIR/dds_interface.sh"
configure_cyclonedds_interface

# ── D455 backend selection ─────────────────────────────────────────────
# D455_BACKEND:
#   realsense    → use ROS 2 realsense2_camera node (default, preferred)
#   pyrealsense  → use pyrealsense2 Python publisher fallback
D455_BACKEND="${D455_BACKEND:-realsense}"

# The D455 is currently connected through USB 2.1.  Use low-bandwidth RGB-D
# profiles by default; users with a USB 3.x link may override them.
# Foxy's realsense2_camera expects comma-separated profile arguments.
COLOR_PROFILE="${REALSENSE_COLOR_PROFILE:-424,240,15}"
DEPTH_PROFILE="${REALSENSE_DEPTH_PROFILE:-480,270,15}"
CAMERA_NAME="${REALSENSE_CAMERA_NAME:-camera}"
POWER_LINE_FREQUENCY="${REALSENSE_POWER_LINE_FREQUENCY:-1}"

echo "[D455] backend=$D455_BACKEND  color=$COLOR_PROFILE  depth=$DEPTH_PROFILE"

case "$D455_BACKEND" in
  realsense)
    # Check that realsense2_camera ROS package is available
    if ! ros2 pkg prefix realsense2_camera >/dev/null 2>&1; then
      echo "[D455] ERROR: realsense2_camera package not found." >&2
      echo "[D455] Install with: sudo apt install ros-foxy-realsense2-camera" >&2
      echo "[D455] Or set D455_BACKEND=pyrealsense to use the Python fallback." >&2
      exit 1
    fi
    ros2 launch realsense2_camera rs_launch.py \
      camera_name:="$CAMERA_NAME" \
      enable_color:=true \
      enable_depth:=true \
      enable_infra1:=false \
      enable_infra2:=false \
      enable_fisheye1:=false \
      enable_fisheye2:=false \
      enable_confidence:=false \
      enable_gyro:=false \
      enable_accel:=false \
      enable_pose:=false \
      align_depth.enable:=true \
      enable_sync:=true \
      rgb_camera.profile:="$COLOR_PROFILE" \
      depth_module.profile:="$DEPTH_PROFILE" \
      rgb_camera.power_line_frequency:="$POWER_LINE_FREQUENCY" \
      "$@"
    ;;

  pyrealsense)
    # Check that pyrealsense2 is importable
    if ! python3 -c "import pyrealsense2" 2>/dev/null; then
      echo "[D455] ERROR: pyrealsense2 not importable." >&2
      echo "[D455] Install with: pip install pyrealsense2" >&2
      exit 1
    fi
    PYTHON_BIN="${PYTHON_BIN:-python3}"
    exec "$PYTHON_BIN" "$SCRIPT_DIR/d455_rgbd_publisher.py" "$@"
    ;;

  *)
    echo "[D455] ERROR: unknown backend '$D455_BACKEND'. Valid: realsense, pyrealsense" >&2
    exit 1
    ;;
esac
