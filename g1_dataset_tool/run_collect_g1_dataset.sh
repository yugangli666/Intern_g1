#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# run_collect_g1_dataset.sh — Launch the G1 dataset collection tool.
#
# Usage:
#   cd /home/unitree/Intern_g1
#   bash g1_dataset_tool/run_collect_g1_dataset.sh --instruction "Test"
#
#   bash g1_dataset_tool/run_collect_g1_dataset.sh \
#     --instruction "Navigate to the TV" --enable-motion
#
# All CLI arguments are forwarded to collect_g1_dataset.py unchanged.
# ---------------------------------------------------------------------------
set -euo pipefail

# ── Locate project root ──────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "=============================================="
echo "  G1 Dataset Collection Launcher"
echo "=============================================="
echo "  Script dir:   $SCRIPT_DIR"
echo "  Project root: $PROJECT_ROOT"
echo "=============================================="

# ── Check Python script exists ───────────────────────────────────────────
COLLECT_SCRIPT="$SCRIPT_DIR/collect_g1_dataset.py"
if [ ! -f "$COLLECT_SCRIPT" ]; then
    echo "ERROR: collect_g1_dataset.py not found at $COLLECT_SCRIPT" >&2
    exit 1
fi

# ── Try to source the G1 Foxy environment (if on the robot) ──────────────
# We attempt to source ros_foxy_env.sh from g1_client/.  If that fails
# (e.g. on a dev machine without Foxy), we fall back to whatever ROS 2
# environment is already active in the current shell.
#
# IMPORTANT: ros_foxy_env.sh's source_g1_ros_foxy will call
# remove_ros_distribution_paths which strips ALL /opt/ros/* paths from the
# environment BEFORE trying to source Foxy.  If Foxy isn't available, this
# leaves the shell in a broken state.  To guard against this, we save the
# critical ROS 2 env vars before attempting Foxy and restore them on failure.
G1_CLIENT_DIR="$PROJECT_ROOT/g1_client"
ROSY_ENV_SCRIPT="$G1_CLIENT_DIR/ros_foxy_env.sh"
DDS_SCRIPT="$G1_CLIENT_DIR/dds_interface.sh"

ROS_ENV_SOURCED=false

# Save current ROS 2 environment (in case Foxy setup strips it and fails)
_SAVED_PATH="${PATH:-}"
_SAVED_LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"
_SAVED_PYTHONPATH="${PYTHONPATH:-}"
_SAVED_AMENT_PREFIX_PATH="${AMENT_PREFIX_PATH:-}"
_SAVED_CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:-}"
_SAVED_ROS_DISTRO="${ROS_DISTRO:-}"

if [ -f "$ROSY_ENV_SCRIPT" ]; then
    echo "[ENV] Found ros_foxy_env.sh — attempting G1 Foxy environment…"
    # shellcheck disable=SC1090
    if source "$ROSY_ENV_SCRIPT" 2>/dev/null && \
       command -v source_g1_ros_foxy >/dev/null 2>&1; then
        if source_g1_ros_foxy 2>/dev/null; then
            ROS_ENV_SOURCED=true
            echo "[ENV] G1 Foxy environment loaded successfully."
        else
            echo "[ENV] source_g1_ros_foxy failed — restoring previous environment." >&2
            # Restore saved environment
            export PATH="$_SAVED_PATH"
            export LD_LIBRARY_PATH="$_SAVED_LD_LIBRARY_PATH"
            export PYTHONPATH="$_SAVED_PYTHONPATH"
            export AMENT_PREFIX_PATH="$_SAVED_AMENT_PREFIX_PATH"
            export CMAKE_PREFIX_PATH="$_SAVED_CMAKE_PREFIX_PATH"
            if [ -n "$_SAVED_ROS_DISTRO" ]; then
                export ROS_DISTRO="$_SAVED_ROS_DISTRO"
            fi
        fi
    else
        echo "[ENV] ros_foxy_env.sh did not load — falling back to current environment." >&2
    fi
fi

# If we loaded Foxy, also configure CycloneDDS
if [ "$ROS_ENV_SOURCED" = true ] && [ -f "$DDS_SCRIPT" ]; then
    # shellcheck disable=SC1090
    source "$DDS_SCRIPT" 2>/dev/null || true
    if command -v configure_cyclonedds_interface >/dev/null 2>&1; then
        configure_cyclonedds_interface 2>/dev/null || true
    fi
fi

# If no ROS environment is active, check if we can at least find ros2
if [ "${ROS_DISTRO:-}" = "" ]; then
    echo "[ENV] WARNING: ROS_DISTRO is not set." >&2
    echo "[ENV] If you are on the G1 robot, make sure ros_foxy_env.sh is configured." >&2
    echo "[ENV] If you are on a dev machine, source your ROS 2 setup.bash first, e.g.:" >&2
    echo "[ENV]   source /opt/ros/jazzy/setup.bash" >&2
    echo "[ENV] Continuing anyway — the script may fail if rclpy is not importable." >&2
fi

# ── Check Python can import rclpy ────────────────────────────────────────
PYTHON_BIN="${PYTHON_BIN:-python3}"
echo "[ENV] Using Python: $PYTHON_BIN ($($PYTHON_BIN --version 2>&1))"
echo "[ENV] ROS_DISTRO=${ROS_DISTRO:-<unset>}"
echo "[ENV] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset>}"
echo "[ENV] ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"

if ! "$PYTHON_BIN" -c "import rclpy" 2>/dev/null; then
    echo "==============================================" >&2
    echo "  WARNING: rclpy is not importable." >&2
    echo "  The script may fail when it tries to init ROS." >&2
    echo "  Make sure your ROS 2 environment is properly sourced." >&2
    echo "==============================================" >&2
fi

# ── Detect whether --enable-motion is among the arguments ────────────────
MOTION_ENABLED=false
for arg in "$@"; do
    if [ "$arg" = "--enable-motion" ]; then
        MOTION_ENABLED=true
        break
    fi
done

if [ "$MOTION_ENABLED" = true ]; then
    echo "=============================================="
    echo "  MOTION CONTROL IS ENABLED"
    echo "  The robot WILL move in response to keyboard."
    echo "  Ensure:"
    echo "    - Robot is physically supported"
    echo "    - Emergency stop is accessible"
    echo "    - No other motion controller is running"
    echo "=============================================="
fi

# ── Launch the collection script ─────────────────────────────────────────
echo ""
echo "[RUN] $PYTHON_BIN $COLLECT_SCRIPT $*"
echo ""

cd "$PROJECT_ROOT"
exec "$PYTHON_BIN" "$COLLECT_SCRIPT" "$@"
