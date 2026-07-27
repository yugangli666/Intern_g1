#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

if [ -f "$PROJECT_ROOT/nav/install/setup.bash" ]; then
    source "$PROJECT_ROOT/nav/setup_local_nav.sh"
elif [ -f /workspace/nav_ws/install/setup.bash ]; then
    set +u
    source /workspace/nav_ws/install/setup.bash
    set -u
else
    echo "[ERROR] ROS 2 environment is unavailable" >&2
    exit 2
fi

set_parameter() {
    local node="$1"
    local name="$2"
    local value="$3"
    local output
    output="$(timeout 8 ros2 param set "$node" "$name" "$value" 2>&1)" || {
        echo "[ERROR] failed to set $node $name=$value: $output" >&2
        return 1
    }
    grep -qi "successful" <<< "$output" || {
        echo "[ERROR] $node rejected $name=$value: $output" >&2
        return 1
    }
}

timeout 4 ros2 topic pub --once /robot_nav_stop_task std_msgs/msg/Bool "{data: true}" \
    >/dev/null 2>&1 || true
timeout 4 ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: true}" \
    >/dev/null 2>&1 || true

set_parameter /controller_server FollowPath.vx_max 0.12
set_parameter /controller_server FollowPath.vx_min 0.0
set_parameter /controller_server FollowPath.vy_max 0.08
set_parameter /controller_server FollowPath.wz_max 0.25
set_parameter /controller_server FollowPath.ax_max 0.30
set_parameter /controller_server FollowPath.ax_min -0.30
set_parameter /controller_server FollowPath.ay_max 0.30
set_parameter /controller_server FollowPath.ay_min -0.30
set_parameter /controller_server FollowPath.az_max 0.50

set_parameter /behavior_server max_rotational_vel 0.20
set_parameter /behavior_server min_rotational_vel 0.08
set_parameter /behavior_server rotational_acc_lim 0.40
set_parameter /behavior_server local_frame moz1/map

set_parameter /velocity_smoother feedback OPEN_LOOP
set_parameter /velocity_smoother max_velocity '[0.12, 0.08, 0.25]'
set_parameter /velocity_smoother min_velocity '[0.0, -0.08, -0.25]'
set_parameter /velocity_smoother max_accel '[0.25, 0.25, 0.50]'
set_parameter /velocity_smoother max_decel '[-0.40, -0.40, -0.80]'
set_parameter /velocity_smoother velocity_timeout 0.30

set_parameter /collision_monitor pointcloud.enabled true
set_parameter /collision_monitor pointcloud.min_height -0.05
set_parameter /collision_monitor pointcloud.max_height 1.80
set_parameter /collision_monitor CircleLimit.enabled true
set_parameter /collision_monitor CircleLimit.radius 0.80
set_parameter /collision_monitor CircleLimit.slowdown_ratio 0.10

echo "[OK] guarded Nav2 runtime limits applied"
echo "  linear: <= 0.12 m/s"
echo "  angular: <= 0.25 rad/s"
echo "  collision slowdown: 0.80 m at 10% speed"
