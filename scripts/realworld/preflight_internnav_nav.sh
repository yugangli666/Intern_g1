#!/usr/bin/env bash

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
MODE="check"

case "${1:-}" in
    --require-clean)
        MODE="clean"
        ;;
    --runtime)
        MODE="runtime"
        ;;
    --motion)
        MODE="motion"
        ;;
    "")
        ;;
    *)
        echo "Usage: $0 [--require-clean|--runtime|--motion]" >&2
        exit 64
        ;;
esac

SERVER_HEALTH_URL="${SERVER_HEALTH_URL:-http://127.0.0.1:5801/health}"
PRIMARY_RGB_TOPIC="${PRIMARY_RGB_TOPIC:-/camera/cam_high_extra/image_undistorted}"
SECONDARY_RGB_TOPIC="${SECONDARY_RGB_TOPIC:-/camera/cam_high/image_raw}"
ODOM_TOPIC="${ODOM_TOPIC:-/moz1/odom_global}"
GLOBAL_FRAME="${GLOBAL_FRAME:-moz1/map}"
BASE_FRAME="${BASE_FRAME:-moz1/base_link}"
STATE_FILE="${PREFLIGHT_STATE_FILE:-/tmp/internnav_nav/preflight.env}"
ALLOW_LEGACY_IDLE_STACK="${ALLOW_LEGACY_IDLE_STACK:-0}"
MAX_LEGACY_BRIDGES="${MAX_LEGACY_BRIDGES:-2}"

if [ -f "$PROJECT_ROOT/nav/install/setup.bash" ]; then
    source "$PROJECT_ROOT/nav/setup_local_nav.sh"
elif [ -f /workspace/nav_ws/install/setup.bash ]; then
    set +u
    source /workspace/nav_ws/install/setup.bash
    set -u
elif [ -f /opt/ros/jazzy/setup.bash ]; then
    set +u
    source /opt/ros/jazzy/setup.bash
    set -u
else
    echo "[ERROR] No ROS 2 environment is available" >&2
    exit 2
fi

fatal=0
inference_only=0

fail() {
    echo "[ERROR] $*" >&2
    fatal=1
}

warn_hold() {
    echo "[HOLD] $*" >&2
    inference_only=1
}

warn_only() {
    echo "[WARN] $*" >&2
}

node_snapshot="$(ros2 node list 2>/dev/null || true)"

count_node() {
    local target="$1"
    awk -F/ -v target="$target" '$NF == target { count += 1 } END { print count + 0 }' \
        <<< "$node_snapshot"
}

vln_count="$(count_node vln_node)"
bridge_count="$(count_node message_forward)"
client_count="$(count_node internnav_nav_client)"

legacy_override=0
if [ "$MODE" = "motion" ] && [ "$ALLOW_LEGACY_IDLE_STACK" = "1" ]; then
    legacy_override=1
fi

if [ "$vln_count" -gt 0 ] && [ "$legacy_override" -ne 1 ]; then
    fail "/vln_node is running; InternNav must own the navigation task exclusively"
fi
if [ "$bridge_count" -gt 1 ] && [ "$legacy_override" -ne 1 ]; then
    fail "multiple message_forward nodes detected: $bridge_count"
fi
if [ "$legacy_override" -eq 1 ] \
    && { [ "$bridge_count" -lt 1 ] || [ "$bridge_count" -gt "$MAX_LEGACY_BRIDGES" ]; }; then
    fail "legacy motion requires 1..$MAX_LEGACY_BRIDGES message_forward nodes (found $bridge_count)"
fi
if [ "$client_count" -gt 1 ]; then
    fail "multiple internnav_nav_client nodes detected: $client_count"
fi

control_nodes=(
    controller_server
    planner_server
    smoother_server
    behavior_server
    bt_navigator
    waypoint_follower
    velocity_smoother
    collision_monitor
    docking_server
    route_server
    nav_executor
    lifecycle_manager_navigation
    lifecycle_manager_slam
)
for node_name in "${control_nodes[@]}"; do
    count="$(count_node "$node_name")"
    if [ "$legacy_override" -eq 1 ]; then
        if [ "$node_name" = "nav_executor" ]; then
            continue
        fi
        if [ "$node_name" = "lifecycle_manager_slam" ] && [ "$count" -le 2 ]; then
            continue
        fi
    fi
    if [ "$count" -gt 1 ]; then
        fail "multiple $node_name nodes detected: $count"
    fi
done

if [ "$MODE" = "clean" ]; then
    for node_name in message_forward internnav_nav_client "${control_nodes[@]}"; do
        count="$(count_node "$node_name")"
        if [ "$count" -gt 0 ]; then
            fail "existing navigation node prevents a clean local start: $node_name ($count)"
        fi
    done
fi

if ! curl -fsS --max-time 3 "$SERVER_HEALTH_URL" >/dev/null 2>&1; then
    fail "InternVLA health check failed: $SERVER_HEALTH_URL"
fi

topic_fresh() {
    local topic="$1"
    timeout 7 ros2 topic echo "$topic" --once >/dev/null 2>&1
}

if ! topic_fresh "$PRIMARY_RGB_TOPIC"; then
    fail "primary camera has no fresh message: $PRIMARY_RGB_TOPIC"
fi
if ! topic_fresh "$SECONDARY_RGB_TOPIC"; then
    warn_only "secondary monitor camera has no fresh message: $SECONDARY_RGB_TOPIC"
fi

odom_hz_output="$(timeout 7 ros2 topic hz "$ODOM_TOPIC" 2>&1 || true)"
odom_rate="$(awk '/average rate:/ { value=$3 } END { print value }' <<< "$odom_hz_output")"
if [ -z "$odom_rate" ]; then
    warn_hold "odometry has no measurable rate: $ODOM_TOPIC"
else
    echo "[OK] odometry average rate: $odom_rate Hz"
fi

tf_output="$(timeout 5 ros2 run tf2_ros tf2_echo "$GLOBAL_FRAME" "$BASE_FRAME" 2>&1 || true)"
if ! grep -q "Translation:" <<< "$tf_output"; then
    warn_hold "TF unavailable: $GLOBAL_FRAME -> $BASE_FRAME"
else
    echo "[OK] TF available: $GLOBAL_FRAME -> $BASE_FRAME"
fi

if [ "$MODE" = "runtime" ] || [ "$MODE" = "motion" ]; then
    if [ "$legacy_override" -ne 1 ] && [ "$bridge_count" -ne 1 ]; then
        fail "runtime requires exactly one message_forward node (found $bridge_count)"
    fi
    for node_name in controller_server velocity_smoother collision_monitor; do
        count="$(count_node "$node_name")"
        if [ "$count" -ne 1 ]; then
            fail "runtime requires exactly one $node_name node (found $count)"
        fi
    done
    action_snapshot="$(ros2 action list 2>/dev/null || true)"
    grep -qx "/follow_path" <<< "$action_snapshot" || fail "/follow_path action server is unavailable"
    grep -qx "/spin" <<< "$action_snapshot" || fail "/spin action server is unavailable"
    lifecycle="$(timeout 5 ros2 lifecycle get /controller_server 2>&1 || true)"
    grep -qi "active" <<< "$lifecycle" || fail "Nav2 controller_server is not active"
fi

if [ "$MODE" = "motion" ]; then
    nav_status="$(timeout 5 ros2 topic echo /nav_task_status --once --field data 2>/dev/null || true)"
    if ! grep -Eq '(^|[[:space:]])0([[:space:]]|$)' <<< "$nav_status"; then
        fail "legacy navigation task is not confirmed idle: ${nav_status:-no status}"
    fi
    collision_source="$(timeout 5 ros2 param get /collision_monitor pointcloud.enabled 2>&1 || true)"
    collision_zone="$(timeout 5 ros2 param get /collision_monitor CircleLimit.enabled 2>&1 || true)"
    grep -qi "true" <<< "$collision_source" || fail "collision pointcloud source is disabled"
    grep -qi "true" <<< "$collision_zone" || fail "collision slowdown zone is disabled"
fi

mkdir -p "$(dirname "$STATE_FILE")"
printf 'INFERENCE_ONLY=%s\n' "$inference_only" > "$STATE_FILE"

if [ "$fatal" -ne 0 ]; then
    exit 2
fi

if [ "$inference_only" -ne 0 ]; then
    echo "[OK] preflight passed in inference-only HOLD mode"
else
    echo "[OK] preflight passed"
fi
