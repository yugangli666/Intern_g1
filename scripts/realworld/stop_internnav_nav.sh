#!/usr/bin/env bash

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
RUNTIME_DIR="${INTERNNAV_RUNTIME_DIR:-/tmp/internnav_nav}"
mkdir -p "$RUNTIME_DIR"

exec 9>"$RUNTIME_DIR/start.lock"
if ! flock -n 9; then
    echo "[ERROR] another InternNav start/stop operation is in progress" >&2
    exit 2
fi

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
fi

cancel_all_goals() {
    local action_name="$1"
    local zero_uuid='[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]'
    timeout 4 ros2 service call \
        "$action_name/_action/cancel_goal" \
        action_msgs/srv/CancelGoal \
        "{goal_info: {goal_id: {uuid: $zero_uuid}, stamp: {sec: 0, nanosec: 0}}}" \
        >/dev/null 2>&1 || true
}

if command -v ros2 >/dev/null 2>&1; then
    cancel_all_goals /follow_path
    cancel_all_goals /spin
    timeout 4 ros2 topic pub --once /robot_nav_stop_task std_msgs/msg/Bool \
        "{data: true}" >/dev/null 2>&1 || true
fi

stop_pid_file() {
    local label="$1"
    local pid_file="$2"
    local expected_pattern="$3"
    if [ ! -s "$pid_file" ]; then
        return
    fi
    local pid cmdline
    pid="$(cat "$pid_file")"
    if kill -0 "$pid" 2>/dev/null; then
        cmdline="$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)"
        if ! grep -Eq "$expected_pattern" <<< "$cmdline"; then
            echo "[WARN] ignored stale $label PID file; PID $pid belongs to another process" >&2
            rm -f "$pid_file"
            return
        fi
        kill -TERM "$pid" 2>/dev/null || true
        for _ in $(seq 1 20); do
            kill -0 "$pid" 2>/dev/null || break
            sleep 0.25
        done
        if kill -0 "$pid" 2>/dev/null; then
            kill -KILL "$pid" 2>/dev/null || true
        fi
        echo "[OK] stopped $label PID $pid"
    fi
    rm -f "$pid_file"
}

stop_pid_file client "$RUNTIME_DIR/client.pid" 'internnav_nav_client\.py'
stop_pid_file Nav2 "$RUNTIME_DIR/nav2.pid" 'bringup_spirit_glim_launch\.py'
stop_pid_file inference-server "$RUNTIME_DIR/server.pid" \
    'docker_start_server_in_container\.sh|http_internvla_server\.py'

echo "[OK] InternNav-owned processes stopped"
