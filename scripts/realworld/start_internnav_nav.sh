#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
RUNTIME_DIR="${INTERNNAV_RUNTIME_DIR:-/tmp/internnav_nav}"
mkdir -p "$RUNTIME_DIR"

exec 9>"$RUNTIME_DIR/start.lock"
if ! flock -n 9; then
    echo "[ERROR] another InternNav start/stop operation is in progress" >&2
    exit 2
fi

SERVER_URL="${SERVER_URL:-http://127.0.0.1:5801/eval_dual}"
SERVER_HEALTH_URL="${SERVER_HEALTH_URL:-${SERVER_URL%/eval_dual}/health}"
PRIMARY_RGB_TOPIC="${PRIMARY_RGB_TOPIC:-/moz_robot/camera/cam_high_extra/image_raw}"
SECONDARY_RGB_TOPIC="${SECONDARY_RGB_TOPIC:-/moz_robot/camera/cam_high/image_raw}"
ODOM_TOPIC="${ODOM_TOPIC:-/moz1/odom_global}"
GLOBAL_FRAME="${GLOBAL_FRAME:-moz1/map}"
BASE_FRAME="${BASE_FRAME:-moz1/base_link}"
INFERENCE_FPS="${INFERENCE_FPS:-0.2}"
INSTRUCTION="${INSTRUCTION:-Walk forward to the door}"
DEPTH_MODE="${DEPTH_MODE:-dummy}"
DRY_RUN="${DRY_RUN:-1}"
ENABLE_MOTION="${ENABLE_MOTION:-0}"
ALLOW_DUMMY_DEPTH_MOTION="${ALLOW_DUMMY_DEPTH_MOTION:-0}"
ALLOW_LEGACY_IDLE_STACK="${ALLOW_LEGACY_IDLE_STACK:-0}"
MOTION_ARMED="${MOTION_ARMED:-NO}"
MAX_MOTION_STEPS="${MAX_MOTION_STEPS:-1}"
MOTION_MAX_DISTANCE="${MOTION_MAX_DISTANCE:-0.15}"
MOTION_MAX_SPIN_DEGREES="${MOTION_MAX_SPIN_DEGREES:-10.0}"
MOTION_TIMEOUT="${MOTION_TIMEOUT:-5.0}"
MAX_LINEAR_COMMAND="${MAX_LINEAR_COMMAND:-0.15}"
MAX_ANGULAR_COMMAND="${MAX_ANGULAR_COMMAND:-0.30}"
MAX_INFERENCES="${MAX_INFERENCES:-0}"
RESULT_DIR="${RESULT_DIR:-$PROJECT_ROOT/experiment_records/internnav_nav/$(date +%Y%m%d_%H%M%S)}"
START_SERVER="${START_SERVER:-auto}"
START_NAV2="${START_NAV2:-1}"
AUTO_BUILD_NAV="${AUTO_BUILD_NAV:-1}"

if [ "$DEPTH_MODE" != "dummy" ]; then
    echo "[ERROR] only DEPTH_MODE=dummy is currently implemented" >&2
    exit 2
fi
if [ "$ENABLE_MOTION" = "1" ]; then
    if [ "$DRY_RUN" != "0" ] || [ "$ALLOW_DUMMY_DEPTH_MOTION" != "1" ] \
        || [ "$MOTION_ARMED" != "YES" ]; then
        echo "[ERROR] motion requires DRY_RUN=0, ALLOW_DUMMY_DEPTH_MOTION=1, MOTION_ARMED=YES" >&2
        exit 2
    fi
else
    if [ "$DRY_RUN" != "1" ]; then
        echo "[ERROR] DRY_RUN=0 requires ENABLE_MOTION=1" >&2
        exit 2
    fi
fi

pid_is_live() {
    local pid_file="$1"
    [ -s "$pid_file" ] && kill -0 "$(cat "$pid_file")" 2>/dev/null
}

for component in client nav2; do
    if pid_is_live "$RUNTIME_DIR/$component.pid"; then
        echo "[ERROR] $component is already running with PID $(cat "$RUNTIME_DIR/$component.pid")" >&2
        exit 2
    fi
done

owned_pids=()
started_ok=0
cleanup_on_error() {
    status=$?
    if [ "$started_ok" -eq 0 ]; then
        for pid in "${owned_pids[@]}"; do
            kill -TERM "$pid" 2>/dev/null || true
        done
    fi
    exit "$status"
}
trap cleanup_on_error EXIT

if [ ! -f "$PROJECT_ROOT/nav/install/setup.bash" ]; then
    if [ "$AUTO_BUILD_NAV" != "1" ]; then
        echo "[ERROR] local nav overlay is not built" >&2
        exit 2
    fi
    "$PROJECT_ROOT/nav/build_local_nav.sh"
fi
source "$PROJECT_ROOT/nav/setup_local_nav.sh"

if ! curl -fsS --max-time 3 "$SERVER_HEALTH_URL" >/dev/null 2>&1; then
    if [ "$START_SERVER" = "0" ]; then
        echo "[ERROR] inference server is unavailable and START_SERVER=0" >&2
        exit 2
    fi
    nohup bash "$PROJECT_ROOT/docker_start_server_in_container.sh" \
        > "$RUNTIME_DIR/server.log" 2>&1 &
    server_pid=$!
    printf '%s\n' "$server_pid" > "$RUNTIME_DIR/server.pid"
    owned_pids+=("$server_pid")

    server_ready=0
    for _ in $(seq 1 150); do
        if curl -fsS --max-time 3 "$SERVER_HEALTH_URL" >/dev/null 2>&1; then
            server_ready=1
            break
        fi
        kill -0 "$server_pid" 2>/dev/null || break
        sleep 2
    done
    if [ "$server_ready" -ne 1 ]; then
        echo "[ERROR] inference server did not become healthy; see $RUNTIME_DIR/server.log" >&2
        exit 2
    fi
fi

export SERVER_HEALTH_URL PRIMARY_RGB_TOPIC SECONDARY_RGB_TOPIC ODOM_TOPIC GLOBAL_FRAME BASE_FRAME
export ALLOW_LEGACY_IDLE_STACK
export PREFLIGHT_STATE_FILE="$RUNTIME_DIR/preflight.env"

if [ "$START_NAV2" = "1" ]; then
    "$SCRIPT_DIR/preflight_internnav_nav.sh" --require-clean

    nohup ros2 launch nav2_bringup bringup_spirit_glim_launch.py \
        start_nav_executor:=false \
        > "$RUNTIME_DIR/nav2.log" 2>&1 &
    nav2_pid=$!
    printf '%s\n' "$nav2_pid" > "$RUNTIME_DIR/nav2.pid"
    owned_pids+=("$nav2_pid")

    nav2_discovered=0
    for _ in $(seq 1 60); do
        if ! kill -0 "$nav2_pid" 2>/dev/null; then
            break
        fi
        actions="$(ros2 action list 2>/dev/null || true)"
        nodes="$(ros2 node list 2>/dev/null || true)"
        if grep -qx "/follow_path" <<< "$actions" \
            && grep -qx "/spin" <<< "$actions" \
            && grep -Eq '(^|/)message_forward$' <<< "$nodes"; then
            nav2_discovered=1
            break
        fi
        sleep 2
    done
    if [ "$nav2_discovered" -ne 1 ]; then
        echo "[ERROR] local Nav2 did not expose the required nodes/actions; see $RUNTIME_DIR/nav2.log" >&2
        exit 2
    fi
    if [ "$ENABLE_MOTION" = "1" ]; then
        "$SCRIPT_DIR/configure_safe_nav2_motion.sh"
        "$SCRIPT_DIR/preflight_internnav_nav.sh" --motion
    else
        "$SCRIPT_DIR/preflight_internnav_nav.sh" --runtime
    fi
else
    if [ "$ENABLE_MOTION" = "1" ]; then
        "$SCRIPT_DIR/configure_safe_nav2_motion.sh"
        "$SCRIPT_DIR/preflight_internnav_nav.sh" --motion
    else
        "$SCRIPT_DIR/preflight_internnav_nav.sh"
    fi
fi

client_args=(
    --server-url "$SERVER_URL"
    --instruction "$INSTRUCTION"
    --primary-rgb-topic "$PRIMARY_RGB_TOPIC"
    --secondary-rgb-topic "$SECONDARY_RGB_TOPIC"
    --odom-topic "$ODOM_TOPIC"
    --global-frame "$GLOBAL_FRAME"
    --base-frame "$BASE_FRAME"
    --inference-fps "$INFERENCE_FPS"
    --depth-mode dummy
    --max-inferences "$MAX_INFERENCES"
    --output-dir "$RESULT_DIR"
    --lock-file "$RUNTIME_DIR/client.lock"
)

if [ "$ENABLE_MOTION" = "1" ]; then
    export INTERNNAV_MOTION_ARMED=YES
    client_args+=(
        --no-dry-run
        --enable-motion
        --allow-dummy-depth-motion
        --max-motion-steps "$MAX_MOTION_STEPS"
        --motion-max-distance "$MOTION_MAX_DISTANCE"
        --motion-max-spin-degrees "$MOTION_MAX_SPIN_DEGREES"
        --motion-timeout "$MOTION_TIMEOUT"
        --max-linear-command "$MAX_LINEAR_COMMAND"
        --max-angular-command "$MAX_ANGULAR_COMMAND"
    )
    if [ "$ALLOW_LEGACY_IDLE_STACK" = "1" ]; then
        client_args+=(--allow-legacy-idle-stack)
    fi
else
    client_args+=(--dry-run)
fi

nohup python3 "$SCRIPT_DIR/internnav_nav_client.py" "${client_args[@]}" \
    > "$RUNTIME_DIR/client.log" 2>&1 &
client_pid=$!
printf '%s\n' "$client_pid" > "$RUNTIME_DIR/client.pid"
owned_pids+=("$client_pid")

sleep 2
if ! kill -0 "$client_pid" 2>/dev/null; then
    echo "[ERROR] InternNav client exited during startup; see $RUNTIME_DIR/client.log" >&2
    exit 2
fi

started_ok=1
trap - EXIT
if [ "$ENABLE_MOTION" = "1" ]; then
    echo "[OK] InternNav guarded motion started"
else
    echo "[OK] InternNav dry-run started"
fi
echo "  client PID: $client_pid"
if [ "$START_NAV2" = "1" ]; then
    echo "  Nav2 PID:   $nav2_pid"
fi
echo "  results:    $RESULT_DIR"
echo "  logs:       $RUNTIME_DIR"
