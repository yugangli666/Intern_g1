#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
NAV_WS_SETUP="${NAV_WS_SETUP:-/workspace/nav_ws/install/setup.bash}"
LIVOX_WS="${LIVOX_WS:-/home/spirit-ai/thor_livox_mid360s_installer/livox_ws}"

ACTION="${ACTION:-run}"
KEEP_DEPS_RUNNING="${KEEP_DEPS_RUNNING:-1}"
STOP_DEPS_FILE="${STOP_DEPS_FILE:-}"
DEPS_ENV_FILE="${DEPS_ENV_FILE:-/tmp/internnav_oneclick_deps.env}"
SKIP_RUNTIME_INSTALL="${SKIP_RUNTIME_INSTALL:-1}"

SERVER_URL="${SERVER_URL:-http://127.0.0.1:5801/eval_dual}"
HEALTH_URL="${HEALTH_URL:-${SERVER_URL%/eval_dual}/health}"
PRIMARY_RGB_TOPIC="${PRIMARY_RGB_TOPIC:-/moz_robot/camera/cam_high/image_raw}"
SECONDARY_RGB_TOPIC="${SECONDARY_RGB_TOPIC:-/moz_robot/camera/cam_high_extra/image_raw}"
ODOM_TOPIC="${ODOM_TOPIC:-/moz1/odom_global}"
LIVOX_TOPIC="${LIVOX_TOPIC:-/livox/lidar}"
INSTRUCTION="${INSTRUCTION:-}"
if [ -z "$INSTRUCTION" ] && [ "$#" -gt 0 ]; then
    INSTRUCTION="$*"
fi
WATCHDOG_SECONDS="${WATCHDOG_SECONDS:-300}"
SPIN_OVERSHOOT_DEGREES="${SPIN_OVERSHOOT_DEGREES:-5.0}"
SPIN_CONTROL_MODE="${SPIN_CONTROL_MODE:-fallback_turn}"
SPIN_REPLAN_FPS="${SPIN_REPLAN_FPS:-1.0}"
DISCRETE_TURN_DEGREES="${DISCRETE_TURN_DEGREES:-15.0}"
MAX_FALLBACK_TURN_DEGREES_PER_SEGMENT="${MAX_FALLBACK_TURN_DEGREES_PER_SEGMENT:-${MAX_FALLBACK_TURN_DEGREES:-45.0}}"
MAX_FALLBACK_TURN_TOTAL_DEGREES="${MAX_FALLBACK_TURN_TOTAL_DEGREES:-180.0}"
TURN_ONLY_MAX_INFERENCES="${TURN_ONLY_MAX_INFERENCES:-8}"
FALLBACK_TURN_TIMEOUT="${FALLBACK_TURN_TIMEOUT:-20.0}"
FALLBACK_TURN_RESET_ON_TRAJECTORY="${FALLBACK_TURN_RESET_ON_TRAJECTORY:-1}"
COMMAND_ANGULAR_SIGN="${COMMAND_ANGULAR_SIGN:-1.0}"
TRAJECTORY_TRACKER="${TRAJECTORY_TRACKER:-hybrid}"
MPC_ODOM_MAX_AGE="${MPC_ODOM_MAX_AGE:-1.0}"
MPC_START_GRACE="${MPC_START_GRACE:-1.0}"
MPC_MAX_TRACK_DISTANCE="${MPC_MAX_TRACK_DISTANCE:-0.8}"
MPC_HORIZON="${MPC_HORIZON:-12}"
MPC_REF_GAP="${MPC_REF_GAP:-3}"
MPC_DESIRED_V="${MPC_DESIRED_V:-0.10}"
MPC_V_MAX="${MPC_V_MAX:-0.15}"
MPC_W_MAX="${MPC_W_MAX:-0.25}"
MPC_GOAL_TOLERANCE="${MPC_GOAL_TOLERANCE:-0.05}"
MPC_CONTROL_RATE="${MPC_CONTROL_RATE:-10.0}"
MPC_MAX_SOLVE_TIME="${MPC_MAX_SOLVE_TIME:-0.15}"
MPC_PROGRESS_TIMEOUT="${MPC_PROGRESS_TIMEOUT:-2.0}"
MPC_MIN_PROGRESS="${MPC_MIN_PROGRESS:-0.03}"
MPC_MAX_CROSS_TRACK="${MPC_MAX_CROSS_TRACK:-0.25}"
PURE_PURSUIT_LOOKAHEAD="${PURE_PURSUIT_LOOKAHEAD:-0.25}"
PURE_PURSUIT_ALIGN_DEGREES="${PURE_PURSUIT_ALIGN_DEGREES:-45.0}"
PURE_PURSUIT_PROGRESS_TIMEOUT="${PURE_PURSUIT_PROGRESS_TIMEOUT:-3.0}"
TURN_DIRECTION_CHECK_DELAY="${TURN_DIRECTION_CHECK_DELAY:-0.5}"
TURN_DIRECTION_MIN_YAW_DEGREES="${TURN_DIRECTION_MIN_YAW_DEGREES:-3.0}"
FAST_CHECK_SECONDS="${FAST_CHECK_SECONDS:-2}"
PRIMARY_CAMERA_TIMEOUT="${PRIMARY_CAMERA_TIMEOUT:-8}"
SECONDARY_CAMERA_TIMEOUT="${SECONDARY_CAMERA_TIMEOUT:-2}"
ODOM_TIMEOUT="${ODOM_TIMEOUT:-4}"
CAMERA_CONFIG="${CAMERA_CONFIG:-$PROJECT_ROOT/scripts/realworld/camera_configs/moz1_dummy_identity.json}"
CAMERA_POSE_JSON="${CAMERA_POSE_JSON:-}"
CAMERA_POSE_SOURCE="${CAMERA_POSE_SOURCE:-}"
if [ "$ACTION" = "run" ]; then
    RESULT_DIR="${RESULT_DIR:-$PROJECT_ROOT/experiment_records/vln_oneclick_$(date +%Y%m%d_%H%M%S)}"
    STARTUP_LOG_DIR="$RESULT_DIR/startup_logs"
    mkdir -p "$STARTUP_LOG_DIR"
    PREFLIGHT_LOG="$STARTUP_LOG_DIR/preflight.log"
    exec > >(tee -a "$PREFLIGHT_LOG") 2>&1
else
    RESULT_DIR="${RESULT_DIR:-}"
    STARTUP_LOG_DIR="${STARTUP_LOG_DIR:-}"
fi

started_server=0
started_nav2=0
started_livox=0
server_pid=""
server_pgid=""
nav2_pid=""
nav2_pgid=""
livox_pid=""
livox_pgid=""
client_started=0

log() {
    echo "[$(date +'%F %T')] $*"
}

source_ros() {
    if [ ! -f "$NAV_WS_SETUP" ]; then
        echo "[ERROR] ROS setup not found: $NAV_WS_SETUP" >&2
        exit 2
    fi
    set +u
    # shellcheck disable=SC1090
    source "$NAV_WS_SETUP"
    set -u
}

safe_stop_robot() {
    log "Publishing zero velocity and stop-task signal"
    ros2 topic pub --once /cmd_vel_nav geometry_msgs/msg/Twist '{}' >/tmp/internnav_oneclick_stop_cmd.log 2>&1 || true
    ros2 topic pub --once /robot_nav_stop_task std_msgs/msg/Bool '{data: true}' >/tmp/internnav_oneclick_stop_task.log 2>&1 || true
}

process_group_alive() {
    local pgid="$1"
    [ -n "$pgid" ] && kill -0 -- "-$pgid" 2>/dev/null
}

stop_process_group() {
    local pgid="$1"
    local pid="${2:-}"
    [ -z "$pgid" ] && [ -n "$pid" ] && pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')"
    if [ -n "$pgid" ]; then
        kill -INT -- "-$pgid" 2>/dev/null || true
        for _ in $(seq 1 30); do
            process_group_alive "$pgid" || return 0
            sleep 0.2
        done
        kill -TERM -- "-$pgid" 2>/dev/null || true
    elif [ -n "$pid" ]; then
        kill -INT "$pid" 2>/dev/null || true
    fi
}

start_detached() {
    local name="$1"
    local log_file="$2"
    local command="$3"
    local pid_var="$4"
    local pgid_var="$5"
    nohup setsid bash -lc "$command" > "$log_file" 2>&1 < /dev/null &
    local pid=$!
    sleep 0.2
    local pgid
    pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')"
    [ -z "$pgid" ] && pgid="$pid"
    printf -v "$pid_var" '%s' "$pid"
    printf -v "$pgid_var" '%s' "$pgid"
    log "Started $name pid=$pid pgid=$pgid"
}

stop_started_deps() {
    if [ "$started_server" = "1" ]; then
        stop_process_group "$server_pgid" "$server_pid"
    fi
    if [ "$started_nav2" = "1" ]; then
        stop_process_group "$nav2_pgid" "$nav2_pid"
    fi
    if [ "$started_livox" = "1" ]; then
        stop_process_group "$livox_pgid" "$livox_pid"
    fi
}

cleanup() {
    local rc=$?
    if [ "$client_started" = "1" ]; then
        safe_stop_robot
    fi
    if [ "$KEEP_DEPS_RUNNING" != "1" ]; then
        stop_started_deps
    elif [ "$started_server$started_nav2$started_livox" != "000" ]; then
        log "Keeping started dependencies running (KEEP_DEPS_RUNNING=1)"
    fi
    exit "$rc"
}
trap cleanup EXIT

write_pids() {
    local pids_file="$STARTUP_LOG_DIR/pids.env"
    {
        echo "RESULT_DIR=$RESULT_DIR"
        echo "started_server=$started_server"
        echo "server_pid=$server_pid"
        echo "server_pgid=$server_pgid"
        echo "started_nav2=$started_nav2"
        echo "nav2_pid=$nav2_pid"
        echo "nav2_pgid=$nav2_pgid"
        echo "started_livox=$started_livox"
        echo "livox_pid=$livox_pid"
        echo "livox_pgid=$livox_pgid"
    } > "$pids_file"
    cp "$pids_file" "$DEPS_ENV_FILE" 2>/dev/null || true
}

node_list_clean() {
    ros2 node list 2>/dev/null | sed 's#^/##' || true
}

required_nodes_ready() {
    local nodes
    nodes="$(node_list_clean)"
    grep -qx velocity_smoother <<<"$nodes" \
        && grep -qx collision_monitor <<<"$nodes" \
        && grep -qx message_forward <<<"$nodes"
}

topic_once() {
    local topic="$1"
    local type="${2:-}"
    local seconds="${3:-$FAST_CHECK_SECONDS}"
    if [ -n "$type" ]; then
        timeout "$seconds" ros2 topic echo --once "$topic" "$type" >/dev/null 2>&1
    else
        timeout "$seconds" ros2 topic echo --once "$topic" >/dev/null 2>&1
    fi
}

health_ok() {
    curl -fsS --max-time 1 "$HEALTH_URL" >/dev/null 2>&1
}

wait_for_health() {
    for _ in $(seq 1 120); do
        if health_ok; then
            return 0
        fi
        sleep 1
    done
    return 1
}

wait_for_required_nodes() {
    for _ in $(seq 1 90); do
        if required_nodes_ready; then
            return 0
        fi
        sleep 1
    done
    return 1
}

print_camera_help() {
    cat <<'HELP'
[ERROR] Primary camera has no frames.
Start camera stack from another terminal:
  docker exec -it agent_ui bash
  cd /home/spirit-ai/codebase/runcode/livingroom_debug/mozbrain
  ./moz1_run_robot_control_posttrain_livingroom_sync.sh
HELP
}

find_latest_pids_file() {
    if [ -n "$STOP_DEPS_FILE" ]; then
        echo "$STOP_DEPS_FILE"
        return 0
    fi
    if [ -f "$DEPS_ENV_FILE" ]; then
        echo "$DEPS_ENV_FILE"
        return 0
    fi
    find "$PROJECT_ROOT/experiment_records" -path '*/startup_logs/pids.env' -type f 2>/dev/null \
        | sort -r \
        | head -1
}

stop_deps_action() {
    local pids_file
    pids_file="$(find_latest_pids_file)"
    if [ -z "$pids_file" ] || [ ! -f "$pids_file" ]; then
        echo "[ERROR] No pids.env found. Set STOP_DEPS_FILE=/path/to/pids.env" >&2
        exit 2
    fi
    log "Stopping dependencies recorded in $pids_file"
    # shellcheck disable=SC1090
    source "$pids_file"
    safe_stop_robot
    if [ "${started_server:-0}" = "1" ]; then
        stop_process_group "${server_pgid:-}" "${server_pid:-}"
    fi
    if [ "${started_nav2:-0}" = "1" ]; then
        stop_process_group "${nav2_pgid:-}" "${nav2_pid:-}"
    fi
    if [ "${started_livox:-0}" = "1" ]; then
        stop_process_group "${livox_pgid:-}" "${livox_pid:-}"
    fi
    log "stop-deps completed"
}

generate_post_run_analysis() {
    python3 - "$RESULT_DIR" <<'PY'
import json
from pathlib import Path
import sys

run = Path(sys.argv[1])
summary_path = run / "summary.json"
events_path = run / "events.jsonl"
metadata_path = run / "metadata.json"
summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
events = [json.loads(line) for line in events_path.read_text().splitlines()] if events_path.exists() else []
metadata = json.loads(metadata_path.read_text()) if metadata_path.exists() else {}
instruction = metadata.get("instruction", "unknown")

responses = []
for path in sorted(run.glob("response_*.json")):
    responses.append((path.name, json.loads(path.read_text())))

counts = {"trajectory": 0, "discrete_action": 0, "pixel_goal": 0, "stop": 0, "other": 0}
actions = {}
for _name, body in responses:
    matched = False
    if "trajectory" in body:
        counts["trajectory"] += 1
        matched = True
    if "pixel_goal" in body:
        counts["pixel_goal"] += 1
        matched = True
    if "discrete_action" in body:
        counts["discrete_action"] += 1
        matched = True
        action = tuple(body.get("discrete_action") or [])
        actions[action] = actions.get(action, 0) + 1
        if any(item in {0, 9} for item in action):
            counts["stop"] += 1
    if not matched:
        counts["other"] += 1

status_states = summary.get("status_states") or []
topic_counts = summary.get("topic_counts") or {}
cmd = (summary.get("max_twist_commands") or {}).get("/cmd_vel_nav") or {}
base = summary.get("max_base_command") or {}
durations = summary.get("nonzero_command_durations") or {}
delay = summary.get("approx_cmd_vel_nav_to_base_delay_s") or {}
yaw_change = summary.get("yaw_change_deg")
fault_codes = summary.get("fault_codes") or []
grounding_counts = {}
for event in events:
    status = event.get("grounding_status")
    if status:
        grounding_counts[status] = grounding_counts.get(status, 0) + 1
tracker_fallbacks = [
    event for event in events if event.get("state") == "MPC_FALLBACK_TO_PURE_PURSUIT"
]
tracker_ticks = [event for event in events if event.get("state") == "TRACKER_TICK"]

turn_budget_reached = any(
    event.get("state") == "MODEL_TURN_ONLY"
    or event.get("reason") in {"fallback_turn_limit_reached"}
    or str(event.get("reason", "")).startswith("turn-only responses reached")
    for event in events
)
diagnosis = []
if counts["trajectory"] == 0 and counts["discrete_action"] > 0:
    diagnosis.append("MODEL_NO_TRAJECTORY")
if turn_budget_reached:
    diagnosis.append("ACTION_HOLD_BY_TURN_BUDGET")
if (
    topic_counts.get("/cmd_vel_nav", 0) > 0
    and topic_counts.get("/cmd_vel", 0) > 0
    and topic_counts.get("/mx_base_vel_command", 0) > 0
):
    diagnosis.append("ROS_CHAIN_OK")
try:
    if abs(float(yaw_change or 0.0)) > 5.0:
        diagnosis.append("BASE_EXECUTED_YAW")
except (TypeError, ValueError):
    pass
if metadata.get("depth_mode") == "dummy":
    diagnosis.append("DUMMY_DEPTH_MODE")
if "TURN_DIRECTION_MISMATCH" in fault_codes:
    diagnosis.append("TURN_DIRECTION_MISMATCH")
if tracker_fallbacks:
    diagnosis.append("MPC_FALLBACK_USED")
if any(
    event.get("state") == "MPC_COMPLETE" and event.get("active_tracker") == "pure_pursuit"
    for event in events
):
    diagnosis.append("PURE_PURSUIT_FALLBACK_SUCCEEDED")
if any(
    event.get("state") == "HOLD" and event.get("active_tracker") == "pure_pursuit"
    for event in events
):
    diagnosis.append("PURE_PURSUIT_FAILED")

grid_path = None
try:
    from PIL import Image, ImageDraw

    images = sorted(run.glob("input_*.jpg"))
    if images:
        if len(images) <= 9:
            selected = images
        else:
            indexes = [round(i * (len(images) - 1) / 8) for i in range(9)]
            selected = [images[i] for i in indexes]
        thumbs = []
        for image_path in selected:
            image = Image.open(image_path).convert("RGB")
            image.thumbnail((320, 240))
            canvas = Image.new("RGB", (320, 270), "white")
            x = (320 - image.width) // 2
            canvas.paste(image, (x, 0))
            draw = ImageDraw.Draw(canvas)
            label = image_path.stem.replace("input_", "#")
            draw.rectangle((0, 240, 320, 270), fill=(20, 20, 20))
            draw.text((10, 248), label, fill=(255, 255, 255))
            thumbs.append(canvas)
        while len(thumbs) < 9:
            thumbs.append(Image.new("RGB", (320, 270), "white"))
        grid = Image.new("RGB", (960, 810), "white")
        for idx, thumb in enumerate(thumbs[:9]):
            grid.paste(thumb, ((idx % 3) * 320, (idx // 3) * 270))
        grid_path = run / "visual_summary_grid.jpg"
        grid.save(grid_path, quality=92)
except Exception:
    grid_path = None

lines = [
    "# VLN One-Click Run Analysis",
    "",
    f"- Result dir: `{run}`",
    f"- Instruction: `{instruction}`",
    "- Mode: `mpc_tracking`, one-click startup, no manual adjustment during run.",
    f"- Diagnosis tags: `{', '.join(diagnosis) if diagnosis else 'unknown'}`",
]
if grid_path is not None:
    lines.append(f"- Visual summary: `{grid_path.name}`")
lines.extend(
    [
        "",
        "## Outcome",
        f"- Final state: `{status_states[-1] if status_states else 'unknown'}`",
        f"- Emergency messages: `{summary.get('emergency_messages')}`",
        f"- Max displacement: `{summary.get('max_displacement_m')}` m",
        f"- Yaw change: `{summary.get('yaw_change_deg')}` deg",
        f"- Max `/cmd_vel_nav`: linear `{cmd.get('linear')}`, angular `{cmd.get('angular')}`",
        f"- Max base command: linear `{base.get('linear')}`, angular `{base.get('angular')}`",
        f"- Max fallback yaw coverage: `{summary.get('max_status_fallback_turn_yaw_coverage_deg')}` deg",
        f"- Max fallback actual travel: `{summary.get('max_status_fallback_turn_actual_travel_deg')}` deg",
        f"- Fallback budget source: `{summary.get('fallback_turn_budget_source')}`",
        f"- Fault codes: `{fault_codes}`",
        "",
        "## Model Outputs",
        f"- Total inferences: `{len(responses)}`",
        f"- `discrete_action`: `{counts['discrete_action']}`",
        f"- `trajectory`: `{counts['trajectory']}`",
        f"- `pixel_goal`: `{counts['pixel_goal']}`",
        f"- STOP-like discrete actions: `{counts['stop']}`",
        f"- Discrete action counts: `{actions}`",
        f"- Grounding status counts: `{grounding_counts}`",
        "- Target lock: `unverified` (no object detector configured)",
        f"- Configured trajectory tracker: `{metadata.get('trajectory_tracker', 'mpc')}`",
        f"- MPC to Pure Pursuit fallbacks: `{len(tracker_fallbacks)}`",
        f"- Tracker diagnostic samples: `{len(tracker_ticks)}`",
        "",
        "## Command Chain",
        f"- Topic counts: `{topic_counts}`",
        f"- Nonzero durations: `{durations}`",
        f"- Approx `/cmd_vel_nav` to base delay: `{delay}`",
        "",
        "## Action Timeline",
    ]
)
for event in events:
    parts = [f"request `{event.get('request_id')}`", f"state `{event.get('state')}`"]
    for key in (
        "reason",
        "motion_kind",
        "motion_succeeded",
        "mpc_succeeded",
        "active_tracker",
        "fallback_reason",
        "mpc_solve_ms",
        "wrapped_heading_error_deg",
        "cross_track_error",
        "path_progress",
        "goal_distance",
        "raw_v",
        "raw_w",
        "published_v",
        "published_w",
        "world_trajectory_points",
        "world_path_arc_length",
        "latency_ms",
        "turn_only_inferences",
        "turn_only_elapsed_s",
        "turn_only_total_deg",
        "fallback_turn_duration_s",
        "fallback_turn_used_deg",
        "fallback_turn_remaining_deg",
        "fallback_turn_nominal_segment_deg",
        "fallback_turn_actual_segment_deg",
        "fallback_turn_actual_travel_deg",
        "fallback_turn_yaw_coverage_deg",
        "fallback_turn_remaining_coverage_deg",
        "grounding_status",
        "target_locked",
        "fault_code",
        "expected_yaw_sign",
        "observed_yaw_delta_deg",
        "command_angular_z",
        "actions_used",
        "actions_raw",
    ):
        if key in event:
            parts.append(f"{key}=`{event.get(key)}`")
    lines.append("- " + ", ".join(parts))

lines.extend(
    [
        "",
        "## Layered Conclusion",
        "- `MODEL_NO_TRAJECTORY` means the model never produced a forward trajectory for MPC.",
        "- `ACTION_HOLD_BY_TURN_BUDGET` means actual odom yaw coverage or the turn-only inference limit ended the search.",
        "- `ROS_CHAIN_OK` means velocity messages reached the ROS command chain topics recorded in the bag.",
        "- `BASE_EXECUTED_YAW` means odometry changed enough to show actual yaw execution.",
        "- `TURN_DIRECTION_MISMATCH` means odom yaw moved opposite to the model's left/right action and triggered E_STOP.",
        "- `DUMMY_DEPTH_MODE` means real depth/camera extrinsics were not yet used for grounding.",
        "- `target_locked` remains unverified until an independent object detector is configured.",
    ]
)
(run / "post_run_analysis.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
print(run / "post_run_analysis.md")
PY
}
status_action() {
    local failed=0
    log "Status check only; no dependencies will be started and no motion command will be sent"
    if health_ok; then log "OK inference service: $HEALTH_URL"; else log "MISSING inference service: $HEALTH_URL"; failed=1; fi
    if topic_once "$LIVOX_TOPIC" "" "$FAST_CHECK_SECONDS"; then log "OK Livox data: $LIVOX_TOPIC"; else log "MISSING Livox data: $LIVOX_TOPIC"; failed=1; fi
    if required_nodes_ready; then log "OK Nav2 velocity chain"; else log "MISSING Nav2 velocity chain nodes"; failed=1; fi
    if topic_once "$PRIMARY_RGB_TOPIC" sensor_msgs/msg/Image "$PRIMARY_CAMERA_TIMEOUT"; then log "OK primary camera: $PRIMARY_RGB_TOPIC"; else log "MISSING primary camera: $PRIMARY_RGB_TOPIC"; failed=1; fi
    if topic_once "$SECONDARY_RGB_TOPIC" sensor_msgs/msg/Image "$SECONDARY_CAMERA_TIMEOUT"; then log "OK secondary camera: $SECONDARY_RGB_TOPIC"; else log "WARN secondary camera not confirmed: $SECONDARY_RGB_TOPIC"; fi
    if topic_once "$ODOM_TOPIC" nav_msgs/msg/Odometry "$ODOM_TIMEOUT"; then log "OK odom: $ODOM_TOPIC"; else log "MISSING odom: $ODOM_TOPIC"; failed=1; fi
    return "$failed"
}

run_action() {
    if [ -z "$INSTRUCTION" ]; then
        cat >&2 <<'HELP'
[ERROR] INSTRUCTION is required for ACTION=run.
Examples:
  INSTRUCTION="Move toward the target object and stop in front of it." bash scripts/realworld/run_vln_dishwasher_oneclick.sh
  bash scripts/realworld/run_vln_dishwasher_oneclick.sh "Move toward the target object and stop in front of it."
HELP
        exit 2
    fi

    log "Checking inference service: $HEALTH_URL"
    if health_ok; then
        log "Reusing healthy inference service"
    else
        log "Starting inference service"
        server_cmd="cd \"$PROJECT_ROOT\" && SKIP_RUNTIME_INSTALL=\"$SKIP_RUNTIME_INSTALL\" exec bash docker_start_server_in_container.sh"
        start_detached "inference service" "$STARTUP_LOG_DIR/server.log" "$server_cmd" server_pid server_pgid
        started_server=1
        write_pids
        if ! wait_for_health; then
            echo "[ERROR] Inference service did not become healthy. See $STARTUP_LOG_DIR/server.log" >&2
            exit 2
        fi
    fi

    log "Checking Livox topic: $LIVOX_TOPIC"
    if topic_once "$LIVOX_TOPIC" "" "$FAST_CHECK_SECONDS"; then
        log "Reusing existing Livox topic with data"
    else
        log "Starting Livox driver"
        livox_cmd="cd \"$LIVOX_WS\" && source install/setup.bash && exec ros2 launch livox_ros_driver2 msg_MID360s_launch.py"
        start_detached "Livox driver" "$STARTUP_LOG_DIR/livox.log" "$livox_cmd" livox_pid livox_pgid
        started_livox=1
        write_pids
        for _ in $(seq 1 60); do
            if topic_once "$LIVOX_TOPIC" "" 3; then
                break
            fi
            sleep 1
        done
        if ! topic_once "$LIVOX_TOPIC" "" 3; then
            echo "[ERROR] Livox topic has no data after startup: $LIVOX_TOPIC" >&2
            exit 2
        fi
    fi

    log "Checking Nav2 velocity chain"
    if required_nodes_ready; then
        log "Reusing existing Nav2 velocity chain"
    else
        log "Starting Nav2 bringup"
        nav2_cmd="cd /workspace && source \"$NAV_WS_SETUP\" && exec ros2 launch nav2_bringup bringup_spirit_glim_launch.py"
        start_detached "Nav2 bringup" "$STARTUP_LOG_DIR/nav2.log" "$nav2_cmd" nav2_pid nav2_pgid
        started_nav2=1
        write_pids
        if ! wait_for_required_nodes; then
            echo "[ERROR] Nav2 velocity chain did not become ready. See $STARTUP_LOG_DIR/nav2.log" >&2
            exit 2
        fi
    fi

    write_pids

    log "Checking primary camera: $PRIMARY_RGB_TOPIC"
    if ! topic_once "$PRIMARY_RGB_TOPIC" sensor_msgs/msg/Image "$PRIMARY_CAMERA_TIMEOUT"; then
        print_camera_help
        exit 2
    fi
    log "Primary camera OK"

    log "Checking secondary camera: $SECONDARY_RGB_TOPIC"
    if topic_once "$SECONDARY_RGB_TOPIC" sensor_msgs/msg/Image "$SECONDARY_CAMERA_TIMEOUT"; then
        log "Secondary camera OK"
    else
        log "Secondary camera not confirmed; continuing because primary camera is used for inference"
    fi

    log "Checking odom: $ODOM_TOPIC"
    if ! topic_once "$ODOM_TOPIC" nav_msgs/msg/Odometry "$ODOM_TIMEOUT"; then
        echo "[ERROR] Odom topic has no frames: $ODOM_TOPIC" >&2
        exit 2
    fi
    log "Odom OK"

    log "Launching VLN closed-loop test"
    log "Instruction: $INSTRUCTION"
    client_started=1
    set +e
    timeout --foreground --signal=INT --kill-after=60 "$WATCHDOG_SECONDS" env \
        CONTROL_MODE=mpc_tracking \
        TRAJECTORY_TRACKER="$TRAJECTORY_TRACKER" \
        COMMAND_ANGULAR_SIGN="$COMMAND_ANGULAR_SIGN" \
        ENABLE_MOTION=1 \
        MOTION_ARMED=YES \
        ALLOW_FORWARD_MOTION=1 \
        FORWARD_ARMED=YES \
        ALLOW_LEGACY_IDLE_STACK=1 \
        PRIMARY_RGB_TOPIC="$PRIMARY_RGB_TOPIC" \
        SECONDARY_RGB_TOPIC="$SECONDARY_RGB_TOPIC" \
        ODOM_TOPIC="$ODOM_TOPIC" \
        INSTRUCTION="$INSTRUCTION" \
        MAX_INFERENCES=0 \
        MAX_MOTION_STEPS=0 \
        MPC_ODOM_MAX_AGE="$MPC_ODOM_MAX_AGE" \
        MPC_START_GRACE="$MPC_START_GRACE" \
        MPC_MAX_TRACK_DISTANCE="$MPC_MAX_TRACK_DISTANCE" \
        MPC_HORIZON="$MPC_HORIZON" \
        MPC_REF_GAP="$MPC_REF_GAP" \
        MPC_DESIRED_V="$MPC_DESIRED_V" \
        MPC_V_MAX="$MPC_V_MAX" \
        MPC_W_MAX="$MPC_W_MAX" \
        MPC_GOAL_TOLERANCE="$MPC_GOAL_TOLERANCE" \
        MPC_CONTROL_RATE="$MPC_CONTROL_RATE" \
        MPC_MAX_SOLVE_TIME="$MPC_MAX_SOLVE_TIME" \
        MPC_PROGRESS_TIMEOUT="$MPC_PROGRESS_TIMEOUT" \
        MPC_MIN_PROGRESS="$MPC_MIN_PROGRESS" \
        MPC_MAX_CROSS_TRACK="$MPC_MAX_CROSS_TRACK" \
        PURE_PURSUIT_LOOKAHEAD="$PURE_PURSUIT_LOOKAHEAD" \
        PURE_PURSUIT_ALIGN_DEGREES="$PURE_PURSUIT_ALIGN_DEGREES" \
        PURE_PURSUIT_PROGRESS_TIMEOUT="$PURE_PURSUIT_PROGRESS_TIMEOUT" \
        MAX_SPIN_DEGREES=10.0 \
        MAX_FORWARD_DISTANCE=0.20 \
        LINEAR_SPEED=0.10 \
        ANGULAR_SPEED=0.20 \
        MAX_BASE_LINEAR=0.20 \
        MAX_BASE_ANGULAR=0.30 \
        YAW_TOLERANCE_DEGREES=2.0 \
        SPIN_OVERSHOOT_DEGREES="$SPIN_OVERSHOOT_DEGREES" \
        SPIN_CONTROL_MODE="$SPIN_CONTROL_MODE" \
        SPIN_REPLAN_FPS="$SPIN_REPLAN_FPS" \
        DISCRETE_TURN_DEGREES="$DISCRETE_TURN_DEGREES" \
        MAX_FALLBACK_TURN_DEGREES_PER_SEGMENT="$MAX_FALLBACK_TURN_DEGREES_PER_SEGMENT" \
        MAX_FALLBACK_TURN_TOTAL_DEGREES="$MAX_FALLBACK_TURN_TOTAL_DEGREES" \
        TURN_ONLY_MAX_INFERENCES="$TURN_ONLY_MAX_INFERENCES" \
        FALLBACK_TURN_TIMEOUT="$FALLBACK_TURN_TIMEOUT" \
        FALLBACK_TURN_RESET_ON_TRAJECTORY="$FALLBACK_TURN_RESET_ON_TRAJECTORY" \
        TURN_DIRECTION_CHECK_DELAY="$TURN_DIRECTION_CHECK_DELAY" \
        TURN_DIRECTION_MIN_YAW_DEGREES="$TURN_DIRECTION_MIN_YAW_DEGREES" \
        MOTION_TIMEOUT=15.0 \
        PREFLIGHT_WAIT=8.0 \
        CAMERA_CONFIG="$CAMERA_CONFIG" \
        CAMERA_POSE_JSON="$CAMERA_POSE_JSON" \
        CAMERA_POSE_SOURCE="$CAMERA_POSE_SOURCE" \
        RESULT_DIR="$RESULT_DIR" \
        bash "$SCRIPT_DIR/run_minimal_closed_loop.sh"
    client_rc=$?
    set -e

    safe_stop_robot
    client_started=0

    if [ ! -s "$RESULT_DIR/summary.json" ]; then
        log "summary.json missing; reindexing/analyzing rosbag"
        ros2 bag reindex "$RESULT_DIR/rosbag" || true
        python3 "$SCRIPT_DIR/analyze_closed_loop_bag.py" "$RESULT_DIR/rosbag" \
            --output "$RESULT_DIR/summary.json" | tee "$RESULT_DIR/summary.log" || true
    fi

    log "Generating post-run analysis"
    generate_post_run_analysis || true

    log "VLN one-click finished with client exit code: $client_rc"
    log "Result directory: $RESULT_DIR"
    return "$client_rc"
}

if [ -n "$RESULT_DIR" ]; then
    log "Result directory: $RESULT_DIR"
fi
if [ -n "$STARTUP_LOG_DIR" ]; then
    log "Startup logs: $STARTUP_LOG_DIR"
fi
log "Action: $ACTION"
source_ros

case "$ACTION" in
    run)
        run_action
        ;;
    status)
        status_action
        ;;
    stop-deps)
        stop_deps_action
        ;;
    *)
        echo "[ERROR] Unknown ACTION=$ACTION (expected run|status|stop-deps)" >&2
        exit 2
        ;;
esac
