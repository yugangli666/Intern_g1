#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

SERVER_URL="${SERVER_URL:-http://127.0.0.1:5801/eval_dual}"
HEALTH_URL="${HEALTH_URL:-${SERVER_URL%/eval_dual}/health}"
PRIMARY_RGB_TOPIC="${PRIMARY_RGB_TOPIC:-/camera/cam_high_extra/image_undistorted}"
SECONDARY_RGB_TOPIC="${SECONDARY_RGB_TOPIC:-/camera/cam_high/image_raw}"
ODOM_TOPIC="${ODOM_TOPIC:-/moz1/odom_global}"
INSTRUCTION="${INSTRUCTION:-Walk forward to the door}"
ENABLE_MOTION="${ENABLE_MOTION:-0}"
ALLOW_FORWARD_MOTION="${ALLOW_FORWARD_MOTION:-0}"
MOTION_ARMED="${MOTION_ARMED:-NO}"
FORWARD_ARMED="${FORWARD_ARMED:-NO}"
ALLOW_LEGACY_IDLE_STACK="${ALLOW_LEGACY_IDLE_STACK:-0}"
MAX_INFERENCES="${MAX_INFERENCES:-1}"
MAX_MOTION_STEPS="${MAX_MOTION_STEPS:-1}"
MAX_SPIN_DEGREES="${MAX_SPIN_DEGREES:-3.0}"
SKIP_TURN_ONLY_MOTION="${SKIP_TURN_ONLY_MOTION:-0}"
MAX_FORWARD_DISTANCE="${MAX_FORWARD_DISTANCE:-0.04}"
LINEAR_SPEED="${LINEAR_SPEED:-0.04}"
ANGULAR_SPEED="${ANGULAR_SPEED:-0.08}"
MAX_BASE_LINEAR="${MAX_BASE_LINEAR:-0.08}"
MAX_BASE_ANGULAR="${MAX_BASE_ANGULAR:-0.12}"
USE_BASE_MOTION_SERVICE="${USE_BASE_MOTION_SERVICE:-1}"
BASE_MOTION_SERVICE_TIMEOUT="${BASE_MOTION_SERVICE_TIMEOUT:-3.0}"
YAW_TOLERANCE_DEGREES="${YAW_TOLERANCE_DEGREES:-1.0}"
SPIN_OVERSHOOT_DEGREES="${SPIN_OVERSHOOT_DEGREES:-2.0}"
SPIN_CONTROL_MODE="${SPIN_CONTROL_MODE:-target_yaw}"
SPIN_REPLAN_FPS="${SPIN_REPLAN_FPS:-1.0}"
DISCRETE_TURN_DEGREES="${DISCRETE_TURN_DEGREES:-15.0}"
MAX_FALLBACK_TURN_DEGREES_PER_SEGMENT="${MAX_FALLBACK_TURN_DEGREES_PER_SEGMENT:-${MAX_FALLBACK_TURN_DEGREES:-45.0}}"
MAX_FALLBACK_TURN_TOTAL_DEGREES="${MAX_FALLBACK_TURN_TOTAL_DEGREES:-180.0}"
TURN_ONLY_MAX_INFERENCES="${TURN_ONLY_MAX_INFERENCES:-8}"
FALLBACK_TURN_TIMEOUT="${FALLBACK_TURN_TIMEOUT:-20.0}"
FALLBACK_TURN_RESET_ON_TRAJECTORY="${FALLBACK_TURN_RESET_ON_TRAJECTORY:-1}"
TURN_DIRECTION_CHECK_DELAY="${TURN_DIRECTION_CHECK_DELAY:-0.5}"
TURN_DIRECTION_MIN_YAW_DEGREES="${TURN_DIRECTION_MIN_YAW_DEGREES:-3.0}"
MOTION_TIMEOUT="${MOTION_TIMEOUT:-3.0}"
COMMAND_ANGULAR_SIGN="${COMMAND_ANGULAR_SIGN:-1.0}"
PREFLIGHT_WAIT="${PREFLIGHT_WAIT:-8.0}"
CONTROL_MODE="${CONTROL_MODE:-pulse}"
TRAJECTORY_TRACKER="${TRAJECTORY_TRACKER:-mpc}"
MPC_DESIRED_V="${MPC_DESIRED_V:-0.10}"
MPC_V_MAX="${MPC_V_MAX:-0.15}"
MPC_W_MAX="${MPC_W_MAX:-0.25}"
MPC_GOAL_TOLERANCE="${MPC_GOAL_TOLERANCE:-0.05}"
MPC_ODOM_MAX_AGE="${MPC_ODOM_MAX_AGE:-1.0}"
MPC_START_GRACE="${MPC_START_GRACE:-1.0}"
MPC_MAX_TRACK_DISTANCE="${MPC_MAX_TRACK_DISTANCE:-0.8}"
MPC_HORIZON="${MPC_HORIZON:-12}"
MPC_REF_GAP="${MPC_REF_GAP:-3}"
MPC_CONTROL_RATE="${MPC_CONTROL_RATE:-10.0}"
MPC_MAX_SOLVE_TIME="${MPC_MAX_SOLVE_TIME:-0.15}"
MPC_PROGRESS_TIMEOUT="${MPC_PROGRESS_TIMEOUT:-2.0}"
MPC_MIN_PROGRESS="${MPC_MIN_PROGRESS:-0.03}"
MPC_MAX_CROSS_TRACK="${MPC_MAX_CROSS_TRACK:-0.25}"
PURE_PURSUIT_LOOKAHEAD="${PURE_PURSUIT_LOOKAHEAD:-0.25}"
PURE_PURSUIT_ALIGN_DEGREES="${PURE_PURSUIT_ALIGN_DEGREES:-45.0}"
PURE_PURSUIT_PROGRESS_TIMEOUT="${PURE_PURSUIT_PROGRESS_TIMEOUT:-3.0}"
CAMERA_CONFIG="${CAMERA_CONFIG:-$PROJECT_ROOT/scripts/realworld/camera_configs/moz1_dummy_identity.json}"
CAMERA_POSE_JSON="${CAMERA_POSE_JSON:-}"
CAMERA_POSE_SOURCE="${CAMERA_POSE_SOURCE:-}"
RESULT_DIR="${RESULT_DIR:-$PROJECT_ROOT/experiment_records/minimal_closed_loop/$(date +%Y%m%d_%H%M%S)}"

if [ "$ENABLE_MOTION" = "1" ] && [ "$MOTION_ARMED" != "YES" ]; then
    echo "[ERROR] motion requires ENABLE_MOTION=1 and MOTION_ARMED=YES" >&2
    exit 2
fi
if [ "$ALLOW_FORWARD_MOTION" = "1" ]; then
    if [ "$ENABLE_MOTION" != "1" ] || [ "$FORWARD_ARMED" != "YES" ]; then
        echo "[ERROR] forward requires motion mode and FORWARD_ARMED=YES" >&2
        exit 2
    fi
fi
if ! curl -fsS --max-time 3 "$HEALTH_URL" >/dev/null; then
    echo "[ERROR] inference service is not healthy: $HEALTH_URL" >&2
    exit 2
fi

case "$CONTROL_MODE" in
    pulse) ;;
    mpc_tracking)
        case "$TRAJECTORY_TRACKER" in
            mpc|hybrid|pure_pursuit) ;;
            *)
                echo "[ERROR] unknown TRAJECTORY_TRACKER: $TRAJECTORY_TRACKER (expected mpc|hybrid|pure_pursuit)" >&2
                exit 2
                ;;
        esac
        if [ "$TRAJECTORY_TRACKER" != "pure_pursuit" ] && ! python3 -c "import casadi" >/dev/null 2>&1; then
            echo "[INFO] control-mode mpc_tracking 需要 casadi，尝试安装..." >&2
            if ! pip3 install casadi --no-cache-dir; then
                echo "[ERROR] casadi 安装失败；mpc_tracking 无法运行" >&2
                exit 2
            fi
        fi
        echo "[INFO] control-mode=mpc_tracking tracker=$TRAJECTORY_TRACKER (desired_v=$MPC_DESIRED_V v_max=$MPC_V_MAX w_max=$MPC_W_MAX goal_tol=$MPC_GOAL_TOLERANCE max_track=$MPC_MAX_TRACK_DISTANCE mpc_rate=$MPC_CONTROL_RATE)"
        ;;
    *)
        echo "[ERROR] unknown CONTROL_MODE: $CONTROL_MODE (expected pulse|mpc_tracking)" >&2
        exit 2
        ;;
esac

set +u
source /workspace/nav_ws/install/setup.bash
set -u
mkdir -p "$RESULT_DIR"

bag_dir="$RESULT_DIR/rosbag"
ros2 bag record --storage mcap --output "$bag_dir" \
    /moz1/odom_global \
    /cmd_vel_nav \
    /cmd_vel_smoothed \
    /cmd_vel \
    /mx_base_vel_command \
    /internnav/status \
    /emergency_stop \
    > "$RESULT_DIR/rosbag.log" 2>&1 &
bag_pid=$!

stop_bag() {
    if kill -0 "$bag_pid" 2>/dev/null; then
        kill -INT "$bag_pid" 2>/dev/null || true
        for _ in $(seq 1 30); do
            kill -0 "$bag_pid" 2>/dev/null || break
            sleep 0.1
        done
        if kill -0 "$bag_pid" 2>/dev/null; then
            kill -TERM "$bag_pid" 2>/dev/null || true
        fi
        wait "$bag_pid" 2>/dev/null || true
    fi
}
trap stop_bag EXIT

client_args=(
    --server-url "$SERVER_URL"
    --instruction "$INSTRUCTION"
    --primary-rgb-topic "$PRIMARY_RGB_TOPIC"
    --secondary-rgb-topic "$SECONDARY_RGB_TOPIC"
    --odom-topic "$ODOM_TOPIC"
    --output-dir "$RESULT_DIR"
    --max-inferences "$MAX_INFERENCES"
    --max-motion-steps "$MAX_MOTION_STEPS"
    --max-spin-degrees "$MAX_SPIN_DEGREES"
    --max-forward-distance "$MAX_FORWARD_DISTANCE"
    --linear-speed "$LINEAR_SPEED"
    --angular-speed "$ANGULAR_SPEED"
    --max-base-linear "$MAX_BASE_LINEAR"
    --max-base-angular "$MAX_BASE_ANGULAR"
    --yaw-tolerance-degrees "$YAW_TOLERANCE_DEGREES"
    --spin-overshoot-degrees "$SPIN_OVERSHOOT_DEGREES"
    --spin-control-mode "$SPIN_CONTROL_MODE"
    --spin-replan-fps "$SPIN_REPLAN_FPS"
    --discrete-turn-degrees "$DISCRETE_TURN_DEGREES"
    --max-fallback-turn-degrees-per-segment "$MAX_FALLBACK_TURN_DEGREES_PER_SEGMENT"
    --max-fallback-turn-total-degrees "$MAX_FALLBACK_TURN_TOTAL_DEGREES"
    --turn-only-max-inferences "$TURN_ONLY_MAX_INFERENCES"
    --fallback-turn-timeout "$FALLBACK_TURN_TIMEOUT"
    --turn-direction-check-delay "$TURN_DIRECTION_CHECK_DELAY"
    --turn-direction-min-yaw-degrees "$TURN_DIRECTION_MIN_YAW_DEGREES"
    --motion-timeout "$MOTION_TIMEOUT"
    --base-motion-service-timeout "$BASE_MOTION_SERVICE_TIMEOUT"
    --preflight-wait "$PREFLIGHT_WAIT"
    --control-mode "$CONTROL_MODE"
    --command-angular-sign "$COMMAND_ANGULAR_SIGN"
    --camera-config "$CAMERA_CONFIG"
)
if [ "$SKIP_TURN_ONLY_MOTION" = "1" ]; then
    client_args+=(--skip-turn-only-motion)
fi
if [ -n "$CAMERA_POSE_JSON" ]; then
    client_args+=(--camera-pose-json "$CAMERA_POSE_JSON")
fi
if [ -n "$CAMERA_POSE_SOURCE" ]; then
    client_args+=(--camera-pose-source "$CAMERA_POSE_SOURCE")
fi
if [ "$CONTROL_MODE" = "mpc_tracking" ]; then
    client_args+=(
        --mpc-desired-v "$MPC_DESIRED_V"
        --mpc-v-max "$MPC_V_MAX"
        --mpc-w-max "$MPC_W_MAX"
        --mpc-goal-tolerance "$MPC_GOAL_TOLERANCE"
        --mpc-odom-max-age "$MPC_ODOM_MAX_AGE"
        --mpc-start-grace "$MPC_START_GRACE"
        --mpc-max-track-distance "$MPC_MAX_TRACK_DISTANCE"
        --mpc-horizon "$MPC_HORIZON"
        --mpc-ref-gap "$MPC_REF_GAP"
        --trajectory-tracker "$TRAJECTORY_TRACKER"
        --mpc-control-rate "$MPC_CONTROL_RATE"
        --mpc-max-solve-time "$MPC_MAX_SOLVE_TIME"
        --mpc-progress-timeout "$MPC_PROGRESS_TIMEOUT"
        --mpc-min-progress "$MPC_MIN_PROGRESS"
        --mpc-max-cross-track "$MPC_MAX_CROSS_TRACK"
        --pure-pursuit-lookahead "$PURE_PURSUIT_LOOKAHEAD"
        --pure-pursuit-align-degrees "$PURE_PURSUIT_ALIGN_DEGREES"
        --pure-pursuit-progress-timeout "$PURE_PURSUIT_PROGRESS_TIMEOUT"
    )
fi
if [ "$ALLOW_LEGACY_IDLE_STACK" = "1" ]; then
    client_args+=(--allow-legacy-idle-stack)
fi
if [ "$USE_BASE_MOTION_SERVICE" = "0" ]; then
    client_args+=(--no-use-base-motion-service)
fi
if [ "$FALLBACK_TURN_RESET_ON_TRAJECTORY" = "1" ]; then
    client_args+=(--fallback-turn-reset-on-trajectory)
else
    client_args+=(--no-fallback-turn-reset-on-trajectory)
fi
if [ "$ENABLE_MOTION" = "1" ]; then
    export INTERNNAV_MOTION_ARMED=YES
    client_args+=(--enable-motion --allow-dummy-depth-motion)
fi
if [ "$ALLOW_FORWARD_MOTION" = "1" ]; then
    export INTERNNAV_FORWARD_ARMED=YES
    client_args+=(--allow-forward-motion)
fi

python3 "$SCRIPT_DIR/internnav_direct_control_client.py" "${client_args[@]}" \
    2>&1 | tee "$RESULT_DIR/client.log"

stop_bag
trap - EXIT
python3 "$SCRIPT_DIR/analyze_closed_loop_bag.py" "$bag_dir" \
    --output "$RESULT_DIR/summary.json" | tee "$RESULT_DIR/summary.log"

echo "[OK] minimal closed-loop result: $RESULT_DIR"
