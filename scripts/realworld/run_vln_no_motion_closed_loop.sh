#!/usr/bin/env bash
# InternNav 无运动在线 VLN 闭环测试脚本
# 严格保证机器人不实际运动，只验证传感器→推理→控制计算链路

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
NAV_WS_SETUP="${NAV_WS_SETUP:-/workspace/nav_ws/install/setup.bash}"

# 安全边界：强制无运动
export ENABLE_MOTION=0
export ALLOW_FORWARD_MOTION=0
export MAX_MOTION_STEPS=0
unset INTERNNAV_MOTION_ARMED
unset INTERNNAV_FORWARD_ARMED

# 服务器和话题配置
SERVER_URL="${SERVER_URL:-http://127.0.0.1:5801/eval_dual}"
HEALTH_URL="${HEALTH_URL:-${SERVER_URL%/eval_dual}/health}"
PRIMARY_RGB_TOPIC="${PRIMARY_RGB_TOPIC:-/moz_robot/camera/cam_high/image_raw}"
SECONDARY_RGB_TOPIC="${SECONDARY_RGB_TOPIC:-/moz_robot/camera/cam_high_extra/image_raw}"
ODOM_TOPIC="${ODOM_TOPIC:-/moz1/odom_global}"

# 测试参数
INSTRUCTION="${INSTRUCTION:-Move to the sofa and stop in front of it.}"
MAX_INFERENCES="${MAX_INFERENCES:-6}"
INFERENCE_FPS="${INFERENCE_FPS:-0.2}"
CONTROL_MODE="${CONTROL_MODE:-pulse}"
SKIP_RUNTIME_INSTALL="${SKIP_RUNTIME_INSTALL:-1}"
INTERNNAV_ALLOW_EAGER_ATTN="${INTERNNAV_ALLOW_EAGER_ATTN:-0}"
# Dry-run 非零速度预览：1=计算并发布非零预览到 /internnav/dry_run_cmd_vel（仍无实机运动）
DRY_RUN_PREVIEW_MOTION="${DRY_RUN_PREVIEW_MOTION:-0}"

# 输出目录（预览模式使用独立前缀，避免与纯零速 dry-run 混淆）
if [ "$DRY_RUN_PREVIEW_MOTION" = "1" ]; then
    RESULT_DIR="${RESULT_DIR:-$PROJECT_ROOT/experiment_records/vln_no_motion_preview_$(date +%Y%m%d_%H%M%S)}"
else
    RESULT_DIR="${RESULT_DIR:-$PROJECT_ROOT/experiment_records/vln_no_motion_$(date +%Y%m%d_%H%M%S)}"
fi
mkdir -p "$RESULT_DIR"
STARTUP_LOG="$RESULT_DIR/startup.log"
exec > >(tee -a "$STARTUP_LOG") 2>&1

log() {
    echo "[$(date +'%F %T')] $*"
}

log "=========================================="
log "InternNav 无运动在线 VLN 闭环测试"
log "=========================================="
log "结果目录: $RESULT_DIR"
log "指令: $INSTRUCTION"
log "安全边界: ENABLE_MOTION=0, ALLOW_FORWARD_MOTION=0, MAX_MOTION_STEPS=0"

# Source ROS
if [ ! -f "$NAV_WS_SETUP" ]; then
    echo "[ERROR] ROS setup not found: $NAV_WS_SETUP" >&2
    exit 2
fi
set +u
# shellcheck disable=SC1090
source "$NAV_WS_SETUP"
set -u

health_ok() {
    curl -fsS --max-time 2 "$HEALTH_URL" >/dev/null 2>&1
}

topic_hz_check() {
    local topic="$1"
    local min_hz="${2:-0.5}"
    local hz_output
    hz_output="$(timeout 7 ros2 topic hz "$topic" 2>&1 || true)"
    awk -v min="$min_hz" '
        /average rate:/ { seen=1; ok=($3 >= min) }
        END { exit (seen && ok) ? 0 : 1 }
    ' <<< "$hz_output"
}

# 第一步：话题预检
log "步骤 1/5: 话题预检"
TOPIC_REPORT="$RESULT_DIR/topic_report.json"
TOPIC_REPORT_MD="$RESULT_DIR/topic_report.md"

python3 -c "
import json
import subprocess
import sys
from datetime import datetime

topics = {
    'primary_rgb': '$PRIMARY_RGB_TOPIC',
    'secondary_rgb': '$SECONDARY_RGB_TOPIC',
    'odom': '$ODOM_TOPIC',
    'dry_run_cmd_vel': '/internnav/dry_run_cmd_vel',
    'cmd_vel': '/cmd_vel',
    'cmd_vel_nav': '/cmd_vel_nav',
    'cmd_vel_smoothed': '/cmd_vel_smoothed',
    'mx_base_vel_command': '/mx_base_vel_command',
}

report = {'timestamp': datetime.now().isoformat(), 'topics': {}}

for name, topic in topics.items():
    try:
        info = subprocess.run(['ros2', 'topic', 'info', '-v', topic],
                            capture_output=True, text=True, timeout=5)
        exists = info.returncode == 0
        pub_count = info.stdout.count('Publisher count:')
        report['topics'][name] = {
            'topic': topic,
            'exists': exists,
            'info': info.stdout if exists else 'Topic not found',
        }
    except Exception as e:
        report['topics'][name] = {'topic': topic, 'exists': False, 'error': str(e)}

with open('$TOPIC_REPORT', 'w') as f:
    json.dump(report, f, indent=2)

# 生成 Markdown 报告
lines = ['# 话题预检报告', '', f'时间: {report[\"timestamp\"]}', '']
for name, data in report['topics'].items():
    status = '✅' if data.get('exists') else '❌'
    lines.append(f'## {status} {name}')
    lines.append(f'- 话题: \`{data[\"topic\"]}\`')
    lines.append(f'- 存在: {data.get(\"exists\")}')
    lines.append('')

with open('$TOPIC_REPORT_MD', 'w') as f:
    f.write('\\n'.join(lines))

print('话题预检完成')
"

log "话题预检完成，报告: $TOPIC_REPORT_MD"

# 验证关键输入话题
log "验证输入话题频率..."
primary_hz_output="$(timeout 8 ros2 topic hz "$PRIMARY_RGB_TOPIC" 2>&1 || true)"
if ! grep -q "average rate" <<< "$primary_hz_output"; then
    log "[ERROR] 主相机话题无数据: $PRIMARY_RGB_TOPIC"
    log "$primary_hz_output"
    exit 2
fi
log "✅ 主相机: $PRIMARY_RGB_TOPIC"

odom_hz_output="$(timeout 5 ros2 topic hz "$ODOM_TOPIC" 2>&1 || true)"
if ! grep -q "average rate" <<< "$odom_hz_output"; then
    log "[ERROR] 里程计话题无数据: $ODOM_TOPIC"
    log "$odom_hz_output"
    exit 2
fi
log "✅ 里程计: $ODOM_TOPIC"

# 验证 dry_run_cmd_vel 无订阅者（客户端启动前）
log "验证 /internnav/dry_run_cmd_vel 无危险订阅者..."
if ros2 topic info -v /internnav/dry_run_cmd_vel 2>&1 | grep -q "Subscription count: [1-9]"; then
    log "[WARN] /internnav/dry_run_cmd_vel 已有订阅者，请确认无实机控制链路连接"
fi

# 第二步：推理服务检查和启动
log "步骤 2/5: 推理服务检查"
if health_ok; then
    log "✅ 推理服务已运行: $HEALTH_URL"
else
    log "推理服务未运行，正在启动..."
    cd "$PROJECT_ROOT"
    nohup bash -c "SKIP_RUNTIME_INSTALL=$SKIP_RUNTIME_INSTALL INTERNNAV_ALLOW_EAGER_ATTN=$INTERNNAV_ALLOW_EAGER_ATTN exec bash docker_start_server_in_container.sh" \
        > "$RESULT_DIR/inference_server.log" 2>&1 &
    SERVER_PID=$!
    log "推理服务启动 PID=$SERVER_PID，等待健康检查..."

    for i in {1..120}; do
        sleep 1
        if health_ok; then
            log "✅ 推理服务就绪"
            break
        fi
        if [ "$i" -eq 120 ]; then
            log "[ERROR] 推理服务120秒后仍未就绪，日志: $RESULT_DIR/inference_server.log"
            exit 2
        fi
    done
fi

# 第三步：实机控制话题安全采样（3秒）
log "步骤 3/5: 实机控制话题安全验证（3秒采样）"
SAFETY_CHECK_LOG="$RESULT_DIR/safety_preflight.log"
python3 -c "
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
import time
import sys

rclpy.init()
node = Node('safety_preflight')

samples = {
    '/cmd_vel': [],
    '/cmd_vel_nav': [],
    '/cmd_vel_smoothed': [],
    '/mx_base_vel_command': [],
}

def make_callback(topic):
    def cb(msg):
        if hasattr(msg, 'linear'):
            val = (msg.linear.x, msg.linear.y, msg.angular.z)
        else:
            val = (msg.x, msg.y, msg.z)
        samples[topic].append(val)
    return cb

for topic in samples:
    if topic == '/mx_base_vel_command':
        node.create_subscription(Vector3, topic, make_callback(topic), 10)
    else:
        node.create_subscription(Twist, topic, make_callback(topic), 10)

deadline = time.time() + 3.0
while time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)

rclpy.shutdown()

# 检查是否有非零速度
failed = False
for topic, vals in samples.items():
    nonzero = [v for v in vals if abs(v[0]) > 0.001 or abs(v[1]) > 0.001 or abs(v[2]) > 0.001]
    if nonzero:
        print(f'[ERROR] {topic} 有非零速度: {nonzero[:3]}')
        failed = True
    else:
        print(f'[OK] {topic}: {len(vals)} 条消息，全部为零或无消息')

sys.exit(1 if failed else 0)
" 2>&1 | tee "$SAFETY_CHECK_LOG"

if [ "${PIPESTATUS[0]}" -ne 0 ]; then
    log "[ERROR] 实机控制话题安全检查失败，详见: $SAFETY_CHECK_LOG"
    exit 2
fi
log "✅ 实机控制话题安全检查通过"

# 第四步：启动客户端（无运动模式）
log "步骤 4/5: 启动 InternNav 客户端（无运动 dry-run 模式）"

# 启动 rosbag 录制
log "启动 rosbag 录制..."
ROSBAG_DIR="$RESULT_DIR/rosbag"
ros2 bag record \
    -o "$ROSBAG_DIR" \
    --storage mcap \
    /internnav/dry_run_cmd_vel \
    /internnav/dry_run_stop_task \
    /internnav/dry_run_emergency_stop \
    /cmd_vel /cmd_vel_nav /cmd_vel_smoothed /mx_base_vel_command \
    /internnav/status \
    /moz1/odom_global \
    > "$RESULT_DIR/rosbag.log" 2>&1 &
ROSBAG_PID=$!
log "Rosbag 录制启动 PID=$ROSBAG_PID"
sleep 2

# 运行客户端
log "运行客户端: $MAX_INFERENCES 次推理, $INFERENCE_FPS Hz"
cd "$PROJECT_ROOT"

# 组装预览开关（仅在启用时传递，保持纯零速 dry-run 默认行为不变）
PREVIEW_ARGS=()
if [ "$DRY_RUN_PREVIEW_MOTION" = "1" ]; then
    PREVIEW_ARGS+=(--dry-run-preview-motion)
    log "Dry-run 非零速度预览已启用：非零速度只发布到 /internnav/dry_run_cmd_vel"
fi

set +e
python3 scripts/realworld/internnav_direct_control_client.py \
    --server-url "$SERVER_URL" \
    --instruction "$INSTRUCTION" \
    --primary-rgb-topic "$PRIMARY_RGB_TOPIC" \
    --secondary-rgb-topic "$SECONDARY_RGB_TOPIC" \
    --odom-topic "$ODOM_TOPIC" \
    --output-dir "$RESULT_DIR" \
    --max-inferences "$MAX_INFERENCES" \
    --inference-fps "$INFERENCE_FPS" \
    --control-mode "$CONTROL_MODE" \
    "${PREVIEW_ARGS[@]}" \
    > "$RESULT_DIR/client.log" 2>&1
CLIENT_RC=$?
set -e
log "客户端退出，退出码: $CLIENT_RC"

# 停止 rosbag
log "停止 rosbag 录制..."
kill -INT "$ROSBAG_PID" 2>/dev/null || true
sleep 2

# 第五步：后处理和安全验证
log "步骤 5/5: 后处理和安全验证"

# 分析 rosbag
log "分析 rosbag..."
if [ -d "$ROSBAG_DIR" ]; then
    ros2 bag reindex "$ROSBAG_DIR" 2>&1 | tee -a "$RESULT_DIR/rosbag.log" || true
    python3 "$SCRIPT_DIR/analyze_closed_loop_bag.py" "$ROSBAG_DIR" \
        --output "$RESULT_DIR/summary.json" \
        2>&1 | tee "$RESULT_DIR/summary.log" || log "[WARN] Bag 分析失败"
fi

# 验证实机控制话题全程无非零速度
log "验证实机控制话题安全性..."
python3 -c "
import json
from pathlib import Path

summary_path = Path('$RESULT_DIR/summary.json')
if not summary_path.exists():
    print('[WARN] summary.json 不存在，跳过验证')
    exit(0)

summary = json.loads(summary_path.read_text())
control_topics = ['/cmd_vel', '/cmd_vel_nav', '/cmd_vel_smoothed', '/mx_base_vel_command']
safe = True

for topic in control_topics:
    max_cmd = summary.get('max_twist_commands', {}).get(topic)
    if max_cmd:
        linear = max_cmd.get('linear', 0.0)
        angular = max_cmd.get('angular', 0.0)
        if abs(linear) > 0.001 or abs(angular) > 0.001:
            print(f'[ERROR] {topic} 检测到非零速度: linear={linear}, angular={angular}')
            safe = False
        else:
            print(f'[OK] {topic}: 全程零速度')
    else:
        count = summary.get('topic_counts', {}).get(topic, 0)
        print(f'[OK] {topic}: {count} 条消息（无非零样本）')

if summary.get('max_base_command'):
    base = summary['max_base_command']
    linear = base.get('linear', 0.0)
    angular = base.get('angular', 0.0)
    if abs(linear) > 0.001 or abs(angular) > 0.001:
        print(f'[ERROR] /mx_base_vel_command 检测到非零: linear={linear}, angular={angular}')
        safe = False

# 预览模式：验证 dry-run 话题确实出现了非零预览（验收条件）
preview_mode = '$DRY_RUN_PREVIEW_MOTION' == '1'
dry_run = summary.get('dry_run_cmd_vel', {})
dr_linear = dry_run.get('linear', 0.0)
dr_angular = dry_run.get('angular', 0.0)
dr_nonzero = dry_run.get('nonzero_samples', 0)
print(f'[INFO] /internnav/dry_run_cmd_vel: samples={dry_run.get(\"samples\", 0)}, '
      f'nonzero={dr_nonzero}, max_linear={dr_linear:.4f}, max_angular={dr_angular:.4f}')
if preview_mode:
    if dr_nonzero > 0:
        print(f'[OK] 预览验证通过：dry_run_cmd_vel 出现 {dr_nonzero} 个非零预览命令')
    else:
        print('[ERROR] 预览模式下 dry_run_cmd_vel 未出现任何非零预览命令')
        safe = False

if safe:
    print('[✅] 安全验证通过：所有实机控制话题全程零速度')
    if preview_mode:
        print('[✅] 预览验证通过：非零速度只出现在 /internnav/dry_run_cmd_vel')
else:
    print('[❌] 安全验证失败')
    exit(1)
" 2>&1 | tee "$RESULT_DIR/safety_verification.log"

SAFETY_RC="${PIPESTATUS[0]}"

# 生成测试报告
log "生成测试报告..."
cat > "$RESULT_DIR/README.md" <<EOF
# InternNav 无运动在线 VLN 闭环测试

## 测试信息
- 时间: $(date +'%F %T')
- 指令: \`$INSTRUCTION\`
- 最大推理次数: $MAX_INFERENCES
- 推理频率: $INFERENCE_FPS Hz
- 控制模式: $CONTROL_MODE
- 主相机: \`$PRIMARY_RGB_TOPIC\`
- 里程计: \`$ODOM_TOPIC\`

## 安全边界
- \`ENABLE_MOTION=0\`
- \`ALLOW_FORWARD_MOTION=0\`
- \`MAX_MOTION_STEPS=0\`
- 命令路由到: \`/internnav/dry_run_cmd_vel\`

## 测试结果
- 客户端退出码: $CLIENT_RC
- 安全验证: $([ "$SAFETY_RC" -eq 0 ] && echo "✅ 通过" || echo "❌ 失败")

## 文件清单
- \`client.log\` - 客户端日志
- \`events.jsonl\` - 推理事件序列
- \`metadata.json\` - 测试元数据
- \`summary.json\` - Rosbag 分析摘要
- \`rosbag/\` - 完整录制
- \`topic_report.json\` - 话题预检报告
- \`safety_verification.log\` - 安全验证日志

## 下一步
查看 \`events.jsonl\` 了解每次推理的详细输出，包括：
- 图像话题和时间戳
- 里程计位姿
- 模型原始输出
- 解析后的动作/轨迹
- 计算的速度命令（preview，未执行）
EOF

log "=========================================="
log "测试完成"
log "结果目录: $RESULT_DIR"
log "客户端退出码: $CLIENT_RC"
log "安全验证: $([ "$SAFETY_RC" -eq 0 ] && echo "通过" || echo "失败")"
log "=========================================="

exit "$CLIENT_RC"
