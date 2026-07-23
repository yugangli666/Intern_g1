#!/usr/bin/env bash
# 快速启动无运动测试的便捷脚本

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 默认配置
export INSTRUCTION="${INSTRUCTION:-Move to the sofa and stop in front of it.}"
export MAX_INFERENCES="${MAX_INFERENCES:-6}"
export INFERENCE_FPS="${INFERENCE_FPS:-0.2}"
export CONTROL_MODE="${CONTROL_MODE:-pulse}"

# 如果传入参数，覆盖指令
if [ $# -gt 0 ]; then
    export INSTRUCTION="$*"
fi

echo "========================================"
echo "InternNav 无运动测试快速启动"
echo "========================================"
echo "指令: $INSTRUCTION"
echo "推理次数: $MAX_INFERENCES"
echo "推理频率: $INFERENCE_FPS Hz"
echo "========================================"
echo ""

exec bash "$SCRIPT_DIR/run_vln_no_motion_closed_loop.sh"
