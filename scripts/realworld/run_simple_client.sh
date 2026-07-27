#!/bin/bash
# 简化的InternNav客户端启动脚本（本机测试 - 仅RGB模式）
# 适配ROS Jazzy环境

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "============================================================"
echo "  简化InternNav客户端启动 (仅RGB模式)"
echo "============================================================"
echo "项目目录: $PROJECT_ROOT"
echo ""

# 设置Python路径
export PYTHONPATH="$PROJECT_ROOT:$PYTHONPATH"

# 默认参数
SERVER_URL="${SERVER_URL:-http://localhost:5801/eval_dual}"
RGB_TOPIC="${RGB_TOPIC:-/moz_robot/camera/cam_high_extra/image_raw}"
FPS="${FPS:-5.0}"
INSTRUCTION="${INSTRUCTION:-Walk forward to the door}"
MAX_INFERENCES="${MAX_INFERENCES:-0}"
OUTPUT_DIR="${OUTPUT_DIR:-}"
REQUEST_TIMEOUT="${REQUEST_TIMEOUT:-120}"

derive_base_url() {
    python3 - "$SERVER_URL" <<'PY'
import sys
from urllib.parse import urlparse

parsed = urlparse(sys.argv[1])
if not parsed.scheme or not parsed.netloc:
    sys.exit(1)

print(f"{parsed.scheme}://{parsed.netloc}")
PY
}

if ! SERVER_BASE_URL="$(derive_base_url 2>/dev/null)"; then
    SERVER_BASE_URL="http://localhost:5801"
fi

echo "配置:"
echo "  推理服务: $SERVER_URL"
echo "  健康检查: $SERVER_BASE_URL/health"
echo "  RGB话题: $RGB_TOPIC"
echo "  FPS: $FPS"
echo "  导航指令: $INSTRUCTION"
echo "  最大推理次数: $MAX_INFERENCES (0=无限)"
echo "  请求超时: ${REQUEST_TIMEOUT}s"
if [ -n "$OUTPUT_DIR" ]; then
    echo "  结果目录: $OUTPUT_DIR"
fi
echo ""

# 检查推理服务
echo "检查推理服务..."
if curl -fsS --max-time 2 "$SERVER_BASE_URL/health" > /dev/null 2>&1; then
    echo "✓ 推理服务可达"
else
    echo "✗ 警告: 推理服务似乎未运行"
    echo "  请检查: docker ps | grep internnav_server"
fi

# 检查相机话题
echo "检查相机话题..."
if ros2 topic list 2>/dev/null | grep -q "$RGB_TOPIC"; then
    echo "✓ RGB话题存在: $RGB_TOPIC"
else
    echo "✗ 警告: RGB话题不存在"
    echo "  可用的图像话题:"
    ros2 topic list 2>/dev/null | grep image | head -5
fi

echo ""
echo "============================================================"
echo "  启动客户端 (按 Ctrl+C 停止)"
echo "============================================================"
echo ""

# 启动客户端（仅RGB模式，不需要深度话题）
cd "$PROJECT_ROOT"
EXTRA_ARGS=(--max_inferences "$MAX_INFERENCES")
EXTRA_ARGS+=(--request_timeout "$REQUEST_TIMEOUT")
if [ -n "$OUTPUT_DIR" ]; then
    EXTRA_ARGS+=(--output_dir "$OUTPUT_DIR")
fi
exec python3 "$SCRIPT_DIR/simple_client.py" \
    --server_url "$SERVER_URL" \
    --rgb_topic "$RGB_TOPIC" \
    --fps "$FPS" \
    --instruction "$INSTRUCTION" \
    "${EXTRA_ARGS[@]}" \
    "$@"
