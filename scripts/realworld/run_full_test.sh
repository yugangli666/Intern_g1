#!/bin/bash
# InternNav 完整测试启动脚本（包含GMSL相机）

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "============================================================"
echo "  InternNav 完整测试 - GMSL相机 + 推理客户端"
echo "============================================================"
echo ""

# 配置参数
CAMERA_TOPIC="${CAMERA_TOPIC:-/camera/cam_high/image_raw}"
INSTRUCTION="${INSTRUCTION:-Walk forward to the door}"
FPS="${FPS:-5.0}"
SERVER_URL="${SERVER_URL:-http://localhost:5801/eval_dual}"

# GMSL相机参数
DEVICE_IDS="${DEVICE_IDS:-4,5}"
OUTPUT_WIDTH="${OUTPUT_WIDTH:-1408}"
OUTPUT_HEIGHT="${OUTPUT_HEIGHT:-1280}"

echo "配置参数:"
echo "  相机设备ID: $DEVICE_IDS"
echo "  相机分辨率: ${OUTPUT_WIDTH}x${OUTPUT_HEIGHT}"
echo "  RGB话题: $CAMERA_TOPIC"
echo "  推理服务: $SERVER_URL"
echo "  导航指令: $INSTRUCTION"
echo "  推理FPS: $FPS"
echo ""

# 检查推理服务
echo "[1/4] 检查推理服务..."
if curl -s --max-time 2 http://localhost:5801/ > /dev/null 2>&1; then
    echo "  ✓ 推理服务运行中"
else
    echo "  ✗ 推理服务未运行"
    echo "  请启动: docker start internnav_server"
    exit 1
fi

# 检查ROS环境
echo ""
echo "[2/4] 检查ROS环境..."
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo "  ✓ ROS Jazzy环境加载"
else
    echo "  ✗ ROS Jazzy未安装"
    exit 1
fi

if [ -f "$HOME/pygmsl/ros2/install/setup.bash" ]; then
    source "$HOME/pygmsl/ros2/install/setup.bash"
    echo "  ✓ GMSL ROS包加载"
else
    echo "  ✗ GMSL ROS包未找到: $HOME/pygmsl/ros2/install/setup.bash"
    exit 1
fi

# 启动GMSL相机
echo ""
echo "[3/4] 启动GMSL相机..."
echo "  设备ID: $DEVICE_IDS"
echo "  分辨率: ${OUTPUT_WIDTH}x${OUTPUT_HEIGHT}"
echo ""

# 在后台启动相机
ros2 launch gmsl gmsl_multi_camera_launch.py \
    device_ids:=$DEVICE_IDS \
    output_width:=$OUTPUT_WIDTH \
    output_height:=$OUTPUT_HEIGHT \
    > /tmp/gmsl_camera.log 2>&1 &

CAMERA_PID=$!
echo "  相机节点PID: $CAMERA_PID"
echo "  等待相机初始化..."
sleep 5

# 检查相机是否成功启动
if ! kill -0 $CAMERA_PID 2>/dev/null; then
    echo "  ✗ 相机启动失败"
    echo "  查看日志: tail -50 /tmp/gmsl_camera.log"
    exit 1
fi

echo "  ✓ 相机启动成功"

# 检查相机话题
echo ""
echo "  检查相机话题..."
sleep 3

AVAILABLE_TOPICS=$(ros2 topic list | grep -E 'camera.*image' || true)
if [ -z "$AVAILABLE_TOPICS" ]; then
    echo "  ✗ 未找到相机图像话题"
    echo "  可用话题:"
    ros2 topic list | grep camera || echo "    无相机话题"
    echo ""
    echo "  停止相机节点..."
    kill $CAMERA_PID 2>/dev/null || true
    exit 1
fi

echo "  可用的相机话题:"
echo "$AVAILABLE_TOPICS" | sed 's/^/    /'

# 如果指定的话题不存在，使用第一个找到的
if ! echo "$AVAILABLE_TOPICS" | grep -q "$CAMERA_TOPIC"; then
    CAMERA_TOPIC=$(echo "$AVAILABLE_TOPICS" | head -1)
    echo ""
    echo "  使用话题: $CAMERA_TOPIC"
fi

# 清理函数
cleanup() {
    echo ""
    echo "============================================================"
    echo "  清理并退出..."
    echo "============================================================"
    if [ ! -z "$CAMERA_PID" ]; then
        echo "  停止相机节点 (PID: $CAMERA_PID)..."
        kill $CAMERA_PID 2>/dev/null || true
        sleep 2
        kill -9 $CAMERA_PID 2>/dev/null || true
    fi
    echo "  完成"
    exit 0
}

# 捕获退出信号
trap cleanup SIGINT SIGTERM EXIT

# 启动InternNav客户端
echo ""
echo "[4/4] 启动InternNav推理客户端..."
echo "============================================================"
echo "  按 Ctrl+C 停止测试"
echo "============================================================"
echo ""

export PYTHONPATH="$PROJECT_ROOT:$PYTHONPATH"

cd "$PROJECT_ROOT"
python3 "$SCRIPT_DIR/simple_client.py" \
    --server_url "$SERVER_URL" \
    --rgb_topic "$CAMERA_TOPIC" \
    --fps "$FPS" \
    --instruction "$INSTRUCTION" \
    "$@"

# 脚本结束时会自动调用cleanup
