#!/bin/bash
# 快速测试InternNav推理服务

echo "=========================================="
echo "  InternNav 快速测试"
echo "=========================================="
echo ""

# 1. 检查推理服务
echo "[1/4] 检查推理服务..."
if docker ps | grep -q internnav_server; then
    echo "  ✓ Docker容器运行中"
else
    echo "  ✗ Docker容器未运行"
    echo "  启动容器: docker start internnav_server"
    exit 1
fi

if curl -s --max-time 2 http://localhost:5801/ > /dev/null 2>&1; then
    echo "  ✓ HTTP服务响应正常"
else
    echo "  ✗ HTTP服务无响应"
    echo "  查看日志: docker exec internnav_server tail -20 /tmp/server.log"
    exit 1
fi

# 2. 检查GPU
echo ""
echo "[2/4] 检查GPU使用..."
if nvidia-smi > /dev/null 2>&1; then
    echo "  ✓ GPU可用"
else
    echo "  ✗ GPU不可用"
fi

# 3. 检查相机话题
echo ""
echo "[3/4] 检查相机话题..."
RGB_TOPIC="/camera/camera/color/image_raw"
if ros2 topic list 2>/dev/null | grep -q "$RGB_TOPIC"; then
    echo "  ✓ RGB话题存在: $RGB_TOPIC"

    # 检查话题频率
    echo "  检查话题频率（5秒）..."
    timeout 5 ros2 topic hz $RGB_TOPIC 2>/dev/null | grep "average rate" || echo "  无法检测频率"
else
    echo "  ✗ RGB话题不存在"
    echo "  可用话题:"
    ros2 topic list 2>/dev/null | grep image | head -5
    exit 1
fi

# 4. 系统状态
echo ""
echo "[4/4] 系统状态总结..."
echo "  - 推理服务: ✓ 运行中 (http://localhost:5801)"
echo "  - RGB相机: ✓ 可用 ($RGB_TOPIC)"
echo "  - ROS版本: $(ros2 --version 2>&1 | head -1 || echo 'Unknown')"
echo ""
echo "=========================================="
echo "  ✓ 系统就绪！可以启动客户端测试"
echo "=========================================="
echo ""
echo "快速启动命令:"
echo ""
echo "  cd /home/spirit-ai/Intern_g1/scripts/realworld"
echo "  INSTRUCTION=\"Walk forward\" bash run_simple_client.sh"
echo ""
