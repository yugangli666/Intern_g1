#!/bin/bash

################################################################################
# InternNav 完整启动脚本
# 用途: 一键启动推理服务、相机和导航客户端
# 作者: spirit-ai
# 日期: 2026-07-14
################################################################################

set -e

echo "============================================================"
echo "  InternNav 完整启动脚本"
echo "============================================================"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 配置变量
PROJECT_DIR="/home/spirit-ai/Intern_g1"
CONTAINER_NAME="internnav_server"
SERVER_PORT=5801

echo -e "${GREEN}[步骤1/4]${NC} 启动Docker推理服务..."
echo "------------------------------------------------------------"

cd "$PROJECT_DIR"

# 检查容器是否已存在
if docker ps -a | grep -q "$CONTAINER_NAME"; then
    echo -e "${YELLOW}警告: 容器 $CONTAINER_NAME 已存在${NC}"
    read -p "是否删除并重新创建? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "删除旧容器..."
        docker rm -f "$CONTAINER_NAME"
    else
        echo "使用现有容器..."
        docker start "$CONTAINER_NAME" 2>/dev/null || true
    fi
else
    echo "启动新容器..."
    bash docker_run_server.sh
fi

echo -e "${GREEN}[步骤2/4]${NC} 等待推理服务启动..."
echo "------------------------------------------------------------"

echo "等待60秒供模型加载..."
for i in {60..1}; do
    echo -ne "倒计时: $i 秒\r"
    sleep 1
done
echo

echo -e "${GREEN}[步骤3/4]${NC} 验证推理服务..."
echo "------------------------------------------------------------"

# 检查容器状态
if docker ps | grep -q "$CONTAINER_NAME"; then
    echo -e "${GREEN}✓${NC} 容器运行正常"
else
    echo -e "${RED}✗${NC} 容器未运行"
    docker logs "$CONTAINER_NAME" | tail -20
    exit 1
fi

# 检查HTTP服务
if curl -s http://localhost:$SERVER_PORT/health > /dev/null 2>&1; then
    echo -e "${GREEN}✓${NC} 推理服务响应正常"
else
    echo -e "${YELLOW}!${NC} 推理服务可能还在启动中，请稍后手动测试"
fi

# 检查GPU
if nvidia-smi > /dev/null 2>&1; then
    echo -e "${GREEN}✓${NC} GPU可用"
else
    echo -e "${RED}✗${NC} GPU不可用"
fi

echo
echo -e "${GREEN}[步骤4/4]${NC} 启动指导"
echo "------------------------------------------------------------"

echo
echo "推理服务已启动！接下来需要在不同终端执行："
echo
echo -e "${YELLOW}终端2 - 启动GMSL相机:${NC}"
echo "source /opt/ros/jazzy/setup.bash"
echo "source ~/pygmsl/ros2/install/setup.bash"
echo "ros2 launch gmsl gmsl_multi_camera_launch.py device_ids:=2 output_width:=1408 output_height:=1280"
echo
echo -e "${YELLOW}终端3 - 启动导航客户端:${NC}"
echo "cd $PROJECT_DIR/scripts/realworld"
echo "bash run_simple_client.sh"
echo
echo -e "${YELLOW}或使用自定义指令:${NC}"
echo "INSTRUCTION=\"Walk forward to the door\" bash run_simple_client.sh"
echo
echo "============================================================"
echo -e "${GREEN}推理服务启动完成！${NC}"
echo "============================================================"
echo
echo "查看日志: docker logs -f $CONTAINER_NAME"
echo "查看GPU: nvidia-smi"
echo "健康检查: curl http://localhost:$SERVER_PORT/health"
echo
