#!/usr/bin/env bash
# InternNav Docker 快速启动脚本 - 简化版

set -euo pipefail

DOCKER_IMAGE="harbor.i.spirit-ai.com:443/slam_nav/nav_release:jazzy-thor-vln-deps-fixed-20260714"
CONTAINER_NAME="internnav_server"

echo "=========================================="
echo "  InternNav Docker 快速启动"
echo "=========================================="

# 停止旧容器
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "[INFO] 停止旧容器..."
    docker stop "$CONTAINER_NAME" 2>/dev/null || true
    docker rm "$CONTAINER_NAME" 2>/dev/null || true
fi

# 启动容器
echo "[INFO] 启动Docker容器..."
docker run -d \
    --name "$CONTAINER_NAME" \
    --gpus all \
    --net=host \
    --privileged \
    -v /home/spirit-ai/Intern_g1:/workspace/InternNav:rw \
    -v /home/spirit-ai/model_vln:/workspace/model_vln:ro \
    -e CUDA_VISIBLE_DEVICES=0 \
    -e PYTHONUNBUFFERED=1 \
    -e ROS_DOMAIN_ID=33 \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -w /workspace/InternNav \
    "$DOCKER_IMAGE" \
    bash -lc "while true; do sleep 3600; done"

echo "[INFO] 容器已启动: $CONTAINER_NAME"
echo ""
echo "下一步操作："
echo "1. 进入容器: docker exec -it $CONTAINER_NAME bash"
echo "2. 安装依赖: cd /workspace/InternNav && pip3 install -r requirements/internvla_n1.txt"
echo "3. 启动服务: python3 scripts/realworld/http_internvla_server.py --device cuda:0 --model_path /workspace/model_vln --port 5801"
echo ""
echo "或者运行一键启动脚本："
echo "docker exec -it $CONTAINER_NAME bash /workspace/InternNav/docker_start_server_in_container.sh"
