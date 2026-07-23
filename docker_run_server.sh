#!/usr/bin/env bash
# InternNav Docker 推理服务启动脚本
# 用途：在Docker容器中启动InternVLA-N1模型推理服务

set -euo pipefail

# ============= 配置参数 =============
DOCKER_IMAGE="harbor.i.spirit-ai.com:443/slam_nav/nav_release:jazzy-thor-vln-deps-fixed-20260714"
CONTAINER_NAME="internnav_server"
HOST_PROJECT_DIR="/home/spirit-ai/Intern_g1"
HOST_MODEL_DIR="/home/spirit-ai/model_vln"
CONTAINER_WORKSPACE="/workspace/InternNav"
SERVER_PORT=5801

# GPU设置
CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}"

# 模型选择: NavDP 或 DualVLN
MODEL_VARIANT="${MODEL_VARIANT:-NavDP}"

# ============= 函数定义 =============
check_prerequisites() {
    echo "[INFO] 检查前置条件..."

    # 检查Docker
    if ! command -v docker &> /dev/null; then
        echo "[ERROR] Docker未安装，请先安装Docker"
        exit 1
    fi

    # 检查项目目录
    if [ ! -d "$HOST_PROJECT_DIR" ]; then
        echo "[ERROR] 项目目录不存在: $HOST_PROJECT_DIR"
        exit 1
    fi

    # 检查模型目录
    if [ ! -d "$HOST_MODEL_DIR" ]; then
        echo "[ERROR] 模型目录不存在: $HOST_MODEL_DIR"
        exit 1
    fi

    # 检查模型文件
    if [ ! -f "$HOST_MODEL_DIR/config.json" ]; then
        echo "[ERROR] 模型配置文件不存在: $HOST_MODEL_DIR/config.json"
        exit 1
    fi

    echo "[INFO] 前置条件检查通过"
}

stop_existing_container() {
    if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        echo "[INFO] 停止并删除现有容器: $CONTAINER_NAME"
        docker stop "$CONTAINER_NAME" 2>/dev/null || true
        docker rm "$CONTAINER_NAME" 2>/dev/null || true
    fi
}

start_container() {
    echo "[INFO] 启动Docker容器..."
    echo "[INFO] 使用GPU: $CUDA_VISIBLE_DEVICES"
    echo "[INFO] 模型变体: $MODEL_VARIANT"
    echo "[INFO] 服务端口: $SERVER_PORT"

    docker run -d \
        --name "$CONTAINER_NAME" \
        --gpus "device=$CUDA_VISIBLE_DEVICES" \
        --net=host \
        --privileged \
        -v "$HOST_PROJECT_DIR:$CONTAINER_WORKSPACE:rw" \
        -v "$HOST_MODEL_DIR:/workspace/model_vln:ro" \
        -e CUDA_VISIBLE_DEVICES="$CUDA_VISIBLE_DEVICES" \
        -e MODEL_VARIANT="$MODEL_VARIANT" \
        -e SERVER_PORT="$SERVER_PORT" \
        -e PYTHONUNBUFFERED=1 \
        -e DISPLAY="${DISPLAY:-:0}" \
        -w "$CONTAINER_WORKSPACE" \
        "$DOCKER_IMAGE" \
        bash -c "tail -f /dev/null"

    echo "[INFO] 容器启动成功: $CONTAINER_NAME"

    # 等待容器完全启动
    sleep 2
}

install_dependencies() {
    echo "[INFO] 安装项目依赖..."

    docker exec -it "$CONTAINER_NAME" bash -c "
        set -e
        cd $CONTAINER_WORKSPACE

        # 安装核心依赖
        if [ -f requirements/core_requirements.txt ]; then
            echo '[INFO] 安装核心依赖...'
            pip3 install -r requirements/core_requirements.txt --no-cache-dir || true
        fi

        # 安装InternVLA-N1依赖
        if [ -f requirements/internvla_n1.txt ]; then
            echo '[INFO] 安装InternVLA-N1依赖...'
            pip3 install -r requirements/internvla_n1.txt --no-cache-dir || true
        fi

        # 安装项目本身
        echo '[INFO] 安装InternNav项目...'
        pip3 install -e . --no-deps || true

        echo '[INFO] 依赖安装完成'
    "
}

start_inference_server() {
    echo "[INFO] 启动推理服务..."

    # 确定模型路径
    if [ "$MODEL_VARIANT" = "DualVLN" ]; then
        MODEL_PATH="/workspace/model_vln"
    else
        MODEL_PATH="/workspace/model_vln"
    fi

    docker exec -d "$CONTAINER_NAME" bash -c "
        set -e
        cd $CONTAINER_WORKSPACE

        echo '[INFO] 启动InternVLA-N1推理服务...'
        echo '[INFO] 模型路径: $MODEL_PATH'
        echo '[INFO] 服务端口: $SERVER_PORT'

        # 启动HTTP推理服务
        python3 scripts/realworld/http_internvla_server.py \
            --device cuda:0 \
            --model_path $MODEL_PATH \
            --resize_w 384 \
            --resize_h 384 \
            --num_history 8 \
            --plan_step_gap 8 \
            --port $SERVER_PORT \
            --skip_warmup \
            2>&1 | tee /workspace/InternNav/server.log
    "

    echo "[INFO] 推理服务已在后台启动"
    echo "[INFO] 查看日志: docker exec -it $CONTAINER_NAME tail -f $CONTAINER_WORKSPACE/server.log"
}

show_status() {
    echo ""
    echo "============================================"
    echo "  InternNav 推理服务启动完成"
    echo "============================================"
    echo ""
    echo "容器名称: $CONTAINER_NAME"
    echo "服务地址: http://localhost:$SERVER_PORT/eval_dual"
    echo "模型路径: $MODEL_PATH"
    echo "GPU设备:  cuda:$CUDA_VISIBLE_DEVICES"
    echo ""
    echo "常用命令:"
    echo "  查看容器状态: docker ps | grep $CONTAINER_NAME"
    echo "  查看服务日志: docker exec -it $CONTAINER_NAME tail -f $CONTAINER_WORKSPACE/server.log"
    echo "  进入容器:     docker exec -it $CONTAINER_NAME bash"
    echo "  停止服务:     docker stop $CONTAINER_NAME"
    echo "  删除容器:     docker rm $CONTAINER_NAME"
    echo ""
    echo "测试服务:"
    echo "  curl http://localhost:$SERVER_PORT/health"
    echo ""
    echo "============================================"
}

# ============= 主流程 =============
main() {
    echo "========================================"
    echo "  InternNav Docker 推理服务启动脚本"
    echo "========================================"
    echo ""

    check_prerequisites
    stop_existing_container
    start_container

    echo ""
    read -p "是否安装Python依赖? (y/n, 首次运行选y): " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        install_dependencies
    fi

    echo ""
    read -p "是否立即启动推理服务? (y/n): " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        start_inference_server
        sleep 3
    fi

    show_status
}

# 执行主流程
main "$@"
