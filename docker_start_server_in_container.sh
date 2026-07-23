#!/usr/bin/env bash
# 在容器内启动推理服务的脚本
# 用法: 在容器内运行此脚本，或从宿主机运行: docker exec -it internnav_server bash /workspace/InternNav/docker_start_server_in_container.sh

set -euo pipefail

PROJECT_DIR=""
for candidate in /workspace/Intern_g1 /workspace/InternNav; do
    if [ -d "$candidate" ]; then
        PROJECT_DIR="$candidate"
        break
    fi
done

if [ -z "$PROJECT_DIR" ]; then
    echo "[ERROR] 未找到项目目录。请确认已挂载 /workspace/Intern_g1 或 /workspace/InternNav"
    exit 1
fi

cd "$PROJECT_DIR"
export LD_LIBRARY_PATH="/opt/hpcx/ucx/lib:/opt/hpcx/ucc/lib:${LD_LIBRARY_PATH:-}"
# flash_attn 的 CUDA 扩展 flash_attn_2_cuda 以顶层模块导入，需把 flash_attn 包目录加入 PYTHONPATH。
FLASH_ATTN_DIR="/usr/local/lib/python3.12/dist-packages/flash_attn"
export PYTHONPATH="$FLASH_ATTN_DIR:$PROJECT_DIR:${PYTHONPATH:-}"

echo "=========================================="
echo "  容器内推理服务启动脚本"
echo "=========================================="
echo "[INFO] 项目目录: $PROJECT_DIR"
echo "[INFO] LD_LIBRARY_PATH: $LD_LIBRARY_PATH"
echo "[INFO] PYTHONPATH: $PYTHONPATH"

# 检查模型文件
if [ ! -f "/workspace/model_vln/config.json" ]; then
    echo "[ERROR] 模型配置不存在: /workspace/model_vln/config.json，请检查模型挂载"
    exit 1
fi

echo "[INFO] 模型文件检查通过"

# 安装依赖（如果尚未安装）。一键实机复用场景可设置
# SKIP_RUNTIME_INSTALL=1 跳过重复安装，只保留 import 级验证。
if [ "${SKIP_RUNTIME_INSTALL:-0}" = "1" ]; then
    echo "[INFO] SKIP_RUNTIME_INSTALL=1，跳过 pip 安装步骤"
    if ! python3 -c "import quaternion" >/dev/null 2>&1; then
        echo "[ERROR] quaternion 不可导入；请先以 SKIP_RUNTIME_INSTALL=0 完整启动一次"
        exit 1
    fi
else
    echo "[INFO] 检查并安装依赖..."
    if ! python3 -c "import quaternion" >/dev/null 2>&1; then
        echo "[INFO] 安装 numpy-quaternion..."
        pip3 install numpy-quaternion --no-cache-dir
    fi

    echo "[INFO] 安装项目..."
    pip3 install -e . --no-deps --ignore-requires-python || echo "[WARN] 项目安装失败，尝试继续（如果已安装）..."
fi

# 验证 flash-attn 可用（默认强制启用，不回退 eager）。
# 例外：INTERNNAV_ALLOW_EAGER_ATTN=1 时允许 flash-attn 缺失并由 agent 回退 eager。
echo "[INFO] 验证 flash-attn..."
if ! python3 -c "import torch, flash_attn, flash_attn_2_cuda; print('[INFO] flash-attn', flash_attn.__version__, 'OK')"; then
    if [ "${INTERNNAV_ALLOW_EAGER_ATTN:-0}" = "1" ]; then
        echo "[WARN] flash-attn 不可用；INTERNNAV_ALLOW_EAGER_ATTN=1，将由 agent 回退 eager。"
    else
        echo "[ERROR] flash-attn 无法导入。请确认已编译安装 flash-attn 且 flash_attn_2_cuda 扩展存在。"
        echo "[ERROR] PYTHONPATH 需包含: $FLASH_ATTN_DIR"
        echo "[ERROR] 或设置 INTERNNAV_ALLOW_EAGER_ATTN=1 回退到 eager。"
        exit 1
    fi
fi

# 验证关键模块可导入
if ! python3 -c "import torch; from internnav.agent.internvla_n1_agent_realworld import InternVLAN1AsyncAgent" 2>/dev/null; then
    echo "[ERROR] 无法导入必需模块，请检查安装"
    exit 1
fi
echo "[INFO] 模块导入验证通过"

# 启动推理服务
echo ""
echo "[INFO] 启动InternVLA-N1推理服务..."
echo "[INFO] 模型路径: /workspace/model_vln"
echo "[INFO] 服务端口: 5801"
echo "[INFO] GPU设备: cuda:0"
echo ""

exec python3 scripts/realworld/http_internvla_server.py \
    --model_path /workspace/model_vln \
    --device cuda:0 \
    --host 0.0.0.0 \
    --resize_w 384 \
    --resize_h 384 \
    --num_history 8 \
    --plan_step_gap 8 \
    --port 5801 \
    --skip_warmup
