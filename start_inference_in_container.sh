#!/bin/bash
# 在nav_vln_dev容器内启动InternVLA推理服务器

echo "========================================="
echo "在容器内启动InternVLA推理服务器"
echo "========================================="

# 检查Intern_g1是否存在
if ! docker exec nav_vln_dev test -d /workspace/Intern_g1 2>/dev/null; then
    echo "错误: Intern_g1未挂载到容器"
    echo "正在将Intern_g1复制到容器..."
    docker cp ~/Intern_g1 nav_vln_dev:/workspace/
    echo "✓ 复制完成"
fi

# 检查模型路径
if ! docker exec nav_vln_dev test -d /workspace/model_vln 2>/dev/null; then
    echo "模型路径不存在，尝试使用/workspace/models"
    MODEL_PATH="/workspace/models"
else
    MODEL_PATH="/workspace/model_vln"
fi

echo "模型路径: $MODEL_PATH"
echo "监听地址: 0.0.0.0:5801"
echo "========================================="
echo ""
echo "启动服务器（后台运行）..."

# 在容器内后台启动推理服务器
docker exec -d nav_vln_dev bash -c "cd /workspace/Intern_g1 && \
    python3 scripts/realworld/http_internvla_server.py \
    --model_path $MODEL_PATH \
    --device cuda:0 \
    --host 0.0.0.0 \
    --port 5801 \
    --resize_w 384 \
    --resize_h 384 \
    --num_history 8 \
    --plan_step_gap 8 \
    > /tmp/inference_server.log 2>&1"

echo ""
echo "等待服务器启动..."
sleep 5

# 检查服务器是否启动
if docker exec nav_vln_dev bash -c "curl -s http://localhost:5801/ > /dev/null 2>&1"; then
    echo "✓ 推理服务器已启动"
    echo ""
    echo "查看日志: docker exec nav_vln_dev tail -f /tmp/inference_server.log"
    echo "测试推理: docker exec nav_vln_dev python3 /workspace/test_inference_env.py"
else
    echo "⚠ 服务器可能还在启动中，请稍候..."
    echo "查看日志: docker exec nav_vln_dev tail -f /tmp/inference_server.log"
fi
