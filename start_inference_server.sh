#!/bin/bash
# 启动InternVLA推理服务器

cd /home/spirit-ai/Intern_g1

# 设置模型路径
MODEL_PATH="${MODEL_PATH:-/home/spirit-ai/model_vln/InternVLA-N1-w-NavDP}"

if [ ! -d "$MODEL_PATH" ]; then
    echo "错误: 模型路径不存在: $MODEL_PATH"
    echo "请设置正确的模型路径，例如："
    echo "  export MODEL_PATH=/home/spirit-ai/model_vln/InternVLA-N1-w-NavDP"
    exit 1
fi

echo "========================================="
echo "启动InternVLA推理服务器"
echo "========================================="
echo "模型路径: $MODEL_PATH"
echo "监听地址: 0.0.0.0:5801"
echo "========================================="

# 启动服务器
python3 scripts/realworld/http_internvla_server.py \
    --model_path "$MODEL_PATH" \
    --device cuda:0 \
    --host 0.0.0.0 \
    --port 5801 \
    --resize_w 384 \
    --resize_h 384 \
    --num_history 8 \
    --plan_step_gap 8
