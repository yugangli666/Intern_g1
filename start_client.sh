#!/bin/bash

################################################################################
# 导航客户端启动脚本
# 用途: 启动InternNav导航客户端
# 作者: spirit-ai
# 日期: 2026-07-14
################################################################################

echo "============================================================"
echo "  InternNav 导航客户端启动"
echo "============================================================"

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# 配置变量
INSTRUCTION="${INSTRUCTION:-Walk forward to the door}"
SERVER_URL="${SERVER_URL:-http://localhost:5801/eval_dual}"
RGB_TOPIC="${RGB_TOPIC:-/camera/camera/color/image_raw}"
FPS="${FPS:-5.0}"

PROJECT_DIR="/home/spirit-ai/Intern_g1"

echo -e "${GREEN}[1/2]${NC} 配置信息"
echo "------------------------------------------------------------"
echo "  导航指令: $INSTRUCTION"
echo "  推理服务: $SERVER_URL"
echo "  RGB话题: $RGB_TOPIC"
echo "  推理频率: $FPS FPS"
echo

echo -e "${GREEN}[2/2]${NC} 启动客户端..."
echo "------------------------------------------------------------"

cd "$PROJECT_DIR/scripts/realworld"

# 启动客户端
INSTRUCTION="$INSTRUCTION" \
SERVER_URL="$SERVER_URL" \
RGB_TOPIC="$RGB_TOPIC" \
FPS="$FPS" \
bash run_simple_client.sh
