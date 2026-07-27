#!/bin/bash

################################################################################
# GMSL相机启动脚本
# 用途: 启动GMSL相机并验证
# 作者: spirit-ai
# 日期: 2026-07-14
################################################################################

echo "============================================================"
echo "  GMSL相机启动脚本"
echo "============================================================"

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# 配置变量
DEVICE_IDS="${DEVICE_IDS:-2}"
OUTPUT_WIDTH="${OUTPUT_WIDTH:-1408}"
OUTPUT_HEIGHT="${OUTPUT_HEIGHT:-1280}"

echo -e "${GREEN}[1/3]${NC} 加载ROS环境..."
echo "------------------------------------------------------------"

# 加载ROS环境
source /opt/ros/jazzy/setup.bash
source ~/pygmsl/ros2/install/setup.bash

echo -e "${GREEN}✓${NC} ROS Jazzy环境已加载"
echo "ROS_DISTRO: $ROS_DISTRO"

echo
echo -e "${GREEN}[2/3]${NC} 启动GMSL相机..."
echo "------------------------------------------------------------"

echo "配置:"
echo "  - Device IDs: $DEVICE_IDS"
echo "  - 输出分辨率: ${OUTPUT_WIDTH}x${OUTPUT_HEIGHT}"
echo

# 启动相机
ros2 launch gmsl gmsl_multi_camera_launch.py \
    device_ids:=$DEVICE_IDS \
    output_width:=$OUTPUT_WIDTH \
    output_height:=$OUTPUT_HEIGHT
