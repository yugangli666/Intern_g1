# InternNav 本机完整部署和测试指南

## 📋 系统说明

**本机角色**: 推理服务端 + 导航客户端（不是G1机器人）
- **推理服务**: 在Docker容器中运行
- **导航客户端**: 在本机ROS Foxy环境中运行
- **硬件**: D455深度相机连接到本机

---

## ✅ 当前部署状态

### 推理服务端（已完成 ✅）

- **容器名称**: `internnav_server`
- **服务地址**: `http://localhost:5801/eval_dual`
- **状态**: 运行中
- **模型**: InternVLA-N1 (16.7GB)

**验证服务运行**:
```bash
docker ps | grep internnav_server
docker exec internnav_server tail -20 /tmp/server.log
curl http://localhost:5801/
```

---

## 🚀 启动导航客户端（本机测试）

### 前置条件检查

1. **检查ROS Foxy环境**:
```bash
source /home/spirit-ai/Intern_g1/g1_client/ros_foxy_env.sh
source_g1_ros_foxy
ros2 --version  # 应该显示 Foxy 版本
```

2. **检查D455相机连接**:
```bash
# 检查USB设备
lsusb | grep Intel

# 或检查realsense设备
rs-enumerate-devices
```

3. **检查realsense2_camera包**:
```bash
ros2 pkg prefix realsense2_camera
```

### 步骤1: 启动D455相机

在**终端1**中运行：

```bash
cd /home/spirit-ai/Intern_g1/g1_client

# 使用默认设置（低带宽模式，适合USB 2.x）
bash run_d455_camera.sh

# 或使用高分辨率（如果是USB 3.x连接）
REALSENSE_COLOR_PROFILE=640,480,30 \
REALSENSE_DEPTH_PROFILE=640,480,30 \
bash run_d455_camera.sh
```

**预期输出**: ROS节点启动，发布RGB-D图像话题

**验证相机**（在另一个终端）:
```bash
source /home/spirit-ai/Intern_g1/g1_client/ros_foxy_env.sh
source_g1_ros_foxy
ros2 topic list | grep camera
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/aligned_depth_to_color/image_raw
```

### 步骤2: 启动导航客户端

在**终端2**中运行：

```bash
cd /home/spirit-ai/Intern_g1/g1_client

# 基本测试（dry-run模式，不控制机器人运动）
bash run_g1_client.sh \
    --server_url http://localhost:5801/eval_dual \
    --instruction "Walk forward to the door" \
    --dry-run

# 完整运行（需要连接机器人控制）
bash run_g1_client.sh \
    --server_url http://localhost:5801/eval_dual \
    --instruction "Walk forward to the door"
```

### 客户端参数说明

查看完整参数：
```bash
cd /home/spirit-ai/Intern_g1/g1_client
python3 http_internvla_client_g1.py --help
```

常用参数：
- `--server_url`: 推理服务地址（默认: http://localhost:5801/eval_dual）
- `--instruction`: 导航指令
- `--dry-run`: 仅测试推理，不执行运动控制
- `--target-fps`: 目标帧率（默认: 5）
- `--enable-motion`: 启用机器人运动控制

---

## 🧪 完整测试流程

### 测试1: 推理服务健康检查

```bash
# 查看容器状态
docker ps | grep internnav_server

# 查看服务日志
docker exec internnav_server tail -50 /tmp/server.log

# 查看GPU使用
nvidia-smi

# 测试HTTP响应
curl http://localhost:5801/
```

### 测试2: D455相机数据流

```bash
# 终端1: 启动相机
cd /home/spirit-ai/Intern_g1/g1_client
bash run_d455_camera.sh

# 终端2: 验证话题
source /home/spirit-ai/Intern_g1/g1_client/ros_foxy_env.sh
source_g1_ros_foxy
ros2 topic list
ros2 topic echo /camera/color/image_raw --once
```

### 测试3: 端到端导航（dry-run）

```bash
# 终端1: 推理服务（已运行）
docker ps | grep internnav_server

# 终端2: D455相机
cd /home/spirit-ai/Intern_g1/g1_client
bash run_d455_camera.sh

# 终端3: 导航客户端（dry-run模式）
cd /home/spirit-ai/Intern_g1/g1_client
bash run_g1_client.sh \
    --server_url http://localhost:5801/eval_dual \
    --instruction "From the center, turn left and walk toward the black TV." \
    --dry-run
```

**预期行为**:
- 相机采集RGB-D图像
- 客户端发送图像到推理服务
- 推理服务返回导航动作或轨迹
- 终端显示推理结果（但不执行运动）

---

## 📊 系统架构

```
本机 (192.168.12.223)
├── Docker容器 (internnav_server)
│   └── InternVLA-N1推理服务 :5801
│       └── 接收RGB-D图像 → 输出导航指令
│
├── ROS Foxy环境
│   ├── D455相机节点
│   │   └── 发布 /camera/color/image_raw
│   │   └── 发布 /camera/aligned_depth_to_color/image_raw
│   │
│   └── 导航客户端
│       └── 订阅相机话题 → HTTP POST到推理服务 → 输出/执行动作
│
└── 硬件
    └── Intel RealSense D455 (USB连接)
```

---

## 🔧 常见问题排查

### 问题1: 推理服务无响应
```bash
# 检查容器运行状态
docker ps | grep internnav_server

# 重启容器
docker restart internnav_server

# 查看详细日志
docker exec internnav_server tail -100 /tmp/server.log
```

### 问题2: 找不到D455相机
```bash
# 检查USB连接
lsusb | grep Intel

# 检查realsense设备
rs-enumerate-devices

# 重新插拔USB线缆
# 或使用pyrealsense后端
D455_BACKEND=pyrealsense bash run_d455_camera.sh
```

### 问题3: ROS Foxy环境问题
```bash
# 重新source环境
cd /home/spirit-ai/Intern_g1/g1_client
source ros_foxy_env.sh
source_g1_ros_foxy

# 检查ROS版本
ros2 --version

# 检查话题
ros2 topic list
```

### 问题4: 客户端无法连接推理服务
```bash
# 检查网络连接
curl http://localhost:5801/

# 检查端口占用
sudo lsof -i :5801

# 修改服务URL
bash run_g1_client.sh --server_url http://127.0.0.1:5801/eval_dual --dry-run
```

---

## 📝 快速命令参考

### 推理服务管理
```bash
# 启动容器（如果未运行）
docker start internnav_server

# 查看日志
docker exec internnav_server tail -f /tmp/server.log

# 重启服务
docker restart internnav_server

# 停止服务
docker stop internnav_server
```

### 导航客户端管理
```bash
# 进入客户端目录
cd /home/spirit-ai/Intern_g1/g1_client

# 启动相机（终端1）
bash run_d455_camera.sh

# 启动客户端（终端2）
bash run_g1_client.sh \
    --server_url http://localhost:5801/eval_dual \
    --instruction "导航指令" \
    --dry-run

# 查看ROS话题
source ros_foxy_env.sh && source_g1_ros_foxy
ros2 topic list
```

---

## 🎯 下一步

1. **测试推理延迟**: 运行dry-run测试，观察推理时间
2. **调整参数**: 根据实际场景调整相机分辨率、FPS等
3. **机器人集成**: 如果要控制机器人，需要配置运动控制接口
4. **数据采集**: 可以使用数据采集工具记录导航数据

---

**创建时间**: 2026-07-14  
**系统**: Ubuntu + ROS Foxy + Docker  
**硬件**: NVIDIA Thor + Intel RealSense D455
