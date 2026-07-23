# InternNav 完整启动指南

**文档版本**: 1.0  
**更新时间**: 2026-07-14  
**适用环境**: 本机一体化部署 (ROS2 Jazzy + GMSL相机 + Docker)

---

## 📋 系统架构

### 当前配置
- **部署模式**: 本机一体化（推理 + 客户端）
- **ROS版本**: ROS2 Jazzy
- **相机**: GMSL相机（仅RGB，不需要深度）
- **相机话题**: `/camera/cam_high/image_raw` 或 `/camera/camera/color/image_raw`
- **推理服务**: Docker容器 @ `http://localhost:5801/eval_dual`
- **GPU**: CUDA 13.0
- **模型**: InternVLA-N1 (16.7GB)

---

## 🚀 快速启动（三步）

### 步骤1: 启动推理服务

```bash
cd /home/spirit-ai/Intern_g1
bash docker_run_server.sh
```

首次运行时：
- 选择 `y` 安装依赖
- 选择 `y` 启动推理服务
- 等待约60秒

### 步骤2: 启动GMSL相机

```bash
source /opt/ros/jazzy/setup.bash
source ~/pygmsl/ros2/install/setup.bash
ros2 launch gmsl gmsl_multi_camera_launch.py device_ids:=2 output_width:=1408 output_height:=1280
```

### 步骤3: 启动导航客户端

```bash
cd /home/spirit-ai/Intern_g1/scripts/realworld
bash run_simple_client.sh
```

---

## 📖 详细启动流程

### 一、启动推理服务（Docker容器）

```bash
cd /home/spirit-ai/Intern_g1
bash docker_run_server.sh
```

**等待时间**: 约60秒（模型加载）

### 二、验证推理服务

```bash
# 检查容器状态
docker ps | grep internnav_server

# 测试HTTP接口
curl http://localhost:5801/health

# 查看日志
docker logs -f internnav_server

# 查看GPU
nvidia-smi
```

### 三、启动GMSL相机

```bash
source /opt/ros/jazzy/setup.bash
source ~/pygmsl/ros2/install/setup.bash

ros2 launch gmsl gmsl_multi_camera_launch.py \
    device_ids:=2 \
    output_width:=1408 \
    output_height:=1280
```

**验证相机**：
```bash
# 列出相机话题
ros2 topic list | grep camera

# 检查频率
ros2 topic hz /camera/cam_high/image_raw

# 查看一帧数据
ros2 topic echo /camera/cam_high/image_raw --once
```

### 四、启动导航客户端

```bash
cd /home/spirit-ai/Intern_g1/scripts/realworld

# 基础测试
bash run_simple_client.sh

# 自定义指令
INSTRUCTION="Turn left and walk to the door" bash run_simple_client.sh

# 使用cam_high话题
RGB_TOPIC="/camera/cam_high/image_raw" INSTRUCTION="Walk forward" bash run_simple_client.sh

# 低频率测试
FPS=2.0 INSTRUCTION="Walk forward" bash run_simple_client.sh
```

---

## 🎯 多终端启动步骤

建议使用tmux或多个终端窗口：

### 终端1 - 推理服务
```bash
cd /home/spirit-ai/Intern_g1
bash docker_run_server.sh
# 等待服务启动后
docker logs -f internnav_server
```

### 终端2 - GMSL相机
```bash
source /opt/ros/jazzy/setup.bash
source ~/pygmsl/ros2/install/setup.bash
ros2 launch gmsl gmsl_multi_camera_launch.py device_ids:=2 output_width:=1408 output_height:=1280
```

### 终端3 - 导航客户端
```bash
cd /home/spirit-ai/Intern_g1/scripts/realworld
INSTRUCTION="Walk forward to the door" bash run_simple_client.sh
```

### 终端4 - 监控
```bash
watch -n 1 nvidia-smi
```

---

## 🔧 配置参数说明

### Docker环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `CUDA_VISIBLE_DEVICES` | `0` | 使用的GPU编号 |
| `SERVER_PORT` | `5801` | 推理服务端口 |
| `MODEL_VARIANT` | `NavDP` | 模型变体 (NavDP/DualVLN) |
| `CONTAINER_NAME` | `internnav_server` | 容器名称 |

**使用示例**：
```bash
# 使用GPU 1
CUDA_VISIBLE_DEVICES=1 bash docker_run_server.sh

# 使用不同端口
SERVER_PORT=5802 bash docker_run_server.sh

# 使用DualVLN模型
MODEL_VARIANT=DualVLN bash docker_run_server.sh
```

### 客户端环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `INSTRUCTION` | `Walk forward to the door` | 导航指令 |
| `SERVER_URL` | `http://localhost:5801/eval_dual` | 推理服务地址 |
| `RGB_TOPIC` | `/camera/camera/color/image_raw` | RGB话题名称 |
| `FPS` | `5.0` | 推理频率 |

### 推理服务参数

```bash
python3 scripts/realworld/http_internvla_server.py \
    --device cuda:0 \              # GPU设备
    --model_path /workspace/model_vln \  # 模型路径
    --resize_w 384 \               # 输入宽度
    --resize_h 384 \               # 输入高度
    --num_history 8 \              # 历史帧数（4-8）
    --plan_step_gap 8 \            # 规划步长
    --port 5801 \                  # 服务端口
    --skip_warmup                  # 跳过预热
```

---

## 🎮 测试场景

### 场景1: 基础推理测试（低频）
```bash
cd /home/spirit-ai/Intern_g1/scripts/realworld
FPS=2.0 INSTRUCTION="Walk forward" bash run_simple_client.sh
```

### 场景2: 标准导航测试
```bash
INSTRUCTION="Turn left and walk to the black TV" bash run_simple_client.sh
```

### 场景3: 使用cam_high话题
```bash
RGB_TOPIC="/camera/cam_high/image_raw" \
INSTRUCTION="Walk to the door" \
bash run_simple_client.sh
```

### 场景4: 高频率压力测试
```bash
FPS=10.0 INSTRUCTION="Go forward" bash run_simple_client.sh
```

---

## 🛠️ 故障排查

### 问题1: 推理服务无响应

```bash
# 检查容器状态
docker ps -a | grep internnav_server

# 查看日志
docker logs internnav_server

# 重启容器
docker restart internnav_server

# 进入容器排查
docker exec -it internnav_server bash
tail -50 /tmp/server.log
```

### 问题2: 找不到RGB话题

```bash
# 列出所有相机话题
ros2 topic list | grep -E "camera|image"

# 检查话题类型
ros2 topic info /camera/cam_high/image_raw

# 检查话题频率
ros2 topic hz /camera/cam_high/image_raw

# 查看一帧数据
ros2 topic echo /camera/cam_high/image_raw --once
```

### 问题3: GMSL相机未启动

```bash
# 检查相机节点
ros2 node list | grep gmsl

# 查看相机信息
ros2 node info /gmsl_camera_node

# 重新启动相机
source /opt/ros/jazzy/setup.bash
source ~/pygmsl/ros2/install/setup.bash
ros2 launch gmsl gmsl_multi_camera_launch.py device_ids:=2
```

### 问题4: GPU内存不足

```bash
# 减少历史帧数（在容器内）
docker exec -it internnav_server bash

python3 scripts/realworld/http_internvla_server.py \
    --device cuda:0 \
    --model_path /workspace/model_vln \
    --num_history 4 \      # 从8减到4
    --resize_w 256 \       # 降低分辨率
    --resize_h 256 \
    --port 5801
```

### 问题5: Flash-Attention安装问题

```bash
# 检查安装状态
docker exec -it internnav_server bash
pip3 list | grep flash

# 手动安装
pip3 install flash-attn==2.7.4.post1 --no-build-isolation
```

### 问题6: 端口被占用

```bash
# 查找占用端口的进程
sudo lsof -i :5801

# 使用其他端口
SERVER_PORT=5802 bash docker_run_server.sh
```

---

## ⚡ 快速命令参考

### 推理服务管理

```bash
cd /home/spirit-ai/Intern_g1

# 启动
bash docker_run_server.sh

# 查看日志
docker logs -f internnav_server

# 重启
docker restart internnav_server

# 停止
docker stop internnav_server

# 删除
docker rm -f internnav_server

# 进入容器
docker exec -it internnav_server bash

# 健康检查
curl http://localhost:5801/health

# 查看容器资源
docker stats internnav_server
```

### 相机管理

```bash
# 启动相机
source /opt/ros/jazzy/setup.bash
source ~/pygmsl/ros2/install/setup.bash
ros2 launch gmsl gmsl_multi_camera_launch.py device_ids:=2

# 列出话题
ros2 topic list | grep camera

# 检查频率
ros2 topic hz /camera/cam_high/image_raw

# 查看话题信息
ros2 topic info /camera/cam_high/image_raw

# 查看节点
ros2 node list | grep gmsl
```

### 客户端管理

```bash
cd /home/spirit-ai/Intern_g1/scripts/realworld

# 基础启动
bash run_simple_client.sh

# 自定义指令
INSTRUCTION="Walk forward" bash run_simple_client.sh

# 自定义话题
RGB_TOPIC="/camera/cam_high/image_raw" bash run_simple_client.sh

# 自定义频率
FPS=2.0 bash run_simple_client.sh

# 组合配置
RGB_TOPIC="/camera/cam_high/image_raw" \
FPS=5.0 \
INSTRUCTION="Turn left and walk to the door" \
bash run_simple_client.sh
```

### 监控命令

```bash
# GPU状态
nvidia-smi

# 实时GPU监控
watch -n 1 nvidia-smi

# 容器资源
docker stats internnav_server

# 推理服务日志
docker exec internnav_server tail -f /tmp/server.log

# ROS话题监控
ros2 topic list
ros2 topic hz /camera/cam_high/image_raw
```

---

## 📊 性能指标

### 预期性能
- **推理延迟**: 150-300ms
- **推理频率**: 3-10 FPS
- **GPU内存**: 3-5GB
- **模型大小**: 16.7GB
- **容器大小**: 39.4GB

### 系统要求
- **GPU**: NVIDIA GPU，至少16GB显存
- **内存**: 至少32GB RAM
- **存储**: 至少50GB可用空间
- **架构**: ARM64
- **CUDA**: 12.0+

---

## 🗂️ 关键文件位置

```
/home/spirit-ai/Intern_g1/
├── docker_run_server.sh                  # 一键启动脚本
├── docker_quick_start.sh                 # 快速启动容器
├── docker_start_server_in_container.sh   # 容器内服务启动
├── scripts/realworld/
│   ├── simple_client.py                  # 简化客户端（仅RGB）
│   ├── run_simple_client.sh              # 客户端启动脚本
│   ├── quick_test.sh                     # 系统检查脚本
│   └── http_internvla_server.py          # 推理服务器
├── requirements/
│   ├── core_requirements.txt             # 核心依赖
│   └── internvla_n1.txt                  # InternVLA-N1依赖
├── START_GUIDE.md                        # 本文档
├── DOCKER_DEPLOYMENT_GUIDE.md            # 详细部署指南
├── QUICK_START.md                        # 快速开始
└── DOCKER_QUICK_START.md                 # Docker快速启动

/home/spirit-ai/model_vln/
├── config.json                           # 模型配置
├── model-00001-of-00004.safetensors      # 4.9GB
├── model-00002-of-00004.safetensors      # 5.0GB
├── model-00003-of-00004.safetensors      # 4.9GB
├── model-00004-of-00004.safetensors      # 1.9GB
├── model.safetensors.index.json          # 模型索引
└── tokenizer_config.json                 # 分词器配置
```

---

## ✅ 启动检查清单

- [ ] Docker容器运行中（`docker ps | grep internnav_server`）
- [ ] 推理服务响应（`curl http://localhost:5801/health`）
- [ ] GPU可用（`nvidia-smi`显示正常）
- [ ] GMSL相机发布数据（`ros2 topic hz /camera/cam_high/image_raw`）
- [ ] ROS环境加载（`echo $ROS_DISTRO` 显示 `jazzy`）
- [ ] 客户端可以接收图像

---

## 📚 相关文档

- **技术交接文档**: `InternNav_G1_Technical_Handover_20260629.docx`
- **Docker详细指南**: `DOCKER_DEPLOYMENT_GUIDE.md`
- **快速开始**: `QUICK_START.md`
- **项目README**: `README.md`
- **InternNav官方文档**: https://internrobotics.github.io/user_guide/internnav/

---

## 🔄 历史变更记录

### 2026-07-14
- 架构从 G1机器人 改为 本机一体化部署
- ROS版本从 Foxy 升级到 Jazzy
- 相机从 RealSense D455 (RGB-D) 改为 GMSL相机 (RGB only)
- 移除深度图像依赖
- 创建简化客户端 `simple_client.py`

---

**文档维护**: spirit-ai  
**最后更新**: 2026-07-14
