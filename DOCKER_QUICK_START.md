# InternNav Docker 启动方法速查表

## 🚀 三种启动方式

### 方式一：一键自动启动（推荐新手）

```bash
cd /home/spirit-ai/Intern_g1
bash docker_run_server.sh
```

按提示操作：
1. 首次运行选择 `y` 安装依赖
2. 选择 `y` 立即启动推理服务
3. 等待约60秒模型加载完成

---

### 方式二：快速启动（推荐熟悉用户）

```bash
cd /home/spirit-ai/Intern_g1

# 1. 启动容器
bash docker_quick_start.sh

# 2. 在容器内启动服务
docker exec -it internnav_server bash /workspace/InternNav/docker_start_server_in_container.sh
```

---

### 方式三：手动分步启动（推荐开发调试）

```bash
# 1. 启动容器
docker run -d \
    --name internnav_server \
    --gpus all \
    --net=host \
    --privileged \
    -v /home/spirit-ai/Intern_g1:/workspace/InternNav:rw \
    -v /home/spirit-ai/model_vln:/workspace/model_vln:ro \
    -e CUDA_VISIBLE_DEVICES=0 \
    -e PYTHONUNBUFFERED=1 \
    -w /workspace/InternNav \
    harbor.i.spirit-ai.com:443/slam_nav/nav_release:jazzy-thor-vln-deps-fixed-20260714 \
    bash -lc "while true; do sleep 3600; done"

# 2. 进入容器
docker exec -it internnav_server bash

# 3. 在容器内执行以下命令：
cd /workspace/InternNav

# 安装依赖（首次运行）
pip3 install -r requirements/internvla_n1.txt
pip3 install -r requirements/core_requirements.txt
pip3 install -e . --no-deps

# 启动推理服务
python3 scripts/realworld/http_internvla_server.py \
    --device cuda:0 \
    --model_path /workspace/model_vln \
    --resize_w 384 \
    --resize_h 384 \
    --num_history 8 \
    --plan_step_gap 8 \
    --port 5801 \
    --skip_warmup
```

---

## ✅ 验证服务是否启动成功

```bash
# 方法1: 检查容器状态
docker ps | grep internnav_server

# 方法2: 查看日志
docker logs -f internnav_server

# 方法3: 测试HTTP接口（需要等待服务完全启动，约60秒）
curl http://localhost:5801/health

# 方法4: 查看GPU使用情况
nvidia-smi
```

---

## 🔧 常用管理命令

```bash
# 查看容器日志
docker logs -f internnav_server

# 进入容器
docker exec -it internnav_server bash

# 停止容器
docker stop internnav_server

# 启动已停止的容器
docker start internnav_server

# 重启容器
docker restart internnav_server

# 删除容器
docker rm -f internnav_server

# 查看容器资源使用
docker stats internnav_server
```

---

## 📊 服务信息

- **容器名称**: `internnav_server`
- **服务地址**: `http://localhost:5801/eval_dual`
- **模型路径**: `/workspace/model_vln` (容器内)
- **项目路径**: `/workspace/InternNav` (容器内)
- **GPU设备**: `cuda:0`
- **输入分辨率**: 384x384
- **历史帧数**: 8帧

---

## 🧪 测试推理服务

### 从G1机器人测试（推荐）

在G1机器人上（192.168.0.225）：

```bash
cd /home/unitree/Intern_g1/g1_client

# 1. 启动D455相机
bash run_d455_camera.sh

# 2. 启动导航客户端（dry-run模式，不控制机器人）
bash run_g1_client.sh \
    --server_url http://192.168.0.170:5801/eval_dual \
    --instruction "From the center, turn left and walk toward the black TV." \
    --dry-run
```

### 从容器内测试

```bash
docker exec -it internnav_server bash

cd /workspace/InternNav
python3 scripts/realworld/http_internvla_client.py \
    --server_url http://localhost:5801/eval_dual \
    --instruction "Walk forward to the door"
```

---

## ⚙️ 自定义配置

### 使用不同的GPU

```bash
CUDA_VISIBLE_DEVICES=1 bash docker_run_server.sh
```

### 使用不同的端口

```bash
SERVER_PORT=5802 bash docker_run_server.sh
```

### 使用DualVLN模型变体

```bash
MODEL_VARIANT=DualVLN bash docker_run_server.sh
```

### 组合配置

```bash
CUDA_VISIBLE_DEVICES=1 SERVER_PORT=5802 MODEL_VARIANT=DualVLN bash docker_run_server.sh
```

---

## ❌ 故障排查

### 问题1: 容器启动失败

```bash
# 检查Docker和NVIDIA支持
docker run --rm --gpus all nvidia/cuda:12.0.0-base-ubuntu22.04 nvidia-smi
```

### 问题2: 模型文件找不到

```bash
# 检查模型文件
ls -lh /home/spirit-ai/model_vln/*.safetensors

# 应该看到4个文件，总大小约16.7GB
```

### 问题3: GPU内存不足

```bash
# 减少历史帧数
python3 scripts/realworld/http_internvla_server.py \
    --num_history 4 \  # 从8减少到4
    --device cuda:0 \
    --model_path /workspace/model_vln \
    --port 5801
```

### 问题4: 端口被占用

```bash
# 查看占用端口的进程
sudo lsof -i :5801

# 使用其他端口
SERVER_PORT=5802 bash docker_run_server.sh
```

### 问题5: 依赖安装失败

```bash
# 进入容器手动排查
docker exec -it internnav_server bash

# 更新pip
pip3 install --upgrade pip

# 分步安装
pip3 install torch torchvision torchaudio
pip3 install transformers==4.51.0
pip3 install flash-attn==2.7.4.post1
```

---

## 📁 文件说明

| 文件 | 说明 |
|------|------|
| `docker_run_server.sh` | 全自动启动脚本（推荐） |
| `docker_quick_start.sh` | 快速启动容器脚本 |
| `docker_start_server_in_container.sh` | 容器内服务启动脚本 |
| `DOCKER_DEPLOYMENT_GUIDE.md` | 详细部署指南 |
| `DOCKER_QUICK_START.md` | 本文档 |

---

## 📚 更多信息

- **技术交接文档**: `InternNav_G1_Technical_Handover_20260629.docx`
- **详细部署指南**: `DOCKER_DEPLOYMENT_GUIDE.md`
- **项目README**: `README.md`
- **InternVLA-N1模型**: `~/model_vln/README.md`

---

## 🎯 完整启动流程示例

```bash
# 1. 进入项目目录
cd /home/spirit-ai/Intern_g1

# 2. 启动Docker容器和服务（一键完成）
bash docker_run_server.sh
# 首次运行选择 y 安装依赖
# 选择 y 启动推理服务

# 3. 等待服务启动（约60秒）
sleep 60

# 4. 验证服务
curl http://localhost:5801/health

# 5. 查看服务日志
docker logs -f internnav_server

# 6. 在G1机器人上测试（可选）
# ssh unitree@192.168.0.225
# cd /home/unitree/Intern_g1/g1_client
# bash run_d455_camera.sh
# bash run_g1_client.sh --server_url http://192.168.0.170:5801/eval_dual --dry-run
```

---

**创建时间**: 2026-07-14  
**适用环境**: Jetson/ARM64 + ROS2 Jazzy + CUDA 13.0
