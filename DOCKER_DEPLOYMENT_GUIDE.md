# InternNav Docker 部署指南

本文档详细说明如何在Docker容器中运行InternVLA-N1推理服务。

## 目录
- [系统要求](#系统要求)
- [快速开始](#快速开始)
- [详细步骤](#详细步骤)
- [配置说明](#配置说明)
- [常见问题](#常见问题)
- [进阶使用](#进阶使用)

---

## 系统要求

### 硬件要求
- **GPU**: NVIDIA GPU，至少16GB显存（推荐24GB+）
- **内存**: 至少32GB RAM
- **存储**: 至少50GB可用空间
- **架构**: ARM64（适用于Jetson等设备）

### 软件要求
- Docker >= 20.10
- NVIDIA Container Toolkit（nvidia-docker2）
- CUDA Driver >= 12.0
- 操作系统: Ubuntu 20.04+ / Jetson Linux

### 前置准备
```bash
# 1. 验证Docker安装
docker --version

# 2. 验证NVIDIA Docker支持
docker run --rm --gpus all nvidia/cuda:12.0.0-base-ubuntu22.04 nvidia-smi

# 3. 检查GPU可用性
nvidia-smi
```

---

## 快速开始

### 方式一：使用自动化脚本（推荐）

```bash
# 1. 进入项目目录
cd /home/spirit-ai/Intern_g1

# 2. 运行启动脚本
bash docker_run_server.sh

# 脚本会自动：
# - 检查前置条件
# - 停止旧容器
# - 启动新容器
# - 询问是否安装依赖（首次运行选y）
# - 询问是否启动推理服务
```

### 方式二：手动执行

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

# 3. 安装依赖（首次运行）
cd /workspace/InternNav
pip3 install -r requirements/core_requirements.txt
pip3 install -r requirements/internvla_n1.txt
pip3 install -e . --no-deps

# 4. 启动推理服务
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

## 详细步骤

### 步骤1: 拉取Docker镜像

```bash
# 如果镜像不在本地，先拉取
docker pull harbor.i.spirit-ai.com:443/slam_nav/nav_release:jazzy-thor-vln-deps-fixed-20260714

# 查看镜像信息
docker image inspect harbor.i.spirit-ai.com:443/slam_nav/nav_release:jazzy-thor-vln-deps-fixed-20260714
```

### 步骤2: 准备目录结构

```bash
# 确认项目目录存在
ls -la /home/spirit-ai/Intern_g1

# 确认模型目录存在
ls -la /home/spirit-ai/model_vln

# 预期的模型文件
/home/spirit-ai/model_vln/
├── config.json
├── model-00001-of-00004.safetensors
├── model-00002-of-00004.safetensors
├── model-00003-of-00004.safetensors
├── model-00004-of-00004.safetensors
├── model.safetensors.index.json
├── tokenizer_config.json
├── vocab.json
└── ... (其他配置文件)
```

### 步骤3: 启动容器

```bash
# 使用自动化脚本
cd /home/spirit-ai/Intern_g1
bash docker_run_server.sh

# 或使用自定义配置
CUDA_VISIBLE_DEVICES=0 MODEL_VARIANT=NavDP bash docker_run_server.sh
```

### 步骤4: 验证服务

```bash
# 等待服务启动（约30-60秒）
sleep 60

# 检查服务健康状态
curl http://localhost:5801/health

# 检查容器日志
docker logs internnav_server

# 实时查看推理服务日志
docker exec -it internnav_server tail -f /workspace/InternNav/server.log
```

### 步骤5: 测试推理

```bash
# 进入容器测试
docker exec -it internnav_server bash

# 在容器内运行测试脚本
cd /workspace/InternNav
python3 scripts/realworld/test_inference.py \
    --server_url http://localhost:5801/eval_dual \
    --instruction "From the center, turn left and walk toward the black TV."
```

---

## 配置说明

### 环境变量

| 变量名 | 默认值 | 说明 |
|--------|--------|------|
| `CUDA_VISIBLE_DEVICES` | `0` | 使用的GPU设备ID |
| `MODEL_VARIANT` | `NavDP` | 模型变体：NavDP 或 DualVLN |
| `SERVER_PORT` | `5801` | 推理服务端口 |
| `CONTAINER_NAME` | `internnav_server` | 容器名称 |

### 自定义配置示例

```bash
# 使用GPU 1
CUDA_VISIBLE_DEVICES=1 bash docker_run_server.sh

# 使用DualVLN模型变体
MODEL_VARIANT=DualVLN bash docker_run_server.sh

# 使用不同端口
SERVER_PORT=5802 bash docker_run_server.sh

# 组合配置
CUDA_VISIBLE_DEVICES=1 MODEL_VARIANT=DualVLN SERVER_PORT=5802 bash docker_run_server.sh
```

### 推理服务参数

在容器内手动启动时可调整的参数：

```bash
python3 scripts/realworld/http_internvla_server.py \
    --device cuda:0 \              # GPU设备
    --model_path /workspace/model_vln \  # 模型路径
    --resize_w 384 \               # 输入图像宽度
    --resize_h 384 \               # 输入图像高度
    --num_history 8 \              # 历史帧数量
    --plan_step_gap 8 \            # 规划步长间隔
    --port 5801 \                  # 服务端口
    --skip_warmup                  # 跳过预热（加快启动）
```

---

## 常见问题

### Q1: 容器启动失败

**现象**: `docker: Error response from daemon: could not select device driver`

**解决方案**:
```bash
# 检查NVIDIA Docker支持
docker run --rm --gpus all nvidia/cuda:12.0.0-base-ubuntu22.04 nvidia-smi

# 如果失败，重新安装nvidia-container-toolkit
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

### Q2: 模型加载失败

**现象**: `FileNotFoundError: model-00001-of-00004.safetensors`

**解决方案**:
```bash
# 检查模型文件完整性
ls -lh /home/spirit-ai/model_vln/*.safetensors

# 确认所有4个分片都存在
# 每个文件大小应该在1.8GB - 5GB之间
```

### Q3: GPU内存不足

**现象**: `CUDA out of memory`

**解决方案**:
```bash
# 1. 减小batch size或history size
python3 scripts/realworld/http_internvla_server.py \
    --num_history 4 \  # 从8减少到4
    ...

# 2. 使用更小的输入分辨率
python3 scripts/realworld/http_internvla_server.py \
    --resize_w 256 \   # 从384减少到256
    --resize_h 256 \
    ...

# 3. 清理GPU缓存
docker exec -it internnav_server bash -c "python3 -c 'import torch; torch.cuda.empty_cache()'"
```

### Q4: 端口被占用

**现象**: `Address already in use: 5801`

**解决方案**:
```bash
# 查找占用端口的进程
sudo lsof -i :5801

# 停止占用进程或使用其他端口
SERVER_PORT=5802 bash docker_run_server.sh
```

### Q5: 依赖安装失败

**现象**: pip install 报错

**解决方案**:
```bash
# 进入容器手动安装
docker exec -it internnav_server bash

# 更新pip
pip3 install --upgrade pip

# 分步安装依赖
pip3 install torch torchvision torchaudio
pip3 install transformers==4.51.0
pip3 install flash-attn==2.7.4.post1
pip3 install -r requirements/core_requirements.txt
```

### Q6: ROS环境问题

**现象**: ROS相关命令不可用

**解决方案**:
```bash
# 在容器内source ROS环境
docker exec -it internnav_server bash

# 加载ROS 2 Jazzy环境
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=33
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

---

## 进阶使用

### 1. 多GPU并行部署

```bash
# 在GPU 0上启动服务1
CUDA_VISIBLE_DEVICES=0 SERVER_PORT=5801 CONTAINER_NAME=internnav_server_0 bash docker_run_server.sh

# 在GPU 1上启动服务2
CUDA_VISIBLE_DEVICES=1 SERVER_PORT=5802 CONTAINER_NAME=internnav_server_1 bash docker_run_server.sh
```

### 2. 持久化容器配置

创建 `docker-compose.yml`:

```yaml
version: '3.8'

services:
  internnav_server:
    image: harbor.i.spirit-ai.com:443/slam_nav/nav_release:jazzy-thor-vln-deps-fixed-20260714
    container_name: internnav_server
    privileged: true
    network_mode: host
    environment:
      - CUDA_VISIBLE_DEVICES=0
      - PYTHONUNBUFFERED=1
      - ROS_DOMAIN_ID=33
      - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    volumes:
      - /home/spirit-ai/Intern_g1:/workspace/InternNav:rw
      - /home/spirit-ai/model_vln:/workspace/model_vln:ro
    working_dir: /workspace/InternNav
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]
    command: >
      bash -c "
        pip3 install -r requirements/core_requirements.txt &&
        pip3 install -r requirements/internvla_n1.txt &&
        pip3 install -e . --no-deps &&
        python3 scripts/realworld/http_internvla_server.py
          --device cuda:0
          --model_path /workspace/model_vln
          --resize_w 384
          --resize_h 384
          --num_history 8
          --plan_step_gap 8
          --port 5801
          --skip_warmup
      "
```

使用docker-compose启动:
```bash
docker-compose up -d
```

### 3. 监控和日志

```bash
# 实时查看容器资源使用
docker stats internnav_server

# 导出日志
docker logs internnav_server > /tmp/internnav_server.log 2>&1

# 使用nvidia-smi监控GPU
watch -n 1 nvidia-smi
```

### 4. 开发模式

如果需要在容器内开发调试:

```bash
# 启动交互式容器
docker run -it --rm \
    --gpus all \
    --net=host \
    -v /home/spirit-ai/Intern_g1:/workspace/InternNav:rw \
    -v /home/spirit-ai/model_vln:/workspace/model_vln:ro \
    harbor.i.spirit-ai.com:443/slam_nav/nav_release:jazzy-thor-vln-deps-fixed-20260714 \
    bash

# 在容器内进行开发
cd /workspace/InternNav
# ... 开发和测试 ...
```

### 5. 连接G1机器人

当服务在工作站运行后，G1机器人可以通过网络访问:

```bash
# 在G1机器人上（192.168.0.225）
cd /home/unitree/Intern_g1/g1_client

# 启动D455相机
bash run_d455_camera.sh

# 启动导航客户端（指向工作站IP）
bash run_g1_client.sh \
    --server_url http://192.168.0.170:5801/eval_dual \
    --instruction "From the center, turn left and walk toward the black TV." \
    --dry-run  # 首次测试建议加dry-run
```

---

## 容器管理命令速查

```bash
# 启动容器
docker start internnav_server

# 停止容器
docker stop internnav_server

# 重启容器
docker restart internnav_server

# 删除容器
docker rm -f internnav_server

# 进入容器
docker exec -it internnav_server bash

# 查看容器日志
docker logs -f internnav_server

# 查看容器详情
docker inspect internnav_server

# 从容器复制文件
docker cp internnav_server:/workspace/InternNav/server.log ./

# 向容器复制文件
docker cp ./config.yaml internnav_server:/workspace/InternNav/
```

---

## 性能优化建议

1. **使用SSD存储模型文件** - 模型加载速度更快
2. **预热GPU** - 首次推理可能较慢，建议预先运行几次
3. **调整worker数量** - 根据GPU内存调整并发请求数
4. **使用混合精度** - 模型已使用bfloat16，无需额外配置
5. **监控内存使用** - 避免内存泄漏导致性能下降

---

## 技术支持

如遇到问题，请检查：
1. Docker日志: `docker logs internnav_server`
2. 服务日志: `docker exec -it internnav_server cat /workspace/InternNav/server.log`
3. GPU状态: `nvidia-smi`
4. 网络连接: `curl http://localhost:5801/health`

参考文档：
- 技术交接文档: `InternNav_G1_Technical_Handover_20260629.docx`
- 项目README: `README.md`
- InternNav官方文档: https://internrobotics.github.io/user_guide/internnav/

---

**最后更新**: 2026-07-14
