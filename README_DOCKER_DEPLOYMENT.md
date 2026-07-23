# InternNav Docker 部署文件说明

本目录包含在Docker容器中运行InternNav/InternVLA-N1推理服务的所有必要脚本和文档。

## 📁 文件列表

### 启动脚本

| 文件名 | 用途 | 推荐场景 |
|--------|------|---------|
| `docker_run_server.sh` | **全自动启动脚本** - 检查环境、启动容器、安装依赖、启动服务 | ✅ 新手首选 |
| `docker_quick_start.sh` | **快速启动脚本** - 仅启动容器，手动启动服务 | ✅ 熟悉用户 |
| `docker_start_server_in_container.sh` | **容器内服务启动脚本** - 在已运行的容器内启动推理服务 | ✅ 开发调试 |

### 文档

| 文件名 | 内容 | 目标读者 |
|--------|------|---------|
| `DOCKER_QUICK_START.md` | **速查表** - 常用命令和快速启动方法 | ✅ 所有用户 |
| `DOCKER_DEPLOYMENT_GUIDE.md` | **完整指南** - 详细的部署步骤、配置说明、故障排查 | ✅ 深度用户 |
| `README_DOCKER_DEPLOYMENT.md` | **本文档** - 文件说明和快速导航 | ✅ 首次使用 |

### 原始文档

| 文件名 | 说明 |
|--------|------|
| `InternNav_G1_Technical_Handover_20260629.docx` | 技术交接文档（2026-06-29） |
| `README.md` | InternNav项目原始README |

---

## 🚀 快速开始（3步）

### 第一次使用

```bash
# 1. 进入项目目录
cd /home/spirit-ai/Intern_g1

# 2. 运行自动化脚本
bash docker_run_server.sh

# 3. 按提示选择 y 安装依赖并启动服务
```

### 已经使用过

```bash
# 快速启动
cd /home/spirit-ai/Intern_g1
bash docker_quick_start.sh
docker exec -it internnav_server bash /workspace/InternNav/docker_start_server_in_container.sh
```

---

## 📖 使用哪个文档？

### 我是新手，第一次部署
👉 阅读 **`DOCKER_QUICK_START.md`** - 包含最常用的命令和启动方法

### 我遇到了问题，需要排查
👉 阅读 **`DOCKER_DEPLOYMENT_GUIDE.md`** - 包含详细的故障排查和解决方案

### 我想了解技术背景和历史问题
👉 阅读 **`InternNav_G1_Technical_Handover_20260629.docx`** - 完整的技术交接文档

### 我想自定义配置和优化性能
👉 阅读 **`DOCKER_DEPLOYMENT_GUIDE.md`** 的"进阶使用"部分

---

## 🎯 典型使用场景

### 场景1: 工作站推理服务

在工作站（192.168.0.170）上启动推理服务，供G1机器人调用：

```bash
# 工作站端
cd /home/spirit-ai/Intern_g1
bash docker_run_server.sh

# G1机器人端（192.168.0.225）
ssh unitree@192.168.0.225
cd /home/unitree/Intern_g1/g1_client
bash run_d455_camera.sh
bash run_g1_client.sh \
    --server_url http://192.168.0.170:5801/eval_dual \
    --instruction "Walk to the door" \
    --dry-run
```

### 场景2: 本地开发调试

在本地容器中进行开发和调试：

```bash
cd /home/spirit-ai/Intern_g1
bash docker_quick_start.sh
docker exec -it internnav_server bash

# 在容器内进行开发
cd /workspace/InternNav
# ... 编辑代码，测试功能 ...
python3 scripts/realworld/http_internvla_server.py --port 5801
```

### 场景3: 数据采集

使用G1机器人采集训练数据：

```bash
# G1机器人端
cd /home/unitree/Intern_g1
bash g1_dataset_tool/run_collect_g1_dataset.sh \
    --instruction "Navigate to the kitchen" \
    --enable-motion \
    --target-fps 5

# 数据会自动同步到工作站
# /home/ubuntu/InternNav/g1_dataset_runs_from_g1/
```

---

## 🔧 核心配置说明

### Docker镜像信息

```
镜像: harbor.i.spirit-ai.com:443/slam_nav/nav_release:jazzy-thor-vln-deps-fixed-20260714
架构: ARM64
大小: 39.4 GB
基础: Ubuntu 24.04 + ROS2 Jazzy + CUDA 13.0 + PyTorch 2.8.0
```

### 模型信息

```
模型: InternVLA-N1 (Dual System)
路径: /home/spirit-ai/model_vln
大小: 16.7 GB (4个safetensors分片)
架构: InternVLAN1ForCausalLM (基于Qwen2.5-VL)
精度: bfloat16
```

### 网络配置

```
工作站IP: 192.168.0.170
G1机器人IP: 192.168.0.225
服务端口: 5801
API端点: /eval_dual
```

---

## 💡 重要提示

### ⚠️ 首次运行注意事项

1. **GPU内存要求**: 模型需要约16GB GPU显存
2. **依赖安装时间**: 首次安装依赖约需10-15分钟
3. **模型加载时间**: 首次加载模型约需60秒
4. **网络要求**: 确保工作站和G1机器人在同一网络

### ✅ 验证清单

启动后请验证以下项目：

- [ ] 容器正在运行: `docker ps | grep internnav_server`
- [ ] GPU被正确使用: `nvidia-smi`
- [ ] 服务端口监听: `curl http://localhost:5801/health`
- [ ] 模型文件完整: `ls -lh /home/spirit-ai/model_vln/*.safetensors`
- [ ] 日志无错误: `docker logs internnav_server`

### 🐛 常见问题快速修复

```bash
# GPU不可用
docker run --rm --gpus all nvidia/cuda:12.0.0-base-ubuntu22.04 nvidia-smi

# 端口被占用
SERVER_PORT=5802 bash docker_run_server.sh

# 内存不足
# 修改 num_history 从 8 降到 4

# 容器无法启动
docker stop internnav_server && docker rm internnav_server
bash docker_quick_start.sh
```

---

## 📞 获取帮助

### 查看日志

```bash
# 容器日志
docker logs -f internnav_server

# 推理服务日志（如果存在）
docker exec -it internnav_server cat /workspace/InternNav/server.log

# 系统日志
journalctl -u docker -f
```

### 性能监控

```bash
# GPU使用情况
watch -n 1 nvidia-smi

# 容器资源使用
docker stats internnav_server

# 系统资源
htop
```

### 调试模式

```bash
# 交互式进入容器
docker exec -it internnav_server bash

# 手动运行推理服务（查看详细输出）
cd /workspace/InternNav
python3 scripts/realworld/http_internvla_server.py \
    --device cuda:0 \
    --model_path /workspace/model_vln \
    --port 5801 \
    --skip_warmup
```

---

## 🔗 相关链接

- **InternNav GitHub**: https://github.com/InternRobotics/InternNav
- **InternVLA-N1 主页**: https://internrobotics.github.io/internvla-n1.github.io/
- **技术报告**: https://internrobotics.github.io/internvla-n1.github.io/static/pdfs/InternVLA_N1.pdf
- **DualVLN论文**: https://arxiv.org/abs/2512.08186

---

## 📝 更新日志

- **2026-07-14**: 创建Docker部署脚本和文档
- **2026-06-29**: 技术交接文档（原始）

---

## 📄 许可证

本项目遵循原InternNav项目的许可证。详见 `LICENSE` 文件。

---

**最后更新**: 2026-07-14  
**维护者**: Spirit AI Team
