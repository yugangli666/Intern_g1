# InternNav 快速启动命令速查表

## 🚀 一键启动（推荐）

```bash
# 启动推理服务（终端1）
cd /home/spirit-ai/Intern_g1
./start_all.sh

# 启动相机（终端2）
cd /home/spirit-ai/Intern_g1
./start_camera.sh

# 启动客户端（终端3）
cd /home/spirit-ai/Intern_g1
./start_client.sh
```

---

## 📋 分步启动

### 步骤1: 推理服务
```bash
cd /home/spirit-ai/Intern_g1
bash docker_run_server.sh
# 首次运行: 选择 y 安装依赖和启动服务
# 等待60秒
```

### 步骤2: 相机
```bash
source /opt/ros/jazzy/setup.bash
source ~/pygmsl/ros2/install/setup.bash
ros2 launch gmsl gmsl_multi_camera_launch.py device_ids:=2 output_width:=1408 output_height:=1280
```

### 步骤3: 客户端
```bash
cd /home/spirit-ai/Intern_g1/scripts/realworld
bash run_simple_client.sh
```

---

## 🎯 自定义启动

### 相机 - 不同设备ID
```bash
DEVICE_IDS=4,5 ./start_camera.sh
```

### 客户端 - 自定义指令
```bash
INSTRUCTION="Turn left and walk to the door" ./start_client.sh
```

### 客户端 - 自定义话题
```bash
RGB_TOPIC="/camera/cam_high/image_raw" ./start_client.sh
```

### 客户端 - 低频率测试
```bash
FPS=2.0 ./start_client.sh
```

### 客户端 - 组合配置
```bash
RGB_TOPIC="/camera/cam_high/image_raw" \
FPS=5.0 \
INSTRUCTION="Walk forward" \
./start_client.sh
```

---

## ✅ 验证命令

```bash
# 推理服务
docker ps | grep internnav_server
curl http://localhost:5801/health
docker logs -f internnav_server

# 相机
ros2 topic list | grep camera
ros2 topic hz /camera/cam_high/image_raw

# GPU
nvidia-smi
watch -n 1 nvidia-smi
```

---

## 🛠️ 管理命令

```bash
# 重启推理服务
docker restart internnav_server

# 停止推理服务
docker stop internnav_server

# 查看日志
docker logs -f internnav_server

# 进入容器
docker exec -it internnav_server bash
```

---

## 📁 脚本文件

| 文件 | 用途 |
|------|------|
| `start_all.sh` | 启动推理服务（包含验证） |
| `start_camera.sh` | 启动GMSL相机 |
| `start_client.sh` | 启动导航客户端 |
| `START_GUIDE.md` | 详细启动指南 |
| `QUICK_COMMANDS.md` | 本文档 |

---

## 🔥 最常用命令

```bash
# === 完整启动流程 ===
# 终端1
cd /home/spirit-ai/Intern_g1 && ./start_all.sh

# 终端2（等推理服务启动后）
cd /home/spirit-ai/Intern_g1 && ./start_camera.sh

# 终端3（等相机启动后）
cd /home/spirit-ai/Intern_g1 && ./start_client.sh

# === 快速测试 ===
# 推理服务健康检查
curl http://localhost:5801/health

# 相机话题检查
ros2 topic hz /camera/cam_high/image_raw

# GPU状态
nvidia-smi
```

---

**更新时间**: 2026-07-14
