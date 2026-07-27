# InternNav 本机测试 - 快速开始指南

## 🎯 系统配置

- **ROS**: Jazzy
- **相机**: `/camera/camera/color/image_raw`
- **推理服务**: Docker容器 `internnav_server` @ `http://localhost:5801`
- **客户端模式**: 仅RGB（不需要深度图像）

---

## 🚀 快速开始（3步）

### 步骤1: 系统检查

```bash
cd /home/spirit-ai/Intern_g1/scripts/realworld
bash quick_test.sh
```

**预期输出**: 所有检查项显示 ✓

### 步骤2: 启动客户端

```bash
# 方式1: 使用默认指令
bash run_simple_client.sh

# 方式2: 自定义指令
INSTRUCTION="Turn left and walk to the door" bash run_simple_client.sh

# 方式3: 低频率测试
FPS=2.0 INSTRUCTION="Walk forward" bash run_simple_client.sh
```

### 步骤3: 观察输出

```
============================================================
  简化InternNav客户端启动 (仅RGB模式)
============================================================
[INFO] 客户端已启动
[INFO]   RGB话题: /camera/camera/color/image_raw
[INFO]   推理服务: http://localhost:5801/eval_dual
[INFO]   导航指令: Walk forward
[INFO]   目标FPS: 5.0
============================================================
[INFO] 等待RGB图像数据...
------------------------------------------------------------
[推理 #1] 完成
  HTTP延迟: 234ms
  像素目标: [192, 300]
  S2输出: forward
------------------------------------------------------------
```

---

## 📝 使用说明

### 基本用法

```bash
cd /home/spirit-ai/Intern_g1/scripts/realworld

# 基本测试
INSTRUCTION="Walk forward" bash run_simple_client.sh

# 自定义FPS
FPS=3.0 INSTRUCTION="Turn left" bash run_simple_client.sh

# 自定义RGB话题
RGB_TOPIC="/your/topic" INSTRUCTION="Go to the door" bash run_simple_client.sh
```

### 高级用法（直接调用Python）

```bash
cd /home/spirit-ai/Intern_g1

python3 scripts/realworld/simple_client.py \
    --instruction "Walk forward to the door" \
    --server_url http://localhost:5801/eval_dual \
    --rgb_topic /camera/camera/color/image_raw \
    --fps 5.0 \
    --image_width 384 \
    --image_height 384
```

### 查看所有参数

```bash
python3 scripts/realworld/simple_client.py --help
```

---

## 🔧 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `INSTRUCTION` | `Walk forward to the door` | 导航指令 |
| `SERVER_URL` | `http://localhost:5801/eval_dual` | 推理服务地址 |
| `RGB_TOPIC` | `/camera/camera/color/image_raw` | RGB图像话题 |
| `FPS` | `5.0` | 推理频率（帧/秒） |

---

## 📊 测试场景

### 场景1: 基础推理测试（低频）
```bash
FPS=2.0 INSTRUCTION="Walk forward" bash run_simple_client.sh
```

### 场景2: 标准导航测试
```bash
INSTRUCTION="Turn left and walk to the black TV" bash run_simple_client.sh
```

### 场景3: 高频率压力测试
```bash
FPS=10.0 INSTRUCTION="Walk to the door" bash run_simple_client.sh
```

### 场景4: 自定义RGB话题
```bash
RGB_TOPIC="/image" INSTRUCTION="Go forward" bash run_simple_client.sh
```

---

## 🛠️ 故障排查

### 问题1: 推理服务无响应

```bash
# 检查容器
docker ps | grep internnav_server

# 重启容器
docker restart internnav_server

# 查看日志
docker exec internnav_server tail -50 /tmp/server.log
```

### 问题2: 找不到RGB话题

```bash
# 列出所有图像话题
ros2 topic list | grep image

# 使用正确的话题
RGB_TOPIC="/你的话题名称" bash run_simple_client.sh
```

### 问题3: 客户端无图像数据

```bash
# 检查话题是否发布数据
ros2 topic hz /camera/camera/color/image_raw

# 查看话题信息
ros2 topic info /camera/camera/color/image_raw

# 手动查看一帧
ros2 topic echo /camera/camera/color/image_raw --once
```

### 问题4: Python依赖缺失

```bash
# 安装必要的包
pip3 install opencv-python pillow requests numpy

# ROS Python包（通常已安装）
pip3 install rclpy cv-bridge
```

---

## 📈 性能指标

**预期性能**:
- **推理延迟**: 150-300ms（取决于GPU和模型）
- **推理频率**: 3-10 FPS
- **GPU内存**: ~3-5GB

**监控命令**:
```bash
# 实时GPU监控
watch -n 1 nvidia-smi

# 查看推理日志
docker exec internnav_server tail -f /tmp/server.log
```

---

## 🎯 与G1机器人客户端的区别

| 特性 | G1客户端 | 简化客户端 |
|------|----------|------------|
| **ROS版本** | Foxy | Jazzy |
| **输入数据** | RGB + 深度 | 仅RGB |
| **机器人控制** | ✓ (Unitree API) | ✗ (无) |
| **运动执行** | ✓ (MPC/PID) | ✗ (仅推理) |
| **依赖** | unitree_api, casadi | 最小依赖 |
| **用途** | 实际导航 | 推理测试 |

---

## 📂 相关文件

```
/home/spirit-ai/Intern_g1/
├── scripts/realworld/
│   ├── simple_client.py          # 简化客户端（仅RGB）
│   ├── run_simple_client.sh      # 启动脚本
│   └── quick_test.sh             # 系统检查脚本
├── SIMPLE_CLIENT_GUIDE.md        # 测试指南（旧版）
├── QUICK_START.md                # 本文件
└── DOCKER_DEPLOYMENT_SUMMARY.txt # Docker部署总结
```

---

## ✅ 快速命令参考

```bash
# 进入脚本目录
cd /home/spirit-ai/Intern_g1/scripts/realworld

# 系统检查
bash quick_test.sh

# 启动测试（默认）
bash run_simple_client.sh

# 启动测试（自定义）
INSTRUCTION="Your command here" bash run_simple_client.sh

# 查看推理服务状态
docker ps | grep internnav_server
docker exec internnav_server tail -20 /tmp/server.log

# 查看相机话题
ros2 topic list | grep camera
ros2 topic hz /camera/camera/color/image_raw

# 查看GPU
nvidia-smi
```

---

## 🎉 完成！

系统已就绪，可以开始测试InternNav推理服务！

**下一步**: 运行 `bash quick_test.sh` 验证系统状态，然后启动客户端。

---

**文档版本**: 1.0  
**更新时间**: 2026-07-14  
**适用环境**: ROS Jazzy + Docker + 仅RGB模式
