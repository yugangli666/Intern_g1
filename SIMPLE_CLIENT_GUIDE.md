# InternNav 本机测试指南（ROS Jazzy + 简化客户端）

## 🎯 系统配置

- **ROS版本**: Jazzy (不是Foxy)
- **相机**: 当前运行中的相机（通过ROS话题）
- **推理服务**: Docker容器中的InternVLA-N1
- **客户端**: 简化版客户端（无需G1机器人控制库）

---

## ✅ 当前状态检查

### 1. 检查推理服务
```bash
# 检查容器运行
docker ps | grep internnav_server

# 查看服务日志
docker exec internnav_server tail -20 /tmp/server.log

# 测试HTTP响应
curl http://localhost:5801/
```

### 2. 检查相机话题
```bash
# 列出所有话题
ros2 topic list | grep camera

# 检查RGB话题
ros2 topic info /camera/camera/color/image_raw

# 检查深度话题（如果有）
ros2 topic list | grep depth
```

---

## 🚀 快速启动测试

### 方式一：使用启动脚本（推荐）

```bash
cd /home/spirit-ai/Intern_g1/scripts/realworld

# 使用默认配置启动
INSTRUCTION="Walk forward to the door" bash run_simple_client.sh

# 或自定义配置
INSTRUCTION="Turn left and walk to the black TV" \
SERVER_URL="http://localhost:5801/eval_dual" \
RGB_TOPIC="/camera/camera/color/image_raw" \
FPS=3.0 \
bash run_simple_client.sh
```

### 方式二：直接运行Python脚本

```bash
cd /home/spirit-ai/Intern_g1

# 基本用法
python3 scripts/realworld/simple_client.py \
    --instruction "Walk forward to the door" \
    --server_url http://localhost:5801/eval_dual \
    --rgb_topic /camera/camera/color/image_raw \
    --fps 5.0

# 如果有深度话题
python3 scripts/realworld/simple_client.py \
    --instruction "Walk forward to the door" \
    --server_url http://localhost:5801/eval_dual \
    --rgb_topic /camera/camera/color/image_raw \
    --depth_topic /camera/camera/aligned_depth_to_color/image_raw \
    --fps 5.0
```

### 查看所有参数

```bash
python3 scripts/realworld/simple_client.py --help
```

---

## 📊 预期输出

客户端启动后，您应该看到：

```
=== 简化的InternNav客户端启动 ===
项目目录: /home/spirit-ai/Intern_g1

配置:
  推理服务: http://localhost:5801/eval_dual
  RGB话题: /camera/camera/color/image_raw
  深度话题: /camera/camera/aligned_depth_to_color/image_raw
  FPS: 5.0
  指令: Walk forward to the door

✓ 推理服务可达
✓ RGB话题存在

启动客户端...
[INFO] 客户端已启动
[INFO]   RGB话题: /camera/camera/color/image_raw
[INFO]   深度话题: /camera/camera/aligned_depth_to_color/image_raw
[INFO]   推理服务: http://localhost:5801/eval_dual
[INFO]   导航指令: Walk forward to the door
[INFO]   目标FPS: 5.0

等待图像数据...
等待图像数据...
[INFO] [0] 推理完成
[INFO]   延迟: 234ms
[INFO]   响应: {...}
[INFO] [1] 推理完成
[INFO]   延迟: 189ms
[INFO]   响应: {...}
```

---

## 🔧 故障排查

### 问题1: 找不到相机话题

```bash
# 列出所有图像话题
ros2 topic list | grep image

# 使用找到的话题
python3 scripts/realworld/simple_client.py \
    --rgb_topic /你找到的话题名称 \
    --instruction "Walk forward"
```

### 问题2: 推理服务未响应

```bash
# 检查容器状态
docker ps | grep internnav_server

# 如果未运行，启动容器
docker start internnav_server

# 查看详细日志
docker logs -f internnav_server
```

### 问题3: 缺少Python依赖

```bash
# 安装必要的包
pip3 install opencv-python pillow requests numpy

# 检查ROS Python包
pip3 install cv_bridge rclpy
```

### 问题4: 没有深度话题

如果系统中没有深度话题，客户端会等待深度数据。有两个解决方案：

**方案A**: 修改脚本仅使用RGB（需要修改代码）

**方案B**: 查找可用的深度话题
```bash
ros2 topic list | grep -iE 'depth|距离'
```

---

## 📝 测试场景

### 场景1: 基础推理测试
```bash
INSTRUCTION="Walk forward" \
FPS=2.0 \
bash run_simple_client.sh
```

### 场景2: 导航到目标
```bash
INSTRUCTION="Turn left and walk to the black TV" \
FPS=5.0 \
bash run_simple_client.sh
```

### 场景3: 高频率测试
```bash
INSTRUCTION="Go to the door" \
FPS=10.0 \
bash run_simple_client.sh
```

---

## 🎯 与原始G1客户端的区别

| 特性 | G1客户端 | 简化客户端 |
|------|----------|------------|
| ROS版本 | Foxy | Jazzy |
| 机器人控制 | Unitree G1 API | 无（仅推理） |
| 运动执行 | MPC/PID控制器 | 无 |
| 依赖库 | unitree_api, casadi | 仅基础库 |
| 用途 | 实际机器人导航 | 推理服务测试 |

---

## 📂 相关文件

- **简化客户端**: `/home/spirit-ai/Intern_g1/scripts/realworld/simple_client.py`
- **启动脚本**: `/home/spirit-ai/Intern_g1/scripts/realworld/run_simple_client.sh`
- **原始G1客户端**: `/home/spirit-ai/Intern_g1/g1_client/http_internvla_client_g1.py`
- **推理服务**: Docker容器 `internnav_server`

---

## 🚦 下一步

1. ✅ 推理服务已运行
2. ✅ 简化客户端已创建
3. ⏭️ 启动客户端测试推理
4. ⏭️ 检查推理结果和延迟
5. ⏭️ 根据需要调整参数

---

**创建时间**: 2026-07-14  
**适用场景**: 本机测试InternNav推理服务，无需G1机器人硬件
