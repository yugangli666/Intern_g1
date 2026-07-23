# InternNav 完整测试指南 - GMSL相机版本

## 🚀 一键启动（最简单）

```bash
cd /home/spirit-ai/Intern_g1/scripts/realworld
bash run_full_test.sh
```

这个脚本会自动：
1. ✅ 检查推理服务
2. ✅ 加载ROS Jazzy环境
3. ✅ 启动GMSL相机
4. ✅ 启动InternNav推理客户端
5. ✅ Ctrl+C时自动清理

---

## 🎛️ 自定义参数

### 基本用法

```bash
# 自定义导航指令
INSTRUCTION="Turn left and walk to the door" bash run_full_test.sh

# 自定义FPS
FPS=3.0 bash run_full_test.sh

# 自定义相机话题
CAMERA_TOPIC="/camera/cam0/image_raw" bash run_full_test.sh
```

### 高级配置

```bash
# 完整配置
DEVICE_IDS="4,5" \
OUTPUT_WIDTH="1408" \
OUTPUT_HEIGHT="1280" \
CAMERA_TOPIC="/camera/cam_high/image_raw" \
INSTRUCTION="Walk forward to the door" \
FPS=5.0 \
bash run_full_test.sh
```

---

## 📋 环境变量说明

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `DEVICE_IDS` | `4,5` | GMSL相机设备ID |
| `OUTPUT_WIDTH` | `1408` | 相机输出宽度 |
| `OUTPUT_HEIGHT` | `1280` | 相机输出高度 |
| `CAMERA_TOPIC` | `/camera/cam_high/image_raw` | RGB图像话题 |
| `INSTRUCTION` | `Walk forward to the door` | 导航指令 |
| `FPS` | `5.0` | 推理频率 |
| `SERVER_URL` | `http://localhost:5801/eval_dual` | 推理服务地址 |

---

## 📊 预期输出

```
============================================================
  InternNav 完整测试 - GMSL相机 + 推理客户端
============================================================

配置参数:
  相机设备ID: 4,5
  相机分辨率: 1408x1280
  RGB话题: /camera/cam_high/image_raw
  推理服务: http://localhost:5801/eval_dual
  导航指令: Walk forward to the door
  推理FPS: 5.0

[1/4] 检查推理服务...
  ✓ 推理服务运行中

[2/4] 检查ROS环境...
  ✓ ROS Jazzy环境加载
  ✓ GMSL ROS包加载

[3/4] 启动GMSL相机...
  设备ID: 4,5
  分辨率: 1408x1280
  相机节点PID: 12345
  等待相机初始化...
  ✓ 相机启动成功
  
  检查相机话题...
  可用的相机话题:
    /camera/cam0/image_raw
    /camera/cam1/image_raw
  
  使用话题: /camera/cam0/image_raw

[4/4] 启动InternNav推理客户端...
============================================================
  按 Ctrl+C 停止测试
============================================================

[INFO] 客户端已启动
[INFO]   RGB话题: /camera/cam0/image_raw
[INFO]   推理服务: http://localhost:5801/eval_dual
[INFO]   导航指令: Walk forward to the door
============================================================
------------------------------------------------------------
[推理 #1] 完成
  HTTP延迟: 234ms
  像素目标: [704, 640]
  S2输出: forward
------------------------------------------------------------
[推理 #2] 完成
  HTTP延迟: 189ms
  像素目标: [710, 635]
------------------------------------------------------------
```

---

## 🛠️ 故障排查

### 问题1: 推理服务未运行

```bash
# 启动推理服务
docker start internnav_server

# 检查状态
docker ps | grep internnav_server
```

### 问题2: GMSL包未找到

```bash
# 检查GMSL ROS包
ls ~/pygmsl/ros2/install/setup.bash

# 如果不存在，需要先编译GMSL包
```

### 问题3: 相机启动失败

```bash
# 查看相机日志
tail -50 /tmp/gmsl_camera.log

# 检查设备ID是否正确
DEVICE_IDS="4,5" bash run_full_test.sh
```

### 问题4: 找不到相机话题

```bash
# 手动检查可用话题
ros2 topic list | grep camera

# 使用正确的话题
CAMERA_TOPIC="/找到的话题" bash run_full_test.sh
```

---

## 🔄 手动分步测试（调试用）

如果自动脚本有问题，可以手动分步执行：

### 步骤1: 启动相机（终端1）

```bash
source /opt/ros/jazzy/setup.bash
source ~/pygmsl/ros2/install/setup.bash

ros2 launch gmsl gmsl_multi_camera_launch.py \
    device_ids:=4,5 \
    output_width:=1408 \
    output_height:=1280
```

### 步骤2: 检查话题（终端2）

```bash
source /opt/ros/jazzy/setup.bash

# 列出相机话题
ros2 topic list | grep camera

# 检查话题频率
ros2 topic hz /camera/cam0/image_raw
```

### 步骤3: 启动客户端（终端2）

```bash
cd /home/spirit-ai/Intern_g1/scripts/realworld

RGB_TOPIC="/camera/cam0/image_raw" \
INSTRUCTION="Walk forward" \
bash run_simple_client.sh
```

---

## 📂 相关文件

```
/home/spirit-ai/Intern_g1/scripts/realworld/
├── run_full_test.sh        # 完整测试脚本（推荐）
├── run_simple_client.sh    # 客户端启动脚本
├── simple_client.py        # 客户端代码
├── quick_test.sh           # 系统检查脚本
└── ...
```

---

## ✅ 快速命令参考

```bash
# 基本测试
cd /home/spirit-ai/Intern_g1/scripts/realworld
bash run_full_test.sh

# 自定义指令
INSTRUCTION="Turn left" bash run_full_test.sh

# 低频测试
FPS=2.0 bash run_full_test.sh

# 检查推理服务
docker ps | grep internnav_server

# 查看相机日志
tail -f /tmp/gmsl_camera.log

# 查看推理日志
docker exec internnav_server tail -f /tmp/server.log
```

---

## 🎯 成功标志

测试成功时，您应该看到：
- ✅ 相机成功启动
- ✅ 客户端接收到RGB图像
- ✅ 推理服务返回结果
- ✅ 显示像素目标和导航指令
- ✅ HTTP延迟在100-500ms范围内

---

**创建时间**: 2026-07-14  
**适用**: GMSL相机 + ROS Jazzy + InternNav推理服务
