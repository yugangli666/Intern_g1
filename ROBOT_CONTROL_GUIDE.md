# InternNav 机器人运动控制指南

**文档版本**: 1.0  
**更新时间**: 2026-07-14  
**适用场景**: 添加实际机器人运动控制

---

## 🤖 当前系统状态

### ✅ 已有组件
- **推理服务**: Docker容器 (InternVLA-N1模型)
- **视觉输入**: GMSL相机 (RGB)
- **导航客户端**: `simple_client.py` (仅推理测试，**无运动控制**)

### ❌ 缺失组件
- **机器人控制接口**: 需要集成
- **运动控制器**: MPC/PID控制器
- **机器人通信**: ROS2话题或SDK

---

## 🎯 需要的组件

### 1. 确定机器人平台

首先需要确认您使用的机器人类型：

#### 选项A: Unitree机器人 (G1, Go2, H1等)
```bash
# 项目中已有完整的G1控制代码
# 位置: /home/spirit-ai/Intern_g1/g1_client/
```

#### 选项B: 其他ROS2机器人
```bash
# 使用标准ROS2 /cmd_vel话题
```

#### 选项C: 自定义机器人
```bash
# 需要提供控制API
```

---

## 📦 项目中已有的控制代码

### 完整的G1机器人客户端

项目中有两个完整的G1控制客户端：

#### 1. DualVLN客户端 (推荐)
**文件**: `/home/spirit-ai/Intern_g1/g1_client/http_internvla_client_g1.py`

**功能**:
- ✅ RGB-D图像采集 (RealSense D455)
- ✅ HTTP推理服务调用
- ✅ MPC + PID 运动控制
- ✅ Unitree G1 SDK集成
- ✅ 导航日志记录
- ✅ 安全停止机制

**依赖**:
```bash
# Python包
pip3 install casadi  # MPC控制器
pip3 install scipy   # 轨迹插值

# ROS2包 (需要ROS2 Foxy)
- unitree_go
- unitree_api
- rclpy
- cv_bridge
- message_filters
```

**启动方式** (在G1机器人上):
```bash
cd /home/unitree/Intern_g1/g1_client

# 1. 启动D455相机
bash run_d455_camera.sh

# 2. 启动G1客户端
bash run_g1_client.sh \
  --server_url http://192.168.0.170:5801/eval_dual \
  --instruction "Walk forward to the door"
```

#### 2. Pixel-Goal客户端
**文件**: `/home/spirit-ai/Intern_g1/pixel_goal_nav/g1_client.py`

**功能**:
- ✅ 像素目标导航
- ✅ MPC控制器
- ✅ Dry-run模式（推理测试）
- ✅ 完整的运动执行

---

## 🔧 为当前系统添加运动控制

### 方案一：适配G1客户端到Jazzy + GMSL相机

由于您当前使用：
- ROS2 **Jazzy** (原G1客户端使用Foxy)
- **GMSL相机** (原G1客户端使用D455)

需要修改现有的G1客户端：

#### 步骤1: 创建适配版本

```bash
cd /home/spirit-ai/Intern_g1/scripts/realworld
cp simple_client.py robot_control_client.py
```

#### 步骤2: 添加运动控制模块

需要添加的核心功能：

**A. Unitree SDK集成**
```python
from unitree_api.msg import Request, RequestHeader, RequestIdentity
from unitree_go.msg import SportModeState

# 控制发布器
self.control_pub = self.create_publisher(Request, '/api/sport/request', 5)

# 里程计订阅
self.odom_sub = self.create_subscription(SportModeState, '/lf/odommodestate', callback, qos)
```

**B. 运动控制器** (MPC/PID)
```python
# 已有实现: /home/spirit-ai/Intern_g1/g1_client/controllers.py
from controllers import Mpc_controller, PID_controller

# 初始化MPC
mpc = Mpc_controller(trajectory, desired_v=0.3, v_max=0.4, w_max=0.4)

# 计算控制指令
controls, states = mpc.solve(current_pose)
v, w = controls[0]  # 线速度、角速度
```

**C. 发送控制指令**
```python
def send_velocity_command(v, w):
    """发送速度指令到Unitree G1"""
    request = Request()
    request.header = RequestHeader()
    request.header.identity = RequestIdentity()
    request.header.identity.id = 12345
    request.header.identity.api_id = 1008  # Move command
    
    # 构造速度指令
    data = {
        "x": float(v),      # 前进速度 m/s
        "y": 0.0,
        "z": float(w)       # 旋转速度 rad/s
    }
    request.parameter = json.dumps(data)
    
    self.control_pub.publish(request)
```

---

### 方案二：使用标准ROS2 cmd_vel话题

如果您的机器人支持标准ROS2接口：

```python
from geometry_msgs.msg import Twist

# 创建发布器
self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

def send_velocity(v, w):
    """发送速度指令"""
    msg = Twist()
    msg.linear.x = v   # 前进速度 m/s
    msg.angular.z = w  # 旋转速度 rad/s
    self.cmd_vel_pub.publish(msg)
```

---

### 方案三：使用现成的G1客户端（最快）

**如果您有Unitree G1机器人**，最快的方式是直接使用现成的G1客户端：

```bash
# 在G1机器人上
cd /home/unitree/Intern_g1/g1_client

# 安装依赖
pip3 install -r requirements_g1.txt

# 启动相机（如果使用D455）
bash run_d455_camera.sh

# 启动客户端
bash run_g1_client.sh \
  --server_url http://192.168.0.170:5801/eval_dual \
  --instruction "Walk forward to the door" \
  --log-dir ./logs
```

---

## 📋 完整的运动控制集成清单

### 必需组件

- [ ] **机器人SDK/API**
  - Unitree: `unitree_go`, `unitree_api`
  - 通用: ROS2 `geometry_msgs`

- [ ] **运动控制器**
  - MPC控制器 (需要`casadi`)
  - PID控制器 (简单实现)

- [ ] **里程计数据**
  - 机器人位姿 (x, y, theta)
  - IMU数据（可选）

- [ ] **安全机制**
  - 速度限制
  - 碰撞检测
  - 紧急停止

### 可选组件

- [ ] **深度信息** (用于避障)
- [ ] **轨迹规划器**
- [ ] **导航日志记录**
- [ ] **可视化工具**

---

## 🚀 快速启动方案

### 如果您使用Unitree G1:

**工作站端**:
```bash
# 启动推理服务
cd /home/spirit-ai/Intern_g1
./start_all.sh
```

**G1机器人端**:
```bash
# 1. 上传代码到G1
scp -r /home/spirit-ai/Intern_g1/g1_client unitree@192.168.0.225:/home/unitree/Intern_g1/

# 2. 在G1上启动
ssh unitree@192.168.0.225
cd /home/unitree/Intern_g1/g1_client

# 3. 启动相机
bash run_d455_camera.sh

# 4. 启动导航客户端（带运动控制）
bash run_g1_client.sh \
  --server_url http://192.168.0.170:5801/eval_dual \
  --instruction "Walk forward to the door"
```

---

### 如果您使用其他ROS2机器人:

需要创建适配版本的客户端。我可以帮您：

1. **提供您的机器人信息**:
   - 机器人类型/型号
   - 控制话题名称（如`/cmd_vel`）
   - 里程计话题名称（如`/odom`）
   - 速度限制 (v_max, w_max)

2. **我帮您创建适配脚本**

---

## 🛠️ 依赖安装

### Unitree G1所需依赖

```bash
# Python包
pip3 install casadi scipy pillow requests numpy opencv-python

# ROS2包（需要Unitree SDK）
# 通常已随Unitree SDK安装
```

### 通用ROS2机器人依赖

```bash
# Python包
pip3 install rclpy opencv-python pillow requests numpy

# ROS2包
sudo apt install ros-jazzy-geometry-msgs ros-jazzy-nav-msgs
```

---

## 📊 控制参数说明

### MPC控制器参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `desired_v` | 0.3 | 期望线速度 (m/s) |
| `v_max` | 0.4 | 最大线速度 (m/s) |
| `w_max` | 0.4 | 最大角速度 (rad/s) |
| `N` | 20 | 预测时域 |
| `T` | 0.1 | 时间步长 (s) |

### PID控制器参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `Kp_v` | 1.0 | 线速度比例增益 |
| `Kp_w` | 2.0 | 角速度比例增益 |
| `v_max` | 0.4 | 最大线速度 (m/s) |
| `w_max` | 0.4 | 最大角速度 (rad/s) |

---

## 🔍 测试流程

### 1. Dry-run测试（无运动）

```bash
bash run_g1_client.sh \
  --server_url http://192.168.0.170:5801/eval_dual \
  --instruction "Walk forward" \
  --dry-run
```

### 2. 低速测试

```bash
bash run_g1_client.sh \
  --server_url http://192.168.0.170:5801/eval_dual \
  --instruction "Walk forward" \
  --desired_v 0.1 \
  --v_max 0.2
```

### 3. 正常测试

```bash
bash run_g1_client.sh \
  --server_url http://192.168.0.170:5801/eval_dual \
  --instruction "Walk forward to the door"
```

---

## 🔐 安全注意事项

1. **首次测试必须使用 `--dry-run`**
2. **确保周围环境安全**，无障碍物
3. **准备紧急停止按钮**
4. **从低速开始测试** (`v_max=0.2`)
5. **有人全程监控机器人**
6. **测试前检查**:
   - 推理服务正常
   - 相机数据正常
   - 机器人通信正常
   - 紧急停止功能正常

---

## 📚 相关文件

```
/home/spirit-ai/Intern_g1/
├── g1_client/                          # G1机器人完整客户端
│   ├── http_internvla_client_g1.py    # 主客户端（RGB-D + 运动控制）
│   ├── controllers.py                  # MPC + PID控制器
│   ├── run_g1_client.sh                # 启动脚本
│   ├── run_d455_camera.sh              # D455相机启动
│   └── README.md                       # G1客户端文档
├── pixel_goal_nav/                     # 像素目标导航
│   ├── g1_client.py                    # 像素目标G1客户端
│   └── controllers.py                  # MPC控制器
└── scripts/realworld/
    ├── simple_client.py                # 简化客户端（仅推理）
    └── robot_control_client.py         # 待创建：带运动控制的客户端
```

---

## ❓ 常见问题

### Q1: 我的机器人不是Unitree G1怎么办？

A: 需要适配您的机器人接口。请提供：
- 机器人型号
- 控制接口（ROS话题或SDK）
- 里程计接口
- 速度限制参数

### Q2: 可以只用RGB相机吗？

A: 可以，但：
- 避障能力下降
- 需要依赖视觉推理判断距离
- 建议在开阔环境测试

### Q3: MPC和PID有什么区别？

A: 
- **MPC**: 模型预测控制，轨迹跟踪更平滑，计算量大
- **PID**: 简单快速，适合基础导航，可能有震荡

### Q4: 如何调整速度参数？

A: 
```bash
# 保守设置（室内）
--desired_v 0.2 --v_max 0.3 --w_max 0.3

# 正常设置
--desired_v 0.3 --v_max 0.4 --w_max 0.4

# 快速设置（开阔环境）
--desired_v 0.4 --v_max 0.6 --w_max 0.5
```

---

## 📞 下一步

请告诉我：

1. **您的机器人类型**:
   - [ ] Unitree G1
   - [ ] Unitree Go2/H1
   - [ ] 其他ROS2机器人
   - [ ] 自定义机器人

2. **您的需求**:
   - [ ] 直接使用G1客户端
   - [ ] 需要适配到Jazzy + GMSL
   - [ ] 创建通用ROS2版本
   - [ ] 自定义集成

3. **当前环境**:
   - 机器人IP/连接方式
   - 控制话题名称
   - 里程计话题名称

我可以根据您的具体情况创建适配脚本！

---

**文档维护**: spirit-ai  
**最后更新**: 2026-07-14
