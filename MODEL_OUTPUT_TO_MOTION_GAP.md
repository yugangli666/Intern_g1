# 推理模型输出与运动控制缺口分析

**日期**: 2026-07-14  
**分析对象**: InternVLA-N1 推理模型 → 机器人运动控制

---

## 🎯 推理模型返回什么？

### 模型输出格式

推理服务 (`http://localhost:5801/eval_dual`) 返回 **JSON格式**，有两种输出模式：

#### 模式1: 轨迹模式 (Trajectory Mode) - **主要模式**

```json
{
  "trajectory": [
    [x1, y1],  // 相对于当前位置的轨迹点
    [x2, y2],
    [x3, y3],
    ...
  ],
  "pixel_goal": [u, v]  // 可选：图像中的像素目标
}
```

**特点**:
- `trajectory`: **局部坐标系**下的轨迹点序列 (单位：米)
- 相对于机器人当前位置
- 通常包含 10-30 个点
- 用于MPC控制器跟踪

**示例**:
```json
{
  "trajectory": [
    [0.1, 0.0],
    [0.2, 0.05],
    [0.3, 0.1],
    [0.4, 0.15],
    [0.5, 0.2]
  ],
  "pixel_goal": [192, 300]
}
```

#### 模式2: 离散动作模式 (Discrete Action Mode) - **备用模式**

```json
{
  "discrete_action": [0]     // 停止
  // 或
  "discrete_action": [1]     // 前进
  // 或
  "discrete_action": [2]     // 左转
  // 或
  "discrete_action": [3]     // 右转
  // 或
  "discrete_action": [5]     // 低头看
}
```

**特点**:
- 简单的离散指令
- 不包含具体轨迹
- 需要预定义的速度映射

---

## 🔄 当前系统处理流程

### simple_client.py (当前客户端)

```
1. 接收RGB图像 → 
2. 发送到推理服务 → 
3. 收到响应:
   {
     "trajectory": [[x1, y1], [x2, y2], ...],
     "pixel_goal": [u, v]
   }
4. ❌ 打印到日志 (无运动控制)
5. ❌ 结束
```

**问题**: 收到轨迹后什么都不做！

---

## 📊 距离运动控制还差什么？

### 完整的控制流程应该是：

```
┌─────────────┐
│  RGB相机    │ 
└──────┬──────┘
       │ 图像
       ▼
┌─────────────┐
│ 推理服务    │ InternVLA-N1模型
└──────┬──────┘
       │ 输出
       ▼
   ┌──────────────────┐
   │ trajectory: [[x,y], │
   │              [x,y]] │  ← 我们在这里！
   └──────┬────────────┘
          │
          ▼
   ┌─────────────────────┐
   │ ❌ 缺失部分1:        │
   │ 坐标转换             │  局部坐标 → 世界坐标
   │ (需要里程计数据)      │
   └──────┬──────────────┘
          │
          ▼
   ┌─────────────────────┐
   │ ❌ 缺失部分2:        │
   │ MPC/PID 控制器       │  轨迹 → 速度指令
   │ (需要casadi)         │
   └──────┬──────────────┘
          │
          ▼
   ┌─────────────────────┐
   │ v = 0.3 m/s         │  线速度
   │ w = 0.2 rad/s       │  角速度
   └──────┬──────────────┘
          │
          ▼
   ┌─────────────────────┐
   │ ❌ 缺失部分3:        │
   │ 底盘控制接口         │  速度 → CAN指令
   │ (需要mc_core集成)    │
   └──────┬──────────────┘
          │
          ▼
   ┌─────────────────────┐
   │ CAN总线              │
   │ /dev/can_ohms_0      │  电机指令
   └──────┬──────────────┘
          │
          ▼
   ┌─────────────────────┐
   │ 机器人电机           │  实际运动
   └─────────────────────┘
```

---

## 🔍 缺失组件详细分析

### 缺失1: 坐标系转换模块

**作用**: 将局部轨迹转换到世界坐标系

**输入**:
- `trajectory`: 相对轨迹点 `[[x1, y1], [x2, y2], ...]`
- `current_pose`: 当前位姿 `[x, y, theta]` (从里程计)

**输出**:
- `world_trajectory`: 世界坐标系轨迹

**代码示例** (已在G1客户端中):
```python
def local_to_world(trajectory, current_pose):
    """局部坐标 → 世界坐标"""
    x, y, yaw = current_pose
    world_traj = []
    
    # 构建变换矩阵
    w_T_b = np.array([
        [np.cos(yaw), -np.sin(yaw), 0, x],
        [np.sin(yaw),  np.cos(yaw), 0, y],
        [0.0, 0.0, 1.0, 0],
        [0.0, 0.0, 0.0, 1.0]
    ])
    
    for point in trajectory:
        # 转换到世界坐标
        w_P = (w_T_b @ np.array([point[0], point[1], 0.0, 1.0]))[:2]
        world_traj.append(w_P)
    
    return np.array(world_traj)
```

**依赖**:
- ✅ numpy (已安装)
- ❌ 里程计数据 `/odom` (缺失)

---

### 缺失2: MPC/PID 控制器

**作用**: 根据轨迹计算速度指令

**输入**:
- `world_trajectory`: 世界坐标系轨迹
- `current_pose`: 当前位姿 `[x, y, theta]`

**输出**:
- `v`: 线速度 (m/s)
- `w`: 角速度 (rad/s)

**代码示例** (已在G1客户端中):
```python
from controllers import Mpc_controller

# 初始化MPC
mpc = Mpc_controller(
    world_trajectory,
    desired_v=0.3,    # 期望速度
    v_max=0.4,        # 最大线速度
    w_max=0.4         # 最大角速度
)

# 计算控制指令
controls, states = mpc.solve(current_pose)
v, w = controls[0]  # 第一步的速度指令
```

**依赖**:
- ❌ **casadi** (关键！未安装)
- ✅ scipy (已安装)
- ✅ numpy (已安装)
- ❌ 里程计数据 (缺失)

---

### 缺失3: 底盘控制接口

**作用**: 将速度指令发送给电机

**输入**:
- `v`: 线速度 (m/s)
- `w`: 角速度 (rad/s)

**输出**:
- CAN总线电机指令

**需要实现**:
```python
import can

class ChassisController:
    def __init__(self):
        # 连接CAN总线
        self.can_bus = can.Bus(channel='can0', bustype='socketcan')
    
    def send_velocity(self, v, w):
        """发送速度指令到底盘"""
        # 将速度转换为左右轮速度
        wheel_base = 0.5  # 轮距 (需要根据实际调整)
        v_left = v - w * wheel_base / 2
        v_right = v + w * wheel_base / 2
        
        # 构造CAN消息
        # 格式需要根据mc_core的协议确定
        msg = can.Message(
            arbitration_id=0x123,
            data=[...],  # 速度数据
            is_extended_id=False
        )
        self.can_bus.send(msg)
```

**依赖**:
- ❌ python-can (需安装)
- ❌ mc_core通信协议文档
- ❌ CAN总线配置 (需要 `sudo ip link set can0 up`)

---

### 缺失4: 里程计数据发布

**作用**: 提供机器人当前位姿

**输入**:
- CAN总线反馈的轮式里程计
- 或 IMU + 轮式里程计融合

**输出**:
- ROS2 `/odom` 话题 (nav_msgs/Odometry)

**需要实现**:
```python
from nav_msgs.msg import Odometry

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # 订阅CAN总线或mc_core
        # 读取位姿数据
        
    def publish_odom(self, x, y, theta, vx, vy, vth):
        """发布里程计"""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # 位置
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = np.sin(theta / 2)
        odom.pose.pose.orientation.w = np.cos(theta / 2)
        
        # 速度
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth
        
        self.odom_pub.publish(odom)
```

**依赖**:
- ✅ rclpy (已有)
- ✅ nav_msgs (已有)
- ❌ 位姿数据源 (需要从CAN或mc_core读取)

---

## 📋 完整的缺失清单

### 优先级1: 立即需要

1. **casadi** (最关键！)
   ```bash
   pip3 install casadi
   ```
   
2. **python-can** (CAN通信)
   ```bash
   pip3 install python-can
   ```

3. **CAN总线配置**
   ```bash
   sudo ip link set can0 type can bitrate 1000000
   sudo ip link set can0 up
   ```

### 优先级2: 核心功能

4. **底盘控制节点**
   - 订阅 `/cmd_vel`
   - 发送CAN指令
   - 需要mc_core协议文档

5. **里程计发布节点**
   - 读取CAN反馈
   - 发布 `/odom`

6. **控制器集成**
   - 复制G1的MPC控制器
   - 集成到客户端

### 优先级3: 集成与测试

7. **修改simple_client.py**
   - 添加控制逻辑
   - 集成坐标转换
   - 集成MPC控制器

8. **安全机制**
   - 速度限制
   - 紧急停止
   - 碰撞检测

---

## 🎯 最短路径：从推理到运动

### 方案A: 简化版本 (最快，2-3小时)

**跳过完整的里程计和坐标转换**，直接使用简化控制：

```python
class SimpleMotionController(SimpleInternNavClient):
    def __init__(self, args):
        super().__init__(args)
        # 添加速度发布器
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # 简单的比例控制器
        self.Kp_v = 0.5
        self.Kp_w = 2.0
    
    def inference_callback(self):
        """推理回调 - 添加运动控制"""
        result = self.call_inference()
        
        if 'trajectory' in result:
            trajectory = result['trajectory']
            
            # 简化：只看第一个点
            if len(trajectory) > 0:
                target = trajectory[0]  # [x, y]
                
                # 计算到目标的距离和角度
                distance = np.sqrt(target[0]**2 + target[1]**2)
                angle = np.arctan2(target[1], target[0])
                
                # 简单的比例控制
                v = self.Kp_v * distance
                w = self.Kp_w * angle
                
                # 限速
                v = np.clip(v, 0, 0.3)
                w = np.clip(w, -0.4, 0.4)
                
                # 发布速度指令
                cmd = Twist()
                cmd.linear.x = float(v)
                cmd.angular.z = float(w)
                self.cmd_vel_pub.publish(cmd)
                
                self.get_logger().info(f'速度指令: v={v:.2f} w={w:.2f}')
```

**优点**: 
- 不需要里程计
- 不需要casadi
- 快速验证整个链路

**缺点**:
- 控制效果差
- 无轨迹跟踪
- 容易震荡

---

### 方案B: 完整版本 (最佳，1-2天)

1. 安装casadi
2. 创建里程计节点
3. 复制G1的MPC控制器
4. 创建底盘控制节点
5. 完整集成测试

---

## 🔧 立即可以做的事情

### 测试1: 验证推理输出 (5分钟)

```bash
# 查看实际的推理输出
docker logs internnav_server 2>&1 | grep "json_output" | tail -5
```

### 测试2: 安装依赖 (5分钟)

```bash
pip3 install casadi python-can
```

### 测试3: 测试控制器 (10分钟)

```bash
cd /home/spirit-ai/Intern_g1
python3 << 'EOF'
import sys
sys.path.append('g1_client')
from controllers import Mpc_controller
import numpy as np

# 测试MPC控制器
trajectory = np.array([[0.5, 0], [1.0, 0], [1.5, 0]])
mpc = Mpc_controller(trajectory, desired_v=0.3, v_max=0.4, w_max=0.4)
current_pose = [0, 0, 0]  # x, y, theta
controls, states = mpc.solve(current_pose)
print(f"✓ MPC计算成功: v={controls[0][0]:.2f}, w={controls[0][1]:.2f}")
EOF
```

---

## 📊 总结表格

| 组件 | 当前状态 | 输入 | 输出 | 缺失 |
|------|----------|------|------|------|
| **推理服务** | ✅ 运行中 | RGB图像 | trajectory + pixel_goal | 无 |
| **坐标转换** | ❌ 缺失 | trajectory + odom | world_trajectory | 里程计 |
| **MPC控制器** | ❌ 缺失 | world_trajectory + pose | v, w | casadi |
| **底盘控制** | ❌ 缺失 | v, w | CAN指令 | mc_core集成 |
| **里程计** | ❌ 缺失 | CAN反馈 | /odom | 数据源 |

---

## 💡 推荐行动

**今天**:
1. 安装 casadi: `pip3 install casadi python-can`
2. 测试MPC控制器代码
3. 创建简化版本的运动控制

**本周**:
1. 了解mc_core的CAN协议
2. 创建底盘控制节点
3. 创建里程计节点
4. 完整集成测试

---

**核心答案**:
- **推理模型返回**: 轨迹点序列 `[[x, y], ...]` (局部坐标系)
- **距离运动控制差**: casadi库 + 底盘控制接口 + 里程计数据
- **最快路径**: 安装casadi → 简化的比例控制器 → 连接/cmd_vel

---

**文档维护**: spirit-ai  
**最后更新**: 2026-07-14
