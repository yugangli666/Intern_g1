# 机器人运动控制缺失组件分析报告

**设备**: NVIDIA Jetson AGX Thor Developer Kit  
**系统**: Ubuntu 24.04.2 LTS (ARM64)  
**ROS**: ROS2 Jazzy  
**日期**: 2026-07-14

---

## 🔍 系统现状分析

### ✅ 已有组件

#### 1. 硬件平台
- **主控**: NVIDIA Jetson AGX Thor (ARM64)
- **CAN总线**: 4个CAN接口 (`/dev/can_ohms_0-3`)
- **激光雷达**: Livox Mid-360s
- **相机**: GMSL相机 (RGB)
- **IMU**: `/livox/imu` (sensor_msgs/msg/Imu)

#### 2. 软件组件
- **推理服务**: Docker容器 (InternVLA-N1) ✅
- **ROS2环境**: Jazzy ✅
- **相机驱动**: pygmsl (GMSL相机) ✅
- **激光雷达驱动**: livox_ros_driver2 ✅
- **导航客户端**: simple_client.py (仅推理) ✅
- **Python库**: numpy, scipy ✅

#### 3. 运动控制基础设施
- **mc_core**: 运动控制核心库 (C++/EtherCAT)
- **CAN总线**: 硬件接口已就绪
- **nav2包**: ROS2导航栈部分组件

### ❌ 缺失组件

#### 1. **运动控制依赖库** (最关键)
```bash
# 缺失的Python包
- casadi         # MPC控制器核心库
- opencv-python  # 视觉处理
```

#### 2. **机器人控制接口**
- ❌ 无 `/cmd_vel` 话题发布器
- ❌ 无里程计数据发布 (`/odom`)
- ❌ 无底盘控制节点
- ❌ 无速度控制接口

#### 3. **控制器实现**
- ❌ 缺少MPC控制器（需要casadi）
- ❌ 缺少PID控制器
- ❌ 缺少轨迹跟踪器

#### 4. **机器人状态反馈**
- ❌ 无位姿估计 (x, y, theta)
- ❌ 无速度反馈 (v, w)
- ❌ IMU数据未集成到导航

---

## 🎯 需要补充的组件

### 优先级1：安装控制依赖 (必需)

```bash
# 安装casadi（MPC控制器必需）
pip3 install casadi

# 安装opencv（图像处理）
pip3 install opencv-python

# 验证安装
python3 -c "import casadi; print('CaSADi version:', casadi.__version__)"
```

### 优先级2：创建底盘控制接口

需要创建一个ROS2节点，连接：
- **输入**: 速度指令 (`/cmd_vel` - geometry_msgs/Twist)
- **输出**: CAN总线电机指令

**两种方案**:

#### 方案A: 使用mc_core (推荐)
```bash
# mc_core是现成的运动控制库
cd /home/spirit-ai/code/mc_core
mkdir build && cd build
cmake .. -DTARGET_ARCH=aarch64 -DOS_TARGET=gnulinux
make -j$(nproc)

# 需要创建ROS2包装器
# 将mc_core集成到ROS2
```

#### 方案B: 创建简化的CAN控制节点
```python
# 简化的底盘控制节点
import can
from geometry_msgs.msg import Twist

class ChassisController(Node):
    def __init__(self):
        super().__init__('chassis_controller')
        self.can_bus = can.Bus(channel='can0', bustype='socketcan')
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
    
    def cmd_vel_callback(self, msg):
        # 将Twist消息转换为CAN指令
        v = msg.linear.x
        w = msg.angular.z
        self.send_can_command(v, w)
```

### 优先级3：创建里程计发布节点

需要获取机器人位姿并发布到ROS2：

```python
class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        # 从CAN总线或mc_core读取位姿
        # 或使用IMU + 轮式里程计融合
```

### 优先级4：集成导航客户端

修改 `simple_client.py` 添加运动控制：

```python
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class RobotControlClient(SimpleInternNavClient):
    def __init__(self, args):
        super().__init__(args)
        
        # 添加控制发布器
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 添加里程计订阅
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # 添加控制器（需要先安装casadi）
        from controllers import Mpc_controller
        self.controller = None  # 初始化后设置
    
    def execute_navigation(self, pixel_goal, action):
        """根据推理结果执行导航"""
        if self.controller is None:
            # 根据像素目标创建轨迹
            trajectory = self.pixel_to_world_trajectory(pixel_goal)
            self.controller = Mpc_controller(trajectory)
        
        # MPC计算控制指令
        current_pose = self.get_current_pose()
        controls, _ = self.controller.solve(current_pose)
        v, w = controls[0]
        
        # 发布速度指令
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_vel_pub.publish(cmd)
```

---

## 📋 完整集成清单

### 阶段1: 基础依赖 (30分钟)
- [ ] 安装 casadi: `pip3 install casadi`
- [ ] 安装 opencv-python: `pip3 install opencv-python`
- [ ] 验证安装成功

### 阶段2: 底盘控制 (2-4小时)
- [ ] 了解mc_core的CAN通信协议
- [ ] 创建ROS2底盘控制节点
- [ ] 测试 `/cmd_vel` → CAN 指令
- [ ] 验证电机响应

### 阶段3: 状态反馈 (2-3小时)
- [ ] 创建里程计发布节点
- [ ] 集成IMU数据
- [ ] 发布 `/odom` 话题
- [ ] 验证位姿数据

### 阶段4: 控制器集成 (2-3小时)
- [ ] 复制G1的控制器代码
- [ ] 修改 simple_client.py
- [ ] 添加MPC/PID控制器
- [ ] 集成轨迹跟踪

### 阶段5: 测试验证 (2-4小时)
- [ ] Dry-run测试（无运动）
- [ ] 低速测试 (v_max=0.2)
- [ ] 正常速度测试
- [ ] 完整导航测试

**总预计时间**: 8-14小时

---

## 🚀 快速启动方案

### 方案1: 最小可行方案 (2小时)

创建简化的速度控制节点，手动映射：

```python
#!/usr/bin/env python3
"""最简单的速度控制测试节点"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SimpleVelocityController(Node):
    def __init__(self):
        super().__init__('simple_velocity_controller')
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)
        
    def cmd_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z
        self.get_logger().info(f'收到速度指令: v={v:.2f} w={w:.2f}')
        # TODO: 发送到CAN总线或mc_core

def main():
    rclpy.init()
    node = SimpleVelocityController()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

### 方案2: 使用现成的G1代码适配 (4-6小时)

1. 复制G1控制器代码
2. 替换Unitree SDK为本地CAN接口
3. 适配到Jazzy + GMSL

### 方案3: 完整集成mc_core (6-8小时)

1. 编译mc_core
2. 创建ROS2包装器
3. 完整的控制闭环

---

## 🔧 立即可以做的事情

### 1. 安装依赖 (5分钟)

```bash
pip3 install casadi opencv-python
```

### 2. 测试控制器代码 (10分钟)

```bash
cd /home/spirit-ai/Intern_g1
python3 << 'EOF'
# 测试MPC控制器是否可以导入
import sys
sys.path.append('g1_client')

try:
    from controllers import Mpc_controller, PID_controller
    print("✓ 控制器代码可用")
except ImportError as e:
    print(f"✗ 缺少依赖: {e}")
    print("需要安装: pip3 install casadi scipy")

try:
    import casadi
    print(f"✓ CaSADi已安装: {casadi.__version__}")
except ImportError:
    print("✗ CaSADi未安装: pip3 install casadi")
EOF
```

### 3. 检查mc_core状态 (5分钟)

```bash
cd /home/spirit-ai/code/mc_core
cat README.MD | head -50
ls -la mc_core/
```

### 4. 查看CAN总线状态 (5分钟)

```bash
# 检查CAN接口
ip link show | grep can

# 如果没有配置，需要设置
# sudo ip link set can0 type can bitrate 1000000
# sudo ip link set can0 up
```

---

## 💡 推荐方案

### 短期方案（今天内完成）:
1. **安装casadi**: `pip3 install casadi opencv-python`
2. **创建测试节点**: 简单的 `/cmd_vel` 订阅器
3. **手动测试**: 发布速度指令，打印日志

### 中期方案（本周完成）:
1. **了解mc_core**: 阅读文档，理解CAN协议
2. **创建底盘控制节点**: 连接 `/cmd_vel` → CAN
3. **创建里程计节点**: CAN → `/odom`

### 长期方案（完整集成）:
1. **集成MPC控制器**: 复制G1代码
2. **完整导航闭环**: 推理 → 规划 → 控制 → 反馈
3. **性能优化**: 调参、测试、稳定性

---

## 📞 下一步行动

请选择您想要的方案：

1. **快速测试**: 先安装casadi，测试控制器代码能否运行
2. **了解mc_core**: 查看运动控制库，了解底盘接口
3. **创建简化版本**: 先实现基本的速度控制，不考虑闭环
4. **完整集成**: 从头到尾实现完整的导航控制

告诉我您的选择，我可以提供具体的实现代码！

---

**关键结论**: 
- ✅ 硬件完整（CAN总线、传感器）
- ✅ 推理服务正常
- ❌ **缺少casadi** (最关键)
- ❌ **缺少底盘控制节点**
- ❌ **缺少里程计发布节点**

**最快路径**: 安装casadi → 创建简单的cmd_vel订阅器 → 连接到CAN总线

---

**文档维护**: spirit-ai  
**最后更新**: 2026-07-14
