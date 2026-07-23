# InternNav 本地运动控制模块快照

本目录是从 `/workspace/nav_ws` 中复制出来的运动控制相关内容，用于在当前项目内保存和迁移导航运动控制链路。原导航模块 `/workspace/nav_ws` 未被修改。

## 复制范围

- `control_bridge/`
  - 来源：`/workspace/nav_ws/src/control_bridge`
  - 作用：将 Nav2 输出的 `/cmd_vel` 转成底盘控制话题 `/mx_base_vel_command`。
  - 关键文件：
    - `src/message_forward.cpp`
    - `include/common.hpp`

- `navigation_interfaces/`
  - 来源：`/workspace/nav_ws/src/navigation_interfaces`
  - 作用：提供导航任务服务定义。
  - 关键文件：
    - `srv/RobotNavTaskService.srv`
    - `srv/VLNService.srv`

- `nav2_simple_commander/`
  - 来源：`/workspace/nav_ws/src/navigation2/nav2_simple_commander`
  - 作用：提供 `nav_executor` 和 `BasicNavigator` 封装。
  - 关键文件：
    - `nav2_simple_commander/nav_executor.py`
    - `nav2_simple_commander/robot_navigator.py`

- `nav2_bringup/`
  - 来源：`/workspace/nav_ws/src/navigation2/nav2_bringup`
  - 作用：保存 Spirit/Glim/Nav2 启动、参数和图配置。
  - 关键文件：
    - `launch/bringup_spirit_glim_launch.py`
    - `launch/navigation_launch.py`
    - `launch/glim_launch.py`
    - `params/spirit_params.yaml`
    - `graphs/spirit_graph.geojson`

## 当前确认的运动控制链路

测试确认过的速度链路如下：

```text
/cmd_vel_nav
  -> velocity_smoother
  -> /cmd_vel_smoothed
  -> collision_monitor
  -> /cmd_vel
  -> control_bridge/message_forward
  -> /mx_base_vel_command
  -> MCControlNode
```

`message_forward.cpp` 中的坐标映射为：

```text
mx_base_vel_command.x = cmd_vel.linear.y
mx_base_vel_command.y = -cmd_vel.linear.x
mx_base_vel_command.z = cmd_vel.angular.z
```

并且 `message_forward` 会订阅：

- `/cmd_vel`
- `/emergency_stop`
- `/robot_nav_stop_task`

当收到急停或停止导航任务信号后，会在短时间内输出零速度。

## 已做过的运动控制验证

通过 `/cmd_vel_nav` 发布短时小角速度：

```bash
ros2 topic pub --rate 10 --times 8 /cmd_vel_nav geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.12}}"
```

随后发布零速度：

```bash
ros2 topic pub --rate 10 --times 8 /cmd_vel_nav geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

观测结果：

- `/cmd_vel` 收到 `angular.z = 0.12`
- `/mx_base_vel_command` 收到 `z = 0.12`
- 停止命令后两个话题均回到 `0.0`

## 使用建议

如果要基于当前项目继续集成运动控制，优先从这些入口开始：

1. 速度桥接：`control_bridge/src/message_forward.cpp`
2. 导航任务服务：`nav2_simple_commander/nav2_simple_commander/nav_executor.py`
3. Nav2 启动链路：`nav2_bringup/launch/bringup_spirit_glim_launch.py`
4. 控制参数：`nav2_bringup/params/spirit_params.yaml`

## 本地 Overlay

构建并加载本项目副本：

```bash
cd /workspace/Intern_g1
nav/build_local_nav.sh
source nav/setup_local_nav.sh
```

构建脚本先加载 `/workspace/nav_ws/install/setup.bash` 提供 GLIM、地图定位和其他外部依赖，再把四个本地包安装到 `nav/install`。`ros2 pkg prefix` 验证会确保运动控制相关包来自本项目 overlay。

InternNav dry-run 的统一入口：

```bash
scripts/realworld/start_internnav_nav.sh
scripts/realworld/stop_internnav_nav.sh
```

统一启动使用 `start_nav_executor:=false`，直接面向 `/follow_path` 与 `/spin` action；当前 dummy depth 模式只生成预览，不发送 goal。预检发现旧 `/vln_node`、重复控制节点、相机/里程计/TF 异常时会拒绝启动或进入 HOLD。
