# Unitree G1：InternNav 从 Humble 适配到 Foxy 的 Claude 指令

先在 Claude Code 所在终端执行：

```bash
export G1_PASSWORD='123'
```

然后将以下内容整体交给 Claude Code。

---

请部署并适配当前 InternNav 的 G1 客户端到目标机器人。你拥有远端修改权限，但必须遵守安全约束。

## 目标

- SSH：`unitree@192.168.0.225`
- 密码：从环境变量 `G1_PASSWORD` 读取；不得写入仓库、脚本、日志或最终报告。
- 目标目录：`/home/unitree/Intern_g1`
- 工作站项目：`/home/ubuntu/InternNav`
- 工作站待部署客户端：`/home/ubuntu/InternNav/g1_client`
- 目标机：Ubuntu 20.04、Jetson/Tegra、Python 3.8、ROS 2 Foxy。
- Unitree Foxy overlay：`/home/unitree/unitree_ros2/cyclonedds_ws/install/local_setup.bash`
- 参考稳定版：[yugangli666/JanusVLN_G1](https://github.com/yugangli666/JanusVLN_G1)，固定参考提交 `74605e56eb3c36ca7e686ef221e4b5356f4ec761`。

当前 `~/Intern_g1` 原本按 ROS 2 Humble 写的，现需适配该机器人上的 ROS 2 Foxy。要求新写独立 Foxy 适配脚本，不能仅修改环境变量。

## 严格边界

1. 不得执行真实机器人运动，不得使用 `--init_robot`。
2. 必须为 InternNav 客户端增加 `--dry-run`，验证阶段只允许 dry-run。
3. dry-run 时不得向 `/api/sport/request`、`/cmd_vel` 或任何控制/FSM topic 发布消息，退出时的零速度消息也必须阻断。
4. 不得执行 `git reset`、`git checkout`、`git clean`、`rm -rf`。
5. 不覆盖 `logs/`、模型权重或用户数据；同步禁止使用 `rsync --delete`。
6. 禁止把参考仓库的 JanusVLN `/eval_janus` 协议替换进当前 InternNav。必须保留 InternNav 的 RGB-D `/eval_dual` 请求格式。
7. 密码只能经环境变量传给 `sshpass`，不能出现在命令历史、代码、文件或输出中。

## 第一阶段：只读检查

先通过 SSH 进行只读检查并汇报：

- `/etc/os-release`、`uname -m`、`python3 --version`
- `/opt/ros` 下的 ROS 发行版
- `/opt/ros/foxy/setup.bash` 是否存在
- `~/unitree_ros2/cyclonedds_ws/install/local_setup.bash` 是否存在
- source Foxy + Unitree overlay 后确认：

  ```bash
  ros2 pkg prefix unitree_api
  ros2 pkg prefix unitree_go
  ```

- 现有 `~/Intern_g1/g1_client` 文件
- 现有启动脚本中所有 Humble、Noetic、Foxy 路径
- D455 的 `/dev/video*`
- `ros2 pkg prefix realsense2_camera`
- Python 中 `pyrealsense2`、`cv2` 是否能 import
- 现有 ROS topic，重点确认 `/lf/odommodestate`、`/api/sport/request`
- G1 到 `192.168.0.170:5801` 的路由和 HTTP 可达性。

不要擅自启动、停止或重启工作站模型服务。

## 第二阶段：备份与非破坏性部署

在目标机创建备份目录：

```text
/home/unitree/Intern_g1/backups/foxy_adapt_YYYYmmdd_HHMMSS
```

备份当前 `g1_client` 中即将修改的文件。

然后将工作站的：

```text
/home/ubuntu/InternNav/g1_client/
```

非破坏性同步至：

```text
/home/unitree/Intern_g1/g1_client/
```

要求：

- 使用 `rsync -a --backup --suffix=.before_foxy_adapt` 或等价方式；
- 排除 `logs/`、`__pycache__/`；
- 不得 `--delete`；
- 不覆盖远端日志；
- 部署后对比关键文件。

## 第三阶段：新增独立 Foxy 环境脚本

在目标机新增：

```text
/home/unitree/Intern_g1/g1_client/ros_foxy_env.sh
```

该脚本必须：

1. 可被 source，不自行 exec。
2. 实现 `remove_ros_distribution_paths`：
   - 从 `PATH`、`LD_LIBRARY_PATH`、`PYTHONPATH`、`PKG_CONFIG_PATH` 移除所有 `/opt/ros/*` 路径；
   - 清理 `AMENT_PREFIX_PATH`、`CMAKE_PREFIX_PATH`、`COLCON_PREFIX_PATH`、`COLCON_CURRENT_PREFIX`、`ROS_DISTRO`、`ROS_VERSION`、`ROS_PYTHON_VERSION`、`ROS_PACKAGE_PATH`、`ROS_ROOT`、`ROS_ETC_DIR`、`ROS_MASTER_URI`。
3. 实现 `source_g1_ros_foxy`：
   - 默认 ROS setup：`/opt/ros/foxy/setup.bash`
   - 默认 Unitree overlay：`/home/unitree/unitree_ros2/cyclonedds_ws/install/local_setup.bash`
   - 支持 `ROS_SETUP`、`UNITREE_ROS_SETUP`、`UNITREE_ROS_DOMAIN_ID` 覆盖。
   - 兼容调用方启用了 `set -u`。
   - 成功后设置：

     ```bash
     export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
     export ROS_DOMAIN_ID="${UNITREE_ROS_DOMAIN_ID:-0}"
     ```

   - 检查 `ROS_DISTRO=foxy`；检查 `unitree_api`、`unitree_go`；失败时输出明确错误并返回非零。
4. 不得 source `~/unitree_ros2/setup.sh`，因为它可能混入 Noetic。
5. 不得 source Humble。

可以参考稳定版中的 `g1_client/ros_foxy_env.sh`，但路径必须适配本机。

## 第四阶段：修改启动脚本

修改：

```text
/home/unitree/Intern_g1/g1_client/run_g1_client.sh
/home/unitree/Intern_g1/g1_client/run_d455_camera.sh
```

要求：

- 两个脚本只加载 `ros_foxy_env.sh`、`source_g1_ros_foxy` 和现有 `dds_interface.sh`。
- 不再 source：

  ```text
  /opt/ros/humble/setup.bash
  ~/unitree_ros2/setup.sh
  ~/ros2_ws/install/setup.bash
  ```

- 保留 CycloneDDS 网卡选择逻辑。
- `run_g1_client.sh` 支持 `PYTHON_BIN` 覆盖，默认 `python3`。
- 使用 `set -euo pipefail`，但 source Foxy 环境阶段要正确处理 nounset。

## 第五阶段：适配 D455 RGB-D

当前 InternNav 客户端依赖同步的：

```text
RGB:   /camera/camera/color/image_raw                       rgb8
Depth: /camera/camera/aligned_depth_to_color/image_raw      16UC1
```

并通过 `/eval_dual` 同时上传 `image` 与 `depth` multipart 字段。因此禁止降级为仅 RGB，也禁止发送伪造的全零深度。

更新 `run_d455_camera.sh`：

- 默认设置为 640x480@30；
- 支持 `D455_BACKEND=realsense` 与 `D455_BACKEND=pyrealsense`；
- 如果 Foxy 的 `realsense2_camera` 可用，则启动 RGB、Depth 和对齐深度，并确保输出 topic、编码与 InternNav 默认值一致。

如果 `realsense2_camera` 不可用，但 `pyrealsense2` 可 import，则新增：

```text
/home/unitree/Intern_g1/g1_client/d455_rgbd_publisher.py
```

它必须：

- 使用 `pyrealsense2` 读取 D455 color + depth；
- depth 对齐到 color；
- 发布同步时间戳的 ROS 2 `sensor_msgs/Image`；
- RGB 编码为 `rgb8`；
- 深度编码为 `16UC1`，单位 mm；
- 默认 topic 为：

  ```text
  /camera/camera/color/image_raw
  /camera/camera/aligned_depth_to_color/image_raw
  ```

- 支持以参数或环境变量覆盖设备、分辨率、帧率和 topic。

如果两种 RGB-D 后端都不可用，停止在明确的阻塞状态，列出已验证结果和准确安装/编译命令；不要假设安装成功。

## 第六阶段：为 InternNav 增加安全 dry-run

修改：

```text
/home/unitree/Intern_g1/g1_client/http_internvla_client_g1.py
```

增加：

```bash
--dry-run
```

要求：

- dry-run 仍要执行 RGB-D 相机订阅、RGB/Depth 同步、odom 订阅、向 `/eval_dual` 发 HTTP 请求、模型输出解析和日志记录。
- dry-run 必须阻断所有控制消息：
  - `G1Manager.move()`
  - `G1Manager.send_fsm_command()`
  - `/api/sport/request`
  - `/cmd_vel`
  - shutdown 阶段的零速度 publish。
- 日志中明确写出：

  ```text
  DRY_RUN: computed but not published
  ```

- 不得改动 `/eval_dual` 格式，必须保留 `image`、`depth`、`json`。
- 不得切换为 `/eval_janus`。

## 第七阶段：补充文档

新建或更新：

```text
/home/unitree/Intern_g1/g1_client/README_FOXY.md
```

必须包含：

- Foxy 环境加载方式；
- 可覆盖环境变量；
- CycloneDDS 网卡配置；
- D455 两种后端；
- RGB/Depth topic、编码、深度单位；
- 相机设备节点检查；
- 只进行 dry-run 的完整启动顺序；
- 服务端仍使用 `http://192.168.0.170:5801/eval_dual`；
- Foxy/Humble/Noetic 混用、Unitree package 找不到、topic 无数据、D455 无深度、HTTP 不可达的排查方法；
- 明确写出真实运动不在本次任务范围，移除 `--dry-run` 前需要现场安全授权。

## 第八阶段：验证

必须依次执行并记录实际结果：

1. 对 shell 脚本：

   ```bash
   bash -n ros_foxy_env.sh run_g1_client.sh run_d455_camera.sh
   ```

2. 对新增或修改 Python：

   ```bash
   python3 -m py_compile ...
   ```

3. 新 shell 测试：

   ```bash
   cd ~/Intern_g1/g1_client
   source ros_foxy_env.sh
   source_g1_ros_foxy
   echo "$ROS_DISTRO"
   ros2 pkg prefix unitree_api
   ros2 pkg prefix unitree_go
   ```

4. 确认启动脚本中不存在 Humble 或 Noetic 环境路径。
5. 启动 D455 后确认 RGB 和对齐深度 topic 均存在、至少收到一帧、编码和分辨率正确。
6. 仅在相机、odom、HTTP 都可用时运行不超过 3 步的 dry-run。
7. dry-run 期间证明 `/api/sport/request` 与 `/cmd_vel` 没有发布任何消息。
8. 不得进行真实导航或 `--init_robot`。

## 最终交付格式

最后给出：

1. 修改和新增的文件清单；
2. 远端备份目录；
3. 每项验证命令及真实结果；
4. 实际 D455 backend、topic、DDS 网卡、ROS_DOMAIN_ID；
5. 阻塞项及下一步精确命令；
6. 一条可直接复制的仅 dry-run 命令。

不要只给方案。请实际完成部署、脚本改造、静态验证及安全 dry-run 验证。
