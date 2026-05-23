# agv_inventory_system

`agv_inventory_system` 是基于 ROS2 Humble 的无人车自动盘库功能包，用于在仓储通道内完成巡航、货柜/间隙识别、数字识别、任务调度、升降机构控制和回充辅助等现场盘库流程。

本 README 面向现场运行维护人员，重点说明构建、启动、任务调用和常用调试命令。更完整的命令表和调试流程请参考：

- [docs/run_commands.md](docs/run_commands.md)
- [docs/debug_manual.md](docs/debug_manual.md)
- [docs/inventory_debug_manual.txt](docs/inventory_debug_manual.txt)
- [models/README.md](models/README.md)

## 功能概览

| 模块 | 运行节点/脚本 | 作用 |
|---|---|---|
| 走廊跟随 | `corridor_follower_node` | 基于 `/scan` 做走廊居中和速度控制，发布 `/cmd_vel`。 |
| 数字识别 | `number_recognizer_node` | 从 C100 相机图像中检测 A4 区域、分割数字并通过 ONNX CNN 分类。 |
| 距离估计 | `distance_estimator_node` | 融合识别结果和雷达数据，发布目标距离。 |
| 间隙检测 | `gap_detector_node` | 检测左右间隙宽度和可进入状态。 |
| 任务管理 | `mission_manager_node` | 管理盘库任务状态机、任务日志、进出缝和取消任务流程。 |
| 操作界面 | `inventory_operation_gui.py` | 现场操作总控 GUI，默认随完整系统启动。 |
| 自动回充 | `inventory_auto_recharger.py` | 盘库结束或取消后的回充辅助流程。 |
| 升降控制 | `lift_relay_controller.py` | 通过继电器控制升降机构，并提供升降服务接口。 |

## 目录结构

| 路径 | 说明 |
|---|---|
| `src/` | C++ ROS2 节点和核心实现。 |
| `include/agv_inventory_system/` | C++ 头文件。 |
| `scripts/` | 运行时 Python 节点和 GUI。 |
| `config/` | ROS2 参数、路线点、仓库布局和间隙扫描配置。 |
| `launch/` | 完整系统和识别节点启动文件。 |
| `models/` | 运行时模型文件，默认包含 `digit_cnn.onnx`。 |
| `msg/`、`srv/` | 自定义消息和服务接口。 |
| `docs/` | 调试手册、运行命令和现场文档。 |
| `tools/` | 离线工具、维护工具和人工测试工具。 |
| `test/` | 当前 CMake 使用的测试/离线 C++ 源码。 |

## 运行依赖

基础环境：

- Ubuntu / ROS2 Humble 工作区，项目默认放在 `/home/wheeltec/wheeltec_ros2`。
- 已安装并可启动底盘导航包 `wheeltec_nav2`。
- 已连接 C100 左右相机、激光雷达、里程计/定位、升降继电器等现场硬件。

本包声明或使用的主要 ROS 依赖包括：

- `rclcpp`、`rclpy`、`sensor_msgs`、`nav_msgs`、`nav2_msgs`
- `geometry_msgs`、`std_msgs`、`std_srvs`、`tf2_ros`
- `cv_bridge`、`image_transport`、`visualization_msgs`
- `yaml-cpp`、OpenCV、`usb_cam`

## 构建

```bash
cd /home/wheeltec/wheeltec_ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select agv_inventory_system
source install/setup.bash
```

如果 CMake 缓存导致接口或参数未刷新，可清理后重建：

```bash
colcon build --packages-select agv_inventory_system --cmake-clean-cache
source install/setup.bash
```

## 启动

启动完整盘库系统：

```bash
ros2 launch agv_inventory_system inventory_system.launch.py
```

完整系统默认会启动：

- `wheeltec_nav2`
- 左右 C100 相机节点
- 走廊跟随、数字识别、距离估计、间隙检测、任务管理节点
- 自动回充、升降继电器控制、盘库操作 GUI

常用启动参数：

```bash
ros2 launch agv_inventory_system inventory_system.launch.py launch_nav2:=false
ros2 launch agv_inventory_system inventory_system.launch.py enable_inventory_operation_gui:=false
ros2 launch agv_inventory_system inventory_system.launch.py inventory_params_file:=/path/to/inventory_system.yaml
```

单独启动识别链路：

```bash
ros2 launch agv_inventory_system recognizer.launch.py
```

如现场相机 by-path 发生变化，可覆盖默认设备路径：

```bash
ros2 launch agv_inventory_system inventory_system.launch.py \
  c100_right_video_device:=/dev/v4l/by-path/<right-camera> \
  c100_left_video_device:=/dev/v4l/by-path/<left-camera>
```

## 任务控制

任务服务接口：

```bash
ros2 interface show agv_inventory_system/srv/StartMission
```

当前请求字段：

```text
string[] targets
bool return_home
bool run_full_inventory
string target_gap
int32[] scan_cabinets
```

调用单个货柜/间隙盘库：

```bash
ros2 service call /inventory/start_mission agv_inventory_system/srv/StartMission \
  "{targets: [], return_home: false, run_full_inventory: false, target_gap: 'gap_02_03_04', scan_cabinets: [4]}"
```

调用全量盘库默认序列：

```bash
ros2 service call /inventory/start_mission agv_inventory_system/srv/StartMission \
  "{targets: [], return_home: true, run_full_inventory: true, target_gap: '', scan_cabinets: []}"
```

保留目标编号方式调用：

```bash
ros2 service call /inventory/start_mission agv_inventory_system/srv/StartMission \
  "{targets: ['2-3-1-2', '2-16-2-3'], return_home: true, run_full_inventory: false, target_gap: '', scan_cabinets: []}"
```

取消当前任务：

```bash
ros2 service call /inventory/cancel_mission std_srvs/srv/Trigger "{}"
```

## 升降机构接口

按时间控制升降：

```bash
ros2 service call /lift/up agv_inventory_system/srv/LiftMoveTimed \
  "{direction: 'up', duration_sec: 1.5}"
```

常用服务名：

- `/lift/up`
- `/lift/down`
- `/lift/home`
- `/lift/move_timed`
- `/lift/stop`
- `/lift/all_off`

移动到目标高度：

```bash
ros2 service call /lift/move_to_estimated_height agv_inventory_system/srv/LiftMoveToHeight \
  "{target_height_m: 0.35}"
```

具体服务名以现场 `ros2 service list` 输出为准。

## 常用调试命令

确认盘库服务是否存在：

```bash
ros2 service list | grep inventory
```

查看任务状态和日志：

```bash
ros2 topic echo /inventory/mission_state
ros2 topic echo /inventory/mission_log
```

查看识别和间隙结果：

```bash
ros2 topic echo /inventory/recognized_number
ros2 topic echo /inventory/gap_status
ros2 topic echo /inventory/target_distance
```

查看定位、里程计、TF 和雷达：

```bash
ros2 topic echo /amcl_pose
ros2 topic echo /odom_combined
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /scan --once
```

查看相机和识别调试图像：

```bash
ros2 run image_tools showimage --ros-args -r image:=/c100_right/image_raw
ros2 run image_tools showimage --ros-args -r image:=/inventory/visualization
ros2 run image_tools showimage --ros-args -r image:=/inventory/debug_a4_region
ros2 run image_tools showimage --ros-args -r image:=/inventory/debug_digits
```

检查关键参数：

```bash
grep -n "start_service_name\|cancel_service_name\|inventory_plan\|real_motion_target_gap\|scan_duration_sec\|lift_motion_duration_sec\|grid_motion_enabled" config/inventory_system.yaml
```

## 模型和参数

默认数字识别模型为：

```text
models/digit_cnn.onnx
```

模型路径和输入尺寸由 `config/inventory_system.yaml` 中的 `number_recognizer_node` 参数控制。更换模型时，确保 ONNX 输入尺寸与以下参数一致：

- `number_recognizer_node.digit_input_size`
- `number_recognizer_node.classifier_input_size`

## 自定义接口

消息：

- `msg/RecognizedNumber.msg`
- `msg/GapStatus.msg`
- `msg/LiftState.msg`

服务：

- `srv/StartMission.srv`
- `srv/LiftMoveTimed.srv`
- `srv/LiftMoveToHeight.srv`

字段变更后，以 `ros2 interface show` 的输出为准，再同步更新现场调用命令。
