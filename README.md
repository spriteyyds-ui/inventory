# wheeltec_inventory_system

基于 ROS2 Humble 的无人车自动盘库功能包（C++）。

## 当前节点架构

- `corridor_follower_node`
  - 订阅：`/scan`、`/inventory/corridor_enable`、`/inventory/corridor_reverse`
  - 发布：`/cmd_vel`
  - 服务：`/inventory/set_corridor_following` (`std_srvs/srv/SetBool`)
  - 作用：激光雷达走廊中央保持（PID）。

- `number_recognizer_node`
  - 订阅：`/camera/color/image_raw`（可配）
  - 发布：`/inventory/recognized_number`、`/inventory/debug_a4_region`、`/inventory/debug_digits`、`/inventory/visualization`
  - 服务：`/inventory/trigger_recognition` (`std_srvs/srv/SetBool`)
  - 作用：A4 检测 + 数字分割 + ONNX CNN 分类。

- `distance_estimator_node`
  - 订阅：`/scan`、`/inventory/recognized_number`
  - 发布：`/inventory/target_distance`
  - 作用：融合视觉和雷达估算目标距离。

- `gap_detector_node`
  - 订阅：`/scan`
  - 发布：`/inventory/gap_status`
  - 作用：检测左右间隙宽度与可进入状态。

- `mission_manager_node`
  - 订阅：`/inventory/recognized_number`、`/inventory/target_distance`、`/inventory/gap_status`、`/odom`、`/scan`、`/ultrasonic_data_A~F`
  - 发布：`/inventory/mission_state`、`/inventory/mission_log`、`/cmd_vel`、`/inventory/corridor_enable`、`/inventory/corridor_reverse`
  - 服务：`/inventory/start_mission`、`/inventory/cancel_mission`
  - 作用：任务状态机（巡航、跟踪、等缝、入缝、返回、取消回家）。

## 自定义接口

- `msg/RecognizedNumber.msg`
  - `string number`
  - `float32 confidence`
  - `bool valid`
  - `float32 horizontal_offset`
  - `int32 attempts`
  - `float32 estimated_distance`

- `msg/GapStatus.msg`
  - `bool gap_detected`
  - `float32 gap_width`
  - `string side`

- `srv/StartMission.srv`
  - Request: `string[] targets`, `bool return_home`
  - Response: `bool accepted`, `string message`

## 构建

```bash
cd ~/wheeltec_ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select wheeltec_inventory_system
source install/setup.bash
```

## 启动

- 启动完整系统：

```bash
ros2 launch wheeltec_inventory_system inventory_system.launch.py
```

- 仅启动识别节点：

```bash
ros2 launch wheeltec_inventory_system recognizer.launch.py
```

## 任务控制

- 启动任务（单目标）：

```bash
ros2 service call /inventory/start_mission wheeltec_inventory_system/srv/StartMission "{targets: ['2-3-1-2'], return_home: false}"
```

- 启动任务（多目标）：

```bash
ros2 service call /inventory/start_mission wheeltec_inventory_system/srv/StartMission "{targets: ['2-3-1-2', '2-16-2-3'], return_home: true}"
```

- 取消任务（立即停车并返回初始位置）：

```bash
ros2 service call /inventory/cancel_mission std_srvs/srv/Trigger "{}"
```

## 常用调试命令

- 查看任务状态：

```bash
ros2 topic echo /inventory/mission_state
```

- 查看任务日志：

```bash
ros2 topic echo /inventory/mission_log
```

- 查看识别结果：

```bash
ros2 topic echo /inventory/recognized_number
```

- 调出摄像头与识别调试画面：

```bash
ros2 run image_tools showimage --ros-args -r image:=/camera/color/image_raw
ros2 run image_tools showimage --ros-args -r image:=/inventory/visualization
ros2 run image_tools showimage --ros-args -r image:=/inventory/debug_a4_region
ros2 run image_tools showimage --ros-args -r image:=/inventory/debug_digits
```

## 调试手册

完整步骤请看：`docs/inventory_debug_manual.txt`
