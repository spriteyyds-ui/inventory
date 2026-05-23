# agv_inventory_system 现场操作与调试手册

本手册是 `agv_inventory_system` 的主操作文档，面向现场运行、联调和故障排查。运行命令、调试流程和常见问题统一维护在这里，避免旧接口和重复文档造成误用。

## 1. 适用环境

- Ubuntu 22.04 + ROS2 Humble。
- 工作区默认路径：`/home/wheeltec/wheeltec_ros2`。
- 功能包：`agv_inventory_system`。
- 底盘导航：`wheeltec_nav2` 已可正常启动。
- 现场硬件：C100 左右相机、LiDAR、里程计/定位、升降继电器和盘库相关外设已连接。

建议终端分工：

- T1：底层驱动、Nav2、TF、LiDAR。
- T2：盘库系统 launch。
- T3：service、topic、状态机和参数检查。
- T4：相机和识别图像调试。
- T5：路线点、Nav2 action、定位和实车安全观察。

## 2. 构建与环境刷新

```bash
cd /home/wheeltec/wheeltec_ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select agv_inventory_system
source install/setup.bash
```

接口、launch、参数或安装文件变更后，所有终端都要重新执行：

```bash
source /home/wheeltec/wheeltec_ros2/install/setup.bash
```

如果 CMake 缓存导致接口或安装内容未刷新：

```bash
colcon build --packages-select agv_inventory_system --cmake-clean-cache
source install/setup.bash
```

如果提示找不到 `yaml-cpp`：

```bash
sudo apt update
sudo apt install libyaml-cpp-dev
colcon build --symlink-install --packages-select agv_inventory_system
```

## 3. 启动系统

启动完整盘库系统：

```bash
cd /home/wheeltec/wheeltec_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
RCUTILS_COLORIZED_OUTPUT=0 ros2 launch agv_inventory_system inventory_system.launch.py 2>&1 | tee "$HOME/Desktop/inventory_system_log.txt"
```

完整系统默认启动：

- `wheeltec_nav2`
- 左右 C100 相机节点
- `corridor_follower_node`
- `number_recognizer_node`
- `distance_estimator_node`
- `gap_detector_node`
- `mission_manager_node`
- `inventory_auto_recharger.py`
- `lift_relay_controller.py`
- `inventory_operation_gui.py`

常用 launch 参数：

```bash
ros2 launch agv_inventory_system inventory_system.launch.py launch_nav2:=false
ros2 launch agv_inventory_system inventory_system.launch.py enable_inventory_operation_gui:=false
ros2 launch agv_inventory_system inventory_system.launch.py inventory_params_file:=/path/to/inventory_system.yaml
```

单独启动识别链路：

```bash
ros2 launch agv_inventory_system recognizer.launch.py
```

覆盖 C100 相机设备路径：

```bash
ros2 launch agv_inventory_system inventory_system.launch.py \
  c100_right_video_device:=/dev/v4l/by-path/<right-camera> \
  c100_left_video_device:=/dev/v4l/by-path/<left-camera>
```

## 4. 启动后健康检查

确认节点：

```bash
ros2 node list | grep -E "corridor|recognizer|distance|gap|mission|lift|inventory"
```

确认 service：

```bash
ros2 service list | grep -E "/inventory|/lift"
```

应至少看到：

- `/inventory/start_mission`
- `/inventory/cancel_mission`
- `/lift/up`
- `/lift/down`
- `/lift/home`
- `/lift/stop`
- `/lift/all_off`

确认 topic：

```bash
ros2 topic list | grep -E "/scan|/odom|/odom_combined|/c100|/inventory|/lift"
```

确认 Nav2 action：

```bash
ros2 action list | grep navigate_to_pose
ros2 action info /navigate_to_pose
```

确认 TF 和传感器：

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo base_link laser
ros2 topic echo /scan --once
ros2 topic echo /odom_combined --once
ros2 topic echo /amcl_pose --once
```

确认相机图像：

```bash
ros2 topic info /c100_right/image_raw
ros2 topic info /c100_left/image_raw
ros2 run image_tools showimage --ros-args -r image:=/c100_right/image_raw
```

如果 `showimage` 不存在：

```bash
sudo apt update
sudo apt install ros-humble-image-tools
```

## 5. 当前任务接口

正式任务入口：

- `/inventory/start_mission`，类型：`agv_inventory_system/srv/StartMission`
- `/inventory/cancel_mission`，类型：`std_srvs/srv/Trigger`

查看当前字段：

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
---
bool accepted
string message
```

字段使用原则：

- 单货柜盘库：`run_full_inventory: false`，填写 `target_gap` 和单个 `scan_cabinets`。
- 同侧多柜盘库：`run_full_inventory: false`，`target_gap` 从目标侧首个 gap 开始，`scan_cabinets` 按要扫描的柜号顺序填写。
- 全量盘库：`run_full_inventory: true`，默认序列来自 `config/inventory_system.yaml` 的 `inventory_plan`。
- 保留目标编号模式：可填写 `targets`，格式示例为 `2-3-1-2`，含义通常为仓库号、货柜号、层号、深度格。

## 6. 任务调用

单货柜/单间隙盘库：

```bash
ros2 service call /inventory/start_mission agv_inventory_system/srv/StartMission \
  "{targets: [], return_home: false, run_full_inventory: false, target_gap: 'gap_02_03_04', scan_cabinets: [4]}"
```

全量盘库默认序列：

```bash
ros2 service call /inventory/start_mission agv_inventory_system/srv/StartMission \
  "{targets: [], return_home: true, run_full_inventory: true, target_gap: '', scan_cabinets: []}"
```

全量盘库指定柜号序列：

```bash
ros2 service call /inventory/start_mission agv_inventory_system/srv/StartMission \
  "{targets: [], return_home: true, run_full_inventory: true, target_gap: '', scan_cabinets: [4, 3, 8, 7]}"
```

目标编号方式：

```bash
ros2 service call /inventory/start_mission agv_inventory_system/srv/StartMission \
  "{targets: ['2-3-1-2', '2-16-2-3'], return_home: true, run_full_inventory: false, target_gap: '', scan_cabinets: []}"
```

取消任务：

```bash
ros2 service call /inventory/cancel_mission std_srvs/srv/Trigger "{}"
```

硬停车，只发布一次零速度，不走任务状态机：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## 7. 状态和日志观察

```bash
ros2 topic echo /inventory/mission_state
ros2 topic echo /inventory/mission_log
```

典型流程状态：

- `IDLE`：空闲。
- `NAV_ROUTE`：Nav2 按路线点巡航。
- `TARGET_TRACKING`：识别目标并跟踪接近。
- `SEARCH_GAP`：搜索目标间隙。
- `WAITING_GAP`：确认缝宽、稳定帧和安全距离。
- `ENTERING_GAP`：转向入缝并直行到目标深度。
- `INVENTORYING` / `SINGLE_CABINET_IN_GAP_SCAN` / `FULL_INVENTORY_IN_GAP_SCAN`：缝内扫描。
- `SINGLE_CABINET_EXIT_GAP` / `FULL_INVENTORY_EXIT_GAP`：倒退出缝。
- `DONE`：任务完成。
- `ERROR`：任务失败或安全中止。

调试时同时观察：

```bash
ros2 topic echo /inventory/recognized_number
ros2 topic echo /inventory/gap_status
ros2 topic echo /inventory/target_distance
ros2 topic echo /inventory/entry_side
ros2 topic echo /odom_combined
```

## 8. 数字识别调试

识别相关默认 topic：

- 相机输入：`/c100_right/image_raw`
- 识别结果：`/inventory/recognized_number`
- A4 调试图：`/inventory/debug_a4_region`
- 数字分割调试图：`/inventory/debug_digits`
- 可视化图：`/inventory/visualization`
- 识别使能 topic：`/inventory/recognizer_enable`
- 手动使能 service：`/inventory/trigger_recognition`

查看识别结果：

```bash
ros2 topic echo /inventory/recognized_number
ros2 topic echo /inventory/recognizer_enable
```

查看调试图：

```bash
ros2 run image_tools showimage --ros-args -r image:=/inventory/visualization
ros2 run image_tools showimage --ros-args -r image:=/inventory/debug_a4_region
ros2 run image_tools showimage --ros-args -r image:=/inventory/debug_digits
```

识别失败时按顺序检查：

1. `/c100_right/image_raw` 是否有图像。
2. A4 是否完整进入画面，是否反光、过暗或过曝。
3. `/inventory/debug_a4_region` 中 A4 是否被稳定框住。
4. `/inventory/debug_digits` 中数字是否被正确分割。
5. `confidence` 是否低于阈值。
6. `estimated_distance` 和 `/inventory/target_distance` 是否明显异常。
7. `/inventory/recognizer_enable` 是否在当前状态被关闭。

识别参数只维护在：

```bash
/home/wheeltec/wheeltec_ros2/src/agv_inventory_system/config/inventory_system.yaml
```

修改 YAML 后需要重启 `number_recognizer_node` 或完整系统。单独识别 launch 和完整系统 launch 都加载这份配置。

确认参数是否生效：

```bash
ros2 param get /number_recognizer_node onnx_model_path
ros2 param get /number_recognizer_node digit_input_size
ros2 param get /number_recognizer_node classifier_input_size
```

模型文件说明见 [../models/README.md](../models/README.md)。

## 9. 路线、Nav2 和目标识别联调

路线文件：

```bash
less /home/wheeltec/wheeltec_ros2/src/agv_inventory_system/config/route_waypoints.yaml
```

重点检查：

- `routes.<route_name>.waypoints` 是巡航路线点，不是货柜精确坐标。
- `waypoints` 使用 `map` 坐标系，必须在建图和定位稳定后标定。
- `cabinet_side_map` 中目标货柜号必须能找到 `left` 或 `right`。
- `side_route_map` 中 `left` / `right` 必须能找到真实路线名。
- 路线名必须存在于 `routes` 下。

检查占位坐标：

```bash
grep -n "{x: 0.0, y: 0.0, yaw: 0.0}" \
  /home/wheeltec/wheeltec_ros2/src/agv_inventory_system/config/route_waypoints.yaml
```

如果仍有占位坐标，只能做软件连通测试，不要直接实车自动巡航。

观察 Nav2：

```bash
ros2 action info /navigate_to_pose
ros2 topic echo /amcl_pose
ros2 run tf2_ros tf2_echo map base_link
```

识别到目标后，预期现象：

- mission log 出现目标识别稳定。
- 当前 Nav2 巡航目标被取消。
- 小车短时间发布零速度停车。
- `/inventory/mission_state` 从 `NAV_ROUTE` 进入 `TARGET_TRACKING` 或后续状态。

如果识别有结果但不取消 Nav2：

```bash
ros2 param get /mission_manager_node target_recognition_stable_frames
ros2 param get /mission_manager_node target_recognition_stable_time_sec
ros2 topic echo /inventory/recognized_number
```

## 10. 找缝调试

找缝依赖 LiDAR `/scan` 和 `gap_detector_node`，输出 `/inventory/gap_status`。`mission_manager_node` 会发布：

- `/inventory/gap_context`
- `/inventory/gap_detector_enable`
- `/inventory/entry_side`

查看 gap 状态：

```bash
ros2 topic echo /inventory/gap_status
ros2 topic echo /inventory/entry_side
ros2 topic echo /inventory/gap_detector_enable
```

核心字段：

- `gap_detected`：连续稳定帧达标。
- `entry_width_ok`：估计缝宽满足要求。
- `allow_enter`：综合判定允许入缝。
- `gap_width`：估计缝宽。
- `active_side`：当前检测方向，来自 `/inventory/entry_side`。
- `stable_frame_count`：连续稳定帧计数。

查看参数：

```bash
ros2 param get /gap_detector_node left_gap_sector_start_deg
ros2 param get /gap_detector_node left_gap_sector_end_deg
ros2 param get /gap_detector_node right_gap_sector_start_deg
ros2 param get /gap_detector_node right_gap_sector_end_deg
ros2 param get /gap_detector_node open_point_min_distance
ros2 param get /gap_detector_node required_entry_width
ros2 param get /gap_detector_node stable_frames_required
```

排查顺序：

1. `/scan` 是否有数据。
2. `tf2_echo base_link laser` 是否可用。
3. `/inventory/gap_detector_enable` 是否为 true。
4. `/inventory/entry_side` 是否和实际进入侧一致。
5. `gap_status.active_side` 是否跟随切换。
6. `allow_enter` 为 false 时，优先看 `entry_width_ok`、安全距离和稳定帧。

在线调参示例：

```bash
ros2 param set /gap_detector_node left_gap_sector_start_deg 60.0
ros2 param set /gap_detector_node left_gap_sector_end_deg 120.0
ros2 param set /gap_detector_node right_gap_sector_start_deg -120.0
ros2 param set /gap_detector_node right_gap_sector_end_deg -60.0
ros2 param set /gap_detector_node open_point_min_distance 0.50
ros2 param set /gap_detector_node required_entry_width 0.40
ros2 param set /gap_detector_node stable_frames_required 3
```

## 11. 入缝调试

`ENTERING_GAP` 重点看三类控制：

- 转入角度：`entry_turn_*`
- 入缝直行和 yaw hold：`entry_straight_*`
- 目标侧距离保持：`entry_side_hold_*`

查看参数：

```bash
ros2 param get /mission_manager_node entry_turn_yaw_delta_rad
ros2 param get /mission_manager_node entry_align_yaw_tolerance_rad
ros2 param get /mission_manager_node entry_turn_linear_speed
ros2 param get /mission_manager_node entry_turn_angular_speed
ros2 param get /mission_manager_node entry_straight_speed
ros2 param get /mission_manager_node entry_straight_yaw_kp
ros2 param get /mission_manager_node entry_straight_max_angular_speed
ros2 param get /mission_manager_node entry_side_hold_target_distance_m
ros2 param get /mission_manager_node entry_side_hold_kp
```

调试原则：

- `entry_side=left` 时，入缝转向 `angular.z` 应为正。
- `entry_side=right` 时，入缝转向 `angular.z` 应为负。
- 大转向只应出现在转入阶段，直行阶段只允许小角速度 yaw hold 或侧距修正。
- 阿克曼底盘不要按原地旋转思路调试。
- 安全停车优先级高于侧距保持，不要为了走完距离关闭安全保护。

目标深度格：

```text
target_depth_center_m = (target_depth_index - 0.5) * grid_depth_m
target_straight_distance = target_depth_center_m + entry_center_offset_m
```

标定建议：

1. 先把 `entry_center_offset_m` 设为 `0.0`。
2. 如果所有深度都停车偏浅，增加 `entry_center_offset_m`。
3. 如果所有深度都停车偏深，减小 `entry_center_offset_m`。
4. 每次小量调整，并验证浅格和深格误差是否一致。

## 12. 缝内扫描和升降机构

缝内扫描由 `mission_manager_node` 调用扫描序列和升降机构相关逻辑。常见参数：

- `scan_duration_sec`
- `scan_timeout_sec`
- `lift_motion_duration_sec`
- `lift_service_timeout_sec`
- `grid_motion_enabled`
- `grid_move_timeout_sec`

升降机构服务：

```bash
ros2 service call /lift/up agv_inventory_system/srv/LiftMoveTimed \
  "{direction: 'up', duration_sec: 1.5}"

ros2 service call /lift/down agv_inventory_system/srv/LiftMoveTimed \
  "{direction: 'down', duration_sec: 1.5}"

ros2 service call /lift/home agv_inventory_system/srv/LiftMoveTimed \
  "{direction: 'down', duration_sec: 2.0}"

ros2 service call /lift/stop std_srvs/srv/Trigger "{}"

ros2 service call /lift/all_off std_srvs/srv/Trigger "{}"

ros2 service call /lift/move_to_estimated_height agv_inventory_system/srv/LiftMoveToHeight \
  "{target_height_m: 0.35}"
```

查看升降状态：

```bash
ros2 topic echo /lift/state
ros2 service list | grep /lift
```

如果扫描或升降阶段不结束：

```bash
ros2 topic echo /inventory/mission_log
ros2 topic echo /odom_combined
ros2 param get /mission_manager_node scan_timeout_sec
ros2 param get /mission_manager_node lift_service_timeout_sec
ros2 param get /mission_manager_node grid_motion_enabled
ros2 param get /mission_manager_node grid_move_timeout_sec
```

## 13. 出缝和返航

出缝相关状态：

- `SINGLE_CABINET_EXIT_GAP`
- `FULL_INVENTORY_EXIT_GAP`

常见参数：

- `exit_gap_after_each_scan`
- `exit_gap_mode`
- `exit_gap_speed`
- `exit_gap_extra_distance_m`
- `exit_gap_timeout_sec`
- `exit_gap_distance_m`
- `exit_gap_turn_enabled`
- `exit_gap_turn_angular_speed`
- `exit_gap_turn_timeout_sec`

返航/回充：

```bash
ros2 param get /mission_manager_node return_target_mode
ros2 param set /mission_manager_node return_target_mode start
ros2 param set /mission_manager_node return_target_mode charge
ros2 service call /inventory/cancel_mission std_srvs/srv/Trigger "{}"
```

返航不动时检查：

```bash
ros2 topic echo /inventory/mission_state
ros2 action list | grep navigate_to_pose
ros2 topic echo /odom_combined
ros2 topic echo /odom
```

## 14. CSV 和日志

盘库系统启动日志建议通过 `tee` 保存：

```bash
RCUTILS_COLORIZED_OUTPUT=0 ros2 launch agv_inventory_system inventory_system.launch.py 2>&1 | tee "$HOME/Desktop/inventory_system_log.txt"
```

gap detector CSV 日志默认目录：

```bash
ls -lt /tmp/agv_inventory_gap_logs
tail -f /tmp/agv_inventory_gap_logs/$(ls -t /tmp/agv_inventory_gap_logs | head -n 1)
```

重点看：

- `active_side`
- 扇区角度
- 有效点数和开口点数
- `stable_count`
- `gap_detected`
- `entry_width_ok`
- `allow_enter`
- 当前调整索引和偏移

## 15. 常见问题排查

### 找不到 /inventory/start_mission

- 可能原因：`mission_manager_node` 未启动、launch 失败、参数文件未加载。
- 检查：`ros2 node list`、`ros2 service list | grep inventory`、`ros2 topic echo /inventory/mission_log`。
- 处理：重新启动完整系统，查看终端报错。

### StartMission 字段不匹配

- 现象：service call 报字段不存在或字段类型不匹配。
- 可能原因：当前终端没有 source 新 install，或同时 source 了旧工作区。
- 检查：`ros2 interface show agv_inventory_system/srv/StartMission`。
- 处理：重新编译并在所有终端重新 source 当前工作区。

### Nav2 goal rejected 或 NAV_ROUTE 不动

- 可能原因：Nav2 未就绪、map/odom/TF 不完整、路线点不可达、frame 不一致。
- 检查：`ros2 action info /navigate_to_pose`、`ros2 topic echo /amcl_pose`、`ros2 run tf2_ros tf2_echo map base_link`。
- 处理：先修外部导航和定位，再检查 `config/route_waypoints.yaml`。

### waypoints 仍是 0.0

- 只能做软件连通测试，不要实车自动巡航。
- 使用建图后的 `map` 坐标在 RViz 中重新标定路线点。

### 无法识别目标柜

- 可能原因：相机无图、A4 不完整、数字分割失败、置信度低、识别使能关闭。
- 检查：`/c100_right/image_raw`、`/inventory/recognized_number`、`/inventory/debug_a4_region`、`/inventory/debug_digits`。
- 处理：先看 debug 图像，再调整相机视角、光照和识别参数。

### 找不到缝或 allow_enter 一直为 false

- 可能原因：`/scan` 无数据、`entry_side` 反了、扇区不对、稳定帧要求过高、真实缝宽不足。
- 检查：`ros2 topic echo /scan --once`、`ros2 topic echo /inventory/entry_side`、`ros2 topic echo /inventory/gap_status`。
- 处理：先确认实际开缝和检测侧，再调整 gap detector 参数。

### 入缝方向反了或入缝偏斜

- 可能原因：`entry_side` 错、底盘 `angular.z` 方向不符合预期、雷达左右扇区或 TF 错。
- 检查：`/inventory/entry_side`、`/odom_combined`、`/inventory/mission_log`。
- 处理：先确认雷达坐标系和底盘角速度方向，再调 `entry_turn_*`、`entry_straight_*`、`entry_side_hold_*`。

### 入缝太浅或太深

- 浅格和深格都偏同一方向：调整 `entry_center_offset_m`。
- 只有深格偏差明显：检查里程计累计、地面打滑和最大动态入缝距离。
- 末端越过明显：降低 `entry_straight_speed` 并确认 odom 没有延迟。

### 出缝超时

- 可能原因：`exit_gap_distance_m` 与速度/超时不匹配，odom 未更新，出缝转向 yaw 无效。
- 检查：`ros2 topic echo /odom_combined`、`ros2 topic echo /inventory/mission_log`。
- 处理：确认 odom 正常，再检查 `exit_gap_speed`、`exit_gap_timeout_sec`、`exit_gap_turn_timeout_sec`。

### 扫描或升降杆阶段一直不结束

- 可能原因：扫描、升降或 grid movement 超时参数不合理，真实设备接口没有返回完成。
- 检查：`/inventory/mission_log`、`/lift/state`、`/odom_combined`。
- 处理：检查 `scan_timeout_sec`、`lift_service_timeout_sec`、`grid_motion_enabled`、`grid_move_timeout_sec`。

### TF 不连通

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo base_link laser
```

- `NAV_ROUTE` 依赖 map 定位。
- `gap_detector` 依赖雷达点能转换到 `base_link`。
- 先修 TF，再调盘库流程。

## 16. 关键配置速查

```bash
grep -n "start_service_name\|cancel_service_name\|inventory_plan\|real_motion_target_gap\|scan_duration_sec\|lift_service_timeout_sec\|exit_gap_\|grid_motion_enabled" \
  config/inventory_system.yaml
```

常用配置文件：

- `config/inventory_system.yaml`：主参数文件。
- `config/route_waypoints.yaml`：路线点和货柜侧别。
- `config/gap_scan_map.yaml`：gap 和扫描柜号映射。
- `config/warehouse_layout.yaml`：仓库布局。
