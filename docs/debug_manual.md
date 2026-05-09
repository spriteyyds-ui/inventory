# wheeltec_inventory_system 调试手册

## 1. 当前正式入口

当前正式任务入口是 `/inventory/start_mission`，由 `mission_manager_node` 创建，service 类型是 `wheeltec_inventory_system/srv/StartMission`。取消入口是 `/inventory/cancel_mission`，service 类型是 `std_srvs/srv/Trigger`。

`/inventory/start_test_gap_scan` 已退场，不再作为运行入口使用。当前代码、配置、launch、srv 中没有正式创建该 service；如果还能看到它，优先检查旧终端、旧 install 环境或旧文档命令。

`StartMission.srv` 当前字段：

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

字段用法以 `ros2 interface show wheeltec_inventory_system/srv/StartMission` 输出为准。按当前代码逻辑：

- 单货柜盘库：`run_full_inventory: false`，填写 `target_gap` 和单个 `scan_cabinets`。当前配置为 `real_motion_target_gap: gap_02_03_04`、`real_motion_target_cabinet: 4`。
- 侧排盘库：`run_full_inventory: false`，`target_gap` 从 `side_row_first_gap` 开始，`scan_cabinets` 使用当前配置的侧排序列，例如 `[4, 3]` 或 `[4, 3, 2, 1]`。
- 多柜或全部盘库：`run_full_inventory: true` 会进入全部盘库；如果 `target_gap` 为空且 `scan_cabinets` 或 `targets` 多于 1 个，也会按全部盘库分流。
- 全部盘库默认序列来自 `config/inventory_system.yaml` 的 `inventory_plan`，当前为 `[4, 3, 8, 7]`。

## 2. 编译与环境刷新

在工作区根目录编译：

```bash
cd /home/wheeltec/wheeltec_ros2
colcon build --packages-select wheeltec_inventory_system
```

编译完成后刷新环境：

```bash
source install/setup.bash
```

修改 `srv` 后必须重新编译并重新 `source install/setup.bash`。新终端也必须重新 source，否则可能继续使用旧接口定义。

## 3. 接口检查

查看 StartMission 字段：

```bash
ros2 interface show wheeltec_inventory_system/srv/StartMission
```

查看 inventory service：

```bash
ros2 service list | grep inventory
```

应看到 `/inventory/start_mission` 和 `/inventory/cancel_mission`。确认旧入口不存在：

```bash
ros2 service type /inventory/start_test_gap_scan
```

如果该命令仍返回类型，说明当前运行环境仍有旧节点或旧安装产物，需要停止旧进程、重新 source，并确认 launch 未启动旧入口。

## 4. 单货柜盘库调试流程

1. 启动系统：

```bash
ros2 launch wheeltec_inventory_system inventory_system.launch.py
```

当前 launch 会启动 `corridor_follower_node`、`c100_right_camera`、`number_recognizer_node`、`distance_estimator_node`、`gap_detector_node`、`mission_manager_node`。

2. 查看 service：

```bash
ros2 service list | grep inventory
```

3. 调用 `/inventory/start_mission`。字段如果后续变化，以 `ros2 interface show` 输出为准：

```bash
ros2 service call /inventory/start_mission wheeltec_inventory_system/srv/StartMission "{targets: [], return_home: false, run_full_inventory: false, target_gap: 'gap_02_03_04', scan_cabinets: [4]}"
```

4. 观察状态和日志：

```bash
ros2 topic echo /inventory/mission_state
ros2 topic echo /inventory/mission_log
```

5. 按顺序检查识别、找缝、入缝、扫描、升降杆、出缝：

```bash
ros2 topic echo /inventory/recognized_number
ros2 topic echo /inventory/gap_status
ros2 topic echo /inventory/target_distance
ros2 topic echo /odom_combined
ros2 topic echo /scan --once
```

## 5. 全部盘库调试流程

全部盘库通过 `run_full_inventory` 启动。默认目标序列来自 `config/inventory_system.yaml`：

- `full_inventory_enabled: true`
- `inventory_plan: [4, 3, 8, 7]`
- `inventory_left_route: left_route`
- `inventory_right_route: right_route`
- `return_home_between_sides: true`
- `return_home_after_full_inventory: true`

调用默认全部盘库：

```bash
ros2 service call /inventory/start_mission wheeltec_inventory_system/srv/StartMission "{targets: [], return_home: true, run_full_inventory: true, target_gap: '', scan_cabinets: []}"
```

调用指定序列：

```bash
ros2 service call /inventory/start_mission wheeltec_inventory_system/srv/StartMission "{targets: [], return_home: true, run_full_inventory: true, target_gap: '', scan_cabinets: [4, 3, 8, 7]}"
```

运行时观察当前目标、当前 gap、当前 `scan_cabinets`：

```bash
ros2 topic echo /inventory/mission_log
ros2 topic echo /inventory/mission_state
ros2 topic echo /inventory/gap_status
```

gap 到扫描柜号的映射在 `config/gap_scan_map.yaml`，当前包括 `gap_01_02_03`、`gap_02_03_04`、`gap_05_06_07`、`gap_06_07_08`。

## 6. 状态机调试

当前状态枚举在 `src/mission_manager_node.cpp` 中，正式流程会根据任务类型进入通用状态、单柜状态或全部盘库状态。

- `IDLE`：空闲，等待任务。
- `REQUEST_OPEN_GAP`：请求打开目标 gap。
- `WAIT_OPEN_READY`：等待外部开缝完成。
- `NAV_ROUTE`：Nav2 按路线巡航。
- `TARGET_TRACKING`：识别目标货柜并跟踪接近。
- `SEARCH_GAP`：沿指定方向搜索可进入 gap。
- `WAITING_GAP`：确认 gap 宽度、稳定帧和安全距离。
- `ENTERING_GAP`：转向、直行入缝，并做 yaw hold、侧距保持和安全停止。
- `INVENTORYING` / `SINGLE_CABINET_IN_GAP_SCAN` / `FULL_INVENTORY_IN_GAP_SCAN`：执行缝内扫描流程。
- `SINGLE_CABINET_EXIT_GAP` / `FULL_INVENTORY_EXIT_GAP`：倒退出缝。
- `REQUEST_CLOSE_GAP`：请求关闭当前 gap。
- `WAIT_CLOSE_DONE`：等待关缝完成。
- `FULL_INVENTORY_SAME_SIDE_NEXT_SEARCH`：同侧下一目标搜索。
- `FULL_INVENTORY_RETURN_HOME_BETWEEN_SIDES`：跨侧前返回起点或指定返回点。
- `DONE`：任务完成。
- `ERROR`：任务失败或安全中止。

## 7. 识别调试

当前识别相关 topic 和 service 来自 `config/inventory_system.yaml`：

- 相机输入：`/c100_right/image_raw`
- 识别结果：`/inventory/recognized_number`
- A4 调试图：`/inventory/debug_a4_region`
- 数字分割调试图：`/inventory/debug_digits`
- 可视化图：`/inventory/visualization`
- 识别使能 topic：`/inventory/recognizer_enable`
- 手动使能 service：`/inventory/trigger_recognition`
- 距离叠加输入：`/inventory/target_distance`

常用命令：

```bash
ros2 topic echo /inventory/recognized_number
ros2 topic echo /inventory/recognizer_enable
ros2 run image_tools showimage --ros-args -r image:=/inventory/debug_a4_region
ros2 run image_tools showimage --ros-args -r image:=/inventory/debug_digits
```

识别失败时按顺序检查：

1. 相机 topic 是否有图像。
2. 光照是否导致 A4 反光、过暗或过曝。
3. A4 区域是否完整进入画面。
4. `/inventory/debug_digits` 中数字是否被正确分割。
5. `confidence` 是否低于阈值。
6. `estimated_distance` 和 `/inventory/target_distance` 是否明显异常。
7. `recognizer_enable` 是否在当前状态被关闭。

## 8. 找缝与入缝调试

找缝依赖 LiDAR `/scan` 和 `gap_detector_node`，输出 topic 是 `/inventory/gap_status`。`mission_manager_node` 会发布：

- `/inventory/gap_context`
- `/inventory/gap_detector_enable`
- `/inventory/entry_side`

关键状态：

- `SEARCH_GAP`：沿路线配置的方向寻找候选 gap。
- `WAITING_GAP`：检查 `allow_enter`、`entry_width_ok`、稳定帧、前方与侧方安全。
- `ENTERING_GAP`：入缝转向和直行，使用 yaw hold、侧向距离保持、雷达安全检查。

排查顺序：

1. `/scan` 是否有数据。
2. `tf2_echo base_link laser` 是否可用。
3. `/inventory/gap_detector_enable` 是否为 true。
4. `/inventory/entry_side` 是否和实际进入侧一致。
5. `/inventory/gap_status` 中 `allow_enter`、`gap_width`、`stable_frame_count`、`active_side` 是否合理。
6. 入缝偏斜时检查 `entry_turn_*`、`entry_straight_*`、`entry_side_hold_*` 参数。
7. 安全停止时先确认前方和侧向雷达扇区，不要直接放宽安全阈值。

## 9. 缝内扫描与升降杆调试

扫描接口和升降杆接口不能删除。主流程通过项目中的正式接口调用扫描与升降杆逻辑：

- `InventoryScanner`
- `LiftController`
- `scan_sequence_generator`
- `scan_sequence_executor`

当前内部完成条件可能是按时间推进，例如：

- `scan_duration_sec`
- `scan_timeout_sec`
- `lift_motion_duration_sec`
- `lift_motion_timeout_sec`
- `grid_motion_duration_sec`
- `grid_motion_timeout_sec`

外部接口必须保持最终真实系统设计，后续接真实扫描设备和升降杆时仍应通过清晰接口完成：开始扫描、移动升降杆、触发扫描、报告结果、完成货柜盘库。正式命名不要使用 `sim`、`mock`、`placeholder`、`test`。

## 10. 出缝与继续盘库调试

出缝状态包括 `SINGLE_CABINET_EXIT_GAP`、`SINGLE_CABINET_FINAL_EXIT_GAP`、`FULL_INVENTORY_EXIT_GAP`。当前出缝参数在 `config/inventory_system.yaml`：

- `exit_gap_after_each_scan`
- `exit_gap_mode`
- `exit_gap_speed`
- `exit_gap_extra_distance_m`
- `exit_gap_timeout_sec`
- `exit_gap_distance_m`
- `exit_gap_turn_enabled`
- `exit_gap_turn_angular_speed`
- `exit_gap_turn_yaw_tolerance_rad`
- `exit_gap_turn_timeout_sec`

全部盘库继续执行时还会使用：

- `same_side_next_search_enabled`
- `same_side_search_speed`
- `same_side_search_timeout_sec`
- `same_side_pose_hold_enabled`
- `return_home_between_sides`
- `return_home_after_full_inventory`

同侧下一目标会进入 `FULL_INVENTORY_SAME_SIDE_NEXT_SEARCH`；跨侧可能进入 `FULL_INVENTORY_RETURN_HOME_BETWEEN_SIDES`；最后一个目标完成后进入 `FULL_INVENTORY_COMPLETE`，再根据配置返回或结束。

## 11. 常见问题排查

### service 类型不更新

- 现象：`ros2 service call` 报字段不存在或字段类型不匹配。
- 可能原因：修改过 `srv` 后未重新编译或当前终端未重新 source。
- 检查命令：`ros2 interface show wheeltec_inventory_system/srv/StartMission`
- 处理建议：重新执行 `colcon build --packages-select wheeltec_inventory_system`，然后在所有终端执行 `source install/setup.bash`。

### 找不到 /inventory/start_mission

- 现象：`ros2 service list` 中没有 `/inventory/start_mission`。
- 可能原因：`mission_manager_node` 未启动、launch 失败、参数文件未加载。
- 检查命令：`ros2 node list`、`ros2 service list | grep inventory`、`ros2 topic echo /inventory/mission_log`
- 处理建议：重新启动 `ros2 launch wheeltec_inventory_system inventory_system.launch.py`，检查终端启动错误。

### 还能看到 /inventory/start_test_gap_scan

- 现象：旧入口仍存在。
- 可能原因：旧节点未退出、旧 install 环境仍被 source、旧命令仍在运行。
- 检查命令：`ros2 service type /inventory/start_test_gap_scan`
- 处理建议：停止相关旧进程，重新 source 当前工作区，确认本包中没有启动该 service。

### Goal was rejected

- 现象：Nav2 目标被拒绝。
- 可能原因：Nav2 未就绪、map/odom/tf 不完整、路线点不在可达区域、目标 frame 不一致。
- 检查命令：`ros2 topic echo /amcl_pose`、`ros2 run tf2_ros tf2_echo map base_link`、`ros2 topic echo /inventory/mission_log`
- 处理建议：先确认定位稳定，再检查 `config/route_waypoints.yaml` 中路线点和 frame。

### 无法识别目标柜

- 现象：一直停留在目标识别或跟踪阶段。
- 可能原因：相机无图、A4 未检测到、数字分割失败、置信度低、识别使能关闭。
- 检查命令：`ros2 topic echo /inventory/recognized_number`、`ros2 topic echo /inventory/recognizer_enable`
- 处理建议：先看 debug 图像，再调整相机视角、光照和识别参数。

### 找不到缝

- 现象：停留在 `SEARCH_GAP` 或 `WAITING_GAP`，`allow_enter` 一直为 false。
- 可能原因：`/scan` 无数据、`entry_side` 反了、gap 检测扇区不对、稳定帧要求过高、真实缝宽不足。
- 检查命令：`ros2 topic echo /scan --once`、`ros2 topic echo /inventory/entry_side`、`ros2 topic echo /inventory/gap_status`
- 处理建议：先确认实际开缝和检测侧，再调整 gap detector 参数。

### 入缝偏斜

- 现象：进入 gap 时角度或侧距明显偏离。
- 可能原因：odom yaw 不准、入缝角速度方向不对、侧距保持目标或雷达扇区不匹配。
- 检查命令：`ros2 topic echo /odom_combined`、`ros2 topic echo /inventory/mission_log`
- 处理建议：检查 `entry_turn_*`、`entry_straight_yaw_kp`、`entry_side_hold_target_distance_m` 和左右侧雷达扇区。

### 出缝超时

- 现象：停在 `SINGLE_CABINET_EXIT_GAP` 或 `FULL_INVENTORY_EXIT_GAP`。
- 可能原因：`exit_gap_distance_m` 与速度/超时不匹配，odom 未更新，出缝转向 yaw 无效。
- 检查命令：`ros2 topic echo /odom_combined`、`ros2 topic echo /inventory/mission_log`
- 处理建议：确认 odom 正常，再检查 `exit_gap_speed`、`exit_gap_timeout_sec`、`exit_gap_turn_timeout_sec`。

### 扫描或升降杆阶段一直不结束

- 现象：停在缝内扫描状态。
- 可能原因：扫描或升降杆超时参数不合理，后续真实设备接口未返回完成，grid movement 无有效 odom。
- 检查命令：`ros2 topic echo /inventory/mission_log`、`ros2 topic echo /odom_combined`
- 处理建议：检查 `scan_timeout_sec`、`lift_motion_timeout_sec`、`grid_motion_enabled`、`grid_move_timeout_sec`。

### 修改 srv 后编译通过但 service call 报错

- 现象：编译成功，但调用字段仍按旧格式解析。
- 可能原因：调用终端未 source 新 install，或同时 source 了旧工作区。
- 检查命令：`ros2 interface show wheeltec_inventory_system/srv/StartMission`
- 处理建议：关闭旧终端或重新 source 当前 `/home/wheeltec/wheeltec_ros2/install/setup.bash`，再重新调用。
