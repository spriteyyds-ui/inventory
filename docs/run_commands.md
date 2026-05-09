| 指令 | 含义 |
|---|---|
| `cd /home/wheeltec/wheeltec_ros2` | 进入 ROS2 工作区。 |
| `colcon build --packages-select wheeltec_inventory_system` | 编译 wheeltec_inventory_system。 |
| `source install/setup.bash` | 刷新当前终端环境。 |
| `ros2 interface show wheeltec_inventory_system/srv/StartMission` | 查看 StartMission.srv 字段；service call 字段以该输出为准。 |
| `ros2 service list` | 查看当前 services，确认 inventory 相关入口是否存在。 |
| `ros2 launch wheeltec_inventory_system inventory_system.launch.py` | 启动盘库系统正式 launch。 |
| `ros2 service call /inventory/start_mission wheeltec_inventory_system/srv/StartMission "{targets: [], return_home: false, run_full_inventory: false, target_gap: 'gap_02_03_04', scan_cabinets: [4]}"` | 调用单货柜盘库；字段变更时先执行 ros2 interface show wheeltec_inventory_system/srv/StartMission 后按字段补全。 |
| `ros2 service call /inventory/start_mission wheeltec_inventory_system/srv/StartMission "{targets: [], return_home: true, run_full_inventory: true, target_gap: '', scan_cabinets: []}"` | 调用全部盘库默认序列；字段变更时先执行 ros2 interface show wheeltec_inventory_system/srv/StartMission 后按字段补全。 |
| `ros2 service call /inventory/cancel_mission std_srvs/srv/Trigger "{}"` | 取消当前任务。 |
| `ros2 topic echo /amcl_pose` | 查看 AMCL 位姿。 |
| `ros2 topic echo /odom_combined` | 查看主里程计。 |
| `ros2 run tf2_ros tf2_echo map base_link` | 查看 map 到 base_link 的 TF。 |
| `ros2 topic echo /scan --once` | 查看一帧 LiDAR 数据。 |
| `ros2 topic echo /inventory/recognized_number` | 查看识别结果 topic。 |
| `ros2 run image_tools showimage --ros-args -r image:=/inventory/debug_a4_region` | 查看 A4 区域 debug 图像。 |
| `ros2 run image_tools showimage --ros-args -r image:=/inventory/debug_digits` | 查看数字分割 debug 图像。 |
| `ros2 service type /inventory/start_test_gap_scan` | 检查旧 start_test_gap_scan service 是否仍残留。 |
| `grep -R "TEST_REAL\|OVERALL_TEST" -n src include config launch srv docs` | 检查 TEST_REAL / OVERALL_TEST 字符串是否残留。 |
| `grep -n "start_service_name\|cancel_service_name\|inventory_plan\|real_motion_target_gap\|scan_duration_sec\|lift_motion_duration_sec\|exit_gap_\|grid_motion_enabled" config/inventory_system.yaml` | 查看参数文件关键字段。 |
| `ros2 topic echo /inventory/mission_log` | 查看 mission_manager 运行日志。 |
| `colcon build --packages-select wheeltec_inventory_system --cmake-clean-cache` | 清理 CMake 缓存后重新编译本包。 |
