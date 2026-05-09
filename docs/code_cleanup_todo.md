# wheeltec_inventory_system 代码清洗 TODO

## 当前目标

最终只保留两个正式业务流程：
1. 单货柜盘库流程；
2. 全部盘库主流程。

测试流程中的有效真实运动逻辑要并入正式主流程。
测试状态名、测试参数名、测试 service 入口要逐步删除。

## 当前 grep 基线

本文件基于以下命令结果建立：

```bash
git status --short
grep -R "TEST_REAL\|OVERALL_TEST\|test_real\|overall_test\|test_gap_scan\|StartTestGapScan" -n src include config launch srv
grep -R "start_mission\|start_test_gap_scan\|StartMission\|StartTestGapScan\|create_service" -n src include config launch srv
grep -R "inventory_plan\|full_inventory_sequence\|same_side_next_search\|exit_gap\|grid_motion\|post_gap_detect_advance\|recognition_fallback" -n src include config launch srv
```

当前残留主要集中在 `src/mission_manager_node.cpp`，正式参数已经有一部分写入 `config/inventory_system.yaml` 和 `config/test_gap_scan_params.yaml`，但内部变量、状态、兼容入口仍大量保留 `test_real`、`overall_test`、`test_gap_scan` 命名。

## TODO-1：TEST_REAL 真实运动逻辑正式化

目标：
- 将 TEST_REAL 中已验证的单货柜真实运动流程并入正式单货柜盘库流程。
- 保留开缝、导航、识别、找缝、入缝、深度格移动、扫描、出缝、关缝逻辑。
- 删除 TEST_REAL 命名。

需要处理的文件：
- `src/mission_manager_node.cpp:246-310`：真实运动、侧向连续扫描、出缝、格位运动参数声明仍写入 `test_real_*` 成员。
- `src/mission_manager_node.cpp:565-580`：`TEST_REAL_*` 状态枚举。
- `src/mission_manager_node.cpp:803-834`：`TEST_REAL_*` 状态字符串转换。
- `src/mission_manager_node.cpp:994`：`test_real_exit_phase_to_string` 命名需要正式化。
- `src/mission_manager_node.cpp:1065-1103`：`publish_test_real_log`、`fail_test_real_motion` 调用链。
- `src/mission_manager_node.cpp:1130-1136`：真实运动识别状态判断仍绑定 `TEST_REAL_*`。
- `src/mission_manager_node.cpp:1432-1535`：配置加载使用正式参数名，但写入 `test_real_*` 成员。
- `src/mission_manager_node.cpp:1737-1765`：真实运动配置日志仍引用 `test_real_*`。
- `src/mission_manager_node.cpp:2390-2405`：找缝、识别辅助状态判断仍包含 `TEST_REAL_*`。
- `src/mission_manager_node.cpp:2726-2790`：识别后切换等待缝隙/下一缝逻辑仍使用 TEST_REAL 状态名。
- `src/mission_manager_node.cpp:2790-2890`：真实运动识别回调和目标识别日志。
- `src/mission_manager_node.cpp:4009`：`set_test_real_state` 需要改成正式状态切换接口或合并进通用 `set_state`。
- `src/mission_manager_node.cpp:4057-4074`：侧向连续扫描上下文 reset 仍用 `test_real_side_row_*`。
- `src/mission_manager_node.cpp:5183-5298`：单柜/侧向连续扫描请求校验和上下文初始化仍依赖 `StartTestGapScan::Request`。
- `src/mission_manager_node.cpp:5324-6314`：缝内扫描运行时、深度格移动、出缝、重入、侧向连续扫描完成处理。
- `src/mission_manager_node.cpp:6330-6411`：换缝继续、走廊转移、下一缝重入逻辑。
- `src/mission_manager_node.cpp:6511-6530`：真实运动控制停止和失败处理。
- `src/mission_manager_node.cpp:6575-6636`：`/inventory/start_test_gap_scan` 中启动 TEST_REAL 的入口。
- `src/mission_manager_node.cpp:6767-6771`：取消任务时停止 TEST_REAL 的分支。
- `src/mission_manager_node.cpp:6902-7001`：SEARCH_GAP 失败/等待分支进入 TEST_REAL。
- `src/mission_manager_node.cpp:7094-7152`：入缝准备和 TEST_REAL_ENTERING_GAP 切换。
- `src/mission_manager_node.cpp:7671-7690`：入缝完成后进入 TEST_REAL_IN_GAP_SCAN / ADJUSTED / NEXT_GAP_SCAN。
- `src/mission_manager_node.cpp:7878-7985`：TEST_REAL 主状态处理 case。
- `src/mission_manager_node.cpp:8141-8176`：TEST_REAL 导航、跟踪、识别等待、等待缝隙、入缝 case。
- `src/mission_manager_node.cpp:8414-8444`：`test_real_*` 配置成员。
- `src/mission_manager_node.cpp:8491-8525`：`test_real_*` 运行时成员。
- `config/test_gap_scan_params.yaml:15-61`：格位运动、真实运动、侧向连续扫描参数仍放在测试配置文件中。
- `config/inventory_system.yaml:630-631`：已有正式 `grid_motion_duration_sec` / `grid_motion_timeout_sec`，下一轮需统一来源。

完成标准：
- 不再依赖 TEST_REAL_* 状态作为单货柜正式流程；
- test_real_* 参数改成正式参数；
- /inventory/start_mission 可以触发单货柜完整流程；
- 编译通过；
- grep 复查无不必要 TEST_REAL 残留。

## TODO-2：OVERALL_TEST 全部盘库逻辑正式化

目标：
- 将 OVERALL_TEST 中已验证的全部盘库任务队列、同侧继续、换缝继续、最终返回逻辑并入正式全部盘库主流程。
- 删除 OVERALL_TEST 命名。

需要处理的文件：
- `src/mission_manager_node.cpp:312-379`：全部盘库、同侧继续、识别 fallback、post gap advance 参数声明仍写入 `overall_test_*` 成员。
- `src/mission_manager_node.cpp:581-594`：`OVERALL_TEST_*` 状态枚举。
- `src/mission_manager_node.cpp:835-862`：`OVERALL_TEST_*` 状态字符串转换。
- `src/mission_manager_node.cpp:1070-1101`：`publish_overall_test_log` 和整体盘库失败分派。
- `src/mission_manager_node.cpp:1139-1146`：整体盘库识别状态判断仍绑定 `OVERALL_TEST_*`。
- `src/mission_manager_node.cpp:1539-1647`：正式配置参数加载后仍写入 `overall_test_*` 成员。
- `src/mission_manager_node.cpp:1776-1806`：整体盘库配置日志仍引用 `overall_test_*`。
- `src/mission_manager_node.cpp:4077-4097`：整体盘库上下文 reset。
- `src/mission_manager_node.cpp:4114-4437`：路线覆盖、队列校验、启动整体盘库序列、失败处理。
- `src/mission_manager_node.cpp:4451-4527`：路径后识别等待、目标距离对齐启动。
- `src/mission_manager_node.cpp:4527-4561`：整体盘库识别回调。
- `src/mission_manager_node.cpp:4593-4738`：同侧下一个货柜搜索。
- `src/mission_manager_node.cpp:4746-4924`：识别 fallback 子流程。
- `src/mission_manager_node.cpp:4930-5009`：目标距离对齐、整体缝内扫描。
- `src/mission_manager_node.cpp:5027-5169`：换侧返航、完成后返航、推进下一个目标。
- `src/mission_manager_node.cpp:5533-5534`：整体盘库缝内 MOVE_TO_GRID 仍依赖 `test_real_grid_motion_enabled_`。
- `src/mission_manager_node.cpp:5844-5891`：出缝完成后推进整体盘库或失败处理。
- `src/mission_manager_node.cpp:6560-6571`：`/inventory/start_test_gap_scan` 中启动 OVERALL_TEST 的入口。
- `src/mission_manager_node.cpp:6764-6765`：取消任务时停止 OVERALL_TEST 的分支。
- `src/mission_manager_node.cpp:6900-6997`：SEARCH_GAP 分支进入 OVERALL_TEST。
- `src/mission_manager_node.cpp:7146-7248`：入缝准备和 post gap advance 启动。
- `src/mission_manager_node.cpp:7254-7321`：post gap advance 状态处理。
- `src/mission_manager_node.cpp:7433-7674`：缝隙检测失败和入缝完成进入 OVERALL_TEST_IN_GAP_SCAN。
- `src/mission_manager_node.cpp:8079-8136`：OVERALL_TEST 主状态处理 case。
- `src/mission_manager_node.cpp:8445-8474`：`overall_test_*` 配置成员。
- `src/mission_manager_node.cpp:8526-8542`：`overall_test_*` 运行时成员。
- `config/test_gap_scan_params.yaml:61`：`inventory_plan` 仍在测试配置文件中。
- `config/test_gap_scan_params.yaml:73`：`same_side_next_search_enabled` 仍在测试配置文件中。
- `config/test_gap_scan_params.yaml:111-122`：`recognition_fallback_*`、`post_gap_detect_advance_*` 仍在测试配置文件中。

完成标准：
- 不再依赖 OVERALL_TEST_* 状态作为全部盘库正式流程；
- overall_test_* 参数改成正式参数；
- 全部盘库任务使用正式 inventory_plan / full_inventory_sequence；
- 编译通过；
- grep 复查无不必要 OVERALL_TEST 残留。

## TODO-3：正式入口统一与测试入口退场

目标：
- /inventory/start_mission 成为唯一正式任务入口；
- /inventory/start_test_gap_scan 不再作为业务入口；
- 如果必须保留兼容入口，需要明确标注 deprecated，并且只转发到正式流程，不保留独立测试状态机。

需要处理的文件：
- `src/mission_manager_node.cpp:40-41`：同时 include `StartMission` 和 `StartTestGapScan`。
- `src/mission_manager_node.cpp:81-86`：`start_service_name` 与 `start_test_gap_scan_service_name` 参数声明。
- `src/mission_manager_node.cpp:483-505`：同时创建 `/inventory/start_mission` 和 `/inventory/start_test_gap_scan` service。
- `src/mission_manager_node.cpp:4216`、`4229`、`5184`、`5239`、`5254`、`6415`：多个 helper 仍以 `StartTestGapScan::Request` 为输入类型。
- `src/mission_manager_node.cpp:6414-6458`：`build_test_gap_scan_plans` 构建兼容入口任务。
- `src/mission_manager_node.cpp:6492-6507`：`current_test_gap_scan_plan` / `fail_test_gap_scan`。
- `src/mission_manager_node.cpp:6533-6654`：`start_test_gap_scan_callback` 与 `start_mission_callback` 并存。
- `src/mission_manager_node.cpp:6649`：兼容入口提示仍直接暴露 `/inventory/start_test_gap_scan`。
- `src/mission_manager_node.cpp:6763-6771`：取消逻辑仍区分 `test_gap_scan_active_`。
- `src/mission_manager_node.cpp:7785-7869`：`handle_test_gap_scan_state` 仍作为兼容状态机外壳。
- `src/mission_manager_node.cpp:8050-8075`：timer 仍分派 `test_gap_scan_active_` 和 TEST_REAL 兼容状态。
- `src/mission_manager_node.cpp:8258-8288`：service 名、测试配置文件、测试计划、测试映射成员。
- `src/mission_manager_node.cpp:8486-8489`：`test_gap_scan_active_`、队列和错误成员。
- `src/mission_manager_node.cpp:8663-8665`：`StartMission` 与 `StartTestGapScan` service 成员。
- `CMakeLists.txt:40-41`：仍生成 `srv/StartMission.srv` 和 `srv/StartTestGapScan.srv`。
- `srv/StartTestGapScan.srv`：兼容 service 文件仍存在。
- `config/test_gap_scan_params.yaml:133`：`test_inventory_plan` 仍是测试命名的计划配置。

完成标准：
- 单货柜任务和全部盘库任务都通过 /inventory/start_mission 进入；
- StartTestGapScan 不再承载主流程；
- test_gap_scan_params.yaml 中正式参数已迁移；
- 编译通过；
- grep 复查 test_gap_scan / StartTestGapScan 残留只剩兼容或已删除。

## 每轮执行规则

每次只执行一个 TODO。
执行前必须 grep 定位。
执行后必须：
1. colcon build --packages-select wheeltec_inventory_system
2. grep 复查旧命名
3. 更新 docs/code_cleanup_todo.md 中该 TODO 的完成情况
4. 输出修改文件、删除内容、保留逻辑、编译结果、残留问题
