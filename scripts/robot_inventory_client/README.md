# ROS 小车端盘库 HTTP Bridge

本项目是 ROS 小车端 Python HTTP 服务，用于接收 Java Spring Boot 后端的盘库控制指令，并桥接调用现有 ROS2 `agv_inventory_system` 的 `/inventory/start_mission` 服务。

V0 版本只保留一个业务入口：

```http
POST /robot/control
```

请求体继续兼容旧格式：

```json
{"status":"1"}
```

`status="1"` 的语义是启动整体盘库。整体盘库序列不由网页传入，默认使用 ROS2 `mission_manager_node` 配置中的 `inventory_plan`，当前为 `[4, 3, 8, 7]`。

任务完成后，小车端沿用 `upload_client.py` 向 Java 后端上报：

```json
{"status":"2"}
```

## 部署前必须修改

部署到小车或联调环境前，请先修改 `config.py` 中的 Java 后端真实 IP：

```python
JAVA_STATUS_URL = "http://Java后端IP:8080/api/robot/status"
JAVA_RESULT_URL = "http://Java后端IP:8080/api/rfid/scan-result"
```

例如：

```python
JAVA_STATUS_URL = "http://192.168.1.100:8080/api/robot/status"
JAVA_RESULT_URL = "http://192.168.1.100:8080/api/rfid/scan-result"
```

小车端还需要先 source ROS2 和项目工作空间，确保 Python 可以导入 `rclpy` 和 `agv_inventory_system.srv.StartMission`。

## 文件结构

```text
robot_inventory_client/
├── __init__.py
├── robot_api.py
├── ros_inventory_bridge.py
├── upload_client.py
├── config.py
├── requirements.txt
└── README.md
```

## 安装依赖

进入 `robot_inventory_client` 目录后执行：

```bash
pip install -r requirements.txt
```

也可以手动安装：

```bash
pip install fastapi uvicorn requests pydantic
```

V0 HTTP bridge 本身不直接访问 RFID 串口；扫码、升降杆、导航、入缝和返回流程继续由 ROS2 `agv_inventory_system` 内部节点负责。

## 启动方式

不要使用 `--workers` 多进程模式。当前服务使用进程内运行状态和后台线程，多进程会导致状态不同步。

开发联调时也尽量不要使用 `--reload`，避免 reload 进程带来重复导入、后台线程重启和状态混乱。

方式一：进入 `robot_inventory_client` 目录启动：

```bash
uvicorn robot_api:app --host 0.0.0.0 --port 8000
```

方式二：从上一级目录使用包路径启动：

```bash
uvicorn robot_inventory_client.robot_api:app --host 0.0.0.0 --port 8000
```

## Java 后端调用小车端

小车端接口：

```http
POST /robot/control
```

请求体固定为字符串格式：

```json
{"status":"1"}
```

成功响应：

```json
{
  "code": 200,
  "message": "小车已接收到 status，正在触发整体盘库",
  "status": 1
}
```

如果盘库已经在进行中，再次发送 `{"status":"1"}` 会返回 HTTP 409：

```json
{
  "detail": "盘库已在进行中"
}
```

## 状态流转

状态更新顺序固定为：

```text
收到 status=1
FastAPI 后台线程触发
is_running=True
调用 ROS2 /inventory/start_mission
等待 /inventory/mission_state 进入 DONE
发送 {"status":"2"}
is_running=False
```

如果 ROS2 service 不可用、拒绝任务或任务进入 `ERROR`，程序会记录错误日志，并尽力上报 `{"status":"ERROR","message":"..."}`。

## 回调 Java 后端

完成状态请求体：

```json
{"status":"2"}
```

## 网络联调步骤

1. 在 Java 后端机器上 ping 小车 IP：

```bash
ping 小车IP
```

2. 浏览器访问 FastAPI 文档：

```text
http://小车IP:8000/docs
```

3. 调用小车控制接口。

Linux 或 macOS：

```bash
curl -X POST http://小车IP:8000/robot/control \
  -H "Content-Type: application/json" \
  -d '{"status":"1"}'
```

Windows CMD：

```bash
curl -X POST http://小车IP:8000/robot/control ^
  -H "Content-Type: application/json" ^
  -d "{\"status\":\"1\"}"
```

PowerShell：

```powershell
Invoke-RestMethod -Method Post `
  -Uri "http://小车IP:8000/robot/control" `
  -ContentType "application/json" `
  -Body '{"status":"1"}'
```

## ROS2 验证

小车端启动前确认：

```bash
source /opt/ros/humble/setup.bash
source ~/wheeltec_ros2/install/setup.bash
ros2 service list | grep inventory
ros2 topic echo /inventory/mission_state
```
