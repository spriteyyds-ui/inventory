# Copyright (c) 2026 郁有冬 <spriteyyds@gmail.com>. All rights reserved.
_COPYRIGHT = (
    "========================================\n"
    " agv_inventory_system\n"
    " Copyright (c) 2026 郁有冬 <spriteyyds@gmail.com>\n"
    " All rights reserved.\n"
    "========================================\n"
)
"""FastAPI entrypoint for receiving Java backend robot control commands."""

import threading
from typing import Optional

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel, StrictStr

try:
    from .config import (
        FASTAPI_HOST,
        FASTAPI_PORT,
        SCAN_RESULT_FILE,
        TEST_SCAN_RESULT,
        USE_TEST_SCAN_RESULT,
    )
    from .ros_inventory_bridge import (
        STAGE_WAITING_DEPART_CHARGER,
        STAGE_WAITING_RECHARGE_CANCEL,
        RosInventoryBridgeError,
        run_full_inventory_until_done,
    )
    from .status_store import (
        build_status_response,
        record_receive_status,
        record_upload_status,
    )
    from .upload_client import (
        load_scan_result_file,
        send_error_status,
        send_finish_status,
        send_scan_result,
    )
except ImportError:
    from config import (  # type: ignore
        FASTAPI_HOST,
        FASTAPI_PORT,
        SCAN_RESULT_FILE,
        TEST_SCAN_RESULT,
        USE_TEST_SCAN_RESULT,
    )
    from ros_inventory_bridge import (  # type: ignore
        STAGE_WAITING_DEPART_CHARGER,
        STAGE_WAITING_RECHARGE_CANCEL,
        RosInventoryBridgeError,
        run_full_inventory_until_done,
    )
    from status_store import (  # type: ignore
        build_status_response,
        record_receive_status,
        record_upload_status,
    )
    from upload_client import (  # type: ignore
        load_scan_result_file,
        send_error_status,
        send_finish_status,
        send_scan_result,
    )


app = FastAPI(title="Robot Inventory Client")

is_running = False

_state_lock = threading.Lock()
_worker_thread: Optional[threading.Thread] = None
_job_stage = "IDLE"
_job_message = ""

_RECHARGE_WAITING_STAGES = {
    STAGE_WAITING_RECHARGE_CANCEL,
    STAGE_WAITING_DEPART_CHARGER,
}


class ControlRequest(BaseModel):
    status: StrictStr


def _set_running_state(running: bool) -> None:
    global is_running, _job_stage, _job_message
    with _state_lock:
        is_running = running
        if not running:
            _job_stage = "IDLE"
            _job_message = ""


def _set_job_stage(stage: str, message: str) -> None:
    global _job_stage, _job_message
    with _state_lock:
        _job_stage = stage
        _job_message = message


def _record_ros_progress(stage: str, message: str) -> None:
    _set_job_stage(stage, message)
    if stage in _RECHARGE_WAITING_STAGES:
        record_receive_status(True, "正在停止充电并准备启动盘库")
    elif stage == "STARTING_MISSION":
        record_receive_status(True, "停止充电并离桩完成，正在调用 ROS2 start_mission")
    elif stage == "RUNNING_MISSION":
        record_receive_status(True, f"ROS2 start_mission 成功：{message}")


def _record_exception_upload(reason: str) -> None:
    uploaded = send_error_status(reason)
    if uploaded:
        record_upload_status(
            True,
            f"Java 异常记录保存成功: {reason}",
            source="robot_api_error",
        )
    else:
        record_upload_status(
            False,
            f"Java 异常记录保存失败: {reason}",
            source="robot_api_error",
            exception=reason,
        )


def _classify_ros_failure(exc: Exception) -> str:
    reason = str(exc)
    category = getattr(exc, "category", "")
    if category == "START_MISSION_REJECTED":
        return f"ROS2 start_mission 被拒绝: {reason}"
    if category == "CHARGING_FLAG_CLEAR_TIMEOUT":
        return f"等待 robot_charging_flag=false 超时: {reason}"
    if str(category).startswith("STOP_AUTO_CHARGE_DEPART"):
        return f"停止充电并离桩失败: {reason}"
    if category:
        return f"ROS2 盘库启动/执行失败[{category}]: {reason}"
    return f"整体盘库触发或执行失败: {reason}"


def _run_full_inventory_job() -> None:
    """Trigger ROS2 full inventory and report completion to Java backend."""
    try:
        print("收到 status=1，开始触发 ROS2 整体盘库")
        result = run_full_inventory_until_done(progress_callback=_record_ros_progress)
        print(
            "ROS2 整体盘库完成："
            f"prestart_message={result.prestart_message}, "
            f"accepted_message={result.accepted_message}, final_state={result.final_state}"
        )
        record_receive_status(
            True,
            f"ROS2 整体盘库完成：final_state={result.final_state}",
        )
        if USE_TEST_SCAN_RESULT:
            scan_result = TEST_SCAN_RESULT
            print("测试通信开关已开启，使用内置测试扫描结果")
        else:
            scan_result = load_scan_result_file(SCAN_RESULT_FILE)
            print(f"读取扫描结果文件成功：{SCAN_RESULT_FILE}")

        if not send_scan_result(scan_result):
            print("扫描结果上传失败，已保存本地备份")
        if not send_finish_status():
            print("整体盘库已完成，但 status=2 上报失败")
    except RosInventoryBridgeError as exc:
        reason = _classify_ros_failure(exc)
        print(reason)
        record_receive_status(False, reason, getattr(exc, "category", ""))
        _record_exception_upload(reason)
    except Exception as exc:
        reason = _classify_ros_failure(exc)
        print(reason)
        record_receive_status(False, reason, reason)
        _record_exception_upload(reason)
    finally:
        _set_running_state(False)
        print("整体盘库后台任务结束，is_running=False")


@app.get("/health")
def health():
    return {
        "ok": True,
        "service": "robot_api",
    }


@app.get("/status")
def status():
    return build_status_response()


@app.post("/robot/control")
def control_robot(request: ControlRequest):
    """Receive control status from Java backend."""
    global _worker_thread, is_running, _job_stage, _job_message

    if request.status != "1":
        record_receive_status(False, "仅支持 status=1 开始盘库", "unsupported status")
        raise HTTPException(status_code=400, detail="仅支持 status=1 开始盘库")

    with _state_lock:
        if is_running:
            if _job_stage in _RECHARGE_WAITING_STAGES:
                message = "正在停止充电并准备启动盘库"
                record_receive_status(True, message)
                return {
                    "code": 200,
                    "message": message,
                    "status": 1,
                    "stage": _job_stage,
                }
            message = _job_message or "盘库已在进行中"
            record_receive_status(False, message, "mission already running")
            raise HTTPException(status_code=409, detail=message)

        is_running = True
        _job_stage = "RECEIVED"
        _job_message = "小车已接收到 status=1，正在检查自动充电状态并准备启动整体盘库"
        record_receive_status(True, _job_message)

    _worker_thread = threading.Thread(
        target=_run_full_inventory_job,
        name="ros2-full-inventory-worker",
        daemon=True,
    )
    _worker_thread.start()

    return {
        "code": 200,
        "message": "小车已接收到 status=1，正在检查自动充电状态并准备启动整体盘库",
        "status": 1,
        "stage": "RECEIVED",
    }


if __name__ == "__main__":
    print(_COPYRIGHT)
    import uvicorn

    uvicorn.run(app, host=FASTAPI_HOST, port=FASTAPI_PORT)
