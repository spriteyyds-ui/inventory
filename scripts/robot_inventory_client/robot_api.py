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
    from .ros_inventory_bridge import run_full_inventory_until_done
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
    from ros_inventory_bridge import run_full_inventory_until_done  # type: ignore
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


class ControlRequest(BaseModel):
    status: StrictStr


def _set_running_state(running: bool) -> None:
    global is_running
    with _state_lock:
        is_running = running


def _run_full_inventory_job() -> None:
    """Trigger ROS2 full inventory and report completion to Java backend."""
    try:
        print("收到 status=1，开始触发 ROS2 整体盘库")
        result = run_full_inventory_until_done()
        print(
            "ROS2 整体盘库完成："
            f"accepted_message={result.accepted_message}, final_state={result.final_state}"
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
    except Exception as exc:
        reason = str(exc)
        print(f"整体盘库触发或执行失败：{reason}")
        record_upload_status(
            False,
            f"任务异常未上传: {reason}",
            source="robot_api",
            exception=reason,
        )
        send_error_status(reason)
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
    global _worker_thread, is_running

    if request.status != "1":
        record_receive_status(False, "仅支持 status=1 开始盘库", "unsupported status")
        raise HTTPException(status_code=400, detail="仅支持 status=1 开始盘库")

    with _state_lock:
        if is_running:
            record_receive_status(False, "盘库已在进行中", "mission already running")
            raise HTTPException(status_code=409, detail="盘库已在进行中")

        is_running = True
        record_receive_status(True, "小车已接收到 status=1，正在触发整体盘库")

    _worker_thread = threading.Thread(
        target=_run_full_inventory_job,
        name="ros2-full-inventory-worker",
        daemon=True,
    )
    _worker_thread.start()

    return {
        "code": 200,
        "message": "小车已接收到 status，正在触发整体盘库",
        "status": 1,
    }


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host=FASTAPI_HOST, port=FASTAPI_PORT)
