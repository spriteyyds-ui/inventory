"""FastAPI entrypoint for receiving Java backend robot control commands."""

import threading
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel, StrictStr

try:
    from .config import FASTAPI_HOST, FASTAPI_PORT
    from .ros_inventory_bridge import run_full_inventory_until_done
    from .upload_client import send_error_status, send_finish_status
except ImportError:
    from config import FASTAPI_HOST, FASTAPI_PORT  # type: ignore
    from ros_inventory_bridge import run_full_inventory_until_done  # type: ignore
    from upload_client import send_error_status, send_finish_status  # type: ignore


app = FastAPI(title="Robot Inventory Client")

is_running = False

_state_lock = threading.Lock()
_worker_thread: threading.Thread | None = None


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
        if not send_finish_status():
            print("整体盘库已完成，但 status=2 上报失败")
    except Exception as exc:
        reason = str(exc)
        print(f"整体盘库触发或执行失败：{reason}")
        send_error_status(reason)
    finally:
        _set_running_state(False)
        print("整体盘库后台任务结束，is_running=False")


@app.post("/robot/control")
def control_robot(request: ControlRequest):
    """Receive control status from Java backend."""
    global _worker_thread, is_running

    if request.status != "1":
        raise HTTPException(status_code=400, detail="仅支持 status=1 开始盘库")

    with _state_lock:
        if is_running:
            raise HTTPException(status_code=409, detail="盘库已在进行中")

        is_running = True

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
