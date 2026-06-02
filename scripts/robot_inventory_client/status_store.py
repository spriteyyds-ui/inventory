"""Small status store used by the robot HTTP API and operation GUI."""

from __future__ import annotations

import json
import os
import tempfile
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional


STATUS_DIR = Path("/tmp/agv_inventory_system")
ROBOT_API_STATUS_FILE = STATUS_DIR / "robot_api_status.json"
RFID_UPLOAD_STATUS_FILE = STATUS_DIR / "rfid_upload_status.json"

_LOCK = threading.Lock()


def now_iso() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


def _empty_state() -> Dict[str, Any]:
    return {
        "ok": True,
        "service": "robot_api",
        "last_receive_success": None,
        "last_receive_time": "",
        "last_receive_message": "暂无任务接收记录",
        "last_upload_success": None,
        "last_upload_time": "",
        "last_upload_message": "暂无上传记录",
        "last_upload_status_code": None,
        "last_upload_source": "",
        "last_exception": "",
        "updated_at": "",
    }


def _safe_read_json(path: Path) -> Dict[str, Any]:
    try:
        if not path.exists():
            return {}
        data = json.loads(path.read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def _atomic_write_json(path: Path, data: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(
        prefix=f".{path.name}.",
        suffix=".tmp",
        dir=str(path.parent),
        text=True,
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as output:
            json.dump(data, output, ensure_ascii=False, sort_keys=True)
            output.write("\n")
        os.replace(tmp_name, path)
    finally:
        if os.path.exists(tmp_name):
            os.unlink(tmp_name)


def read_robot_api_state() -> Dict[str, Any]:
    with _LOCK:
        state = _empty_state()
        state.update(_safe_read_json(ROBOT_API_STATUS_FILE))
        return state


def update_robot_api_state(**updates: Any) -> Dict[str, Any]:
    with _LOCK:
        state = _empty_state()
        state.update(_safe_read_json(ROBOT_API_STATUS_FILE))
        state.update(updates)
        state["ok"] = True
        state["service"] = "robot_api"
        state["updated_at"] = now_iso()
        _atomic_write_json(ROBOT_API_STATUS_FILE, state)
        return state


def record_receive_status(success: bool, message: str, exception: str = "") -> Dict[str, Any]:
    return update_robot_api_state(
        last_receive_success=bool(success),
        last_receive_time=now_iso(),
        last_receive_message=message or "",
        last_exception=exception or "",
    )


def record_upload_status(
    success: Optional[bool],
    message: str,
    status_code: Optional[int] = None,
    source: str = "robot_api",
    exception: str = "",
) -> Dict[str, Any]:
    return update_robot_api_state(
        last_upload_success=success,
        last_upload_time=now_iso(),
        last_upload_message=message or "",
        last_upload_status_code=status_code,
        last_upload_source=source,
        last_exception=exception or "",
    )


def _status_sort_key(status: Dict[str, Any]) -> str:
    value = status.get("last_upload_time")
    return value if isinstance(value, str) else ""


def latest_upload_status() -> Dict[str, Any]:
    api_state = read_robot_api_state()
    candidates = []
    if api_state.get("last_upload_time"):
        candidates.append({
            "last_upload_success": api_state.get("last_upload_success"),
            "last_upload_time": api_state.get("last_upload_time", ""),
            "last_upload_message": api_state.get("last_upload_message", ""),
            "last_upload_status_code": api_state.get("last_upload_status_code"),
            "last_upload_source": api_state.get("last_upload_source") or "robot_api",
        })

    cpp_state = _safe_read_json(RFID_UPLOAD_STATUS_FILE)
    if cpp_state.get("last_upload_time"):
        candidates.append({
            "last_upload_success": cpp_state.get("last_upload_success"),
            "last_upload_time": cpp_state.get("last_upload_time", ""),
            "last_upload_message": cpp_state.get("last_upload_message", ""),
            "last_upload_status_code": cpp_state.get("last_upload_status_code"),
            "last_upload_source": cpp_state.get("last_upload_source") or "mission_manager",
        })

    if not candidates:
        return {
            "last_upload_success": None,
            "last_upload_time": "",
            "last_upload_message": "暂无上传记录",
            "last_upload_status_code": None,
            "last_upload_source": "",
        }

    return max(candidates, key=_status_sort_key)


def build_status_response() -> Dict[str, Any]:
    state = read_robot_api_state()
    upload_status = latest_upload_status()
    state.update(upload_status)
    if upload_status.get("last_upload_success") is True:
        state["last_exception"] = ""
    elif upload_status.get("last_upload_success") is False:
        state["last_exception"] = upload_status.get("last_upload_message", "")
    state["ok"] = True
    state["service"] = "robot_api"
    return state
