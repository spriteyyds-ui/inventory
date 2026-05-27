"""HTTP upload client for Java backend callbacks."""

import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List

import requests

try:
    from .config import (
        FAILED_UPLOAD_DIR,
        JAVA_RESULT_URL,
        JAVA_STATUS_URL,
        UPLOAD_TIMEOUT_SECONDS,
    )
except ImportError:
    from config import (  # type: ignore
        FAILED_UPLOAD_DIR,
        JAVA_RESULT_URL,
        JAVA_STATUS_URL,
        UPLOAD_TIMEOUT_SECONDS,
    )


ScanResult = List[Dict[str, Any]]


def _now_iso() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


def save_failed_scan_result(scan_result: ScanResult, reason: str) -> Path:
    """Save failed scan-result upload payload to a local JSON file."""
    FAILED_UPLOAD_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    file_path = FAILED_UPLOAD_DIR / f"scan_result_{timestamp}.json"

    payload = {
        "failedAt": _now_iso(),
        "reason": reason,
        "javaResultUrl": JAVA_RESULT_URL,
        "scanResult": scan_result,
    }

    file_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(f"扫描结果上传失败，已保存本地备份：{file_path}")
    return file_path


def send_robot_status(status: str, message: str | None = None) -> bool:
    """向 Java 后端发送机器人状态。"""
    payload: Dict[str, Any] = {"status": status}
    if message:
        payload["message"] = message

    try:
        response = requests.post(
            JAVA_STATUS_URL,
            json=payload,
            timeout=UPLOAD_TIMEOUT_SECONDS,
        )
        print(f"Java 状态响应：{response.status_code} {response.text}")
        response.raise_for_status()
        return True
    except requests.RequestException as exc:
        print(f"发送机器人状态失败 status={status}：{exc}")
        return False


def send_finish_status() -> bool:
    """向 Java 后端发送盘点完成状态（status="2" 代表完工）"""
    return send_robot_status("2")


def send_error_status(reason: str) -> bool:
    """Best-effort error status callback for V0 bridge failures."""
    return send_robot_status("ERROR", reason)


def send_scan_result(scan_result: ScanResult) -> bool:
    """向 Java 后端发送扫描结果"""
    try:
        response = requests.post(
            JAVA_RESULT_URL,
            json=scan_result,
            timeout=UPLOAD_TIMEOUT_SECONDS,
        )
        print(f"Java 扫描结果响应：{response.status_code} {response.text}")
        response.raise_for_status()
        return True
    except requests.RequestException as exc:
        reason = str(exc)
        print(f"发送扫描结果失败：{reason}")
        save_failed_scan_result(scan_result, reason)
        return False
