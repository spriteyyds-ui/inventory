"""HTTP upload client for Java backend callbacks."""

import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

import requests

try:
    from .config import (
        FAILED_UPLOAD_DIR,
        JAVA_RESULT_URL,
        JAVA_STATUS_URL,
        JAVA_VERIFY_TLS,
        UPLOAD_TIMEOUT_SECONDS,
    )
    from .status_store import record_upload_status
except ImportError:
    from config import (  # type: ignore
        FAILED_UPLOAD_DIR,
        JAVA_RESULT_URL,
        JAVA_STATUS_URL,
        JAVA_VERIFY_TLS,
        UPLOAD_TIMEOUT_SECONDS,
    )
    from status_store import record_upload_status  # type: ignore


ScanResult = List[Dict[str, Any]]


def _now_iso() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


def build_robot_audit_payload(scan_result: ScanResult) -> Dict[str, ScanResult]:
    """Build the Java RobotAuditReqDTO payload."""
    return {"scanCells": scan_result}


def load_scan_result_file(file_path: Path) -> ScanResult:
    """Load robot scan cells from a local JSON result file."""
    if not file_path.exists():
        raise FileNotFoundError(f"扫描结果文件不存在：{file_path}")

    data = json.loads(file_path.read_text(encoding="utf-8-sig"))
    if isinstance(data, dict) and "javaPayload" in data:
        data = data["javaPayload"]

    if isinstance(data, dict) and "scanCells" in data:
        data = data["scanCells"]

    if not isinstance(data, list):
        raise ValueError("扫描结果 JSON 顶层必须是数组，或包含 scanCells 的对象")

    for index, item in enumerate(data):
        if not isinstance(item, dict):
            raise ValueError(f"扫描结果第 {index + 1} 项必须是对象")
        if not isinstance(item.get("locationRfid"), str) or not item["locationRfid"]:
            raise ValueError(f"扫描结果第 {index + 1} 项缺少 locationRfid")
        rfids = item.get("rfids")
        if rfids is None:
            item["rfids"] = []
        elif not isinstance(rfids, list) or not all(isinstance(rfid, str) for rfid in rfids):
            raise ValueError(f"扫描结果第 {index + 1} 项 rfids 必须是字符串数组")

    return data


def save_failed_scan_result(scan_result: ScanResult, reason: str) -> Path:
    """Save failed scan-result upload payload to a local JSON file."""
    FAILED_UPLOAD_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    file_path = FAILED_UPLOAD_DIR / f"scan_result_{timestamp}.json"

    java_payload = build_robot_audit_payload(scan_result)
    payload = {
        "failedAt": _now_iso(),
        "reason": reason,
        "javaResultUrl": JAVA_RESULT_URL,
        "javaPayload": java_payload,
    }

    file_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(f"扫描结果上传失败，已保存本地备份：{file_path}")
    return file_path


def send_robot_status(status: str, message: Optional[str] = None) -> bool:
    """向 Java 后端发送机器人状态。"""
    payload: Dict[str, Any] = {"status": status}
    if message:
        payload["message"] = message

    try:
        response = requests.post(
            JAVA_STATUS_URL,
            json=payload,
            timeout=UPLOAD_TIMEOUT_SECONDS,
            verify=JAVA_VERIFY_TLS,
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
    payload = build_robot_audit_payload(scan_result)
    try:
        response = requests.post(
            JAVA_RESULT_URL,
            json=payload,
            timeout=UPLOAD_TIMEOUT_SECONDS,
            verify=JAVA_VERIFY_TLS,
        )
        print(f"Java 扫描结果响应：{response.status_code} {response.text}")
        status_code = response.status_code
        response_text = response.text
        if status_code != 200:
            reason = f"HTTP 非 200: {status_code}"
            record_upload_status(
                False,
                reason,
                status_code=status_code,
                source="robot_api",
                exception=reason,
            )
            save_failed_scan_result(scan_result, reason)
            return False

        try:
            response_data = response.json()
        except ValueError as exc:
            reason = f"响应解析失败: {exc}"
            record_upload_status(
                False,
                reason,
                status_code=status_code,
                source="robot_api",
                exception=reason,
            )
            save_failed_scan_result(scan_result, reason)
            return False

        if not isinstance(response_data, dict):
            reason = "响应解析失败: JSON 顶层不是对象"
            record_upload_status(
                False,
                reason,
                status_code=status_code,
                source="robot_api",
                exception=reason,
            )
            save_failed_scan_result(scan_result, reason)
            return False

        success_value = response_data.get("success")
        message_value = response_data.get("message")
        if message_value is None:
            message_value = response_data.get("msg")
        message = str(message_value) if message_value is not None else "success=true"

        if success_value is False:
            reason = message or "服务器业务失败 success=false"
            record_upload_status(
                False,
                reason,
                status_code=status_code,
                source="robot_api",
                exception=reason,
            )
            save_failed_scan_result(scan_result, reason)
            return False
        if success_value is not True:
            reason = message if message_value is not None else "响应缺少 success=true"
            record_upload_status(
                False,
                reason,
                status_code=status_code,
                source="robot_api",
                exception=reason,
            )
            save_failed_scan_result(scan_result, reason)
            return False

        record_upload_status(
            True,
            message,
            status_code=status_code,
            source="robot_api",
        )
        return True
    except requests.RequestException as exc:
        reason = str(exc)
        print(f"发送扫描结果失败：{reason}")
        record_upload_status(
            False,
            reason,
            status_code=None,
            source="robot_api",
            exception=reason,
        )
        save_failed_scan_result(scan_result, reason)
        return False
