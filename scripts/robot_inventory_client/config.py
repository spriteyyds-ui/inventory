"""Global configuration for the robot inventory client.

Business configuration is loaded from ``config/inventory_system.yaml``
under the ``robot_inventory_client.ros__parameters`` section.

If PyYAML is not installed, the YAML file is missing, or parsing fails,
all values fall back to safe production defaults (192.168.1.100).
"""

from pathlib import Path

# ──── Internal: resolve YAML path relative to this file, not CWD ────
_SCRIPT_DIR = Path(__file__).resolve().parent
_PROJECT_ROOT = _SCRIPT_DIR.parent.parent  # agv_inventory_system/
_YAML_PATH = _PROJECT_ROOT / "config" / "inventory_system.yaml"


def _load_yaml():
    """Best-effort YAML loading.  Never raises."""
    try:
        import yaml  # type: ignore
    except ImportError:
        print("[config] WARNING: PyYAML not installed – using built-in defaults")
        return {}

    if not _YAML_PATH.exists():
        print(f"[config] WARNING: YAML not found: {_YAML_PATH} – using built-in defaults")
        return {}

    try:
        with open(_YAML_PATH, "r", encoding="utf-8") as fh:
            full = yaml.safe_load(fh) or {}
        section = full.get("robot_inventory_client", {})
        params = section.get("ros__parameters", {})
        if not params:
            print("[config] WARNING: robot_inventory_client.ros__parameters is empty in YAML")
        return params if isinstance(params, dict) else {}
    except Exception as exc:
        print(f"[config] WARNING: failed to parse YAML ({exc}) – using built-in defaults")
        return {}


_yaml = _load_yaml()


def _y(key, default):
    """Read *key* from the loaded YAML section, fall back to *default*."""
    val = _yaml.get(key)
    return val if val is not None else default


# ═══════════════════════════════════════════════════════════════════════
# Robot identity
# ═══════════════════════════════════════════════════════════════════════
ROBOT_ID = "car-001"

# ═══════════════════════════════════════════════════════════════════════
# Java backend URLs (read from YAML, fallback to 192.168.1.100)
# ═══════════════════════════════════════════════════════════════════════
JAVA_RESULT_URL = _y(
    "java_result_url",
    "https://192.168.1.100:8099/RobotInspection/inventoryAudit",
)
JAVA_STATUS_URL = _y(
    "java_status_url",
    "https://192.168.1.100:8099/RobotInspection/errorReport",
)

# Set to False when Java uses a self-signed or otherwise untrusted certificate.
JAVA_VERIFY_TLS = _y("java_verify_tls", False)

# HTTP request timeout in seconds
UPLOAD_TIMEOUT_SECONDS = _y("request_timeout_sec", 5)

# Upload retry count
UPLOAD_RETRY_COUNT = _y("upload_retry_count", 3)

# ═══════════════════════════════════════════════════════════════════════
# FastAPI server configuration
# ═══════════════════════════════════════════════════════════════════════
FASTAPI_HOST = _y("api_host", "0.0.0.0")
FASTAPI_PORT = _y("api_port", 8000)

# ═══════════════════════════════════════════════════════════════════════
# ROS2 mission-manager bridge configuration
# ═══════════════════════════════════════════════════════════════════════
ROS_START_MISSION_SERVICE = "/inventory/start_mission"
ROS_MISSION_STATE_TOPIC = "/inventory/mission_state"
ROS_AUTO_RECHARGE_STATUS_TOPIC = "/inventory/auto_recharge/status"
ROS_CHARGING_FLAG_TOPIC = "robot_charging_flag"
ROS_RECHARGE_FLAG_TOPIC = "robot_recharge_flag"
ROS_STOP_AUTO_CHARGE_AND_DEPART_SERVICE = "/inventory/stop_auto_charge_and_depart"
ROS_SERVICE_WAIT_TIMEOUT_SECONDS = 10
ROS_SERVICE_CALL_TIMEOUT_SECONDS = 30
ROS_RECHARGE_STATUS_SAMPLE_TIMEOUT_SECONDS = 2
ROS_RECHARGE_DEPART_WAIT_TIMEOUT_SECONDS = 90
ROS_RECHARGE_POLL_INTERVAL_SECONDS = 0.2
# 0 means wait indefinitely. V0 uses a long timeout to avoid hanging forever on failures.
ROS_MISSION_TIMEOUT_SECONDS = 4 * 60 * 60

# ═══════════════════════════════════════════════════════════════════════
# Directory used when scan-result upload fails.
# ═══════════════════════════════════════════════════════════════════════
FAILED_UPLOAD_DIR = Path(__file__).resolve().parent / "failed_uploads"

# File written by the robot inventory logic after a full inventory mission.
SCAN_RESULT_FILE = Path(__file__).resolve().parent / "scan_result.json"

# ═══════════════════════════════════════════════════════════════════════
# Test communication switch for scan-result upload only.
# False: read the real robot scan result from SCAN_RESULT_FILE.
# True: upload TEST_SCAN_RESULT to Java instead.
# ═══════════════════════════════════════════════════════════════════════
USE_TEST_SCAN_RESULT = False

TEST_SCAN_RESULT = [
    {
        "locationRfid": "shelf_1_1_1",
        "rfids": ["18A1778857359570", "2A1776341138789"],
    },
    {
        "locationRfid": "shelf_1_1_2",
        "rfids": ["RFID_TAG_99999"],
    },
]