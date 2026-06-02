"""Global configuration for the robot inventory client."""

from pathlib import Path


# Robot identity
ROBOT_ID = "car-001"

# Java backend URLs. Replace Java backend IP before deployment.
JAVA_STATUS_URL = "https://172.26.130.75:8099/RobotInspection/errorReport"
JAVA_RESULT_URL = "https://172.26.130.75:8099/RobotInspection/inventoryAudit"

# Set to False when Java uses a self-signed or otherwise untrusted HTTPS certificate.
JAVA_VERIFY_TLS = False

# HTTP request timeout in seconds
UPLOAD_TIMEOUT_SECONDS = 5

# FastAPI server configuration
FASTAPI_HOST = "0.0.0.0"
FASTAPI_PORT = 8000
#校园网
#FASTAPI_HOST = "172.26.193.87"
#FASTAPI_PORT = 8000

# ROS2 mission-manager bridge configuration.
ROS_START_MISSION_SERVICE = "/inventory/start_mission"
ROS_MISSION_STATE_TOPIC = "/inventory/mission_state"
ROS_SERVICE_WAIT_TIMEOUT_SECONDS = 10
ROS_SERVICE_CALL_TIMEOUT_SECONDS = 30
# 0 means wait indefinitely. V0 uses a long timeout to avoid hanging forever on failures.
ROS_MISSION_TIMEOUT_SECONDS = 4 * 60 * 60

# Directory used when scan-result upload fails.
FAILED_UPLOAD_DIR = Path(__file__).resolve().parent / "failed_uploads"

# File written by the robot inventory logic after a full inventory mission.
SCAN_RESULT_FILE = Path(__file__).resolve().parent / "scan_result.json"

# Test communication switch for scan-result upload only.
# False: read the real robot scan result from SCAN_RESULT_FILE.
# True: upload TEST_SCAN_RESULT to Java instead.
#测试用的扫描结果，用于调试，默认关闭由小车扫描结果上传功能，开启后上传测试结果
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
