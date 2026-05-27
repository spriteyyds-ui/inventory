"""Global configuration for the robot inventory client."""

from pathlib import Path


# Robot identity
ROBOT_ID = "car-001"

# Java backend URLs. Replace Java backend IP before deployment.
JAVA_STATUS_URL = "http://Java后端IP:8080/api/robot/status"
JAVA_RESULT_URL = "http://Java后端IP:8080/api/rfid/scan-result"

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
