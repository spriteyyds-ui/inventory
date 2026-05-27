"""ROS2 bridge for starting the existing inventory mission manager."""

from __future__ import annotations

import time
from dataclasses import dataclass

try:
    from .config import (
        ROS_MISSION_STATE_TOPIC,
        ROS_MISSION_TIMEOUT_SECONDS,
        ROS_SERVICE_CALL_TIMEOUT_SECONDS,
        ROS_SERVICE_WAIT_TIMEOUT_SECONDS,
        ROS_START_MISSION_SERVICE,
    )
except ImportError:
    from config import (  # type: ignore
        ROS_MISSION_STATE_TOPIC,
        ROS_MISSION_TIMEOUT_SECONDS,
        ROS_SERVICE_CALL_TIMEOUT_SECONDS,
        ROS_SERVICE_WAIT_TIMEOUT_SECONDS,
        ROS_START_MISSION_SERVICE,
    )


@dataclass(frozen=True)
class FullInventoryResult:
    accepted_message: str
    final_state: str


def run_full_inventory_until_done() -> FullInventoryResult:
    """Call /inventory/start_mission and wait until mission_manager reports a terminal state."""
    try:
        import rclpy
        from agv_inventory_system.srv import StartMission
        from std_msgs.msg import String
    except ImportError as exc:
        raise RuntimeError(
            "ROS2 Python environment is unavailable. Run this on the robot after sourcing "
            "/opt/ros/humble/setup.bash and the agv_inventory_system workspace."
        ) from exc

    initialized_here = False
    if not rclpy.ok():
        rclpy.init(args=None)
        initialized_here = True

    node = rclpy.create_node("robot_inventory_http_bridge")
    latest_state: str | None = None
    terminal_state: str | None = None

    def on_mission_state(msg: String) -> None:
        nonlocal latest_state, terminal_state
        state = msg.data.strip()
        latest_state = state
        print(f"ROS2 mission_state: {state}")
        if state in {"DONE", "ERROR"}:
            terminal_state = state

    try:
        node.create_subscription(String, ROS_MISSION_STATE_TOPIC, on_mission_state, 10)
        client = node.create_client(StartMission, ROS_START_MISSION_SERVICE)

        print(f"等待 ROS2 service: {ROS_START_MISSION_SERVICE}")
        if not client.wait_for_service(timeout_sec=ROS_SERVICE_WAIT_TIMEOUT_SECONDS):
            raise RuntimeError(f"ROS2 service unavailable: {ROS_START_MISSION_SERVICE}")

        request = StartMission.Request()
        request.targets = []
        request.return_home = False
        request.run_full_inventory = True
        request.target_gap = ""
        request.scan_cabinets = []

        print("调用 ROS2 /inventory/start_mission: run_full_inventory=true")
        future = client.call_async(request)
        call_deadline = time.monotonic() + ROS_SERVICE_CALL_TIMEOUT_SECONDS
        while rclpy.ok() and not future.done():
            if time.monotonic() >= call_deadline:
                raise RuntimeError("ROS2 start_mission service call timed out")
            rclpy.spin_once(node, timeout_sec=0.2)

        response = future.result()
        if response is None:
            raise RuntimeError("ROS2 start_mission service returned no response")
        if not response.accepted:
            raise RuntimeError(f"ROS2 start_mission rejected: {response.message}")

        print(f"ROS2 start_mission accepted: {response.message}")

        mission_deadline = (
            None
            if ROS_MISSION_TIMEOUT_SECONDS <= 0
            else time.monotonic() + ROS_MISSION_TIMEOUT_SECONDS
        )
        while rclpy.ok() and terminal_state is None:
            if mission_deadline is not None and time.monotonic() >= mission_deadline:
                raise RuntimeError(
                    "ROS2 full inventory timed out"
                    + (f", latest_state={latest_state}" if latest_state else "")
                )
            rclpy.spin_once(node, timeout_sec=0.5)

        if terminal_state == "ERROR":
            raise RuntimeError("ROS2 full inventory ended with ERROR")
        if terminal_state != "DONE":
            raise RuntimeError(
                "ROS2 full inventory stopped before terminal state"
                + (f", latest_state={latest_state}" if latest_state else "")
            )

        return FullInventoryResult(
            accepted_message=response.message,
            final_state=terminal_state,
        )
    finally:
        node.destroy_node()
        if initialized_here:
            rclpy.shutdown()
