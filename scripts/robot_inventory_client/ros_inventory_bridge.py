# Copyright (c) 2026 郁有冬 <spriteyyds@gmail.com>. All rights reserved.
"""ROS2 bridge for starting the existing inventory mission manager."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Optional

try:
    from .config import (
        ROS_AUTO_RECHARGE_STATUS_TOPIC,
        ROS_CHARGING_FLAG_TOPIC,
        ROS_RECHARGE_DEPART_WAIT_TIMEOUT_SECONDS,
        ROS_RECHARGE_FLAG_TOPIC,
        ROS_RECHARGE_POLL_INTERVAL_SECONDS,
        ROS_RECHARGE_STATUS_SAMPLE_TIMEOUT_SECONDS,
        ROS_MISSION_STATE_TOPIC,
        ROS_MISSION_TIMEOUT_SECONDS,
        ROS_SERVICE_CALL_TIMEOUT_SECONDS,
        ROS_SERVICE_WAIT_TIMEOUT_SECONDS,
        ROS_START_MISSION_SERVICE,
        ROS_STOP_AUTO_CHARGE_AND_DEPART_SERVICE,
    )
except ImportError:
    from config import (  # type: ignore
        ROS_AUTO_RECHARGE_STATUS_TOPIC,
        ROS_CHARGING_FLAG_TOPIC,
        ROS_RECHARGE_DEPART_WAIT_TIMEOUT_SECONDS,
        ROS_RECHARGE_FLAG_TOPIC,
        ROS_RECHARGE_POLL_INTERVAL_SECONDS,
        ROS_RECHARGE_STATUS_SAMPLE_TIMEOUT_SECONDS,
        ROS_MISSION_STATE_TOPIC,
        ROS_MISSION_TIMEOUT_SECONDS,
        ROS_SERVICE_CALL_TIMEOUT_SECONDS,
        ROS_SERVICE_WAIT_TIMEOUT_SECONDS,
        ROS_START_MISSION_SERVICE,
        ROS_STOP_AUTO_CHARGE_AND_DEPART_SERVICE,
    )


STAGE_CHECKING_RECHARGE = "CHECKING_RECHARGE"
STAGE_WAITING_RECHARGE_CANCEL = "WAITING_RECHARGE_CANCEL"
STAGE_WAITING_DEPART_CHARGER = "WAITING_DEPART_CHARGER"
STAGE_STARTING_MISSION = "STARTING_MISSION"
STAGE_RUNNING_MISSION = "RUNNING_MISSION"

_START_BLOCKING_AUTO_RECHARGE_STATUSES = {
    "STARTING",
    "NAVIGATING",
    "DOCKING",
    "CHARGING",
    "CANCELING",
    # mission_manager currently treats COMPLETE as still docked/controlled and
    # rejects start_mission until stop_auto_charge_and_depart releases it.
    "COMPLETE",
}

ProgressCallback = Optional[Callable[[str, str], None]]


class RosInventoryBridgeError(RuntimeError):
    """Runtime error with a stable category for the HTTP layer."""

    def __init__(self, message: str, category: str = "ROS2_BRIDGE_ERROR") -> None:
        super().__init__(message)
        self.category = category


@dataclass(frozen=True)
class FullInventoryResult:
    accepted_message: str
    final_state: str
    prestart_message: str = ""


def _status_text(value: Optional[str]) -> str:
    return value if value else "UNKNOWN"


def _flag_text(value: Optional[bool]) -> str:
    if value is None:
        return "UNKNOWN"
    return "true" if value else "false"


def _int_flag_text(value: Optional[int]) -> str:
    return "UNKNOWN" if value is None else str(value)


def _normal_status(value: Optional[str]) -> str:
    return (value or "").strip().upper()


def _status_blocks_start(status: Optional[str]) -> bool:
    return _normal_status(status) in _START_BLOCKING_AUTO_RECHARGE_STATUSES


def _emit_progress(progress_callback: ProgressCallback, stage: str, message: str) -> None:
    print(message)
    if progress_callback is not None:
        progress_callback(stage, message)


def run_full_inventory_until_done(progress_callback: ProgressCallback = None) -> FullInventoryResult:
    """Call /inventory/start_mission and wait until mission_manager reports a terminal state."""
    try:
        import rclpy
        from agv_inventory_system.srv import StartMission
        from std_msgs.msg import Bool, Int8
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
    except ImportError as exc:
        raise RosInventoryBridgeError(
            "ROS2 Python environment is unavailable. Run this on the robot after sourcing "
            "/opt/ros/humble/setup.bash and the agv_inventory_system workspace.",
            category="ROS2_ENV_UNAVAILABLE",
        ) from exc

    initialized_here = False
    if not rclpy.ok():
        rclpy.init(args=None)
        initialized_here = True

    node = rclpy.create_node("robot_inventory_http_bridge")
    latest_state: str | None = None
    terminal_state: str | None = None
    latest_auto_recharge_status: str | None = None
    latest_charging_flag: bool | None = None
    latest_recharge_flag: int | None = None

    def snapshot_text() -> str:
        return (
            f"mission_state={_status_text(latest_state)}, "
            f"auto_recharge/status={_status_text(latest_auto_recharge_status)}, "
            f"robot_charging_flag={_flag_text(latest_charging_flag)}, "
            f"robot_recharge_flag={_int_flag_text(latest_recharge_flag)}"
        )

    def on_mission_state(msg: String) -> None:
        nonlocal latest_state, terminal_state
        state = msg.data.strip()
        latest_state = state
        print(f"ROS2 mission_state: {state}")
        if state in {"DONE", "ERROR"}:
            terminal_state = state

    def on_auto_recharge_status(msg: String) -> None:
        nonlocal latest_auto_recharge_status
        latest_auto_recharge_status = msg.data.strip().upper()

    def on_charging_flag(msg: Bool) -> None:
        nonlocal latest_charging_flag
        latest_charging_flag = bool(msg.data)

    def on_recharge_flag(msg: Int8) -> None:
        nonlocal latest_recharge_flag
        latest_recharge_flag = int(msg.data)

    def spin_until(deadline: float, timeout_sec: float = 0.2) -> None:
        if time.monotonic() >= deadline:
            return
        rclpy.spin_once(
            node,
            timeout_sec=max(0.01, min(timeout_sec, deadline - time.monotonic())),
        )

    def wait_for_initial_recharge_sample() -> None:
        deadline = time.monotonic() + max(0.0, ROS_RECHARGE_STATUS_SAMPLE_TIMEOUT_SECONDS)
        while rclpy.ok() and time.monotonic() < deadline:
            if (
                latest_auto_recharge_status is not None
                and latest_charging_flag is not None
                and latest_recharge_flag is not None
            ):
                return
            spin_until(deadline, ROS_RECHARGE_POLL_INTERVAL_SECONDS)

    def call_stop_auto_charge_and_depart() -> str:
        stop_client = node.create_client(Trigger, ROS_STOP_AUTO_CHARGE_AND_DEPART_SERVICE)
        _emit_progress(
            progress_callback,
            STAGE_WAITING_RECHARGE_CANCEL,
            "检测到自动回充/充电状态，准备调用 "
            f"{ROS_STOP_AUTO_CHARGE_AND_DEPART_SERVICE}；{snapshot_text()}",
        )
        if not stop_client.wait_for_service(timeout_sec=ROS_SERVICE_WAIT_TIMEOUT_SECONDS):
            raise RosInventoryBridgeError(
                "停止充电并离桩失败：ROS2 service unavailable: "
                f"{ROS_STOP_AUTO_CHARGE_AND_DEPART_SERVICE}。"
                "请确认 mission_manager_node 已启动，且该服务类型为 std_srvs/srv/Trigger。",
                category="STOP_AUTO_CHARGE_DEPART_SERVICE_UNAVAILABLE",
            )

        future = stop_client.call_async(Trigger.Request())
        call_deadline = time.monotonic() + ROS_SERVICE_CALL_TIMEOUT_SECONDS
        while rclpy.ok() and not future.done():
            if time.monotonic() >= call_deadline:
                raise RosInventoryBridgeError(
                    "停止充电并离桩失败："
                    f"{ROS_STOP_AUTO_CHARGE_AND_DEPART_SERVICE} service call timed out",
                    category="STOP_AUTO_CHARGE_DEPART_CALL_TIMEOUT",
                )
            spin_until(call_deadline, ROS_RECHARGE_POLL_INTERVAL_SECONDS)

        response = future.result()
        if response is None:
            raise RosInventoryBridgeError(
                "停止充电并离桩失败：stop_auto_charge_and_depart returned no response",
                category="STOP_AUTO_CHARGE_DEPART_NO_RESPONSE",
            )
        if not response.success:
            raise RosInventoryBridgeError(
                "停止充电并离桩失败："
                f"{ROS_STOP_AUTO_CHARGE_AND_DEPART_SERVICE} rejected: {response.message}",
                category="STOP_AUTO_CHARGE_DEPART_REJECTED",
            )
        return response.message

    def wait_until_recharge_released(require_idle_state: bool) -> str:
        deadline = time.monotonic() + max(0.1, ROS_RECHARGE_DEPART_WAIT_TIMEOUT_SECONDS)
        last_progress_time = 0.0
        while rclpy.ok():
            now = time.monotonic()
            if now >= deadline:
                base_message = (
                    "小车正在充电，停止充电并离桩未完成，盘库未启动；"
                    f"等待超时 {ROS_RECHARGE_DEPART_WAIT_TIMEOUT_SECONDS:.1f}s；"
                    f"{snapshot_text()}"
                )
                if latest_charging_flag is not False:
                    raise RosInventoryBridgeError(
                        base_message + "；等待 robot_charging_flag=false 超时。",
                        category="CHARGING_FLAG_CLEAR_TIMEOUT",
                    )
                raise RosInventoryBridgeError(
                    base_message,
                    category="STOP_AUTO_CHARGE_DEPART_WAIT_TIMEOUT",
                )

            if latest_state == "ERROR":
                raise RosInventoryBridgeError(
                    "停止充电并离桩失败：mission_manager 进入 ERROR；" + snapshot_text(),
                    category="STOP_AUTO_CHARGE_DEPART_FAILED",
                )

            status_released = not _status_blocks_start(latest_auto_recharge_status)
            charging_released = latest_charging_flag is False
            recharge_control_released = latest_recharge_flag in (None, 0)
            state_released = latest_state == "IDLE" if require_idle_state else latest_state != "STOP_AUTO_CHARGE_AND_DEPART"
            if (
                status_released
                and charging_released
                and recharge_control_released
                and state_released
            ):
                return "停止自动充电并离桩完成，可以启动盘库；" + snapshot_text()

            if now - last_progress_time >= 2.0:
                _emit_progress(
                    progress_callback,
                    STAGE_WAITING_DEPART_CHARGER,
                    "正在等待自动回充释放/离桩完成；" + snapshot_text(),
                )
                last_progress_time = now
            spin_until(deadline, ROS_RECHARGE_POLL_INTERVAL_SECONDS)

        raise RosInventoryBridgeError(
            "ROS2 context stopped while waiting for recharge release; " + snapshot_text(),
            category="ROS2_CONTEXT_STOPPED",
        )

    def recharge_or_charge_blocks_start() -> bool:
        return (
            _status_blocks_start(latest_auto_recharge_status)
            or latest_charging_flag is True
            or (latest_recharge_flag is not None and latest_recharge_flag != 0)
            or latest_state in {"AUTO_RECHARGING", "STOP_AUTO_CHARGE_AND_DEPART"}
        )

    try:
        node.create_subscription(String, ROS_MISSION_STATE_TOPIC, on_mission_state, 10)
        node.create_subscription(
            String,
            ROS_AUTO_RECHARGE_STATUS_TOPIC,
            on_auto_recharge_status,
            10,
        )
        node.create_subscription(Bool, ROS_CHARGING_FLAG_TOPIC, on_charging_flag, 10)
        node.create_subscription(Int8, ROS_RECHARGE_FLAG_TOPIC, on_recharge_flag, 10)
        client = node.create_client(StartMission, ROS_START_MISSION_SERVICE)

        _emit_progress(
            progress_callback,
            STAGE_CHECKING_RECHARGE,
            "收到整体盘库启动请求，先检查自动回充/充电状态。",
        )
        wait_for_initial_recharge_sample()
        prestart_message = "启动前自动回充状态检查完成；" + snapshot_text()

        if latest_state == "STOP_AUTO_CHARGE_AND_DEPART":
            _emit_progress(
                progress_callback,
                STAGE_WAITING_DEPART_CHARGER,
                "mission_manager 正在执行停止自动充电并离桩，等待完成；" + snapshot_text(),
            )
            prestart_message = wait_until_recharge_released(require_idle_state=True)
        elif recharge_or_charge_blocks_start():
            stop_message = call_stop_auto_charge_and_depart()
            _emit_progress(
                progress_callback,
                STAGE_WAITING_DEPART_CHARGER,
                "已调用停止自动充电并离桩服务，等待 robot_charging_flag=false 和离桩完成："
                f"{stop_message}",
            )
            prestart_message = wait_until_recharge_released(require_idle_state=True)
        else:
            _emit_progress(
                progress_callback,
                STAGE_STARTING_MISSION,
                "当前未检测到自动回充/充电占用，准备启动整体盘库；" + snapshot_text(),
            )

        print(f"等待 ROS2 service: {ROS_START_MISSION_SERVICE}")
        if not client.wait_for_service(timeout_sec=ROS_SERVICE_WAIT_TIMEOUT_SECONDS):
            raise RosInventoryBridgeError(
                f"ROS2 start_mission service unavailable: {ROS_START_MISSION_SERVICE}",
                category="START_MISSION_SERVICE_UNAVAILABLE",
            )

        request = StartMission.Request()
        request.targets = []
        request.return_home = False
        request.run_full_inventory = True
        request.target_gap = ""
        request.scan_cabinets = []

        print("调用 ROS2 /inventory/start_mission: run_full_inventory=true")
        _emit_progress(
            progress_callback,
            STAGE_STARTING_MISSION,
            f"调用 ROS2 {ROS_START_MISSION_SERVICE}: run_full_inventory=true",
        )
        future = client.call_async(request)
        call_deadline = time.monotonic() + ROS_SERVICE_CALL_TIMEOUT_SECONDS
        while rclpy.ok() and not future.done():
            if time.monotonic() >= call_deadline:
                raise RosInventoryBridgeError(
                    "ROS2 start_mission service call timed out",
                    category="START_MISSION_CALL_TIMEOUT",
                )
            spin_until(call_deadline, 0.2)

        response = future.result()
        if response is None:
            raise RosInventoryBridgeError(
                "ROS2 start_mission service returned no response",
                category="START_MISSION_NO_RESPONSE",
            )
        if not response.accepted:
            raise RosInventoryBridgeError(
                f"ROS2 start_mission rejected: {response.message}",
                category="START_MISSION_REJECTED",
            )

        print(f"ROS2 start_mission accepted: {response.message}")
        _emit_progress(
            progress_callback,
            STAGE_RUNNING_MISSION,
            f"ROS2 start_mission 成功接收：{response.message}",
        )
        terminal_state = None

        mission_deadline = (
            None
            if ROS_MISSION_TIMEOUT_SECONDS <= 0
            else time.monotonic() + ROS_MISSION_TIMEOUT_SECONDS
        )
        while rclpy.ok() and terminal_state is None:
            if mission_deadline is not None and time.monotonic() >= mission_deadline:
                raise RosInventoryBridgeError(
                    "ROS2 full inventory timed out"
                    + (f", latest_state={latest_state}" if latest_state else ""),
                    category="MISSION_TIMEOUT",
                )
            rclpy.spin_once(node, timeout_sec=0.5)

        if terminal_state == "ERROR":
            raise RosInventoryBridgeError(
                "ROS2 full inventory ended with ERROR",
                category="MISSION_ERROR",
            )
        if terminal_state != "DONE":
            raise RosInventoryBridgeError(
                "ROS2 full inventory stopped before terminal state"
                + (f", latest_state={latest_state}" if latest_state else ""),
                category="MISSION_STOPPED",
            )

        return FullInventoryResult(
            accepted_message=response.message,
            final_state=terminal_state,
            prestart_message=prestart_message,
        )
    finally:
        node.destroy_node()
        if initialized_here:
            rclpy.shutdown()
