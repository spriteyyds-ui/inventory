#!/usr/bin/env python3
# Copyright (c) 2026 郁有冬 <spriteyyds@gmail.com>. All rights reserved.
_COPYRIGHT = (
    "========================================\n"
    " agv_inventory_system\n"
    " Copyright (c) 2026 郁有冬 <spriteyyds@gmail.com>\n"
    " All rights reserved.\n"
    "========================================\n"
)
from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from agv_inventory_system.msg import LiftState
from agv_inventory_system.srv import LiftMoveTimed


class LiftRelayDiagnosticNode(Node):
    def __init__(self) -> None:
        super().__init__("lift_relay_diagnostic_node")
        self.action = self.declare_parameter("action", "sequence").value
        self.duration_sec = float(self.declare_parameter("duration_sec", 2.0).value)
        self.service_timeout_sec = float(self.declare_parameter("service_timeout_sec", 5.0).value)
        self.state_topic = self.declare_parameter("lift_state_topic", "/lift/state").value

        self.state = None
        self.create_subscription(LiftState, self.state_topic, self.state_callback, 10)
        self.lift_service_clients = {
            "all_off": self.create_client(Trigger, "/lift/all_off"),
            "stop": self.create_client(Trigger, "/lift/stop"),
            "reset_height": self.create_client(Trigger, "/lift/reset_estimated_height"),
            "up": self.create_client(LiftMoveTimed, "/lift/up"),
            "down": self.create_client(LiftMoveTimed, "/lift/down"),
            "home": self.create_client(LiftMoveTimed, "/lift/home"),
        }

    def state_callback(self, msg: LiftState) -> None:
        self.state = msg

    def wait_for_service(self, name: str) -> bool:
        client = self.lift_service_clients[name]
        deadline = time.monotonic() + max(0.1, self.service_timeout_sec)
        while rclpy.ok() and time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=0.1):
                return True
        self.get_logger().error(f"service unavailable: {name}")
        return False

    def call_trigger(self, name: str) -> bool:
        if not self.wait_for_service(name):
            return False
        self.get_logger().info(f"调用 /lift/{name}")
        future = self.lift_service_clients[name].call_async(Trigger.Request())
        if not self.wait_future(future):
            return False
        result = future.result()
        ok = bool(result and result.success)
        message = result.message if result else ""
        self.get_logger().info(f"/lift/{name} result={ok} message={message}")
        return ok

    def call_motion(self, name: str) -> bool:
        if not self.wait_for_service(name):
            return False
        req = LiftMoveTimed.Request()
        req.direction = name
        req.duration_sec = float(self.duration_sec)
        self.get_logger().info(f"调用 /lift/{name} duration={req.duration_sec:.2f}s")
        future = self.lift_service_clients[name].call_async(req)
        if not self.wait_future(future):
            self.call_trigger("stop")
            return False
        result = future.result()
        ok = bool(result and result.success)
        message = result.message if result else ""
        height = result.estimated_height_m if result else 0.0
        self.get_logger().info(
            f"/lift/{name} result={ok} height={height:.3f}m message={message}")
        return ok

    def wait_future(self, future) -> bool:
        deadline = time.monotonic() + max(0.1, self.service_timeout_sec + self.duration_sec + 1.0)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if future.done():
                return True
        self.get_logger().error("service call timeout")
        return False

    def run(self) -> int:
        action = str(self.action).strip().lower()
        if action not in ("all_off", "stop", "reset_height", "up", "down", "home", "sequence"):
            self.get_logger().error(f"unsupported action: {self.action}")
            return 2

        if action in ("all_off", "stop", "reset_height"):
            return 0 if self.call_trigger(action) else 1
        if action in ("up", "down", "home"):
            ok = self.call_motion(action)
            self.call_trigger("stop")
            return 0 if ok else 1

        sequence = [
            ("trigger", "all_off"),
            ("motion", "up"),
            ("trigger", "stop"),
            ("motion", "down"),
            ("trigger", "stop"),
            ("trigger", "all_off"),
        ]
        for kind, name in sequence:
            ok = self.call_trigger(name) if kind == "trigger" else self.call_motion(name)
            self.print_state()
            if not ok:
                self.call_trigger("stop")
                self.call_trigger("all_off")
                return 1
        return 0

    def print_state(self) -> None:
        rclpy.spin_once(self, timeout_sec=0.05)
        if self.state is None:
            self.get_logger().warn("未收到 /lift/state")
            return
        self.get_logger().info(
            f"state={self.state.state} direction={self.state.moving_direction} "
            f"moving={self.state.is_moving} height={self.state.estimated_height_m:.3f}m "
            f"error={self.state.error_message}"
        )


def main(args=None) -> None:
    print(_COPYRIGHT)
    rclpy.init(args=args)
    node = LiftRelayDiagnosticNode()
    exit_code = 1
    try:
        exit_code = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
