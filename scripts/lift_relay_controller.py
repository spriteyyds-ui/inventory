#!/usr/bin/env python3
# Copyright (c) 2026 郁有冬 <spriteyyds@gmail.com>. All rights reserved.
from __future__ import annotations

_COPYRIGHT = (
    "========================================\n"
    " agv_inventory_system\n"
    " Copyright (c) 2026 郁有冬 <spriteyyds@gmail.com>\n"
    " All rights reserved.\n"
    "========================================\n"
)

import math
import os
import threading
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from agv_inventory_system.msg import LiftState
from agv_inventory_system.srv import LiftMoveTimed, LiftMoveToHeight


FUNC_WRITE_SINGLE_COIL = 0x05
FUNC_WRITE_SINGLE_REGISTER = 0x06
COIL_ON = 0xFF00
COIL_OFF = 0x0000


def calc_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def append_crc(payload: bytes) -> bytes:
    crc = calc_crc16(payload)
    return payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def hex_bytes(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


class RelayError(RuntimeError):
    pass


class SerialPortNotFound(RelayError):
    pass


class MotionInterrupted(RuntimeError):
    pass


class RelayModbusClient:
    def __init__(
        self,
        port: str,
        baud: int,
        slave: int,
        timeout: float,
        modbus_mode: str,
        register_base_addr: int,
        register_on_value: int,
        register_off_value: int,
        verbose: bool,
        port_wait_timeout_sec: float,
        port_wait_interval_sec: float,
        open_retry_count: int,
        open_retry_interval_sec: float,
        logger=None,
    ) -> None:
        self.port = port
        self.baud = baud
        self.slave = slave
        self.timeout = timeout
        self.modbus_mode = modbus_mode
        self.register_base_addr = register_base_addr
        self.register_on_value = register_on_value
        self.register_off_value = register_off_value
        self.verbose = verbose
        self.port_wait_timeout_sec = max(0.0, port_wait_timeout_sec)
        self.port_wait_interval_sec = max(0.01, port_wait_interval_sec)
        self.open_retry_count = max(0, open_retry_count)
        self.open_retry_interval_sec = max(0.0, open_retry_interval_sec)
        self.logger = logger
        self._serial = None

    def open(self, wait_for_port: bool = True, retry: bool = True) -> None:
        if self._serial is not None and getattr(self._serial, "is_open", False):
            return
        try:
            import serial
        except ImportError as exc:
            raise RelayError("pyserial is required for lift relay control") from exc

        retry_count = self.open_retry_count if retry else 0
        last_exc = None
        for attempt in range(retry_count + 1):
            try:
                if wait_for_port:
                    self._wait_for_port()
                self._serial = serial.Serial(
                    port=self.port,
                    baudrate=self.baud,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=self.timeout,
                    write_timeout=self.timeout,
                )
                self._serial.reset_input_buffer()
                self._serial.reset_output_buffer()
                return
            except SerialPortNotFound:
                self._serial = None
                raise
            except Exception as exc:
                self._serial = None
                last_exc = exc
                if attempt >= retry_count:
                    break
                self._warn(f"open serial failed, retry {attempt + 1}/{retry_count}... {exc}")
                time.sleep(self.open_retry_interval_sec)
        raise RelayError(f"open serial failed: {last_exc}") from last_exc

    def _wait_for_port(self) -> None:
        if not self.port.startswith("/dev/") or os.path.exists(self.port):
            return
        deadline = time.monotonic() + self.port_wait_timeout_sec
        while time.monotonic() < deadline:
            time.sleep(self.port_wait_interval_sec)
            if os.path.exists(self.port):
                return
        raise SerialPortNotFound(
            f"serial port {self.port} not found after waiting {self.port_wait_timeout_sec:.1f} seconds")

    def _warn(self, message: str) -> None:
        if self.logger is not None:
            self.logger.warn(message)

    def close(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            finally:
                self._serial = None

    def reopen(self) -> None:
        self.close()
        self.open()

    def write_channel(self, channel: int, on: bool) -> None:
        if channel not in (1, 2, 3, 4):
            raise ValueError(f"relay channel must be 1..4, got {channel}")
        if self.modbus_mode == "register06":
            frame = self._build_write_register_frame(channel, on)
            expected_function = FUNC_WRITE_SINGLE_REGISTER
        else:
            frame = self._build_write_coil_frame(channel, on)
            expected_function = FUNC_WRITE_SINGLE_COIL
        self._write_and_check(frame, expected_function)

    def _build_write_coil_frame(self, channel: int, on: bool) -> bytes:
        coil_addr = channel - 1
        value = COIL_ON if on else COIL_OFF
        payload = bytes(
            [
                self.slave,
                FUNC_WRITE_SINGLE_COIL,
                (coil_addr >> 8) & 0xFF,
                coil_addr & 0xFF,
                (value >> 8) & 0xFF,
                value & 0xFF,
            ]
        )
        return append_crc(payload)

    def _build_write_register_frame(self, channel: int, on: bool) -> bytes:
        register_addr = self.register_base_addr + channel - 1
        value = self.register_on_value if on else self.register_off_value
        payload = bytes(
            [
                self.slave,
                FUNC_WRITE_SINGLE_REGISTER,
                (register_addr >> 8) & 0xFF,
                register_addr & 0xFF,
                (value >> 8) & 0xFF,
                value & 0xFF,
            ]
        )
        return append_crc(payload)

    def _write_and_check(self, frame: bytes, expected_function: int) -> None:
        if self._serial is None:
            self.open()
        if self._serial is None:
            raise RelayError("serial port is not open")

        try:
            self._serial.reset_input_buffer()
            if self.verbose:
                print(f"TX: {hex_bytes(frame)}")
            self._serial.write(frame)
            self._serial.flush()
            response = self._serial.read(8)
            if self.verbose:
                print(f"RX: {hex_bytes(response)}")
        except Exception as exc:
            self.close()
            raise RelayError(f"serial write/read failed: {exc}") from exc

        if len(response) != 8:
            raise RelayError(f"no or incomplete Modbus response: expected 8 bytes, got {len(response)}")
        payload = response[:-2]
        received_crc = response[-2] | (response[-1] << 8)
        calculated_crc = calc_crc16(payload)
        if received_crc != calculated_crc:
            raise RelayError(
                f"Modbus CRC mismatch: received=0x{received_crc:04X} calculated=0x{calculated_crc:04X}"
            )
        if response[0] != self.slave:
            raise RelayError(f"unexpected slave address: expected={self.slave} got={response[0]}")
        if response[1] != expected_function:
            raise RelayError(
                f"unexpected function code: expected=0x{expected_function:02X} got=0x{response[1]:02X}"
            )
        if response[:6] != frame[:6]:
            raise RelayError(f"unexpected Modbus echo: tx={hex_bytes(frame[:6])} rx={hex_bytes(response[:6])}")


@dataclass
class MotionCommand:
    direction: str
    duration_sec: float
    force_zero_on_complete: bool = False
    completed: threading.Event = None
    success: bool = False
    message: str = ""


class LiftRelayController(Node):
    def __init__(self) -> None:
        super().__init__("lift_relay_controller")
        self.port = self.declare_parameter("lift_default_port", "/dev/lift_relay").value
        self.baud = int(self.declare_parameter("lift_baud", 38400).value)
        self.slave = int(self.declare_parameter("lift_slave", 1).value)
        self.speed_mps = float(self.declare_parameter("lift_speed_mps", 0.026).value)
        self.min_height_m = float(self.declare_parameter("lift_min_height_m", 0.0).value)
        self.max_height_m = float(self.declare_parameter("lift_max_height_m", 1.5).value)
        self.manual_duration_sec = float(self.declare_parameter("lift_manual_duration_sec", 2.0).value)
        self.home_duration_sec = float(self.declare_parameter("lift_home_duration_sec", 2.0).value)
        self.home_force_zero_when_complete = bool(
            self.declare_parameter("lift_home_force_zero_when_complete", False).value)
        self.max_run_duration_sec = float(self.declare_parameter("lift_max_run_duration_sec", 2.0).value)
        self.settle_sec = float(self.declare_parameter("lift_settle_sec", 0.2).value)
        self.serial_timeout_sec = float(self.declare_parameter("lift_serial_timeout_sec", 1.0).value)
        self.reconnect_attempts = int(self.declare_parameter("lift_reconnect_attempts", 2).value)
        self.action_recover_attempts = int(self.declare_parameter("lift_action_recover_attempts", 5).value)
        self.port_wait_timeout_sec = float(self.declare_parameter("lift_port_wait_timeout_sec", 5.0).value)
        self.port_wait_interval_sec = float(self.declare_parameter("lift_port_wait_interval_sec", 0.2).value)
        self.open_retry_count = int(self.declare_parameter("lift_open_retry_count", 5).value)
        self.open_retry_interval_sec = float(self.declare_parameter("lift_open_retry_interval_sec", 0.5).value)
        self.modbus_mode = self.declare_parameter("lift_modbus_mode", "coil05").value
        self.register_base_addr = int(self.declare_parameter("lift_register_base_addr", 0).value)
        self.register_on_value = int(self.declare_parameter("lift_register_on_value", 1).value)
        self.register_off_value = int(self.declare_parameter("lift_register_off_value", 0).value)
        self.verbose_modbus = bool(self.declare_parameter("lift_verbose_modbus", False).value)
        self.state_topic = self.declare_parameter("lift_state_topic", "/lift/state").value

        self.speed_mps = max(0.001, self.speed_mps)
        self.min_height_m = min(self.min_height_m, self.max_height_m)
        self.max_height_m = max(self.max_height_m, self.min_height_m)
        self.manual_duration_sec = self._safe_duration(self.manual_duration_sec)
        self.home_duration_sec = self._safe_duration(self.home_duration_sec)
        self.max_run_duration_sec = self._safe_duration(self.max_run_duration_sec)
        self.settle_sec = max(0.0, min(self.settle_sec, 2.0))
        self.action_recover_attempts = max(0, self.action_recover_attempts)
        self.port_wait_timeout_sec = max(0.0, self.port_wait_timeout_sec)
        self.port_wait_interval_sec = max(0.01, self.port_wait_interval_sec)
        self.open_retry_count = max(0, self.open_retry_count)
        self.open_retry_interval_sec = max(0.0, self.open_retry_interval_sec)
        if self.modbus_mode not in ("coil05", "register06"):
            self.get_logger().warn(f"未知 lift_modbus_mode={self.modbus_mode}，回退到 coil05")
            self.modbus_mode = "coil05"

        self.client = RelayModbusClient(
            self.port,
            self.baud,
            self.slave,
            max(0.05, self.serial_timeout_sec),
            self.modbus_mode,
            self.register_base_addr,
            self.register_on_value,
            self.register_off_value,
            self.verbose_modbus,
            self.port_wait_timeout_sec,
            self.port_wait_interval_sec,
            self.open_retry_count,
            self.open_retry_interval_sec,
            self.get_logger(),
        )

        self.state_lock = threading.RLock()
        self.serial_lock = threading.RLock()
        self.stop_event = threading.Event()
        self.motion_thread: Optional[threading.Thread] = None
        self.estimated_height_m = self.min_height_m
        self.state = "IDLE"
        self.moving_direction = "none"
        self.is_moving = False
        self.error_message = ""
        self.y3_on = False
        self.last_update_time = time.monotonic()

        self.state_pub = self.create_publisher(LiftState, self.state_topic, 10)
        self.create_timer(0.2, self.publish_state)

        self.stop_srv = self.create_service(Trigger, "/lift/stop", self.handle_stop)
        self.all_off_srv = self.create_service(Trigger, "/lift/all_off", self.handle_all_off)
        self.reset_estimated_height_srv = self.create_service(
            Trigger, "/lift/reset_estimated_height", self.handle_reset_estimated_height)
        self.up_srv = self.create_service(LiftMoveTimed, "/lift/up", self.handle_up)
        self.down_srv = self.create_service(LiftMoveTimed, "/lift/down", self.handle_down)
        self.home_srv = self.create_service(LiftMoveTimed, "/lift/home", self.handle_home)
        self.move_timed_srv = self.create_service(LiftMoveTimed, "/lift/move_timed", self.handle_move_timed)
        self.move_to_height_srv = self.create_service(
            LiftMoveToHeight, "/lift/move_to_estimated_height", self.handle_move_to_height)

        try:
            self.client.open(wait_for_port=False, retry=False)
            self._safe_all_off_best_effort("startup")
            self.get_logger().info(
                f"升降杆继电器控制节点已启动 port={self.port} baud={self.baud} "
                f"slave={self.slave} mode={self.modbus_mode}"
            )
        except Exception as exc:
            self.get_logger().warn(f"启动串口或 all_off 未完成，节点保持运行等待后续重连: {exc}")
            with self.state_lock:
                self.error_message = f"startup serial unavailable: {exc}"
                self.state = "ERROR"

    def _safe_duration(self, value: float) -> float:
        if not math.isfinite(value) or value <= 0.0:
            return 2.0
        return min(value, 300.0)

    def _clamp_height(self, height: float) -> float:
        return min(self.max_height_m, max(self.min_height_m, height))

    def publish_state(self) -> None:
        msg = LiftState()
        with self.state_lock:
            msg.estimated_height_m = float(self.estimated_height_m)
            msg.moving_direction = self.moving_direction
            msg.is_moving = self.is_moving
            msg.state = self.state
            msg.error_message = self.error_message
        self.state_pub.publish(msg)

    def handle_up(self, request, response):
        duration = float(request.duration_sec) if request.duration_sec > 0.0 else self.manual_duration_sec
        return self._start_motion_response("up", duration, False, response)

    def handle_down(self, request, response):
        duration = float(request.duration_sec) if request.duration_sec > 0.0 else self.manual_duration_sec
        return self._start_motion_response("down", duration, False, response)

    def handle_home(self, request, response):
        duration = float(request.duration_sec) if request.duration_sec > 0.0 else self.home_duration_sec
        with self.state_lock:
            enough_to_home = duration * self.speed_mps >= max(0.0, self.estimated_height_m - self.min_height_m)
        return self._start_motion_response(
            "down",
            duration,
            self.home_force_zero_when_complete or enough_to_home,
            response,
        )

    def handle_move_timed(self, request, response):
        direction = request.direction.strip().lower()
        if direction not in ("up", "down"):
            response.success = False
            response.message = "direction must be up or down"
            response.estimated_height_m = self.estimated_height_m
            return response
        return self._start_motion_response(direction, float(request.duration_sec), False, response)

    def handle_move_to_height(self, request, response):
        target = self._clamp_height(float(request.target_height_m))
        with self.state_lock:
            delta = target - self.estimated_height_m
        if abs(delta) <= 0.001:
            response.success = True
            response.message = "已在目标预估高度"
            response.estimated_height_m = target
            return response
        direction = "up" if delta > 0.0 else "down"
        duration = abs(delta) / self.speed_mps
        return self._start_motion_response(direction, duration, False, response)

    def _start_motion_response(self, direction: str, duration: float, force_zero: bool, response):
        if direction not in ("up", "down"):
            response.success = False
            response.message = "direction must be up or down"
            response.estimated_height_m = self.estimated_height_m
            return response
        if not math.isfinite(duration) or duration <= 0.0:
            duration = self.manual_duration_sec
        duration = min(duration, self.max_run_duration_sec)

        with self.state_lock:
            if self.is_moving:
                response.success = False
                response.message = "升降杆正在运动，请先 stop"
                response.estimated_height_m = float(self.estimated_height_m)
                return response
            self.stop_event.clear()
            self.state = "MOVING_UP" if direction == "up" else "MOVING_DOWN"
            self.moving_direction = direction
            self.is_moving = True
            self.error_message = ""
            self.last_update_time = time.monotonic()
            command = MotionCommand(direction, duration, force_zero, threading.Event())
            self.motion_thread = threading.Thread(target=self._run_motion, args=(command,), daemon=True)
            self.motion_thread.start()

        per_recovery_timeout = (
            self.port_wait_timeout_sec
            + max(1, self.open_retry_count + 1) * max(self.open_retry_interval_sec, self.serial_timeout_sec)
            + 4.0 * self.settle_sec
            + 1.0
        )
        wait_timeout = duration + 2.0 * self.settle_sec + max(1.0, self.serial_timeout_sec) * 6
        wait_timeout += max(0, self.action_recover_attempts) * per_recovery_timeout
        if not command.completed.wait(wait_timeout):
            self.stop_event.set()
            self.request_stop("motion service timeout")
            response.success = False
            response.message = f"{direction}动作等待完成超时"
        else:
            response.success = command.success
            response.message = command.message
        with self.state_lock:
            response.estimated_height_m = float(self.estimated_height_m)
        return response

    def handle_stop(self, request, response):
        del request
        ok, message = self.request_stop("service stop")
        response.success = ok
        response.message = message
        return response

    def handle_all_off(self, request, response):
        del request
        ok, message = self.request_all_off("service all_off")
        response.success = ok
        response.message = message
        return response

    def handle_reset_estimated_height(self, request, response):
        del request
        with self.state_lock:
            if self.is_moving or self.state in ("MOVING_UP", "MOVING_DOWN"):
                response.success = False
                response.message = "lift is moving; please stop before reset estimated height"
                return response

            self.estimated_height_m = self.min_height_m
            self.moving_direction = "none"
            self.is_moving = False
            if self.state != "ERROR":
                self.state = "IDLE"

        self.get_logger().info("reset estimated height only, no motor movement")
        self.publish_state()
        response.success = True
        response.message = "estimated height reset to zero"
        return response

    def request_stop(self, reason: str) -> tuple[bool, str]:
        self.stop_event.set()
        try:
            self._write_y3(False)
            with self.state_lock:
                self.is_moving = False
                self.moving_direction = "none"
                if self.state != "ERROR":
                    self.state = "IDLE"
            return True, f"{reason}: Y3 OFF"
        except Exception as exc:
            self._set_error(f"{reason}: stop failed: {exc}")
            return False, str(exc)

    def request_all_off(self, reason: str) -> tuple[bool, str]:
        self.stop_event.set()
        try:
            self._all_off()
            with self.state_lock:
                self.is_moving = False
                self.moving_direction = "none"
                if self.state != "ERROR":
                    self.state = "IDLE"
            return True, f"{reason}: all channels OFF"
        except Exception as exc:
            self._set_error(f"{reason}: all_off failed: {exc}")
            return False, str(exc)

    def _run_motion(self, command: MotionCommand) -> None:
        target_duration = min(command.duration_sec, self.max_run_duration_sec)
        actual_motion_time = 0.0
        recovery_count = 0
        try:
            while rclpy.ok() and not self.stop_event.is_set() and actual_motion_time < target_duration:
                motor_running = False
                last_height_update: Optional[float] = None
                try:
                    self._prepare_direction(command.direction, restart_action_on_reconnect=True)
                    motor_running = True
                    last_height_update = time.monotonic()

                    while rclpy.ok() and not self.stop_event.is_set() and actual_motion_time < target_duration:
                        now = time.monotonic()
                        dt = min(
                            max(0.0, now - last_height_update),
                            max(0.0, target_duration - actual_motion_time),
                        )
                        if dt > 0.0:
                            self._integrate_height(command.direction, dt)
                            actual_motion_time += dt
                        last_height_update = now

                        remaining = target_duration - actual_motion_time
                        if remaining <= 0.0:
                            break
                        time.sleep(min(0.02, remaining))

                    if motor_running and last_height_update is not None:
                        now = time.monotonic()
                        dt = min(
                            max(0.0, now - last_height_update),
                            max(0.0, target_duration - actual_motion_time),
                        )
                        if dt > 0.0:
                            self._integrate_height(command.direction, dt)
                            actual_motion_time += dt
                    self._write_y3(False)
                    motor_running = False
                except MotionInterrupted:
                    raise
                except Exception as exc:
                    if motor_running and last_height_update is not None:
                        now = time.monotonic()
                        dt = min(
                            max(0.0, now - last_height_update),
                            max(0.0, target_duration - actual_motion_time),
                        )
                        if dt > 0.0:
                            self._integrate_height(command.direction, dt)
                            actual_motion_time += dt
                    if self.stop_event.is_set():
                        raise MotionInterrupted("升降动作被 stop/all_off 打断") from exc
                    recovered = False
                    last_recovery_error: Exception = exc
                    while recovery_count < self.action_recover_attempts and not self.stop_event.is_set():
                        recovery_count += 1
                        try:
                            self._recover_action_after_serial_error(
                                command.direction, exc, recovery_count, self.action_recover_attempts)
                            recovered = True
                            break
                        except Exception as recover_exc:
                            last_recovery_error = recover_exc
                            self.get_logger().warn(
                                f"{command.direction}动作级恢复 {recovery_count}/"
                                f"{self.action_recover_attempts} 未完成: {recover_exc}")
                    if not recovered:
                        raise RelayError(
                            f"{command.direction}动作恢复失败: 已重试 {recovery_count}/"
                            f"{self.action_recover_attempts} 次，实际运动 {actual_motion_time:.2f}s，"
                            f"最后错误: {last_recovery_error}"
                        ) from last_recovery_error

            if not rclpy.ok() or self.stop_event.is_set():
                raise MotionInterrupted("升降动作被 stop/all_off 打断")
            if command.force_zero_on_complete and not self.stop_event.is_set():
                with self.state_lock:
                    self.estimated_height_m = self.min_height_m
            with self.state_lock:
                if self.state != "ERROR":
                    self.state = "IDLE"
                    self.error_message = ""
                self.is_moving = False
                self.moving_direction = "none"
            if self.stop_event.is_set():
                command.success = False
                command.message = "升降动作被 stop/all_off 打断"
            else:
                command.success = True
                command.message = (
                    f"{command.direction}动作完成 actual_motion={actual_motion_time:.2f}s "
                    f"recoveries={recovery_count} height={self.estimated_height_m:.3f}m")
        except MotionInterrupted as exc:
            try:
                self._write_y3(False)
            except Exception as stop_exc:
                self._set_error(f"stop 打断后关闭 Y3 失败: {stop_exc}")
                command.success = False
                command.message = str(stop_exc)
            else:
                with self.state_lock:
                    if self.state != "ERROR":
                        self.state = "IDLE"
                    self.is_moving = False
                    self.moving_direction = "none"
                command.success = False
                command.message = str(exc)
        except Exception as exc:
            self._handle_motion_exception(exc)
            command.success = False
            command.message = str(exc)
        finally:
            command.completed.set()

    def _prepare_direction(self, direction: str, restart_action_on_reconnect: bool = False) -> None:
        self._write_y3(False, restart_action_on_reconnect=restart_action_on_reconnect)
        self._sleep_interruptible(self.settle_sec)
        if direction == "up":
            y1_on, y2_on = True, True
        else:
            y1_on, y2_on = False, False
        self._write_direction_channels(
            y1_on, y2_on, restart_action_on_reconnect=restart_action_on_reconnect)
        self._sleep_interruptible(self.settle_sec)
        self._write_y3(True, restart_action_on_reconnect=restart_action_on_reconnect)

    def _sleep_interruptible(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if self.stop_event.is_set():
                self._write_y3(False)
                raise MotionInterrupted("升降动作被 stop/all_off 打断")
            time.sleep(min(0.02, max(0.0, deadline - time.monotonic())))

    def _integrate_height(self, direction: str, dt: float) -> None:
        if dt <= 0.0 or not math.isfinite(dt):
            return
        delta = self.speed_mps * dt
        with self.state_lock:
            if direction == "up":
                self.estimated_height_m = self._clamp_height(self.estimated_height_m + delta)
            else:
                self.estimated_height_m = self._clamp_height(self.estimated_height_m - delta)

    def _recover_action_after_serial_error(
        self, direction: str, exc: Exception, attempt: int, total_attempts: int
    ) -> None:
        self._log_serial_error_hint(exc)
        self.get_logger().warn(
            f"{direction}动作串口异常，执行动作级恢复 {attempt}/{total_attempts}: {exc}")
        self.client.close()
        with self.serial_lock:
            self.client.open(wait_for_port=True, retry=True)
            self._all_off_after_reconnect()
        self.get_logger().warn(
            f"{direction}动作级恢复 {attempt}/{total_attempts} 成功: 已 all_off，继续剩余运动")

    def _log_serial_error_hint(self, exc: Exception) -> None:
        text = str(exc).lower()
        if "device disconnected or multiple access on port" in text or "returned no data" in text:
            self.get_logger().warn(
                "串口读写异常提示: 可能是 USB 掉线/重枚举、继电器 24V 供电不稳，"
                "或 diagnostic/test 工具及其他进程同时打开了同一串口。"
            )

    def _write_y3(self, on: bool, restart_action_on_reconnect: bool = False) -> None:
        with self.serial_lock:
            self._write_channel_with_reconnect(
                3, on, restart_action_on_reconnect=restart_action_on_reconnect)
            with self.state_lock:
                self.y3_on = on

    def _write_direction_channels(
        self, y1_on: bool, y2_on: bool, restart_action_on_reconnect: bool = False
    ) -> None:
        with self.serial_lock:
            with self.state_lock:
                if self.y3_on:
                    raise RelayError("refuse to switch Y1/Y2 while Y3 is ON")
            self._write_channel_with_reconnect(
                1, y1_on, restart_action_on_reconnect=restart_action_on_reconnect)
            self._write_channel_with_reconnect(
                2, y2_on, restart_action_on_reconnect=restart_action_on_reconnect)

    def _all_off(self) -> None:
        with self.serial_lock:
            self._write_channel_with_reconnect(3, False)
            with self.state_lock:
                self.y3_on = False
            self._write_channel_with_reconnect(1, False)
            self._write_channel_with_reconnect(2, False)
            self._write_channel_with_reconnect(4, False)

    def _write_channel_with_reconnect(
        self, channel: int, on: bool, restart_action_on_reconnect: bool = False
    ) -> None:
        last_exc: Optional[Exception] = None
        for attempt in range(max(1, self.reconnect_attempts + 1)):
            try:
                self.client.write_channel(channel, on)
                return
            except Exception as exc:
                last_exc = exc
                self._log_serial_error_hint(exc)
                self.client.close()
                if restart_action_on_reconnect:
                    raise RelayError(
                        f"channel {channel} write failed during motion safe sequence; "
                        f"restart full action sequence required: {exc}"
                    ) from exc
                if attempt >= self.reconnect_attempts:
                    break
                self.get_logger().warn(
                    f"relay serial error, reconnect {attempt + 1}/{self.reconnect_attempts}: {exc}")
                time.sleep(0.05)
                try:
                    self.client.reopen()
                    self._all_off_after_reconnect()
                except Exception as reconnect_exc:
                    last_exc = reconnect_exc
                    self._log_serial_error_hint(reconnect_exc)
                    self.get_logger().warn(
                        f"relay reconnect/all_off failed {attempt + 1}/{self.reconnect_attempts}: "
                        f"{reconnect_exc}")
        raise RelayError(str(last_exc))

    def _all_off_after_reconnect(self) -> None:
        self.client.write_channel(3, False)
        with self.state_lock:
            self.y3_on = False
        self.client.write_channel(1, False)
        self.client.write_channel(2, False)
        self.client.write_channel(4, False)

    def _handle_motion_exception(self, exc: Exception) -> None:
        message = f"升降动作异常: {exc}"
        try:
            self._write_y3(False)
            try:
                self._all_off()
            except Exception as all_off_exc:
                message += f"; all_off failed: {all_off_exc}"
        except Exception as stop_exc:
            message += f"; stop failed: {stop_exc}"
        self._set_error(message)

    def _safe_all_off_best_effort(self, reason: str) -> None:
        try:
            self._all_off()
        except Exception as exc:
            raise RelayError(f"{reason} all_off failed: {exc}") from exc

    def _set_error(self, message: str) -> None:
        with self.state_lock:
            self.error_message = message
            self.state = "ERROR"
            self.is_moving = False
            self.moving_direction = "none"
        self.get_logger().error(message)

    def shutdown_safely(self) -> None:
        ok, message = self.request_stop("shutdown")
        if not ok:
            self.get_logger().error(f"关闭节点 stop 失败: {message}")
            return
        ok, message = self.request_all_off("shutdown")
        if not ok:
            self.get_logger().error(f"关闭节点 all_off 失败: {message}")
        self.client.close()


def main(args=None) -> None:
    print(_COPYRIGHT)
    rclpy.init(args=args)
    node = LiftRelayController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().warn("收到 Ctrl+C，正在关闭升降杆继电器。")
    finally:
        node.shutdown_safely()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
