#!/usr/bin/env python3
# Copyright (c) 2026 郁有冬 <spriteyyds@gmail.com>. All rights reserved.
_COPYRIGHT = (
    "========================================\n"
    " agv_inventory_system\n"
    " Copyright (c) 2026 郁有冬 <spriteyyds@gmail.com>\n"
    " All rights reserved.\n"
    "========================================\n"
)
"""
camera_manager_node.py
Manages left/right HJ (C100) USB camera processes for the inventory system.

Key behaviors:
- Only one HJ camera active at a time (left or right), or OFF.
- Astra camera is NOT managed by this node.
- Switching side: stop old camera process, wait for device release,
  apply v4l2 exposure params, start new camera process, wait for first frame.
- "off" side: stop current camera and release device.
- On startup: detect and clean up residual camera processes from this project.
- Uses direct binary (usb_cam_node_exe) instead of `ros2 run` for accurate PID.
- Saves PID and PGID; kills entire process group on stop.
- Cleanup is idempotent and safe against repeated calls.
- Publishes status on /inventory/camera_manager/status (JSON string).
- Provides SwitchCamera service on /inventory/camera_manager/switch_camera.
"""

import glob
import json
import os
import signal
import shutil
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from agv_inventory_system.srv import SwitchCamera


# Camera state enum values
CAMERA_STATE_OFF = "OFF"
CAMERA_STATE_STARTING = "STARTING"
CAMERA_STATE_READY = "READY"
CAMERA_STATE_STOPPING = "STOPPING"
CAMERA_STATE_ERROR = "ERROR"

# V4L2 controls to apply before starting usb_cam
V4L2_CONTROLS = [
    ("white_balance_automatic", 1),
    ("auto_exposure", 1),
    ("exposure_dynamic_framerate", 0),
    ("exposure_time_absolute", 35),
    ("brightness", 40),
    ("contrast", 40),
    ("sharpness", 3),
    ("gain", 0),
    ("backlight_compensation", 0),
    ("gamma", 100),
    ("power_line_frequency", 1),
]

# Process management constants
SIGTERM_WAIT_SEC = 3.0
SIGKILL_WAIT_SEC = 2.0
DEVICE_RELEASE_WAIT_SEC = 3.0
DEVICE_RELEASE_POLL_SEC = 0.2
STARTUP_TIMEOUT_SEC = 10.0
STARTUP_RETRY_COUNT = 1
FIRST_FRAME_TIMEOUT_SEC = 10.0

# Known binary name for identity check
USB_CAM_NODE_EXE = "usb_cam_node_exe"


def _find_usb_cam_binary() -> str:
    """Find usb_cam_node_exe binary path."""
    # 1. Try shutil.which (covers PATH-sourced environments)
    found = shutil.which("usb_cam_node_exe")
    if found:
        return found
    # 2. Try common install locations
    candidates = glob.glob(
        "/home/wheeltec/wheeltec_ros2/install/usb_cam/lib/usb_cam/usb_cam_node_exe"
    )
    for c in candidates:
        if os.path.isfile(c) and os.access(c, os.X_OK):
            return c
    # 3. Fallback: hope it's on PATH at runtime
    return "usb_cam_node_exe"


def _is_known_camera_process(pid: int, left_device: str, right_device: str) -> bool:
    """Check if a PID is a known usb_cam_node_exe process from this project."""
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            cmdline = f.read().decode("utf-8", errors="replace").replace("\x00", " ").strip()
    except (FileNotFoundError, PermissionError):
        return False

    # Must contain usb_cam_node_exe in the command
    if USB_CAM_NODE_EXE not in cmdline:
        return False

    # Must reference one of our camera devices
    if left_device and left_device in cmdline:
        return True
    if right_device and right_device in cmdline:
        return True

    return False


def _detect_residual_cameras(left_device: str, right_device: str, logger) -> list:
    """
    Scan /proc for processes holding left/right HJ camera devices.
    Returns list of (pid, device, cmdline) for known residual processes.
    Raises RuntimeError if unknown process holds a device.
    """
    residuals = []

    for device in (left_device, right_device):
        if not device or not os.path.exists(device):
            continue

        try:
            result = subprocess.run(
                ["fuser", device],
                capture_output=True, text=True, timeout=3)
        except (subprocess.TimeoutExpired, FileNotFoundError):
            continue

        if result.returncode != 0:
            # fuser returns 1 if no process has the device
            continue

        # Parse PIDs from fuser output (space-separated, may have trailing chars)
        pids_raw = result.stdout.strip().split()
        for pid_str in pids_raw:
            # fuser output may include suffix like "/dev/video0:  12345 "
            pid_clean = pid_str.strip().rstrip("r").rstrip("c")
            try:
                pid = int(pid_clean)
            except ValueError:
                continue

            try:
                with open(f"/proc/{pid}/cmdline", "rb") as f:
                    cmdline = f.read().decode("utf-8", errors="replace").replace("\x00", " ").strip()
            except (FileNotFoundError, PermissionError):
                continue

            if _is_known_camera_process(pid, left_device, right_device):
                logger.warn(
                    f"[startup] 发现残留 HJ 相机进程: PID={pid} device={device} "
                    f"cmdline={cmdline[:200]}")
                residuals.append((pid, device, cmdline))
            else:
                # Unknown process holds the device — do NOT auto-kill
                raise RuntimeError(
                    f"设备 {device} 被未知进程占用，无法自动清理。"
                    f" PID={pid} cmdline={cmdline[:200]}。"
                    f"请手动检查并释放设备后重试。")

    return residuals


class CameraManagerNode(Node):
    def __init__(self):
        super().__init__('camera_manager_node')

        # Parameters
        self.declare_parameter('left_camera_device',
            '/dev/v4l/by-path/platform-3610000.usb-usb-0:2.4.2:1.0-video-index0')
        self.declare_parameter('right_camera_device',
            '/dev/v4l/by-path/platform-3610000.usb-usb-0:2.4.1:1.0-video-index0')
        self.declare_parameter('left_camera_topic', '/c100_left/image_raw')
        self.declare_parameter('right_camera_topic', '/c100_right/image_raw')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('pixel_format', 'mjpeg2rgb')
        self.declare_parameter('startup_timeout_sec', 10.0)
        self.declare_parameter('startup_retry_count', 1)
        self.declare_parameter('first_frame_timeout_sec', 10.0)

        self.left_device_ = self.get_parameter('left_camera_device').value
        self.right_device_ = self.get_parameter('right_camera_device').value
        self.left_topic_ = self.get_parameter('left_camera_topic').value
        self.right_topic_ = self.get_parameter('right_camera_topic').value
        self.image_width_ = self.get_parameter('image_width').value
        self.image_height_ = self.get_parameter('image_height').value
        self.pixel_format_ = self.get_parameter('pixel_format').value
        self.startup_timeout_sec_ = self.get_parameter('startup_timeout_sec').value
        self.startup_retry_count_ = self.get_parameter('startup_retry_count').value
        self.first_frame_timeout_sec_ = self.get_parameter('first_frame_timeout_sec').value

        # Resolve binary path
        self.usb_cam_binary_ = _find_usb_cam_binary()
        self.get_logger().info(
            f"usb_cam binary: {self.usb_cam_binary_}")

        # State
        self.lock_ = threading.Lock()
        self.requested_side_ = ""
        self.active_side_ = ""
        self.state_ = CAMERA_STATE_OFF
        self.error_message_ = ""
        self.process_ = None  # subprocess.Popen
        self.process_pid_ = None
        self.process_pgid_ = None
        self.first_frame_received_ = False
        self.first_frame_time_ = None
        self.start_time_ = None
        self.switch_start_time_ = None
        self.active_device_ = ""
        self.active_topic_ = ""
        self.retry_count_ = 0
        self._cleanup_done = False

        # Status publisher (transient local so late subscribers get last status)
        control_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub_ = self.create_publisher(String,
            '/inventory/camera_manager/status', control_qos)

        # SwitchCamera service
        self.srv_ = self.create_service(
            SwitchCamera, '/inventory/camera_manager/switch_camera',
            self._switch_camera_callback)

        # Image subscriber (created dynamically for active camera)
        self.image_sub_ = None

        # Startup residual detection and cleanup
        self._startup_residual_cleanup()

        # Publish initial status
        self._publish_status()

        self.get_logger().info(
            f"camera_manager started: left_device={self.left_device_} "
            f"right_device={self.right_device_} "
            f"left_topic={self.left_topic_} right_topic={self.right_topic_}")

    # ------------------------------------------------------------------
    # Startup residual detection
    # ------------------------------------------------------------------

    def _startup_residual_cleanup(self):
        """Detect and clean up residual HJ camera processes from this project."""
        self.get_logger().info("[startup] 检查残留 HJ 相机进程...")
        try:
            residuals = _detect_residual_cameras(
                self.left_device_, self.right_device_, self.get_logger())
        except RuntimeError as e:
            self.get_logger().error(f"[startup] 残留检测失败: {e}")
            self._set_state_internal(CAMERA_STATE_ERROR, str(e))
            return

        if not residuals:
            self.get_logger().info("[startup] 无残留 HJ 相机进程")
            return

        for pid, device, cmdline in residuals:
            self.get_logger().warn(
                f"[startup] 清理残留进程: PID={pid} device={device}")
            self._kill_pid(pid, f"residual({device})")

        # Wait for devices to be released
        for pid, device, _ in residuals:
            self._wait_device_released(device, "residual")

        self.get_logger().info(
            f"[startup] 残留清理完成，共清理 {len(residuals)} 个进程")

    # ------------------------------------------------------------------
    # Device helpers
    # ------------------------------------------------------------------

    def _device_for_side(self, side: str) -> str:
        if side == "left":
            return self.left_device_
        return self.right_device_

    def _topic_for_side(self, side: str) -> str:
        if side == "left":
            return self.left_topic_
        return self.right_topic_

    def _publish_status(self):
        msg = String()
        msg.data = json.dumps({
            "requested_side": self.requested_side_,
            "active_side": self.active_side_,
            "state": self.state_,
            "error_message": self.error_message_,
        })
        self.status_pub_.publish(msg)

    def _set_state(self, state: str, error: str = ""):
        self.state_ = state
        self.error_message_ = error
        self._publish_status()

    def _set_state_internal(self, state: str, error: str = ""):
        """Set state without lock (caller must hold lock)."""
        self.state_ = state
        self.error_message_ = error
        self._publish_status()

    def _log_and_set_state(self, state: str, message: str):
        """Log and set state thread-safely."""
        self.get_logger().info(f"[camera_manager] {state}: {message}")
        with self.lock_:
            self._set_state_internal(state, message)

    # ------------------------------------------------------------------
    # Service callback
    # ------------------------------------------------------------------

    def _switch_camera_callback(self, request, response):
        """Handle SwitchCamera service request."""
        side = request.side.strip().lower()
        if side not in ("left", "right", "off"):
            response.success = False
            response.message = f"Invalid side '{request.side}', must be 'left', 'right', or 'off'"
            return response

        # Handle "off" — stop current camera synchronously
        if side == "off":
            self.get_logger().info("[camera_manager] 收到 camera off 请求")
            with self.lock_:
                if self.process_ is None:
                    self._set_state_internal(CAMERA_STATE_OFF)
                    response.success = True
                    response.message = "Camera already OFF"
                    return response

            # Stop outside lock
            self._stop_current_camera()
            with self.lock_:
                self._set_state_internal(CAMERA_STATE_OFF)
            response.success = True
            response.message = "Camera stopped and device released"
            return response

        with self.lock_:
            # If already on requested side and READY, return immediately
            if (self.active_side_ == side and self.state_ == CAMERA_STATE_READY
                    and self.process_ is not None and self.process_.poll() is None):
                self.get_logger().info(
                    f"switch_camera({side}): already READY on {side}, immediate success")
                response.success = True
                response.message = f"Already READY on {side}"
                return response

            # If currently STARTING or STOPPING for the same side, reject
            if self.state_ in (CAMERA_STATE_STARTING, CAMERA_STATE_STOPPING):
                response.success = False
                response.message = (
                    f"Camera is currently {self.state_}, "
                    f"requested_side={self.requested_side_}")
                return response

        # Perform switch in background thread to not block service response
        self.get_logger().info(
            f"switch_camera({side}): current state={self.state_} "
            f"active_side={self.active_side_}, starting async switch")
        thread = threading.Thread(target=self._do_switch, args=(side,),
            daemon=True)
        thread.start()

        response.success = True
        response.message = f"Switch to {side} initiated"
        return response

    # ------------------------------------------------------------------
    # Switch logic
    # ------------------------------------------------------------------

    def _do_switch(self, side: str):
        """Perform camera switch in background thread."""
        with self.lock_:
            self.requested_side_ = side
            self.switch_start_time_ = time.time()
            self.retry_count_ = 0

        self._log_and_set_state(CAMERA_STATE_STARTING,
            f"Starting switch to {side}")

        # Step 1: Stop old camera if running
        if not self._stop_current_camera():
            return  # Error state already set

        # Step 2: Apply v4l2 controls and start new camera (with retry)
        for attempt in range(1 + self.startup_retry_count_):
            self.retry_count_ = attempt
            if attempt > 0:
                self.get_logger().warn(
                    f"switch_camera({side}): retry attempt {attempt}/"
                    f"{self.startup_retry_count_}")

            if self._start_camera(side):
                # Success
                return

            # Failed - stop and retry
            self._kill_process_force()
            if attempt < self.startup_retry_count_:
                time.sleep(1.0)

        # All retries exhausted
        self._log_and_set_state(CAMERA_STATE_ERROR,
            f"Failed to start {side} camera after "
            f"{1 + self.startup_retry_count_} attempts")

    # ------------------------------------------------------------------
    # Stop camera
    # ------------------------------------------------------------------

    def _stop_current_camera(self) -> bool:
        """Stop current camera process and wait for device release."""
        if self.process_ is None:
            return True

        old_side = self.active_side_
        old_pid = self.process_pid_
        old_pgid = self.process_pgid_
        self._log_and_set_state(CAMERA_STATE_STOPPING,
            f"Stopping {old_side} camera PID={old_pid} PGID={old_pgid}")

        stop_start = time.time()

        # Remove image subscriber
        if self.image_sub_ is not None:
            self.destroy_subscription(self.image_sub_)
            self.image_sub_ = None

        # Send SIGTERM to process group
        self._kill_pid(old_pid, old_side, pgid=old_pgid, sig=signal.SIGTERM,
                        label="SIGTERM")

        # Wait for SIGTERM to take effect
        try:
            self.process_.wait(timeout=SIGTERM_WAIT_SEC)
            self.get_logger().info(
                f"{old_side} camera PID={old_pid} exited after SIGTERM")
        except subprocess.TimeoutExpired:
            self.get_logger().warn(
                f"{old_side} camera PID={old_pid} did not exit after "
                f"{SIGTERM_WAIT_SEC}s, sending SIGKILL to PGID={old_pgid}")
            self._kill_pid(old_pid, old_side, pgid=old_pgid, sig=signal.SIGKILL,
                            label="SIGKILL")
            try:
                self.process_.wait(timeout=SIGKILL_WAIT_SEC)
            except subprocess.TimeoutExpired:
                self.get_logger().error(
                    f"{old_side} camera PID={old_pid} still alive after SIGKILL")

        self.process_ = None
        self.process_pid_ = None
        self.process_pgid_ = None
        self.active_side_ = ""
        self.active_device_ = ""
        self.first_frame_received_ = False

        # Wait for device to be released
        old_device = self._device_for_side(old_side)
        if old_device:
            self._wait_device_released(old_device, old_side)

        stop_elapsed = time.time() - stop_start
        self.get_logger().info(
            f"[shutdown] Camera {old_side} PID={old_pid} PGID={old_pgid} stopped, "
            f"device release took {stop_elapsed:.2f}s")
        return True

    def _kill_pid(self, pid: int, side: str, pgid: int = None,
                  sig: int = signal.SIGTERM, label: str = "SIGTERM"):
        """Send signal to process and optionally its process group."""
        if pgid is not None and pgid > 0:
            try:
                os.killpg(pgid, sig)
                self.get_logger().info(
                    f"[shutdown] Sent {label} to PGID={pgid} ({side})")
            except ProcessLookupError:
                self.get_logger().info(
                    f"[shutdown] PGID={pgid} ({side}) already gone")
            except PermissionError as e:
                self.get_logger().warn(
                    f"[shutdown] Permission denied sending {label} to PGID={pgid}: {e}")
        else:
            try:
                os.kill(pid, sig)
                self.get_logger().info(
                    f"[shutdown] Sent {label} to PID={pid} ({side})")
            except ProcessLookupError:
                self.get_logger().info(
                    f"[shutdown] PID={pid} ({side}) already gone")

    def _wait_device_released(self, device: str, side: str):
        """Wait until the device is no longer busy."""
        deadline = time.time() + DEVICE_RELEASE_WAIT_SEC
        while time.time() < deadline:
            try:
                result = subprocess.run(
                    ["fuser", device],
                    capture_output=True, text=True, timeout=2)
                if result.returncode != 0:
                    self.get_logger().info(
                        f"[shutdown] Device {device} ({side}) released")
                    return
            except (subprocess.TimeoutExpired, FileNotFoundError):
                pass

            if not os.path.exists(device):
                self.get_logger().info(
                    f"[shutdown] Device {device} ({side}) no longer exists, "
                    f"considering released")
                return

            time.sleep(DEVICE_RELEASE_POLL_SEC)

        # Final check
        try:
            result = subprocess.run(
                ["fuser", device],
                capture_output=True, text=True, timeout=2)
            if result.returncode != 0:
                self.get_logger().info(
                    f"[shutdown] Device {device} ({side}) released (late)")
                return
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass

        self.get_logger().warn(
            f"[shutdown] Device {device} ({side}) may still be busy after "
            f"{DEVICE_RELEASE_WAIT_SEC}s")

    # ------------------------------------------------------------------
    # V4L2 controls
    # ------------------------------------------------------------------

    def _apply_v4l2_controls(self, device: str, side: str) -> bool:
        """Apply v4l2 exposure parameters to the device."""
        if not os.path.exists(device):
            self.get_logger().error(
                f"Device {device} ({side}) not found for v4l2 setup")
            return False

        self.get_logger().info(
            f"Applying v4l2 controls to {side} camera device={device}")
        success_count = 0
        for name, value in V4L2_CONTROLS:
            try:
                result = subprocess.run(
                    ["v4l2-ctl", f"--device={device}",
                     f"--set-ctrl={name}={value}"],
                    capture_output=True, text=True, timeout=5)
                if result.returncode == 0:
                    success_count += 1
                else:
                    self.get_logger().warn(
                        f"v4l2 set {name}={value} failed on {side}: "
                        f"{result.stderr.strip()}")
            except (subprocess.TimeoutExpired, FileNotFoundError) as e:
                self.get_logger().warn(
                    f"v4l2 set {name}={value} error on {side}: {e}")

        self.get_logger().info(
            f"v4l2 controls applied to {side}: "
            f"{success_count}/{len(V4L2_CONTROLS)} succeeded")
        for name, _ in V4L2_CONTROLS:
            try:
                result = subprocess.run(
                    ["v4l2-ctl", f"--device={device}",
                     f"--get-ctrl={name}"],
                    capture_output=True, text=True, timeout=5)
                if result.returncode == 0:
                    self.get_logger().info(
                        f"  {side} {result.stdout.strip()}")
            except (subprocess.TimeoutExpired, FileNotFoundError):
                pass

        return True

    # ------------------------------------------------------------------
    # Start camera (direct binary)
    # ------------------------------------------------------------------

    def _start_camera(self, side: str) -> bool:
        """Start usb_cam process for the given side and wait for first frame."""
        device = self._device_for_side(side)
        topic = self._topic_for_side(side)

        # Apply v4l2 controls before starting usb_cam
        if not self._apply_v4l2_controls(device, side):
            self.get_logger().error(
                f"v4l2 control application failed for {side}")
            return False

        # Build command — use direct binary for accurate PID tracking
        cmd = [
            self.usb_cam_binary_,
            "--ros-args",
            "-r", f"__node:=c100_{side}_camera",
            "-r", f"image_raw:={topic}",
            "-r", f"camera_info:=/c100_{side}/camera_info",
            "-p", f"video_device:={device}",
            "-p", f"camera_name:=c100_{side}",
            "-p", f"image_width:={self.image_width_}",
            "-p", f"image_height:={self.image_height_}",
            "-p", "framerate:=30.0",
            "-p", f"pixel_format:={self.pixel_format_}",
        ]

        self.get_logger().info(
            f"Starting {side} camera: device={device} topic={topic} "
            f"cmd={' '.join(cmd)}")
        start_time = time.time()

        try:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid)
        except Exception as e:
            self.get_logger().error(
                f"Failed to start {side} camera process: {e}")
            return False

        pgid = os.getpgid(proc.pid)
        self.get_logger().info(
            f"{side} camera process started PID={proc.pid} PGID={pgid}")

        with self.lock_:
            self.process_ = proc
            self.process_pid_ = proc.pid
            self.process_pgid_ = pgid
            self.active_device_ = device
            self.active_topic_ = topic
            self.first_frame_received_ = False
            self.first_frame_time_ = None
            self.start_time_ = start_time

        # Wait for process to be alive
        alive_deadline = time.time() + self.startup_timeout_sec_
        while time.time() < alive_deadline:
            if proc.poll() is not None:
                stderr_output = ""
                try:
                    _, stderr_data = proc.communicate(timeout=1)
                    stderr_output = stderr_data.decode('utf-8', errors='replace')[:500]
                except Exception:
                    pass
                self.get_logger().error(
                    f"{side} camera PID={proc.pid} exited prematurely "
                    f"code={proc.returncode} stderr={stderr_output}")
                return False
            time.sleep(0.5)
            if time.time() - start_time > 2.0:
                break

        # Subscribe to the image topic and wait for first frame
        self._wait_for_first_frame(side, topic)

        with self.lock_:
            if self.first_frame_received_:
                self.active_side_ = side
                self._set_state_internal(CAMERA_STATE_READY)
                total_elapsed = time.time() - self.switch_start_time_
                self.get_logger().info(
                    f"[camera_manager] READY: side={side} PID={proc.pid} PGID={pgid} "
                    f"device={device} topic={topic} "
                    f"first_frame_at={self.first_frame_time_ - start_time:.2f}s "
                    f"total_switch={total_elapsed:.2f}s "
                    f"retry_count={self.retry_count_}")
                return True
            else:
                self.get_logger().error(
                    f"Camera {side} no first frame received within "
                    f"{self.first_frame_timeout_sec_}s")
                return False

    def _wait_for_first_frame(self, side: str, topic: str):
        """Subscribe to image topic and wait for first frame."""
        self.get_logger().info(
            f"Waiting for first frame on {topic} for {side} camera...")

        def image_callback(msg):
            with self.lock_:
                if not self.first_frame_received_:
                    self.first_frame_received_ = True
                    self.first_frame_time_ = time.time()
                    elapsed = self.first_frame_time_ - self.start_time_
                    self.get_logger().info(
                        f"First frame received for {side} camera: "
                        f"size={msg.width}x{msg.height} "
                        f"encoding={msg.encoding} "
                        f"elapsed={elapsed:.2f}s")

        self.image_sub_ = self.create_subscription(
            Image, topic, image_callback, QoSProfile(depth=1))

        deadline = time.time() + self.first_frame_timeout_sec_
        while time.time() < deadline:
            with self.lock_:
                if self.first_frame_received_:
                    return
                if self.process_ and self.process_.poll() is not None:
                    self.get_logger().error(
                        f"Camera {side} process exited while waiting "
                        f"for first frame")
                    return
            time.sleep(0.1)

        self.get_logger().error(
            f"First frame timeout for {side} camera after "
            f"{self.first_frame_timeout_sec_}s")

    # ------------------------------------------------------------------
    # Force kill
    # ------------------------------------------------------------------

    def _kill_process_force(self):
        """Force kill the current process and its process group."""
        if self.process_ is None:
            return

        pid = self.process_pid_
        pgid = self.process_pgid_

        # SIGKILL to process group
        if pgid is not None and pgid > 0:
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        elif pid is not None:
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

        try:
            self.process_.wait(timeout=SIGKILL_WAIT_SEC)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            pass

        if self.image_sub_ is not None:
            self.destroy_subscription(self.image_sub_)
            self.image_sub_ = None

        self.process_ = None
        self.process_pid_ = None
        self.process_pgid_ = None
        self.active_side_ = ""
        self.active_device_ = ""
        self.first_frame_received_ = False

    # ------------------------------------------------------------------
    # Cleanup (idempotent)
    # ------------------------------------------------------------------

    def cleanup(self):
        """Clean up all child processes on shutdown. Idempotent."""
        if self._cleanup_done:
            self.get_logger().info("[shutdown] cleanup already done, skipping")
            return
        self._cleanup_done = True

        self.get_logger().info("[shutdown] camera_manager cleanup: starting")

        with self.lock_:
            proc = self.process_
            pid = self.process_pid_
            pgid = self.process_pgid_
            side = self.active_side_

        if proc is not None and proc.poll() is None:
            self.get_logger().info(
                f"[shutdown] Stopping active camera: side={side} PID={pid} PGID={pgid}")

            # SIGTERM to process group
            if pgid is not None and pgid > 0:
                try:
                    os.killpg(pgid, signal.SIGTERM)
                    self.get_logger().info(
                        f"[shutdown] Sent SIGTERM to PGID={pgid}")
                except ProcessLookupError:
                    self.get_logger().info(
                        f"[shutdown] PGID={pgid} already gone")
            elif pid is not None:
                try:
                    os.kill(pid, signal.SIGTERM)
                    self.get_logger().info(
                        f"[shutdown] Sent SIGTERM to PID={pid}")
                except ProcessLookupError:
                    pass

            try:
                proc.wait(timeout=SIGTERM_WAIT_SEC)
                self.get_logger().info(
                    f"[shutdown] Camera PID={pid} exited after SIGTERM")
            except subprocess.TimeoutExpired:
                self.get_logger().warn(
                    f"[shutdown] Camera PID={pid} did not exit after "
                    f"{SIGTERM_WAIT_SEC}s, sending SIGKILL")
                if pgid is not None and pgid > 0:
                    try:
                        os.killpg(pgid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                elif pid is not None:
                    try:
                        os.kill(pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                try:
                    proc.wait(timeout=SIGKILL_WAIT_SEC)
                    self.get_logger().info(
                        f"[shutdown] Camera PID={pid} exited after SIGKILL")
                except subprocess.TimeoutExpired:
                    self.get_logger().error(
                        f"[shutdown] Camera PID={pid} still alive after SIGKILL!")

            # Verify device release
            if side:
                device = self._device_for_side(side)
                if device:
                    self._wait_device_released(device, side)
                    self.get_logger().info(
                        f"[shutdown] Device {device} ({side}) release check done")
        else:
            self.get_logger().info("[shutdown] No active camera process to stop")

        with self.lock_:
            self.process_ = None
            self.process_pid_ = None
            self.process_pgid_ = None
            self._set_state_internal(CAMERA_STATE_OFF)

        if self.image_sub_ is not None:
            self.destroy_subscription(self.image_sub_)
            self.image_sub_ = None

        self.get_logger().info(
            f"[shutdown] camera_manager cleanup complete, "
            f"final state={CAMERA_STATE_OFF}")


def main(args=None):
    print(_COPYRIGHT)
    rclpy.init(args=args)
    node = CameraManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[shutdown] KeyboardInterrupt received")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
