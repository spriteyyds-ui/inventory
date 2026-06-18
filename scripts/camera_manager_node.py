#!/usr/bin/env python3
"""
camera_manager_node.py
Manages left/right HJ (C100) USB camera processes for the inventory system.

Key behaviors:
- Only one HJ camera active at a time (left or right).
- Astra camera is NOT managed by this node.
- Switching side: stop old camera process, wait for device release,
  apply v4l2 exposure params, start new camera process, wait for first frame.
- Publishes status on /inventory/camera_manager/status (JSON string).
- Provides SwitchCamera service on /inventory/camera_manager/switch_camera.
- Cleans up child processes on shutdown.
"""

import json
import os
import signal
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

        # State
        self.lock_ = threading.Lock()
        self.requested_side_ = ""
        self.active_side_ = ""
        self.state_ = CAMERA_STATE_OFF
        self.error_message_ = ""
        self.process_ = None  # subprocess.Popen
        self.process_pid_ = None
        self.first_frame_received_ = False
        self.first_frame_time_ = None
        self.start_time_ = None
        self.switch_start_time_ = None
        self.active_device_ = ""
        self.active_topic_ = ""
        self.retry_count_ = 0

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

        # Publish initial status
        self._publish_status()

        self.get_logger().info(
            f"camera_manager started: left_device={self.left_device_} "
            f"right_device={self.right_device_} "
            f"left_topic={self.left_topic_} right_topic={self.right_topic_}")

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

    def _switch_camera_callback(self, request, response):
        """Handle SwitchCamera service request."""
        side = request.side.strip().lower()
        if side not in ("left", "right"):
            response.success = False
            response.message = f"Invalid side '{request.side}', must be 'left' or 'right'"
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

    def _stop_current_camera(self) -> bool:
        """Stop current camera process and wait for device release."""
        if self.process_ is None:
            return True

        old_side = self.active_side_
        old_pid = self.process_pid_
        self._log_and_set_state(CAMERA_STATE_STOPPING,
            f"Stopping {old_side} camera PID={old_pid}")

        stop_start = time.time()

        # Remove image subscriber
        if self.image_sub_ is not None:
            self.destroy_subscription(self.image_sub_)
            self.image_sub_ = None

        # SIGTERM
        try:
            if self.process_.poll() is None:
                os.kill(self.process_.pid, signal.SIGTERM)
                self.get_logger().info(
                    f"Sent SIGTERM to {old_side} camera PID={old_pid}")
        except ProcessLookupError:
            self.get_logger().info(
                f"{old_side} camera PID={old_pid} already exited")
            self.process_ = None
            self.process_pid_ = None
            self.active_side_ = ""
            self.active_device_ = ""
            self.first_frame_received_ = False
            return True

        # Wait for SIGTERM to take effect
        try:
            self.process_.wait(timeout=SIGTERM_WAIT_SEC)
            self.get_logger().info(
                f"{old_side} camera PID={old_pid} exited after SIGTERM")
        except subprocess.TimeoutExpired:
            self.get_logger().warn(
                f"{old_side} camera PID={old_pid} did not exit after "
                f"{SIGTERM_WAIT_SEC}s, sending SIGKILL")
            try:
                os.kill(self.process_.pid, signal.SIGKILL)
                self.process_.wait(timeout=SIGKILL_WAIT_SEC)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                pass

        self.process_ = None
        self.process_pid_ = None
        self.active_side_ = ""
        self.active_device_ = ""
        self.first_frame_received_ = False

        # Wait for device to be released
        old_device = self._device_for_side(old_side)
        if old_device:
            self._wait_device_released(old_device, old_side)

        stop_elapsed = time.time() - stop_start
        self.get_logger().info(
            f"Camera {old_side} PID={old_pid} stopped, "
            f"device release took {stop_elapsed:.2f}s")
        return True

    def _wait_device_released(self, device: str, side: str):
        """Wait until the device is no longer busy."""
        deadline = time.time() + DEVICE_RELEASE_WAIT_SEC
        while time.time() < deadline:
            # Check if any process has the device open
            try:
                result = subprocess.run(
                    ["fuser", device],
                    capture_output=True, text=True, timeout=2)
                if result.returncode != 0:
                    # fuser returns 1 if no process has the device
                    self.get_logger().info(
                        f"Device {device} ({side}) released")
                    return
            except (subprocess.TimeoutExpired, FileNotFoundError):
                # fuser not available or timed out, try alternative
                pass

            # Alternative: check if device exists and is accessible
            if not os.path.exists(device):
                self.get_logger().info(
                    f"Device {device} ({side}) no longer exists, "
                    f"considering released")
                return

            time.sleep(DEVICE_RELEASE_POLL_SEC)

        self.get_logger().warn(
            f"Device {device} ({side}) may still be busy after "
            f"{DEVICE_RELEASE_WAIT_SEC}s, proceeding anyway")

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

        # Readback for verification
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

    def _start_camera(self, side: str) -> bool:
        """Start usb_cam process for the given side and wait for first frame."""
        device = self._device_for_side(side)
        topic = self._topic_for_side(side)

        # Apply v4l2 controls before starting usb_cam
        if not self._apply_v4l2_controls(device, side):
            self.get_logger().error(
                f"v4l2 control application failed for {side}")
            return False

        # Build usb_cam command
        cmd = [
            "ros2", "run", "usb_cam", "usb_cam_node_exe",
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

        with self.lock_:
            self.process_ = proc
            self.process_pid_ = proc.pid
            self.active_device_ = device
            self.active_topic_ = topic
            self.first_frame_received_ = False
            self.first_frame_time_ = None
            self.start_time_ = start_time

        self.get_logger().info(
            f"{side} camera process started PID={proc.pid}")

        # Wait for process to be alive
        alive_deadline = time.time() + self.startup_timeout_sec_
        while time.time() < alive_deadline:
            if proc.poll() is not None:
                # Process exited prematurely
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
            # Check if the topic is being published
            time.sleep(0.5)
            # Give it a moment to start publishing
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
                    f"Camera {side} READY: PID={proc.pid} device={device} "
                    f"topic={topic} first_frame_at={self.first_frame_time_ - start_time:.2f}s "
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

        # Wait for first frame
        deadline = time.time() + self.first_frame_timeout_sec_
        while time.time() < deadline:
            with self.lock_:
                if self.first_frame_received_:
                    return
                # Check if process is still alive
                if self.process_ and self.process_.poll() is not None:
                    self.get_logger().error(
                        f"Camera {side} process exited while waiting "
                        f"for first frame")
                    return
            time.sleep(0.1)

        self.get_logger().error(
            f"First frame timeout for {side} camera after "
            f"{self.first_frame_timeout_sec_}s")

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

    def _kill_process_force(self):
        """Force kill the current process."""
        if self.process_ is None:
            return
        try:
            if self.process_.poll() is None:
                os.kill(self.process_.pid, signal.SIGKILL)
                try:
                    self.process_.wait(timeout=SIGKILL_WAIT_SEC)
                except subprocess.TimeoutExpired:
                    pass
        except ProcessLookupError:
            pass

        if self.image_sub_ is not None:
            self.destroy_subscription(self.image_sub_)
            self.image_sub_ = None

        self.process_ = None
        self.process_pid_ = None
        self.active_side_ = ""
        self.active_device_ = ""
        self.first_frame_received_ = False

    def cleanup(self):
        """Clean up all child processes on shutdown."""
        self.get_logger().info("camera_manager cleanup: stopping camera process")
        with self.lock_:
            if self.process_ is not None and self.process_.poll() is None:
                try:
                    os.kill(self.process_.pid, signal.SIGTERM)
                    try:
                        self.process_.wait(timeout=5.0)
                    except subprocess.TimeoutExpired:
                        os.kill(self.process_.pid, signal.SIGKILL)
                        try:
                            self.process_.wait(timeout=2.0)
                        except subprocess.TimeoutExpired:
                            pass
                except ProcessLookupError:
                    pass
            self.process_ = None
            self.process_pid_ = None
            self._set_state_internal(CAMERA_STATE_OFF)
        self.get_logger().info("camera_manager cleanup complete")


def main(args=None):
    rclpy.init(args=args)
    node = CameraManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
