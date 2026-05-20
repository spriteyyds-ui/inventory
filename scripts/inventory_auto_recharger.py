#!/usr/bin/env python3
# coding=utf-8

import math
import threading
import time

import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Int8
from std_msgs.msg import UInt8
from std_srvs.srv import Trigger
from turtlesim.srv import Spawn
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


PI = 3.1415926535897


class InventoryAutoRecharger(Node):
    def __init__(self):
        super().__init__("inventory_auto_recharger")

        self.start_service_name = self.declare_parameter(
            "start_service_name", "/inventory/auto_recharge/start").value
        self.cancel_service_name = self.declare_parameter(
            "cancel_service_name", "/inventory/auto_recharge/cancel").value
        self.nav2_active_wait_timeout_sec = float(self.declare_parameter(
            "nav2_active_wait_timeout_sec", 60.0).value)
        self.nav_feedback_timeout_sec = float(self.declare_parameter(
            "nav_feedback_timeout_sec", 120.0).value)
        self.nav_goal_frame = self.declare_parameter("nav_goal_frame", "map").value
        self.charger_pose_x = float(self.declare_parameter("charger_pose_x", 0.21353509531521975).value)
        self.charger_pose_y = float(self.declare_parameter("charger_pose_y", 0.026333288560603657).value)
        self.charger_pose_z = float(self.declare_parameter("charger_pose_z", -0.121433301285056).value)
        self.charger_pose_w = float(self.declare_parameter("charger_pose_w", 0.9925995936625266).value)
        self.robot["BatteryCapacity"] = int(self.declare_parameter("robot_BatteryCapacity", 20000).value)
        self.robot["car_mode"] = self.declare_parameter("car_mode", "senior_akm").value
        self.diff_point = float(self.declare_parameter("diff_point", 1.2).value)
        self.diff_angle = float(self.declare_parameter("diff_angle", -15.0).value)
        if self.robot["car_mode"][0:4] != "mini":
            self.robot["Type"] = "Plus"
        else:
            self.robot["Type"] = "Mini"

        self.nav_controller = BasicNavigator()

        self.robot_security_off_pub = self.create_publisher(Int8, "/chassis_security", 10)
        self.charger_marker_pub = self.create_publisher(MarkerArray, "/goal_marker", 10)
        self.recharger_flag_pub = self.create_publisher(Int8, "robot_recharge_flag", 5)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 5)

        self.create_subscription(Float32, "PowerVoltage", self.voltage_callback, 10)
        self.create_subscription(Bool, "robot_charging_flag", self.charging_flag_callback, 10)
        self.create_subscription(Float32, "robot_charging_current", self.charging_current_callback, 10)
        self.create_subscription(UInt8, "robot_red_flag", self.red_flag_callback, 10)
        self.create_subscription(PoseStamped, "/charger_position_update", self.position_update_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        self.set_charge = self.create_client(Spawn, "/set_charge")
        self.start_srv = self.create_service(Trigger, self.start_service_name, self.start_callback)
        self.cancel_srv = self.create_service(Trigger, self.cancel_service_name, self.cancel_callback)

        self.state_lock = threading.Lock()
        self.start_requested = False
        self.recharge_running = False
        self.recharge_thread = None
        self.stop_requested = False
        self.cancel_requested = False
        self.nav_lock = threading.RLock()
        self.charge_control_lock = threading.RLock()

        self.last_marker_time = time.monotonic()
        self.timer = self.create_timer(0.2, self.timer_callback)

        sec = Int8()
        sec.data = 1
        self.robot_security_off_pub.publish(sec)
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info("盘库自动回充节点已启动，等待 /inventory/auto_recharge/start 请求。")

    robot = {
        "Type": "Plus",
        "BatteryCapacity": 5000,
        "Voltage": 25.0,
        "Charging": 0,
        "Charging_current": 0.0,
        "RED": 0,
        "Rotation_Z": 0.0,
        "car_mode": "mini_mec",
    }

    nav_end_z = 0.0
    start_turn = 0
    find_redsignal = 0
    red_count = 0
    chargeflag = 0
    lost_power_once = 1
    charge_complete = 0
    last_charge_complete = 0
    diff_point = 1.2
    diff_angle = -15.0

    def start_callback(self, request, response):
        del request
        with self.state_lock:
            if self.recharge_running or self.start_requested:
                response.success = True
                response.message = "自动回充流程已在运行中，请勿重复启动。"
                self.get_logger().info(response.message)
                return response
            self.start_requested = True
            response.success = True
            response.message = "已收到自动回充请求，开始执行自动回充流程。"
        self.get_logger().info(response.message)
        return response

    def cancel_callback(self, request, response):
        del request
        with self.state_lock:
            active = self.recharge_running or self.start_requested
            was_running = self.recharge_running
            if not active:
                response.success = True
                response.message = "当前没有正在运行的自动回充流程"
                self.get_logger().info(response.message)
                return response
            self.stop_requested = True
            self.cancel_requested = True
            if self.start_requested and not self.recharge_running:
                self.start_requested = False

        self.shutdown_chassis_recharge_mode("取消自动回充")

        if not was_running:
            with self.state_lock:
                self.stop_requested = False
                self.cancel_requested = False

        response.success = True
        response.message = "已取消自动回充流程"
        self.get_logger().info(response.message)
        return response

    def timer_callback(self):
        now = time.monotonic()
        if now - self.last_marker_time >= 1.0:
            self.publish_charger_marker()
            self.last_marker_time = now

        start_thread = False
        with self.state_lock:
            if self.start_requested and not self.recharge_running:
                self.start_requested = False
                self.recharge_running = True
                self.stop_requested = False
                self.cancel_requested = False
                start_thread = True

        if start_thread:
            self.get_logger().info("自动回充后台线程启动。")
            self.recharge_thread = threading.Thread(target=self.run_recharge_flow, daemon=True)
            self.recharge_thread.start()

    def voltage_callback(self, topic):
        self.robot["Voltage"] = topic.data

    def charging_flag_callback(self, topic):
        if self.robot["Charging"] == 0 and topic.data:
            self.get_logger().info("Charging started! 已检测到开始充电。")
        if self.robot["Charging"] == 1 and not topic.data:
            self.get_logger().warn("Charging disconnected! 充电连接已断开。")
        self.robot["Charging"] = 1 if topic.data else 0

    def charging_current_callback(self, topic):
        self.robot["Charging_current"] = topic.data

    def red_flag_callback(self, topic):
        self.red_count = int(topic.data)
        self.robot["RED"] = 1 if topic.data > 0 else 0
        if self.start_turn == 1 and self.robot["RED"] == 1:
            self.find_redsignal += 1

    def position_update_callback(self, topic):
        pose = topic.pose
        tmp_yaw = math.atan2(
            2 * (pose.orientation.w * pose.orientation.z),
            1 - 2 * (pose.orientation.z ** 2))
        diff_x = math.cos(tmp_yaw)
        diff_y = math.sin(tmp_yaw)
        self.charger_pose_x = pose.position.x + diff_x * self.diff_point
        self.charger_pose_y = pose.position.y + diff_y * self.diff_point
        tmp_angle = math.radians(self.diff_angle)
        new_yaw = (tmp_yaw + tmp_angle) / 2.0
        self.charger_pose_z = math.sin(new_yaw)
        self.charger_pose_w = math.cos(new_yaw)
        self.get_logger().info("已更新充电桩导航位姿。")

    def odom_callback(self, topic):
        self.robot["Rotation_Z"] = topic.pose.pose.position.z

    def publish_zero_velocity(self):
        self.cmd_vel_pub.publish(Twist())

    def wait_for_nav2_ready(self):
        self.get_logger().info("已收到自动回充请求，正在等待 Nav2 激活。")
        deadline = time.monotonic() + max(0.1, self.nav2_active_wait_timeout_sec)
        while rclpy.ok() and time.monotonic() < deadline:
            if self.should_stop_recharge():
                self.get_logger().warn("等待 Nav2 激活期间收到取消请求，停止自动回充流程。")
                return False
            if self.nav_controller.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info("Nav2 NavigateToPose action server 已可用，开始自动回充流程。")
                return True
            self.get_logger().info("正在等待 Nav2 NavigateToPose action server...")
        if self.should_stop_recharge():
            self.get_logger().warn("等待 Nav2 激活期间收到取消请求，停止自动回充流程。")
            return False
        self.get_logger().error("Nav2 未激活，无法启动自动回充，请确认导航系统已启动。")
        return False

    def set_charge_mode(self, value, max_callcount=10):
        self.get_logger().info("准备调用 /set_charge，value=%s。" % value)
        if not self.set_charge.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                "/set_charge 调用失败，底盘可能未进入自动贴桩模式。"
                "原因：服务不可用，请确认底盘节点已启动，value=%s。" % value)
            return False

        for call_index in range(max_callcount):
            req = Spawn.Request()
            req.x = float(value)
            future = self.set_charge.call_async(req)
            deadline = time.monotonic() + 2.0
            while rclpy.ok() and not future.done() and time.monotonic() < deadline:
                time.sleep(0.05)
            if future.done():
                try:
                    response = future.result()
                    if response is not None and response.name == "true":
                        self.get_logger().info("/set_charge 调用成功，底盘回充模式 value=%s。" % value)
                        return True
                    response_name = response.name if response is not None else "None"
                    self.get_logger().warn(
                        "/set_charge 第 %d 次调用未成功，value=%s，response=%s。" %
                        (call_index + 1, value, response_name))
                except Exception as exc:
                    self.get_logger().warn("调用 /set_charge 异常: %s" % exc)
            else:
                self.get_logger().warn(
                    "/set_charge 第 %d 次调用等待响应超时，value=%s。" %
                    (call_index + 1, value))
            time.sleep(0.5)
        self.get_logger().error(
            "/set_charge 调用失败，底盘可能未进入自动贴桩模式。"
            "已重试 %d 次，value=%s。" % (max_callcount, value))
        return False

    def publish_recharger_flag(self, set_velflag=0):
        with self.charge_control_lock:
            if set_velflag == 1:
                topic = Int8()
                topic.data = int(self.chargeflag)
                self.get_logger().info(
                    "发布 robot_recharge_flag，car_mode=%s，chargeflag=%d。" %
                    (self.robot["car_mode"], self.chargeflag))
                for _ in range(10):
                    self.recharger_flag_pub.publish(topic)
            self.get_logger().info(
                "准备设置底盘自动贴桩模式：car_mode=%s，/set_charge=%d。" %
                (self.robot["car_mode"], self.chargeflag))
            return self.set_charge_mode(self.chargeflag)

    def charger_pose(self):
        nav_goal = PoseStamped()
        nav_goal.header.frame_id = self.nav_goal_frame
        nav_goal.header.stamp = self.get_clock().now().to_msg()
        nav_goal.pose.position.x = self.charger_pose_x
        nav_goal.pose.position.y = self.charger_pose_y
        nav_goal.pose.orientation.z = self.charger_pose_z
        nav_goal.pose.orientation.w = self.charger_pose_w
        return nav_goal

    def publish_charger_position(self):
        nav_goal = self.charger_pose()
        self.publish_charger_marker()
        with self.nav_lock:
            accepted = self.nav_controller.goToPose(nav_goal)
        if accepted:
            self.get_logger().info(
                "Nav2 自动回充目标已发送: x=%.3f y=%.3f frame=%s" %
                (nav_goal.pose.position.x, nav_goal.pose.position.y, nav_goal.header.frame_id))
        return accepted

    def publish_charger_marker(self):
        o_z = self.charger_pose_z
        o_w = self.charger_pose_w
        p_x = self.charger_pose_x
        p_y = self.charger_pose_y
        tmp_yaw = math.atan2(2 * (o_w * o_z), 1 - 2 * (o_z ** 2))
        tmp_angle = math.radians(-self.diff_angle)
        new_yaw = (tmp_yaw + tmp_angle) / 2.0
        o_z = math.sin(new_yaw)
        o_w = math.cos(new_yaw)
        tmp_yaw = math.atan2(2 * (o_w * o_z), 1 - 2 * (o_z ** 2))
        p_x = p_x - math.cos(tmp_yaw) * self.diff_point
        p_y = p_y - math.sin(tmp_yaw) * self.diff_point

        marker_array = MarkerArray()
        marker_shape = Marker()
        marker_shape.id = 0
        marker_shape.header.frame_id = self.nav_goal_frame
        marker_shape.type = Marker.ARROW
        marker_shape.action = Marker.ADD
        marker_shape.scale.x = 0.5
        marker_shape.scale.y = 0.05
        marker_shape.scale.z = 0.05
        marker_shape.pose.position.x = p_x
        marker_shape.pose.position.y = p_y
        marker_shape.pose.position.z = 0.1
        marker_shape.pose.orientation.z = o_z
        marker_shape.pose.orientation.w = o_w
        marker_shape.color.r = 1.0
        marker_shape.color.a = 1.0
        marker_array.markers.append(marker_shape)

        marker_string = Marker()
        marker_string.id = 1
        marker_string.header.frame_id = self.nav_goal_frame
        marker_string.type = Marker.TEXT_VIEW_FACING
        marker_string.action = Marker.ADD
        marker_string.scale.z = 0.5
        marker_string.color.r = 1.0
        marker_string.color.a = 1.0
        marker_string.pose.position.x = p_x
        marker_string.pose.position.y = p_y
        marker_string.pose.position.z = 0.1
        marker_string.pose.orientation.z = o_z
        marker_string.pose.orientation.w = o_w
        marker_string.text = "Charger"
        marker_array.markers.append(marker_string)
        self.charger_marker_pub.publish(marker_array)

    def cancel_nav_goal(self):
        try:
            with self.nav_lock:
                self.nav_controller.cancelTask()
        except Exception as exc:
            self.get_logger().warn("取消自动回充 Nav2 goal 异常: %s" % exc)

    def should_stop_recharge(self):
        with self.state_lock:
            return self.stop_requested or self.cancel_requested

    def request_recharge_stop(self):
        with self.state_lock:
            self.stop_requested = True
            self.cancel_requested = True
        self.cancel_nav_goal()
        self.publish_zero_velocity()

    def stop_charge(self):
        self.shutdown_chassis_recharge_mode("停止自动回充")

    def shutdown_chassis_recharge_mode(self, reason):
        self.cancel_nav_goal()
        self.publish_zero_velocity()
        with self.charge_control_lock:
            self.lost_power_once = 1
            self.chargeflag = 0
            self.get_logger().info(
                "%s：已发布 /cmd_vel=0，chargeflag=0，准备调用 /set_charge=0。" % reason)
            set_charge_ok = self.set_charge_mode(0, max_callcount=3)
            topic = Int8()
            topic.data = 0
            for _ in range(10):
                self.recharger_flag_pub.publish(topic)
            self.get_logger().info("%s：已发布 robot_recharge_flag=0。" % reason)
            if not set_charge_ok:
                self.get_logger().error(
                    "/set_charge=0 调用失败，底盘可能仍处于自动贴桩模式，请现场确认。")
            if reason == "取消自动回充":
                self.get_logger().info("已取消自动回充流程，已关闭底盘回充模式。")
            else:
                self.get_logger().info("%s：已关闭底盘回充模式。" % reason)
            return set_charge_ok

    def start_docking_by_red_signal(self, reason="发现红外信号"):
        with self.charge_control_lock:
            if self.should_stop_recharge():
                self.get_logger().warn("%s，但已收到取消请求，不再开启底盘自动贴桩模式。" % reason)
                return
            if "akm" in self.robot["car_mode"]:
                self.chargeflag = 2
            else:
                self.chargeflag = 1
            self.get_logger().info(
                "%s，开始底盘自动贴桩/回充：car_mode=%s，RED=%d，chargeflag=%d，/set_charge=%d。" %
                (reason, self.robot["car_mode"], self.robot["RED"], self.chargeflag, self.chargeflag))
            self.publish_recharger_flag(1)

    def battery_percent(self):
        if self.robot["Type"] == "Plus":
            return (self.robot["Voltage"] - 20.0) / 5.0
        if self.robot["Type"] == "Mini":
            return (self.robot["Voltage"] - 10.0) / 2.5
        return 0.0

    def recharge_estimate_text(self):
        percent = self.battery_percent()
        need_percent = max(0.0, 1.0 - percent)
        need_percent_form = format(need_percent, ".0%")
        battery_capacity = float(self.robot["BatteryCapacity"])
        left_battery = max(0.0, battery_capacity * percent)
        need_charge_battery = max(0.0, battery_capacity - left_battery)
        charging_current = float(self.robot["Charging_current"])
        if charging_current > 1e-6:
            mAh_time = 1.0 / charging_current / 1000.0
            need_charge_hours = need_charge_battery * mAh_time
            need_time_text = "%.2f 小时" % need_charge_hours
        else:
            mAh_time = 0.0
            need_time_text = "无法估算，充电电流为 0"
        return (
            "BatteryCapacity=%d mAh，当前电量估算=%s，预计还需补充 %.2f mAh，"
            "need_percent_form=%s，mAh_time=%.6f，预计充电耗时=%s" %
            (
                self.robot["BatteryCapacity"],
                format(max(0.0, percent), ".0%"),
                need_charge_battery,
                need_percent_form,
                mAh_time,
                need_time_text,
            ))

    def log_recharge_start_context(self):
        self.get_logger().info(
            "自动回充流程启动信息：car_mode=%s，BatteryCapacity=%d mAh，Voltage=%.2f V，"
            "Charging=%d，Charging_Current=%.2f A，chargeflag=%d，"
            "charger_goal[x=%.3f, y=%.3f, z=%.6f, w=%.6f, frame=%s]。" %
            (
                self.robot["car_mode"],
                self.robot["BatteryCapacity"],
                self.robot["Voltage"],
                self.robot["Charging"],
                self.robot["Charging_current"],
                self.chargeflag,
                self.charger_pose_x,
                self.charger_pose_y,
                self.charger_pose_z,
                self.charger_pose_w,
                self.nav_goal_frame,
            ))

    def log_charging_status(self):
        self.get_logger().info(
            "充电信息：Voltage=%.2f V，Charging_Current=%.2f A，Charging=%d，"
            "chargeflag=%d，charge_complete=%d，last_charge_complete=%d，%s。" %
            (
                self.robot["Voltage"],
                self.robot["Charging_current"],
                self.robot["Charging"],
                self.chargeflag,
                self.charge_complete,
                self.last_charge_complete,
                self.recharge_estimate_text(),
            ))

    def log_waiting_for_charging_status(self):
        self.get_logger().warn(
            "已进入自动贴桩/回充模式，但尚未检测到充电状态，请检查红外、充电触点、"
            "/set_charge 和底盘回充模式。当前 RED=%d，car_mode=%s，chargeflag=%d，"
            "Voltage=%.2f V，Charging_Current=%.2f A。" %
            (
                self.robot["RED"],
                self.robot["car_mode"],
                self.chargeflag,
                self.robot["Voltage"],
                self.robot["Charging_current"],
            ))

    def monitor_charging_complete(self):
        if self.robot["Charging"] != 1:
            self.last_charge_complete = self.charge_complete
            self.charge_complete = 0
            return False
        if ((self.robot["Type"] == "Plus" and self.robot["Voltage"] > 25) or
                (self.robot["Type"] == "Mini" and self.robot["Voltage"] > 12.5)):
            self.charge_complete += 1
        else:
            self.charge_complete = 0
        if self.charge_complete > 10:
            self.charge_complete = 0
            self.last_charge_complete = 0
            self.get_logger().info("已检测到充电完成，自动回充流程结束。")
            self.stop_charge()
            return True
        self.last_charge_complete = self.charge_complete
        return False

    def run_recharge_flow(self):
        try:
            self.log_recharge_start_context()
            if not self.wait_for_nav2_ready():
                return

            self.publish_zero_velocity()
            if self.red_count >= 3:
                self.cancel_nav_goal()
                self.get_logger().info("已捕获到高强度红外信号，使用红外信号对接。")
                self.start_docking_by_red_signal("高强度红外直接对接")
            else:
                self.get_logger().info("开始导航到充电桩位置。")
                if not self.publish_charger_position():
                    self.get_logger().error("Nav2 拒绝自动回充目标，自动回充流程终止。")
                    return

                while rclpy.ok() and not self.should_stop_recharge():
                    with self.nav_lock:
                        task_complete = self.nav_controller.isTaskComplete()
                    if task_complete:
                        with self.nav_lock:
                            result = self.nav_controller.getResult()
                        if result == TaskResult.SUCCEEDED:
                            self.get_logger().info("自动回充导航任务完成：已到达充电桩导航目标。")
                            if self.robot["RED"] == 1:
                                self.get_logger().info(
                                    "已到达充电桩预停点，当前 RED=%d，car_mode=%s，准备开启底盘贴桩模式。" %
                                    (self.robot["RED"], self.robot["car_mode"]))
                                self.start_docking_by_red_signal("已到达充电桩预停点并发现红外信号")
                            else:
                                self.get_logger().warn("未发现红外信号，开始自转寻找。")
                                self.nav_end_z = self.robot["Rotation_Z"]
                                self.start_turn = 1
                                cmd = Twist()
                                cmd.angular.z = 0.2
                                self.cmd_vel_pub.publish(cmd)
                        elif result == TaskResult.CANCELED:
                            self.get_logger().warn("自动回充导航任务取消：Nav2 目标被取消。")
                            return
                        else:
                            self.get_logger().error("自动回充导航任务失败：Nav2 目标未成功完成。")
                            return
                        break

                    with self.nav_lock:
                        feedback = self.nav_controller.getFeedback()
                    if feedback is not None:
                        if (feedback.distance_remaining < 0.2 and
                                Duration.from_msg(feedback.navigation_time) >
                                Duration(seconds=self.nav_feedback_timeout_sec)):
                            self.get_logger().warn("长时间无法到达充电桩目标，取消导航并尝试红外对接。")
                            self.cancel_nav_goal()
                            if self.robot["RED"] == 1:
                                self.start_docking_by_red_signal("Nav2 接近超时但已发现红外信号")
                            else:
                                return
                    time.sleep(0.2)

            last_charge_log = 0.0
            last_waiting_charge_log = 0.0
            while rclpy.ok() and not self.should_stop_recharge():
                now = time.monotonic()
                if self.find_redsignal >= 3:
                    self.find_redsignal = 0
                    self.start_turn = 0
                    self.publish_zero_velocity()
                    self.get_logger().info("已通过自转发现红外信号，开始对接充电。")
                    self.start_docking_by_red_signal("自转找到红外信号")

                if self.start_turn == 1 and abs(self.robot["Rotation_Z"] - self.nav_end_z) > 2 * PI:
                    self.start_turn = 0
                    self.stop_charge()
                    self.get_logger().error("自转已完成，无法找到充电桩位置，已停止自动回充。")
                    return

                if self.robot["Charging"] == 1:
                    if self.monitor_charging_complete():
                        return
                    if now - last_charge_log > 10.0:
                        last_charge_log = now
                        self.log_charging_status()
                elif self.chargeflag != 0 and now - last_waiting_charge_log > 10.0:
                    last_waiting_charge_log = now
                    self.log_waiting_for_charging_status()

                time.sleep(0.5)
        except ValueError as exc:
            if "generator already executing" in str(exc):
                self.get_logger().error(
                    "自动回充流程异常：检测到 rclpy executor 并发执行冲突(generator already executing)。"
                    "已尝试取消自动回充 Nav2 goal，请确认主节点未使用默认 global executor spin。")
            else:
                self.get_logger().error("自动回充流程 ValueError 异常: %s" % exc)
            self.cancel_nav_goal()
            self.publish_zero_velocity()
        except Exception as exc:
            self.get_logger().error("自动回充流程异常: %s，已尝试取消自动回充 Nav2 goal。" % exc)
            self.cancel_nav_goal()
            self.publish_zero_velocity()
        finally:
            with self.state_lock:
                self.recharge_running = False
                self.start_requested = False
                self.stop_requested = False
                self.cancel_requested = False
            self.get_logger().info("自动回充后台流程已结束。")


def main():
    rclpy.init()
    node = InventoryAutoRecharger()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.request_recharge_stop()
        if node.recharge_thread is not None and node.recharge_thread.is_alive():
            node.recharge_thread.join(timeout=2.0)
        executor.remove_node(node)
        executor.shutdown()
        node.publish_zero_velocity()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
