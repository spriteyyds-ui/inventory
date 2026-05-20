#!/usr/bin/env python3
import os
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int8, Int32, String, UInt8
from std_srvs.srv import Trigger
from wheeltec_inventory_system.msg import RecognizedNumber
from wheeltec_inventory_system.srv import StartMission


class InventoryOperationGuiNode(Node):
    def __init__(self):
        super().__init__("inventory_operation_gui")

        self.mission_state_topic = self.declare_parameter(
            "mission_state_topic", "/inventory/mission_state").value
        self.mission_log_topic = self.declare_parameter(
            "mission_log_topic", "/inventory/mission_log").value
        self.recognized_topic = self.declare_parameter(
            "recognized_topic", "/inventory/recognized_number").value
        self.current_target_cabinet_topic = self.declare_parameter(
            "current_target_cabinet_topic", "/inventory/current_target_cabinet").value
        self.power_voltage_topic = self.declare_parameter(
            "power_voltage_topic", "PowerVoltage").value
        self.charging_flag_topic = self.declare_parameter(
            "charging_flag_topic", "robot_charging_flag").value
        self.charging_current_topic = self.declare_parameter(
            "charging_current_topic", "robot_charging_current").value
        self.red_flag_topic = self.declare_parameter(
            "red_flag_topic", "robot_red_flag").value
        self.recharge_flag_topic = self.declare_parameter(
            "recharge_flag_topic", "robot_recharge_flag").value

        self.start_mission_service_name = self.declare_parameter(
            "start_mission_service_name", "/inventory/start_mission").value
        self.cancel_mission_service_name = self.declare_parameter(
            "cancel_mission_service_name", "/inventory/cancel_mission").value
        self.return_to_charge_service_name = self.declare_parameter(
            "return_to_charge_service_name", "/inventory/return_to_charge").value
        self.cancel_auto_recharge_service_name = self.declare_parameter(
            "cancel_auto_recharge_service_name", "/inventory/cancel_auto_recharge").value
        self.auto_recharge_start_service_name = self.declare_parameter(
            "auto_recharge_start_service_name", "/inventory/auto_recharge/start").value
        self.log_history_size = int(self.declare_parameter("log_history_size", 10).value)

        self.values = {
            "mission_state": "未知",
            "current_target_cabinet": "未知",
            "recognition": "未知",
            "voltage": "未知",
            "charging": "未知",
            "charging_current": "未知",
            "red_flag": "未知",
            "recharge_flag": "未知",
        }
        self.mission_logs = deque(maxlen=max(1, self.log_history_size))
        self.subscriptions_ = [
            self.create_subscription(String, self.mission_state_topic, self.mission_state_callback, 10),
            self.create_subscription(String, self.mission_log_topic, self.mission_log_callback, 10),
            self.create_subscription(
                RecognizedNumber, self.recognized_topic, self.recognition_callback, 10),
            self.create_subscription(
                Int32, self.current_target_cabinet_topic, self.current_target_cabinet_callback, 10),
            self.create_subscription(Float32, self.power_voltage_topic, self.voltage_callback, 10),
            self.create_subscription(Bool, self.charging_flag_topic, self.charging_flag_callback, 10),
            self.create_subscription(
                Float32, self.charging_current_topic, self.charging_current_callback, 10),
            self.create_subscription(UInt8, self.red_flag_topic, self.red_flag_callback, 10),
            self.create_subscription(Int8, self.recharge_flag_topic, self.recharge_flag_callback, 10),
        ]

        self.start_mission_client = self.create_client(
            StartMission, self.start_mission_service_name)
        self.cancel_mission_client = self.create_client(
            Trigger, self.cancel_mission_service_name)
        self.return_to_charge_client = self.create_client(
            Trigger, self.return_to_charge_service_name)
        self.cancel_auto_recharge_client = self.create_client(
            Trigger, self.cancel_auto_recharge_service_name)
        self.auto_recharge_start_client = self.create_client(
            Trigger, self.auto_recharge_start_service_name)

    def mission_state_callback(self, msg):
        self.values["mission_state"] = msg.data

    def mission_log_callback(self, msg):
        self.mission_logs.append(msg.data)

    def recognition_callback(self, msg):
        valid_text = "有效" if msg.valid else "无效"
        number = msg.number if msg.number else "无编号"
        self.values["recognition"] = (
            "%s 编号=%s 置信度=%.2f 距离=%.2f m 偏移=%.2f 尝试=%d" %
            (valid_text, number, msg.confidence, msg.estimated_distance,
             msg.horizontal_offset, msg.attempts))

    def current_target_cabinet_callback(self, msg):
        self.values["current_target_cabinet"] = "无目标" if msg.data < 0 else str(msg.data)

    def voltage_callback(self, msg):
        self.values["voltage"] = "%.2f V" % msg.data

    def charging_flag_callback(self, msg):
        self.values["charging"] = "是" if msg.data else "否"

    def charging_current_callback(self, msg):
        self.values["charging_current"] = "%.3f A" % msg.data

    def red_flag_callback(self, msg):
        self.values["red_flag"] = str(msg.data)

    def recharge_flag_callback(self, msg):
        self.values["recharge_flag"] = str(msg.data)


class InventoryOperationGuiApp:
    def __init__(self, root, node, tk, ttk):
        self.root = root
        self.node = node
        self.tk = tk
        self.ttk = ttk
        self.closing = False
        self.pending_futures = []
        self.last_log_snapshot = ()

        self.root.title("无人车盘库操作总控")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.fields = {}
        self.target_cabinets_var = self.tk.StringVar(value="")
        self.target_gap_var = self.tk.StringVar(value="")
        self.command_status = self.tk.StringVar(value="等待操作")
        self.build_widgets()
        self.refresh()

    def build_widgets(self):
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        main = self.ttk.Frame(self.root, padding=12)
        main.grid(row=0, column=0, sticky="nsew")
        main.columnconfigure(0, weight=3)
        main.columnconfigure(1, weight=2)
        main.rowconfigure(1, weight=1)

        operation_frame = self.ttk.LabelFrame(main, text="盘库操作", padding=10)
        operation_frame.grid(row=0, column=0, sticky="ew", padx=(0, 10))
        operation_frame.columnconfigure(1, weight=1)
        self.ttk.Label(operation_frame, text="目标柜号").grid(
            row=0, column=0, sticky="w", padx=(0, 10), pady=3)
        self.ttk.Entry(operation_frame, textvariable=self.target_cabinets_var).grid(
            row=0, column=1, columnspan=2, sticky="ew", pady=3)
        self.ttk.Label(operation_frame, text="目标间隙 ID").grid(
            row=1, column=0, sticky="w", padx=(0, 10), pady=3)
        self.ttk.Entry(operation_frame, textvariable=self.target_gap_var).grid(
            row=1, column=1, columnspan=2, sticky="ew", pady=3)
        self.add_value_row(operation_frame, 2, "启动服务", "start_mission_service")
        self.add_value_row(operation_frame, 3, "取消服务", "cancel_mission_service")

        inventory_button_frame = self.ttk.Frame(operation_frame)
        inventory_button_frame.grid(row=4, column=0, columnspan=3, sticky="ew", pady=(8, 0))
        self.start_inventory_button = self.ttk.Button(
            inventory_button_frame, text="开始盘库", command=self.request_start_inventory)
        self.start_inventory_button.pack(side=self.tk.LEFT, padx=(0, 8))
        self.full_inventory_button = self.ttk.Button(
            inventory_button_frame, text="全量盘库", command=self.request_full_inventory)
        self.full_inventory_button.pack(side=self.tk.LEFT, padx=(0, 8))
        self.cancel_mission_button = self.ttk.Button(
            inventory_button_frame, text="停止/取消任务", command=self.request_cancel_mission)
        self.cancel_mission_button.pack(side=self.tk.LEFT)

        task_frame = self.ttk.LabelFrame(main, text="任务状态", padding=10)
        task_frame.grid(row=1, column=0, sticky="nsew", padx=(0, 10), pady=(10, 0))
        task_frame.columnconfigure(1, weight=1)
        task_frame.rowconfigure(4, weight=1)
        self.add_value_row(task_frame, 0, "任务状态", "mission_state")
        self.add_value_row(task_frame, 1, "当前目标柜号", "current_target_cabinet")
        self.add_value_row(task_frame, 2, "识别结果", "recognition")
        self.ttk.Label(task_frame, text="最近日志").grid(
            row=3, column=0, columnspan=2, sticky="w", pady=(8, 3))
        self.log_text = self.tk.Text(task_frame, width=78, height=10, wrap=self.tk.WORD)
        self.log_text.grid(row=4, column=0, columnspan=2, sticky="nsew")
        self.log_text.configure(state=self.tk.DISABLED)

        charge_frame = self.ttk.LabelFrame(main, text="充电状态", padding=10)
        charge_frame.grid(row=0, column=1, rowspan=2, sticky="nsew")
        charge_frame.columnconfigure(1, weight=1)
        charge_rows = [
            ("电压", "voltage"),
            ("正在充电", "charging"),
            ("充电电流", "charging_current"),
            ("红外状态", "red_flag"),
            ("底盘回充标志", "recharge_flag"),
            ("自动回充节点服务", "auto_recharge_start_service"),
            ("启动回充指令服务", "return_to_charge_service"),
            ("取消回充指令服务", "cancel_auto_recharge_service"),
        ]
        for row_index, (label, key) in enumerate(charge_rows):
            self.add_value_row(charge_frame, row_index, label, key)

        charge_button_frame = self.ttk.Frame(charge_frame)
        charge_button_frame.grid(
            row=len(charge_rows), column=0, columnspan=2, sticky="ew", pady=(10, 0))
        self.return_to_charge_button = self.ttk.Button(
            charge_button_frame, text="启动自动回充", command=self.request_return_to_charge)
        self.return_to_charge_button.pack(side=self.tk.LEFT, padx=(0, 8))
        self.cancel_auto_recharge_button = self.ttk.Button(
            charge_button_frame, text="取消自动回充", command=self.request_cancel_auto_recharge)
        self.cancel_auto_recharge_button.pack(side=self.tk.LEFT)

        status_frame = self.ttk.Frame(main)
        status_frame.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(10, 0))
        status_frame.columnconfigure(1, weight=1)
        self.ttk.Label(status_frame, text="操作结果").grid(
            row=0, column=0, sticky="w", padx=(0, 10))
        self.ttk.Label(status_frame, textvariable=self.command_status).grid(
            row=0, column=1, sticky="ew")

    def add_value_row(self, parent, row_index, label, key):
        self.ttk.Label(parent, text=label).grid(
            row=row_index, column=0, sticky="w", padx=(0, 10), pady=3)
        var = self.tk.StringVar(value="未知")
        self.fields[key] = var
        self.ttk.Label(parent, textvariable=var, wraplength=520).grid(
            row=row_index, column=1, sticky="ew", pady=3)

    def request_start_inventory(self):
        cabinets = self.parse_target_cabinets()
        if cabinets is None:
            return
        if not cabinets:
            message = "请输入目标柜号"
            self.command_status.set(message)
            self.node.get_logger().warn(message)
            return

        request = StartMission.Request()
        request.targets = []
        request.return_home = False
        request.target_gap = self.target_gap_var.get().strip()
        request.scan_cabinets = cabinets
        request.run_full_inventory = len(cabinets) > 1 and not request.target_gap
        self.send_start_mission_request("开始盘库", request)

    def request_full_inventory(self):
        request = StartMission.Request()
        request.targets = []
        request.return_home = True
        request.run_full_inventory = True
        request.target_gap = ""
        request.scan_cabinets = []
        self.send_start_mission_request("全量盘库", request)

    def request_cancel_mission(self):
        self.send_trigger_request(
            "停止/取消任务",
            self.node.cancel_mission_client,
            self.node.cancel_mission_service_name)

    def request_return_to_charge(self):
        self.send_trigger_request(
            "启动自动回充",
            self.node.return_to_charge_client,
            self.node.return_to_charge_service_name)

    def request_cancel_auto_recharge(self):
        self.send_trigger_request(
            "取消自动回充",
            self.node.cancel_auto_recharge_client,
            self.node.cancel_auto_recharge_service_name)

    def parse_target_cabinets(self):
        raw = self.target_cabinets_var.get().replace("，", ",").strip()
        if not raw:
            return []

        cabinets = []
        for item in raw.split(","):
            text = item.strip()
            if not text:
                self.report_input_error("目标柜号格式非法: %s" % raw)
                return None
            try:
                cabinet_id = int(text)
            except ValueError:
                self.report_input_error("目标柜号必须是正整数: %s" % text)
                return None
            if cabinet_id <= 0:
                self.report_input_error("目标柜号必须大于 0: %s" % text)
                return None
            cabinets.append(cabinet_id)
        return cabinets

    def report_input_error(self, message):
        self.command_status.set(message)
        self.node.get_logger().warn(message)

    def send_start_mission_request(self, label, request):
        client = self.node.start_mission_client
        service_name = self.node.start_mission_service_name
        if not client.service_is_ready():
            message = "%s 服务不可用: %s" % (label, service_name)
            self.command_status.set(message)
            self.node.get_logger().warn(message)
            return

        future = client.call_async(request)
        self.pending_futures.append({"label": label, "future": future})
        self.command_status.set("已发送%s请求，等待响应。" % label)
        self.update_button_states()

    def send_trigger_request(self, label, client, service_name):
        if not client.service_is_ready():
            message = "%s 服务不可用: %s" % (label, service_name)
            self.command_status.set(message)
            self.node.get_logger().warn(message)
            return

        future = client.call_async(Trigger.Request())
        self.pending_futures.append({"label": label, "future": future})
        self.command_status.set("已发送%s请求，等待响应。" % label)
        self.update_button_states()

    def refresh(self):
        if self.closing:
            return

        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except Exception as exc:
            self.command_status.set("ROS 回调处理异常: %s" % exc)
            self.node.get_logger().error("操作总控 GUI 处理 ROS 回调异常: %s" % exc)

        self.collect_completed_futures()
        self.update_fields()
        self.update_log_text()
        self.update_button_states()
        self.root.after(200, self.refresh)

    def collect_completed_futures(self):
        still_pending = []
        for item in self.pending_futures:
            future = item["future"]
            label = item["label"]
            if not future.done():
                still_pending.append(item)
                continue
            try:
                response = future.result()
                message = self.response_message(label, response)
                self.command_status.set(message)
                self.node.get_logger().info(message)
            except Exception as exc:
                message = "%s请求异常: %s" % (label, exc)
                self.command_status.set(message)
                self.node.get_logger().error(message)
        self.pending_futures = still_pending

    @staticmethod
    def response_message(label, response):
        if response is None:
            return "%s未返回有效响应。" % label

        detail = getattr(response, "message", "")
        if hasattr(response, "accepted"):
            result = "已接收" if response.accepted else "被拒绝"
            return "%s%s: %s" % (label, result, detail)
        if hasattr(response, "success"):
            result = "成功" if response.success else "失败"
            return "%s%s: %s" % (label, result, detail)
        return "%s已返回: %s" % (label, detail)

    def update_fields(self):
        for key, value in self.node.values.items():
            self.fields[key].set(value)
        self.fields["start_mission_service"].set(
            self.ready_text(self.node.start_mission_client))
        self.fields["cancel_mission_service"].set(
            self.ready_text(self.node.cancel_mission_client))
        self.fields["auto_recharge_start_service"].set(
            self.ready_text(self.node.auto_recharge_start_client))
        self.fields["return_to_charge_service"].set(
            self.ready_text(self.node.return_to_charge_client))
        self.fields["cancel_auto_recharge_service"].set(
            self.ready_text(self.node.cancel_auto_recharge_client))

    def update_log_text(self):
        snapshot = tuple(self.node.mission_logs)
        if snapshot == self.last_log_snapshot:
            return
        self.last_log_snapshot = snapshot
        self.log_text.configure(state=self.tk.NORMAL)
        self.log_text.delete("1.0", self.tk.END)
        self.log_text.insert(self.tk.END, "\n".join(snapshot))
        self.log_text.configure(state=self.tk.DISABLED)

    def update_button_states(self):
        start_pending = self.is_pending("开始盘库")
        full_pending = self.is_pending("全量盘库")
        mission_start_pending = start_pending or full_pending
        cancel_mission_pending = self.is_pending("停止/取消任务")
        return_to_charge_pending = self.is_pending("启动自动回充")
        cancel_auto_recharge_pending = self.is_pending("取消自动回充")

        start_ready = self.node.start_mission_client.service_is_ready()
        cancel_ready = self.node.cancel_mission_client.service_is_ready()
        self.start_inventory_button.configure(
            state=self.tk.NORMAL if start_ready and not mission_start_pending else self.tk.DISABLED)
        self.full_inventory_button.configure(
            state=self.tk.NORMAL if start_ready and not mission_start_pending else self.tk.DISABLED)
        self.cancel_mission_button.configure(
            state=self.tk.NORMAL
            if cancel_ready and not cancel_mission_pending
            else self.tk.DISABLED)
        self.return_to_charge_button.configure(
            state=self.tk.NORMAL
            if self.node.return_to_charge_client.service_is_ready() and not return_to_charge_pending
            else self.tk.DISABLED)
        self.cancel_auto_recharge_button.configure(
            state=self.tk.NORMAL
            if self.node.cancel_auto_recharge_client.service_is_ready() and not cancel_auto_recharge_pending
            else self.tk.DISABLED)

    def is_pending(self, label):
        return any(item["label"] == label for item in self.pending_futures)

    @staticmethod
    def ready_text(client):
        return "可用" if client.service_is_ready() else "不可用"

    def close(self):
        self.closing = True
        self.root.quit()


def main(args=None):
    rclpy.init(args=args)
    node = InventoryOperationGuiNode()
    root = None
    try:
        if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
            node.get_logger().warn("未检测到 DISPLAY，操作总控 GUI 不启动，其他节点不受影响。")
            return

        try:
            import tkinter as tk
            from tkinter import ttk
        except Exception as exc:
            node.get_logger().error(
                "无法导入 tkinter，操作总控 GUI 不启动，其他节点不受影响: %s" % exc)
            return

        try:
            root = tk.Tk()
        except Exception as exc:
            node.get_logger().error(
                "无法创建 tkinter 窗口，操作总控 GUI 不启动，其他节点不受影响: %s" % exc)
            return

        InventoryOperationGuiApp(root, node, tk, ttk)
        node.get_logger().info("操作总控 GUI 已启动。")
        root.mainloop()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        if root is not None:
            try:
                root.destroy()
            except Exception:
                pass


if __name__ == "__main__":
    main()
