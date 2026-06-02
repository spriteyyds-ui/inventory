#!/usr/bin/env python3
import os
import json
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from collections import deque
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int8, Int32, String, UInt8
from std_srvs.srv import Trigger
from agv_inventory_system.msg import LiftState, RecognizedNumber
from agv_inventory_system.srv import LiftMoveTimed, StartMission


REFRESH_PERIOD_MS = 200
WINDOW_WIDTH = 1180
WINDOW_HEIGHT = 780
STATUS_BAR_HEIGHT = 92
LOG_FRAME_HEIGHT = 205
STATUS_TILE_WIDTH = 214
STATUS_TILE_HEIGHT = 82
STATUS_MAIN_MAX_CHARS = 18
STATUS_DETAIL_MAX_CHARS = 32
VALUE_MAX_CHARS = 48
WIDE_VALUE_MAX_CHARS = 72
COMMAND_MAX_CHARS = 96
ERROR_MAX_CHARS = 96
SELECTED_CABINETS_MAX_CHARS = 78
MAIN_ACTION_BUTTON_HEIGHT_PX = 48
MAIN_ACTION_BUTTON_FONT = ("TkDefaultFont", 13, "bold")
LIFT_BASE_HEIGHT_M = 1.25
ROBOT_API_STATUS_MAX_CHARS = 40

ACTION_START_INVENTORY = "开始盘库"
ACTION_FULL_INVENTORY = "全量盘库"
ACTION_CANCEL_MISSION = "停止/中断盘库"
ACTION_SAFE_EXIT_GAP = "停止并退出缝隙"
ACTION_RETURN_HOME = "回地图零点"
ACTION_RETURN_TO_CHARGE = "启动自动回充"
ACTION_CANCEL_AUTO_RECHARGE = "取消自动回充"
ACTION_STOP_AUTO_CHARGE_AND_DEPART = "停止自动充电并离桩"

AUTO_RECHARGE_STATUS_TEXT = {
    "IDLE": "空闲",
    "STARTING": "启动中",
    "NAVIGATING": "导航中",
    "DOCKING": "对接中",
    "CHARGING": "充电中",
    "COMPLETE": "已完成",
    "FAILED": "失败",
    "CANCELED": "已取消",
}

MISSION_STATE_TEXT = {
    "IDLE": "空闲",
    "CORRIDOR_NAV": "通道导航中",
    "TARGET_TRACKING": "目标识别中",
    "SEARCH_GAP": "搜索缝隙",
    "WAITING_GAP": "等待缝隙",
    "ENTERING_GAP": "正在入缝",
    "INVENTORYING": "盘库中",
    "EXIT_GAP": "正在出缝",
    "RETURNING": "返回中",
    "DONE": "已完成",
    "ERROR": "异常",
    "AUTO_RECHARGING": "自动回充中",
    "SAFE_EXIT_GAP": "安全出缝中",
    "STOP_AUTO_CHARGE_AND_DEPART": "停止充电并离桩",
    "FULL_INVENTORY_NAV_TO_OBSERVE": "全量盘库｜前往观察位",
    "FULL_INVENTORY_POST_ROUTE_RECOGNITION_WAIT": "全量盘库｜到点识别",
    "FULL_INVENTORY_CORRIDOR_NAV": "全量盘库｜通道导航中",
    "FULL_INVENTORY_TARGET_TRACKING": "全量盘库｜目标识别中",
    "FULL_INVENTORY_TARGET_DISTANCE_ALIGN": "全量盘库｜目标对齐",
    "FULL_INVENTORY_SEARCH_GAP": "全量盘库｜搜索缝隙",
    "FULL_INVENTORY_WAITING_GAP": "全量盘库｜等待缝隙",
    "FULL_INVENTORY_ENTERING_GAP": "全量盘库｜正在入缝",
    "FULL_INVENTORY_IN_GAP_SCAN": "全量盘库｜缝内扫描",
    "FULL_INVENTORY_INVENTORYING": "全量盘库｜缝内扫描",
    "FULL_INVENTORY_EXIT_GAP": "全量盘库｜正在出缝",
    "SINGLE_CABINET_ENTERING_GAP": "单柜盘库｜正在入缝",
    "SINGLE_CABINET_INVENTORYING": "单柜盘库｜盘库中",
    "SINGLE_CABINET_EXIT_GAP": "单柜盘库｜正在出缝",
}

GAP_PROTECTED_TOKENS = (
    "ENTERING_GAP",
    "IN_GAP_SCAN",
    "IN_GAP",
    "INVENTORYING",
    "EXIT_GAP",
    "SAFE_EXIT_GAP",
)


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
        self.auto_recharge_status_topic = self.declare_parameter(
            "auto_recharge_status_topic", "/inventory/auto_recharge/status").value
        self.lift_state_topic = self.declare_parameter(
            "lift_state_topic", "/lift/state").value

        self.start_mission_service_name = self.declare_parameter(
            "start_mission_service_name", "/inventory/start_mission").value
        self.cancel_mission_service_name = self.declare_parameter(
            "cancel_mission_service_name", "/inventory/cancel_mission").value
        self.safe_exit_gap_service_name = self.declare_parameter(
            "safe_exit_gap_service_name", "/inventory/safe_exit_gap").value
        self.return_home_service_name = self.declare_parameter(
            "return_home_service_name", "/inventory/return_home").value
        self.return_to_charge_service_name = self.declare_parameter(
            "return_to_charge_service_name", "/inventory/return_to_charge").value
        self.cancel_auto_recharge_service_name = self.declare_parameter(
            "cancel_auto_recharge_service_name", "/inventory/cancel_auto_recharge").value
        self.stop_auto_charge_and_depart_service_name = self.declare_parameter(
            "stop_auto_charge_and_depart_service_name",
            "/inventory/stop_auto_charge_and_depart").value
        self.auto_recharge_start_service_name = self.declare_parameter(
            "auto_recharge_start_service_name", "/inventory/auto_recharge/start").value
        self.lift_up_service_name = self.declare_parameter(
            "lift_up_service_name", "/lift/up").value
        self.lift_down_service_name = self.declare_parameter(
            "lift_down_service_name", "/lift/down").value
        self.lift_stop_service_name = self.declare_parameter(
            "lift_stop_service_name", "/lift/stop").value
        self.lift_all_off_service_name = self.declare_parameter(
            "lift_all_off_service_name", "/lift/all_off").value
        self.lift_reset_estimated_height_service_name = self.declare_parameter(
            "lift_reset_estimated_height_service_name", "/lift/reset_estimated_height").value
        self.lift_manual_duration_sec = float(
            self.declare_parameter("lift_manual_duration_sec", 2.0).value)
        self.log_history_size = int(self.declare_parameter("log_history_size", 10).value)
        self.robot_api_status_url = self.declare_parameter(
            "robot_api_status_url", "http://127.0.0.1:8000/status").value
        self.robot_api_health_url = self.declare_parameter(
            "robot_api_health_url", "http://127.0.0.1:8000/health").value
        self.robot_api_poll_interval_ms = int(self.declare_parameter(
            "robot_api_poll_interval_ms", 2000).value)
        self.robot_api_request_timeout_sec = float(self.declare_parameter(
            "robot_api_request_timeout_sec", 0.5).value)

        self.values = {
            "mission_state": "未知",
            "current_target_cabinet": "未知",
            "recognition": "未知",
            "voltage": "未知",
            "charging": "未知",
            "charging_flag_detail": "未知（robot_charging_flag=未知）",
            "charging_current": "未知",
            "red_flag": "未知",
            "recharge_flag": "未知",
            "recharge_flag_detail": "未知（robot_recharge_flag=未知）",
            "auto_recharge_status": "未知",
            "lift_base_height": "%.2f m" % LIFT_BASE_HEIGHT_M,
            "lift_travel_height": "未知",
            "lift_total_height": "未知",
            "lift_state": "未知",
            "lift_error": "",
            "robot_api_service": "未连接",
            "robot_api_address": self.robot_api_status_url,
            "robot_api_receive": "暂无",
            "robot_api_upload": "暂无上传",
            "robot_api_upload_message": "暂无上传记录",
            "robot_api_exception": "无",
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
            self.create_subscription(String, self.auto_recharge_status_topic,
                                     self.auto_recharge_status_callback, 10),
            self.create_subscription(LiftState, self.lift_state_topic, self.lift_state_callback, 10),
        ]

        self.start_mission_client = self.create_client(
            StartMission, self.start_mission_service_name)
        self.cancel_mission_client = self.create_client(
            Trigger, self.cancel_mission_service_name)
        self.safe_exit_gap_client = self.create_client(
            Trigger, self.safe_exit_gap_service_name)
        self.return_home_client = self.create_client(
            Trigger, self.return_home_service_name)
        self.return_to_charge_client = self.create_client(
            Trigger, self.return_to_charge_service_name)
        self.cancel_auto_recharge_client = self.create_client(
            Trigger, self.cancel_auto_recharge_service_name)
        self.stop_auto_charge_and_depart_client = self.create_client(
            Trigger, self.stop_auto_charge_and_depart_service_name)
        self.auto_recharge_start_client = self.create_client(
            Trigger, self.auto_recharge_start_service_name)
        self.lift_up_client = self.create_client(LiftMoveTimed, self.lift_up_service_name)
        self.lift_down_client = self.create_client(LiftMoveTimed, self.lift_down_service_name)
        self.lift_stop_client = self.create_client(Trigger, self.lift_stop_service_name)
        self.lift_all_off_client = self.create_client(Trigger, self.lift_all_off_service_name)
        self.lift_reset_estimated_height_client = self.create_client(
            Trigger, self.lift_reset_estimated_height_service_name)

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
        raw = 1 if msg.data else 0
        text = "充电中" if msg.data else "未充电"
        self.values["charging"] = text
        self.values["charging_flag_detail"] = "%s（robot_charging_flag=%d）" % (text, raw)

    def charging_current_callback(self, msg):
        self.values["charging_current"] = "%.3f A" % msg.data

    def red_flag_callback(self, msg):
        self.values["red_flag"] = str(msg.data)

    def recharge_flag_callback(self, msg):
        text = "开启" if msg.data else "关闭"
        self.values["recharge_flag"] = text
        self.values["recharge_flag_detail"] = "%s（robot_recharge_flag=%d）" % (text, msg.data)

    def auto_recharge_status_callback(self, msg):
        status = msg.data.strip() or "UNKNOWN"
        text = AUTO_RECHARGE_STATUS_TEXT.get(status, "未知状态")
        self.values["auto_recharge_status"] = "%s / %s" % (status, text)

    def lift_state_callback(self, msg):
        # LiftState only provides estimated_height_m, so the GUI height display
        # reuses the existing estimated lift travel rather than a real sensor feedback value.
        estimated_lift_height_m = float(msg.estimated_height_m)
        estimated_lift_height_mm = estimated_lift_height_m * 1000.0
        total_lift_height_m = LIFT_BASE_HEIGHT_M + estimated_lift_height_m
        self.values["lift_travel_height"] = (
            "%.0f mm / %.3f m" % (estimated_lift_height_mm, estimated_lift_height_m))
        self.values["lift_total_height"] = "%.3f m" % total_lift_height_m
        self.values["lift_state"] = msg.state
        self.values["lift_error"] = msg.error_message


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
        self.status_tiles = {}
        self.selected_cabinets = []
        self.cabinet_buttons = {}
        self.robot_api_status_lock = threading.Lock()
        self.robot_api_status_data = {
            "service_state": "未连接",
            "address": self.node.robot_api_status_url,
            "receive": "暂无",
            "upload": "暂无上传",
            "upload_message": "暂无上传记录",
            "exception": "无",
        }
        self.robot_api_poll_running = False
        self.last_robot_api_poll_ms = 0
        self.selected_cabinets_var = self.tk.StringVar(value="当前选择：无")
        self.target_gap_var = self.tk.StringVar(value="")
        self.command_status = self.tk.StringVar(value="等待操作")
        self.error_status = self.tk.StringVar(value="无")
        self.build_widgets()
        self.refresh()

    def build_widgets(self):
        self.configure_styles()
        self.root.geometry("%dx%d" % (WINDOW_WIDTH, WINDOW_HEIGHT))
        self.root.minsize(WINDOW_WIDTH, WINDOW_HEIGHT)
        self.root.resizable(True, True)
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        self.root.configure(bg=self.colors["window"])

        main = self.ttk.Frame(self.root, padding=12, style="App.TFrame")
        main.grid(row=0, column=0, sticky="nsew")
        main.columnconfigure(0, weight=4)
        main.columnconfigure(1, weight=3)
        main.rowconfigure(0, weight=0, minsize=STATUS_BAR_HEIGHT)
        main.rowconfigure(1, weight=1)
        main.rowconfigure(2, weight=0, minsize=LOG_FRAME_HEIGHT)

        status_bar = self.ttk.Frame(main, style="App.TFrame")
        status_bar.grid(row=0, column=0, columnspan=2, sticky="nsew", pady=(0, 10))
        status_bar.configure(height=STATUS_BAR_HEIGHT)
        status_bar.grid_propagate(False)
        status_bar.rowconfigure(0, minsize=STATUS_TILE_HEIGHT)
        for column_index in range(5):
            status_bar.columnconfigure(column_index, weight=1, uniform="status")
        self.add_status_tile(status_bar, 0, "任务状态", "mission_state_overview")
        self.add_status_tile(status_bar, 1, "自动回充", "auto_recharge_status")
        self.add_status_tile(status_bar, 2, "充电状态", "charging")
        self.add_status_tile(status_bar, 3, "当前目标", "current_target_cabinet")
        self.add_status_tile(status_bar, 4, "更新时间", "last_update")

        operation_frame = self.ttk.LabelFrame(
            main, text="任务操作区", padding=10, style="Panel.TLabelframe")
        operation_frame.grid(row=1, column=0, sticky="nsew", padx=(0, 12))
        operation_frame.columnconfigure(0, weight=1)
        operation_frame.rowconfigure(0, weight=0)
        operation_frame.rowconfigure(1, weight=0)
        operation_frame.rowconfigure(2, weight=1, minsize=82)
        operation_frame.rowconfigure(3, weight=2, minsize=190)
        operation_frame.rowconfigure(4, weight=0)

        cabinet_select_frame = self.ttk.LabelFrame(
            operation_frame, text="目标货柜选择", padding=8, style="Panel.TLabelframe")
        cabinet_select_frame.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        cabinet_select_frame.columnconfigure(0, weight=0)
        for column_index in range(1, 19):
            cabinet_select_frame.columnconfigure(column_index, weight=1, uniform="cabinet")
        self.ttk.Label(
            cabinet_select_frame,
            textvariable=self.selected_cabinets_var,
            width=SELECTED_CABINETS_MAX_CHARS,
            style="Value.TLabel").grid(
            row=0, column=0, columnspan=15, sticky="w", pady=(0, 6))
        self.ttk.Button(
            cabinet_select_frame, text="清空选择", command=self.clear_cabinet_selection).grid(
            row=0, column=15, columnspan=4, sticky="e", pady=(0, 6))
        self.ttk.Label(cabinet_select_frame, text="上排/左排").grid(
            row=1, column=0, sticky="w", padx=(0, 6), pady=2)
        self.ttk.Label(cabinet_select_frame, text="下排/右排").grid(
            row=2, column=0, sticky="w", padx=(0, 6), pady=2)
        for cabinet_id in range(1, 19):
            self.add_cabinet_button(cabinet_select_frame, 1, cabinet_id)
        for cabinet_id in range(19, 37):
            self.add_cabinet_button(cabinet_select_frame, 2, cabinet_id)

        task_input_frame = self.ttk.LabelFrame(
            operation_frame, text="任务设置", padding=8, style="Panel.TLabelframe")
        task_input_frame.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        task_input_frame.columnconfigure(1, weight=1)
        task_input_frame.columnconfigure(3, weight=2)
        self.ttk.Label(task_input_frame, text="目标间隙 ID").grid(
            row=0, column=0, sticky="w", padx=(0, 8), pady=3)
        self.ttk.Entry(task_input_frame, textvariable=self.target_gap_var).grid(
            row=0, column=1, sticky="ew", pady=3)
        self.add_var_row(task_input_frame, 0, "当前选择", self.selected_cabinets_var, column=2)

        inventory_button_frame = self.ttk.Frame(operation_frame, style="App.TFrame")
        inventory_button_frame.grid(row=2, column=0, sticky="nsew", pady=(4, 12))
        inventory_button_frame.rowconfigure(0, weight=1, minsize=64)
        for column_index in range(3):
            inventory_button_frame.columnconfigure(column_index, weight=1, uniform="inventory_action")
        self.start_inventory_button = self.make_action_button(
            inventory_button_frame, ACTION_START_INVENTORY,
            self.request_start_inventory, "primary", large=True)
        self.start_inventory_button.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        self.full_inventory_button = self.make_action_button(
            inventory_button_frame, ACTION_FULL_INVENTORY,
            self.request_full_inventory, "primary", large=True)
        self.full_inventory_button.grid(row=0, column=1, sticky="nsew", padx=(0, 10))
        self.cancel_mission_button = self.make_action_button(
            inventory_button_frame, ACTION_CANCEL_MISSION,
            self.request_cancel_mission, "danger", large=True)
        self.cancel_mission_button.grid(row=0, column=2, sticky="nsew")

        safety_frame = self.ttk.LabelFrame(
            operation_frame, text="安全控制 / 回充控制区", padding=10, style="Panel.TLabelframe")
        safety_frame.grid(row=3, column=0, sticky="nsew")
        safety_frame.columnconfigure(0, weight=1)
        safety_frame.rowconfigure(0, weight=0)
        safety_frame.rowconfigure(1, weight=1, minsize=72)
        safety_frame.rowconfigure(2, weight=0)
        safety_frame.rowconfigure(3, weight=1, minsize=82)
        self.ttk.Label(safety_frame, text="回充控制", style="SectionTitle.TLabel").grid(
            row=0, column=0, sticky="w", pady=(0, 6))
        charge_control_frame = self.ttk.Frame(safety_frame, style="Card.TFrame")
        charge_control_frame.grid(row=1, column=0, sticky="nsew", pady=(0, 14))
        charge_control_frame.rowconfigure(0, weight=1, minsize=72)
        for column_index in range(2):
            charge_control_frame.columnconfigure(column_index, weight=1, uniform="charge_control")
        self.return_to_charge_button = self.make_action_button(
            charge_control_frame, ACTION_RETURN_TO_CHARGE,
            self.request_return_to_charge, "primary", large=True)
        self.return_to_charge_button.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        self.cancel_auto_recharge_button = self.make_action_button(
            charge_control_frame, ACTION_CANCEL_AUTO_RECHARGE,
            self.request_cancel_auto_recharge, "warning", large=True)
        self.cancel_auto_recharge_button.grid(row=0, column=1, sticky="nsew")

        self.ttk.Label(safety_frame, text="安全动作", style="SectionTitle.TLabel").grid(
            row=2, column=0, sticky="w", pady=(0, 6))
        safe_action_frame = self.ttk.Frame(safety_frame, style="Card.TFrame")
        safe_action_frame.grid(row=3, column=0, sticky="nsew")
        safe_action_frame.rowconfigure(0, weight=1, minsize=82)
        for column_index in range(3):
            safe_action_frame.columnconfigure(column_index, weight=1, uniform="safe_action")
        self.safe_exit_gap_button = self.make_action_button(
            safe_action_frame, ACTION_SAFE_EXIT_GAP,
            self.request_safe_exit_gap, "danger", large=True)
        self.safe_exit_gap_button.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        self.stop_auto_charge_and_depart_button = self.make_action_button(
            safe_action_frame, ACTION_STOP_AUTO_CHARGE_AND_DEPART,
            self.request_stop_auto_charge_and_depart, "warning", large=True)
        self.stop_auto_charge_and_depart_button.grid(row=0, column=1, sticky="nsew", padx=(0, 10))
        self.return_home_button = self.make_action_button(
            safe_action_frame, ACTION_RETURN_HOME,
            self.request_return_home, "warning", large=True)
        self.return_home_button.grid(row=0, column=2, sticky="nsew")

        robot_api_frame = self.ttk.LabelFrame(
            operation_frame, text="网页/上传状态", padding=8, style="Panel.TLabelframe")
        robot_api_frame.grid(row=4, column=0, sticky="ew", pady=(10, 0))
        robot_api_frame.columnconfigure(1, weight=1)
        robot_api_rows = [
            ("网页服务", "robot_api_service"),
            ("服务地址", "robot_api_address"),
            ("任务接收", "robot_api_receive"),
            ("扫码上传", "robot_api_upload"),
            ("上传消息", "robot_api_upload_message"),
            ("最近异常", "robot_api_exception"),
        ]
        for row_index, (label, key) in enumerate(robot_api_rows):
            self.add_value_row(
                robot_api_frame,
                row_index,
                label,
                key,
                max_chars=ROBOT_API_STATUS_MAX_CHARS,
                value_style="Value.TLabel")

        right_frame = self.ttk.Frame(main, style="App.TFrame")
        right_frame.grid(row=1, column=1, sticky="nsew")
        right_frame.columnconfigure(0, weight=1)
        right_frame.rowconfigure(3, weight=1)

        task_detail_frame = self.ttk.LabelFrame(
            right_frame, text="任务详情", padding=10, style="Panel.TLabelframe")
        task_detail_frame.grid(row=0, column=0, sticky="ew")
        task_detail_frame.columnconfigure(1, weight=1)
        task_rows = [
            ("盘库状态", "mission_state"),
            ("当前目标柜号", "current_target_cabinet"),
            ("识别结果", "recognition"),
        ]
        for row_index, (label, key) in enumerate(task_rows):
            self.add_value_row(task_detail_frame, row_index, label, key)

        charge_frame = self.ttk.LabelFrame(
            right_frame, text="回充与电源", padding=10, style="Panel.TLabelframe")
        charge_frame.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        charge_frame.columnconfigure(1, weight=1)
        charge_rows = [
            ("电压", "voltage"),
            ("充电标志", "charging_flag_detail"),
            ("充电电流", "charging_current"),
            ("自动回充状态", "auto_recharge_status"),
            ("红外状态", "red_flag"),
            ("回充标志", "recharge_flag_detail"),
        ]
        for row_index, (label, key) in enumerate(charge_rows):
            self.add_value_row(charge_frame, row_index, label, key)

        lift_frame = self.ttk.LabelFrame(
            right_frame, text="升降杆", padding=10, style="Panel.TLabelframe")
        lift_frame.grid(row=2, column=0, sticky="new", pady=(10, 0))
        lift_frame.columnconfigure(1, weight=1)
        lift_rows = [
            ("初始高度", "lift_base_height", "Value.TLabel"),
            ("估算升降高度", "lift_travel_height", "Value.TLabel"),
            ("当前总高度", "lift_total_height", "StrongValue.TLabel"),
            ("状态", "lift_state", "Value.TLabel"),
            ("错误", "lift_error", "Value.TLabel"),
        ]
        for row_index, (label, key, value_style) in enumerate(lift_rows):
            self.add_value_row(lift_frame, row_index, label, key, value_style=value_style)

        lift_button_frame = self.ttk.Frame(lift_frame)
        lift_button_frame.grid(
            row=len(lift_rows), column=0, columnspan=2, sticky="ew", pady=(10, 0))
        for column_index in range(5):
            lift_button_frame.columnconfigure(column_index, weight=1, uniform="lift_action")
        self.lift_up_button = self.make_action_button(
            lift_button_frame, "上升", self.request_lift_up, "secondary", width=8)
        self.lift_up_button.grid(row=0, column=0, sticky="ew", padx=(0, 6))
        self.lift_down_button = self.make_action_button(
            lift_button_frame, "下降", self.request_lift_down, "secondary", width=8)
        self.lift_down_button.grid(row=0, column=1, sticky="ew", padx=(0, 6))
        self.lift_stop_button = self.make_action_button(
            lift_button_frame, "停止", self.request_lift_stop, "danger", width=8)
        self.lift_stop_button.grid(row=0, column=2, sticky="ew", padx=(0, 6))
        self.lift_all_off_button = self.make_action_button(
            lift_button_frame, "全关", self.request_lift_all_off, "danger", width=8)
        self.lift_all_off_button.grid(row=0, column=3, sticky="ew", padx=(0, 6))
        self.lift_reset_estimated_height_button = self.make_action_button(
            lift_button_frame, "预估置零", self.request_lift_reset_estimated_height,
            "secondary", width=10)
        self.lift_reset_estimated_height_button.grid(row=0, column=4, sticky="ew")

        service_frame = self.ttk.LabelFrame(
            right_frame, text="接口状态", padding=10, style="Panel.TLabelframe")
        service_frame.grid(row=3, column=0, sticky="nsew", pady=(10, 0))
        service_frame.columnconfigure(1, weight=1)
        service_rows = [
            ("盘库", "start_mission_service"),
            ("中断", "cancel_mission_service"),
            ("安全出缝", "safe_exit_gap_service"),
            ("回零点", "return_home_service"),
            ("启动回充", "return_to_charge_service"),
            ("取消回充", "cancel_auto_recharge_service"),
            ("离桩", "stop_auto_charge_and_depart_service"),
            ("回充底层", "auto_recharge_start_service"),
            ("升降杆上/下/停", "lift_service_group"),
        ]
        for row_index, (label, key) in enumerate(service_rows):
            self.add_value_row(service_frame, row_index, label, key)

        log_frame = self.ttk.LabelFrame(
            main, text="日志与异常", padding=12, style="Panel.TLabelframe")
        log_frame.grid(row=2, column=0, columnspan=2, sticky="nsew", pady=(12, 0))
        log_frame.configure(height=LOG_FRAME_HEIGHT)
        log_frame.grid_propagate(False)
        log_frame.columnconfigure(1, weight=1)
        log_frame.columnconfigure(2, weight=0)
        log_frame.rowconfigure(3, weight=1)
        self.add_var_row(log_frame, 0, "最近操作", self.command_status)
        self.ttk.Button(
            log_frame, text="清空日志", command=self.clear_logs).grid(
            row=0, column=2, sticky="e", padx=(10, 0))
        self.add_var_row(log_frame, 1, "异常信息", self.error_status, value_style="ErrorValue.TLabel")
        self.ttk.Label(log_frame, text="详细日志", style="Key.TLabel").grid(
            row=2, column=0, sticky="w", pady=(8, 2))
        self.log_text = self.tk.Text(
            log_frame, width=110, height=5, wrap=self.tk.WORD,
            bg=self.colors["log_bg"], fg=self.colors["text"],
            relief=self.tk.FLAT, bd=0, padx=10, pady=8,
            font=("TkFixedFont", 9))
        self.log_text.grid(row=3, column=0, columnspan=2, sticky="nsew")
        log_scrollbar = self.ttk.Scrollbar(
            log_frame, orient=self.tk.VERTICAL, command=self.log_text.yview)
        log_scrollbar.grid(row=3, column=2, sticky="ns", padx=(6, 0))
        self.log_text.configure(yscrollcommand=log_scrollbar.set)
        self.log_text.tag_configure("error", foreground=self.colors["danger"])
        self.log_text.tag_configure("warn", foreground=self.colors["warning"])
        self.log_text.tag_configure("normal", foreground=self.colors["text"])
        self.log_text.configure(state=self.tk.DISABLED)

    def configure_styles(self):
        self.colors = {
            "window": "#eef2f6",
            "panel": "#ffffff",
            "panel_border": "#d5dbe3",
            "text": "#172033",
            "muted": "#5f6b7a",
            "primary": "#2563eb",
            "primary_active": "#1d4ed8",
            "secondary": "#e7ecf2",
            "secondary_active": "#d8e0ea",
            "secondary_text": "#172033",
            "danger": "#b42318",
            "danger_active": "#931b12",
            "warning": "#b45309",
            "warning_active": "#92400e",
            "log_bg": "#f8fafc",
            "selected": "#2563eb",
            "unselected": "#f5f7fb",
            "idle_bg": "#eef7f2",
            "idle_fg": "#166534",
            "running_bg": "#eff6ff",
            "running_fg": "#1d4ed8",
            "gap_bg": "#fff7ed",
            "gap_fg": "#c2410c",
            "error_bg": "#fef2f2",
            "error_fg": "#b42318",
            "charging_bg": "#ecfdf5",
            "charging_fg": "#047857",
            "neutral_bg": "#f8fafc",
            "neutral_fg": "#334155",
        }
        style = self.ttk.Style()
        try:
            style.theme_use("clam")
        except self.tk.TclError:
            pass
        style.configure("App.TFrame", background=self.colors["window"])
        style.configure("Card.TFrame", background=self.colors["panel"])
        style.configure("Panel.TLabelframe", background=self.colors["window"])
        style.configure(
            "Panel.TLabelframe.Label",
            background=self.colors["window"],
            foreground=self.colors["text"],
            font=("TkDefaultFont", 10, "bold"))
        style.configure("Key.TLabel", background=self.colors["window"], foreground=self.colors["muted"])
        style.configure("Value.TLabel", background=self.colors["window"], foreground=self.colors["text"])
        style.configure(
            "StrongValue.TLabel",
            background=self.colors["window"],
            foreground=self.colors["text"],
            font=("TkDefaultFont", 11, "bold"))
        style.configure("ErrorValue.TLabel", background=self.colors["window"], foreground=self.colors["danger"])
        style.configure(
            "SectionTitle.TLabel",
            background=self.colors["window"],
            foreground=self.colors["muted"],
            font=("TkDefaultFont", 9, "bold"))
        style.configure("TButton", padding=(10, 5))

    def add_status_tile(self, parent, column_index, label, key):
        tile = self.tk.Frame(
            parent,
            width=STATUS_TILE_WIDTH,
            height=STATUS_TILE_HEIGHT,
            bg=self.colors["panel"],
            highlightbackground=self.colors["panel_border"],
            highlightthickness=1,
            padx=12,
            pady=7)
        tile.grid(row=0, column=column_index, sticky="nsew", padx=(0, 8))
        tile.grid_propagate(False)
        tile.columnconfigure(0, weight=1)
        label_widget = self.tk.Label(
            tile, text=label, bg=self.colors["panel"], fg=self.colors["muted"],
            anchor="w", font=("TkDefaultFont", 9), width=18, height=1)
        label_widget.grid(row=0, column=0, sticky="ew")
        var = self.tk.StringVar(value="未知")
        self.register_field(key, var, STATUS_MAIN_MAX_CHARS)
        value_widget = self.tk.Label(
            tile, textvariable=var, bg=self.colors["panel"], fg=self.colors["text"],
            anchor="w", font=("TkDefaultFont", 11, "bold"),
            width=18, height=1, justify=self.tk.LEFT)
        value_widget.grid(row=1, column=0, sticky="ew", pady=(2, 0))
        detail_var = self.tk.StringVar(value="")
        self.register_field("%s_detail" % key, detail_var, STATUS_DETAIL_MAX_CHARS)
        detail_widget = self.tk.Label(
            tile, textvariable=detail_var, bg=self.colors["panel"], fg=self.colors["muted"],
            anchor="w", font=("TkDefaultFont", 8), width=30, height=1, justify=self.tk.LEFT)
        detail_widget.grid(row=2, column=0, sticky="ew", pady=(1, 0))
        self.status_tiles[key] = {
            "frame": tile,
            "label": label_widget,
            "value": value_widget,
            "detail": detail_widget,
        }

    def make_action_button(self, parent, text, command, variant, width=14, large=False):
        palettes = {
            "primary": (self.colors["primary"], "white", self.colors["primary_active"]),
            "secondary": (
                self.colors["secondary"], self.colors["secondary_text"],
                self.colors["secondary_active"]),
            "danger": (self.colors["danger"], "white", self.colors["danger_active"]),
            "warning": (self.colors["warning"], "white", self.colors["warning_active"]),
        }
        bg, fg, active_bg = palettes[variant]
        button_options = {}
        if large:
            button_options.update({
                "font": MAIN_ACTION_BUTTON_FONT,
                "height": 2,
                "padx": 14,
                "pady": 10,
            })
        else:
            button_options["height"] = 2
        return self.tk.Button(
            parent,
            text=text,
            command=command,
            width=width,
            relief=self.tk.FLAT,
            bd=0,
            bg=bg,
            fg=fg,
            activebackground=active_bg,
            activeforeground=fg,
            disabledforeground="#9aa4b2",
            cursor="hand2",
            **button_options)

    def add_var_row(self, parent, row_index, label, variable, column=0, value_style="Value.TLabel"):
        self.ttk.Label(parent, text=label, style="Key.TLabel", width=10).grid(
            row=row_index, column=column, sticky="w", padx=(0, 10), pady=3)
        self.ttk.Label(
            parent,
            textvariable=variable,
            width=WIDE_VALUE_MAX_CHARS,
            style=value_style).grid(
            row=row_index, column=column + 1, sticky="ew", pady=3)

    def add_value_row(
            self, parent, row_index, label, key, max_chars=VALUE_MAX_CHARS,
            value_style="Value.TLabel"):
        self.ttk.Label(parent, text=label, style="Key.TLabel", width=14).grid(
            row=row_index, column=0, sticky="w", padx=(0, 10), pady=3)
        var = self.tk.StringVar(value="未知")
        self.register_field(key, var, max_chars)
        self.ttk.Label(
            parent,
            textvariable=var,
            width=max_chars,
            style=value_style).grid(
            row=row_index, column=1, sticky="ew", pady=3)

    def register_field(self, key, variable, max_chars=None):
        self.fields.setdefault(key, []).append((variable, max_chars))

    def set_field(self, key, value):
        for variable, max_chars in self.fields.get(key, []):
            variable.set(self.format_one_line(value, max_chars))

    def mission_state_indicates_in_gap(self):
        state = self.node.values.get("mission_state", "")
        return any(token in state for token in GAP_PROTECTED_TOKENS)

    def add_cabinet_button(self, parent, row_index, cabinet_id):
        column_index = ((cabinet_id - 1) % 18) + 1
        button = self.tk.Button(
            parent,
            text=str(cabinet_id),
            width=3,
            height=1,
            relief=self.tk.RAISED,
            bd=1,
            bg=self.colors["unselected"],
            fg=self.colors["text"],
            activebackground="#dbe5f4",
            activeforeground=self.colors["text"],
            command=lambda value=cabinet_id: self.select_cabinet(value))
        button.grid(row=row_index, column=column_index, padx=2, pady=3)
        self.cabinet_buttons[cabinet_id] = button

    def select_cabinet(self, cabinet_id):
        if cabinet_id in self.selected_cabinets:
            self.selected_cabinets.remove(cabinet_id)
        else:
            self.selected_cabinets.append(cabinet_id)
        self.update_cabinet_selection_display()

    def clear_cabinet_selection(self):
        self.selected_cabinets.clear()
        self.update_cabinet_selection_display()

    def update_cabinet_selection_display(self):
        if self.selected_cabinets:
            text = "当前选择：" + ", ".join(str(cabinet_id) for cabinet_id in self.selected_cabinets)
        else:
            text = "当前选择：无"
        self.selected_cabinets_var.set(
            self.format_one_line(text, SELECTED_CABINETS_MAX_CHARS))
        for button_id, button in self.cabinet_buttons.items():
            if button_id in self.selected_cabinets:
                button.configure(relief=self.tk.SUNKEN, bg=self.colors["selected"], fg="white")
            else:
                button.configure(
                    relief=self.tk.RAISED,
                    bg=self.colors["unselected"],
                    fg=self.colors["text"])

    def clear_logs(self):
        self.node.mission_logs.clear()
        self.last_log_snapshot = ()
        self.log_text.configure(state=self.tk.NORMAL)
        self.log_text.delete("1.0", self.tk.END)
        self.log_text.configure(state=self.tk.DISABLED)
        self.set_command_status("已清空任务日志。")
        self.error_status.set("无")

    def request_start_inventory(self):
        if not self.selected_cabinets:
            message = "请先选择目标货柜"
            self.set_command_status(message, error=True)
            self.node.get_logger().warn(message)
            return
        cabinets = list(self.selected_cabinets)

        request = StartMission.Request()
        request.targets = []
        request.return_home = False
        request.target_gap = self.target_gap_var.get().strip()
        request.scan_cabinets = cabinets
        request.run_full_inventory = len(cabinets) > 1 and not request.target_gap
        self.send_start_mission_request(ACTION_START_INVENTORY, request)

    def request_full_inventory(self):
        request = StartMission.Request()
        request.targets = []
        request.return_home = False
        request.run_full_inventory = True
        request.target_gap = ""
        request.scan_cabinets = []
        self.send_start_mission_request(ACTION_FULL_INVENTORY, request)

    def request_cancel_mission(self):
        self.send_trigger_request(
            ACTION_CANCEL_MISSION,
            self.node.cancel_mission_client,
            self.node.cancel_mission_service_name)

    def request_safe_exit_gap(self):
        self.send_trigger_request(
            ACTION_SAFE_EXIT_GAP,
            self.node.safe_exit_gap_client,
            self.node.safe_exit_gap_service_name)

    def request_return_home(self):
        self.send_trigger_request(
            ACTION_RETURN_HOME,
            self.node.return_home_client,
            self.node.return_home_service_name)

    def request_return_to_charge(self):
        if self.mission_state_indicates_in_gap():
            self.set_command_status("当前可能位于缝隙内，请先执行‘停止并退出缝隙’。", error=True)
            return
        self.send_trigger_request(
            ACTION_RETURN_TO_CHARGE,
            self.node.return_to_charge_client,
            self.node.return_to_charge_service_name)

    def request_cancel_auto_recharge(self):
        self.send_trigger_request(
            ACTION_CANCEL_AUTO_RECHARGE,
            self.node.cancel_auto_recharge_client,
            self.node.cancel_auto_recharge_service_name)

    def request_stop_auto_charge_and_depart(self):
        self.send_trigger_request(
            ACTION_STOP_AUTO_CHARGE_AND_DEPART,
            self.node.stop_auto_charge_and_depart_client,
            self.node.stop_auto_charge_and_depart_service_name)

    def request_lift_up(self):
        self.send_lift_motion_request("升降杆上升", self.node.lift_up_client, self.node.lift_up_service_name, "up")

    def request_lift_down(self):
        self.send_lift_motion_request(
            "升降杆下降", self.node.lift_down_client, self.node.lift_down_service_name, "down")

    def request_lift_stop(self):
        self.send_trigger_request("升降杆停止", self.node.lift_stop_client, self.node.lift_stop_service_name)

    def request_lift_all_off(self):
        self.send_trigger_request(
            "升降杆全关", self.node.lift_all_off_client, self.node.lift_all_off_service_name)

    def request_lift_reset_estimated_height(self):
        self.send_trigger_request(
            "预估高度置零",
            self.node.lift_reset_estimated_height_client,
            self.node.lift_reset_estimated_height_service_name)

    def send_start_mission_request(self, label, request):
        client = self.node.start_mission_client
        service_name = self.node.start_mission_service_name
        if not client.service_is_ready():
            message = "%s 服务不可用: %s" % (label, service_name)
            self.set_command_status(message, error=True)
            self.node.get_logger().warn(message)
            return

        future = client.call_async(request)
        self.pending_futures.append({"label": label, "future": future})
        self.set_command_status("已发送%s请求，等待响应。" % label)
        self.update_button_states()

    def send_trigger_request(self, label, client, service_name):
        if not client.service_is_ready():
            message = "%s 服务不可用: %s" % (label, service_name)
            self.set_command_status(message, error=True)
            self.node.get_logger().warn(message)
            return

        future = client.call_async(Trigger.Request())
        self.pending_futures.append({"label": label, "future": future})
        self.set_command_status("已发送%s请求，等待响应。" % label)
        self.update_button_states()

    def send_lift_motion_request(self, label, client, service_name, direction):
        if not client.service_is_ready():
            message = "%s 服务不可用: %s" % (label, service_name)
            self.set_command_status(message, error=True)
            self.node.get_logger().warn(message)
            return

        request = LiftMoveTimed.Request()
        request.direction = direction
        request.duration_sec = float(self.node.lift_manual_duration_sec)
        future = client.call_async(request)
        self.pending_futures.append({"label": label, "future": future})
        self.set_command_status("已发送%s请求，等待响应。" % label)
        self.update_button_states()

    def set_command_status(self, message, error=False):
        stamp = datetime.now().strftime("%H:%M:%S")
        self.command_status.set(
            self.format_one_line("[%s] %s" % (stamp, message), COMMAND_MAX_CHARS))
        if error:
            self.error_status.set(
                self.format_one_line("[%s] %s" % (stamp, message), ERROR_MAX_CHARS))
        elif self.error_status.get() == "未知":
            self.error_status.set("无")

    def maybe_poll_robot_api_status(self):
        now_ms = int(time.monotonic() * 1000)
        interval_ms = max(500, int(self.node.robot_api_poll_interval_ms))
        if self.robot_api_poll_running:
            return
        if now_ms - self.last_robot_api_poll_ms < interval_ms:
            return
        self.last_robot_api_poll_ms = now_ms
        self.robot_api_poll_running = True
        thread = threading.Thread(
            target=self.poll_robot_api_status_worker,
            name="robot-api-status-poller",
            daemon=True)
        thread.start()

    def poll_robot_api_status_worker(self):
        try:
            status_data = self.fetch_robot_api_status()
        except Exception as exc:
            status_data = {
                "service_state": "未连接",
                "address": self.node.robot_api_status_url,
                "receive": "暂无",
                "upload": "暂无上传",
                "upload_message": "暂无上传记录",
                "exception": str(exc),
            }
        finally:
            with self.robot_api_status_lock:
                self.robot_api_status_data = locals().get("status_data", {
                    "service_state": "异常",
                    "address": self.node.robot_api_status_url,
                    "receive": "暂无",
                    "upload": "暂无上传",
                    "upload_message": "暂无上传记录",
                    "exception": "状态轮询异常",
                })
                self.robot_api_poll_running = False

    def fetch_json_url(self, url):
        request = urllib.request.Request(url, headers={"Accept": "application/json"})
        with urllib.request.urlopen(
                request,
                timeout=max(0.1, float(self.node.robot_api_request_timeout_sec))) as response:
            status_code = getattr(response, "status", response.getcode())
            body = response.read().decode("utf-8", errors="replace")
        try:
            data = json.loads(body)
        except json.JSONDecodeError as exc:
            raise RuntimeError("JSON 解析失败: %s" % exc) from exc
        if not isinstance(data, dict):
            raise RuntimeError("JSON 响应不是对象")
        data["_http_status_code"] = status_code
        return data

    def fetch_robot_api_status(self):
        address = self.robot_api_display_address()
        try:
            data = self.fetch_json_url(self.node.robot_api_status_url)
            if data.get("ok") is True:
                return self.format_robot_api_status(data, address)
            return {
                "service_state": "异常",
                "address": address,
                "receive": "暂无",
                "upload": "暂无上传",
                "upload_message": "暂无上传记录",
                "exception": "status ok=false",
            }
        except Exception as status_exc:
            try:
                health = self.fetch_json_url(self.node.robot_api_health_url)
                if health.get("ok") is True:
                    return {
                        "service_state": "运行中",
                        "address": address,
                        "receive": "暂无",
                        "upload": "暂无上传",
                        "upload_message": "暂无上传记录",
                        "exception": "status异常: %s" % status_exc,
                    }
                return {
                    "service_state": "异常",
                    "address": address,
                    "receive": "暂无",
                    "upload": "暂无上传",
                    "upload_message": "暂无上传记录",
                    "exception": "health ok=false",
                }
            except Exception as health_exc:
                return {
                    "service_state": "未连接",
                    "address": address,
                    "receive": "暂无",
                    "upload": "暂无上传",
                    "upload_message": "暂无上传记录",
                    "exception": str(health_exc),
                }

    def robot_api_display_address(self):
        parsed = urllib.parse.urlparse(self.node.robot_api_status_url)
        if parsed.scheme and parsed.netloc:
            return "%s://%s" % (parsed.scheme, parsed.netloc)
        return self.node.robot_api_status_url

    def format_robot_api_status(self, data, address):
        receive_success = data.get("last_receive_success")
        receive_time = self.short_time(data.get("last_receive_time"))
        receive_message = data.get("last_receive_message") or ""
        if receive_success is True:
            receive = "成功 %s" % receive_time if receive_time else "成功"
        elif receive_success is False:
            receive = "失败 %s" % receive_time if receive_time else "失败"
        else:
            receive = "暂无"
        if receive_message and receive not in ("暂无",):
            receive = "%s %s" % (receive, receive_message)

        upload_success = data.get("last_upload_success")
        upload_time = self.short_time(data.get("last_upload_time"))
        upload_status_code = data.get("last_upload_status_code")
        upload_message = data.get("last_upload_message") or "暂无上传记录"
        if upload_success is True:
            upload = "上传成功"
        elif upload_success is False:
            upload = "上传失败"
        else:
            upload = "暂无上传"
        if upload_time:
            upload = "%s %s" % (upload, upload_time)
        if upload_status_code is not None:
            upload = "%s HTTP %s" % (upload, upload_status_code)

        exception = data.get("last_exception") or "无"
        if upload_success is False and upload_message:
            exception = upload_message
        return {
            "service_state": "运行中",
            "address": address,
            "receive": self.format_one_line(receive, ROBOT_API_STATUS_MAX_CHARS),
            "upload": self.format_one_line(upload, ROBOT_API_STATUS_MAX_CHARS),
            "upload_message": self.format_one_line(upload_message, ROBOT_API_STATUS_MAX_CHARS),
            "exception": self.format_one_line(exception, ROBOT_API_STATUS_MAX_CHARS),
        }

    @staticmethod
    def short_time(value):
        text = str(value or "")
        if not text:
            return ""
        try:
            normalized = text.replace("Z", "+00:00")
            parsed = datetime.fromisoformat(normalized)
            return parsed.strftime("%H:%M:%S")
        except ValueError:
            if len(text) >= 19 and "T" in text:
                return text[11:19]
            return text

    def apply_robot_api_status_fields(self):
        with self.robot_api_status_lock:
            status_data = dict(self.robot_api_status_data)
        self.node.values["robot_api_service"] = status_data.get("service_state", "未连接")
        self.node.values["robot_api_address"] = status_data.get(
            "address", self.node.robot_api_status_url)
        self.node.values["robot_api_receive"] = status_data.get("receive", "暂无")
        self.node.values["robot_api_upload"] = status_data.get("upload", "暂无上传")
        self.node.values["robot_api_upload_message"] = status_data.get(
            "upload_message", "暂无上传记录")
        self.node.values["robot_api_exception"] = status_data.get("exception", "无")

    def refresh(self):
        if self.closing:
            return

        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except Exception as exc:
            self.set_command_status("ROS 回调处理异常: %s" % exc, error=True)
            self.node.get_logger().error("操作总控 GUI 处理 ROS 回调异常: %s" % exc)

        self.collect_completed_futures()
        self.maybe_poll_robot_api_status()
        self.apply_robot_api_status_fields()
        self.update_fields()
        self.update_log_text()
        self.update_button_states()
        self.root.after(REFRESH_PERIOD_MS, self.refresh)

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
                failed = hasattr(response, "success") and not response.success
                failed = failed or (hasattr(response, "accepted") and not response.accepted)
                self.set_command_status(message, error=failed)
                self.node.get_logger().info(message)
            except Exception as exc:
                message = "%s请求异常: %s" % (label, exc)
                self.set_command_status(message, error=True)
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

    @staticmethod
    def normalize_one_line(text):
        value = str(text or "")
        return " ".join(value.replace("\r", " ").replace("\n", " ").split())

    @classmethod
    def format_one_line(cls, text, max_chars=None):
        value = cls.normalize_one_line(text)
        if max_chars is None or max_chars <= 0 or len(value) <= max_chars:
            return value
        if max_chars <= 3:
            return value[:max_chars]
        return value[:max_chars - 3] + "..."

    @classmethod
    def truncate_text(cls, text, max_chars):
        return cls.format_one_line(text, max_chars)

    @staticmethod
    def raw_state_token(raw_state):
        state = (raw_state or "").strip()
        if not state:
            return "UNKNOWN"
        return state.split()[0].strip(":：")

    def format_mission_state_display(self, raw_state):
        token = self.raw_state_token(raw_state)
        if token == "UNKNOWN":
            main_text = "未知状态"
        else:
            main_text = MISSION_STATE_TEXT.get(token, token)
        return (
            self.truncate_text(main_text, STATUS_MAIN_MAX_CHARS),
            self.truncate_text(token, STATUS_DETAIL_MAX_CHARS),
        )

    def mission_state_display(self):
        return self.format_mission_state_display(self.node.values.get("mission_state", "未知"))

    def status_category(self, key, value):
        if key == "mission_state_overview":
            token = self.raw_state_token(self.node.values.get("mission_state", ""))
            if "ERROR" in token or "FAIL" in token:
                return "error"
            if any(token_part in token for token_part in ("ENTERING_GAP", "IN_GAP", "EXIT_GAP", "INVENTORYING", "SAFE_EXIT_GAP")):
                return "gap"
            if token in ("IDLE", "DONE"):
                return "idle"
            if token == "AUTO_RECHARGING":
                return "charging"
            return "running"
        if key == "auto_recharge_status":
            upper = (value or "").upper()
            if "FAILED" in upper:
                return "error"
            if "CHARGING" in upper or "COMPLETE" in upper:
                return "charging"
            if any(token in upper for token in ("STARTING", "NAVIGATING", "DOCKING")):
                return "running"
            if "IDLE" in upper or "CANCELED" in upper:
                return "idle"
            return "neutral"
        if key == "charging":
            return "charging" if "充电中" in value else "idle"
        if key == "current_target_cabinet":
            return "neutral" if value in ("未知", "无目标") else "running"
        return "neutral"

    def set_status_tile_category(self, key, category):
        tile = self.status_tiles.get(key)
        if not tile:
            return
        palettes = {
            "idle": (self.colors["idle_bg"], self.colors["idle_fg"]),
            "running": (self.colors["running_bg"], self.colors["running_fg"]),
            "gap": (self.colors["gap_bg"], self.colors["gap_fg"]),
            "error": (self.colors["error_bg"], self.colors["error_fg"]),
            "charging": (self.colors["charging_bg"], self.colors["charging_fg"]),
            "neutral": (self.colors["neutral_bg"], self.colors["neutral_fg"]),
        }
        bg, fg = palettes.get(category, palettes["neutral"])
        tile["frame"].configure(bg=bg, highlightbackground=fg)
        tile["label"].configure(bg=bg, fg=self.colors["muted"])
        tile["value"].configure(bg=bg, fg=fg)
        tile["detail"].configure(bg=bg, fg=self.colors["muted"])

    @staticmethod
    def ready_group_text(clients):
        ready_count = sum(1 for client in clients if client.service_is_ready())
        return "%d/%d 可用" % (ready_count, len(clients))

    def update_fields(self):
        for key, value in self.node.values.items():
            self.set_field(key, value)
        mission_text, mission_detail = self.mission_state_display()
        mission_full_text = mission_text
        if mission_detail:
            mission_full_text = "%s / %s" % (mission_text, mission_detail)
        self.set_field("mission_state", mission_full_text)
        self.set_field("mission_state_overview", mission_text)
        self.set_field("mission_state_overview_detail", mission_detail)
        self.set_field("auto_recharge_status_detail", "")
        self.set_field("charging_detail", "")
        self.set_field("current_target_cabinet_detail", "")
        self.set_field("last_update_detail", "")
        self.set_field("auto_recharge_status", self.node.values.get("auto_recharge_status", "未知"))
        self.set_field("charging", self.node.values.get("charging", "未知"))
        self.set_field("current_target_cabinet", self.node.values.get("current_target_cabinet", "未知"))
        self.set_field("last_update", datetime.now().strftime("%H:%M:%S"))
        self.set_field("start_mission_service", self.ready_text(self.node.start_mission_client))
        self.set_field("cancel_mission_service", self.ready_text(self.node.cancel_mission_client))
        self.set_field("safe_exit_gap_service", self.ready_text(self.node.safe_exit_gap_client))
        self.set_field("return_home_service", self.ready_text(self.node.return_home_client))
        self.set_field(
            "auto_recharge_start_service", self.ready_text(self.node.auto_recharge_start_client))
        self.set_field("return_to_charge_service", self.ready_text(self.node.return_to_charge_client))
        self.set_field(
            "cancel_auto_recharge_service", self.ready_text(self.node.cancel_auto_recharge_client))
        self.set_field(
            "stop_auto_charge_and_depart_service",
            self.ready_text(self.node.stop_auto_charge_and_depart_client))
        self.set_field("lift_up_service", self.ready_text(self.node.lift_up_client))
        self.set_field("lift_down_service", self.ready_text(self.node.lift_down_client))
        self.set_field("lift_stop_service", self.ready_text(self.node.lift_stop_client))
        self.set_field("lift_all_off_service", self.ready_text(self.node.lift_all_off_client))
        self.set_field(
            "lift_reset_estimated_height_service",
            self.ready_text(self.node.lift_reset_estimated_height_client))
        self.set_field(
            "lift_service_group",
            self.ready_group_text((
                self.node.lift_up_client,
                self.node.lift_down_client,
                self.node.lift_stop_client,
                self.node.lift_all_off_client,
                self.node.lift_reset_estimated_height_client,
            )))
        self.set_status_tile_category(
            "mission_state_overview",
            self.status_category("mission_state_overview", mission_text))
        self.set_status_tile_category(
            "auto_recharge_status",
            self.status_category("auto_recharge_status", self.node.values.get("auto_recharge_status", "")))
        self.set_status_tile_category(
            "charging",
            self.status_category("charging", self.node.values.get("charging", "")))
        self.set_status_tile_category(
            "current_target_cabinet",
            self.status_category(
                "current_target_cabinet", self.node.values.get("current_target_cabinet", "")))
        self.set_status_tile_category("last_update", "neutral")
        lift_error = self.node.values.get("lift_error", "")
        if lift_error:
            self.error_status.set(self.format_one_line(lift_error, ERROR_MAX_CHARS))

    def update_log_text(self):
        snapshot = tuple(self.node.mission_logs)
        if snapshot == self.last_log_snapshot:
            return
        self.last_log_snapshot = snapshot
        self.log_text.configure(state=self.tk.NORMAL)
        self.log_text.delete("1.0", self.tk.END)
        latest_error = ""
        for line in snapshot:
            tag = "normal"
            upper_line = line.upper()
            if "ERROR" in upper_line or "FAILED" in upper_line or "失败" in line or "异常" in line:
                tag = "error"
                latest_error = line
            elif "WARN" in upper_line or "警告" in line:
                tag = "warn"
            self.log_text.insert(self.tk.END, line + "\n", tag)
        if latest_error:
            self.error_status.set(self.format_one_line(latest_error, ERROR_MAX_CHARS))
        elif not self.node.values.get("lift_error", "") and not self.pending_futures:
            self.error_status.set("无")
        self.log_text.see(self.tk.END)
        self.log_text.configure(state=self.tk.DISABLED)

    def update_button_states(self):
        start_pending = self.is_pending(ACTION_START_INVENTORY)
        full_pending = self.is_pending(ACTION_FULL_INVENTORY)
        mission_start_pending = start_pending or full_pending
        cancel_mission_pending = self.is_pending(ACTION_CANCEL_MISSION)
        safe_exit_gap_pending = self.is_pending(ACTION_SAFE_EXIT_GAP)
        return_home_pending = self.is_pending(ACTION_RETURN_HOME)
        return_to_charge_pending = self.is_pending(ACTION_RETURN_TO_CHARGE)
        cancel_auto_recharge_pending = self.is_pending(ACTION_CANCEL_AUTO_RECHARGE)
        stop_auto_charge_and_depart_pending = self.is_pending(ACTION_STOP_AUTO_CHARGE_AND_DEPART)
        lift_up_pending = self.is_pending("升降杆上升")
        lift_down_pending = self.is_pending("升降杆下降")
        lift_stop_pending = self.is_pending("升降杆停止")
        lift_all_off_pending = self.is_pending("升降杆全关")
        lift_reset_estimated_height_pending = self.is_pending("预估高度置零")

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
        self.safe_exit_gap_button.configure(
            state=self.tk.NORMAL
            if self.node.safe_exit_gap_client.service_is_ready() and not safe_exit_gap_pending
            else self.tk.DISABLED)
        self.return_home_button.configure(
            state=self.tk.NORMAL
            if self.node.return_home_client.service_is_ready() and not return_home_pending
            else self.tk.DISABLED)
        self.return_to_charge_button.configure(
            state=self.tk.NORMAL
            if self.node.return_to_charge_client.service_is_ready() and not return_to_charge_pending
            else self.tk.DISABLED)
        self.cancel_auto_recharge_button.configure(
            state=self.tk.NORMAL
            if self.node.cancel_auto_recharge_client.service_is_ready() and not cancel_auto_recharge_pending
            else self.tk.DISABLED)
        self.stop_auto_charge_and_depart_button.configure(
            state=self.tk.NORMAL
            if self.node.stop_auto_charge_and_depart_client.service_is_ready() and
            not stop_auto_charge_and_depart_pending
            else self.tk.DISABLED)
        self.lift_up_button.configure(
            state=self.tk.NORMAL
            if self.node.lift_up_client.service_is_ready() and not lift_up_pending
            else self.tk.DISABLED)
        self.lift_down_button.configure(
            state=self.tk.NORMAL
            if self.node.lift_down_client.service_is_ready() and not lift_down_pending
            else self.tk.DISABLED)
        self.lift_stop_button.configure(
            state=self.tk.NORMAL
            if self.node.lift_stop_client.service_is_ready() and not lift_stop_pending
            else self.tk.DISABLED)
        self.lift_all_off_button.configure(
            state=self.tk.NORMAL
            if self.node.lift_all_off_client.service_is_ready() and not lift_all_off_pending
            else self.tk.DISABLED)
        self.lift_reset_estimated_height_button.configure(
            state=self.tk.NORMAL
            if self.node.lift_reset_estimated_height_client.service_is_ready() and
            not lift_reset_estimated_height_pending
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
