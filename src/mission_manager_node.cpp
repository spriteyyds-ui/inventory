#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "agv_inventory_system/id_utils.hpp"
#include "agv_inventory_system/inventory_scanner.hpp"
#include "agv_inventory_system/msg/gap_status.hpp"
#include "agv_inventory_system/msg/recognized_number.hpp"
#include "agv_inventory_system/rfid_scan_log_writer.hpp"
#include "agv_inventory_system/scan_sequence_generator.hpp"
#include "agv_inventory_system/srv/lift_move_timed.hpp"
#include "agv_inventory_system/srv/start_mission.hpp"
#include "agv_inventory_system/web_api_client.hpp"
#include "yaml-cpp/yaml.h"

class MissionManagerNode : public rclcpp::Node
{
public:
  MissionManagerNode()
  : Node("mission_manager_node")
  {
    recognized_topic_ = declare_parameter<std::string>("recognized_topic", "/inventory/recognized_number");
    distance_topic_ = declare_parameter<std::string>("distance_topic", "/inventory/target_distance");
    gap_topic_ = declare_parameter<std::string>("gap_topic", "/inventory/gap_status");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom_combined");
    odom_fallback_topic_ = declare_parameter<std::string>("odom_fallback_topic", "/odom");
    odom_primary_timeout_sec_ = declare_parameter<double>("odom_primary_timeout_sec", 0.8);
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");

    mission_state_topic_ = declare_parameter<std::string>("mission_state_topic", "/inventory/mission_state");
    mission_log_topic_ = declare_parameter<std::string>("mission_log_topic", "/inventory/mission_log");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    motion_model_ = declare_parameter<std::string>("motion_model", "diff_drive");

    corridor_enable_topic_ =
      declare_parameter<std::string>("corridor_enable_topic", "/inventory/corridor_enable");
    corridor_reverse_topic_ =
      declare_parameter<std::string>("corridor_reverse_topic", "/inventory/corridor_reverse");
    gap_context_topic_ =
      declare_parameter<std::string>("gap_context_topic", "/inventory/gap_context");
    recognizer_enable_topic_ =
      declare_parameter<std::string>("recognizer_enable_topic", "/inventory/recognizer_enable");
    gap_detector_enable_topic_ =
      declare_parameter<std::string>("gap_detector_enable_topic", "/inventory/gap_detector_enable");
    entry_side_topic_ =
      declare_parameter<std::string>("entry_side_topic", "/inventory/entry_side");
    target_lidar_side_topic_ =
      declare_parameter<std::string>("target_lidar_side_topic", "/inventory/target_lidar_side");
    current_target_cabinet_topic_ =
      declare_parameter<std::string>(
      "current_target_cabinet_topic", "/inventory/current_target_cabinet");
    distance_estimator_enable_topic_ =
      declare_parameter<std::string>(
      "distance_estimator_enable_topic", "/inventory/distance_estimator_enable");

    start_service_name_ = declare_parameter<std::string>("start_service_name", "/inventory/start_mission");
    cancel_service_name_ = declare_parameter<std::string>("cancel_service_name", "/inventory/cancel_mission");
    return_home_service_name_ =
      declare_parameter<std::string>("return_home_service_name", "/inventory/return_home");
    return_to_charge_service_name_ =
      declare_parameter<std::string>("return_to_charge_service_name", "/inventory/return_to_charge");
    safe_exit_gap_service_name_ =
      declare_parameter<std::string>("safe_exit_gap_service_name", "/inventory/safe_exit_gap");
    stop_auto_charge_and_depart_service_name_ =
      declare_parameter<std::string>(
      "stop_auto_charge_and_depart_service_name", "/inventory/stop_auto_charge_and_depart");
    inventory_auto_recharge_start_service_name_ =
      declare_parameter<std::string>(
      "inventory_auto_recharge_start_service_name", "/inventory/auto_recharge/start");
    inventory_auto_recharge_cancel_service_name_ =
      declare_parameter<std::string>(
      "inventory_auto_recharge_cancel_service_name", "/inventory/auto_recharge/cancel");
    cancel_auto_recharge_service_name_ =
      declare_parameter<std::string>(
      "cancel_auto_recharge_service_name", "/inventory/cancel_auto_recharge");
    auto_recharge_status_topic_ =
      declare_parameter<std::string>("auto_recharge_status_topic", "/inventory/auto_recharge/status");
    auto_recharge_charging_flag_topic_ =
      declare_parameter<std::string>("auto_recharge_charging_flag_topic", "robot_charging_flag");
    auto_recharge_recharge_flag_topic_ =
      declare_parameter<std::string>("auto_recharge_recharge_flag_topic", "robot_recharge_flag");
    auto_recharge_service_timeout_sec_ =
      declare_parameter<double>("auto_recharge_service_timeout_sec", 3.0);
    auto_recharge_cancel_timeout_sec_ =
      declare_parameter<double>("auto_recharge_cancel_timeout_sec", 3.0);
    auto_recharge_cancel_response_timeout_sec_ =
      declare_parameter<double>("auto_recharge_cancel_response_timeout_sec", 15.0);
    safe_exit_gap_enabled_ = declare_parameter<bool>("safe_exit_gap_enabled", true);
    safe_exit_gap_speed_mps_ = declare_parameter<double>("safe_exit_gap_speed_mps", 0.05);
    safe_exit_gap_yaw_kp_ = declare_parameter<double>("safe_exit_gap_yaw_kp", 0.6);
    safe_exit_gap_max_angular_z_ = declare_parameter<double>("safe_exit_gap_max_angular_z", 0.15);
    safe_exit_gap_extra_clearance_m_ = declare_parameter<double>("safe_exit_gap_extra_clearance_m", 0.30);
    safe_exit_gap_timeout_sec_ = declare_parameter<double>("safe_exit_gap_timeout_sec", 30.0);
    stop_auto_charge_depart_distance_m_ =
      declare_parameter<double>("stop_auto_charge_depart_distance_m", 0.5);
    stop_auto_charge_depart_speed_mps_ =
      declare_parameter<double>("stop_auto_charge_depart_speed_mps", 0.05);
    stop_auto_charge_depart_yaw_kp_ =
      declare_parameter<double>("stop_auto_charge_depart_yaw_kp", 0.6);
    stop_auto_charge_depart_max_angular_z_ =
      declare_parameter<double>("stop_auto_charge_depart_max_angular_z", 0.15);
    stop_auto_charge_depart_timeout_sec_ =
      declare_parameter<double>("stop_auto_charge_depart_timeout_sec", 20.0);
    stop_auto_charge_depart_stop_before_sec_ =
      declare_parameter<double>("stop_auto_charge_depart_stop_before_sec", 0.5);
    stop_auto_charge_depart_stop_after_sec_ =
      declare_parameter<double>("stop_auto_charge_depart_stop_after_sec", 0.5);
    recognizer_trigger_service_ =
      declare_parameter<std::string>("recognizer_trigger_service", "/inventory/trigger_recognition");

    target_list_param_ = declare_parameter<std::vector<std::string>>("target_list", std::vector<std::string>{});
    route_waypoints_file_ =
      declare_parameter<std::string>("route_waypoints_file", "config/route_waypoints.yaml");
    warehouse_layout_file_ =
      declare_parameter<std::string>("warehouse_layout_file", "config/warehouse_layout.yaml");
    gap_scan_map_file_ =
      declare_parameter<std::string>("gap_scan_map_file", "config/gap_scan_map.yaml");
    route_search_failure_policy_ =
      declare_parameter<std::string>("route_search_failure_policy", "error");

    follow_distance_ = declare_parameter<double>("follow_distance", 0.50);
    warehouse_length_ = declare_parameter<double>("warehouse_length", 12.0);
    corridor_speed_ = declare_parameter<double>("corridor_speed", 0.20);
    tracking_speed_ = declare_parameter<double>("tracking_speed", 0.15);
    entry_distance_ = declare_parameter<double>("entry_distance", 0.70);
    turn_speed_ = declare_parameter<double>("turn_speed", 0.40);
    enable_grid_center_entry_ = declare_parameter<bool>("enable_grid_center_entry", true);
    grid_depth_m_ = declare_parameter<double>("grid_depth_m", 2.4);
    left_max_depth_index_ = declare_parameter<int>("left_max_depth_index", 4);
    right_max_depth_index_ = declare_parameter<int>("right_max_depth_index", 3);
    entry_center_offset_m_ = declare_parameter<double>("entry_center_offset_m", 0.0);
    entry_right_target_yaw_rad_ = declare_parameter<double>("entry_right_target_yaw_rad", 1.5708);
    entry_left_target_yaw_rad_ = declare_parameter<double>("entry_left_target_yaw_rad", -1.5708);
    entry_align_yaw_tolerance_rad_ =
      declare_parameter<double>("entry_align_yaw_tolerance_rad", 0.08);
    entry_turn_yaw_stable_required_count_ =
      declare_parameter<int>("entry_turn_yaw_stable_required_count", 3);
    entry_turn_angular_speed_ = declare_parameter<double>("entry_turn_angular_speed", 0.30);
    entry_turn_timeout_sec_ = declare_parameter<double>("entry_turn_timeout_sec", 12.0);
    entry_turn_timeout_sec_ = std::isfinite(entry_turn_timeout_sec_) ?
      std::max(0.1, entry_turn_timeout_sec_) : 12.0;
    entry_straight_speed_ = declare_parameter<double>("entry_straight_speed", 0.08);
    entry_straight_timeout_sec_ = declare_parameter<double>("entry_straight_timeout_sec", 60.0);
    entry_straight_timeout_sec_ = std::isfinite(entry_straight_timeout_sec_) ?
      std::max(0.1, entry_straight_timeout_sec_) : 60.0;
    entry_straight_yaw_kp_ = declare_parameter<double>("entry_straight_yaw_kp", 0.8);
    entry_straight_yaw_deadband_rad_ =
      declare_parameter<double>("entry_straight_yaw_deadband_rad", 0.03);
    entry_straight_max_angular_speed_ =
      declare_parameter<double>("entry_straight_max_angular_speed", 0.10);
    entry_side_hold_target_distance_m_ =
      declare_parameter<double>("entry_side_hold_target_distance_m", 0.60);
    entry_side_hold_kp_ = declare_parameter<double>("entry_side_hold_kp", 0.35);
    entry_side_hold_min_valid_points_ =
      declare_parameter<int>("entry_side_hold_min_valid_points", 5);
    max_dynamic_entry_distance_m_ = declare_parameter<double>("max_dynamic_entry_distance_m", 10.0);
    enter_stop_distance_ = declare_parameter<double>("enter_stop_distance", 0.30);
    enter_slow_distance_ = declare_parameter<double>("enter_slow_distance", 0.55);
    enter_front_sector_start_deg_ = declare_parameter<double>("enter_front_sector_start_deg", -20.0);
    enter_front_sector_end_deg_ = declare_parameter<double>("enter_front_sector_end_deg", 20.0);
    enter_front_left_sector_start_deg_ = declare_parameter<double>(
      "enter_front_left_sector_start_deg", 20.0);
    enter_front_left_sector_end_deg_ = declare_parameter<double>(
      "enter_front_left_sector_end_deg", 75.0);
    enter_left_side_sector_start_deg_ = declare_parameter<double>(
      "enter_left_side_sector_start_deg", 75.0);
    enter_left_side_sector_end_deg_ = declare_parameter<double>(
      "enter_left_side_sector_end_deg", 120.0);
    enter_front_right_sector_start_deg_ = declare_parameter<double>(
      "enter_front_right_sector_start_deg", -75.0);
    enter_front_right_sector_end_deg_ = declare_parameter<double>(
      "enter_front_right_sector_end_deg", -20.0);
    enter_right_side_sector_start_deg_ = declare_parameter<double>(
      "enter_right_side_sector_start_deg", -120.0);
    enter_right_side_sector_end_deg_ = declare_parameter<double>(
      "enter_right_side_sector_end_deg", -75.0);

    tracking_kp_distance_ = declare_parameter<double>("tracking_kp_distance", 0.80);
    tracking_kp_heading_ = declare_parameter<double>("tracking_kp_heading", 1.20);
    tracking_offset_scale_px_ = declare_parameter<double>("tracking_offset_scale_px", 320.0);
    tracking_max_angular_ = declare_parameter<double>("tracking_max_angular", 0.80);

    distance_tolerance_ = declare_parameter<double>("distance_tolerance", 0.08);
    target_distance_aligned_min_m_ =
      declare_parameter<double>("target_distance_aligned_min_m", 0.25);
    target_distance_aligned_max_m_ =
      declare_parameter<double>("target_distance_aligned_max_m", 0.75);
    target_distance_gap_threshold_m_ =
      declare_parameter<double>("target_distance_gap_threshold_m", 1.50);
    const auto target_distance_gap_confirm_count_param =
      declare_parameter<int>("target_distance_gap_confirm_count", 1);
    target_distance_gap_confirm_count_ =
      std::max(1, static_cast<int>(target_distance_gap_confirm_count_param));
    distance_stable_time_sec_ = declare_parameter<double>("distance_stable_time_sec", 1.20);
    recognition_timeout_sec_ = declare_parameter<double>("recognition_timeout_sec", 1.00);
    target_recognition_stable_frames_ =
      declare_parameter<int>("target_recognition_stable_frames", 3);
    target_recognition_stable_time_sec_ =
      declare_parameter<double>("target_recognition_stable_time_sec", 0.30);

    post_track_retreat_distance_ = declare_parameter<double>("post_track_retreat_distance", 0.10);
    retreat_speed_ = declare_parameter<double>("retreat_speed", 0.08);
    wait_gap_stop_settle_sec_ = declare_parameter<double>("wait_gap_stop_settle_sec", 0.25);
    search_gap_speed_ = declare_parameter<double>("search_gap_speed", 0.06);
    search_gap_forward_timeout_sec_ =
      declare_parameter<double>("search_gap_forward_timeout_sec", 4.0);
    search_gap_backward_timeout_sec_ =
      declare_parameter<double>("search_gap_backward_timeout_sec", 6.0);
    gap_detect_cycle_timeout_sec_ = declare_parameter<double>("gap_detect_cycle_timeout_sec", 2.5);
    gap_failures_before_adjust_ = declare_parameter<int>("gap_failures_before_adjust", 3);
    gap_adjust_speed_ = declare_parameter<double>("gap_adjust_speed", 0.08);
    gap_adjust_sequence_ = declare_parameter<std::vector<double>>(
      "gap_adjust_sequence",
      std::vector<double>{-0.10, 0.10, -0.20});

    finish_return_mode_ = normalize_finish_return_mode(
      declare_parameter<std::string>("finish_return_mode", "auto_charge"));
    between_side_auto_charge_wait_sec_ =
      declare_parameter<double>("between_side_auto_charge_wait_sec", 10.0);
    between_side_auto_charge_runtime_sec_ =
      declare_parameter<double>("between_side_auto_charge_runtime_sec", 120.0);
    home_pose_frame_id_ = declare_parameter<std::string>("home_pose_frame_id", "map");
    home_pose_x_ = declare_parameter<double>("home_pose_x", 0.0);
    home_pose_y_ = declare_parameter<double>("home_pose_y", 0.0);
    home_pose_yaw_ = declare_parameter<double>("home_pose_yaw", 0.0);
    nav2_action_name_ = declare_parameter<std::string>("nav2_action_name", "navigate_to_pose");
    nav2_goal_frame_ = declare_parameter<std::string>("nav2_goal_frame", "map");
    nav2_server_wait_timeout_sec_ = declare_parameter<double>("nav2_server_wait_timeout_sec", 1.0);
    nav2_startup_wait_enabled_ = declare_parameter<bool>("nav2_startup_wait_enabled", true);
    nav2_startup_wait_timeout_sec_ =
      declare_parameter<double>("nav2_startup_wait_timeout_sec", 60.0);
    nav2_startup_wait_poll_sec_ =
      declare_parameter<double>("nav2_startup_wait_poll_sec", 0.2);
    nav2_goal_timeout_sec_ = declare_parameter<double>("nav2_goal_timeout_sec", 120.0);
    nav2_route_waypoint_timeout_sec_ =
      declare_parameter<double>("nav2_route_waypoint_timeout_sec", 60.0);
    nav2_cancel_stop_duration_sec_ =
      declare_parameter<double>("nav2_cancel_stop_duration_sec", 0.50);

    continue_on_error_ = declare_parameter<bool>("continue_on_error", false);
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 10.0);

    use_scan_safety_ = declare_parameter<bool>("use_scan_safety", true);
    max_scan_age_sec_ = declare_parameter<double>("max_scan_age_sec", 0.8);

    use_ultrasonic_safety_ = declare_parameter<bool>("use_ultrasonic_safety", true);
    ultrasonic_topics_ = declare_parameter<std::vector<std::string>>(
      "ultrasonic_topics",
      std::vector<std::string>{
        "/ultrasonic_data_A",
        "/ultrasonic_data_B",
        "/ultrasonic_data_C",
        "/ultrasonic_data_D",
        "/ultrasonic_data_E",
        "/ultrasonic_data_F"});
    entry_ultrasonic_stop_distance_ =
      declare_parameter<double>("entry_ultrasonic_stop_distance", 0.25);
    max_ultrasonic_age_sec_ = declare_parameter<double>("max_ultrasonic_age_sec", 0.8);
    web_client_mode_ = declare_parameter<std::string>("web_client_mode", "local");
    open_gap_wait_sec_ = declare_parameter<double>("open_gap_wait_sec", 5.0);
    close_gap_wait_sec_ = declare_parameter<double>("close_gap_wait_sec", 3.0);
    plc_http_enabled_ = declare_parameter<bool>("plc_http_enabled", false);
    plc_server_url_ =
      declare_parameter<std::string>("plc_server_url", "https://58.154.205.27:8099");
    plc_open_endpoint_ =
      declare_parameter<std::string>("plc_open_endpoint", "/http-control-plc/car_open");
    plc_open_endpoint_ =
      declare_parameter<std::string>("plc_open_path", plc_open_endpoint_);
    plc_open_query_param_ = declare_parameter<std::string>("plc_open_query_param", "shelfId");
    plc_close_endpoint_ = declare_parameter<std::string>("plc_close_endpoint", "/close");
    plc_stop_endpoint_ = declare_parameter<std::string>("plc_stop_endpoint", "/stop");
    plc_hello_endpoint_ = declare_parameter<std::string>("plc_hello_endpoint", "/hello");
    plc_verify_tls_ = declare_parameter<bool>("plc_verify_tls", false);
    plc_require_body_success_ = declare_parameter<bool>("plc_require_body_success", false);
    plc_request_timeout_sec_ = declare_parameter<double>("plc_request_timeout_sec", 3.0);
    plc_retry_count_ = declare_parameter<int>("plc_retry_count", 1);
    plc_fail_policy_ = declare_parameter<std::string>("plc_fail_policy", "error");
    const auto plc_supported_cabinets_param =
      declare_parameter<std::vector<int64_t>>(
      "plc_supported_cabinets",
      std::vector<int64_t>{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18});
    plc_supported_cabinets_.assign(
      plc_supported_cabinets_param.begin(),
      plc_supported_cabinets_param.end());
    plc_open_wait_sec_ = declare_parameter<double>("plc_open_wait_sec", 5.0);
    plc_call_close_on_mission_done_ =
      declare_parameter<bool>("plc_call_close_on_mission_done", false);
    plc_call_stop_on_error_ = declare_parameter<bool>("plc_call_stop_on_error", false);
    rfid_upload_enabled_ = declare_parameter<bool>("rfid_upload_enabled", true);
    rfid_upload_url_ =
      declare_parameter<std::string>(
      "rfid_upload_url",
      "https://58.154.205.27:8099/RobotInspection/inventoryAudit");
    rfid_upload_verify_tls_ = declare_parameter<bool>("rfid_upload_verify_tls", false);
    rfid_upload_timeout_sec_ = declare_parameter<double>("rfid_upload_timeout_sec", 3.0);
    rfid_upload_retry_count_ = declare_parameter<int>("rfid_upload_retry_count", 2);
    rfid_upload_fail_policy_ =
      declare_parameter<std::string>("rfid_upload_fail_policy", "error");
    rfid_local_log_enabled_ = declare_parameter<bool>("rfid_local_log_enabled", true);
    rfid_local_log_path_ =
      declare_parameter<std::string>(
      "rfid_local_log_path",
      "/home/wheeltec/wheeltec_ros2/rfid_scan_logs/rfid_scan_records.jsonl");
    rfid_local_log_write_batch_summary_ =
      declare_parameter<bool>("rfid_local_log_write_batch_summary", true);
    rfid_upload_status_path_ =
      declare_parameter<std::string>(
      "rfid_upload_status_path",
      "/tmp/agv_inventory_system/rfid_upload_status.json");
    rfid_upload_require_success_ =
      declare_parameter<bool>("rfid_upload_require_success", false);
    rfid_reader_enabled_ = declare_parameter<bool>("rfid_reader_enabled", true);
    rfid_reader_mode_ =
      declare_parameter<std::string>("rfid_reader_mode", "active_report_serial");
    rfid_serial_device_ = declare_parameter<std::string>("rfid_serial_device", "/dev/ttyUSB0");
    rfid_serial_baud_ = declare_parameter<int>("rfid_serial_baud", 9600);
    rfid_scan_duration_sec_ = declare_parameter<double>("rfid_scan_duration_sec", 5.0);
    rfid_frame_min_length_ = declare_parameter<int>("rfid_frame_min_length", 8);
    rfid_frame_max_length_ = declare_parameter<int>("rfid_frame_max_length", 64);
    scanner_enabled_ = declare_parameter<bool>("scanner_enabled", true);
    scan_duration_sec_ = declare_parameter<double>("scan_duration_sec", 2.0);
    scan_timeout_sec_ = declare_parameter<double>("scan_timeout_sec", 5.0);
    scan_retry_count_ = declare_parameter<int>("scan_retry_count", 0);
    scan_result_timeout_sec_ = declare_parameter<double>("scan_result_timeout_sec", 2.0);
    lift_enabled_ = declare_parameter<bool>("lift_enabled", true);
    lift_up_duration_sec_ = declare_parameter<double>("lift_up_duration_sec", 2.0);
    lift_down_duration_sec_ = declare_parameter<double>("lift_down_duration_sec", 2.0);
    lift_service_timeout_sec_ = declare_parameter<double>("lift_service_timeout_sec", 5.0);
    lift_up_service_name_ = declare_parameter<std::string>("lift_up_service_name", "/lift/up");
    lift_down_service_name_ = declare_parameter<std::string>("lift_down_service_name", "/lift/down");
    lift_stop_service_name_ = declare_parameter<std::string>("lift_stop_service_name", "/lift/stop");
    lift_all_off_service_name_ = declare_parameter<std::string>("lift_all_off_service_name", "/lift/all_off");
    lift_home_service_name_ = declare_parameter<std::string>("lift_home_service_name", "/lift/home");
    grid_motion_duration_sec_ = declare_parameter<double>("grid_motion_duration_sec", 1.0);
    grid_motion_timeout_sec_ = declare_parameter<double>("grid_motion_timeout_sec", 10.0);
    single_cabinet_motion_enabled_ = declare_parameter<bool>("real_motion_enabled", false);
    single_cabinet_motion_target_cabinet_ =
      declare_parameter<int>("real_motion_target_cabinet", 3);
    single_cabinet_motion_target_gap_ =
      declare_parameter<std::string>("real_motion_target_gap", "gap_03_02");
    single_cabinet_motion_stop_after_scan_ =
      declare_parameter<bool>("real_motion_stop_after_scan", true);
    single_cabinet_final_recognition_wait_sec_ =
      declare_parameter<double>("real_motion_final_recognition_wait_sec", 8.0);
    single_cabinet_side_row_enabled_ = declare_parameter<bool>("side_row_inventory_enabled", false);
    single_cabinet_side_row_name_ =
      declare_parameter<std::string>("side_row_inventory_name", "row_01_02_03_04");
    single_cabinet_side_row_first_gap_ =
      declare_parameter<std::string>("side_row_first_gap", "gap_02_03_04");
    const auto first_gap_scan_sequence_param =
      declare_parameter<std::vector<int64_t>>(
      "side_row_first_gap_scan_sequence",
      std::vector<int64_t>{4, 3});
    single_cabinet_side_row_first_gap_scan_sequence_.assign(
      first_gap_scan_sequence_param.begin(),
      first_gap_scan_sequence_param.end());
    single_cabinet_side_row_second_gap_ =
      declare_parameter<std::string>("side_row_second_gap", "gap_01_02_03");
    const auto second_gap_scan_sequence_param =
      declare_parameter<std::vector<int64_t>>(
      "side_row_second_gap_scan_sequence",
      std::vector<int64_t>{2, 1});
    single_cabinet_side_row_second_gap_scan_sequence_.assign(
      second_gap_scan_sequence_param.begin(),
      second_gap_scan_sequence_param.end());
    single_cabinet_side_row_corridor_transfer_enabled_ =
      declare_parameter<bool>("side_row_corridor_transfer_enabled", true);
    single_cabinet_side_row_corridor_transfer_target_cabinet_ =
      declare_parameter<int>("side_row_corridor_transfer_target_cabinet", 2);
    single_cabinet_side_row_corridor_transfer_direction_ =
      declare_parameter<std::string>(
      "side_row_corridor_transfer_direction",
      "toward_cabinet_1");
    single_cabinet_exit_after_each_scan_ = declare_parameter<bool>("exit_gap_after_each_scan", true);
    single_cabinet_exit_mode_ = declare_parameter<std::string>("exit_gap_mode", "reverse");
    single_cabinet_exit_speed_ = declare_parameter<double>("exit_gap_speed", 0.05);
    single_cabinet_exit_extra_distance_m_ =
      declare_parameter<double>("exit_gap_extra_distance_m", 0.10);
    single_cabinet_exit_timeout_sec_ = declare_parameter<double>("exit_gap_timeout_sec", 40.0);
    single_cabinet_exit_distance_m_ = declare_parameter<double>("exit_gap_distance_m", 1.20);
    single_cabinet_exit_turn_enabled_ = declare_parameter<bool>("exit_gap_turn_enabled", true);
    single_cabinet_exit_turn_angular_speed_ =
      declare_parameter<double>("exit_gap_turn_angular_speed", 0.25);
    single_cabinet_exit_turn_yaw_tolerance_rad_ =
      declare_parameter<double>("exit_gap_turn_yaw_tolerance_rad", 0.08);
    single_cabinet_exit_turn_timeout_sec_ =
      declare_parameter<double>("exit_gap_turn_timeout_sec", 8.0);
    single_cabinet_reentry_for_position_adjustment_ =
      declare_parameter<bool>("reentry_for_position_adjustment", true);
    single_cabinet_grid_motion_enabled_ =
      declare_parameter<bool>("grid_motion_enabled", false);
    single_cabinet_grid_spacing_m_ =
      declare_parameter<double>("grid_spacing_m", 0.30);
    single_cabinet_grid_move_speed_ =
      declare_parameter<double>("grid_move_speed", 0.04);
    single_cabinet_grid_move_timeout_sec_ =
      declare_parameter<double>("grid_move_timeout_sec", 10.0);
    single_cabinet_grid_move_return_between_layers_ =
      declare_parameter<bool>("grid_move_return_between_layers", false);
    single_cabinet_close_gap_after_final_exit_ =
      declare_parameter<bool>("close_gap_after_final_exit", true);
    full_inventory_enabled_ = declare_parameter<bool>("full_inventory_enabled", true);
    const auto full_inventory_sequence_param =
      declare_parameter<std::vector<int64_t>>(
      "inventory_plan",
      std::vector<int64_t>{4, 3, 8, 7});
    full_inventory_sequence_.assign(
      full_inventory_sequence_param.begin(),
      full_inventory_sequence_param.end());
    full_inventory_left_route_ =
      declare_parameter<std::string>("inventory_left_route", "left_route");
    full_inventory_right_route_ =
      declare_parameter<std::string>("inventory_right_route", "right_route");
    recognize_in_idle_ = declare_parameter<bool>("recognize_in_idle", true);
    full_inventory_recognize_during_nav_ =
      declare_parameter<bool>("recognize_during_nav", false);
    full_inventory_same_side_next_search_enabled_ =
      declare_parameter<bool>("same_side_next_search_enabled", true);
    full_inventory_same_side_search_speed_ =
      declare_parameter<double>("same_side_search_speed", 0.04);
    full_inventory_same_side_search_timeout_sec_ =
      declare_parameter<double>("same_side_search_timeout_sec", 20.0);
    full_inventory_same_side_pose_hold_enabled_ =
      declare_parameter<bool>("same_side_pose_hold_enabled", true);
    full_inventory_same_side_left_fixed_y_m_ =
      declare_parameter<double>("same_side_left_fixed_y_m", 0.575);
    full_inventory_same_side_left_fixed_yaw_rad_ =
      declare_parameter<double>("same_side_left_fixed_yaw_rad", -3.1400);
    full_inventory_same_side_right_fixed_y_m_ =
      declare_parameter<double>("same_side_right_fixed_y_m", -0.625);
    full_inventory_same_side_right_fixed_yaw_rad_ =
      declare_parameter<double>("same_side_right_fixed_yaw_rad", -3.1400);
    full_inventory_same_side_yaw_kp_ =
      declare_parameter<double>("same_side_yaw_kp", 0.40);
    full_inventory_same_side_yaw_deadband_rad_ =
      declare_parameter<double>("same_side_yaw_deadband_rad", 0.03);
    full_inventory_same_side_y_kp_ =
      declare_parameter<double>("same_side_y_kp", 0.30);
    full_inventory_same_side_y_deadband_m_ =
      declare_parameter<double>("same_side_y_deadband_m", 0.03);
    full_inventory_same_side_y_correction_sign_ =
      declare_parameter<double>("same_side_y_correction_sign", 1.0);
    full_inventory_same_side_max_angular_ =
      declare_parameter<double>("same_side_max_angular", 0.15);
    full_inventory_same_side_recognition_delay_enabled_ =
      declare_parameter<bool>("full_inventory_same_side_recognition_delay_enabled", true);
    full_inventory_same_side_recognition_delay_distance_m_ =
      declare_parameter<double>("full_inventory_same_side_recognition_delay_distance_m", 1.0);
    rear_target_handling_enabled_ =
      declare_parameter<bool>("rear_target_handling_enabled", true);
    rear_target_handle_mode_ =
      declare_parameter<std::string>("rear_target_handle_mode", "hold_entry_yaw_backup");
    rear_target_turn_yaw_tolerance_rad_ =
      declare_parameter<double>("rear_target_turn_yaw_tolerance_rad", 0.08);
    rear_target_turn_timeout_sec_ =
      declare_parameter<double>("rear_target_turn_timeout_sec", 10.0);
    rear_target_backup_enabled_ =
      declare_parameter<bool>("rear_target_backup_enabled", true);
    rear_target_backup_distance_m_ =
      declare_parameter<double>("rear_target_backup_distance_m", 1.50);
    rear_target_backup_speed_ =
      declare_parameter<double>("rear_target_backup_speed", 0.08);
    rear_target_backup_timeout_sec_ =
      declare_parameter<double>("rear_target_backup_timeout_sec", 30.0);
    full_inventory_final_recognition_wait_sec_ =
      declare_parameter<double>("overall_final_recognition_wait_sec", 5.0);
    full_inventory_recognition_fallback_enabled_ =
      declare_parameter<bool>("recognition_fallback_enabled", true);
    full_inventory_recognition_fallback_speed_ =
      declare_parameter<double>("recognition_fallback_speed", 0.04);
    full_inventory_recognition_fallback_wait_sec_ =
      declare_parameter<double>("recognition_fallback_wait_sec", 2.0);
    full_inventory_recognition_fallback_timeout_sec_ =
      declare_parameter<double>("recognition_fallback_timeout_sec", 20.0);
    full_inventory_recognition_fallback_sequence_ =
      declare_parameter<std::vector<double>>(
      "recognition_fallback_sequence_m",
      std::vector<double>{-0.30, 0.60, -0.30});
    post_gap_detect_advance_enabled_ =
      declare_parameter<bool>("post_gap_detect_advance_enabled", true);
    post_gap_detect_advance_distance_m_ =
      declare_parameter<double>("post_gap_detect_advance_distance_m", 0.25);
    post_gap_detect_advance_speed_ =
      declare_parameter<double>("post_gap_detect_advance_speed", 0.04);
    post_gap_detect_advance_timeout_sec_ =
      declare_parameter<double>("post_gap_detect_advance_timeout_sec", 8.0);
    scan_layers_ = declare_parameter<int>("scan_layers", 2);
    scan_depth_count_ = declare_parameter<int>("scan_depth_count", 3);
    web_base_url_ = declare_parameter<std::string>("web_base_url", "");
    web_open_gap_endpoint_ =
      declare_parameter<std::string>("web_open_gap_endpoint", "/api/gap/open");
    web_close_gap_endpoint_ =
      declare_parameter<std::string>("web_close_gap_endpoint", "/api/gap/close");
    web_status_endpoint_ =
      declare_parameter<std::string>("web_status_endpoint", "/api/robot/status");
    web_result_endpoint_ =
      declare_parameter<std::string>("web_result_endpoint", "/api/inventory/result");

    recognized_sub_ = create_subscription<agv_inventory_system::msg::RecognizedNumber>(
      recognized_topic_,
      10,
      [this](const agv_inventory_system::msg::RecognizedNumber::SharedPtr msg) {
        recognized_callback(msg);
      });

    distance_sub_ = create_subscription<std_msgs::msg::Float32>(
      distance_topic_,
      10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        latest_distance_ = static_cast<double>(msg->data);
        has_distance_ = std::isfinite(latest_distance_) && latest_distance_ > 0.0;
      });

    gap_sub_ = create_subscription<agv_inventory_system::msg::GapStatus>(
      gap_topic_,
      10,
      [this](const agv_inventory_system::msg::GapStatus::SharedPtr msg) {
        latest_gap_ = *msg;
      });

    auto_recharge_status_sub_ = create_subscription<std_msgs::msg::String>(
      auto_recharge_status_topic_,
      10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        latest_auto_recharge_status_ = agv_inventory_system::trim(msg->data);
        latest_auto_recharge_status_time_ = this->now();
      });

    auto_recharge_charging_flag_sub_ = create_subscription<std_msgs::msg::Bool>(
      auto_recharge_charging_flag_topic_,
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        latest_auto_recharge_charging_ = msg->data;
        latest_auto_recharge_charging_time_ = this->now();
      });

    auto_recharge_recharge_flag_sub_ = create_subscription<std_msgs::msg::Int8>(
      auto_recharge_recharge_flag_topic_,
      10,
      [this](const std_msgs::msg::Int8::SharedPtr msg) {
        latest_auto_recharge_recharge_flag_ = msg->data;
        latest_auto_recharge_recharge_flag_time_ = this->now();
      });

    odom_sub_primary_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      20,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_callback(msg, true);
      });

    if (!odom_fallback_topic_.empty() && odom_fallback_topic_ != odom_topic_) {
      odom_sub_fallback_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_fallback_topic_,
        20,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          odom_callback(msg, false);
        });
    }

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_scan_ = msg;
        if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
          latest_scan_stamp_ = this->now();
        } else {
          latest_scan_stamp_ = rclcpp::Time(msg->header.stamp);
        }
      });

    ultrasonic_ranges_.assign(ultrasonic_topics_.size(), std::numeric_limits<double>::infinity());
    ultrasonic_stamps_.assign(
      ultrasonic_topics_.size(),
      rclcpp::Time(0, 0, get_clock()->get_clock_type()));

    ultrasonic_subs_.reserve(ultrasonic_topics_.size());
    for (std::size_t i = 0; i < ultrasonic_topics_.size(); ++i) {
      ultrasonic_subs_.push_back(
        create_subscription<sensor_msgs::msg::Range>(
          ultrasonic_topics_[i],
          rclcpp::SensorDataQoS(),
          [this, i](const sensor_msgs::msg::Range::SharedPtr msg) {
            if (std::isfinite(msg->range) && msg->range > 0.0F) {
              ultrasonic_ranges_[i] = static_cast<double>(msg->range);
              if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
                ultrasonic_stamps_[i] = this->now();
              } else {
                ultrasonic_stamps_[i] = rclcpp::Time(msg->header.stamp);
              }
            }
          }));
    }

    mission_state_pub_ = create_publisher<std_msgs::msg::String>(mission_state_topic_, 10);
    mission_log_pub_ = create_publisher<std_msgs::msg::String>(mission_log_topic_, 10);
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    corridor_enable_pub_ = create_publisher<std_msgs::msg::Bool>(corridor_enable_topic_, 10);
    corridor_reverse_pub_ = create_publisher<std_msgs::msg::Bool>(corridor_reverse_topic_, 10);
    gap_context_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(gap_context_topic_, 10);
    // 使能控制使用 transient_local，保证晚启动节点也能拿到当前状态。
    auto control_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    entry_side_pub_ = create_publisher<std_msgs::msg::String>(entry_side_topic_, control_qos);
    target_lidar_side_pub_ =
      create_publisher<std_msgs::msg::String>(target_lidar_side_topic_, control_qos);
    current_target_cabinet_pub_ =
      create_publisher<std_msgs::msg::Int32>(current_target_cabinet_topic_, control_qos);
    recognizer_enable_pub_ =
      create_publisher<std_msgs::msg::Bool>(recognizer_enable_topic_, control_qos);
    gap_detector_enable_pub_ =
      create_publisher<std_msgs::msg::Bool>(gap_detector_enable_topic_, control_qos);
    distance_estimator_enable_pub_ =
      create_publisher<std_msgs::msg::Bool>(distance_estimator_enable_topic_, control_qos);

    start_srv_ = create_service<agv_inventory_system::srv::StartMission>(
      start_service_name_,
      [this](
        const std::shared_ptr<agv_inventory_system::srv::StartMission::Request> request,
        std::shared_ptr<agv_inventory_system::srv::StartMission::Response> response) {
        start_service_callback(request, response);
      });

    cancel_srv_ = create_service<std_srvs::srv::Trigger>(
      cancel_service_name_,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        cancel_service_callback(request, response);
      });

    return_home_srv_ = create_service<std_srvs::srv::Trigger>(
      return_home_service_name_,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        return_home_service_callback(request, response);
      });

    return_to_charge_srv_ = create_service<std_srvs::srv::Trigger>(
      return_to_charge_service_name_,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        return_to_charge_service_callback(request, response);
      });

    cancel_auto_recharge_srv_ = create_service<std_srvs::srv::Trigger>(
      cancel_auto_recharge_service_name_,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        cancel_auto_recharge_service_callback(request, response);
      });

    safe_exit_gap_srv_ = create_service<std_srvs::srv::Trigger>(
      safe_exit_gap_service_name_,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        safe_exit_gap_service_callback(request, response);
      });

    stop_auto_charge_and_depart_srv_ = create_service<std_srvs::srv::Trigger>(
      stop_auto_charge_and_depart_service_name_,
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        stop_auto_charge_and_depart_service_callback(request, response);
      });

    recognizer_trigger_client_ = create_client<std_srvs::srv::SetBool>(
      recognizer_trigger_service_);
    inventory_auto_recharge_start_client_ = create_client<std_srvs::srv::Trigger>(
      inventory_auto_recharge_start_service_name_);
    inventory_auto_recharge_cancel_client_ = create_client<std_srvs::srv::Trigger>(
      inventory_auto_recharge_cancel_service_name_);
    lift_up_client_ = create_client<agv_inventory_system::srv::LiftMoveTimed>(
      lift_up_service_name_);
    lift_down_client_ = create_client<agv_inventory_system::srv::LiftMoveTimed>(
      lift_down_service_name_);
    lift_home_client_ = create_client<agv_inventory_system::srv::LiftMoveTimed>(
      lift_home_service_name_);
    lift_stop_client_ = create_client<std_srvs::srv::Trigger>(lift_stop_service_name_);
    lift_all_off_client_ = create_client<std_srvs::srv::Trigger>(lift_all_off_service_name_);
    nav2_client_ = rclcpp_action::create_client<NavigateToPose>(this, nav2_action_name_);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    std::string route_load_error;
    routes_loaded_ = load_route_config(route_load_error);
    if (!routes_loaded_) {
      RCLCPP_ERROR(get_logger(), "路线配置加载失败: %s", route_load_error.c_str());
    }
    std::string layout_load_error;
    warehouse_layout_loaded_ = load_warehouse_layout_config(layout_load_error);
    if (!warehouse_layout_loaded_) {
      RCLCPP_ERROR(get_logger(), "仓库布局配置加载失败: %s", layout_load_error.c_str());
    }
    std::string runtime_load_error;
    inventory_runtime_config_loaded_ = load_inventory_runtime_config(runtime_load_error);
    if (!inventory_runtime_config_loaded_) {
      RCLCPP_WARN(get_logger(), "盘库运行配置加载失败: %s", runtime_load_error.c_str());
    }

    const double period = 1.0 / std::max(1.0, control_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period)),
      std::bind(&MissionManagerNode::on_timer, this));

    set_state(State::IDLE, "系统待机");
    RCLCPP_INFO(
      get_logger(),
      "任务管理节点已启动，odom主话题=%s，备用odom=%s，finish_return_mode=%s，nav2_action=%s",
      odom_topic_.c_str(),
      odom_fallback_topic_.c_str(),
      finish_return_mode_.c_str(),
      nav2_action_name_.c_str());
  }

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  enum class State
  {
    IDLE,
    REQUEST_OPEN_GAP,
    WAIT_OPEN_READY,
    NAV_ROUTE,
    TARGET_TRACKING,
    SEARCH_GAP,
    WAITING_GAP,
    ENTERING_GAP,
    INVENTORYING,
    REQUEST_CLOSE_GAP,
    WAIT_CLOSE_DONE,
    RETURNING,
    RETURNING_HOME,
    AUTO_RECHARGING,
    SAFE_EXIT_GAP,
    STOP_AUTO_CHARGE_AND_DEPART,
    DONE,
    ERROR,
    SINGLE_CABINET_PREPARE_NAV,
    SINGLE_CABINET_NAV_TO_TARGET,
    SINGLE_CABINET_TARGET_TRACKING,
    SINGLE_CABINET_FINAL_RECOGNITION_WAIT,
    SINGLE_CABINET_WAITING_GAP,
    SINGLE_CABINET_ENTERING_GAP,
    SINGLE_CABINET_IN_GAP_SCAN,
    SINGLE_CABINET_STOP_AFTER_SCAN,
    SINGLE_CABINET_EXIT_GAP,
    SINGLE_CABINET_REENTER_FOR_ADJUSTED_SCAN,
    SINGLE_CABINET_ADJUSTED_SIDE_SCAN,
    SINGLE_CABINET_CORRIDOR_TRANSFER,
    SINGLE_CABINET_PREPARE_NEXT_GAP,
    SINGLE_CABINET_REENTER_NEXT_GAP,
    SINGLE_CABINET_NEXT_GAP_SCAN,
    SINGLE_CABINET_FINAL_EXIT_GAP,
    FULL_INVENTORY_PREPARE_TARGET,
    FULL_INVENTORY_NAV_TO_OBSERVE,
    FULL_INVENTORY_POST_ROUTE_RECOGNITION_WAIT,
    FULL_INVENTORY_RECOGNITION_FALLBACK,
    FULL_INVENTORY_TARGET_DISTANCE_ALIGN,
    FULL_INVENTORY_SEARCH_GAP,
    FULL_INVENTORY_POST_GAP_DETECT_ADVANCE,
    FULL_INVENTORY_ENTERING_GAP,
    FULL_INVENTORY_IN_GAP_SCAN,
    FULL_INVENTORY_EXIT_GAP,
    FULL_INVENTORY_ADVANCE_NEXT_TARGET,
    FULL_INVENTORY_REAR_TARGET_REORIENT,
    FULL_INVENTORY_REAR_TARGET_BACKUP,
    FULL_INVENTORY_SAME_SIDE_NEXT_SEARCH,
    FULL_INVENTORY_AUTO_CHARGE_BETWEEN_SIDES,
    FULL_INVENTORY_COMPLETE,
  };

  enum class FullInventoryRecognitionFallbackPhase
  {
    IDLE,
    MOVING,
    WAITING,
  };

  enum class ReturnMode
  {
    NONE,
    SEARCH_TARGET,
    CANCEL_HOME,
    FINISH_HOME,
  };

  enum class PendingInterruptRequest
  {
    NONE,
    RETURN_HOME,
    AUTO_RECHARGE,
  };

  enum class PlcOpenContinuation
  {
    NONE,
    SINGLE_CABINET_PREPARE_NAV,
    FULL_INVENTORY_START_ROUTE,
    FULL_INVENTORY_SAME_SIDE_NEXT_SEARCH,
    FULL_INVENTORY_REAR_TARGET_REORIENT,
    FULL_INVENTORY_ADVANCE_ROUTE,
    FULL_INVENTORY_BETWEEN_SIDE_ROUTE,
  };

  enum class WaitGapPhase
  {
    IDLE,
    RETREATING,
    STOP_BEFORE_DETECT,
    DETECTING_GAP,
    POSE_ADJUSTING,
  };

  enum class SingleCabinetSideRowPhase
  {
    NONE,
    FIRST_PRIMARY_SCAN,
    FIRST_ADJUSTED_SCAN,
    CORRIDOR_TRANSFER,
    SECOND_PRIMARY_SCAN,
    SECOND_ADJUSTED_SCAN,
    COMPLETE,
  };

  enum class SingleCabinetAfterExitAction
  {
    NONE,
    REENTER_ADJUSTED,
    CORRIDOR_TRANSFER,
    CLOSE_AND_DONE,
    FINAL_CLOSE_AND_DONE,
  };

  enum class SingleCabinetExitPhase
  {
    STRAIGHT_REVERSE,
    TURN_TO_CORRIDOR,
  };

  enum class InGapScanRuntimeMode
  {
    SINGLE_CABINET,
    FULL_INVENTORY,
  };

  enum class EntryGapPhase
  {
    IDLE,
    ENTERING_TURN,
    ENTERING_STRAIGHT_ALIGN,
    MOVING_TO_GRID_CENTER,
  };

  enum class EntryMotionMode
  {
    FORWARD_ENTRY,
    REVERSE_ENTRY,
  };

  enum class StopAutoChargeDepartPhase
  {
    IDLE,
    CANCELING,
    STOP_BEFORE,
    DEPARTING,
    STOP_AFTER,
  };

  enum class StopAutoChargeDepartContinuation
  {
    IDLE,
    FULL_INVENTORY_BETWEEN_SIDES,
  };

  enum class SearchDirection
  {
    FORWARD,
    BACKWARD,
  };

  struct Pose2D
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
    std::string frame_id;
    bool valid{false};
  };

  struct RouteConfig
  {
    std::string name;
    std::string frame_id{"map"};
    std::vector<Pose2D> waypoints;
  };

  struct WarehouseRowLayout
  {
    std::string side{"left"};
    std::vector<std::vector<int>> physical_units;
  };

  struct TargetGapPlan
  {
    bool valid{false};
    std::string side{"left"};
    SearchDirection search_direction{SearchDirection::FORWARD};
    std::vector<int> physical_unit;
    std::vector<int> gap_before_unit;
    std::vector<int> gap_after_unit;
    std::size_t unit_index{0};
    std::size_t cabinet_index_in_unit{0};
  };

  struct TargetMetadata
  {
    int cabinet_id{-1};
    int level_index{1};
    int depth_index{1};
    bool depth_defaulted{false};
  };

  struct InventoryGapPlan
  {
    std::string gap_id;
    std::vector<int> scan_cabinets;
  };

  struct EnteringSafetyEval
  {
    double front_min_dist{std::numeric_limits<double>::infinity()};
    double front_left_min_dist{std::numeric_limits<double>::infinity()};
    double left_side_min_dist{std::numeric_limits<double>::infinity()};
    double front_side_min_dist{std::numeric_limits<double>::infinity()};
    double side_min_dist{std::numeric_limits<double>::infinity()};
    double speed_scale{1.0};
    bool blocked{false};
    std::string active_side{"left"};
    std::string motion_direction{"front"};
    std::string block_reason{"NONE"};
  };

  struct EntrySideHoldEval
  {
    bool active{false};
    std::string status{"NOT_EVALUATED"};
    std::string entry_side{"left"};
    double target_distance{0.60};
    double left_side_dist{std::numeric_limits<double>::infinity()};
    double right_side_dist{std::numeric_limits<double>::infinity()};
    double control_side_dist{std::numeric_limits<double>::infinity()};
    double side_error{0.0};
    double yaw_hold_cmd{0.0};
    double side_distance_cmd{0.0};
    double final_angular_cmd{0.0};
    std::size_t left_valid_points{0};
    std::size_t right_valid_points{0};
  };

  struct EnteringYawControl
  {
    double yaw{0.0};
    std::string yaw_frame{"odom_fallback"};
    std::string pose_note{"not evaluated"};
    bool valid{false};
  };

  static std::string state_to_string(State s)
  {
    switch (s) {
      case State::IDLE:
        return "IDLE";
      case State::REQUEST_OPEN_GAP:
        return "REQUEST_OPEN_GAP";
      case State::WAIT_OPEN_READY:
        return "WAIT_OPEN_READY";
      case State::NAV_ROUTE:
        return "NAV_ROUTE";
      case State::TARGET_TRACKING:
        return "TARGET_TRACKING";
      case State::SEARCH_GAP:
        return "SEARCH_GAP";
      case State::WAITING_GAP:
        return "WAITING_GAP";
      case State::ENTERING_GAP:
        return "ENTERING_GAP";
      case State::INVENTORYING:
        return "INVENTORYING";
      case State::REQUEST_CLOSE_GAP:
        return "REQUEST_CLOSE_GAP";
      case State::WAIT_CLOSE_DONE:
        return "WAIT_CLOSE_DONE";
      case State::RETURNING:
        return "RETURNING";
      case State::RETURNING_HOME:
        return "RETURNING_HOME";
      case State::AUTO_RECHARGING:
        return "AUTO_RECHARGING";
      case State::SAFE_EXIT_GAP:
        return "SAFE_EXIT_GAP";
      case State::STOP_AUTO_CHARGE_AND_DEPART:
        return "STOP_AUTO_CHARGE_AND_DEPART";
      case State::DONE:
        return "DONE";
      case State::ERROR:
        return "ERROR";
      case State::SINGLE_CABINET_PREPARE_NAV:
        return "SINGLE_CABINET_PREPARE_NAV";
      case State::SINGLE_CABINET_NAV_TO_TARGET:
        return "SINGLE_CABINET_NAV_TO_TARGET";
      case State::SINGLE_CABINET_TARGET_TRACKING:
        return "SINGLE_CABINET_TARGET_TRACKING";
      case State::SINGLE_CABINET_FINAL_RECOGNITION_WAIT:
        return "SINGLE_CABINET_FINAL_RECOGNITION_WAIT";
      case State::SINGLE_CABINET_WAITING_GAP:
        return "SINGLE_CABINET_WAITING_GAP";
      case State::SINGLE_CABINET_ENTERING_GAP:
        return "SINGLE_CABINET_ENTERING_GAP";
      case State::SINGLE_CABINET_IN_GAP_SCAN:
        return "SINGLE_CABINET_IN_GAP_SCAN";
      case State::SINGLE_CABINET_STOP_AFTER_SCAN:
        return "SINGLE_CABINET_STOP_AFTER_SCAN";
      case State::SINGLE_CABINET_EXIT_GAP:
        return "SINGLE_CABINET_EXIT_GAP";
      case State::SINGLE_CABINET_REENTER_FOR_ADJUSTED_SCAN:
        return "SINGLE_CABINET_REENTER_FOR_ADJUSTED_SCAN";
      case State::SINGLE_CABINET_ADJUSTED_SIDE_SCAN:
        return "SINGLE_CABINET_ADJUSTED_SIDE_SCAN";
      case State::SINGLE_CABINET_CORRIDOR_TRANSFER:
        return "SINGLE_CABINET_CORRIDOR_TRANSFER";
      case State::SINGLE_CABINET_PREPARE_NEXT_GAP:
        return "SINGLE_CABINET_PREPARE_NEXT_GAP";
      case State::SINGLE_CABINET_REENTER_NEXT_GAP:
        return "SINGLE_CABINET_REENTER_NEXT_GAP";
      case State::SINGLE_CABINET_NEXT_GAP_SCAN:
        return "SINGLE_CABINET_NEXT_GAP_SCAN";
      case State::SINGLE_CABINET_FINAL_EXIT_GAP:
        return "SINGLE_CABINET_FINAL_EXIT_GAP";
      case State::FULL_INVENTORY_PREPARE_TARGET:
        return "FULL_INVENTORY_PREPARE_TARGET";
      case State::FULL_INVENTORY_NAV_TO_OBSERVE:
        return "FULL_INVENTORY_NAV_TO_OBSERVE";
      case State::FULL_INVENTORY_POST_ROUTE_RECOGNITION_WAIT:
        return "FULL_INVENTORY_POST_ROUTE_RECOGNITION_WAIT";
      case State::FULL_INVENTORY_RECOGNITION_FALLBACK:
        return "FULL_INVENTORY_RECOGNITION_FALLBACK";
      case State::FULL_INVENTORY_TARGET_DISTANCE_ALIGN:
        return "FULL_INVENTORY_TARGET_DISTANCE_ALIGN";
      case State::FULL_INVENTORY_SEARCH_GAP:
        return "FULL_INVENTORY_SEARCH_GAP";
      case State::FULL_INVENTORY_POST_GAP_DETECT_ADVANCE:
        return "FULL_INVENTORY_POST_GAP_DETECT_ADVANCE";
      case State::FULL_INVENTORY_ENTERING_GAP:
        return "FULL_INVENTORY_ENTERING_GAP";
      case State::FULL_INVENTORY_IN_GAP_SCAN:
        return "FULL_INVENTORY_IN_GAP_SCAN";
      case State::FULL_INVENTORY_EXIT_GAP:
        return "FULL_INVENTORY_EXIT_GAP";
      case State::FULL_INVENTORY_ADVANCE_NEXT_TARGET:
        return "FULL_INVENTORY_ADVANCE_NEXT_TARGET";
      case State::FULL_INVENTORY_REAR_TARGET_REORIENT:
        return "FULL_INVENTORY_REAR_TARGET_REORIENT";
      case State::FULL_INVENTORY_REAR_TARGET_BACKUP:
        return "FULL_INVENTORY_REAR_TARGET_BACKUP";
      case State::FULL_INVENTORY_SAME_SIDE_NEXT_SEARCH:
        return "FULL_INVENTORY_SAME_SIDE_NEXT_SEARCH";
      case State::FULL_INVENTORY_AUTO_CHARGE_BETWEEN_SIDES:
        return "FULL_INVENTORY_AUTO_CHARGE_BETWEEN_SIDES";
      case State::FULL_INVENTORY_COMPLETE:
        return "FULL_INVENTORY_COMPLETE";
      default:
        return "UNKNOWN";
    }
  }

  static std::string plc_open_continuation_to_string(PlcOpenContinuation continuation)
  {
    switch (continuation) {
      case PlcOpenContinuation::SINGLE_CABINET_PREPARE_NAV:
        return "SINGLE_CABINET_PREPARE_NAV";
      case PlcOpenContinuation::FULL_INVENTORY_START_ROUTE:
        return "FULL_INVENTORY_START_ROUTE";
      case PlcOpenContinuation::FULL_INVENTORY_SAME_SIDE_NEXT_SEARCH:
        return "FULL_INVENTORY_SAME_SIDE_NEXT_SEARCH";
      case PlcOpenContinuation::FULL_INVENTORY_REAR_TARGET_REORIENT:
        return "FULL_INVENTORY_REAR_TARGET_REORIENT";
      case PlcOpenContinuation::FULL_INVENTORY_ADVANCE_ROUTE:
        return "FULL_INVENTORY_ADVANCE_ROUTE";
      case PlcOpenContinuation::FULL_INVENTORY_BETWEEN_SIDE_ROUTE:
        return "FULL_INVENTORY_BETWEEN_SIDE_ROUTE";
      case PlcOpenContinuation::NONE:
      default:
        return "NONE";
    }
  }

  static double normalize_angle(double angle)
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  static double normalize_deg(double angle_deg)
  {
    while (angle_deg > 180.0) {
      angle_deg -= 360.0;
    }
    while (angle_deg < -180.0) {
      angle_deg += 360.0;
    }
    return angle_deg;
  }

  static bool in_deg_range(double value_deg, double start_deg, double end_deg)
  {
    value_deg = normalize_deg(value_deg);
    start_deg = normalize_deg(start_deg);
    end_deg = normalize_deg(end_deg);
    if (start_deg <= end_deg) {
      return value_deg >= start_deg && value_deg <= end_deg;
    }
    return value_deg >= start_deg || value_deg <= end_deg;
  }

  static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  static std::string sanitize_frame_id(const std::string & frame)
  {
    if (frame.empty()) {
      return frame;
    }
    if (frame[0] == '/') {
      return frame.substr(1);
    }
    return frame;
  }

  static geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
  }

  static rclcpp::Time zero_time(rclcpp::Clock::SharedPtr clock)
  {
    return rclcpp::Time(0, 0, clock->get_clock_type());
  }

  static bool try_normalize_entry_side(std::string side, std::string & normalized)
  {
    side = agv_inventory_system::trim(side);
    std::transform(side.begin(), side.end(), side.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    if (side == "left" || side == "l") {
      normalized = "left";
      return true;
    }
    if (side == "right" || side == "r") {
      normalized = "right";
      return true;
    }
    normalized.clear();
    return false;
  }

  static std::string normalize_entry_side(std::string side)
  {
    std::string normalized;
    if (try_normalize_entry_side(side, normalized)) {
      return normalized;
    }
    return "left";
  }

  static std::string normalize_finish_return_mode(std::string mode)
  {
    mode = agv_inventory_system::trim(mode);
    std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    if (mode == "map_origin" || mode == "auto_charge") {
      return mode;
    }
    return "auto_charge";
  }

  static std::string normalize_rear_target_handle_mode(std::string mode)
  {
    mode = agv_inventory_system::trim(mode);
    std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    if (mode == "hold_entry_yaw_backup" || mode == "hold_entry_yaw") {
      return "hold_entry_yaw_backup";
    }
    if (mode == "turn_around") {
      return "hold_entry_yaw_backup";
    }
    return "hold_entry_yaw_backup";
  }

  static std::string normalize_plc_fail_policy(std::string policy)
  {
    policy = agv_inventory_system::trim(policy);
    std::transform(policy.begin(), policy.end(), policy.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    if (policy == "continue_without_plc") {
      return policy;
    }
    return "error";
  }

  static std::string normalize_rfid_upload_fail_policy(std::string policy)
  {
    policy = agv_inventory_system::trim(policy);
    std::transform(policy.begin(), policy.end(), policy.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    if (policy == "continue_without_upload") {
      return policy;
    }
    return "error";
  }

  static std::string make_inventory_location_rfid(int cabinet_id, int layer, int grid)
  {
    std::ostringstream oss;
    oss << "shelf_" << cabinet_id
        << "_" << layer << "_" << grid;
    return oss.str();
  }

  static const char * inventory_scan_output_source_label(
    agv_inventory_system::InventoryScanOutputSource source)
  {
    switch (source) {
      case agv_inventory_system::InventoryScanOutputSource::ACTIVE_REPORT_SERIAL_SUCCESS:
        return "active_report_serial_success";
      case agv_inventory_system::InventoryScanOutputSource::ACTIVE_REPORT_SERIAL_EMPTY:
        return "active_report_serial_empty";
      case agv_inventory_system::InventoryScanOutputSource::ACTIVE_REPORT_SERIAL_FAILED:
        return "active_report_serial_failed";
      default:
        return "active_report_serial";
    }
  }

  static const char * rfid_local_log_source_label(
    agv_inventory_system::InventoryScanOutputSource source)
  {
    switch (source) {
      case agv_inventory_system::InventoryScanOutputSource::ACTIVE_REPORT_SERIAL_SUCCESS:
      case agv_inventory_system::InventoryScanOutputSource::ACTIVE_REPORT_SERIAL_EMPTY:
      case agv_inventory_system::InventoryScanOutputSource::ACTIVE_REPORT_SERIAL_FAILED:
      default:
        return "active_report_serial";
    }
  }

  static bool inventory_scan_output_source_succeeded(
    agv_inventory_system::InventoryScanOutputSource source)
  {
    return source == agv_inventory_system::InventoryScanOutputSource::ACTIVE_REPORT_SERIAL_SUCCESS;
  }

  bool finish_return_mode_is_map_origin() const
  {
    return normalize_finish_return_mode(finish_return_mode_) == "map_origin";
  }

  static std::string search_direction_to_string(SearchDirection direction)
  {
    return direction == SearchDirection::BACKWARD ? "backward" : "forward";
  }

  static EntryMotionMode resolve_entry_motion_mode(SearchDirection direction)
  {
    return direction == SearchDirection::BACKWARD ?
           EntryMotionMode::REVERSE_ENTRY : EntryMotionMode::FORWARD_ENTRY;
  }

  static std::string entry_motion_mode_to_string(EntryMotionMode mode)
  {
    return mode == EntryMotionMode::REVERSE_ENTRY ? "REVERSE_ENTRY" : "FORWARD_ENTRY";
  }

  double apply_entry_motion_direction(double speed_abs) const
  {
    const double speed = std::abs(speed_abs);
    return entry_motion_mode_ == EntryMotionMode::REVERSE_ENTRY ? -speed : speed;
  }

  double apply_exit_motion_direction(double speed_abs) const
  {
    return -apply_entry_motion_direction(speed_abs);
  }

  static std::string exit_motion_label_from_linear_cmd(double linear_cmd)
  {
    if (!std::isfinite(linear_cmd)) {
      return "unknown";
    }
    if (linear_cmd > 1e-4) {
      return "straight_forward";
    }
    if (linear_cmd < -1e-4) {
      return "straight_reverse";
    }
    return "straight_stop";
  }

  static bool try_parse_search_direction(std::string direction, SearchDirection & parsed)
  {
    direction = agv_inventory_system::trim(direction);
    std::transform(direction.begin(), direction.end(), direction.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    if (direction == "forward" || direction == "f") {
      parsed = SearchDirection::FORWARD;
      return true;
    }
    if (direction == "backward" || direction == "back" || direction == "b") {
      parsed = SearchDirection::BACKWARD;
      return true;
    }
    return false;
  }

  static std::string entry_gap_phase_to_string(EntryGapPhase phase)
  {
    switch (phase) {
      case EntryGapPhase::ENTERING_TURN:
        return "ENTERING_TURN";
      case EntryGapPhase::ENTERING_STRAIGHT_ALIGN:
        return "ENTERING_STRAIGHT_ALIGN";
      case EntryGapPhase::MOVING_TO_GRID_CENTER:
        return "MOVING_TO_GRID_CENTER";
      case EntryGapPhase::IDLE:
      default:
        return "IDLE";
    }
  }

  static std::string single_cabinet_exit_phase_to_string(SingleCabinetExitPhase phase)
  {
    switch (phase) {
      case SingleCabinetExitPhase::STRAIGHT_REVERSE:
        return "STRAIGHT_REVERSE";
      case SingleCabinetExitPhase::TURN_TO_CORRIDOR:
        return "TURN_TO_CORRIDOR";
      default:
        return "STRAIGHT_REVERSE";
    }
  }

  static std::string cabinet_unit_to_string(const std::vector<int> & ids)
  {
    std::ostringstream oss;
    oss << '[';
    for (std::size_t i = 0; i < ids.size(); ++i) {
      if (i > 0) {
        oss << ',';
      }
      oss << ids[i];
    }
    oss << ']';
    return oss.str();
  }

  static std::string double_vector_to_string(const std::vector<double> & values)
  {
    std::ostringstream oss;
    oss << '[' << std::fixed << std::setprecision(2);
    for (std::size_t i = 0; i < values.size(); ++i) {
      if (i > 0) {
        oss << ',';
      }
      oss << values[i];
    }
    oss << ']';
    return oss.str();
  }

  static std::string gap_plan_to_string(const TargetGapPlan & plan)
  {
    if (!plan.valid) {
      return "invalid";
    }
    return
      cabinet_unit_to_string(plan.gap_before_unit) +
      " 与 " +
      cabinet_unit_to_string(plan.gap_after_unit) +
      " 之间";
  }

  static std::string format_seconds(double seconds)
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2) << seconds;
    return oss.str();
  }

  static std::string format_fixed(double value, int precision)
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(std::max(0, precision)) << value;
    return oss.str();
  }

  void publish_inventory_flow_log(const std::string & text)
  {
    publish_log("[mission_manager][inventory_flow] " + text);
  }

  void publish_single_cabinet_log(const std::string & text)
  {
    publish_log("[mission_manager][single_cabinet] " + text);
  }

  void publish_full_inventory_log(const std::string & text)
  {
    publish_log("[mission_manager][FULL_INVENTORY] " + text);
  }

  void publish_plc_log(const std::string & text)
  {
    publish_log("[mission_manager][PLC] " + text);
  }

  void publish_motion_log(const std::string & text)
  {
    if (full_inventory_active_) {
      publish_full_inventory_log(text);
    } else {
      publish_single_cabinet_log(text);
    }
  }

  static const char * in_gap_scan_mode_label(InGapScanRuntimeMode mode)
  {
    return mode == InGapScanRuntimeMode::FULL_INVENTORY ? "FULL_INVENTORY" : "single_cabinet";
  }

  static const char * rfid_local_log_mission_mode_label(InGapScanRuntimeMode mode)
  {
    return mode == InGapScanRuntimeMode::FULL_INVENTORY ? "FULL_INVENTORY" : "SINGLE_CABINET";
  }

  void publish_in_gap_scan_log(InGapScanRuntimeMode mode, const std::string & text)
  {
    if (mode == InGapScanRuntimeMode::FULL_INVENTORY) {
      publish_full_inventory_log(text);
    } else {
      publish_single_cabinet_log(text);
    }
  }

  void clear_inventory_upload_batch(const std::string & reason)
  {
    if (inventory_upload_batch_.empty()) {
      inventory_upload_batch_finalized_ = false;
      return;
    }
    RCLCPP_WARN(
      get_logger(),
      "[mission_manager][RFID][batch] clear reason=%s item_count=%zu",
      reason.c_str(),
      inventory_upload_batch_.size());
    inventory_upload_batch_.clear();
    inventory_upload_batch_locations_.clear();
    inventory_upload_batch_finalized_ = false;
  }

  void fail_in_gap_scan_runtime(InGapScanRuntimeMode mode, const std::string & reason)
  {
    clear_inventory_upload_batch("error");
    if (mode == InGapScanRuntimeMode::FULL_INVENTORY) {
      fail_full_inventory(reason);
    } else {
      fail_single_cabinet_motion(reason);
    }
  }

  std::string single_cabinet_reject_message() const
  {
    return
      "single cabinet inventory only supports " + single_cabinet_motion_target_gap_ +
      " with scan_cabinets=[" + std::to_string(single_cabinet_motion_target_cabinet_) + "]";
  }

  bool is_nav_route_like_state(State s) const
  {
    return
      s == State::NAV_ROUTE ||
      s == State::SINGLE_CABINET_NAV_TO_TARGET ||
      s == State::FULL_INVENTORY_NAV_TO_OBSERVE;
  }

  bool is_target_tracking_like_state(State s) const
  {
    return
      s == State::TARGET_TRACKING ||
      s == State::SINGLE_CABINET_TARGET_TRACKING ||
      s == State::FULL_INVENTORY_TARGET_DISTANCE_ALIGN;
  }

  bool is_single_cabinet_recognition_state(State s) const
  {
    return
      single_cabinet_motion_active_ &&
      (s == State::SINGLE_CABINET_NAV_TO_TARGET ||
      s == State::SINGLE_CABINET_TARGET_TRACKING ||
      s == State::SINGLE_CABINET_FINAL_RECOGNITION_WAIT);
  }

  bool is_full_inventory_recognition_state(State s) const
  {
    return
      full_inventory_active_ &&
      ((s == State::FULL_INVENTORY_NAV_TO_OBSERVE && full_inventory_recognize_during_nav_) ||
      s == State::FULL_INVENTORY_POST_ROUTE_RECOGNITION_WAIT ||
      s == State::FULL_INVENTORY_RECOGNITION_FALLBACK ||
      s == State::FULL_INVENTORY_SAME_SIDE_NEXT_SEARCH);
  }

  static std::string side_row_phase_to_string(SingleCabinetSideRowPhase phase)
  {
    switch (phase) {
      case SingleCabinetSideRowPhase::FIRST_PRIMARY_SCAN:
        return "FIRST_PRIMARY_SCAN";
      case SingleCabinetSideRowPhase::FIRST_ADJUSTED_SCAN:
        return "FIRST_ADJUSTED_SCAN";
      case SingleCabinetSideRowPhase::CORRIDOR_TRANSFER:
        return "CORRIDOR_TRANSFER";
      case SingleCabinetSideRowPhase::SECOND_PRIMARY_SCAN:
        return "SECOND_PRIMARY_SCAN";
      case SingleCabinetSideRowPhase::SECOND_ADJUSTED_SCAN:
        return "SECOND_ADJUSTED_SCAN";
      case SingleCabinetSideRowPhase::COMPLETE:
        return "COMPLETE";
      case SingleCabinetSideRowPhase::NONE:
      default:
        return "NONE";
    }
  }

  static bool int_vectors_equal(const std::vector<int> & lhs, const std::vector<int> & rhs)
  {
    return lhs == rhs;
  }

  bool transform_pose_2d(
    const Pose2D & input,
    const std::string & target_frame_raw,
    Pose2D & output,
    std::string & error_msg) const
  {
    output = Pose2D{};
    error_msg.clear();

    if (!input.valid) {
      error_msg = "输入位姿无效";
      return false;
    }

    const std::string source_frame = sanitize_frame_id(input.frame_id);
    const std::string target_frame = sanitize_frame_id(target_frame_raw);
    if (source_frame.empty() || target_frame.empty()) {
      error_msg = "源/目标坐标系为空";
      return false;
    }

    if (source_frame == target_frame) {
      output = input;
      output.frame_id = target_frame;
      output.valid = true;
      return true;
    }

    geometry_msgs::msg::PoseStamped in_pose;
    in_pose.header.stamp = this->now();
    in_pose.header.frame_id = source_frame;
    in_pose.pose.position.x = input.x;
    in_pose.pose.position.y = input.y;
    in_pose.pose.position.z = 0.0;
    in_pose.pose.orientation = quaternion_from_yaw(input.yaw);

    try {
      auto out_pose = tf_buffer_->transform(
        in_pose,
        target_frame,
        tf2::durationFromSec(0.25));
      output.x = out_pose.pose.position.x;
      output.y = out_pose.pose.position.y;
      output.yaw = yaw_from_quaternion(out_pose.pose.orientation);
      output.frame_id = target_frame;
      output.valid = true;
      return true;
    } catch (const tf2::TransformException & ex) {
      error_msg = ex.what();
      return false;
    }
  }

  void publish_stop()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);
  }

  void set_corridor_mode(bool enabled, bool reverse)
  {
    std_msgs::msg::Bool en;
    en.data = enabled;
    corridor_enable_pub_->publish(en);

    std_msgs::msg::Bool rev;
    rev.data = reverse;
    corridor_reverse_pub_->publish(rev);
  }

  void publish_state_text(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    mission_state_pub_->publish(msg);
  }

  void publish_log(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    mission_log_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "%s", text.c_str());
  }

  std::filesystem::path resolve_route_config_path(const std::string & raw_path) const
  {
    const std::filesystem::path path(raw_path);
    if (path.is_absolute()) {
      return path;
    }

    try {
      const auto share_dir =
        std::filesystem::path(ament_index_cpp::get_package_share_directory("agv_inventory_system"));
      const auto share_candidate = share_dir / path;
      if (std::filesystem::exists(share_candidate)) {
        return share_candidate;
      }
    } catch (const std::exception & ex) {
      RCLCPP_WARN(get_logger(), "查询包share目录失败: %s", ex.what());
    }

    const auto cwd_candidate = std::filesystem::current_path() / path;
    if (std::filesystem::exists(cwd_candidate)) {
      return cwd_candidate;
    }
    const auto source_candidate =
      std::filesystem::current_path() / "src" / "agv_inventory_system" / path;
    if (std::filesystem::exists(source_candidate)) {
      return source_candidate;
    }
    return cwd_candidate;
  }

  void apply_inventory_runtime_params()
  {
    agv_inventory_system::InventoryScannerConfig scanner_config;
    scanner_config.enabled = scanner_enabled_;
    scanner_config.scan_duration_sec = std::max(0.0, scan_duration_sec_);
    scanner_config.scan_timeout_sec = std::max(scanner_config.scan_duration_sec, scan_timeout_sec_);
    scanner_config.scan_retry_count = std::max(0, scan_retry_count_);
    scanner_config.scan_result_timeout_sec = std::max(0.0, scan_result_timeout_sec_);
    scanner_config.rfid_reader_enabled = rfid_reader_enabled_;
    scanner_config.rfid_reader_mode = rfid_reader_mode_;
    scanner_config.rfid_serial_device = rfid_serial_device_;
    scanner_config.rfid_serial_baud = rfid_serial_baud_;
    scanner_config.rfid_scan_duration_sec = std::max(0.1, rfid_scan_duration_sec_);
    scanner_config.rfid_frame_min_length = std::max(8, rfid_frame_min_length_);
    scanner_config.rfid_frame_max_length =
      std::max(scanner_config.rfid_frame_min_length, rfid_frame_max_length_);
    inventory_scanner_.configure(scanner_config);

    lift_up_duration_sec_ = std::max(0.0, lift_up_duration_sec_);
    lift_down_duration_sec_ = std::max(0.0, lift_down_duration_sec_);
    lift_service_timeout_sec_ = std::max(0.1, lift_service_timeout_sec_);
    plc_request_timeout_sec_ = std::max(0.1, plc_request_timeout_sec_);
    plc_retry_count_ = std::max(0, plc_retry_count_);
    plc_open_wait_sec_ = std::max(0.0, plc_open_wait_sec_);
    plc_fail_policy_ = normalize_plc_fail_policy(plc_fail_policy_);
    rear_target_handle_mode_ = normalize_rear_target_handle_mode(rear_target_handle_mode_);
    rear_target_turn_yaw_tolerance_rad_ =
      std::max(0.001, std::abs(rear_target_turn_yaw_tolerance_rad_));
    rear_target_turn_timeout_sec_ = std::max(0.1, rear_target_turn_timeout_sec_);
    rear_target_backup_distance_m_ =
      std::max(0.0, std::isfinite(rear_target_backup_distance_m_) ?
      std::abs(rear_target_backup_distance_m_) : 1.50);
    rear_target_backup_speed_ =
      std::max(0.0, std::isfinite(rear_target_backup_speed_) ?
      std::abs(rear_target_backup_speed_) : 0.08);
    rear_target_backup_timeout_sec_ =
      std::max(0.1, std::isfinite(rear_target_backup_timeout_sec_) ?
      rear_target_backup_timeout_sec_ : 30.0);
    rfid_upload_timeout_sec_ = std::max(0.1, rfid_upload_timeout_sec_);
    rfid_upload_retry_count_ = std::max(0, rfid_upload_retry_count_);
    rfid_upload_fail_policy_ = normalize_rfid_upload_fail_policy(rfid_upload_fail_policy_);
    agv_inventory_system::RfidScanLogWriterConfig rfid_log_config;
    rfid_log_config.enabled = rfid_local_log_enabled_;
    rfid_log_config.path = rfid_local_log_path_;
    rfid_log_config.write_batch_summary = rfid_local_log_write_batch_summary_;
    rfid_scan_log_writer_.configure(rfid_log_config);
    agv_inventory_system::WebApiClientParams web_params;
    web_params.web_client_mode = web_client_mode_;
    web_params.web_base_url = web_base_url_;
    web_params.web_open_gap_endpoint = web_open_gap_endpoint_;
    web_params.web_close_gap_endpoint = web_close_gap_endpoint_;
    web_params.web_status_endpoint = web_status_endpoint_;
    web_params.web_result_endpoint = web_result_endpoint_;
    web_params.plc_server_url = plc_server_url_;
    web_params.plc_open_endpoint = plc_open_endpoint_;
    web_params.plc_open_query_param = plc_open_query_param_;
    web_params.plc_close_endpoint = plc_close_endpoint_;
    web_params.plc_stop_endpoint = plc_stop_endpoint_;
    web_params.plc_hello_endpoint = plc_hello_endpoint_;
    web_params.plc_verify_tls = plc_verify_tls_;
    web_params.plc_require_body_success = plc_require_body_success_;
    web_params.plc_request_timeout_sec = plc_request_timeout_sec_;
    web_params.plc_retry_count = plc_retry_count_;
    web_params.rfid_upload_enabled = rfid_upload_enabled_;
    web_params.rfid_upload_url = rfid_upload_url_;
    web_params.rfid_upload_verify_tls = rfid_upload_verify_tls_;
    web_params.rfid_upload_timeout_sec = rfid_upload_timeout_sec_;
    web_params.rfid_upload_retry_count = rfid_upload_retry_count_;
    web_params.rfid_upload_fail_policy = rfid_upload_fail_policy_;
    web_params.rfid_upload_require_success = rfid_upload_require_success_;
    web_params.rfid_upload_status_path = rfid_upload_status_path_;
    web_api_client_.setParams(web_params);
  }

  bool load_gap_scan_map_config(std::string & reason)
  {
    reason.clear();
    std::map<std::string, std::vector<int>> gap_map;

    try {
      const auto map_path = resolve_route_config_path(gap_scan_map_file_);
      if (!std::filesystem::exists(map_path)) {
        reason = "gap_scan_map.yaml 不存在: " + map_path.string();
        return false;
      }

      const YAML::Node root = YAML::LoadFile(map_path.string());
      const YAML::Node gaps = root["gaps"];
      if (!gaps || !gaps.IsMap()) {
        reason = "gap_scan_map.yaml 缺少 gaps 映射";
        return false;
      }

      for (const auto gap_item : gaps) {
        const std::string gap_id = gap_item.first.as<std::string>();
        const YAML::Node gap_node = gap_item.second;
        std::vector<int> cabinets;

        if (gap_node["first_scan_cabinet"]) {
          cabinets.push_back(gap_node["first_scan_cabinet"].as<int>());
        }
        const YAML::Node second_cabinets = gap_node["second_scan_cabinets"];
        if (second_cabinets && second_cabinets.IsSequence()) {
          for (std::size_t i = 0; i < second_cabinets.size(); ++i) {
            cabinets.push_back(second_cabinets[i].as<int>());
          }
        }

        if (!cabinets.empty()) {
          gap_map[gap_id] = cabinets;
        }
      }

      gap_scan_map_cabinets_by_gap_id_ = gap_map;
      RCLCPP_INFO(
        get_logger(),
        "已加载盘库 gap 扫描映射: %s gaps=%zu",
        map_path.string().c_str(),
        gap_scan_map_cabinets_by_gap_id_.size());
      return true;
    } catch (const std::exception & ex) {
      reason = "解析 gap_scan_map.yaml 失败: " + std::string(ex.what());
      return false;
    }
  }

  bool load_inventory_runtime_config(std::string & reason)
  {
    reason.clear();
    apply_inventory_runtime_params();

    std::string gap_map_reason;
    if (!load_gap_scan_map_config(gap_map_reason)) {
      reason = gap_map_reason;
      return false;
    }

    configured_gap_inventory_plan_.clear();
    inventory_plan_by_gap_id_.clear();
    for (const auto & item : gap_scan_map_cabinets_by_gap_id_) {
      configured_gap_inventory_plan_.push_back(InventoryGapPlan{item.first, item.second});
      inventory_plan_by_gap_id_[item.first] = item.second;
    }

    RCLCPP_INFO(
      get_logger(),
      "已加载盘库运行配置: mode=%s gap_count=%zu layers=%d depth_count=%d "
      "scanner_duration=%.2f lift_duration=%.2f finish_return_mode=%s",
      web_client_mode_.c_str(),
      configured_gap_inventory_plan_.size(),
      scan_layers_,
      scan_depth_count_,
      scan_duration_sec_,
      lift_up_duration_sec_,
      finish_return_mode_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "单柜盘库配置: enabled=%s target_gap=%s target_cabinet=%d "
      "stop_after_scan=%s final_recognition_wait=%.2f",
      single_cabinet_motion_enabled_ ? "true" : "false",
      single_cabinet_motion_target_gap_.c_str(),
      single_cabinet_motion_target_cabinet_,
      single_cabinet_motion_stop_after_scan_ ? "true" : "false",
      single_cabinet_final_recognition_wait_sec_);
    RCLCPP_INFO(
      get_logger(),
      "PLC HTTP配置: enabled=%s server_url=%s open_endpoint=%s open_query_param=%s "
      "verify_tls=%s require_body_success=%s timeout=%.2f retry_count=%d "
      "fail_policy=%s supported=%s open_wait=%.2f call_close_on_done=%s call_stop_on_error=%s",
      plc_http_enabled_ ? "true" : "false",
      plc_server_url_.c_str(),
      plc_open_endpoint_.c_str(),
      plc_open_query_param_.c_str(),
      plc_verify_tls_ ? "true" : "false",
      plc_require_body_success_ ? "true" : "false",
      plc_request_timeout_sec_,
      plc_retry_count_,
      plc_fail_policy_.c_str(),
      cabinet_unit_to_string(plc_supported_cabinets_).c_str(),
      plc_open_wait_sec_,
      plc_call_close_on_mission_done_ ? "true" : "false",
      plc_call_stop_on_error_ ? "true" : "false");
    RCLCPP_INFO(
      get_logger(),
      "RFID上传配置: enabled=%s url=%s verify_tls=%s timeout=%.2f retry_count=%d fail_policy=%s "
      "require_success=%s reader_enabled=%s reader_mode=%s "
      "serial_device=%s serial_baud=%d serial_scan_duration=%.2f frame_id=00EE00 "
      "frame_min_length=%d frame_max_length=%d local_log=%s local_log_path=%s "
      "local_log_summary=%s status_path=%s",
      rfid_upload_enabled_ ? "true" : "false",
      rfid_upload_url_.c_str(),
      rfid_upload_verify_tls_ ? "true" : "false",
      rfid_upload_timeout_sec_,
      rfid_upload_retry_count_,
      rfid_upload_fail_policy_.c_str(),
      rfid_upload_require_success_ ? "true" : "false",
      rfid_reader_enabled_ ? "true" : "false",
      rfid_reader_mode_.c_str(),
      rfid_serial_device_.empty() ? "<empty>" : rfid_serial_device_.c_str(),
      rfid_serial_baud_,
      rfid_scan_duration_sec_,
      rfid_frame_min_length_,
      rfid_frame_max_length_,
      rfid_local_log_enabled_ ? "true" : "false",
      rfid_local_log_path_.empty() ? "<empty>" : rfid_local_log_path_.c_str(),
      rfid_local_log_write_batch_summary_ ? "true" : "false",
      rfid_upload_status_path_.empty() ? "<empty>" : rfid_upload_status_path_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "侧排盘库配置: enabled=%s name=%s first_gap=%s first_seq=%s "
      "second_gap=%s second_seq=%s transfer_target=%d motion_model=%s "
      "exit_distance=%.2f exit_speed=%.3f "
      "exit_timeout=%.2f exit_turn=%s exit_turn_angular=%.3f exit_turn_tolerance=%.3f "
      "exit_turn_timeout=%.2f grid_motion=%s grid_spacing=%.2f grid_speed=%.3f grid_timeout=%.2f",
      single_cabinet_side_row_enabled_ ? "true" : "false",
      single_cabinet_side_row_name_.c_str(),
      single_cabinet_side_row_first_gap_.c_str(),
      cabinet_unit_to_string(single_cabinet_side_row_first_gap_scan_sequence_).c_str(),
      single_cabinet_side_row_second_gap_.c_str(),
      cabinet_unit_to_string(single_cabinet_side_row_second_gap_scan_sequence_).c_str(),
      single_cabinet_side_row_corridor_transfer_target_cabinet_,
      motion_model_.c_str(),
      single_cabinet_exit_distance_m_,
      single_cabinet_exit_speed_,
      single_cabinet_exit_timeout_sec_,
      single_cabinet_exit_turn_enabled_ ? "true" : "false",
      single_cabinet_exit_turn_angular_speed_,
      single_cabinet_exit_turn_yaw_tolerance_rad_,
      single_cabinet_exit_turn_timeout_sec_,
      single_cabinet_grid_motion_enabled_ ? "true" : "false",
      single_cabinet_grid_spacing_m_,
      single_cabinet_grid_move_speed_,
      single_cabinet_grid_move_timeout_sec_);
    RCLCPP_INFO(
      get_logger(),
      "全部盘库配置: enabled=%s sequence=%s left_route=%s right_route=%s "
      "between_side_auto_charge_wait=%.2f between_side_auto_charge_runtime=%.2f "
      "recognize_during_nav=%s "
      "same_side_search=%s speed=%.3f timeout=%.2f pose_hold=%s "
      "left_fixed_y=%.3f left_fixed_yaw=%.4f right_fixed_y=%.3f right_fixed_yaw=%.4f "
      "yaw_kp=%.3f yaw_deadband=%.3f y_kp=%.3f y_deadband=%.3f "
      "y_sign=%.1f max_angular=%.3f recognition_delay=%s delay_distance=%.2f "
      "rear_target=%s rear_mode=%s rear_turn_tolerance=%.3f rear_turn_timeout=%.2f "
      "rear_backup=%s rear_backup_distance=%.2f rear_backup_speed=%.3f rear_backup_timeout=%.2f "
      "final_recognition_wait=%.2f "
      "recognition_fallback=%s fallback_speed=%.3f fallback_wait=%.2f fallback_timeout=%.2f "
      "fallback_sequence=%s post_gap_advance=%s distance=%.2f speed=%.3f timeout=%.2f",
      full_inventory_enabled_ ? "true" : "false",
      cabinet_unit_to_string(full_inventory_sequence_).c_str(),
      full_inventory_left_route_.c_str(),
      full_inventory_right_route_.c_str(),
      between_side_auto_charge_wait_sec_,
      between_side_auto_charge_runtime_sec_,
      full_inventory_recognize_during_nav_ ? "true" : "false",
      full_inventory_same_side_next_search_enabled_ ? "true" : "false",
      full_inventory_same_side_search_speed_,
      full_inventory_same_side_search_timeout_sec_,
      full_inventory_same_side_pose_hold_enabled_ ? "true" : "false",
      full_inventory_same_side_left_fixed_y_m_,
      full_inventory_same_side_left_fixed_yaw_rad_,
      full_inventory_same_side_right_fixed_y_m_,
      full_inventory_same_side_right_fixed_yaw_rad_,
      full_inventory_same_side_yaw_kp_,
      full_inventory_same_side_yaw_deadband_rad_,
      full_inventory_same_side_y_kp_,
      full_inventory_same_side_y_deadband_m_,
      full_inventory_same_side_y_correction_sign_,
      full_inventory_same_side_max_angular_,
      full_inventory_same_side_recognition_delay_enabled_ ? "true" : "false",
      full_inventory_same_side_recognition_delay_distance_m_,
      rear_target_handling_enabled_ ? "true" : "false",
      rear_target_handle_mode_.c_str(),
      rear_target_turn_yaw_tolerance_rad_,
      rear_target_turn_timeout_sec_,
      rear_target_backup_enabled_ ? "true" : "false",
      rear_target_backup_distance_m_,
      rear_target_backup_speed_,
      rear_target_backup_timeout_sec_,
      full_inventory_final_recognition_wait_sec_,
      full_inventory_recognition_fallback_enabled_ ? "true" : "false",
      full_inventory_recognition_fallback_speed_,
      full_inventory_recognition_fallback_wait_sec_,
      full_inventory_recognition_fallback_timeout_sec_,
      double_vector_to_string(full_inventory_recognition_fallback_sequence_).c_str(),
      post_gap_detect_advance_enabled_ ? "true" : "false",
      post_gap_detect_advance_distance_m_,
      post_gap_detect_advance_speed_,
      post_gap_detect_advance_timeout_sec_);
    return true;
  }

  bool load_route_config(std::string & reason)
  {
    reason.clear();
    std::map<std::string, RouteConfig> routes;
    std::map<std::string, std::string> side_route_map;
    std::map<int, std::string> cabinet_side_map;
    std::map<int, std::string> cabinet_entry_side_map;
    std::map<int, SearchDirection> cabinet_gap_search_direction_map;

    try {
      const auto route_path = resolve_route_config_path(route_waypoints_file_);
      if (!std::filesystem::exists(route_path)) {
        reason = "路线配置文件不存在: " + route_path.string();
        return false;
      }

      const YAML::Node root = YAML::LoadFile(route_path.string());
      const YAML::Node route_root = root["routes"];
      const YAML::Node side_route_root = root["side_route_map"];
      const YAML::Node cabinet_side_root = root["cabinet_side_map"];
      const YAML::Node cabinet_entry_side_root = root["cabinet_entry_side_map"];
      const YAML::Node cabinet_gap_search_direction_root = root["cabinet_gap_search_direction_map"];
      if (!route_root || !route_root.IsMap()) {
        reason = "route_waypoints.yaml 缺少 routes 映射";
        return false;
      }
      if (!side_route_root || !side_route_root.IsMap()) {
        reason = "route_waypoints.yaml 缺少 side_route_map 映射";
        return false;
      }
      if (!cabinet_side_root || !cabinet_side_root.IsMap()) {
        reason = "route_waypoints.yaml 缺少 cabinet_side_map 映射";
        return false;
      }

      for (const auto route_item : route_root) {
        RouteConfig route;
        route.name = route_item.first.as<std::string>();
        const YAML::Node route_node = route_item.second;
        if (!route_node || !route_node.IsMap()) {
          reason = "路线配置不是映射: " + route.name;
          return false;
        }

        route.frame_id = sanitize_frame_id(
          route_node["frame_id"] ? route_node["frame_id"].as<std::string>() : nav2_goal_frame_);
        const YAML::Node waypoints = route_node["waypoints"];
        if (!waypoints || !waypoints.IsSequence() || waypoints.size() == 0U) {
          reason = "路线缺少非空 waypoints: " + route.name;
          return false;
        }

        for (std::size_t i = 0; i < waypoints.size(); ++i) {
          const YAML::Node wp = waypoints[i];
          if (!wp || !wp.IsMap() || !wp["x"] || !wp["y"] || !wp["yaw"]) {
            reason = "路线 waypoint 必须包含 x/y/yaw: " + route.name;
            return false;
          }

          Pose2D pose;
          pose.x = wp["x"].as<double>();
          pose.y = wp["y"].as<double>();
          pose.yaw = wp["yaw"].as<double>();
          pose.frame_id = sanitize_frame_id(
            wp["frame_id"] ? wp["frame_id"].as<std::string>() : route.frame_id);
          if (pose.frame_id.empty()) {
            pose.frame_id = sanitize_frame_id(nav2_goal_frame_);
          }
          pose.valid = true;
          route.waypoints.push_back(pose);
        }

        routes[route.name] = route;
      }

      for (const auto side_route_item : side_route_root) {
        std::string target_side;
        if (!try_normalize_entry_side(side_route_item.first.as<std::string>(), target_side)) {
          reason = "side_route_map side 必须为 left/right: " + side_route_item.first.as<std::string>();
          return false;
        }

        const std::string route_name = side_route_item.second.as<std::string>();
        if (routes.find(route_name) == routes.end()) {
          reason = "side_route_map 引用了不存在的路线: " + route_name;
          return false;
        }
        side_route_map[target_side] = route_name;
      }

      if (side_route_map.find("left") == side_route_map.end() ||
        side_route_map.find("right") == side_route_map.end())
      {
        reason = "side_route_map 必须同时配置 left 和 right";
        return false;
      }

      const auto parse_cabinet_side_map =
        [&reason](
        const YAML::Node & map_root,
        const std::string & map_name,
        std::map<int, std::string> & parsed_map) -> bool
        {
          parsed_map.clear();
          for (const auto map_item : map_root) {
            int cabinet_id = -1;
            const std::string cabinet_text = map_item.first.as<std::string>();
            if (!agv_inventory_system::safe_to_int(cabinet_text, cabinet_id)) {
              reason = map_name + " 货柜号非法: " + cabinet_text;
              return false;
            }

            if (!map_item.second.IsScalar()) {
              reason = map_name + " 条目必须直接映射到 left/right: " + cabinet_text;
              return false;
            }
            std::string side;
            if (!MissionManagerNode::try_normalize_entry_side(map_item.second.as<std::string>(), side)) {
              reason = map_name + " side 必须为 left/right: " + cabinet_text;
              return false;
            }
            parsed_map[cabinet_id] = side;
          }
          return true;
        };

      if (!parse_cabinet_side_map(cabinet_side_root, "cabinet_side_map", cabinet_side_map)) {
        return false;
      }

      if (cabinet_entry_side_root) {
        if (!cabinet_entry_side_root.IsMap()) {
          reason = "route_waypoints.yaml cabinet_entry_side_map 必须为映射";
          return false;
        }
        if (!parse_cabinet_side_map(
            cabinet_entry_side_root, "cabinet_entry_side_map", cabinet_entry_side_map))
        {
          return false;
        }
      } else {
        RCLCPP_WARN(
          get_logger(),
          "route_waypoints.yaml 缺少 cabinet_entry_side_map，将回退为 cabinet_side_map");
      }

      for (const auto & side_pair : cabinet_side_map) {
        if (cabinet_entry_side_map.find(side_pair.first) != cabinet_entry_side_map.end()) {
          continue;
        }
        cabinet_entry_side_map[side_pair.first] = side_pair.second;
        if (cabinet_entry_side_root) {
          RCLCPP_WARN(
            get_logger(),
            "cabinet_entry_side_map 缺少货柜%d，回退为 cabinet_side_map side=%s",
            side_pair.first,
            side_pair.second.c_str());
        }
      }

      if (cabinet_gap_search_direction_root) {
        if (!cabinet_gap_search_direction_root.IsMap()) {
          reason = "route_waypoints.yaml cabinet_gap_search_direction_map 必须为映射";
          return false;
        }
        for (const auto map_item : cabinet_gap_search_direction_root) {
          int cabinet_id = -1;
          const std::string cabinet_text = map_item.first.as<std::string>();
          if (!agv_inventory_system::safe_to_int(cabinet_text, cabinet_id)) {
            reason = "cabinet_gap_search_direction_map 货柜号非法: " + cabinet_text;
            return false;
          }
          if (!map_item.second.IsScalar()) {
            reason =
              "cabinet_gap_search_direction_map 条目必须直接映射到 forward/backward: " +
              cabinet_text;
            return false;
          }
          SearchDirection direction{SearchDirection::FORWARD};
          if (!MissionManagerNode::try_parse_search_direction(
              map_item.second.as<std::string>(), direction))
          {
            reason =
              "cabinet_gap_search_direction_map direction 必须为 forward/backward: " +
              cabinet_text;
            return false;
          }
          cabinet_gap_search_direction_map[cabinet_id] = direction;
        }
      } else {
        RCLCPP_WARN(
          get_logger(),
          "route_waypoints.yaml 缺少 cabinet_gap_search_direction_map，SEARCH_GAP 将默认 forward");
      }

      route_configs_ = routes;
      side_route_map_ = side_route_map;
      cabinet_side_map_ = cabinet_side_map;
      cabinet_entry_side_map_ = cabinet_entry_side_map;
      cabinet_gap_search_direction_map_ = cabinet_gap_search_direction_map;
      RCLCPP_INFO(
        get_logger(),
        "已加载巡航路线配置: %s routes=%zu side_route_map=%zu cabinet_side_map=%zu "
        "cabinet_entry_side_map=%zu cabinet_gap_search_direction_map=%zu",
        route_path.string().c_str(),
        route_configs_.size(),
        side_route_map_.size(),
        cabinet_side_map_.size(),
        cabinet_entry_side_map_.size(),
        cabinet_gap_search_direction_map_.size());
      return true;
    } catch (const std::exception & ex) {
      reason = "解析路线配置失败: " + std::string(ex.what());
      return false;
    }
  }

  bool load_warehouse_layout_config(std::string & reason)
  {
    reason.clear();
    std::map<std::string, WarehouseRowLayout> rows_by_side;

    try {
      const auto layout_path = resolve_route_config_path(warehouse_layout_file_);
      if (!std::filesystem::exists(layout_path)) {
        reason = "仓库布局配置文件不存在: " + layout_path.string();
        return false;
      }

      const YAML::Node root = YAML::LoadFile(layout_path.string());
      const YAML::Node rows_root = root["rows"];
      if (!rows_root || !rows_root.IsMap()) {
        reason = "warehouse_layout.yaml 缺少 rows 映射";
        return false;
      }

      for (const auto row_item : rows_root) {
        const std::string row_name = row_item.first.as<std::string>();
        const YAML::Node row_node = row_item.second;
        if (!row_node || !row_node.IsMap()) {
          reason = "仓库布局 row 不是映射: " + row_name;
          return false;
        }

        WarehouseRowLayout row;
        if (!row_node["side"] || !try_normalize_entry_side(row_node["side"].as<std::string>(), row.side)) {
          reason = "仓库布局 row.side 必须为 left/right: " + row_name;
          return false;
        }

        const YAML::Node units = row_node["physical_units"];
        if (!units || !units.IsSequence() || units.size() == 0U) {
          reason = "仓库布局 row 缺少非空 physical_units: " + row_name;
          return false;
        }

        for (std::size_t unit_index = 0; unit_index < units.size(); ++unit_index) {
          const YAML::Node unit_node = units[unit_index];
          if (!unit_node || !unit_node.IsSequence() || unit_node.size() == 0U) {
            reason = "physical_units 条目必须为非空序列: " + row_name;
            return false;
          }

          std::vector<int> unit;
          unit.reserve(unit_node.size());
          for (std::size_t i = 0; i < unit_node.size(); ++i) {
            unit.push_back(unit_node[i].as<int>());
          }
          row.physical_units.push_back(unit);
        }

        rows_by_side[row.side] = row;
      }

      if (rows_by_side.find("left") == rows_by_side.end() ||
        rows_by_side.find("right") == rows_by_side.end())
      {
        reason = "warehouse_layout.yaml 必须同时配置 left/right 两侧 physical_units";
        return false;
      }

      warehouse_rows_by_side_ = rows_by_side;
      RCLCPP_INFO(
        get_logger(),
        "已加载仓库布局配置: %s rows=%zu",
        layout_path.string().c_str(),
        warehouse_rows_by_side_.size());
      return true;
    } catch (const std::exception & ex) {
      reason = "解析仓库布局配置失败: " + std::string(ex.what());
      return false;
    }
  }

  bool find_physical_unit_for_cabinet(
    int cabinet_id,
    std::vector<int> & physical_unit,
    std::string & side,
    std::size_t * unit_index_out = nullptr) const
  {
    physical_unit.clear();
    side.clear();
    for (const auto & row_item : warehouse_rows_by_side_) {
      const auto & units = row_item.second.physical_units;
      for (std::size_t unit_index = 0; unit_index < units.size(); ++unit_index) {
        const auto & unit = units[unit_index];
        if (std::find(unit.begin(), unit.end(), cabinet_id) == unit.end()) {
          continue;
        }
        physical_unit = unit;
        side = row_item.first;
        if (unit_index_out != nullptr) {
          *unit_index_out = unit_index;
        }
        return true;
      }
    }
    return false;
  }

  bool cabinets_in_same_physical_unit(
    int first_cabinet,
    int second_cabinet,
    std::vector<int> & physical_unit,
    std::string & side) const
  {
    physical_unit.clear();
    side.clear();
    std::size_t first_unit_index = 0;
    if (!find_physical_unit_for_cabinet(first_cabinet, physical_unit, side, &first_unit_index)) {
      return false;
    }

    std::vector<int> second_unit;
    std::string second_side;
    std::size_t second_unit_index = 0;
    if (!find_physical_unit_for_cabinet(
        second_cabinet,
        second_unit,
        second_side,
        &second_unit_index))
    {
      return false;
    }

    return side == second_side && first_unit_index == second_unit_index;
  }

  bool should_use_rear_target_handling(
    int finished_cabinet,
    int next_target_cabinet,
    std::string & reason)
  {
    reason.clear();
    std::vector<int> physical_unit;
    std::string physical_side;
    bool same_physical_unit = false;
    if (!warehouse_layout_loaded_) {
      std::string load_reason;
      warehouse_layout_loaded_ = load_warehouse_layout_config(load_reason);
      if (!warehouse_layout_loaded_) {
        reason = "warehouse_layout_load_failed: " + load_reason;
      }
    }
    if (warehouse_layout_loaded_) {
      same_physical_unit = cabinets_in_same_physical_unit(
        finished_cabinet,
        next_target_cabinet,
        physical_unit,
        physical_side);
    }

    bool use_rear_target = false;
    if (!rear_target_handling_enabled_) {
      reason = "rear_target_disabled";
    } else if (!same_physical_unit) {
      reason = reason.empty() ? "not_same_physical_unit" : reason;
    } else if (physical_unit.size() <= 1U) {
      reason = "single_cabinet_physical_unit";
    } else {
      use_rear_target = true;
      reason = "same_physical_unit_and_rear_search_from_entry_yaw";
    }

    publish_full_inventory_log(
      "[rear_target] finished=" + std::to_string(finished_cabinet) +
      " next=" + std::to_string(next_target_cabinet) +
      " same_physical_unit=" + std::string(same_physical_unit ? "1" : "0") +
      " physical_unit=" + cabinet_unit_to_string(physical_unit) +
      " side=" + (physical_side.empty() ? "unknown" : physical_side) +
      " enabled=" + std::string(rear_target_handling_enabled_ ? "1" : "0") +
      " use_rear_target=" + std::string(use_rear_target ? "1" : "0") +
      " reason=" + reason);
    return use_rear_target;
  }

  bool resolve_current_gap_plan(std::string & reason)
  {
    reason.clear();
    current_gap_plan_ = TargetGapPlan{};

    if (!warehouse_layout_loaded_) {
      warehouse_layout_loaded_ = load_warehouse_layout_config(reason);
      if (!warehouse_layout_loaded_) {
        return false;
      }
    }

    const auto row_it = warehouse_rows_by_side_.find(current_target_side_);
    if (row_it == warehouse_rows_by_side_.end()) {
      reason = "目标侧未配置物理货柜单元: " + current_target_side_;
      return false;
    }

    SearchDirection configured_direction{SearchDirection::FORWARD};
    const auto direction_it = cabinet_gap_search_direction_map_.find(current_target_cabinet_);
    if (direction_it == cabinet_gap_search_direction_map_.end()) {
      RCLCPP_WARN(
        get_logger(),
        "目标货柜%d未配置 gap_search_direction，默认使用 forward",
        current_target_cabinet_);
    } else {
      configured_direction = direction_it->second;
    }

    const auto & units = row_it->second.physical_units;
    for (std::size_t unit_index = 0; unit_index < units.size(); ++unit_index) {
      const auto & unit = units[unit_index];
      const auto cabinet_it = std::find(unit.begin(), unit.end(), current_target_cabinet_);
      if (cabinet_it == unit.end()) {
        continue;
      }

      const std::size_t cabinet_index = static_cast<std::size_t>(std::distance(unit.begin(), cabinet_it));
      const bool has_next = unit_index + 1 < units.size();
      const bool has_prev = unit_index > 0;

      TargetGapPlan plan;
      plan.valid = true;
      plan.side = current_target_side_;
      plan.physical_unit = unit;
      plan.unit_index = unit_index;
      plan.cabinet_index_in_unit = cabinet_index;
      plan.search_direction = configured_direction;

      if (cabinet_index + 1 == unit.size() && has_next) {
        plan.gap_before_unit = unit;
        plan.gap_after_unit = units[unit_index + 1];
      } else if (cabinet_index == 0 && has_prev) {
        plan.gap_before_unit = units[unit_index - 1];
        plan.gap_after_unit = unit;
      } else if (has_next) {
        plan.gap_before_unit = unit;
        plan.gap_after_unit = units[unit_index + 1];
      } else if (has_prev) {
        plan.gap_before_unit = units[unit_index - 1];
        plan.gap_after_unit = unit;
      } else {
        reason = "目标货柜所在物理单元没有相邻间隙: " + std::to_string(current_target_cabinet_);
        return false;
      }

      current_gap_plan_ = plan;
      RCLCPP_INFO(
        get_logger(),
        "目标货柜%d找缝规划: target_side=%s entry_side=%s physical_unit=%s expected_gap=%s "
        "gap_search_direction=%s",
        current_target_cabinet_,
        current_gap_plan_.side.c_str(),
        current_entry_side_.c_str(),
        cabinet_unit_to_string(current_gap_plan_.physical_unit).c_str(),
        gap_plan_to_string(current_gap_plan_).c_str(),
        search_direction_to_string(current_gap_plan_.search_direction).c_str());
      return true;
    }

    reason =
      "目标货柜未出现在 " + current_target_side_ +
      " 侧 physical_units 中: " + std::to_string(current_target_cabinet_);
    return false;
  }

  std::string entry_turn_direction_text() const
  {
    return current_entry_side_ == "right" ? "right(angular.z<0)" : "left(angular.z>0)";
  }

  bool configure_current_entry_profile(std::string & reason)
  {
    reason.clear();
    current_entry_profile_valid_ = false;

    if (!std::isfinite(grid_depth_m_) || grid_depth_m_ <= 0.0) {
      reason = "grid_depth_m 必须为正数";
      return false;
    }

    const int max_depth = current_target_side_ == "right" ?
      right_max_depth_index_ : left_max_depth_index_;
    if (max_depth <= 0) {
      reason = "目标侧最大 depth_index 配置非法: side=" + current_target_side_;
      return false;
    }

    if (current_target_depth_index_ < 1 || current_target_depth_index_ > max_depth) {
      reason =
        "depth_index 越界: target_cabinet=" + std::to_string(current_target_cabinet_) +
        " target_side=" + current_target_side_ +
        " target_depth_index=" + std::to_string(current_target_depth_index_) +
        " allowed=1.." + std::to_string(max_depth);
      return false;
    }

    target_depth_center_m_ =
      (static_cast<double>(current_target_depth_index_) - 0.5) * grid_depth_m_;
    const double dynamic_straight_distance = target_depth_center_m_ + entry_center_offset_m_;
    target_straight_distance_ =
      enable_grid_center_entry_ ? dynamic_straight_distance : entry_distance_;

    if (!std::isfinite(target_straight_distance_) || target_straight_distance_ <= 0.0) {
      reason =
        "入缝目标直行距离非法: target_straight_distance=" +
        std::to_string(target_straight_distance_);
      return false;
    }

    if (std::isfinite(max_dynamic_entry_distance_m_) &&
      max_dynamic_entry_distance_m_ > 0.0 &&
      target_straight_distance_ > max_dynamic_entry_distance_m_)
    {
      reason =
        "入缝目标直行距离超过安全上限: target_straight_distance=" +
        std::to_string(target_straight_distance_) +
        " max_dynamic_entry_distance_m=" + std::to_string(max_dynamic_entry_distance_m_);
      return false;
    }

    current_entry_profile_valid_ = true;
    return true;
  }

  void log_current_target_entry_plan() const
  {
    RCLCPP_INFO(
      get_logger(),
      "目标任务解析: target_cabinet=%d target_side=%s entry_side=%s target_level_index=%d "
      "target_depth_index=%d target_depth_center_m=%.3f entry_center_offset_m=%.3f "
      "target_straight_distance=%.3f physical_unit=%s expected_gap=%s "
      "gap_search_direction=%s entry_turn_direction=%s grid_center_entry=%s",
      current_target_cabinet_,
      current_target_side_.c_str(),
      current_entry_side_.c_str(),
      current_target_level_index_,
      current_target_depth_index_,
      target_depth_center_m_,
      entry_center_offset_m_,
      target_straight_distance_,
      cabinet_unit_to_string(current_gap_plan_.physical_unit).c_str(),
      gap_plan_to_string(current_gap_plan_).c_str(),
      search_direction_to_string(current_gap_plan_.search_direction).c_str(),
      entry_turn_direction_text().c_str(),
      enable_grid_center_entry_ ? "true" : "false(fallback entry_distance)");
  }

  void publish_entry_side()
  {
    if (!entry_side_pub_) {
      return;
    }

    std_msgs::msg::String msg;
    msg.data = current_entry_side_;
    entry_side_pub_->publish(msg);
  }

  void publish_target_lidar_side()
  {
    if (!target_lidar_side_pub_) {
      return;
    }

    std::string side;
    if (!try_normalize_entry_side(current_entry_side_, side)) {
      RCLCPP_WARN(
        get_logger(),
        "[mission_manager][distance] target=%d invalid entry_side=%s, skip target_lidar_side",
        current_target_cabinet_,
        current_entry_side_.c_str());
      return;
    }

    std_msgs::msg::String msg;
    msg.data = side;
    target_lidar_side_pub_->publish(msg);

    RCLCPP_INFO(
      get_logger(),
      "[mission_manager][distance] target=%d publish target_lidar_side=%s",
      current_target_cabinet_,
      side.c_str());
  }

  void publish_current_target_cabinet(int cabinet_id)
  {
    if (!current_target_cabinet_pub_) {
      return;
    }

    std_msgs::msg::Int32 msg;
    msg.data = cabinet_id;
    current_target_cabinet_pub_->publish(msg);
    RCLCPP_INFO(
      get_logger(),
      "[mission_manager][recognizer] publish current_target_cabinet=%d",
      cabinet_id);
  }

  void clear_current_target_cabinet()
  {
    publish_current_target_cabinet(-1);
  }

  void set_gap_detector_enabled(bool enabled)
  {
    if (!gap_detector_enable_pub_) {
      return;
    }
    if (gap_detector_enable_initialized_ && gap_detector_enabled_cmd_ == enabled) {
      return;
    }

    std_msgs::msg::Bool msg;
    msg.data = enabled;
    gap_detector_enable_pub_->publish(msg);
    gap_detector_enabled_cmd_ = enabled;
    gap_detector_enable_initialized_ = true;

    RCLCPP_INFO(
      get_logger(),
      "gap_detector 使能切换: %s",
      enabled ? "ENABLE(SEARCH_GAP/WAITING_GAP检测)" : "DISABLE(停止检测)");
  }

  void set_distance_estimator_enabled(bool enabled, bool force_publish = false)
  {
    if (!distance_estimator_enable_pub_) {
      return;
    }
    if (!force_publish && distance_estimator_enable_initialized_ &&
      distance_estimator_enabled_cmd_ == enabled)
    {
      return;
    }

    std_msgs::msg::Bool msg;
    msg.data = enabled;
    distance_estimator_enable_pub_->publish(msg);
    distance_estimator_enabled_cmd_ = enabled;
    distance_estimator_enable_initialized_ = true;
    if (!enabled) {
      has_distance_ = false;
      latest_distance_ = 0.0;
    }

    RCLCPP_INFO(
      get_logger(),
      "distance_estimator 使能切换: %s%s",
      enabled ? "ENABLE(目标侧向测距)" : "DISABLE(停止目标测距)",
      force_publish ? " [force]" : "");
  }

  void set_recognizer_topic_enabled(bool enabled, bool force_publish = false)
  {
    if (!recognizer_enable_pub_) {
      return;
    }
    if (!force_publish && recognizer_enable_initialized_ && recognizer_enabled_cmd_ == enabled) {
      return;
    }

    std_msgs::msg::Bool msg;
    msg.data = enabled;
    recognizer_enable_pub_->publish(msg);
    recognizer_enabled_cmd_ = enabled;
    recognizer_enable_initialized_ = true;

    RCLCPP_INFO(
      get_logger(),
      "recognizer 使能切换: %s%s",
      enabled ? "ENABLE(识别阶段)" : "DISABLE(停止识别)",
      force_publish ? " [force]" : "");
  }

  bool should_enable_recognizer_for_state(State s) const
  {
    return
      (s == State::IDLE && recognize_in_idle_) ||
      s == State::SINGLE_CABINET_FINAL_RECOGNITION_WAIT ||
      (s == State::FULL_INVENTORY_NAV_TO_OBSERVE && full_inventory_recognize_during_nav_) ||
      s == State::FULL_INVENTORY_POST_ROUTE_RECOGNITION_WAIT ||
      s == State::FULL_INVENTORY_RECOGNITION_FALLBACK;
  }

  void set_state(State s, const std::string & detail)
  {
    if (state_ == State::FULL_INVENTORY_TARGET_DISTANCE_ALIGN &&
      s != State::FULL_INVENTORY_TARGET_DISTANCE_ALIGN)
    {
      target_distance_gap_open_count_ = 0;
    }

    const bool gap_enabled =
      s == State::SEARCH_GAP ||
      s == State::WAITING_GAP ||
      s == State::SINGLE_CABINET_WAITING_GAP ||
      s == State::FULL_INVENTORY_SEARCH_GAP;
    const bool recognizer_enabled = should_enable_recognizer_for_state(s);
    const bool distance_enabled =
      s == State::TARGET_TRACKING ||
      s == State::SINGLE_CABINET_TARGET_TRACKING ||
      s == State::FULL_INVENTORY_TARGET_DISTANCE_ALIGN;

    if (s == State::DONE || s == State::ERROR ||
      (s == State::IDLE && !mission_active_ && !inventory_flow_active_))
    {
      clear_current_target_cabinet();
    }

    set_gap_detector_enabled(gap_enabled);
    set_distance_estimator_enabled(distance_enabled);
    set_recognizer_topic_enabled(recognizer_enabled);
    if (s == State::IDLE) {
      RCLCPP_INFO(
        get_logger(),
        "recognize_in_idle=%s, recognizer %s while IDLE",
        recognize_in_idle_ ? "true" : "false",
        recognizer_enabled ? "enabled" : "disabled");
    }
    state_ = s;
    if (s == State::ENTERING_GAP ||
      s == State::SINGLE_CABINET_ENTERING_GAP ||
      s == State::FULL_INVENTORY_ENTERING_GAP)
    {
      robot_inside_gap_ = true;
    }
    publish_state_text(state_to_string(state_));
    publish_log("[" + state_to_string(state_) + "] " + detail);
  }

  bool state_indicates_in_gap_or_gap_motion() const
  {
    switch (state_) {
      case State::ENTERING_GAP:
      case State::INVENTORYING:
      case State::SINGLE_CABINET_ENTERING_GAP:
      case State::SINGLE_CABINET_IN_GAP_SCAN:
      case State::SINGLE_CABINET_STOP_AFTER_SCAN:
      case State::SINGLE_CABINET_EXIT_GAP:
      case State::SINGLE_CABINET_REENTER_FOR_ADJUSTED_SCAN:
      case State::SINGLE_CABINET_ADJUSTED_SIDE_SCAN:
      case State::SINGLE_CABINET_REENTER_NEXT_GAP:
      case State::SINGLE_CABINET_NEXT_GAP_SCAN:
      case State::SINGLE_CABINET_FINAL_EXIT_GAP:
      case State::FULL_INVENTORY_ENTERING_GAP:
      case State::FULL_INVENTORY_IN_GAP_SCAN:
      case State::FULL_INVENTORY_EXIT_GAP:
      case State::SAFE_EXIT_GAP:
        return true;
      default:
        break;
    }

    return false;
  }

  bool is_in_gap_or_gap_motion_state() const
  {
    return state_indicates_in_gap_or_gap_motion() ||
           robot_inside_gap_ ||
           entry_gap_phase_ == EntryGapPhase::MOVING_TO_GRID_CENTER;
  }

  std::string flow_state_summary() const
  {
    return "mission_active=" + std::string(mission_active_ ? "true" : "false") +
           ",inventory_flow_active=" + std::string(inventory_flow_active_ ? "true" : "false") +
           ",single_cabinet_motion_active=" +
           std::string(single_cabinet_motion_active_ ? "true" : "false") +
           ",full_inventory_active=" + std::string(full_inventory_active_ ? "true" : "false");
  }

  std::string pending_interrupt_request_to_string(PendingInterruptRequest request) const
  {
    switch (request) {
      case PendingInterruptRequest::RETURN_HOME:
        return "回零点";
      case PendingInterruptRequest::AUTO_RECHARGE:
        return "自动回充";
      case PendingInterruptRequest::NONE:
      default:
        return "无";
    }
  }

  void respond_with_pending_interrupt(
    PendingInterruptRequest request,
    const std::string & base_message,
    std_srvs::srv::Trigger::Response & response)
  {
    const bool updated = pending_interrupt_request_ != PendingInterruptRequest::NONE;
    const std::string previous = pending_interrupt_request_to_string(pending_interrupt_request_);
    pending_interrupt_request_ = request;

    response.success = true;
    response.message = base_message;
    if (updated) {
      response.message += " 已更新待执行请求，原待执行请求为" + previous + "。";
    }
    publish_log(response.message);
    publish_state_text(response.message);
  }

  Pose2D configured_home_pose() const
  {
    Pose2D pose;
    pose.x = home_pose_x_;
    pose.y = home_pose_y_;
    pose.yaw = home_pose_yaw_;
    pose.frame_id = sanitize_frame_id(home_pose_frame_id_);
    pose.valid =
      std::isfinite(pose.x) &&
      std::isfinite(pose.y) &&
      std::isfinite(pose.yaw) &&
      !pose.frame_id.empty();
    return pose;
  }

  void prepare_interrupt_mission_control(const std::string & reason)
  {
    RCLCPP_WARN(get_logger(), "盘库任务中断准备：%s", reason.c_str());
    cancel_nav2_route_goal(reason);
    cancel_nav2_return_goal(reason);
    reset_nav_route_runtime();
    set_corridor_mode(false, false);
    publish_stop();
    request_recognizer_enable(false);
    set_recognizer_topic_enabled(false, true);
    set_distance_estimator_enabled(false, true);
    set_gap_detector_enabled(false);
    reset_wait_gap_runtime();
    reset_entry_gap_runtime();
    clear_inventory_upload_batch("cancel");
    reset_single_cabinet_scan_runtime();
    publish_gap_context();
    clear_current_target_cabinet();

    mission_active_ = false;
    inventory_flow_active_ = false;
    cancel_requested_ = false;
    mission_force_map_origin_on_finish_ = false;
    single_cabinet_motion_active_ = false;
    single_cabinet_gap_searching_ = false;
    single_cabinet_target_recognized_logged_ = false;
    single_cabinet_close_requested_ = false;
    full_inventory_active_ = false;
    reset_between_side_auto_charge_runtime();
    gap_request_queue_.clear();
    current_gap_request_index_ = 0;
    target_visible_ = false;
    has_distance_ = false;
    latest_distance_ = 0.0;
    return_mode_ = ReturnMode::NONE;
    nav2_return_in_progress_ = false;
    nav2_result_ready_ = false;
    nav2_result_success_ = false;
    nav2_result_text_.clear();
    nav2_goal_handle_.reset();
    mission_error_reason_ = reason;
    reset_single_cabinet_side_row_context();
    reset_full_inventory_context();
  }

  bool start_return_home_interrupt(const std::string & reason, std::string & message)
  {
    message.clear();
    Pose2D target_pose = configured_home_pose();
    if (!target_pose.valid) {
      message = "回零点目标参数无效，请检查 home_pose_frame_id/home_pose_x/home_pose_y/home_pose_yaw。";
      publish_log(message);
      publish_state_text(message);
      return false;
    }

    prepare_interrupt_mission_control(reason);
    return_mode_ = ReturnMode::CANCEL_HOME;

    std::string nav2_fail_reason;
    if (!begin_nav2_return(target_pose, nav2_fail_reason)) {
      message = "回零点目标发送失败: " + nav2_fail_reason;
      mission_active_ = false;
      return_mode_ = ReturnMode::NONE;
      set_state(State::ERROR, message);
      return false;
    }

    mission_active_ = true;
    message = "已收到回零点指令，当前小车不在缝隙内，正在中断盘库任务并返回零点。";
    set_state(State::RETURNING_HOME, reason + "，返航目标=零点，方式=Nav2");
    publish_state_text(message);
    publish_log(message);
    return true;
  }

  bool start_inventory_auto_recharge(const std::string & reason, std::string & message)
  {
    message.clear();
    prepare_interrupt_mission_control(reason);
    if (!send_auto_recharge_start_request(reason, message)) {
      return false;
    }

    message = "已收到自动充电指令，当前小车不在缝隙内，正在中断盘库任务并启动自动回充流程。";
    set_state(State::AUTO_RECHARGING, reason + "，已通知自动回充节点开始回充");
    publish_state_text(message);
    publish_log(message);
    return true;
  }

  bool send_auto_recharge_start_request(const std::string & reason, std::string & message)
  {
    message.clear();
    if (!inventory_auto_recharge_start_client_) {
      message = "自动回充服务不可用，请确认 inventory_auto_recharger 节点已启动且 Nav2 保持打开。";
      publish_log(message);
      publish_state_text(message);
      return false;
    }

    const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(std::max(0.0, auto_recharge_service_timeout_sec_)));
    if (!inventory_auto_recharge_start_client_->wait_for_service(timeout)) {
      message = "自动回充服务不可用，请确认 inventory_auto_recharger 节点已启动且 Nav2 保持打开。";
      publish_log(message);
      publish_state_text(message);
      return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    (void)inventory_auto_recharge_start_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        const auto response = future.get();
        if (!response) {
          publish_log("自动回充节点未返回有效响应。");
          return;
        }
        const std::string detail =
          std::string("自动回充节点响应: ") +
          (response->success ? "success" : "failed") +
          "，" + response->message;
        if (response->success) {
          publish_log(detail);
        } else {
          RCLCPP_ERROR(get_logger(), "%s", detail.c_str());
          publish_log(detail);
        }
      });

    message = reason + "，已通知自动回充节点开始回充。";
    publish_log(message);
    return true;
  }

  bool send_auto_recharge_cancel_request(const std::string & reason, std::string & message)
  {
    message.clear();
    if (!inventory_auto_recharge_cancel_client_) {
      message = "自动回充取消服务不可用，请确认 inventory_auto_recharger 节点已启动。";
      publish_log(message);
      publish_state_text(message);
      return false;
    }

    const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(std::max(0.0, auto_recharge_cancel_timeout_sec_)));
    if (!inventory_auto_recharge_cancel_client_->wait_for_service(timeout)) {
      message = "自动回充取消服务不可用，请确认 inventory_auto_recharger 节点已启动。";
      publish_log(message);
      publish_state_text(message);
      return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    (void)inventory_auto_recharge_cancel_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        try {
          const auto response = future.get();
          if (!response) {
            publish_log("自动回充取消节点未返回有效响应。");
            return;
          }
          const std::string detail =
            std::string("自动回充取消节点响应: ") +
            (response->success ? "success" : "failed") +
            "，" + response->message;
          if (response->success) {
            publish_log(detail);
          } else {
            RCLCPP_ERROR(get_logger(), "%s", detail.c_str());
            publish_log(detail);
          }
        } catch (const std::exception & ex) {
          const std::string detail = std::string("自动回充取消响应处理异常: ") + ex.what();
          RCLCPP_ERROR(get_logger(), "%s", detail.c_str());
          publish_log(detail);
        }
      });

    message = reason + "，已发送取消自动回充请求。";
    publish_log(message);
    return true;
  }

  bool send_auto_recharge_cancel_request_with_callback(
    const std::string & reason,
    std::function<void(bool, const std::string &)> callback,
    std::string & message)
  {
    message.clear();
    if (!inventory_auto_recharge_cancel_client_) {
      message = "自动回充取消服务不可用，请确认 inventory_auto_recharger 节点已启动。";
      publish_log(message);
      publish_state_text(message);
      return false;
    }

    const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(std::max(0.0, auto_recharge_cancel_timeout_sec_)));
    if (!inventory_auto_recharge_cancel_client_->wait_for_service(timeout)) {
      message = "自动回充取消服务不可用，请确认 inventory_auto_recharger 节点已启动。";
      publish_log(message);
      publish_state_text(message);
      return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    (void)inventory_auto_recharge_cancel_client_->async_send_request(
      request,
      [this, callback](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        try {
          const auto response = future.get();
          if (!response) {
            callback(false, "自动回充取消节点未返回有效响应。");
            return;
          }
          callback(response->success, response->message);
        } catch (const std::exception & ex) {
          callback(false, std::string("自动回充取消响应处理异常: ") + ex.what());
        }
      });

    message = reason + "，已发送取消自动回充请求。";
    publish_log(message);
    return true;
  }

  bool start_final_map_origin_return(const std::string & reason)
  {
    Pose2D target_pose = configured_home_pose();
    if (!target_pose.valid) {
      mission_active_ = false;
      set_state(State::ERROR, "回零点目标参数无效，请检查 home_pose_frame_id/home_pose_x/home_pose_y/home_pose_yaw。");
      return false;
    }

    set_corridor_mode(false, false);
    publish_stop();
    request_recognizer_enable(false);
    set_recognizer_topic_enabled(false, true);
    set_distance_estimator_enabled(false, true);
    set_gap_detector_enabled(false);
    reset_wait_gap_runtime();
    reset_entry_gap_runtime();
    publish_gap_context();

    return_mode_ = ReturnMode::FINISH_HOME;
    std::string nav2_fail_reason;
    if (!begin_nav2_return(target_pose, nav2_fail_reason)) {
      fail_nav2_return("Nav2 回零点目标发送失败: " + nav2_fail_reason);
      return false;
    }

    mission_active_ = true;
    set_state(State::RETURNING_HOME, reason + "，返航目标=map_origin，方式=Nav2");
    return true;
  }

  bool start_final_auto_charge(const std::string & reason)
  {
    std::string message;
    if (!start_inventory_auto_recharge(reason, message)) {
      prepare_interrupt_mission_control(message);
      mission_active_ = false;
      inventory_flow_active_ = false;
      set_state(State::ERROR, message);
      return false;
    }
    return true;
  }

  bool start_finish_action(const std::string & reason, bool force_map_origin)
  {
    if (force_map_origin || finish_return_mode_is_map_origin()) {
      return start_final_map_origin_return(reason);
    }
    return start_final_auto_charge(reason);
  }

  bool consume_pending_interrupt_after_exit()
  {
    if (pending_interrupt_request_ == PendingInterruptRequest::NONE) {
      return false;
    }

    const auto request = pending_interrupt_request_;
    pending_interrupt_request_ = PendingInterruptRequest::NONE;
    std::string message;
    if (request == PendingInterruptRequest::RETURN_HOME) {
      message = "已完成出缝，开始执行之前记录的回零点请求。";
      publish_log(message);
      publish_state_text(message);
      std::string start_message;
      if (!start_return_home_interrupt(message, start_message)) {
        prepare_interrupt_mission_control(start_message);
        mission_active_ = false;
        inventory_flow_active_ = false;
        set_state(State::ERROR, start_message);
      }
      return true;
    }

    if (request == PendingInterruptRequest::AUTO_RECHARGE) {
      message = "已完成出缝，开始执行之前记录的自动回充请求。";
      publish_log(message);
      publish_state_text(message);
      std::string start_message;
      if (!start_inventory_auto_recharge(message, start_message)) {
        prepare_interrupt_mission_control(start_message);
        mission_active_ = false;
        inventory_flow_active_ = false;
        set_state(State::ERROR, start_message);
      }
      return true;
    }

    return false;
  }

  void begin_final_exit_for_pending_after_stop()
  {
    const std::string pending_text =
      pending_interrupt_request_to_string(pending_interrupt_request_);
    publish_single_cabinet_log(
      "检测到待执行中断请求(" + pending_text + ")，先复用最终出缝流程完成出缝");
    single_cabinet_after_exit_action_ = SingleCabinetAfterExitAction::FINAL_CLOSE_AND_DONE;
    single_cabinet_exit_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    single_cabinet_exit_phase_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    single_cabinet_exit_phase_ = SingleCabinetExitPhase::STRAIGHT_REVERSE;
    single_cabinet_exit_effective_timeout_sec_ = single_cabinet_exit_timeout_sec_;
    mission_active_ = true;
    set_single_cabinet_state(
      State::SINGLE_CABINET_FINAL_EXIT_GAP,
      "检测到待执行中断请求，先完成最终出缝");
  }

  void reset_safe_exit_gap_runtime()
  {
    safe_exit_gap_start_distance_ = 0.0;
    safe_exit_gap_target_distance_ = 0.0;
    safe_exit_gap_target_yaw_ = 0.0;
    safe_exit_gap_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }

  void clear_safe_exit_gap_recovery_context()
  {
    safe_exit_gap_context_valid_ = false;
    safe_exit_gap_maybe_inside_gap_ = false;
    safe_exit_gap_distance_m_ = 0.0;
    safe_exit_gap_yaw_rad_ = 0.0;
    safe_exit_gap_yaw_valid_ = false;
    safe_exit_gap_context_source_state_.clear();
    safe_exit_gap_context_stamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }

  void record_safe_exit_gap_yaw_context(const std::string & source_state)
  {
    safe_exit_gap_context_valid_ = true;
    safe_exit_gap_context_source_state_ = source_state;
    safe_exit_gap_context_stamp_ = this->now();
    if (target_gap_yaw_valid_ && std::isfinite(target_gap_yaw_)) {
      safe_exit_gap_yaw_rad_ = normalize_angle(target_gap_yaw_);
      safe_exit_gap_yaw_valid_ = true;
    }
  }

  void update_safe_exit_gap_recovery_context(
    double entering_distance,
    bool maybe_inside_gap,
    const std::string & source_state)
  {
    safe_exit_gap_context_valid_ = true;
    safe_exit_gap_maybe_inside_gap_ = safe_exit_gap_maybe_inside_gap_ || maybe_inside_gap;
    safe_exit_gap_context_source_state_ = source_state;
    safe_exit_gap_context_stamp_ = this->now();
    if (std::isfinite(entering_distance)) {
      safe_exit_gap_distance_m_ = std::max(
        safe_exit_gap_distance_m_,
        std::max(0.0, entering_distance));
    }
    if (target_gap_yaw_valid_ && std::isfinite(target_gap_yaw_)) {
      safe_exit_gap_yaw_rad_ = normalize_angle(target_gap_yaw_);
      safe_exit_gap_yaw_valid_ = true;
    }
  }

  bool safe_exit_gap_recovery_context_available() const
  {
    return safe_exit_gap_context_valid_ && safe_exit_gap_maybe_inside_gap_;
  }

  double safe_exit_gap_distance(std::string & source) const
  {
    const double fallback = std::isfinite(single_cabinet_exit_distance_m_) ?
      single_cabinet_exit_distance_m_ : 1.20;
    double base = fallback;
    source = "fallback_config";
    if (safe_exit_gap_context_valid_ &&
      std::isfinite(safe_exit_gap_distance_m_) &&
      safe_exit_gap_distance_m_ > 0.05)
    {
      base = safe_exit_gap_distance_m_;
      source = "recovery_context";
    } else {
      const double measured = std::isfinite(single_cabinet_last_entering_straight_distance_) ?
        single_cabinet_last_entering_straight_distance_ : 0.0;
      if (measured > 0.05) {
        base = measured;
        source = "last_entering_distance";
      }
    }
    const double extra = std::isfinite(safe_exit_gap_extra_clearance_m_) ?
      std::max(0.0, safe_exit_gap_extra_clearance_m_) : 0.30;
    return std::max(0.05, base + extra);
  }

  double safe_exit_gap_yaw(const Pose2D & current, std::string & source) const
  {
    if (safe_exit_gap_context_valid_ && safe_exit_gap_yaw_valid_ &&
      std::isfinite(safe_exit_gap_yaw_rad_))
    {
      source = "recovery_context";
      return normalize_angle(safe_exit_gap_yaw_rad_);
    }
    if (target_gap_yaw_valid_ && std::isfinite(target_gap_yaw_)) {
      source = "target_gap_yaw";
      return normalize_angle(target_gap_yaw_);
    }
    source = "current_yaw";
    return normalize_angle(current.yaw);
  }

  bool begin_safe_exit_gap_flow(std::string & message)
  {
    message.clear();
    if (!safe_exit_gap_enabled_) {
      message = "安全出缝功能未启用。";
      return false;
    }
    std::string odom_reason;
    if (!current_odom_ready_for_entry(odom_reason)) {
      message = "安全出缝无法执行，里程计不可用: " + odom_reason;
      publish_stop();
      publish_log(message);
      return false;
    }
    const Pose2D current = current_pose_2d();
    if (!current.valid || !std::isfinite(current.yaw)) {
      message = "安全出缝无法执行，当前位姿/yaw 无效。";
      publish_stop();
      publish_log(message);
      return false;
    }

    std::string distance_source;
    const double target_distance = safe_exit_gap_distance(distance_source);
    std::string yaw_source;
    const double target_yaw = safe_exit_gap_yaw(current, yaw_source);
    stop_all_inventory_controls_for_safe_action("安全出缝准备");
    safe_exit_gap_start_distance_ = odom_cumulative_distance_;
    safe_exit_gap_target_distance_ = target_distance;
    safe_exit_gap_target_yaw_ = target_yaw;
    safe_exit_gap_start_time_ = this->now();
    mission_active_ = true;
    inventory_flow_active_ = false;
    single_cabinet_motion_active_ = false;
    full_inventory_active_ = false;
    pending_interrupt_request_ = PendingInterruptRequest::NONE;
    set_state(State::SAFE_EXIT_GAP, "开始安全出缝");
    message =
      "已停止盘库相关动作，开始低速倒退安全出缝。distance_source=" + distance_source +
      " target_distance=" + format_fixed(target_distance, 3) +
      " yaw_source=" + yaw_source +
      " target_yaw=" + format_fixed(target_yaw, 4) +
      " context_state=" + (safe_exit_gap_context_source_state_.empty() ?
      "none" : safe_exit_gap_context_source_state_);
    publish_state_text(message);
    publish_log(message);
    return true;
  }

  void finish_safe_exit_gap_success()
  {
    publish_stop();
    robot_inside_gap_ = false;
    reset_safe_exit_gap_runtime();
    clear_safe_exit_gap_recovery_context();
    mission_active_ = false;
    inventory_flow_active_ = false;
    single_cabinet_motion_active_ = false;
    full_inventory_active_ = false;
    pending_interrupt_request_ = PendingInterruptRequest::NONE;
    set_state(State::IDLE, "safe_exit_gap completed: SAFE_IN_CORRIDOR，系统待机");
    publish_state_text("安全出缝完成，当前已在通道安全位置。");
  }

  void fail_safe_exit_gap(const std::string & reason)
  {
    publish_stop();
    reset_safe_exit_gap_runtime();
    mission_active_ = false;
    inventory_flow_active_ = false;
    mission_error_reason_ = reason;
    set_state(State::ERROR, "安全出缝失败: " + reason);
  }

  void handle_safe_exit_gap_state()
  {
    if (safe_exit_gap_start_time_.nanoseconds() == 0) {
      std::string message;
      if (!begin_safe_exit_gap_flow(message)) {
        fail_safe_exit_gap(message);
      }
      return;
    }

    std::string odom_reason;
    if (!current_odom_ready_for_entry(odom_reason)) {
      fail_safe_exit_gap("里程计异常: " + odom_reason);
      return;
    }
    const Pose2D current = current_pose_2d();
    if (!current.valid || !std::isfinite(current.yaw)) {
      fail_safe_exit_gap("当前位姿/yaw 无效");
      return;
    }

    const double elapsed = (this->now() - safe_exit_gap_start_time_).seconds();
    const double timeout = std::isfinite(safe_exit_gap_timeout_sec_) ?
      std::max(0.1, safe_exit_gap_timeout_sec_) : 30.0;
    if (elapsed > timeout) {
      fail_safe_exit_gap("安全出缝超时");
      return;
    }

    const double traveled = distance_since(safe_exit_gap_start_distance_);
    if (traveled >= safe_exit_gap_target_distance_) {
      finish_safe_exit_gap_success();
      return;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = -std::clamp(std::abs(safe_exit_gap_speed_mps_), 0.0, 0.20);
    if (cmd.linear.x > -1e-4) {
      fail_safe_exit_gap("安全出缝速度非法");
      return;
    }
    cmd.angular.z = yaw_hold_command(
      safe_exit_gap_target_yaw_,
      current.yaw,
      safe_exit_gap_yaw_kp_,
      safe_exit_gap_max_angular_z_);
    cmd_pub_->publish(cmd);
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "[mission_manager][SAFE_EXIT_GAP] traveled=%.2f target=%.2f current_yaw=%.3f "
      "target_yaw=%.3f cmd.linear.x=%.3f cmd.angular.z=%.3f elapsed=%.2f",
      traveled,
      safe_exit_gap_target_distance_,
      current.yaw,
      safe_exit_gap_target_yaw_,
      cmd.linear.x,
      cmd.angular.z,
      elapsed);
  }

  void reset_stop_auto_charge_depart_runtime()
  {
    stop_auto_charge_depart_phase_ = StopAutoChargeDepartPhase::IDLE;
    stop_auto_charge_depart_start_distance_ = 0.0;
    stop_auto_charge_depart_target_yaw_ = 0.0;
    stop_auto_charge_depart_phase_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    stop_auto_charge_depart_continuation_ = StopAutoChargeDepartContinuation::IDLE;
    stop_auto_charge_cancel_response_ready_ = false;
    stop_auto_charge_cancel_response_success_ = false;
    stop_auto_charge_cancel_response_message_.clear();
  }

  bool auto_recharge_status_active_for_depart() const
  {
    const std::string status = agv_inventory_system::trim(latest_auto_recharge_status_);
    return status == "STARTING" ||
      status == "NAVIGATING" ||
      status == "DOCKING" ||
      status == "CHARGING" ||
      status == "CANCELING" ||
      status == "COMPLETE";
  }

  bool auto_recharge_status_released_for_depart() const
  {
    const std::string status = agv_inventory_system::trim(latest_auto_recharge_status_);
    return status.empty() || status == "CANCELED" || status == "IDLE";
  }

  bool auto_recharge_control_active_for_depart() const
  {
    return state_ == State::AUTO_RECHARGING ||
      between_side_auto_charge_cancel_sent_ ||
      auto_recharge_status_active_for_depart() ||
      latest_auto_recharge_charging_ ||
      latest_auto_recharge_recharge_flag_ != 0;
  }

  bool has_active_recharge_or_charging_state() const
  {
    return state_ == State::AUTO_RECHARGING ||
      between_side_auto_charge_active_ ||
      (between_side_auto_charge_cancel_sent_ && !between_side_auto_charge_cancel_response_ready_) ||
      (stop_auto_charge_depart_phase_ == StopAutoChargeDepartPhase::CANCELING &&
      !stop_auto_charge_cancel_response_ready_) ||
      auto_recharge_status_active_for_depart() ||
      latest_auto_recharge_charging_ ||
      latest_auto_recharge_recharge_flag_ != 0;
  }

  bool auto_recharge_status_blocks_start_mission() const
  {
    return auto_recharge_status_active_for_depart();
  }

  bool auto_recharge_control_blocks_start_mission(std::string & reason) const
  {
    reason.clear();
    if (state_ == State::AUTO_RECHARGING) {
      reason = "state=AUTO_RECHARGING";
      return true;
    }
    if (between_side_auto_charge_cancel_sent_ && !between_side_auto_charge_cancel_response_ready_) {
      reason = "between-side auto recharge cancel response pending";
      return true;
    }
    if (auto_recharge_status_blocks_start_mission()) {
      reason = "auto_recharge/status=" + agv_inventory_system::trim(latest_auto_recharge_status_);
      return true;
    }
    if (latest_auto_recharge_charging_) {
      reason = "robot_charging_flag=true";
      return true;
    }
    if (latest_auto_recharge_recharge_flag_ != 0) {
      reason = "robot_recharge_flag=" + std::to_string(latest_auto_recharge_recharge_flag_);
      return true;
    }
    if (stop_auto_charge_depart_phase_ == StopAutoChargeDepartPhase::CANCELING &&
      !stop_auto_charge_cancel_response_ready_)
    {
      reason = "stop auto charge cancel response pending";
      return true;
    }
    return false;
  }

  bool start_stop_auto_charge_and_depart_flow(
    std::string & message,
    StopAutoChargeDepartContinuation continuation = StopAutoChargeDepartContinuation::IDLE)
  {
    message.clear();
    const bool full_inventory_between_side_depart =
      continuation == StopAutoChargeDepartContinuation::FULL_INVENTORY_BETWEEN_SIDES;
    const bool state_in_gap_or_gap_motion = state_indicates_in_gap_or_gap_motion();
    const bool stale_entry_gap_phase =
      entry_gap_phase_ == EntryGapPhase::MOVING_TO_GRID_CENTER;
    if (state_in_gap_or_gap_motion || robot_inside_gap_ || stale_entry_gap_phase) {
      const bool allow_between_side_stale_entry_phase =
        full_inventory_between_side_depart &&
        state_ == State::FULL_INVENTORY_AUTO_CHARGE_BETWEEN_SIDES &&
        !state_in_gap_or_gap_motion &&
        !robot_inside_gap_ &&
        stale_entry_gap_phase;
      if (allow_between_side_stale_entry_phase) {
        publish_log(
          "between-side depart allowed after auto charge, clear stale entry_gap_phase "
          "state=" + state_to_string(state_) +
          " flow_state=" + flow_state_summary() +
          " robot_inside_gap=false entry_gap_phase=" +
          entry_gap_phase_to_string(entry_gap_phase_) +
          " continuation=" + stop_auto_charge_depart_continuation_to_string(continuation));
        reset_entry_gap_runtime();
      } else {
        message =
          "当前小车可能在缝隙内，请先执行停止并退出缝隙。"
          " state=" + state_to_string(state_) +
          " flow_state=" + flow_state_summary() +
          " robot_inside_gap=" + std::string(robot_inside_gap_ ? "true" : "false") +
          " state_in_gap_or_gap_motion=" +
          std::string(state_in_gap_or_gap_motion ? "true" : "false") +
          " entry_gap_phase=" + entry_gap_phase_to_string(entry_gap_phase_) +
          " continuation=" + stop_auto_charge_depart_continuation_to_string(continuation);
        publish_log(message);
        publish_state_text(message);
        return false;
      }
    }
    if (state_ == State::SAFE_EXIT_GAP || state_ == State::STOP_AUTO_CHARGE_AND_DEPART) {
      message = "安全动作正在执行，请勿重复操作。";
      publish_log(message);
      publish_state_text(message);
      return false;
    }
    if (!full_inventory_between_side_depart && !auto_recharge_control_active_for_depart())
    {
      message = "当前未处于自动回充/充电状态，禁止离桩移动。";
      publish_log(message);
      publish_state_text(message);
      return false;
    }
    std::string odom_reason;
    if (!current_odom_ready_for_entry(odom_reason)) {
      message = "停止自动充电后离桩需要 odom，但当前不可用: " + odom_reason;
      publish_stop();
      publish_log(message);
      publish_state_text(message);
      return false;
    }
    const Pose2D current = current_pose_2d();
    if (!current.valid || !std::isfinite(current.yaw)) {
      message = "停止自动充电后离桩需要当前 yaw，但当前位姿无效。";
      publish_stop();
      publish_log(message);
      publish_state_text(message);
      return false;
    }

    if (full_inventory_between_side_depart) {
      cancel_nav2_route_goal("跨侧自动回充完成，准备离桩");
      cancel_nav2_return_goal("跨侧自动回充完成，准备离桩");
      reset_nav_route_runtime();
      reset_wait_gap_runtime();
      reset_entry_gap_runtime();
      set_corridor_mode(false, false);
      request_recognizer_enable(false);
      set_recognizer_topic_enabled(false, true);
      set_distance_estimator_enabled(false, true);
      set_gap_detector_enabled(false);
      publish_gap_context();
      publish_stop();
    } else {
      stop_all_inventory_controls_for_safe_action("停止自动充电并离桩准备");
    }
    reset_stop_auto_charge_depart_runtime();
    stop_auto_charge_depart_continuation_ = continuation;
    mission_active_ = true;
    inventory_flow_active_ = continuation == StopAutoChargeDepartContinuation::FULL_INVENTORY_BETWEEN_SIDES;
    single_cabinet_motion_active_ = false;
    full_inventory_active_ =
      continuation == StopAutoChargeDepartContinuation::FULL_INVENTORY_BETWEEN_SIDES ?
      full_inventory_active_ : false;
    return_mode_ = ReturnMode::NONE;
    pending_interrupt_request_ = PendingInterruptRequest::NONE;
    stop_auto_charge_depart_target_yaw_ = normalize_angle(current.yaw);
    stop_auto_charge_depart_phase_ = StopAutoChargeDepartPhase::CANCELING;
    stop_auto_charge_depart_phase_start_time_ = this->now();

    std::string cancel_message;
    if (!send_auto_recharge_cancel_request_with_callback(
        "停止自动充电并离桩",
        [this](bool success, const std::string & response_message) {
          stop_auto_charge_cancel_response_ready_ = true;
          stop_auto_charge_cancel_response_success_ = success;
          stop_auto_charge_cancel_response_message_ = response_message;
        },
        cancel_message))
    {
      stop_auto_charge_cancel_response_ready_ = true;
      stop_auto_charge_cancel_response_success_ = false;
      stop_auto_charge_cancel_response_message_ = cancel_message;
      RCLCPP_WARN(
        get_logger(),
        "停止自动充电并离桩：底层 cancel 请求未发送成功，仍尝试 mission_manager 离桩: %s",
        cancel_message.c_str());
      publish_log(
        "停止自动充电并离桩：底层 cancel 请求未发送成功，仍尝试 mission_manager 离桩: " +
        cancel_message);
    }

    set_state(State::STOP_AUTO_CHARGE_AND_DEPART, "停止自动充电并离桩：等待自动回充取消");
    message = "已发送停止自动充电请求，等待回充控制释放后离桩移动。";
    publish_state_text(message);
    publish_log(message);
    return true;
  }

  void fail_stop_auto_charge_and_depart(const std::string & reason)
  {
    publish_stop();
    const bool full_inventory_continuation =
      stop_auto_charge_depart_continuation_ ==
      StopAutoChargeDepartContinuation::FULL_INVENTORY_BETWEEN_SIDES;
    reset_stop_auto_charge_depart_runtime();
    mission_active_ = false;
    inventory_flow_active_ = full_inventory_continuation ? inventory_flow_active_ : false;
    return_mode_ = ReturnMode::NONE;
    mission_error_reason_ = reason;
    if (full_inventory_continuation) {
      fail_full_inventory("跨侧离桩失败: " + reason);
    } else {
      set_state(State::ERROR, "停止自动充电并离桩失败: " + reason);
    }
  }

  void finish_stop_auto_charge_and_depart_success()
  {
    publish_stop();
    const bool full_inventory_continuation =
      stop_auto_charge_depart_continuation_ ==
      StopAutoChargeDepartContinuation::FULL_INVENTORY_BETWEEN_SIDES;
    reset_stop_auto_charge_depart_runtime();
    return_mode_ = ReturnMode::NONE;
    if (full_inventory_continuation) {
      mission_active_ = true;
      inventory_flow_active_ = true;
      publish_full_inventory_log("between-side stop_auto_charge_depart finished, continue next side");
      continue_full_inventory_after_between_side_auto_charge();
      return;
    }
    mission_active_ = false;
    inventory_flow_active_ = false;
    set_state(State::IDLE, "SAFE_IN_CORRIDOR，停止自动充电并离桩完成");
    publish_state_text("停止自动充电完成，已离开充电桩。");
  }

  void handle_stop_auto_charge_and_depart_state()
  {
    const double phase_elapsed =
      stop_auto_charge_depart_phase_start_time_.nanoseconds() == 0 ?
      0.0 : (this->now() - stop_auto_charge_depart_phase_start_time_).seconds();
    const double timeout = std::isfinite(stop_auto_charge_depart_timeout_sec_) ?
      std::max(0.1, stop_auto_charge_depart_timeout_sec_) : 20.0;
    if (stop_auto_charge_depart_phase_ != StopAutoChargeDepartPhase::CANCELING &&
      phase_elapsed > timeout)
    {
      fail_stop_auto_charge_and_depart(
        "阶段 " + stop_auto_charge_depart_phase_to_string(stop_auto_charge_depart_phase_) + " 超时");
      return;
    }

    switch (stop_auto_charge_depart_phase_) {
      case StopAutoChargeDepartPhase::CANCELING: {
        publish_stop();
        if (!stop_auto_charge_cancel_response_ready_) {
          const double cancel_timeout = std::isfinite(auto_recharge_cancel_response_timeout_sec_) ?
            std::max(0.1, auto_recharge_cancel_response_timeout_sec_) : 15.0;
          if (phase_elapsed < cancel_timeout) {
            return;
          }
          stop_auto_charge_cancel_response_ready_ = true;
          stop_auto_charge_cancel_response_success_ = false;
          stop_auto_charge_cancel_response_message_ =
            "底层自动回充取消响应超时，timeout=" + format_seconds(cancel_timeout);
          RCLCPP_WARN(
            get_logger(),
            "%s，继续执行 mission_manager 离桩。",
            stop_auto_charge_cancel_response_message_.c_str());
          publish_log(stop_auto_charge_cancel_response_message_ + "，继续执行 mission_manager 离桩。");
        }
        if (!stop_auto_charge_cancel_response_success_) {
          RCLCPP_WARN(
            get_logger(),
            "底层自动回充取消未确认成功，继续执行 mission_manager 离桩: %s",
            stop_auto_charge_cancel_response_message_.c_str());
          publish_log(
            "底层自动回充取消未确认成功，继续执行 mission_manager 离桩: " +
            stop_auto_charge_cancel_response_message_);
        } else {
          publish_log(
            "底层自动回充取消已返回，进入 mission_manager 统一离桩阶段: " +
            stop_auto_charge_cancel_response_message_);
        }
        stop_auto_charge_depart_phase_ = StopAutoChargeDepartPhase::STOP_BEFORE;
        stop_auto_charge_depart_phase_start_time_ = this->now();
        publish_log("离桩前先停车等待。");
        return;
      }
      case StopAutoChargeDepartPhase::STOP_BEFORE: {
        publish_stop();
        const double wait_sec = std::isfinite(stop_auto_charge_depart_stop_before_sec_) ?
          std::max(0.0, stop_auto_charge_depart_stop_before_sec_) : 0.5;
        if (phase_elapsed < wait_sec) {
          return;
        }
        std::string odom_reason;
        if (!current_odom_ready_for_entry(odom_reason)) {
          fail_stop_auto_charge_and_depart("离桩前里程计不可用: " + odom_reason);
          return;
        }
        const Pose2D current = current_pose_2d();
        if (!current.valid || !std::isfinite(current.yaw)) {
          fail_stop_auto_charge_and_depart("离桩前当前 yaw 无效");
          return;
        }
        stop_auto_charge_depart_start_distance_ = odom_cumulative_distance_;
        stop_auto_charge_depart_target_yaw_ = normalize_angle(current.yaw);
        stop_auto_charge_depart_phase_ = StopAutoChargeDepartPhase::DEPARTING;
        stop_auto_charge_depart_phase_start_time_ = this->now();
        publish_log("开始按 stop_auto_charge_depart_speed_mps 符号低速离桩。");
        return;
      }
      case StopAutoChargeDepartPhase::DEPARTING: {
        std::string odom_reason;
        if (!current_odom_ready_for_entry(odom_reason)) {
          fail_stop_auto_charge_and_depart("离桩中里程计不可用: " + odom_reason);
          return;
        }
        const Pose2D current = current_pose_2d();
        if (!current.valid || !std::isfinite(current.yaw)) {
          fail_stop_auto_charge_and_depart("离桩中当前 yaw 无效");
          return;
        }
        const double traveled = distance_since(stop_auto_charge_depart_start_distance_);
        const double target_distance = std::max(
          0.01,
          std::isfinite(stop_auto_charge_depart_distance_m_) ?
          stop_auto_charge_depart_distance_m_ : 0.5);
        if (traveled >= target_distance) {
          publish_stop();
          stop_auto_charge_depart_phase_ = StopAutoChargeDepartPhase::STOP_AFTER;
          stop_auto_charge_depart_phase_start_time_ = this->now();
          publish_log("离桩距离已达到目标，停车等待。");
          return;
        }
        geometry_msgs::msg::Twist cmd;
        const double speed_abs = std::clamp(std::abs(stop_auto_charge_depart_speed_mps_), 0.0, 0.20);
        if (speed_abs <= 1e-4) {
          fail_stop_auto_charge_and_depart("离桩速度非法");
          return;
        }
        cmd.linear.x = std::copysign(speed_abs, stop_auto_charge_depart_speed_mps_);
        cmd.angular.z = yaw_hold_command(
          stop_auto_charge_depart_target_yaw_,
          current.yaw,
          stop_auto_charge_depart_yaw_kp_,
          stop_auto_charge_depart_max_angular_z_);
        cmd_pub_->publish(cmd);
        RCLCPP_INFO_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "[mission_manager][STOP_AUTO_CHARGE_AND_DEPART] traveled=%.2f target=%.2f "
          "cmd.linear.x=%.3f cmd.angular.z=%.3f current_yaw=%.3f target_yaw=%.3f",
          traveled,
          target_distance,
          cmd.linear.x,
          cmd.angular.z,
          current.yaw,
          stop_auto_charge_depart_target_yaw_);
        return;
      }
      case StopAutoChargeDepartPhase::STOP_AFTER: {
        publish_stop();
        const double wait_sec = std::isfinite(stop_auto_charge_depart_stop_after_sec_) ?
          std::max(0.0, stop_auto_charge_depart_stop_after_sec_) : 0.5;
        if (phase_elapsed >= wait_sec) {
          finish_stop_auto_charge_and_depart_success();
        }
        return;
      }
      case StopAutoChargeDepartPhase::IDLE:
      default:
        fail_stop_auto_charge_and_depart("内部 phase 未初始化");
        return;
    }
  }

  Pose2D current_pose_2d() const
  {
    Pose2D pose;
    if (!latest_odom_) {
      return pose;
    }
    pose.x = latest_odom_->pose.pose.position.x;
    pose.y = latest_odom_->pose.pose.position.y;
    pose.yaw = latest_yaw_;
    pose.frame_id = latest_odom_frame_id_.empty() ?
      sanitize_frame_id(odom_topic_) : sanitize_frame_id(latest_odom_frame_id_);
    pose.valid = true;
    return pose;
  }

  void request_recognizer_enable(bool enable)
  {
    if (recognizer_enabled_ == enable) {
      return;
    }

    if (!recognizer_trigger_client_->wait_for_service(std::chrono::milliseconds(200))) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "识别触发服务不可用: %s", recognizer_trigger_service_.c_str());
      return;
    }

    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = enable;
    (void)recognizer_trigger_client_->async_send_request(req);
    recognizer_enabled_ = enable;
  }

  void reset_segment_distance()
  {
    segment_start_distance_ = odom_cumulative_distance_;
  }

  double segment_distance() const
  {
    return std::max(0.0, odom_cumulative_distance_ - segment_start_distance_);
  }

  double distance_since(double start_distance) const
  {
    if (!std::isfinite(start_distance)) {
      return 0.0;
    }
    return std::max(0.0, odom_cumulative_distance_ - start_distance);
  }

  double yaw_hold_command(double target_yaw, double current_yaw, double kp, double max_angular) const
  {
    if (!std::isfinite(target_yaw) || !std::isfinite(current_yaw) ||
      !std::isfinite(kp) || !std::isfinite(max_angular))
    {
      return 0.0;
    }
    const double yaw_error = normalize_angle(target_yaw - current_yaw);
    const double limit = std::max(0.0, std::abs(max_angular));
    return std::clamp(kp * yaw_error, -limit, limit);
  }

  std::string stop_auto_charge_depart_phase_to_string(StopAutoChargeDepartPhase phase) const
  {
    switch (phase) {
      case StopAutoChargeDepartPhase::IDLE:
        return "IDLE";
      case StopAutoChargeDepartPhase::CANCELING:
        return "CANCELING";
      case StopAutoChargeDepartPhase::STOP_BEFORE:
        return "STOP_BEFORE";
      case StopAutoChargeDepartPhase::DEPARTING:
        return "DEPARTING";
      case StopAutoChargeDepartPhase::STOP_AFTER:
        return "STOP_AFTER";
      default:
        return "UNKNOWN";
    }
  }

  std::string stop_auto_charge_depart_continuation_to_string(
    StopAutoChargeDepartContinuation continuation) const
  {
    switch (continuation) {
      case StopAutoChargeDepartContinuation::IDLE:
        return "IDLE";
      case StopAutoChargeDepartContinuation::FULL_INVENTORY_BETWEEN_SIDES:
        return "FULL_INVENTORY_BETWEEN_SIDES";
      default:
        return "UNKNOWN";
    }
  }

  double full_inventory_same_side_recognition_delay_distance() const
  {
    if (!full_inventory_same_side_recognition_delay_enabled_ ||
      !std::isfinite(full_inventory_same_side_recognition_delay_distance_m_))
    {
      return 0.0;
    }
    return std::max(0.0, full_inventory_same_side_recognition_delay_distance_m_);
  }

  bool full_inventory_same_side_recognition_delayed() const
  {
    if (state_ != State::FULL_INVENTORY_SAME_SIDE_NEXT_SEARCH) {
      return false;
    }
    const double delay_distance = full_inventory_same_side_recognition_delay_distance();
    return delay_distance > 1e-4 && segment_distance() < delay_distance;
  }

  bool parse_target_metadata(
    const std::string & code,
    TargetMetadata & target,
    std::string & reason) const
  {
    target = TargetMetadata{};
    reason.clear();

    const std::string cleaned = agv_inventory_system::trim(code);
    if (cleaned.empty()) {
      reason = "目标编号为空";
      return false;
    }

    const auto items = agv_inventory_system::split(cleaned, '-');
    if (items.size() == 1U) {
      if (!agv_inventory_system::safe_to_int(items[0], target.cabinet_id)) {
        reason = "目标编号格式非法: " + code;
        return false;
      }
      target.level_index = 1;
      target.depth_index = 1;
      target.depth_defaulted = true;
      if (target.cabinet_id <= 0) {
        reason = "货柜号必须大于 0: " + code;
        return false;
      }
      return true;
    }

    if (items.size() == 3U || items.size() == 4U) {
      int warehouse_id = 0;
      if (!agv_inventory_system::safe_to_int(items[0], warehouse_id) ||
        !agv_inventory_system::safe_to_int(items[1], target.cabinet_id) ||
        !agv_inventory_system::safe_to_int(items[2], target.level_index))
      {
        reason = "目标编号格式非法: " + code;
        return false;
      }

      if (items.size() == 4U) {
        if (!agv_inventory_system::safe_to_int(items[3], target.depth_index)) {
          reason = "目标 depth_index 非法: " + code;
          return false;
        }
      } else {
        target.depth_index = 1;
        target.depth_defaulted = true;
      }

      if (warehouse_id <= 0) {
        reason = "仓库号必须大于 0: " + code;
        return false;
      }
      if (target.cabinet_id <= 0) {
        reason = "货柜号必须大于 0: " + code;
        return false;
      }
      if (target.level_index <= 0) {
        reason = "target_level_index 必须大于 0: " + code;
        return false;
      }
      return true;
    }

    reason = "目标编号格式非法，应为 货柜号、仓库-货柜-层 或 仓库-货柜-层-深度: " + code;
    return false;
  }

  bool parse_target_cabinet(const std::string & code, int & cabinet_id) const
  {
    TargetMetadata target;
    std::string reason;
    if (!parse_target_metadata(code, target, reason)) {
      return false;
    }
    cabinet_id = target.cabinet_id;
    return true;
  }

  bool prepare_current_target(std::string & reason)
  {
    if (targets_.empty() || current_target_index_ >= targets_.size()) {
      reason = "目标列表为空";
      return false;
    }

    TargetMetadata target;
    if (!parse_target_metadata(targets_[current_target_index_], target, reason)) {
      return false;
    }

    current_target_cabinet_ = target.cabinet_id;
    current_target_level_index_ = target.level_index;
    current_target_depth_index_ = target.depth_index;
    current_target_depth_defaulted_ = target.depth_defaulted;
    current_entry_profile_valid_ = false;
    current_gap_plan_ = TargetGapPlan{};
    reset_entry_gap_runtime();

    if (current_target_depth_defaulted_) {
      RCLCPP_WARN(
        get_logger(),
        "任务目标 %s 未传入 depth_index，默认 target_depth_index=1",
        targets_[current_target_index_].c_str());
    }

    return true;
  }

  bool resolve_current_route(std::string & reason)
  {
    reason.clear();
    if (!routes_loaded_) {
      routes_loaded_ = load_route_config(reason);
      if (!routes_loaded_) {
        return false;
      }
    }

    const auto side_it = cabinet_side_map_.find(current_target_cabinet_);
    if (side_it == cabinet_side_map_.end()) {
      reason = "目标货柜未配置目标侧: " + std::to_string(current_target_cabinet_);
      return false;
    }

    const std::string target_side = normalize_entry_side(side_it->second);
    const auto entry_side_it = cabinet_entry_side_map_.find(current_target_cabinet_);
    std::string entry_side = target_side;
    if (entry_side_it == cabinet_entry_side_map_.end()) {
      RCLCPP_WARN(
        get_logger(),
        "目标货柜%d未配置 entry_side，回退使用 target_side=%s",
        current_target_cabinet_,
        target_side.c_str());
    } else {
      entry_side = normalize_entry_side(entry_side_it->second);
    }
    const auto side_route_it = side_route_map_.find(target_side);
    if (side_route_it == side_route_map_.end()) {
      reason = "目标侧未配置巡航路线: " + target_side;
      return false;
    }

    const auto route_it = route_configs_.find(side_route_it->second);
    if (route_it == route_configs_.end()) {
      reason = "巡航路线不存在: " + side_route_it->second;
      return false;
    }

    current_route_ = route_it->second;
    current_route_name_ = current_route_.name;
    current_target_side_ = target_side;
    current_entry_side_ = entry_side;
    RCLCPP_INFO(
      get_logger(),
      "目标货柜%d解析: route=%s target_side=%s warehouse_side=%s entry_side=%s detection_side=%s waypoints=%zu",
      current_target_cabinet_,
      current_route_name_.c_str(),
      current_target_side_.c_str(),
      current_target_side_.c_str(),
      current_entry_side_.c_str(),
      current_entry_side_.c_str(),
      current_route_.waypoints.size());
    publish_entry_side();
    publish_target_lidar_side();
    publish_current_target_cabinet(current_target_cabinet_);
    return true;
  }

  void reset_target_recognition_stability()
  {
    target_recognition_candidate_id_ = -1;
    target_recognition_stable_count_ = 0;
    target_recognition_first_time_ = zero_time(get_clock());
    target_recognition_last_time_ = zero_time(get_clock());
    target_found_pending_ = false;
  }

  bool update_target_recognition_stability(int rec_id)
  {
    const auto now = this->now();
    const bool same_candidate =
      rec_id == target_recognition_candidate_id_ &&
      target_recognition_last_time_.nanoseconds() != 0 &&
      (now - target_recognition_last_time_).seconds() <= recognition_timeout_sec_;

    if (!same_candidate) {
      target_recognition_candidate_id_ = rec_id;
      target_recognition_stable_count_ = 1;
      target_recognition_first_time_ = now;
      target_recognition_last_time_ = now;
    } else {
      ++target_recognition_stable_count_;
      target_recognition_last_time_ = now;
    }

    return
      target_recognition_stable_count_ >= std::max(1, target_recognition_stable_frames_) &&
      (now - target_recognition_first_time_).seconds() >=
      std::max(0.0, target_recognition_stable_time_sec_);
  }

  void apply_odom_measurement(const nav_msgs::msg::Odometry::SharedPtr msg, bool primary_source)
  {
    latest_odom_ = msg;
    latest_odom_time_ = this->now();
    latest_odom_frame_id_ = sanitize_frame_id(msg->header.frame_id);
    latest_yaw_ = yaw_from_quaternion(msg->pose.pose.orientation);
    has_yaw_ = true;

    if (!has_prev_odom_) {
      prev_odom_x_ = msg->pose.pose.position.x;
      prev_odom_y_ = msg->pose.pose.position.y;
      has_prev_odom_ = true;
      using_primary_odom_ = primary_source;
      return;
    }

    if (using_primary_odom_ != primary_source) {
      // 主备里程计切源时重置积分，避免因两个源坐标偏移导致里程突变。
      using_primary_odom_ = primary_source;
      prev_odom_x_ = msg->pose.pose.position.x;
      prev_odom_y_ = msg->pose.pose.position.y;
      return;
    }

    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;
    const double dx = x - prev_odom_x_;
    const double dy = y - prev_odom_y_;
    odom_cumulative_distance_ += std::hypot(dx, dy);
    prev_odom_x_ = x;
    prev_odom_y_ = y;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg, bool primary_source)
  {
    const auto now = this->now();

    if (primary_source) {
      last_primary_odom_time_ = now;
      apply_odom_measurement(msg, true);
      return;
    }

    last_fallback_odom_time_ = now;
    if ((now - last_primary_odom_time_).seconds() > odom_primary_timeout_sec_) {
      apply_odom_measurement(msg, false);
    }
  }

  bool recognition_is_fresh() const
  {
    if (!latest_recognition_) {
      return false;
    }
    return (this->now() - latest_recognition_time_).seconds() <= recognition_timeout_sec_;
  }

  void switch_single_cabinet_to_waiting_gap_after_recognition(int rec_id)
  {
    const bool in_side_row_transfer =
      single_cabinet_side_row_active_ &&
      single_cabinet_side_row_phase_ == SingleCabinetSideRowPhase::CORRIDOR_TRANSFER;
    if (state_ == State::SINGLE_CABINET_FINAL_RECOGNITION_WAIT) {
      publish_single_cabinet_log(
        "final recognition success cabinet=" + std::to_string(rec_id) +
        (in_side_row_transfer ?
        ", switch to SINGLE_CABINET_PREPARE_NEXT_GAP" : ", switch to SINGLE_CABINET_WAITING_GAP"));
    } else {
      publish_single_cabinet_log(
        "recognized target cabinet=" + std::to_string(rec_id) +
        (in_side_row_transfer ?
        ", switch to SINGLE_CABINET_PREPARE_NEXT_GAP" : ", switch to SINGLE_CABINET_WAITING_GAP"));
    }

    cancel_nav2_route_goal("单柜盘库稳定识别到目标货柜");
    nav2_route_stop_hold_active_ = false;
    nav2_route_cancel_requested_ = false;
    target_found_pending_ = false;
    set_corridor_mode(false, false);
    publish_stop();
    request_recognizer_enable(false);
    set_recognizer_topic_enabled(false, true);
    has_distance_ = false;
    tracking_stable_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    if (in_side_row_transfer) {
      publish_single_cabinet_log(
        "[corridor_transfer] recognized target cabinet=" + std::to_string(rec_id));
      publish_single_cabinet_log(
        "[corridor_transfer] switch to next gap=" + single_cabinet_side_row_second_gap_);
      set_single_cabinet_state(
        State::SINGLE_CABINET_PREPARE_NEXT_GAP,
        "走廊转移已稳定识别目标柜号，准备下一个缝隙");
      return;
    }
    begin_search_gap_flow();
  }

  void begin_single_cabinet_final_recognition_wait()
  {
    cancel_nav2_route_goal("单柜盘库巡航路线已走完，进入末端识别等待");
    nav2_route_goal_in_progress_ = false;
    nav2_route_result_ready_ = false;
    nav2_route_stop_hold_active_ = false;
    nav2_route_cancel_requested_ = false;
    target_found_pending_ = false;
    set_corridor_mode(false, false);
    publish_stop();
    request_recognizer_enable(true);
    set_recognizer_topic_enabled(true, true);
    has_distance_ = false;
    target_visible_ = false;
    single_cabinet_final_recognition_wait_start_ = this->now();
    publish_single_cabinet_log(
      "route finished, wait final recognition target=" +
      std::to_string(current_target_cabinet_) +
      " timeout=" + format_seconds(single_cabinet_final_recognition_wait_sec_));
    set_single_cabinet_state(
      State::SINGLE_CABINET_FINAL_RECOGNITION_WAIT,
      "route finished, wait final recognition target=" +
      std::to_string(current_target_cabinet_) +
      " timeout=" + format_seconds(single_cabinet_final_recognition_wait_sec_) + " sec");
  }

  bool handle_single_cabinet_recognition(int rec_id)
  {
    if (!is_single_cabinet_recognition_state(state_)) {
      return false;
    }

    const bool matched = rec_id == current_target_cabinet_;
    if (!matched) {
      reset_target_recognition_stability();
      RCLCPP_INFO(
        get_logger(),
        "[mission_manager][single_cabinet][recognition] target_mismatch recognized=%d target=%d",
        rec_id,
        current_target_cabinet_);
      return true;
    }

    const bool stable_ready = update_target_recognition_stability(rec_id);
    const int required_count = std::max(1, target_recognition_stable_frames_);
    RCLCPP_INFO(
      get_logger(),
      "[mission_manager][single_cabinet][recognition] state=%s target=%d recognized=%d "
      "stable_count=%d required=%d matched=true ready=%s",
      state_to_string(state_).c_str(),
      current_target_cabinet_,
      rec_id,
      target_recognition_stable_count_,
      required_count,
      stable_ready ? "true" : "false");

    last_target_seen_time_ = this->now();
    target_visible_ = true;

    if (stable_ready) {
      switch_single_cabinet_to_waiting_gap_after_recognition(rec_id);
    }
    return true;
  }

  void recognized_callback(
    const agv_inventory_system::msg::RecognizedNumber::SharedPtr msg)
  {
    latest_recognition_ = msg;
    latest_recognition_time_ = this->now();
    if (single_cabinet_motion_active_) {
      RCLCPP_INFO(
        get_logger(),
        "[mission_manager][single_cabinet][recognition_rx] state=%s target=%d raw_number=%s "
        "valid=%s conf=%.3f",
        state_to_string(state_).c_str(),
        current_target_cabinet_,
        msg->number.c_str(),
        msg->valid ? "true" : "false",
        static_cast<double>(msg->confidence));
    }
    if (!mission_active_) {
      return;
    }
    if (!msg->valid) {
      if (is_nav_route_like_state(state_) ||
        is_single_cabinet_recognition_state(state_) ||
        is_full_inventory_recognition_state(state_))
      {
        reset_target_recognition_stability();
      }
      return;
    }

    int rec_id = -1;
    if (!agv_inventory_system::safe_to_int(msg->number, rec_id)) {
      return;
    }

    if (handle_full_inventory_recognition(rec_id)) {
      return;
    }

    if (handle_single_cabinet_recognition(rec_id)) {
      return;
    }

    if (rec_id != current_target_cabinet_) {
      if (is_nav_route_like_state(state_)) {
        reset_target_recognition_stability();
      }
      return;
    }

    // 返航阶段不再做目标识别触发，避免“返航途中重新识别目标”。
    if (state_ == State::RETURNING || state_ == State::RETURNING_HOME) {
      return;
    }

    last_target_seen_time_ = this->now();
    target_visible_ = true;

    if (is_nav_route_like_state(state_)) {
      if (update_target_recognition_stability(rec_id)) {
        if (single_cabinet_motion_active_ && !single_cabinet_target_recognized_logged_) {
          publish_single_cabinet_log("recognized target cabinet=" + std::to_string(rec_id));
          single_cabinet_target_recognized_logged_ = true;
        }
        target_found_pending_ = true;
        begin_target_found_stop_hold();
      }
      return;
    }

  }

  bool resolve_scan_to_base_transform(
    const sensor_msgs::msg::LaserScan & scan,
    double & tx,
    double & ty,
    double & yaw) const
  {
    tx = 0.0;
    ty = 0.0;
    yaw = 0.0;

    const std::string target = sanitize_frame_id("base_link");
    std::string source = sanitize_frame_id(scan.header.frame_id);
    if (source.empty()) {
      source = target;
    }

    if (source == target) {
      return true;
    }

    try {
      geometry_msgs::msg::TransformStamped tf_msg;
      if (scan.header.stamp.sec == 0 && scan.header.stamp.nanosec == 0) {
        tf_msg = tf_buffer_->lookupTransform(
          target, source, tf2::TimePointZero, tf2::durationFromSec(0.08));
      } else {
        tf_msg = tf_buffer_->lookupTransform(
          target, source, rclcpp::Time(scan.header.stamp), tf2::durationFromSec(0.08));
      }
      tx = tf_msg.transform.translation.x;
      ty = tf_msg.transform.translation.y;
      yaw = yaw_from_quaternion(tf_msg.transform.rotation);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "入缝安全TF转换失败: %s", ex.what());
      return false;
    }
  }

  double min_scan_range_in_sector(
    const sensor_msgs::msg::LaserScan & scan,
    double sector_start_deg,
    double sector_end_deg) const
  {
    double tx = 0.0;
    double ty = 0.0;
    double yaw = 0.0;
    if (!resolve_scan_to_base_transform(scan, tx, ty, yaw)) {
      return std::numeric_limits<double>::infinity();
    }

    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    double best = std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
      const double r = scan.ranges[i];
      if (!std::isfinite(r)) {
        continue;
      }
      if (r < scan.range_min || r > scan.range_max) {
        continue;
      }

      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      const double x_scan = r * std::cos(angle);
      const double y_scan = r * std::sin(angle);
      const double x_base = cy * x_scan - sy * y_scan + tx;
      const double y_base = sy * x_scan + cy * y_scan + ty;
      const double dist = std::hypot(x_base, y_base);
      if (!std::isfinite(dist) || dist <= 0.0) {
        continue;
      }

      const double angle_deg = normalize_deg(std::atan2(y_base, x_base) * 180.0 / M_PI);
      if (!in_deg_range(angle_deg, sector_start_deg, sector_end_deg)) {
        continue;
      }
      best = std::min(best, dist);
    }

    return best;
  }

  static double shifted_scan_sector_deg(double sector_deg, EntryMotionMode motion_mode)
  {
    if (motion_mode != EntryMotionMode::REVERSE_ENTRY) {
      return sector_deg;
    }
    return normalize_deg(sector_deg + 180.0);
  }

  double min_motion_direction_scan_range(
    const sensor_msgs::msg::LaserScan & scan,
    double sector_start_deg,
    double sector_end_deg) const
  {
    return min_scan_range_in_sector(
      scan,
      shifted_scan_sector_deg(sector_start_deg, entry_motion_mode_),
      shifted_scan_sector_deg(sector_end_deg, entry_motion_mode_));
  }

  bool median_lateral_scan_distance_in_sector(
    const sensor_msgs::msg::LaserScan & scan,
    double sector_start_deg,
    double sector_end_deg,
    double & distance,
    std::size_t & valid_points) const
  {
    distance = std::numeric_limits<double>::infinity();
    valid_points = 0;

    double tx = 0.0;
    double ty = 0.0;
    double yaw = 0.0;
    if (!resolve_scan_to_base_transform(scan, tx, ty, yaw)) {
      return false;
    }

    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    std::vector<double> samples;
    samples.reserve(scan.ranges.size());

    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
      const double r = scan.ranges[i];
      if (!std::isfinite(r)) {
        continue;
      }
      if (r < scan.range_min || r > scan.range_max) {
        continue;
      }

      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      const double x_scan = r * std::cos(angle);
      const double y_scan = r * std::sin(angle);
      const double x_base = cy * x_scan - sy * y_scan + tx;
      const double y_base = sy * x_scan + cy * y_scan + ty;
      const double point_dist = std::hypot(x_base, y_base);
      if (!std::isfinite(point_dist) || point_dist <= 0.0) {
        continue;
      }

      const double angle_deg = normalize_deg(std::atan2(y_base, x_base) * 180.0 / M_PI);
      if (!in_deg_range(angle_deg, sector_start_deg, sector_end_deg)) {
        continue;
      }

      const double lateral_dist = std::abs(y_base);
      if (!std::isfinite(lateral_dist) || lateral_dist <= 0.0) {
        continue;
      }
      samples.push_back(lateral_dist);
    }

    valid_points = samples.size();
    if (samples.empty()) {
      return false;
    }

    std::sort(samples.begin(), samples.end());
    const std::size_t mid = samples.size() / 2;
    if (samples.size() % 2 == 0) {
      distance = 0.5 * (samples[mid - 1] + samples[mid]);
    } else {
      distance = samples[mid];
    }
    return std::isfinite(distance);
  }

  bool evaluate_entry_side_distance_hold(EntrySideHoldEval & eval)
  {
    eval = EntrySideHoldEval{};
    eval.entry_side = current_entry_side_;
    eval.target_distance = entry_side_hold_target_distance_m_;

    if (!latest_scan_ || (this->now() - latest_scan_stamp_).seconds() > max_scan_age_sec_) {
      eval.status = "NO_FRESH_SCAN";
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "entry side distance hold disabled: entry_side=%s reason=%s",
        eval.entry_side.c_str(),
        eval.status.c_str());
      return false;
    }

    (void)median_lateral_scan_distance_in_sector(
      *latest_scan_,
      enter_left_side_sector_start_deg_,
      enter_left_side_sector_end_deg_,
      eval.left_side_dist,
      eval.left_valid_points);
    (void)median_lateral_scan_distance_in_sector(
      *latest_scan_,
      enter_right_side_sector_start_deg_,
      enter_right_side_sector_end_deg_,
      eval.right_side_dist,
      eval.right_valid_points);

    const bool entry_is_right = eval.entry_side == "right";
    const std::size_t entry_points = entry_is_right ?
      eval.right_valid_points : eval.left_valid_points;
    eval.control_side_dist = entry_is_right ? eval.right_side_dist : eval.left_side_dist;
    const int required_points = std::max(1, entry_side_hold_min_valid_points_);

    if (!std::isfinite(eval.control_side_dist) ||
      entry_points < static_cast<std::size_t>(required_points))
    {
      eval.status = entry_is_right ? "INSUFFICIENT_RIGHT_POINTS" : "INSUFFICIENT_LEFT_POINTS";
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "entry side distance hold disabled: entry_side=%s reason=%s left_side_dist=%.3f "
        "right_side_dist=%.3f left_points=%zu right_points=%zu required_points=%d",
        eval.entry_side.c_str(),
        eval.status.c_str(),
        eval.left_side_dist,
        eval.right_side_dist,
        eval.left_valid_points,
        eval.right_valid_points,
        required_points);
      return false;
    }

    eval.side_error = eval.control_side_dist - eval.target_distance;
    eval.side_distance_cmd = entry_is_right ?
      -entry_side_hold_kp_ * eval.side_error :
      entry_side_hold_kp_ * eval.side_error;
    eval.active = true;
    eval.status = "ACTIVE";
    return true;
  }

  double min_ultrasonic_range() const
  {
    if (!use_ultrasonic_safety_ || ultrasonic_ranges_.empty()) {
      return std::numeric_limits<double>::infinity();
    }

    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < ultrasonic_ranges_.size(); ++i) {
      if (!std::isfinite(ultrasonic_ranges_[i])) {
        continue;
      }
      if ((this->now() - ultrasonic_stamps_[i]).seconds() > max_ultrasonic_age_sec_) {
        continue;
      }
      if (ultrasonic_ranges_[i] < best) {
        best = ultrasonic_ranges_[i];
      }
    }
    return best;
  }

  EnteringSafetyEval evaluate_entering_safety() const
  {
    EnteringSafetyEval eval;
    eval.active_side = current_entry_side_;
    eval.motion_direction =
      entry_motion_mode_ == EntryMotionMode::REVERSE_ENTRY ? "rear" : "front";

    if (use_scan_safety_) {
      if (!latest_scan_ || (this->now() - latest_scan_stamp_).seconds() > max_scan_age_sec_) {
        eval.blocked = true;
        eval.speed_scale = 0.0;
        eval.block_reason = "NO_FRESH_SCAN";
        return eval;
      }

      eval.front_min_dist = min_motion_direction_scan_range(
        *latest_scan_, enter_front_sector_start_deg_, enter_front_sector_end_deg_);
      if (current_entry_side_ == "right") {
        eval.front_side_min_dist = min_motion_direction_scan_range(
          *latest_scan_, enter_front_right_sector_start_deg_, enter_front_right_sector_end_deg_);
        eval.side_min_dist = min_motion_direction_scan_range(
          *latest_scan_, enter_right_side_sector_start_deg_, enter_right_side_sector_end_deg_);
      } else {
        eval.front_side_min_dist = min_motion_direction_scan_range(
          *latest_scan_, enter_front_left_sector_start_deg_, enter_front_left_sector_end_deg_);
        eval.side_min_dist = min_motion_direction_scan_range(
          *latest_scan_, enter_left_side_sector_start_deg_, enter_left_side_sector_end_deg_);
      }
      eval.front_left_min_dist = eval.front_side_min_dist;
      eval.left_side_min_dist = eval.side_min_dist;

      const double min_key = std::min({eval.front_min_dist, eval.front_side_min_dist, eval.side_min_dist});
      if (!std::isfinite(min_key)) {
        eval.blocked = true;
        eval.speed_scale = 0.0;
        eval.block_reason = "NO_VALID_SCAN_POINT";
        return eval;
      }

      if (min_key < enter_stop_distance_) {
        eval.blocked = true;
        eval.speed_scale = 0.0;
        if (eval.front_min_dist < enter_stop_distance_) {
          eval.block_reason =
            entry_motion_mode_ == EntryMotionMode::REVERSE_ENTRY ? "BLOCKED_REAR" : "BLOCKED_FRONT";
        } else if (eval.front_side_min_dist < enter_stop_distance_) {
          if (entry_motion_mode_ == EntryMotionMode::REVERSE_ENTRY) {
            eval.block_reason =
              current_entry_side_ == "right" ? "BLOCKED_REAR_RIGHT" : "BLOCKED_REAR_LEFT";
          } else {
            eval.block_reason =
              current_entry_side_ == "right" ? "BLOCKED_FRONT_RIGHT" : "BLOCKED_FRONT_LEFT";
          }
        } else {
          eval.block_reason = current_entry_side_ == "right" ? "BLOCKED_RIGHT_SIDE" : "BLOCKED_LEFT_SIDE";
        }
        return eval;
      }

      if (min_key < enter_slow_distance_) {
        const double denom = std::max(1e-3, enter_slow_distance_ - enter_stop_distance_);
        const double scale = (min_key - enter_stop_distance_) / denom;
        eval.speed_scale = std::clamp(scale, 0.15, 1.0);
      } else {
        eval.speed_scale = 1.0;
      }
    }

    if (use_ultrasonic_safety_) {
      const double ultrasonic_range = min_ultrasonic_range();
      if (std::isfinite(ultrasonic_range) && ultrasonic_range < entry_ultrasonic_stop_distance_) {
        eval.blocked = true;
        eval.speed_scale = 0.0;
        eval.block_reason = "BLOCKED_ULTRASONIC";
        return eval;
      }
    }

    eval.blocked = false;
    eval.block_reason = "NONE";
    return eval;
  }

  void reset_nav_route_runtime()
  {
    ++nav2_route_goal_sequence_;
    nav2_route_goal_handle_.reset();
    nav2_route_goal_in_progress_ = false;
    nav2_route_result_ready_ = false;
    nav2_route_result_success_ = false;
    nav2_route_cancel_requested_ = false;
    nav2_route_stop_hold_active_ = false;
    nav2_route_result_text_.clear();
    nav2_route_goal_sent_time_ = zero_time(get_clock());
    nav2_route_stop_hold_start_ = zero_time(get_clock());
    current_route_waypoint_index_ = 0;
    reset_target_recognition_stability();
  }

  void cancel_nav2_route_goal(const std::string & reason)
  {
    nav2_route_cancel_requested_ = true;
    if (nav2_route_goal_handle_) {
      RCLCPP_WARN(get_logger(), "取消 Nav2 巡航目标：%s", reason.c_str());
      (void)nav2_client_->async_cancel_goal(nav2_route_goal_handle_);
      nav2_route_goal_handle_.reset();
    }
    nav2_route_goal_in_progress_ = false;
  }

  bool wait_for_nav2_startup_ready(const std::string & context)
  {
    if (!nav2_startup_wait_enabled_) {
      RCLCPP_INFO(
        get_logger(),
        "[mission_manager][nav2_startup] startup wait disabled context=%s",
        context.c_str());
      return true;
    }
    if (!nav2_client_) {
      RCLCPP_ERROR(
        get_logger(),
        "[mission_manager][nav2_startup] Nav2 action client unavailable context=%s",
        context.c_str());
      return false;
    }

    const double timeout_sec = std::max(0.0, nav2_startup_wait_timeout_sec_);
    const double poll_sec = std::max(0.05, nav2_startup_wait_poll_sec_);
    const auto start_time = std::chrono::steady_clock::now();
    const auto poll_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(poll_sec));
    const auto timeout_duration = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(timeout_sec));
    const auto deadline = start_time + timeout_duration;
    auto next_log_time = start_time;

    if (nav2_client_->wait_for_action_server(std::chrono::nanoseconds(0))) {
      RCLCPP_INFO(
        get_logger(),
        "[mission_manager][nav2_startup] Nav2 action server ready context=%s waited=%.2f timeout=%.2f",
        context.c_str(),
        0.0,
        timeout_sec);
      return true;
    }

    while (rclcpp::ok()) {
      const auto now_time = std::chrono::steady_clock::now();
      if (now_time >= deadline) {
        break;
      }

      const auto remaining = std::chrono::duration_cast<std::chrono::nanoseconds>(
        deadline - now_time);
      const auto wait_duration = std::min(poll_duration, remaining);
      if (nav2_client_->wait_for_action_server(wait_duration)) {
        const double waited_sec =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
        RCLCPP_INFO(
          get_logger(),
          "[mission_manager][nav2_startup] Nav2 action server ready context=%s waited=%.2f timeout=%.2f",
          context.c_str(),
          waited_sec,
          timeout_sec);
        return true;
      }

      const auto log_time = std::chrono::steady_clock::now();
      if (log_time >= next_log_time) {
        const double waited_sec =
          std::chrono::duration<double>(log_time - start_time).count();
        RCLCPP_INFO(
          get_logger(),
          "[mission_manager][nav2_startup] waiting for Nav2 action server context=%s waited=%.2f timeout=%.2f",
          context.c_str(),
          waited_sec,
          timeout_sec);
        next_log_time = log_time + std::chrono::seconds(2);
      }
    }

    const double waited_sec =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
    RCLCPP_ERROR(
      get_logger(),
      "[mission_manager][nav2_startup] timeout waiting for Nav2 action server context=%s waited=%.2f timeout=%.2f",
      context.c_str(),
      waited_sec,
      timeout_sec);
    return false;
  }

  bool send_current_route_waypoint(std::string & fail_reason)
  {
    fail_reason.clear();
    if (current_route_waypoint_index_ >= current_route_.waypoints.size()) {
      fail_reason = "巡航路线点已耗尽";
      return false;
    }
    if (!nav2_client_) {
      fail_reason = "Nav2 action client 未初始化";
      return false;
    }
    if (!nav2_client_->wait_for_action_server(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(std::max(0.1, nav2_server_wait_timeout_sec_)))))
    {
      fail_reason = "Nav2 action server 不可用";
      return false;
    }

    const Pose2D & target_pose = current_route_.waypoints[current_route_waypoint_index_];
    NavigateToPose::Goal goal;
    goal.pose.header.stamp = this->now();
    goal.pose.header.frame_id = sanitize_frame_id(
      target_pose.frame_id.empty() ? nav2_goal_frame_ : target_pose.frame_id);
    goal.pose.pose.position.x = target_pose.x;
    goal.pose.pose.position.y = target_pose.y;
    goal.pose.pose.position.z = 0.0;
    goal.pose.pose.orientation = quaternion_from_yaw(target_pose.yaw);

    ++nav2_route_goal_sequence_;
    const auto goal_sequence = nav2_route_goal_sequence_;
    const auto waypoint_index = current_route_waypoint_index_;

    nav2_route_goal_in_progress_ = true;
    nav2_route_result_ready_ = false;
    nav2_route_result_success_ = false;
    nav2_route_cancel_requested_ = false;
    nav2_route_result_text_.clear();
    nav2_route_goal_sent_time_ = this->now();
    nav2_route_goal_handle_.reset();

    RCLCPP_INFO(
      get_logger(),
      "Nav2巡航请求 route=%s index=%zu/%zu goal[x=%.3f,y=%.3f,yaw=%.3f,frame=%s] target_side=%s",
      current_route_name_.c_str(),
      waypoint_index + 1,
      current_route_.waypoints.size(),
      target_pose.x,
      target_pose.y,
      target_pose.yaw,
      goal.pose.header.frame_id.c_str(),
      current_target_side_.c_str());

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
      [this, goal_sequence](std::shared_ptr<NavigateGoalHandle> goal_handle) {
        if (goal_sequence != nav2_route_goal_sequence_) {
          return;
        }
        if (!goal_handle) {
          nav2_route_goal_in_progress_ = false;
          nav2_route_result_ready_ = true;
          nav2_route_result_success_ = false;
          nav2_route_result_text_ = "Nav2 拒绝巡航目标";
          RCLCPP_WARN(get_logger(), "%s", nav2_route_result_text_.c_str());
          return;
        }

        nav2_route_goal_handle_ = goal_handle;
        RCLCPP_INFO(get_logger(), "Nav2巡航目标已受理");
        if (nav2_route_cancel_requested_ || nav2_route_stop_hold_active_) {
          (void)nav2_client_->async_cancel_goal(nav2_route_goal_handle_);
        }
      };
    options.result_callback =
      [this, goal_sequence](const NavigateGoalHandle::WrappedResult & result) {
        if (goal_sequence != nav2_route_goal_sequence_) {
          return;
        }

        nav2_route_goal_in_progress_ = false;
        nav2_route_goal_handle_.reset();
        if (nav2_route_stop_hold_active_) {
          return;
        }

        nav2_route_result_ready_ = true;
        nav2_route_result_success_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
        if (nav2_route_result_success_) {
          nav2_route_result_text_ = "Nav2 巡航点到达";
        } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
          nav2_route_result_text_ = "Nav2 巡航目标被取消";
        } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
          nav2_route_result_text_ = "Nav2 巡航目标被中止";
        } else {
          nav2_route_result_text_ = "Nav2 巡航目标失败";
        }
        RCLCPP_INFO(get_logger(), "%s", nav2_route_result_text_.c_str());
      };

    (void)nav2_client_->async_send_goal(goal, options);
    return true;
  }

  bool begin_nav_route_for_current_target(
    const std::string & detail,
    std::string & fail_reason,
    State nav_state = State::NAV_ROUTE)
  {
    fail_reason.clear();
    if (current_route_.waypoints.empty()) {
      fail_reason = "当前巡航路线没有waypoints";
      return false;
    }

    reset_nav_route_runtime();
    publish_entry_side();
    set_corridor_mode(false, false);
    const bool recognizer_during_nav = should_enable_recognizer_for_state(nav_state);
    request_recognizer_enable(recognizer_during_nav);
    set_distance_estimator_enabled(false, true);
    set_gap_detector_enabled(false);
    has_distance_ = false;
    target_visible_ = false;
    set_state(nav_state, detail);
    set_recognizer_topic_enabled(recognizer_during_nav, true);

    if (!send_current_route_waypoint(fail_reason)) {
      mission_active_ = false;
      publish_stop();
      request_recognizer_enable(false);
      set_distance_estimator_enabled(false, true);
      if (full_inventory_active_) {
        fail_full_inventory("启动巡航路线失败: " + fail_reason);
      } else if (single_cabinet_motion_active_) {
        fail_single_cabinet_motion("启动巡航路线失败: " + fail_reason);
      } else {
        set_state(State::ERROR, "启动巡航路线失败: " + fail_reason);
      }
      return false;
    }
    return true;
  }

  void begin_target_found_stop_hold()
  {
    if (nav2_route_stop_hold_active_) {
      return;
    }

    cancel_nav2_route_goal("目标货柜识别稳定");
    publish_stop();
    nav2_route_stop_hold_active_ = true;
    nav2_route_stop_hold_start_ = this->now();
    publish_log("目标货柜识别稳定，已取消Nav2巡航并执行短暂停车保持");
  }

  void handle_route_search_failed(const std::string & reason)
  {
    cancel_nav2_route_goal(reason);
    set_corridor_mode(false, false);
    publish_stop();
    request_recognizer_enable(false);

    if (single_cabinet_motion_active_) {
      fail_single_cabinet_motion(reason);
      return;
    }

    if (full_inventory_active_) {
      fail_full_inventory(reason);
      return;
    }

    if (route_search_failure_policy_ == "return") {
      switch_to_returning(ReturnMode::SEARCH_TARGET, reason);
      return;
    }

    mission_active_ = false;
    set_state(State::ERROR, reason);
  }

  void handle_nav_route_state()
  {
    if (target_found_pending_) {
      begin_target_found_stop_hold();
    }

    if (nav2_route_stop_hold_active_) {
      publish_stop();
      if ((this->now() - nav2_route_stop_hold_start_).seconds() >=
        std::max(0.0, nav2_cancel_stop_duration_sec_))
      {
        nav2_route_stop_hold_active_ = false;
        nav2_route_cancel_requested_ = false;
        has_distance_ = false;
        tracking_stable_start_ = zero_time(get_clock());
        if (full_inventory_active_) {
          publish_full_inventory_log(
            "recognized during nav target=" + std::to_string(current_target_cabinet_) +
            ", stop hold finished");
          begin_full_inventory_target_distance_align_after_recognition(
            current_target_cabinet_, "nav recognition stop hold finished");
        } else if (single_cabinet_motion_active_) {
          publish_single_cabinet_log("enter real target tracking cabinet=" + std::to_string(current_target_cabinet_));
          set_single_cabinet_state(State::SINGLE_CABINET_TARGET_TRACKING, "停车完成，进入目标跟踪");
        } else {
          set_state(State::TARGET_TRACKING, "停车完成，进入目标跟踪");
        }
      }
      return;
    }

    if (nav2_route_goal_in_progress_ &&
      (this->now() - nav2_route_goal_sent_time_).seconds() >
      std::max(0.1, nav2_route_waypoint_timeout_sec_))
    {
      handle_route_search_failed("Nav2巡航点超时，未识别到目标货柜");
      return;
    }

    if (!nav2_route_result_ready_) {
      return;
    }

    nav2_route_result_ready_ = false;
    if (!nav2_route_result_success_) {
      handle_route_search_failed(nav2_route_result_text_);
      return;
    }

    ++current_route_waypoint_index_;
    if (current_route_waypoint_index_ >= current_route_.waypoints.size()) {
      if (full_inventory_active_) {
        begin_full_inventory_post_route_recognition_wait();
        return;
      }
      if (single_cabinet_motion_active_) {
        begin_single_cabinet_final_recognition_wait();
        return;
      }
      handle_route_search_failed("巡航路线已走完，仍未识别到目标货柜");
      return;
    }

    std::string fail_reason;
    if (!send_current_route_waypoint(fail_reason)) {
      handle_route_search_failed("发送下一巡航点失败: " + fail_reason);
    }
  }

  void cancel_nav2_return_goal(const std::string & reason)
  {
    if (!nav2_goal_handle_) {
      return;
    }

    RCLCPP_WARN(get_logger(), "取消 Nav2 返航目标：%s", reason.c_str());
    (void)nav2_client_->async_cancel_goal(nav2_goal_handle_);
    nav2_goal_handle_.reset();
    nav2_return_in_progress_ = false;
  }

  bool resolve_return_pose(ReturnMode mode, Pose2D & target_pose, std::string & mode_text)
  {
    (void)mode;
    if (!mission_start_pose_.valid) {
      mode_text = "起始点无效";
      return false;
    }
    if (mission_start_pose_nav2_.valid) {
      target_pose = mission_start_pose_nav2_;
      mode_text = "起始点(Nav2全局)";
      return true;
    }
    target_pose = mission_start_pose_;
    mode_text = "起始点";
    return true;
  }

  bool begin_nav2_return(const Pose2D & target_pose, std::string & fail_reason)
  {
    fail_reason.clear();
    if (!nav2_client_) {
      fail_reason = "Nav2 action client 未初始化";
      return false;
    }
    if (!nav2_client_->wait_for_action_server(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(std::max(0.1, nav2_server_wait_timeout_sec_)))))
    {
      fail_reason = "Nav2 action server 不可用";
      return false;
    }

    NavigateToPose::Goal goal;
    goal.pose.header.stamp = this->now();
    goal.pose.header.frame_id = sanitize_frame_id(target_pose.frame_id);
    goal.pose.pose.position.x = target_pose.x;
    goal.pose.pose.position.y = target_pose.y;
    goal.pose.pose.position.z = 0.0;
    goal.pose.pose.orientation = quaternion_from_yaw(target_pose.yaw);

    const std::string nav_goal_frame = sanitize_frame_id(nav2_goal_frame_);
    if (!nav_goal_frame.empty() && goal.pose.header.frame_id != nav_goal_frame) {
      Pose2D converted;
      std::string tf_error;
      if (!transform_pose_2d(target_pose, nav_goal_frame, converted, tf_error)) {
        fail_reason = "无法转换返航目标到Nav2全局坐标系(" + nav_goal_frame + "): " + tf_error;
        return false;
      }
      goal.pose.header.frame_id = converted.frame_id;
      goal.pose.pose.position.x = converted.x;
      goal.pose.pose.position.y = converted.y;
      goal.pose.pose.orientation = quaternion_from_yaw(converted.yaw);
    }

    const Pose2D current = current_pose_2d();
    if (current.valid) {
      RCLCPP_INFO(
        get_logger(),
        "Nav2返航请求：current[x=%.3f,y=%.3f,yaw=%.3f,frame=%s] -> goal[x=%.3f,y=%.3f,yaw=%.3f,frame=%s]",
        current.x,
        current.y,
        current.yaw,
        current.frame_id.c_str(),
        target_pose.x,
        target_pose.y,
        target_pose.yaw,
        goal.pose.header.frame_id.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Nav2返航请求：current[无效] -> goal[x=%.3f,y=%.3f,yaw=%.3f,frame=%s]",
        target_pose.x,
        target_pose.y,
        target_pose.yaw,
        goal.pose.header.frame_id.c_str());
    }

    nav2_return_in_progress_ = true;
    nav2_result_ready_ = false;
    nav2_result_success_ = false;
    nav2_result_text_.clear();
    nav2_goal_sent_time_ = this->now();
    nav2_goal_handle_.reset();

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback =
      [this](std::shared_ptr<NavigateGoalHandle> goal_handle) {
        if (!goal_handle) {
          nav2_return_in_progress_ = false;
          nav2_result_ready_ = true;
          nav2_result_success_ = false;
          nav2_result_text_ = "Nav2 拒绝返航目标";
          RCLCPP_WARN(get_logger(), "%s", nav2_result_text_.c_str());
          return;
        }
        RCLCPP_INFO(get_logger(), "Nav2返航目标已受理");
        nav2_goal_handle_ = goal_handle;
      };
    options.result_callback =
      [this](const NavigateGoalHandle::WrappedResult & result) {
        nav2_return_in_progress_ = false;
        nav2_goal_handle_.reset();
        nav2_result_ready_ = true;
        nav2_result_success_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
        if (nav2_result_success_) {
          nav2_result_text_ = "Nav2 返航成功";
        } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
          nav2_result_text_ = "Nav2 返航被中止";
        } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
          nav2_result_text_ = "Nav2 返航被取消";
        } else {
          nav2_result_text_ = "Nav2 返航失败";
        }
        RCLCPP_INFO(get_logger(), "%s", nav2_result_text_.c_str());
      };

    (void)nav2_client_->async_send_goal(goal, options);
    return true;
  }

  void finalize_return_success()
  {
    set_corridor_mode(false, false);
    publish_stop();
    nav2_return_in_progress_ = false;
    nav2_result_ready_ = false;
    nav2_result_success_ = false;
    nav2_goal_handle_.reset();
    reset_wait_gap_runtime();
    reset_entry_gap_runtime();
    publish_gap_context();

    if (return_mode_ == ReturnMode::CANCEL_HOME) {
      mission_active_ = false;
      cancel_requested_ = false;
      return_mode_ = ReturnMode::NONE;
      const std::string detail =
        state_ == State::RETURNING_HOME ? "已返回零点，系统待机" : "取消任务后已返回目标位置";
      set_state(State::IDLE, detail);
      return;
    }

    if (return_mode_ == ReturnMode::FINISH_HOME) {
      mission_active_ = false;
      return_mode_ = ReturnMode::NONE;
      set_state(State::DONE, "任务完成并返回目标位置");
      return;
    }

    if (return_mode_ == ReturnMode::SEARCH_TARGET) {
      return_mode_ = ReturnMode::NONE;
      if (continue_on_error_) {
        finish_current_target(false);
      } else {
        mission_active_ = false;
        request_recognizer_enable(false);
        set_state(State::ERROR, "返回后仍未识别到目标货柜");
      }
    }
  }

  void fail_nav2_return(const std::string & reason)
  {
    const std::string detail = reason + "，不执行兜底返航";
    RCLCPP_ERROR(get_logger(), "%s", detail.c_str());
    publish_log(detail);
    nav2_return_in_progress_ = false;
    nav2_result_ready_ = false;
    nav2_result_success_ = false;
    nav2_result_text_.clear();
    nav2_goal_handle_.reset();
    set_corridor_mode(false, false);
    publish_stop();
    request_recognizer_enable(false);
    return_mode_ = ReturnMode::NONE;

    if (full_inventory_active_) {
      fail_full_inventory(detail);
      return;
    }

    if (single_cabinet_motion_active_) {
      fail_single_cabinet_motion(detail);
      return;
    }

    mission_active_ = false;
    set_state(State::ERROR, detail);
  }

  void switch_to_returning(ReturnMode mode, const std::string & reason)
  {
    cancel_nav2_route_goal("切换到返航流程");
    set_corridor_mode(false, false);
    publish_stop();
    request_recognizer_enable(false);

    return_mode_ = mode;
    nav2_return_in_progress_ = false;
    nav2_result_ready_ = false;
    reset_wait_gap_runtime();
    reset_entry_gap_runtime();
    publish_gap_context();

    Pose2D target_pose;
    std::string target_mode_text;
    if (!resolve_return_pose(mode, target_pose, target_mode_text)) {
      if (full_inventory_active_) {
        fail_full_inventory("无法确定返航目标：" + target_mode_text);
        return;
      }
      mission_active_ = false;
      set_state(State::ERROR, "无法确定返航目标：" + target_mode_text);
      return;
    }

    const Pose2D current = current_pose_2d();
    if (current.valid) {
      RCLCPP_INFO(
        get_logger(),
        "触发返航：mode=%d reason=%s current[x=%.3f,y=%.3f,yaw=%.3f,frame=%s] target[%s x=%.3f,y=%.3f,yaw=%.3f,frame=%s]",
        static_cast<int>(mode),
        reason.c_str(),
        current.x,
        current.y,
        current.yaw,
        current.frame_id.c_str(),
        target_mode_text.c_str(),
        target_pose.x,
        target_pose.y,
        target_pose.yaw,
        target_pose.frame_id.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(),
        "触发返航：mode=%d reason=%s current[无效] target[%s x=%.3f,y=%.3f,yaw=%.3f,frame=%s]",
        static_cast<int>(mode),
        reason.c_str(),
        target_mode_text.c_str(),
        target_pose.x,
        target_pose.y,
        target_pose.yaw,
        target_pose.frame_id.c_str());
    }

    std::string nav2_fail_reason;
    if (!begin_nav2_return(target_pose, nav2_fail_reason)) {
      fail_nav2_return("Nav2 返航目标发送失败: " + nav2_fail_reason);
      return;
    }

    std::string detail = reason + "，返航目标=" + target_mode_text + "，方式=Nav2";
    set_state(State::RETURNING, detail);
  }

  void finish_current_target(bool success)
  {
    const std::string current = targets_.at(current_target_index_);
    if (success) {
      publish_log("目标完成: " + current);
    } else {
      publish_log("目标失败: " + current);
    }

    if (!success && !continue_on_error_) {
      mission_active_ = false;
      request_recognizer_enable(false);
      set_corridor_mode(false, false);
      publish_stop();
      set_state(State::ERROR, "目标失败，任务终止");
      return;
    }

    if (current_target_index_ + 1 < targets_.size()) {
      ++current_target_index_;
      std::string reason;
      if (!prepare_current_target(reason)) {
        mission_active_ = false;
        request_recognizer_enable(false);
        set_corridor_mode(false, false);
        publish_stop();
        set_state(State::ERROR, reason);
        return;
      }

      if (!resolve_current_route(reason)) {
        mission_active_ = false;
        request_recognizer_enable(false);
        set_corridor_mode(false, false);
        publish_stop();
        set_state(State::ERROR, reason);
        return;
      }
      if (!configure_current_entry_profile(reason)) {
        mission_active_ = false;
        request_recognizer_enable(false);
        set_corridor_mode(false, false);
        publish_stop();
        set_state(State::ERROR, reason);
        return;
      }
      if (!resolve_current_gap_plan(reason)) {
        mission_active_ = false;
        request_recognizer_enable(false);
        set_corridor_mode(false, false);
        publish_stop();
        set_state(State::ERROR, reason);
        return;
      }
      log_current_target_entry_plan();

      target_visible_ = false;
      has_distance_ = false;
      tracking_stable_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      reset_wait_gap_runtime();
      reset_entry_gap_runtime();
      publish_gap_context();
      reset_segment_distance();
      if (plc_http_enabled_) {
        mission_active_ = false;
        request_recognizer_enable(false);
        set_corridor_mode(false, false);
        publish_stop();
        set_state(
          State::ERROR,
          "legacy targets_ 多目标切换入口本轮未接入 PLC open/wait，禁止在 plc_http_enabled=true 时直接移动");
        return;
      }
      std::string nav_route_fail_reason;
      if (!begin_nav_route_for_current_target("切换下一个目标，开始Nav2巡航路线识别", nav_route_fail_reason)) {
        set_state(State::ERROR, nav_route_fail_reason);
      }
      return;
    }

    if (success) {
      (void)start_finish_action("全部目标完成，执行完成后返航模式", mission_force_map_origin_on_finish_);
    } else {
      mission_active_ = false;
      request_recognizer_enable(false);
      set_corridor_mode(false, false);
      publish_stop();
      set_state(State::ERROR, "存在失败目标且无后续目标");
    }
  }

  void set_flow_state(State s, const std::string & detail)
  {
    state_enter_time_ = this->now();
    set_recognizer_topic_enabled(should_enable_recognizer_for_state(s));
    state_ = s;
    publish_state_text(state_to_string(state_));
    publish_log("[" + state_to_string(state_) + "] [mission_manager][inventory_flow] " + detail);
  }

  void set_single_cabinet_state(State s, const std::string & detail)
  {
    state_enter_time_ = this->now();
    set_state(s, "[mission_manager][single_cabinet] " + detail);
  }

  bool plc_continue_without_plc() const
  {
    return normalize_plc_fail_policy(plc_fail_policy_) == "continue_without_plc";
  }

  bool is_plc_supported_cabinet(int cabinet_id) const
  {
    return std::find(
      plc_supported_cabinets_.begin(),
      plc_supported_cabinets_.end(),
      cabinet_id) != plc_supported_cabinets_.end();
  }

  bool validate_plc_open_config(std::string & reason) const
  {
    reason.clear();
    if (agv_inventory_system::trim(plc_server_url_).empty()) {
      reason = "plc_server_url 为空";
      return false;
    }
    if (plc_server_url_.find("<PLC_GATEWAY_HOST>") != std::string::npos) {
      reason = "plc_server_url 仍包含 <PLC_GATEWAY_HOST> 占位，未配置小车端 PLC Flask 网关地址";
      return false;
    }
    if (agv_inventory_system::trim(plc_open_endpoint_).empty()) {
      reason = "plc_open_endpoint 为空";
      return false;
    }
    if (agv_inventory_system::trim(plc_open_query_param_).empty()) {
      RCLCPP_WARN(
        get_logger(),
        "[mission_manager][PLC] plc_open_query_param 为空，web_api_client 将默认使用 shelfId");
    }
    return true;
  }

  void fail_plc_open(
    int cabinet_id,
    const std::string & context,
    const std::string & reason)
  {
    const std::string message =
      "PLC open failed target_cabinet=" + std::to_string(cabinet_id) +
      " context=" + context +
      " reason=" + reason +
      " fail_policy=" + normalize_plc_fail_policy(plc_fail_policy_);
    publish_plc_log("ERROR " + message);
    if (full_inventory_active_) {
      fail_full_inventory(message);
    } else if (single_cabinet_motion_active_) {
      fail_single_cabinet_motion(message);
    } else {
      mission_error_reason_ = message;
      set_state(State::ERROR, message);
    }
  }

  bool request_plc_open_for_cabinet(int cabinet_id, const std::string & context)
  {
    plc_open_wait_required_ = false;
    if (!plc_http_enabled_) {
      return true;
    }

    const std::string fail_policy = normalize_plc_fail_policy(plc_fail_policy_);
    publish_plc_log(
      "open start mode=" + current_plc_task_mode() +
      " state=" + state_to_string(state_) +
      " context=" + context +
      " target_cabinet=" + std::to_string(cabinet_id) +
      " query_param=" + plc_open_query_param_ +
      " server_url=" + plc_server_url_ +
      " endpoint=" + plc_open_endpoint_ +
      " verify_tls=" + std::string(plc_verify_tls_ ? "true" : "false") +
      " require_body_success=" + std::string(plc_require_body_success_ ? "true" : "false") +
      " timeout=" + format_seconds(plc_request_timeout_sec_) +
      " retry_count=" + std::to_string(plc_retry_count_) +
      " fail_policy=" + fail_policy);

    std::string config_reason;
    if (!validate_plc_open_config(config_reason)) {
      fail_plc_open(cabinet_id, context, "配置错误: " + config_reason);
      return false;
    }

    if (!is_plc_supported_cabinet(cabinet_id)) {
      const std::string reason =
        "目标柜号暂未配置 PLC 控制，不做 19-36 到 1-18 映射 cabinet=" +
        std::to_string(cabinet_id) +
        " supported=" + cabinet_unit_to_string(plc_supported_cabinets_);
      if (plc_continue_without_plc()) {
        RCLCPP_WARN(get_logger(), "[mission_manager][PLC] %s", reason.c_str());
        publish_plc_log("WARNING " + reason + "，continue_without_plc 后继续原流程");
        return true;
      }
      fail_plc_open(cabinet_id, context, reason);
      return false;
    }

    if (web_api_client_.requestOpenCabinet(cabinet_id)) {
      plc_open_wait_required_ = true;
      publish_plc_log(
        "open request success target_cabinet=" + std::to_string(cabinet_id) +
        " HTTP 200 only means request-layer delivered; response body is log-only");
      return true;
    }

    const std::string reason =
      "/open 请求层失败: HTTP 非 200、连接失败或超时 target_cabinet=" +
      std::to_string(cabinet_id);
    if (plc_continue_without_plc()) {
      RCLCPP_WARN(get_logger(), "[mission_manager][PLC] %s", reason.c_str());
      publish_plc_log("WARNING " + reason + "，continue_without_plc 后继续原流程");
      return true;
    }

    fail_plc_open(cabinet_id, context, reason);
    return false;
  }

  bool begin_plc_open_wait_for_target(
    int cabinet_id,
    PlcOpenContinuation continuation,
    const std::string & context)
  {
    plc_open_wait_target_cabinet_ = cabinet_id;
    plc_open_wait_continuation_ = continuation;
    plc_open_wait_context_ = context;

    if (!plc_http_enabled_) {
      clear_plc_open_wait_context();
      publish_plc_log(
        "skip open/wait plc_http_enabled=false mode=" + current_plc_task_mode() +
        " target_cabinet=" + std::to_string(cabinet_id) +
        " continuation=" + plc_open_continuation_to_string(continuation));
      return true;
    }

    publish_stop();
    if (!request_plc_open_for_cabinet(cabinet_id, context)) {
      return false;
    }

    if (!plc_open_wait_required_) {
      publish_plc_log(
        "skip fixed open wait because PLC open was bypassed by fail policy target_cabinet=" +
        std::to_string(cabinet_id) +
        " continuation=" + plc_open_continuation_to_string(continuation));
      handle_plc_open_wait_done(false);
      return true;
    }

    publish_plc_log(
      "enter fixed open wait target_cabinet=" + std::to_string(cabinet_id) +
      " wait_sec=" + format_seconds(plc_open_wait_sec_) +
      " current_state=" + state_to_string(state_) +
      " continuation=" + plc_open_continuation_to_string(continuation));
    set_flow_state(
      State::WAIT_OPEN_READY,
      "[PLC] 固定等待 open 完成，不接收 PLC 状态反馈 target_cabinet=" +
      std::to_string(cabinet_id) +
      " wait=" + format_seconds(plc_open_wait_sec_) +
      " continuation=" + plc_open_continuation_to_string(continuation));
    return true;
  }

  void clear_plc_open_wait_context()
  {
    plc_open_wait_continuation_ = PlcOpenContinuation::NONE;
    plc_open_wait_target_cabinet_ = -1;
    plc_open_wait_required_ = false;
    plc_open_wait_context_.clear();
  }

  std::string current_plc_task_mode() const
  {
    if (full_inventory_active_) {
      return "full";
    }
    if (single_cabinet_motion_active_) {
      return "single";
    }
    if (mission_active_) {
      return "legacy";
    }
    return "idle";
  }

  void handle_plc_open_wait_done(bool waited = true)
  {
    const PlcOpenContinuation continuation = plc_open_wait_continuation_;
    const int cabinet_id = plc_open_wait_target_cabinet_;
    const std::string context = plc_open_wait_context_;
    clear_plc_open_wait_context();

    publish_plc_log(
      std::string(waited ? "fixed open wait done" : "PLC open bypass continuation") +
      " target_cabinet=" + std::to_string(cabinet_id) +
      " waited_sec=" + format_seconds(waited ? plc_open_wait_sec_ : 0.0) +
      " previous_context=" + context +
      " next=" + plc_open_continuation_to_string(continuation));

    switch (continuation) {
      case PlcOpenContinuation::SINGLE_CABINET_PREPARE_NAV:
        set_single_cabinet_state(
          State::SINGLE_CABINET_PREPARE_NAV,
          "PLC open 固定等待结束，prepare nav target_cabinet=" + std::to_string(cabinet_id));
        return;
      case PlcOpenContinuation::FULL_INVENTORY_START_ROUTE:
        (void)start_full_inventory_target_route("PLC open wait done, start target route");
        return;
      case PlcOpenContinuation::FULL_INVENTORY_SAME_SIDE_NEXT_SEARCH:
        start_full_inventory_same_side_next_search();
        return;
      case PlcOpenContinuation::FULL_INVENTORY_REAR_TARGET_REORIENT:
        publish_plc_log(
          "rear target PLC continuation target_cabinet=" + std::to_string(cabinet_id) +
          " next=FULL_INVENTORY_REAR_TARGET_REORIENT");
        start_full_inventory_rear_target_reorient();
        return;
      case PlcOpenContinuation::FULL_INVENTORY_ADVANCE_ROUTE:
        (void)start_full_inventory_target_route("PLC open wait done, advance next target");
        return;
      case PlcOpenContinuation::FULL_INVENTORY_BETWEEN_SIDE_ROUTE:
        (void)start_full_inventory_target_route("PLC open wait done, between-side auto charge finished");
        return;
      case PlcOpenContinuation::NONE:
      default:
        publish_plc_log("WARNING WAIT_OPEN_READY done with no PLC continuation; falling back to single cabinet prepare if active");
        if (single_cabinet_motion_active_) {
          const int target_cabinet = active_single_cabinet_scan_cabinet();
          set_single_cabinet_state(
            State::SINGLE_CABINET_PREPARE_NAV,
            "open wait done, prepare nav target_cabinet=" + std::to_string(target_cabinet));
        }
        return;
    }
  }

  bool find_configured_gap_cabinets(
    const std::string & gap_id,
    std::vector<int> & cabinets) const
  {
    cabinets.clear();
    const auto plan_it = inventory_plan_by_gap_id_.find(gap_id);
    if (plan_it != inventory_plan_by_gap_id_.end()) {
      cabinets = plan_it->second;
      return true;
    }

    const auto map_it = gap_scan_map_cabinets_by_gap_id_.find(gap_id);
    if (map_it != gap_scan_map_cabinets_by_gap_id_.end()) {
      cabinets = map_it->second;
      return true;
    }

    return false;
  }

  bool validate_inventory_gap_plan(const InventoryGapPlan & plan, std::string & reason) const
  {
    reason.clear();
    if (agv_inventory_system::trim(plan.gap_id).empty()) {
      reason = "gap_id 为空";
      return false;
    }
    if (plan.scan_cabinets.empty()) {
      reason = "scan_cabinets 为空: gap=" + plan.gap_id;
      return false;
    }
    for (const auto cabinet_id : plan.scan_cabinets) {
      if (cabinet_id <= 0) {
        reason =
          "柜号必须大于 0: gap=" + plan.gap_id +
          " cabinet=" + std::to_string(cabinet_id);
        return false;
      }
    }
    return true;
  }

  void reset_single_cabinet_side_row_context()
  {
    single_cabinet_side_row_active_ = false;
    single_cabinet_side_row_full_sequence_ = false;
    single_cabinet_side_row_requested_sequence_.clear();
    single_cabinet_side_row_phase_ = SingleCabinetSideRowPhase::NONE;
    single_cabinet_after_exit_action_ = SingleCabinetAfterExitAction::NONE;
    single_cabinet_active_gap_id_.clear();
    single_cabinet_current_scan_cabinet_ = -1;
    single_cabinet_adjusted_scan_cabinet_ = -1;
    single_cabinet_next_gap_target_cabinet_ = -1;
    single_cabinet_last_entering_straight_distance_ = 0.0;
    single_cabinet_exit_target_distance_ = single_cabinet_exit_distance_m_;
    single_cabinet_exit_effective_timeout_sec_ = single_cabinet_exit_timeout_sec_;
    single_cabinet_exit_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    single_cabinet_exit_phase_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    single_cabinet_exit_phase_ = SingleCabinetExitPhase::STRAIGHT_REVERSE;
    reset_single_cabinet_scan_runtime();
  }

  void reset_full_inventory_context()
  {
    full_inventory_active_ = false;
    full_inventory_index_ = 0;
    full_inventory_current_target_ = -1;
    full_inventory_next_target_ = -1;
    full_inventory_current_side_.clear();
    full_inventory_current_route_.clear();
    reset_between_side_auto_charge_runtime();
    full_inventory_same_side_search_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    full_inventory_final_recognition_wait_start_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
    full_inventory_recognition_fallback_start_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
    full_inventory_recognition_fallback_phase_start_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
    full_inventory_recognition_fallback_phase_ = FullInventoryRecognitionFallbackPhase::IDLE;
    full_inventory_recognition_fallback_index_ = 0;
    full_inventory_post_gap_advance_start_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
    clear_full_inventory_rear_target_context("reset_full_inventory_context");
    clear_plc_open_wait_context();
  }

  void clear_full_inventory_rear_target_context(
    const std::string & reason,
    bool keep_last_exit_entry_yaw = false)
  {
    const bool had_context =
      full_inventory_rear_target_pending_ ||
      full_inventory_rear_target_active_ ||
      full_inventory_same_side_heading_override_valid_ ||
      full_inventory_gap_search_direction_override_valid_ ||
      full_inventory_rear_target_backup_started_;
    if (had_context) {
      publish_full_inventory_log("[rear_target] clear rear target override reason=" + reason);
    }
    full_inventory_rear_target_pending_ = false;
    full_inventory_rear_target_active_ = false;
    full_inventory_rear_target_finished_cabinet_ = -1;
    full_inventory_rear_target_next_cabinet_ = -1;
    full_inventory_rear_target_yaw_ = 0.0;
    full_inventory_rear_target_original_gap_direction_ = SearchDirection::FORWARD;
    if (!keep_last_exit_entry_yaw) {
      full_inventory_last_exit_entry_yaw_valid_ = false;
      full_inventory_last_exit_entry_yaw_ = 0.0;
    }
    full_inventory_same_side_heading_override_valid_ = false;
    full_inventory_same_side_heading_override_yaw_rad_ = 0.0;
    full_inventory_gap_search_direction_override_valid_ = false;
    full_inventory_gap_search_direction_override_ = SearchDirection::FORWARD;
    full_inventory_rear_target_turn_start_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
    full_inventory_rear_target_backup_started_ = false;
    full_inventory_rear_target_backup_start_pose_ = Pose2D{};
    full_inventory_rear_target_backup_fixed_y_ = 0.0;
    full_inventory_rear_target_backup_start_time_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }

  SearchDirection effective_gap_search_direction() const
  {
    return full_inventory_gap_search_direction_override_valid_ ?
           full_inventory_gap_search_direction_override_ :
           current_gap_plan_.search_direction;
  }

  void reset_between_side_auto_charge_runtime()
  {
    between_side_auto_charge_active_ = false;
    between_side_auto_charge_cancel_sent_ = false;
    between_side_auto_charge_cancel_response_ready_ = false;
    between_side_auto_charge_cancel_response_success_ = false;
    between_side_auto_charge_cancel_response_message_.clear();
    between_side_auto_charge_fallback_used_ = false;
    between_side_auto_charge_target_index_ = 0;
    between_side_auto_charge_target_ = -1;
    between_side_auto_charge_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    between_side_auto_charge_ready_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    between_side_auto_charge_cancel_request_time_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }

  bool apply_full_inventory_route_overrides(std::string & reason)
  {
    reason.clear();
    if (!full_inventory_left_route_.empty()) {
      if (route_configs_.find(full_inventory_left_route_) == route_configs_.end()) {
        reason = "full_inventory_left_route 不存在: " + full_inventory_left_route_;
        return false;
      }
      side_route_map_["left"] = full_inventory_left_route_;
    }
    if (!full_inventory_right_route_.empty()) {
      if (route_configs_.find(full_inventory_right_route_) == route_configs_.end()) {
        reason = "full_inventory_right_route 不存在: " + full_inventory_right_route_;
        return false;
      }
      side_route_map_["right"] = full_inventory_right_route_;
    }
    return true;
  }

  bool get_cabinet_side(int cabinet_id, std::string & side, std::string & reason) const
  {
    reason.clear();
    const auto side_it = cabinet_side_map_.find(cabinet_id);
    if (side_it == cabinet_side_map_.end()) {
      reason = "货柜未配置 cabinet_side_map: " + std::to_string(cabinet_id);
      return false;
    }
    side = normalize_entry_side(side_it->second);
    return true;
  }

  bool get_cabinet_entry_side(int cabinet_id, std::string & side, std::string & reason) const
  {
    reason.clear();
    const auto side_it = cabinet_entry_side_map_.find(cabinet_id);
    if (side_it == cabinet_entry_side_map_.end()) {
      reason = "货柜未配置 cabinet_entry_side_map: " + std::to_string(cabinet_id);
      return false;
    }
    side = normalize_entry_side(side_it->second);
    return true;
  }

  bool get_cabinet_gap_search_direction(
    int cabinet_id,
    SearchDirection & direction,
    std::string & reason) const
  {
    reason.clear();
    const auto direction_it = cabinet_gap_search_direction_map_.find(cabinet_id);
    if (direction_it == cabinet_gap_search_direction_map_.end()) {
      reason = "货柜未配置 cabinet_gap_search_direction_map: " + std::to_string(cabinet_id);
      return false;
    }
    direction = direction_it->second;
    return true;
  }

  bool get_route_for_side(const std::string & side, std::string & route_name, std::string & reason) const
  {
    reason.clear();
    const std::string normalized_side = normalize_entry_side(side);
    const auto route_it = side_route_map_.find(normalized_side);
    if (route_it == side_route_map_.end()) {
      reason = "side_route_map 未配置侧路线: " + normalized_side;
      return false;
    }
    if (route_configs_.find(route_it->second) == route_configs_.end()) {
      reason = "side_route_map 引用的路线不存在: " + route_it->second;
      return false;
    }
    route_name = route_it->second;
    return true;
  }

  bool validate_full_inventory_sequence(
    const std::vector<int> & sequence,
    std::string & reason) const
  {
    reason.clear();
    if (sequence.empty()) {
      reason = "full_inventory_sequence 为空";
      return false;
    }
    for (const auto cabinet_id : sequence) {
      if (cabinet_id <= 0) {
        reason = "full_inventory_sequence 中货柜号必须大于0: " + std::to_string(cabinet_id);
        return false;
      }
    }
    if (scan_layers_ <= 0 || scan_depth_count_ <= 0) {
      reason =
        "scan_layers/scan_depth_count 必须为正数: layers=" +
        std::to_string(scan_layers_) +
        " depth_count=" + std::to_string(scan_depth_count_);
      return false;
    }
    return true;
  }

  std::vector<int> full_inventory_sequence_from_request(
    const agv_inventory_system::srv::StartMission::Request & request) const
  {
    std::vector<int> sequence;
    for (const auto cabinet_id : request.scan_cabinets) {
      sequence.push_back(static_cast<int>(cabinet_id));
    }
    if (!sequence.empty()) {
      return sequence;
    }
    for (const auto & target_code : request.targets) {
      int cabinet_id = -1;
      if (parse_target_cabinet(target_code, cabinet_id)) {
        sequence.push_back(cabinet_id);
      }
    }
    if (!sequence.empty()) {
      return sequence;
    }
    return full_inventory_sequence_;
  }

  bool should_start_full_inventory(
    const agv_inventory_system::srv::StartMission::Request & request) const
  {
    if (!full_inventory_enabled_) {
      return false;
    }
    if (request.run_full_inventory) {
      return true;
    }
    const bool gap_empty = agv_inventory_system::trim(request.target_gap).empty();
    return gap_empty && (request.scan_cabinets.size() > 1U || request.targets.size() > 1U);
  }

  void record_full_inventory_start_pose()
  {
    mission_start_distance_ = odom_cumulative_distance_;
    reset_segment_distance();
    mission_start_pose_ = current_pose_2d();
    mission_start_pose_nav2_ = Pose2D{};
    if (!mission_start_pose_.valid) {
      RCLCPP_WARN(get_logger(), "[FULL_INVENTORY] 启动时未获取到有效里程计位姿，返航将无法使用起点模式");
      return;
    }

    publish_full_inventory_log(
      "record start pose x=" + std::to_string(mission_start_pose_.x) +
      " y=" + std::to_string(mission_start_pose_.y) +
      " yaw=" + std::to_string(mission_start_pose_.yaw) +
      " frame=" + mission_start_pose_.frame_id);

    std::string tf_error;
    if (transform_pose_2d(mission_start_pose_, nav2_goal_frame_, mission_start_pose_nav2_, tf_error)) {
      publish_full_inventory_log(
        "record Nav2 start pose x=" + std::to_string(mission_start_pose_nav2_.x) +
        " y=" + std::to_string(mission_start_pose_nav2_.y) +
        " yaw=" + std::to_string(mission_start_pose_nav2_.yaw) +
        " frame=" + mission_start_pose_nav2_.frame_id);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "[FULL_INVENTORY] 记录Nav2全局起点失败，将回退到里程计起点：%s",
        tf_error.c_str());
    }
  }

  bool prepare_full_inventory_target(int cabinet_id, std::string & reason)
  {
    reason.clear();
    if (!routes_loaded_) {
      routes_loaded_ = load_route_config(reason);
      if (!routes_loaded_) {
        return false;
      }
    }
    if (!apply_full_inventory_route_overrides(reason)) {
      return false;
    }
    if (!warehouse_layout_loaded_) {
      warehouse_layout_loaded_ = load_warehouse_layout_config(reason);
      if (!warehouse_layout_loaded_) {
        return false;
      }
    }

    SearchDirection search_direction{SearchDirection::FORWARD};
    if (!get_cabinet_gap_search_direction(cabinet_id, search_direction, reason)) {
      return false;
    }
    (void)search_direction;

    targets_ = {std::to_string(cabinet_id)};
    current_target_index_ = 0;
    if (!prepare_current_target(reason)) {
      return false;
    }
    if (!resolve_current_route(reason)) {
      return false;
    }
    if (!configure_current_entry_profile(reason)) {
      return false;
    }
    if (!resolve_current_gap_plan(reason)) {
      return false;
    }

    full_inventory_current_target_ = cabinet_id;
    full_inventory_current_side_ = current_target_side_;
    full_inventory_current_route_ = current_route_name_;
    log_current_target_entry_plan();
    publish_entry_side();
    request_recognizer_enable(false);
    set_recognizer_topic_enabled(false, true);
    set_distance_estimator_enabled(false, true);
    set_gap_detector_enabled(false);
    publish_full_inventory_log(
      "current_target=" + std::to_string(full_inventory_current_target_) +
      " index=" + std::to_string(full_inventory_index_) +
      "/" + std::to_string(full_inventory_sequence_.size()) +
      " side=" + full_inventory_current_side_ +
      " route=" + full_inventory_current_route_ +
      " entry_side=" + current_entry_side_ +
      " search_direction=" + search_direction_to_string(current_gap_plan_.search_direction));
    return true;
  }

  bool start_full_inventory_target_route(const std::string & context)
  {
    clear_full_inventory_rear_target_context("start_full_inventory_target_route");
    mission_active_ = true;
    cancel_requested_ = false;
    target_visible_ = false;
    has_distance_ = false;
    latest_distance_ = 0.0;
    return_mode_ = ReturnMode::NONE;
    nav2_return_in_progress_ = false;
    nav2_result_ready_ = false;
    reset_wait_gap_runtime();
    reset_entry_gap_runtime();
    publish_gap_context();
    tracking_stable_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_target_seen_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    reset_segment_distance();
    request_recognizer_enable(false);
    set_recognizer_topic_enabled(false, true);
    set_distance_estimator_enabled(false, true);
    set_gap_detector_enabled(false);
    publish_full_inventory_log(
      "nav observe stage: recognizer=" +
      std::string(full_inventory_recognize_during_nav_ ? "on" : "off") +
      " distance=off gap=off target=" + std::to_string(current_target_cabinet_));

    std::string fail_reason;
    if (!begin_nav_route_for_current_target(
        "[FULL_INVENTORY] " + context +
        " current_target=" + std::to_string(current_target_cabinet_) +
        " side=" + current_target_side_ +
        " route=" + current_route_name_,
        fail_reason,
        State::FULL_INVENTORY_NAV_TO_OBSERVE))
    {
      fail_full_inventory("启动目标侧路径失败: " + fail_reason);
      return false;
    }
    return true;
  }

  bool start_full_inventory_sequence(
    const std::vector<int> & sequence,
    bool force_map_origin_on_finish,
    std::string & reason)
  {
    reason.clear();
    if (!validate_full_inventory_sequence(sequence, reason)) {
      return false;
    }

    full_inventory_sequence_ = sequence;
    clear_safe_exit_gap_recovery_context();
    full_inventory_active_ = true;
    full_inventory_index_ = 0;
    full_inventory_current_target_ = -1;
    full_inventory_next_target_ =
      full_inventory_sequence_.size() > 1U ? full_inventory_sequence_[1] : -1;
    mission_force_map_origin_on_finish_ = force_map_origin_on_finish;
    reset_between_side_auto_charge_runtime();
    clear_full_inventory_rear_target_context("start_full_inventory_sequence");
    full_inventory_same_side_search_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    full_inventory_final_recognition_wait_start_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
    full_inventory_recognition_fallback_start_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
    full_inventory_recognition_fallback_phase_start_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
    full_inventory_recognition_fallback_phase_ = FullInventoryRecognitionFallbackPhase::IDLE;
    full_inventory_recognition_fallback_index_ = 0;
    full_inventory_post_gap_advance_start_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());

    gap_request_queue_.clear();
    current_gap_request_index_ = 0;
    mission_error_reason_.clear();
    inventory_flow_active_ = true;
    single_cabinet_motion_active_ = false;
    single_cabinet_gap_searching_ = false;
    single_cabinet_target_recognized_logged_ = false;
    single_cabinet_close_requested_ = false;
    reset_single_cabinet_side_row_context();
    reset_single_cabinet_scan_runtime();
    record_full_inventory_start_pose();

    publish_full_inventory_log(
      "START sequence=" + cabinet_unit_to_string(full_inventory_sequence_));
    set_flow_state(
      State::FULL_INVENTORY_PREPARE_TARGET,
      "[FULL_INVENTORY] prepare first target");
    request_recognizer_enable(false);
    set_recognizer_topic_enabled(false, true);
    set_distance_estimator_enabled(false, true);
    set_gap_detector_enabled(false);

    if (!prepare_full_inventory_target(full_inventory_sequence_[full_inventory_index_], reason)) {
      reset_full_inventory_context();
      inventory_flow_active_ = false;
      return false;
    }
    if (!wait_for_nav2_startup_ready("FULL_INVENTORY")) {
      reason = "等待 Nav2 启动完成超时: Nav2 action server 不可用";
      fail_full_inventory(reason);
      return false;
    }
    if (!begin_plc_open_wait_for_target(
        current_target_cabinet_,
        PlcOpenContinuation::FULL_INVENTORY_START_ROUTE,
        "FULL_INVENTORY first target before route"))
    {
      reason = mission_error_reason_.empty() ? "PLC open 失败" : mission_error_reason_;
      return false;
    }
    if (plc_http_enabled_) {
      return true;
    }
    return start_full_inventory_target_route("start target route");
  }

  void fail_full_inventory(const std::string & reason)
  {
    publish_full_inventory_log("FAILED: " + reason);
    cancel_nav2_route_goal("全部盘库失败");
    cancel_nav2_return_goal("全部盘库失败");
    set_corridor_mode(false, false);
    publish_stop();
    request_recognizer_enable(false);
    set_recognizer_topic_enabled(false, true);
    set_distance_estimator_enabled(false, true);
    set_gap_detector_enabled(false);
    clear_inventory_upload_batch("error");
    reset_single_cabinet_scan_runtime();
    reset_between_side_auto_charge_runtime();
    clear_full_inventory_rear_target_context("fail_full_inventory");
    mission_error_reason_ = reason;
    set_flow_state(State::ERROR, "[FULL_INVENTORY] " + reason);
  }

  void begin_full_inventory_post_route_recognition_wait()
  {
    cancel_nav2_route_goal("全部盘库巡航路线已走完，进入停车识别等待");
    nav2_route_goal_in_progress_ = false;
    nav2_route_result_ready_ = false;
    nav2_route_stop_hold_active_ = false;
    nav2_route_cancel_requested_ = false;
    target_found_pending_ = false;
    set_corridor_mode(false, false);
    publish_stop();
    request_recognizer_enable(true);
    set_recognizer_topic_enabled(true, true);
    set_distance_estimator_enabled(false, true);
    set_gap_detector_enabled(false);
    reset_target_recognition_stability();
    has_distance_ = false;
    target_visible_ = false;
    full_inventory_final_recognition_wait_start_ = this->now();
    publish_full_inventory_log(
      "route finished, start recognition wait target=" +
      std::to_string(current_target_cabinet_) +
      " timeout=" + format_seconds(full_inventory_final_recognition_wait_sec_));
    set_state(
      State::FULL_INVENTORY_POST_ROUTE_RECOGNITION_WAIT,
      "[FULL_INVENTORY] route finished, start recognition wait target=" +
      std::to_string(current_target_cabinet_) +
      " timeout=" + format_seconds(full_inventory_final_recognition_wait_sec_));
  }

  void begin_full_inventory_target_distance_align_after_recognition(
    int rec_id,
    const std::string & context)
  {
    const bool from_post_route_wait =
      state_ == State::FULL_INVENTORY_POST_ROUTE_RECOGNITION_WAIT;
    const bool from_same_side_search =
      state_ == State::FULL_INVENTORY_SAME_SIDE_NEXT_SEARCH;
    cancel_nav2_route_goal("全部盘库稳定识别到目标货柜");
    nav2_route_stop_hold_active_ = false;
    nav2_route_cancel_requested_ = false;
    target_found_pending_ = false;
    set_corridor_mode(false, false);
    publish_stop();
    request_recognizer_enable(false);
    set_recognizer_topic_enabled(false, true);
    set_gap_detector_enabled(false);
    publish_target_lidar_side();
    set_distance_estimator_enabled(true, true);
    has_distance_ = false;
    latest_distance_ = 0.0;
    full_inventory_final_recognition_wait_start_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
    full_inventory_recognition_fallback_start_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
    full_inventory_recognition_fallback_phase_start_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
    full_inventory_recognition_fallback_phase_ = FullInventoryRecognitionFallbackPhase::IDLE;
    tracking_stable_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    target_distance_gap_open_count_ = 0;
    if (from_post_route_wait) {
      publish_full_inventory_log(
        "post-route recognition wait success target=" + std::to_string(rec_id));
    }
    if (from_same_side_search) {
      publish_full_inventory_log(
        "same-side next target recognized=" + std::to_string(rec_id) +
        ", recognizer=off, distance=on");
    }
    publish_full_inventory_log(
      "recognized target=" + std::to_string(rec_id) +
      ", recognizer=off, distance=on");
    set_state(
      State::FULL_INVENTORY_TARGET_DISTANCE_ALIGN,
      "[FULL_INVENTORY] recognized target cabinet=" + std::to_string(rec_id) +
      " stable, enter target distance align: " + context);
  }

  bool handle_full_inventory_recognition(int rec_id)
  {
    if (!is_full_inventory_recognition_state(state_)) {
      return false;
    }

    if (full_inventory_same_side_recognition_delayed()) {
      reset_target_recognition_stability();
      const double progress = segment_distance();
      const double delay_distance = full_inventory_same_side_recognition_delay_distance();
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "[mission_manager][FULL_INVENTORY][recognition] delayed ignore state=%s recognized=%d "
        "target=%d progress=%.2f/%.2f",
        state_to_string(state_).c_str(),
        rec_id,
        current_target_cabinet_,
        progress,
        delay_distance);
      return true;
    }

    const bool matched = rec_id == current_target_cabinet_;
    if (!matched) {
      reset_target_recognition_stability();
      RCLCPP_INFO(
        get_logger(),
        "[mission_manager][FULL_INVENTORY][recognition] mismatch recognized=%d target=%d",
        rec_id,
        current_target_cabinet_);
      return true;
    }

    last_target_seen_time_ = this->now();
    target_visible_ = true;

    const bool stable_ready = update_target_recognition_stability(rec_id);
    const int required_count = std::max(1, target_recognition_stable_frames_);
    RCLCPP_INFO(
      get_logger(),
      "[mission_manager][FULL_INVENTORY][recognition] state=%s target=%d recognized=%d "
      "stable_count=%d required=%d ready=%s",
      state_to_string(state_).c_str(),
      current_target_cabinet_,
      rec_id,
      target_recognition_stable_count_,
      required_count,
      stable_ready ? "true" : "false");

    if (stable_ready) {
      begin_full_inventory_target_distance_align_after_recognition(
        rec_id, "recognition stable in " + state_to_string(state_));
    }
    return true;
  }

  void select_full_inventory_same_side_pose_hold_target()
  {
    const int target = full_inventory_current_target_;
    std::string side;
    std::string reason;
    if (!get_cabinet_side(target, side, reason) || (side != "left" && side != "right")) {
      RCLCPP_WARN(
        get_logger(),
        "[FULL_INVENTORY] same-side pose hold side lookup failed target=%d reason=%s side=%s, "
        "fallback to left parameters",
        target,
        reason.c_str(),
        side.c_str());
      side = "left";
    }

    full_inventory_same_side_active_map_side_ = side;
    if (side == "right") {
      full_inventory_same_side_active_fixed_y_m_ = full_inventory_same_side_right_fixed_y_m_;
      full_inventory_same_side_active_fixed_yaw_rad_ = full_inventory_same_side_right_fixed_yaw_rad_;
    } else {
      full_inventory_same_side_active_fixed_y_m_ = full_inventory_same_side_left_fixed_y_m_;
      full_inventory_same_side_active_fixed_yaw_rad_ = full_inventory_same_side_left_fixed_yaw_rad_;
    }
  }

  bool prepare_full_inventory_rear_target_context(
    int finished_cabinet,
    int next_target_cabinet,
    std::string & reason)
  {
    reason.clear();
    if (rear_target_handle_mode_ != "hold_entry_yaw_backup") {
      reason = "unsupported rear_target_handle_mode=" + rear_target_handle_mode_;
      return false;
    }
    if (!full_inventory_last_exit_entry_yaw_valid_ ||
      !std::isfinite(full_inventory_last_exit_entry_yaw_))
    {
      reason =
        "entry_turn_start_yaw_ invalid after exit, cannot derive rear target yaw safely";
      return false;
    }
    if (!current_gap_plan_.valid) {
      reason = "current_gap_plan invalid for rear target";
      return false;
    }

    full_inventory_rear_target_pending_ = true;
    full_inventory_rear_target_active_ = false;
    full_inventory_rear_target_finished_cabinet_ = finished_cabinet;
    full_inventory_rear_target_next_cabinet_ = next_target_cabinet;
    full_inventory_rear_target_yaw_ = normalize_angle(full_inventory_last_exit_entry_yaw_);
    full_inventory_rear_target_original_gap_direction_ = current_gap_plan_.search_direction;
    full_inventory_same_side_heading_override_valid_ = true;
    full_inventory_same_side_heading_override_yaw_rad_ = full_inventory_rear_target_yaw_;
    full_inventory_gap_search_direction_override_valid_ = false;
    full_inventory_gap_search_direction_override_ = SearchDirection::FORWARD;
    full_inventory_rear_target_turn_start_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
    full_inventory_rear_target_backup_started_ = false;
    full_inventory_rear_target_backup_start_pose_ = Pose2D{};
    full_inventory_rear_target_backup_fixed_y_ = 0.0;
    full_inventory_rear_target_backup_start_time_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());

    publish_full_inventory_log(
      "[rear_target] prepare finished=" + std::to_string(finished_cabinet) +
      " next=" + std::to_string(next_target_cabinet) +
      " entry_turn_start_yaw=" + format_fixed(full_inventory_last_exit_entry_yaw_, 4) +
      " rear_target_yaw=" + format_fixed(full_inventory_rear_target_yaw_, 4) +
      " handle_mode=" + rear_target_handle_mode_ +
      " yaw_strategy=hold_entry_turn_start_yaw no_pi_flip=1 heading_override=1 " +
      "gap_direction_override=0");
    publish_full_inventory_log(
      "[rear_target] yaw strategy: hold entry_turn_start_yaw entry_turn_start_yaw=" +
      format_fixed(full_inventory_last_exit_entry_yaw_, 4) +
      " rear_target_yaw=" + format_fixed(full_inventory_rear_target_yaw_, 4) +
      " yaw_strategy=hold_entry_turn_start_yaw no_pi_flip=1");
    publish_full_inventory_log(
      "[rear_target] gap_search_direction keep original=" +
      search_direction_to_string(full_inventory_rear_target_original_gap_direction_) +
      " effective=" + search_direction_to_string(effective_gap_search_direction()) +
      " override=0 reason=no_yaw_pi_flip_hold_entry_yaw");
    return true;
  }

  void start_full_inventory_rear_target_reorient()
  {
    if (!full_inventory_rear_target_pending_) {
      fail_full_inventory("rear target reorient requested without pending context");
      return;
    }
    if (!full_inventory_last_exit_entry_yaw_valid_ ||
      !std::isfinite(full_inventory_last_exit_entry_yaw_) ||
      !std::isfinite(full_inventory_rear_target_yaw_))
    {
      fail_full_inventory("rear target yaw invalid, cannot reorient safely");
      return;
    }

    publish_stop();
    set_corridor_mode(false, false);
    request_recognizer_enable(false);
    set_recognizer_topic_enabled(false, true);
    set_distance_estimator_enabled(false, true);
    set_gap_detector_enabled(false);
    full_inventory_rear_target_turn_start_ = this->now();
    full_inventory_rear_target_active_ = true;
    publish_full_inventory_log(
      "[rear_target] start reorient target=" +
      std::to_string(full_inventory_rear_target_next_cabinet_) +
      " entry_turn_start_yaw=" + format_fixed(full_inventory_last_exit_entry_yaw_, 4) +
      " rear_target_yaw=" + format_fixed(full_inventory_rear_target_yaw_, 4) +
      " tolerance=" + format_fixed(rear_target_turn_yaw_tolerance_rad_, 4) +
      " timeout=" + format_seconds(rear_target_turn_timeout_sec_));
    set_state(
      State::FULL_INVENTORY_REAR_TARGET_REORIENT,
      "[FULL_INVENTORY] rear target reorient target=" +
      std::to_string(full_inventory_rear_target_next_cabinet_));
  }

  void handle_full_inventory_rear_target_reorient_state()
  {
    publish_entry_side();
    set_corridor_mode(false, false);
    request_recognizer_enable(false);
    set_recognizer_topic_enabled(false);
    set_distance_estimator_enabled(false);
    set_gap_detector_enabled(false);

    if (!full_inventory_rear_target_pending_) {
      publish_stop();
      fail_full_inventory("rear target reorient state without pending context");
      return;
    }
    if (full_inventory_rear_target_turn_start_.nanoseconds() == 0) {
      full_inventory_rear_target_turn_start_ = this->now();
    }

    const Pose2D current = current_pose_2d();
    if (!current.valid || !std::isfinite(current.yaw)) {
      publish_stop();
      fail_full_inventory("rear target reorient current pose/yaw invalid");
      return;
    }

    Pose2D map_pose;
    std::string tf_error;
    if (!transform_pose_2d(current, nav2_goal_frame_, map_pose, tf_error)) {
      publish_stop();
      fail_full_inventory("rear target reorient map yaw transform failed: " + tf_error);
      return;
    }
    if (!map_pose.valid || !std::isfinite(map_pose.yaw)) {
      publish_stop();
      fail_full_inventory("rear target reorient map yaw invalid");
      return;
    }

    const double elapsed = (this->now() - full_inventory_rear_target_turn_start_).seconds();
    const double timeout = std::max(0.1, rear_target_turn_timeout_sec_);
    const double yaw_error = normalize_angle(full_inventory_rear_target_yaw_ - map_pose.yaw);
    const double yaw_tolerance = std::max(0.001, rear_target_turn_yaw_tolerance_rad_);
	    const bool turn_done = std::abs(yaw_error) <= yaw_tolerance;
	    if (turn_done) {
	      publish_stop();
	      publish_full_inventory_log(
	        "[rear_target_reorient] done target=" +
	        std::to_string(full_inventory_rear_target_next_cabinet_) +
	        " strategy=hold_entry_yaw entry_turn_start_yaw=" +
	        format_fixed(full_inventory_last_exit_entry_yaw_, 4) +
	        " rear_target_yaw=" + format_fixed(full_inventory_rear_target_yaw_, 4) +
	        " current_yaw=" + format_fixed(map_pose.yaw, 4) +
	        " yaw_error=" + format_fixed(yaw_error, 4) +
	        " elapsed=" + format_seconds(elapsed) +
	        " turn_done=1");
	      start_full_inventory_rear_target_backup();
	      return;
	    }
    if (elapsed >= timeout) {
      publish_stop();
      fail_full_inventory(
        "rear target reorient timeout entry_turn_start_yaw=" +
        format_fixed(full_inventory_last_exit_entry_yaw_, 4) +
        " rear_target_yaw=" + format_fixed(full_inventory_rear_target_yaw_, 4) +
        " current_yaw=" + format_fixed(map_pose.yaw, 4) +
        " yaw_error=" + format_fixed(yaw_error, 4) +
        " elapsed=" + format_seconds(elapsed) +
        " timeout=" + format_seconds(timeout));
      return;
    }

    const double turn_speed = std::isfinite(single_cabinet_exit_turn_angular_speed_) ?
      std::abs(single_cabinet_exit_turn_angular_speed_) : 0.0;
    if (turn_speed <= 1e-4) {
      publish_stop();
      fail_full_inventory("rear target reorient turn speed invalid");
      return;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = yaw_hold_command(
      full_inventory_rear_target_yaw_,
      map_pose.yaw,
      1.0,
      turn_speed);
    cmd_pub_->publish(cmd);
	    RCLCPP_INFO_THROTTLE(
	      get_logger(),
	      *get_clock(),
	      1000,
	      "[mission_manager][FULL_INVENTORY][rear_target_reorient] target=%d "
	      "strategy=hold_entry_yaw entry_turn_start_yaw=%.4f target_yaw=%.4f "
	      "current_yaw=%.4f yaw_error=%.4f angular_cmd=%.3f elapsed=%.2f/%.2f "
	      "turn_done=%s",
	      full_inventory_rear_target_next_cabinet_,
	      full_inventory_last_exit_entry_yaw_,
	      full_inventory_rear_target_yaw_,
	      map_pose.yaw,
      yaw_error,
      cmd.angular.z,
      elapsed,
	      timeout,
	      turn_done ? "true" : "false");
	  }

	  bool current_rear_target_map_pose(Pose2D & map_pose, std::string & reason) const
	  {
	    map_pose = Pose2D{};
	    reason.clear();

	    const Pose2D current = current_pose_2d();
	    if (!current.valid || !std::isfinite(current.x) || !std::isfinite(current.y) ||
	      !std::isfinite(current.yaw))
	    {
	      reason = "current pose invalid";
	      return false;
	    }

	    std::string tf_error;
	    if (!transform_pose_2d(current, nav2_goal_frame_, map_pose, tf_error)) {
	      reason = "map pose transform failed: " + tf_error;
	      return false;
	    }
	    if (!map_pose.valid || !std::isfinite(map_pose.x) || !std::isfinite(map_pose.y) ||
	      !std::isfinite(map_pose.yaw))
	    {
	      reason = "map pose invalid";
	      return false;
	    }
	    return true;
	  }

	  void start_full_inventory_rear_target_backup()
	  {
	    if (!full_inventory_rear_target_pending_) {
	      publish_stop();
	      fail_full_inventory("rear target backup requested without pending context");
	      return;
	    }
	    if (!std::isfinite(full_inventory_rear_target_yaw_)) {
	      publish_stop();
	      fail_full_inventory("rear target backup yaw invalid");
	      return;
	    }

	    if (!rear_target_backup_enabled_ || rear_target_backup_distance_m_ <= 1e-4) {
	      publish_stop();
	      publish_full_inventory_log(
	        "[rear_target_backup] disabled target=" +
	        std::to_string(full_inventory_rear_target_next_cabinet_) +
	        " enabled=" + std::string(rear_target_backup_enabled_ ? "1" : "0") +
	        " distance=" + format_fixed(rear_target_backup_distance_m_, 2) +
	        " next=FULL_INVENTORY_SAME_SIDE_NEXT_SEARCH");
	      start_full_inventory_same_side_next_search();
	      return;
	    }
	    if (!std::isfinite(rear_target_backup_speed_) || rear_target_backup_speed_ <= 1e-4) {
	      publish_stop();
	      fail_full_inventory("rear target backup speed invalid");
	      return;
	    }

	    Pose2D map_pose;
	    std::string pose_reason;
	    if (!current_rear_target_map_pose(map_pose, pose_reason)) {
	      publish_stop();
	      fail_full_inventory("rear target backup start pose invalid: " + pose_reason);
	      return;
	    }

	    publish_stop();
	    set_corridor_mode(false, false);
	    request_recognizer_enable(false);
	    set_recognizer_topic_enabled(false, true);
	    set_distance_estimator_enabled(false, true);
	    set_gap_detector_enabled(false);
	    full_inventory_rear_target_backup_start_pose_ = map_pose;
	    full_inventory_rear_target_backup_fixed_y_ = map_pose.y;
	    full_inventory_rear_target_backup_start_time_ = this->now();
	    full_inventory_rear_target_backup_started_ = true;
	    full_inventory_rear_target_active_ = true;
	    publish_full_inventory_log(
	      "[rear_target_backup] start target=" +
	      std::to_string(full_inventory_rear_target_next_cabinet_) +
	      " start_x=" + format_fixed(map_pose.x, 3) +
	      " start_y=" + format_fixed(map_pose.y, 3) +
	      " start_yaw=" + format_fixed(map_pose.yaw, 4) +
	      " backup_fixed_y=" + format_fixed(full_inventory_rear_target_backup_fixed_y_, 3) +
	      " hold_yaw=" + format_fixed(full_inventory_rear_target_yaw_, 4) +
	      " distance=" + format_fixed(rear_target_backup_distance_m_, 2) +
	      " speed=" + format_fixed(rear_target_backup_speed_, 3) +
	      " timeout=" + format_seconds(rear_target_backup_timeout_sec_));
	    set_state(
	      State::FULL_INVENTORY_REAR_TARGET_BACKUP,
	      "[FULL_INVENTORY] rear target backup target=" +
	      std::to_string(full_inventory_rear_target_next_cabinet_));
	  }

	  void handle_full_inventory_rear_target_backup_state()
	  {
	    publish_entry_side();
	    set_corridor_mode(false, false);
	    request_recognizer_enable(false);
	    set_recognizer_topic_enabled(false);
	    set_distance_estimator_enabled(false);
	    set_gap_detector_enabled(false);

	    if (!full_inventory_rear_target_pending_) {
	      publish_stop();
	      fail_full_inventory("rear target backup state without pending context");
	      return;
	    }
	    if (!full_inventory_rear_target_backup_started_ ||
	      !full_inventory_rear_target_backup_start_pose_.valid)
	    {
	      publish_stop();
	      fail_full_inventory("rear target backup start pose missing");
	      return;
	    }
	    if (full_inventory_rear_target_backup_start_time_.nanoseconds() == 0) {
	      publish_stop();
	      fail_full_inventory("rear target backup start time missing");
	      return;
	    }

	    Pose2D map_pose;
	    std::string pose_reason;
	    if (!current_rear_target_map_pose(map_pose, pose_reason)) {
	      publish_stop();
	      fail_full_inventory("rear target backup current pose invalid: " + pose_reason);
	      return;
	    }

	    const double dx = map_pose.x - full_inventory_rear_target_backup_start_pose_.x;
	    const double dy = map_pose.y - full_inventory_rear_target_backup_start_pose_.y;
	    const double traveled = std::hypot(dx, dy);
	    const double target_distance = std::max(0.0, rear_target_backup_distance_m_);
	    const double elapsed = (this->now() - full_inventory_rear_target_backup_start_time_).seconds();
	    const double timeout = std::max(0.1, rear_target_backup_timeout_sec_);
	    const double hold_yaw = normalize_angle(full_inventory_rear_target_yaw_);
	    const double yaw_error = normalize_angle(hold_yaw - map_pose.yaw);
	    const double y_error = full_inventory_rear_target_backup_fixed_y_ - map_pose.y;

	    if (traveled >= target_distance) {
	      publish_stop();
	      publish_full_inventory_log(
	        "[rear_target_backup] done target=" +
	        std::to_string(full_inventory_rear_target_next_cabinet_) +
	        " traveled=" + format_fixed(traveled, 2) +
	        " next=FULL_INVENTORY_SAME_SIDE_NEXT_SEARCH heading_override=" +
	        std::string(full_inventory_same_side_heading_override_valid_ ? "1" : "0") +
	        " final_search_yaw=" +
	        format_fixed(full_inventory_same_side_heading_override_yaw_rad_, 4));
	      start_full_inventory_same_side_next_search();
	      return;
	    }
	    if (elapsed >= timeout) {
	      publish_stop();
	      fail_full_inventory(
	        "rear target backup timeout target=" +
	        std::to_string(full_inventory_rear_target_next_cabinet_) +
	        " traveled=" + format_fixed(traveled, 2) +
	        " target_distance=" + format_fixed(target_distance, 2) +
	        " elapsed=" + format_seconds(elapsed) +
	        " timeout=" + format_seconds(timeout));
	      return;
	    }

	    const double linear_cmd = -std::abs(rear_target_backup_speed_);
	    const double max_angular = std::isfinite(full_inventory_same_side_max_angular_) ?
	      std::max(0.0, std::abs(full_inventory_same_side_max_angular_)) : 0.15;
	    const double yaw_kp = std::isfinite(full_inventory_same_side_yaw_kp_) ?
	      std::max(0.0, full_inventory_same_side_yaw_kp_) : 0.40;
	    geometry_msgs::msg::Twist cmd;
	    cmd.linear.x = linear_cmd;
	    cmd.angular.z = yaw_hold_command(hold_yaw, map_pose.yaw, yaw_kp, max_angular);
	    cmd_pub_->publish(cmd);

	    RCLCPP_INFO_THROTTLE(
	      get_logger(),
	      *get_clock(),
	      1000,
	      "[mission_manager][FULL_INVENTORY][rear_target_backup] target=%d "
	      "start=(%.2f,%.2f) current=(%.2f,%.2f) backup_fixed_y=%.2f y_error=%.2f "
	      "traveled=%.2f/%.2f linear=%.3f hold_yaw=%.4f current_yaw=%.4f "
	      "yaw_error=%.4f angular=%.3f elapsed=%.2f/%.2f",
	      full_inventory_rear_target_next_cabinet_,
	      full_inventory_rear_target_backup_start_pose_.x,
	      full_inventory_rear_target_backup_start_pose_.y,
	      map_pose.x,
	      map_pose.y,
	      full_inventory_rear_target_backup_fixed_y_,
	      y_error,
	      traveled,
	      target_distance,
	      cmd.linear.x,
	      hold_yaw,
	      map_pose.yaw,
	      yaw_error,
	      cmd.angular.z,
	      elapsed,
	      timeout);
	  }

  void start_full_inventory_same_side_next_search()
  {
    full_inventory_same_side_search_start_ = this->now();
    select_full_inventory_same_side_pose_hold_target();
    const double normal_fixed_yaw = full_inventory_same_side_active_fixed_yaw_rad_;
    if (full_inventory_same_side_heading_override_valid_) {
      full_inventory_same_side_active_fixed_yaw_rad_ =
        normalize_angle(full_inventory_same_side_heading_override_yaw_rad_);
    }
    std::string fixed_y_source = "config_fallback";
    std::string fixed_y_pose_note = "pose hold disabled";
    double fixed_y_start_current_y = std::numeric_limits<double>::quiet_NaN();
    if (full_inventory_same_side_pose_hold_enabled_) {
      Pose2D start_pose;
      if (current_same_side_pose_hold_pose(start_pose, fixed_y_pose_note)) {
        full_inventory_same_side_active_fixed_y_m_ = start_pose.y;
        fixed_y_start_current_y = start_pose.y;
        fixed_y_source = "current_pose";
      }
    }
    reset_target_recognition_stability();
    reset_segment_distance();
    target_visible_ = false;
    has_distance_ = false;
    latest_distance_ = 0.0;
    const double delay_distance = full_inventory_same_side_recognition_delay_distance();
    const bool recognizer_allowed = delay_distance <= 1e-4;
    request_recognizer_enable(recognizer_allowed);
    set_distance_estimator_enabled(false, true);
    set_gap_detector_enabled(false);
    set_corridor_mode(false, false);
    publish_full_inventory_log(
      "same-side next search start target=" +
      std::to_string(full_inventory_current_target_) +
      (recognizer_allowed ?
      " recognizer=on" :
      " recognizer=delayed/off until distance=" + format_fixed(delay_distance, 2) + "m") +
      " distance=off gap=off speed=" +
      format_seconds(full_inventory_same_side_search_speed_) +
      " timeout=" + format_seconds(full_inventory_same_side_search_timeout_sec_));
    if (full_inventory_same_side_pose_hold_enabled_) {
      publish_full_inventory_log(
        "same-side pose hold enabled target=" +
        std::to_string(full_inventory_current_target_) +
        " map_side=" + full_inventory_same_side_active_map_side_ +
        " fixed_y=" + format_fixed(full_inventory_same_side_active_fixed_y_m_, 3) +
        " use_heading_override=" +
        std::string(full_inventory_same_side_heading_override_valid_ ? "true" : "false") +
        " normal_fixed_yaw=" + format_fixed(normal_fixed_yaw, 4) +
        " override_yaw=" +
        format_fixed(full_inventory_same_side_heading_override_yaw_rad_, 4) +
        " fixed_yaw=" + format_fixed(full_inventory_same_side_active_fixed_yaw_rad_, 4) +
        " fixed_y_source=" + fixed_y_source +
        " start_current_y=" + format_fixed(fixed_y_start_current_y, 3) +
        " pose_note=" + fixed_y_pose_note);
    }
    set_state(
      State::FULL_INVENTORY_SAME_SIDE_NEXT_SEARCH,
      "[FULL_INVENTORY] same-side next search target=" +
      std::to_string(full_inventory_current_target_));
    set_recognizer_topic_enabled(recognizer_allowed, true);
  }

  bool current_same_side_pose_hold_pose(Pose2D & pose, std::string & pose_note) const
  {
    pose = Pose2D{};
    pose_note.clear();

    const Pose2D current = current_pose_2d();
    if (!current.valid) {
      pose_note = "pose invalid";
      return false;
    }

    Pose2D map_pose;
    std::string tf_error;
    if (transform_pose_2d(current, nav2_goal_frame_, map_pose, tf_error)) {
      pose = map_pose;
      pose_note = "frame=" + pose.frame_id;
      return true;
    }

    pose = current;
    pose_note =
      "map transform failed: " + tf_error + ", using pose frame=" + current.frame_id;
    return true;
  }

  void handle_full_inventory_same_side_next_search_state()
  {
    set_distance_estimator_enabled(false);
    set_gap_detector_enabled(false);
    if (full_inventory_same_side_search_start_.nanoseconds() == 0) {
      full_inventory_same_side_search_start_ = this->now();
      reset_segment_distance();
    }
    const double elapsed = (this->now() - full_inventory_same_side_search_start_).seconds();
    const double timeout = std::max(0.1, full_inventory_same_side_search_timeout_sec_);
    if (elapsed >= timeout) {
      publish_stop();
      fail_full_inventory(
        "same side recognition search timeout target=" +
        std::to_string(current_target_cabinet_));
      return;
    }

    const double delay_distance = full_inventory_same_side_recognition_delay_distance();
    const double delay_progress = segment_distance();
    const bool recognition_delayed = delay_distance > 1e-4 && delay_progress < delay_distance;
    if (recognition_delayed) {
      request_recognizer_enable(false);
      set_recognizer_topic_enabled(false);
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "[FULL_INVENTORY] same_side_next_search recognition delayed progress=%.2f/%.2f",
        delay_progress,
        delay_distance);
    } else {
      request_recognizer_enable(true);
      set_recognizer_topic_enabled(true);
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = full_inventory_same_side_search_speed_;
    cmd.angular.z = 0.0;
    Pose2D pose;
    std::string pose_note;
    bool pose_hold_active = false;
    double y_error = 0.0;
    double yaw_error = 0.0;
    double yaw_cmd = 0.0;
    double y_cmd = 0.0;

    if (full_inventory_same_side_pose_hold_enabled_) {
      if (current_same_side_pose_hold_pose(pose, pose_note)) {
        pose_hold_active = true;
        y_error = full_inventory_same_side_active_fixed_y_m_ - pose.y;
        yaw_error = normalize_angle(full_inventory_same_side_active_fixed_yaw_rad_ - pose.yaw);
        if (std::abs(yaw_error) >= std::abs(full_inventory_same_side_yaw_deadband_rad_)) {
          yaw_cmd = full_inventory_same_side_yaw_kp_ * yaw_error;
        }
        if (std::abs(y_error) >= std::abs(full_inventory_same_side_y_deadband_m_)) {
          y_cmd = full_inventory_same_side_y_kp_ * y_error;
        }
        const double max_angular =
          std::isfinite(full_inventory_same_side_max_angular_) ?
          std::max(0.0, std::abs(full_inventory_same_side_max_angular_)) : 0.15;
        cmd.angular.z = std::clamp(
          yaw_cmd + full_inventory_same_side_y_correction_sign_ * y_cmd,
          -max_angular,
          max_angular);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "[FULL_INVENTORY] same-side pose hold unavailable: %s, fallback to open-loop search",
          pose_note.c_str());
      }
    }
    cmd_pub_->publish(cmd);
    if (pose_hold_active) {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "[FULL_INVENTORY] same_side_next_search target=%d map_side=%s speed=%.3f current_y=%.3f "
        "fixed_y=%.3f y_error=%.3f current_yaw=%.4f fixed_yaw=%.4f "
        "use_heading_override=%s override_yaw=%.4f final_search_yaw=%.4f "
        "yaw_error=%.4f angular=%.3f elapsed=%.2f/%.2f pose=%s",
        current_target_cabinet_,
        full_inventory_same_side_active_map_side_.c_str(),
        cmd.linear.x,
        pose.y,
        full_inventory_same_side_active_fixed_y_m_,
        y_error,
        pose.yaw,
        full_inventory_same_side_active_fixed_yaw_rad_,
        full_inventory_same_side_heading_override_valid_ ? "true" : "false",
        full_inventory_same_side_heading_override_yaw_rad_,
        full_inventory_same_side_active_fixed_yaw_rad_,
        yaw_error,
        cmd.angular.z,
        elapsed,
        timeout,
        pose_note.c_str());
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "[FULL_INVENTORY] same_side_next_search target=%d map_side=%s speed=%.3f angular=%.3f "
        "use_heading_override=%s override_yaw=%.4f final_search_yaw=%.4f "
        "elapsed=%.2f/%.2f open_loop=true",
        current_target_cabinet_,
        full_inventory_same_side_active_map_side_.c_str(),
        cmd.linear.x,
        cmd.angular.z,
        full_inventory_same_side_heading_override_valid_ ? "true" : "false",
        full_inventory_same_side_heading_override_yaw_rad_,
        full_inventory_same_side_active_fixed_yaw_rad_,
        elapsed,
        timeout);
    }
  }

  void begin_full_inventory_recognition_fallback()
  {
    if (!full_inventory_recognition_fallback_enabled_) {
      fail_full_inventory(
        "post-route recognition wait timeout and recognition fallback disabled target=" +
        std::to_string(current_target_cabinet_));
      return;
    }
    if (full_inventory_recognition_fallback_sequence_.empty()) {
      fail_full_inventory(
        "post-route recognition wait timeout and fallback sequence empty target=" +
        std::to_string(current_target_cabinet_));
      return;
    }

    publish_stop();
    request_recognizer_enable(true);
    set_recognizer_topic_enabled(true, true);
    set_distance_estimator_enabled(false, true);
    set_gap_detector_enabled(false);
    reset_target_recognition_stability();
    full_inventory_recognition_fallback_start_ = this->now();
    full_inventory_recognition_fallback_phase_start_ = this->now();
    full_inventory_recognition_fallback_index_ = 0;
    full_inventory_recognition_fallback_phase_ = FullInventoryRecognitionFallbackPhase::MOVING;
    reset_segment_distance();
    publish_full_inventory_log(
      "recognition fallback start sequence=" +
      double_vector_to_string(full_inventory_recognition_fallback_sequence_));
    publish_full_inventory_log(
      "recognition fallback step=1 move=" +
      format_seconds(full_inventory_recognition_fallback_sequence_.front()));
    set_state(
      State::FULL_INVENTORY_RECOGNITION_FALLBACK,
      "[FULL_INVENTORY] recognition fallback target=" +
      std::to_string(current_target_cabinet_));
  }

  void handle_full_inventory_post_route_recognition_wait_state()
  {
    publish_stop();
    request_recognizer_enable(true);
    set_recognizer_topic_enabled(true);
    set_distance_estimator_enabled(false);
    set_gap_detector_enabled(false);
    if (full_inventory_final_recognition_wait_start_.nanoseconds() == 0) {
      full_inventory_final_recognition_wait_start_ = this->now();
    }

    const double elapsed =
      (this->now() - full_inventory_final_recognition_wait_start_).seconds();
    const double timeout = std::max(0.0, full_inventory_final_recognition_wait_sec_);
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "[mission_manager][FULL_INVENTORY] post-route recognition wait: target=%d elapsed=%.2f/%.2f",
      current_target_cabinet_,
      elapsed,
      timeout);

    if (elapsed >= timeout) {
      publish_full_inventory_log(
        "post-route recognition wait timeout: target=" +
        std::to_string(current_target_cabinet_));
      begin_full_inventory_recognition_fallback();
    }
  }

  void handle_full_inventory_recognition_fallback_state()
  {
    request_recognizer_enable(true);
    set_recognizer_topic_enabled(true);
    set_distance_estimator_enabled(false);
    set_gap_detector_enabled(false);

    if (full_inventory_recognition_fallback_start_.nanoseconds() == 0) {
      begin_full_inventory_recognition_fallback();
      return;
    }

    const double elapsed_total =
      (this->now() - full_inventory_recognition_fallback_start_).seconds();
    const double fallback_step_timeout =
      std::max(0.1, full_inventory_recognition_fallback_timeout_sec_);

    if (full_inventory_recognition_fallback_index_ >=
      full_inventory_recognition_fallback_sequence_.size())
    {
      publish_stop();
      fail_full_inventory(
        "recognition fallback sequence exhausted target=" +
        std::to_string(current_target_cabinet_));
      return;
    }

    const double step =
      full_inventory_recognition_fallback_sequence_[full_inventory_recognition_fallback_index_];
    switch (full_inventory_recognition_fallback_phase_) {
      case FullInventoryRecognitionFallbackPhase::MOVING: {
        const double target_distance = std::abs(step);
        const double speed = std::abs(full_inventory_recognition_fallback_speed_);
        const double move_elapsed =
          (this->now() - full_inventory_recognition_fallback_phase_start_).seconds();
        const double required_time = speed > 1e-4 ? target_distance / speed : fallback_step_timeout;
        const double move_timeout = std::max(fallback_step_timeout, required_time + 2.0);
        if (move_elapsed >= move_timeout) {
          publish_stop();
          fail_full_inventory(
            "recognition fallback move timeout target=" +
            std::to_string(current_target_cabinet_) +
            " step=" + std::to_string(full_inventory_recognition_fallback_index_ + 1U) +
            " move=" + format_seconds(step) +
            " elapsed=" + format_seconds(move_elapsed) +
            " timeout=" + format_seconds(move_timeout));
          return;
        }
        if (target_distance <= 1e-4 || segment_distance() >= target_distance) {
          publish_stop();
          full_inventory_recognition_fallback_phase_ = FullInventoryRecognitionFallbackPhase::WAITING;
          full_inventory_recognition_fallback_phase_start_ = this->now();
          break;
        }

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = (step >= 0.0 ? 1.0 : -1.0) *
          std::abs(full_inventory_recognition_fallback_speed_);
        cmd.angular.z = 0.0;
        cmd_pub_->publish(cmd);
        RCLCPP_INFO_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "[mission_manager][FULL_INVENTORY] recognition_fallback moving step=%zu/%zu "
          "move=%.2f traveled=%.2f/%.2f elapsed=%.2f/%.2f",
          full_inventory_recognition_fallback_index_ + 1U,
          full_inventory_recognition_fallback_sequence_.size(),
          step,
          segment_distance(),
          target_distance,
          elapsed_total,
          move_timeout);
        break;
      }

      case FullInventoryRecognitionFallbackPhase::WAITING: {
        publish_stop();
        const double wait_elapsed =
          (this->now() - full_inventory_recognition_fallback_phase_start_).seconds();
        if (wait_elapsed < std::max(0.0, full_inventory_recognition_fallback_wait_sec_)) {
          break;
        }

        ++full_inventory_recognition_fallback_index_;
        if (full_inventory_recognition_fallback_index_ >=
          full_inventory_recognition_fallback_sequence_.size())
        {
          fail_full_inventory(
            "recognition fallback sequence exhausted target=" +
            std::to_string(current_target_cabinet_));
          return;
        }

        full_inventory_recognition_fallback_phase_ = FullInventoryRecognitionFallbackPhase::MOVING;
        full_inventory_recognition_fallback_phase_start_ = this->now();
        reset_segment_distance();
        publish_full_inventory_log(
          "recognition fallback step=" +
          std::to_string(full_inventory_recognition_fallback_index_ + 1U) +
          " move=" +
          format_seconds(
            full_inventory_recognition_fallback_sequence_[full_inventory_recognition_fallback_index_]));
        break;
      }

      case FullInventoryRecognitionFallbackPhase::IDLE:
      default:
        full_inventory_recognition_fallback_phase_ = FullInventoryRecognitionFallbackPhase::MOVING;
        full_inventory_recognition_fallback_phase_start_ = this->now();
        reset_segment_distance();
        break;
    }
  }

  void handle_full_inventory_target_distance_align_state()
  {
    request_recognizer_enable(false);
    set_recognizer_topic_enabled(false);
    set_gap_detector_enabled(false);
    set_distance_estimator_enabled(true);

    if (!has_distance_ || !std::isfinite(latest_distance_) || latest_distance_ <= 0.0) {
      publish_stop();
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "[mission_manager][FULL_INVENTORY] target distance align waiting for distance target=%d",
        current_target_cabinet_);
      return;
    }

    const double aligned_min = target_distance_aligned_min_m_;
    const double aligned_max = std::max(target_distance_aligned_max_m_, aligned_min);
    const double gap_threshold = std::max(target_distance_gap_threshold_m_, aligned_max);

    if (latest_distance_ >= gap_threshold) {
      ++target_distance_gap_open_count_;
      tracking_stable_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      publish_stop();
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "[mission_manager][FULL_INVENTORY] target distance align class=gap_open target=%d "
        "distance=%.2f aligned_min=%.2f aligned_max=%.2f gap_threshold=%.2f "
        "confirm=%d/%d cmd.linear.x=0.000",
        current_target_cabinet_,
        latest_distance_,
        aligned_min,
        aligned_max,
        gap_threshold,
        target_distance_gap_open_count_,
        target_distance_gap_confirm_count_);
      if (target_distance_gap_open_count_ >= target_distance_gap_confirm_count_) {
        publish_full_inventory_log(
          "target distance align gap_open: target_distance=" +
          format_fixed(latest_distance_, 2) +
          " >= threshold=" + format_fixed(gap_threshold, 2) +
          "，认为已扫到缝隙/开放空间，提前进入 SEARCH_GAP");
        set_distance_estimator_enabled(false, true);
        begin_search_gap_flow();
      }
      return;
    }

    target_distance_gap_open_count_ = 0;

    if (latest_distance_ >= aligned_min && latest_distance_ <= aligned_max) {
      publish_stop();
      if (tracking_stable_start_.nanoseconds() == 0) {
        tracking_stable_start_ = this->now();
      }
      const double stable_elapsed = (this->now() - tracking_stable_start_).seconds();
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "[mission_manager][FULL_INVENTORY] target distance align class=aligned target=%d "
        "distance=%.2f aligned_min=%.2f aligned_max=%.2f gap_threshold=%.2f "
        "stable_elapsed=%.2f/%.2f cmd.linear.x=0.000",
        current_target_cabinet_,
        latest_distance_,
        aligned_min,
        aligned_max,
        gap_threshold,
        stable_elapsed,
        distance_stable_time_sec_);
      if (stable_elapsed >= distance_stable_time_sec_) {
        publish_stop();
        set_distance_estimator_enabled(false, true);
        publish_full_inventory_log(
          "target distance align aligned: distance=" + format_fixed(latest_distance_, 2) +
          " in [" + format_fixed(aligned_min, 2) + ", " + format_fixed(aligned_max, 2) +
          "] stable_elapsed=" + format_fixed(stable_elapsed, 2) +
          "，进入 SEARCH_GAP");
        begin_search_gap_flow();
      }
      return;
    }

    tracking_stable_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    const double distance_error = latest_distance_ - follow_distance_;
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = std::clamp(
      tracking_kp_distance_ * distance_error,
      -std::abs(tracking_speed_),
      std::abs(tracking_speed_));
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);

    const char * range_class =
      latest_distance_ < aligned_min ? "too_close" : "middle_adjust";
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "[mission_manager][FULL_INVENTORY] target distance align class=%s target=%d "
      "distance=%.2f aligned_min=%.2f aligned_max=%.2f gap_threshold=%.2f "
      "follow=%.2f tolerance=%.2f distance_error=%.2f cmd.linear.x=%.3f",
      range_class,
      current_target_cabinet_,
      latest_distance_,
      aligned_min,
      aligned_max,
      gap_threshold,
      follow_distance_,
      distance_tolerance_,
      distance_error,
      cmd.linear.x);
  }

  void handle_full_inventory_scan_state()
  {
    const int cabinet_id = full_inventory_current_target_;
    if (cabinet_id <= 0) {
      fail_full_inventory("全部盘库扫描柜号非法");
      return;
    }
    request_recognizer_enable(false);
    set_recognizer_topic_enabled(false);
    set_distance_estimator_enabled(false);
    set_gap_detector_enabled(false);

    if (!single_cabinet_scan_active_ || single_cabinet_scan_cabinet_ != cabinet_id) {
      publish_full_inventory_log(
        "FULL_INVENTORY_IN_GAP_SCAN runtime start cabinet=" + std::to_string(cabinet_id) +
        " current_depth=" + std::to_string(current_target_depth_index_));
    }

    if (!execute_single_cabinet_scan_runtime_tick(cabinet_id, InGapScanRuntimeMode::FULL_INVENTORY)) {
      return;
    }

    publish_full_inventory_log(
      "FULL_INVENTORY_IN_GAP_SCAN runtime finished cabinet=" + std::to_string(cabinet_id));
    clear_full_inventory_rear_target_context("current target scan complete");
    set_state(
      State::FULL_INVENTORY_EXIT_GAP,
      "[FULL_INVENTORY] in-gap sequence finished, reverse exit gap cabinet=" +
      std::to_string(cabinet_id));
  }

  bool should_auto_charge_between_targets(
    int current_target,
    int next_target,
    std::string & current_side,
    std::string & next_side,
    std::string & reason) const
  {
    if (!get_cabinet_side(current_target, current_side, reason)) {
      return false;
    }
    if (!get_cabinet_side(next_target, next_side, reason)) {
      return false;
    }
    return current_side != next_side;
  }

  bool auto_recharge_status_is_ready() const
  {
    const std::string status = agv_inventory_system::trim(latest_auto_recharge_status_);
    return status == "CHARGING" || status == "COMPLETE";
  }

  bool auto_recharge_status_is_failure() const
  {
    const std::string status = agv_inventory_system::trim(latest_auto_recharge_status_);
    return status == "FAILED" || status == "CANCELED";
  }

  bool auto_recharge_charging_flag_ready() const
  {
    return latest_auto_recharge_charging_;
  }

  bool send_between_side_auto_recharge_cancel_request(std::string & message)
  {
    message.clear();
    if (!inventory_auto_recharge_cancel_client_) {
      message = "自动回充取消服务不可用，请确认 inventory_auto_recharger 节点已启动。";
      publish_log(message);
      publish_state_text(message);
      return false;
    }

    const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(std::max(0.0, auto_recharge_cancel_timeout_sec_)));
    if (!inventory_auto_recharge_cancel_client_->wait_for_service(timeout)) {
      message = "自动回充取消服务不可用，请确认 inventory_auto_recharger 节点已启动。";
      publish_log(message);
      publish_state_text(message);
      return false;
    }

    between_side_auto_charge_cancel_response_ready_ = false;
    between_side_auto_charge_cancel_response_success_ = false;
    between_side_auto_charge_cancel_response_message_.clear();
    between_side_auto_charge_cancel_request_time_ = this->now();

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    (void)inventory_auto_recharge_cancel_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        try {
          const auto response = future.get();
          if (!response) {
            between_side_auto_charge_cancel_response_ready_ = true;
            between_side_auto_charge_cancel_response_success_ = false;
            between_side_auto_charge_cancel_response_message_ = "自动回充取消节点未返回有效响应";
            publish_log(between_side_auto_charge_cancel_response_message_);
            return;
          }
          between_side_auto_charge_cancel_response_ready_ = true;
          between_side_auto_charge_cancel_response_success_ = response->success;
          between_side_auto_charge_cancel_response_message_ = response->message;
          const std::string detail =
            std::string("跨侧自动回充取消节点响应: ") +
            (response->success ? "success" : "failed") +
            "，" + response->message;
          if (response->success) {
            publish_log(detail);
          } else {
            RCLCPP_ERROR(get_logger(), "%s", detail.c_str());
            publish_log(detail);
          }
        } catch (const std::exception & ex) {
          between_side_auto_charge_cancel_response_ready_ = true;
          between_side_auto_charge_cancel_response_success_ = false;
          between_side_auto_charge_cancel_response_message_ =
            std::string("自动回充取消响应处理异常: ") + ex.what();
          RCLCPP_ERROR(get_logger(), "%s", between_side_auto_charge_cancel_response_message_.c_str());
          publish_log(between_side_auto_charge_cancel_response_message_);
        }
      });

    message = "[FULL_INVENTORY] between-side auto charge finished，已发送取消自动回充请求。";
    publish_log(message);
    return true;
  }

  void start_between_side_auto_charge(int next_target)
  {
    clear_full_inventory_rear_target_context("start_between_side_auto_charge");
    reset_between_side_auto_charge_runtime();
    between_side_auto_charge_active_ = true;
    between_side_auto_charge_target_index_ = full_inventory_index_;
    between_side_auto_charge_target_ = next_target;
    between_side_auto_charge_start_time_ = this->now();
    latest_auto_recharge_status_.clear();
    latest_auto_recharge_charging_ = false;

    std::string message;
    if (!send_auto_recharge_start_request("[FULL_INVENTORY] side changed, start auto charge before next side", message)) {
      fail_full_inventory("跨侧自动回充启动失败: " + message);
      return;
    }

    publish_full_inventory_log(
      "side changed, auto charge before next side target=" + std::to_string(next_target) +
      " wait_after_ready=" + format_seconds(between_side_auto_charge_wait_sec_) +
      " fallback_runtime=" + format_seconds(between_side_auto_charge_runtime_sec_));
    set_state(
      State::FULL_INVENTORY_AUTO_CHARGE_BETWEEN_SIDES,
      "[FULL_INVENTORY] auto charge between sides target=" + std::to_string(next_target));
  }

  void continue_full_inventory_after_between_side_auto_charge()
  {
    clear_full_inventory_rear_target_context("continue_after_between_side_auto_charge");
    if (between_side_auto_charge_target_index_ >= full_inventory_sequence_.size()) {
      fail_full_inventory("跨侧自动回充后 full_inventory_index 越界");
      return;
    }

    const int target = full_inventory_sequence_[between_side_auto_charge_target_index_];
    if (target != between_side_auto_charge_target_) {
      fail_full_inventory(
        "跨侧自动回充后目标不一致 expected=" +
        std::to_string(between_side_auto_charge_target_) +
        " actual=" + std::to_string(target));
      return;
    }

    reset_between_side_auto_charge_runtime();
    std::string prepare_reason;
    if (!prepare_full_inventory_target(target, prepare_reason)) {
      fail_full_inventory("跨侧自动回充后准备目标失败: " + prepare_reason);
      return;
    }
    publish_full_inventory_log(
      "between-side auto charge finished, switch to side=" + full_inventory_current_side_ +
      " route=" + full_inventory_current_route_ +
      " target=" + std::to_string(target));
    if (!begin_plc_open_wait_for_target(
        current_target_cabinet_,
        PlcOpenContinuation::FULL_INVENTORY_BETWEEN_SIDE_ROUTE,
        "between-side auto charge finished before route"))
    {
      return;
    }
    if (!plc_http_enabled_) {
      (void)start_full_inventory_target_route("between-side auto charge finished, switch side");
    }
  }

  void handle_full_inventory_auto_charge_between_sides_state()
  {
    publish_stop();
    if (!between_side_auto_charge_active_) {
      fail_full_inventory("跨侧自动回充状态未初始化");
      return;
    }

    const double elapsed =
      (this->now() - between_side_auto_charge_start_time_).seconds();
    const bool ready = auto_recharge_status_is_ready() || auto_recharge_charging_flag_ready();
    const std::string auto_recharge_status =
      agv_inventory_system::trim(latest_auto_recharge_status_);

    if (between_side_auto_charge_cancel_sent_) {
      if (!between_side_auto_charge_cancel_response_ready_) {
        const double cancel_elapsed =
          between_side_auto_charge_cancel_request_time_.nanoseconds() == 0 ?
          0.0 : (this->now() - between_side_auto_charge_cancel_request_time_).seconds();
        if (cancel_elapsed >= std::max(0.1, auto_recharge_cancel_response_timeout_sec_)) {
          between_side_auto_charge_cancel_response_ready_ = true;
          between_side_auto_charge_cancel_response_success_ = false;
          between_side_auto_charge_cancel_response_message_ =
            "跨侧自动回充取消响应超时，timeout=" +
            format_seconds(auto_recharge_cancel_response_timeout_sec_) +
            " status=" + auto_recharge_status;
          RCLCPP_WARN(get_logger(), "%s", between_side_auto_charge_cancel_response_message_.c_str());
          publish_full_inventory_log(
            between_side_auto_charge_cancel_response_message_ +
            "，继续执行 mission_manager 跨侧离桩。");
        } else {
          if (auto_recharge_status == "CANCELING" || auto_recharge_status == "CANCELED") {
            RCLCPP_INFO_THROTTLE(
              get_logger(),
              *get_clock(),
              1000,
              "[mission_manager][FULL_INVENTORY] waiting between-side auto charge cancel response, "
              "ignore transitional status=%s",
              auto_recharge_status.c_str());
          } else {
            RCLCPP_INFO_THROTTLE(
              get_logger(),
              *get_clock(),
              1000,
              "[mission_manager][FULL_INVENTORY] waiting between-side auto charge cancel response");
          }
          return;
        }
      }

      if (!between_side_auto_charge_cancel_response_success_) {
        RCLCPP_WARN(
          get_logger(),
          "跨侧自动回充取消未确认成功，继续执行 mission_manager 跨侧离桩: %s",
          between_side_auto_charge_cancel_response_message_.c_str());
        publish_full_inventory_log(
          "between-side auto charge cancel not confirmed, continue depart; reason=" +
          between_side_auto_charge_cancel_response_message_);
      }

      publish_full_inventory_log(
        "between-side auto charge cancel handled, start mission_manager depart; fallback=" +
        std::string(between_side_auto_charge_fallback_used_ ? "true" : "false"));
      std::string depart_message;
      if (!start_stop_auto_charge_and_depart_flow(
          depart_message,
          StopAutoChargeDepartContinuation::FULL_INVENTORY_BETWEEN_SIDES))
      {
        fail_full_inventory("跨侧离桩启动失败: " + depart_message);
      }
      return;
    }

    if (auto_recharge_status_is_failure() && !ready) {
      fail_full_inventory("跨侧自动回充失败，status=" + latest_auto_recharge_status_);
      return;
    }

    if (ready && between_side_auto_charge_ready_time_.nanoseconds() == 0) {
      between_side_auto_charge_ready_time_ = this->now();
      publish_full_inventory_log(
        "between-side auto charge ready status=" + latest_auto_recharge_status_ +
        " charging_flag=" + std::string(latest_auto_recharge_charging_ ? "true" : "false") +
        ", wait " + format_seconds(between_side_auto_charge_wait_sec_) + " before cancel");
    }

    bool should_cancel = false;
    if (between_side_auto_charge_ready_time_.nanoseconds() != 0) {
      const double ready_elapsed =
        (this->now() - between_side_auto_charge_ready_time_).seconds();
      should_cancel = ready_elapsed >= std::max(0.0, between_side_auto_charge_wait_sec_);
    } else if (elapsed >= std::max(0.1, between_side_auto_charge_runtime_sec_)) {
      between_side_auto_charge_fallback_used_ = true;
      should_cancel = true;
      publish_full_inventory_log(
        "between-side auto charge fallback runtime reached, no readable ready state; runtime=" +
        format_seconds(elapsed) +
        " configured=" + format_seconds(between_side_auto_charge_runtime_sec_));
    }

    if (!should_cancel) {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "[mission_manager][FULL_INVENTORY] between-side auto charge waiting target=%d "
        "status=%s charging=%s elapsed=%.2f runtime=%.2f ready=%s",
        between_side_auto_charge_target_,
        auto_recharge_status.c_str(),
        latest_auto_recharge_charging_ ? "true" : "false",
        elapsed,
        between_side_auto_charge_runtime_sec_,
        ready ? "true" : "false");
      return;
    }

    if (!between_side_auto_charge_cancel_sent_) {
      std::string message;
      bool cancel_request_sent = true;
      if (!send_between_side_auto_recharge_cancel_request(message)) {
        cancel_request_sent = false;
        between_side_auto_charge_cancel_response_ready_ = true;
        between_side_auto_charge_cancel_response_success_ = false;
        between_side_auto_charge_cancel_response_message_ = message;
        publish_full_inventory_log(
          "between-side auto charge cancel request failed, continue mission_manager depart; reason=" +
          message);
      }
      between_side_auto_charge_cancel_sent_ = true;
      if (cancel_request_sent) {
        publish_full_inventory_log(
          "between-side auto charge cancel sent, waiting response; fallback=" +
          std::string(between_side_auto_charge_fallback_used_ ? "true" : "false"));
      }
      return;
    }
  }

  void advance_full_inventory_sequence()
  {
    if (!full_inventory_active_) {
      fail_full_inventory("advance requested while full inventory inactive");
      return;
    }
    if (full_inventory_index_ >= full_inventory_sequence_.size()) {
      fail_full_inventory("full_inventory_index 越界");
      return;
    }

    const int finished_target = full_inventory_sequence_[full_inventory_index_];

    if (full_inventory_index_ + 1U >= full_inventory_sequence_.size()) {
      publish_full_inventory_log(
        "exit gap done target=" + std::to_string(finished_target) +
        " next_target=-1 same_side=false");
      if (!flush_inventory_upload_batch_for_mission_complete("all_cabinets_complete")) {
        fail_full_inventory("RFID final batch upload failed at all_cabinets_complete");
        return;
      }
      publish_full_inventory_log("all targets completed, execute finish_return_mode=" + finish_return_mode_);
      (void)start_finish_action(
        "[FULL_INVENTORY] all targets completed, execute finish_return_mode",
        mission_force_map_origin_on_finish_);
      return;
    }

    const int next_target = full_inventory_sequence_[full_inventory_index_ + 1U];
    std::string current_side;
    std::string next_side;
    std::string side_reason;
    const bool auto_charge_between =
      should_auto_charge_between_targets(finished_target, next_target, current_side, next_side, side_reason);
    if (!side_reason.empty()) {
      fail_full_inventory("判断目标侧失败: " + side_reason);
      return;
    }
    const bool same_side = current_side == next_side;
    publish_full_inventory_log(
      "exit gap done target=" + std::to_string(finished_target) +
      " next_target=" + std::to_string(next_target) +
      " same_side=" + std::string(same_side ? "true" : "false"));

    ++full_inventory_index_;
    full_inventory_next_target_ =
      full_inventory_index_ + 1U < full_inventory_sequence_.size() ?
      full_inventory_sequence_[full_inventory_index_ + 1U] : -1;

    if (auto_charge_between) {
      publish_full_inventory_log(
        "next_target=" + std::to_string(next_target) +
        " side changed " + current_side + " -> " + next_side +
        ", auto charge first");
      start_between_side_auto_charge(next_target);
      return;
    }

    std::string prepare_reason;
    if (!prepare_full_inventory_target(next_target, prepare_reason)) {
      fail_full_inventory("准备同侧下一个目标失败: " + prepare_reason);
      return;
    }

    if (same_side && full_inventory_same_side_next_search_enabled_) {
      clear_full_inventory_rear_target_context("prepare same-side transition", true);
      std::string rear_target_reason;
      const bool use_rear_target =
        should_use_rear_target_handling(finished_target, next_target, rear_target_reason);
      PlcOpenContinuation continuation = PlcOpenContinuation::FULL_INVENTORY_SAME_SIDE_NEXT_SEARCH;
      std::string context = "advance same-side next target before search";
      if (use_rear_target) {
        std::string rear_prepare_reason;
        if (!prepare_full_inventory_rear_target_context(
            finished_target,
            next_target,
            rear_prepare_reason))
        {
          fail_full_inventory("准备后方目标处理失败: " + rear_prepare_reason);
          return;
        }
        continuation = PlcOpenContinuation::FULL_INVENTORY_REAR_TARGET_REORIENT;
        context = "advance rear-target same physical unit before reorient";
      }
      if (!begin_plc_open_wait_for_target(
          current_target_cabinet_,
          continuation,
          context))
      {
        return;
      }
      if (!plc_http_enabled_) {
        if (use_rear_target) {
          start_full_inventory_rear_target_reorient();
        } else {
          start_full_inventory_same_side_next_search();
        }
      }
      return;
    }

    clear_full_inventory_rear_target_context("route restart or non same-side transition");
    publish_full_inventory_log(
      "next_target=" + std::to_string(next_target) +
      " route restart side=" + next_side);
    if (!begin_plc_open_wait_for_target(
        current_target_cabinet_,
        PlcOpenContinuation::FULL_INVENTORY_ADVANCE_ROUTE,
        "advance next target before route"))
    {
      return;
    }
    if (!plc_http_enabled_) {
      (void)start_full_inventory_target_route("advance next target");
    }
  }

  bool is_side_row_sequence_request(const std::vector<int> & cabinets) const
  {
    std::vector<int> first_gap_full = single_cabinet_side_row_first_gap_scan_sequence_;
    std::vector<int> full = single_cabinet_side_row_first_gap_scan_sequence_;
    full.insert(
      full.end(),
      single_cabinet_side_row_second_gap_scan_sequence_.begin(),
      single_cabinet_side_row_second_gap_scan_sequence_.end());
    return int_vectors_equal(cabinets, first_gap_full) || int_vectors_equal(cabinets, full);
  }

  bool is_valid_single_cabinet_side_row_request(
    const std::string & gap_id,
    const std::vector<int> & scan_cabinets,
    bool run_full_inventory) const
  {
    if (!single_cabinet_side_row_enabled_ || run_full_inventory) {
      return false;
    }
    if (agv_inventory_system::trim(gap_id) != single_cabinet_side_row_first_gap_) {
      return false;
    }
    return is_side_row_sequence_request(scan_cabinets);
  }

  std::string side_row_reject_message() const
  {
    std::vector<int> full = single_cabinet_side_row_first_gap_scan_sequence_;
    full.insert(
      full.end(),
      single_cabinet_side_row_second_gap_scan_sequence_.begin(),
      single_cabinet_side_row_second_gap_scan_sequence_.end());
    return
      "side row inventory only supports " + single_cabinet_side_row_name_ +
      " starting from " + single_cabinet_side_row_first_gap_ +
      " with scan_cabinets=" + cabinet_unit_to_string(full);
  }

  std::string active_single_cabinet_gap_id() const
  {
    if (single_cabinet_side_row_active_ && !single_cabinet_active_gap_id_.empty()) {
      return single_cabinet_active_gap_id_;
    }
    return single_cabinet_motion_target_gap_;
  }

  int active_single_cabinet_scan_cabinet() const
  {
    if (single_cabinet_side_row_active_ && single_cabinet_current_scan_cabinet_ > 0) {
      return single_cabinet_current_scan_cabinet_;
    }
    return single_cabinet_motion_target_cabinet_;
  }

  bool prepare_single_cabinet_target_cabinet(int cabinet_id, std::string & reason)
  {
    targets_ = {std::to_string(cabinet_id)};
    current_target_index_ = 0;
    if (!load_configs_and_prepare_current_target(reason)) {
      return false;
    }
    return true;
  }

  bool is_valid_single_cabinet_motion_request(
    const std::string & gap_id,
    const std::vector<int> & scan_cabinets,
    bool run_full_inventory) const
  {
    if (run_full_inventory) {
      return false;
    }
    if (agv_inventory_system::trim(gap_id) != single_cabinet_motion_target_gap_) {
      return false;
    }
    if (scan_cabinets.size() != 1U) {
      return false;
    }
    return scan_cabinets[0] == single_cabinet_motion_target_cabinet_;
  }

  bool start_single_cabinet_side_row_context(
    const std::string & gap_id,
    const std::vector<int> & scan_cabinets,
    InventoryGapPlan & plan,
    std::string & reason)
  {
    reason.clear();
    if (single_cabinet_side_row_first_gap_scan_sequence_.size() < 2U ||
      single_cabinet_side_row_second_gap_scan_sequence_.size() < 2U)
    {
      reason = "side-row 配置至少需要 first_gap/second_gap 各两个扫描柜号";
      return false;
    }
    if (!is_valid_single_cabinet_side_row_request(gap_id, scan_cabinets, false)) {
      reason = side_row_reject_message();
      return false;
    }

    reset_single_cabinet_side_row_context();
    single_cabinet_side_row_requested_sequence_ = scan_cabinets;

    std::vector<int> full = single_cabinet_side_row_first_gap_scan_sequence_;
    full.insert(
      full.end(),
      single_cabinet_side_row_second_gap_scan_sequence_.begin(),
      single_cabinet_side_row_second_gap_scan_sequence_.end());
    single_cabinet_side_row_full_sequence_ =
      int_vectors_equal(single_cabinet_side_row_requested_sequence_, full);
    single_cabinet_side_row_active_ = true;
    single_cabinet_side_row_phase_ = SingleCabinetSideRowPhase::FIRST_PRIMARY_SCAN;
    single_cabinet_active_gap_id_ = single_cabinet_side_row_first_gap_;
    single_cabinet_current_scan_cabinet_ = single_cabinet_side_row_first_gap_scan_sequence_.front();
    single_cabinet_adjusted_scan_cabinet_ =
      single_cabinet_side_row_first_gap_scan_sequence_.size() > 1U ?
      single_cabinet_side_row_first_gap_scan_sequence_[1] : -1;
    single_cabinet_next_gap_target_cabinet_ = single_cabinet_side_row_corridor_transfer_target_cabinet_;

    plan.gap_id = single_cabinet_active_gap_id_;
    plan.scan_cabinets = single_cabinet_side_row_requested_sequence_;
    return true;
  }

  void publish_single_cabinet_side_row_log(const std::string & text)
  {
    publish_single_cabinet_log("[side_row] " + text);
  }

  bool prepare_single_cabinet_target_search(int cabinet_id, const std::string & context)
  {
    single_cabinet_current_scan_cabinet_ = cabinet_id;
    std::string reason;
    if (!prepare_single_cabinet_target_cabinet(cabinet_id, reason)) {
      fail_single_cabinet_motion(context + "失败: " + reason);
      return false;
    }

    mission_active_ = true;
    cancel_requested_ = false;
    target_visible_ = false;
    has_distance_ = false;
    latest_distance_ = 0.0;
    tracking_stable_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_target_seen_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    reset_wait_gap_runtime();
    reset_entry_gap_runtime();
    publish_gap_context();
    begin_search_gap_flow();
    return true;
  }

  void reset_single_cabinet_scan_runtime()
  {
    single_cabinet_scan_steps_.clear();
    single_cabinet_scan_step_index_ = 0;
    single_cabinet_scan_cabinet_ = -1;
    single_cabinet_scan_active_ = false;
    single_cabinet_scan_step_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

    single_cabinet_grid_have_previous_depth_ = false;
    single_cabinet_grid_previous_cabinet_ = -1;
    single_cabinet_grid_previous_layer_ = -1;
    single_cabinet_grid_previous_depth_ = -1;
    single_cabinet_grid_move_active_ = false;
    single_cabinet_grid_move_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    single_cabinet_grid_move_target_distance_ = 0.0;
    single_cabinet_grid_move_cmd_speed_ = 0.0;
    single_cabinet_grid_move_step_ = agv_inventory_system::ScanStep{};
    single_cabinet_grid_move_start_pose_ = Pose2D{};
    inventory_scanner_.reset();
    reset_lift_runtime();
  }

  bool begin_single_cabinet_scan_runtime(
    int cabinet_id,
    InGapScanRuntimeMode mode = InGapScanRuntimeMode::SINGLE_CABINET)
  {
    single_cabinet_scan_steps_ = scan_sequence_generator_.generateCabinetSnakeSequence(
      cabinet_id,
      scan_layers_,
      scan_depth_count_);
    if (single_cabinet_scan_steps_.empty()) {
      fail_in_gap_scan_runtime(mode, "生成扫描序列为空: cabinet=" + std::to_string(cabinet_id));
      return false;
    }

    single_cabinet_scan_step_index_ = 0;
    single_cabinet_scan_cabinet_ = cabinet_id;
    single_cabinet_scan_active_ = true;
    single_cabinet_scan_step_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    inventory_scanner_.reset();
    reset_lift_runtime();
    if (mode == InGapScanRuntimeMode::FULL_INVENTORY) {
      const int initial_depth = std::max(1, current_target_depth_index_);
      single_cabinet_grid_have_previous_depth_ = true;
      single_cabinet_grid_previous_cabinet_ = cabinet_id;
      single_cabinet_grid_previous_layer_ = 1;
      single_cabinet_grid_previous_depth_ = initial_depth;
    } else {
      single_cabinet_grid_have_previous_depth_ = false;
      single_cabinet_grid_previous_cabinet_ = -1;
      single_cabinet_grid_previous_layer_ = -1;
      single_cabinet_grid_previous_depth_ = -1;
    }
    single_cabinet_grid_move_active_ = false;
    single_cabinet_grid_move_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

    publish_in_gap_scan_log(
      mode,
      "in gap, start runtime scan cabinet=" + std::to_string(cabinet_id) +
      " step_count=" + std::to_string(single_cabinet_scan_steps_.size()) +
      " previous_depth=" + std::to_string(single_cabinet_grid_previous_depth_));
    RCLCPP_INFO(
      get_logger(),
      "[mission_manager][%s][scan_runtime] start cabinet=%d step_count=%zu "
      "previous_depth=%d grid_motion=%s spacing=%.2f speed=%.3f timeout=%.2f",
      in_gap_scan_mode_label(mode),
      cabinet_id,
      single_cabinet_scan_steps_.size(),
      single_cabinet_grid_previous_depth_,
      single_cabinet_grid_motion_enabled_ ? "true" : "false",
      single_cabinet_grid_spacing_m_,
      single_cabinet_grid_move_speed_,
      single_cabinet_grid_move_timeout_sec_);
    return true;
  }

  bool inventory_device_step_started(
    const agv_inventory_system::ScanStep & step,
    InGapScanRuntimeMode mode)
  {
    if (single_cabinet_scan_step_start_time_.nanoseconds() != 0) {
      return true;
    }

    single_cabinet_scan_step_start_time_ = this->now();
    const std::string step_type =
      agv_inventory_system::ScanSequenceGenerator::stepTypeToString(step.step_type);
    std::string action_text = "device action";
    if (step.step_type == agv_inventory_system::ScanStepType::SCAN_GRID) {
      action_text = "scan grid";
      if (!inventory_scanner_.start_grid_scan(
          step.cabinet_id,
          1,
          step.layer_index,
          step.depth_index))
      {
        fail_in_gap_scan_runtime(mode, "启动扫描失败");
        return true;
      }
    } else if (step.step_type == agv_inventory_system::ScanStepType::MOVE_LIFT_TO_LEVEL) {
      action_text = "move lift to level";
      if (!start_lift_step(step, mode)) {
        fail_in_gap_scan_runtime(mode, "启动升降杆失败");
        return true;
      }
    } else if (step.step_type == agv_inventory_system::ScanStepType::MOVE_LIFT_HOME) {
      action_text = "move lift home";
      if (!start_lift_step(step, mode)) {
        fail_in_gap_scan_runtime(mode, "启动升降杆回原点失败");
        return true;
      }
    }
    RCLCPP_INFO(
      get_logger(),
      "[mission_manager][%s][scan_runtime] %s step_index=%zu/%zu step=%s "
      "cabinet=%d layer=%d depth=%d",
      in_gap_scan_mode_label(mode),
      action_text.c_str(),
      single_cabinet_scan_step_index_,
      single_cabinet_scan_steps_.size(),
      step_type.c_str(),
      step.cabinet_id,
      step.layer_index,
      step.depth_index);
    publish_in_gap_scan_log(
      mode,
      action_text + " step=" + step_type +
      " cabinet=" + std::to_string(step.cabinet_id) +
      " layer=" + std::to_string(step.layer_index) +
      " depth=" + std::to_string(step.depth_index) +
      " step_index=" + std::to_string(single_cabinet_scan_step_index_));
    return false;
  }

  void reset_lift_runtime()
  {
    lift_step_active_ = false;
    lift_step_future_ =
      std::shared_future<agv_inventory_system::srv::LiftMoveTimed::Response::SharedPtr>();
    lift_step_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    lift_step_service_name_.clear();
    lift_step_description_.clear();
  }

  bool start_lift_step(
    const agv_inventory_system::ScanStep & step,
    InGapScanRuntimeMode mode)
  {
    (void)mode;
    if (!lift_enabled_) {
      reset_lift_runtime();
      RCLCPP_INFO(
        get_logger(),
        "[mission_manager][lift] disabled, skip step=%s layer=%d",
        agv_inventory_system::ScanSequenceGenerator::stepTypeToString(step.step_type).c_str(),
        step.layer_index);
      return true;
    }

    rclcpp::Client<agv_inventory_system::srv::LiftMoveTimed>::SharedPtr client;
    double duration_sec = lift_up_duration_sec_;
    std::string direction = "up";
    if (step.step_type == agv_inventory_system::ScanStepType::MOVE_LIFT_HOME) {
      client = lift_home_client_;
      duration_sec = lift_down_duration_sec_;
      direction = "down";
      lift_step_service_name_ = lift_home_service_name_;
      lift_step_description_ = "home";
    } else {
      client = lift_up_client_;
      duration_sec = lift_up_duration_sec_;
      direction = "up";
      lift_step_service_name_ = lift_up_service_name_;
      lift_step_description_ = "move_to_level_" + std::to_string(step.layer_index);
    }

    if (!client || !client->service_is_ready()) {
      RCLCPP_ERROR(
        get_logger(),
        "[mission_manager][lift] service unavailable: %s",
        lift_step_service_name_.c_str());
      return false;
    }

    auto request = std::make_shared<agv_inventory_system::srv::LiftMoveTimed::Request>();
    request->direction = direction;
    request->duration_sec = static_cast<float>(std::max(0.0, duration_sec));
    lift_step_future_ = client->async_send_request(request).share();
    lift_step_active_ = true;
    lift_step_start_time_ = this->now();
    RCLCPP_INFO(
      get_logger(),
      "[mission_manager][lift] start service=%s direction=%s duration=%.2f step=%s layer=%d",
      lift_step_service_name_.c_str(),
      direction.c_str(),
      duration_sec,
      agv_inventory_system::ScanSequenceGenerator::stepTypeToString(step.step_type).c_str(),
      step.layer_index);
    return true;
  }

  bool cache_inventory_result_for_upload(
    const agv_inventory_system::ScanStep & step,
    InGapScanRuntimeMode mode)
  {
    const int level = step.layer_index > 0 ? step.layer_index : 1;
    const int grid = step.depth_index > 0 ? step.depth_index : 1;
    const std::string location_rfid =
      make_inventory_location_rfid(step.cabinet_id, level, grid);
    const auto & scan_output = inventory_scanner_.last_scan_output();

    std::vector<std::string> rfids = scan_output.rfids;

    if (inventory_upload_batch_locations_.find(location_rfid) !=
      inventory_upload_batch_locations_.end())
    {
      RCLCPP_WARN(
        get_logger(),
        "[mission_manager][%s][RFID][batch] duplicate locationRfid, overwrite cached result "
        "cabinet=%d level=%d grid=%d locationRfid=%s rfid_count=%zu",
        in_gap_scan_mode_label(mode),
        step.cabinet_id,
        level,
        grid,
        location_rfid.c_str(),
        rfids.size());
      for (auto & item : inventory_upload_batch_) {
        if (item.location_rfid == location_rfid) {
          item.rfids = rfids;
          break;
        }
      }
    } else {
      agv_inventory_system::InventoryUploadItem item;
      item.location_rfid = location_rfid;
      item.rfids = rfids;
      inventory_upload_batch_.push_back(item);
      inventory_upload_batch_locations_.insert(location_rfid);
    }

    RCLCPP_INFO(
      get_logger(),
      "[mission_manager][%s][RFID][batch] cached locationRfid=%s rfid_count=%zu "
      "batch_count=%zu source=%s fallback=%s result=%s",
      in_gap_scan_mode_label(mode),
      location_rfid.c_str(),
      rfids.size(),
      inventory_upload_batch_.size(),
      inventory_scan_output_source_label(scan_output.source),
      "false",
      inventory_scanner_.last_scan_result().c_str());
    write_rfid_scan_cell_cached_log(
      mode,
      step.cabinet_id,
      level,
      grid,
      location_rfid,
      rfids,
      scan_output);
    return true;
  }

  void warn_rfid_local_log_failed(const std::string & event, const std::string & error_message)
  {
    RCLCPP_WARN(
      get_logger(),
      "[mission_manager][RFID][local_log] write failed event=%s path=%s error=%s",
      event.c_str(),
      rfid_local_log_path_.empty() ? "<empty>" : rfid_local_log_path_.c_str(),
      error_message.c_str());
  }

  void write_rfid_scan_cell_cached_log(
    InGapScanRuntimeMode mode,
    int cabinet_id,
    int level,
    int grid,
    const std::string & location_rfid,
    const std::vector<std::string> & rfids,
    const agv_inventory_system::InventoryScanOutput & scan_output)
  {
    agv_inventory_system::RfidScanCellLogRecord record;
    record.mission_mode = rfid_local_log_mission_mode_label(mode);
    record.cabinet = cabinet_id;
    record.layer = level;
    record.grid = grid;
    record.location_rfid = location_rfid;
    record.rfids = rfids;
    record.reader_mode = rfid_reader_mode_;
    record.source = rfid_local_log_source_label(scan_output.source);
    record.fallback_to_placeholder = false;
    record.scan_success = inventory_scan_output_source_succeeded(scan_output.source);
    record.batch_item_count = inventory_upload_batch_.size();

    std::string error_message;
    if (!rfid_scan_log_writer_.appendScanCellCached(record, &error_message)) {
      warn_rfid_local_log_failed("scan_cell_cached", error_message);
    }
  }

  void write_rfid_batch_prepare_log(const std::string & reason)
  {
    std::string error_message;
    if (!rfid_scan_log_writer_.appendBatchUploadPrepare(
        reason,
        inventory_upload_batch_.size(),
        rfid_upload_url_,
        &error_message))
    {
      warn_rfid_local_log_failed("final_batch_upload_prepare", error_message);
    }
  }

  void write_rfid_batch_success_log(std::size_t item_count)
  {
    std::string error_message;
    if (!rfid_scan_log_writer_.appendBatchUploadSuccess(item_count, &error_message)) {
      warn_rfid_local_log_failed("final_batch_upload_success", error_message);
    }
  }

  void write_rfid_batch_failed_log(std::size_t item_count)
  {
    std::string error_message;
    if (!rfid_scan_log_writer_.appendBatchUploadFailed(
        item_count,
        rfid_upload_fail_policy_,
        &error_message))
    {
      warn_rfid_local_log_failed("final_batch_upload_failed", error_message);
    }
  }

  void write_rfid_batch_skipped_log(const std::string & reason, std::size_t item_count)
  {
    std::string error_message;
    if (!rfid_scan_log_writer_.appendBatchUploadSkipped(reason, item_count, &error_message)) {
      warn_rfid_local_log_failed("final_batch_upload_skipped", error_message);
    }
  }

  bool flush_inventory_upload_batch(InGapScanRuntimeMode mode, const std::string & reason)
  {
    if (inventory_upload_batch_finalized_) {
      RCLCPP_INFO(
        get_logger(),
        "[mission_manager][%s][RFID][batch] final upload already handled, skip reason=%s",
        in_gap_scan_mode_label(mode),
        reason.c_str());
      return true;
    }

    if (inventory_upload_batch_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "[mission_manager][%s][RFID][batch] empty at mission complete, skip upload "
        "reason=%s upload_enabled=%s reader_mode=%s "
        "serial_device=%s scan_duration=%.2f",
        in_gap_scan_mode_label(mode),
        reason.c_str(),
        rfid_upload_enabled_ ? "true" : "false",
        rfid_reader_mode_.c_str(),
        rfid_serial_device_.empty() ? "<empty>" : rfid_serial_device_.c_str(),
        rfid_scan_duration_sec_);
      inventory_upload_batch_finalized_ = true;
      return true;
    }

    RCLCPP_INFO(
      get_logger(),
      "[mission_manager][%s][RFID][batch] all requested cabinets complete; "
      "prepare final RFID batch upload item_count=%zu url=%s reason=%s",
      in_gap_scan_mode_label(mode),
      inventory_upload_batch_.size(),
      rfid_upload_url_.c_str(),
      reason.c_str());
    write_rfid_batch_prepare_log(reason);

    if (!rfid_upload_enabled_) {
      const std::size_t skipped_item_count = inventory_upload_batch_.size();
      RCLCPP_WARN(
        get_logger(),
        "[mission_manager][%s][RFID][batch] upload disabled, drop cached item_count=%zu",
        in_gap_scan_mode_label(mode),
        skipped_item_count);
      write_rfid_batch_skipped_log("rfid_upload_disabled", skipped_item_count);
      web_api_client_.writeRfidUploadStatus(
        false,
        0,
        "上传已跳过: rfid_upload_enabled=false",
        "mission_manager");
      clear_inventory_upload_batch("upload disabled");
      inventory_upload_batch_finalized_ = true;
      return true;
    }

    const auto upload_status =
      web_api_client_.reportInventoryResultsWithStatus(inventory_upload_batch_, reason);
    web_api_client_.writeRfidUploadStatus(
      upload_status.display_success,
      upload_status.status_code,
      upload_status.message,
      "mission_manager");
    if (upload_status.success) {
      const std::size_t uploaded_item_count = inventory_upload_batch_.size();
      RCLCPP_INFO(
        get_logger(),
        "[mission_manager][%s][RFID][batch] upload success item_count=%zu url=%s "
        "display_success=%s status_code=%ld message=%s",
        in_gap_scan_mode_label(mode),
        uploaded_item_count,
        rfid_upload_url_.c_str(),
        upload_status.display_success ? "true" : "false",
        upload_status.status_code,
        upload_status.message.c_str());
      write_rfid_batch_success_log(uploaded_item_count);
      clear_inventory_upload_batch("upload_done");
      inventory_upload_batch_finalized_ = true;
      return true;
    }

    const std::size_t failed_item_count = inventory_upload_batch_.size();
    RCLCPP_ERROR(
      get_logger(),
      "[mission_manager][%s][RFID][batch] upload failed item_count=%zu url=%s fail_policy=%s "
      "status_code=%ld message=%s",
      in_gap_scan_mode_label(mode),
      failed_item_count,
      rfid_upload_url_.c_str(),
      rfid_upload_fail_policy_.c_str(),
      upload_status.status_code,
      upload_status.message.c_str());
    write_rfid_batch_failed_log(failed_item_count);
    if (rfid_upload_fail_policy_ == "continue_without_upload") {
      RCLCPP_WARN(
        get_logger(),
        "[mission_manager][%s][RFID][batch] continue_without_upload, drop cached data after failure "
        "item_count=%zu",
        in_gap_scan_mode_label(mode),
        failed_item_count);
      clear_inventory_upload_batch("upload_failed_continue");
      inventory_upload_batch_finalized_ = true;
      return true;
    }

    clear_inventory_upload_batch("upload_failed_error");
    return false;
  }

  InGapScanRuntimeMode current_inventory_upload_mode() const
  {
    return full_inventory_active_ ?
      InGapScanRuntimeMode::FULL_INVENTORY :
      InGapScanRuntimeMode::SINGLE_CABINET;
  }

  bool flush_inventory_upload_batch_for_mission_complete(const std::string & reason)
  {
    return flush_inventory_upload_batch(current_inventory_upload_mode(), reason);
  }

  bool check_lift_step_finished(
    const agv_inventory_system::ScanStep & step,
    InGapScanRuntimeMode mode)
  {
    if (!lift_enabled_ || !lift_step_active_) {
      return true;
    }

    if (lift_step_start_time_.nanoseconds() != 0 &&
      (this->now() - lift_step_start_time_).seconds() > std::max(0.1, lift_service_timeout_sec_))
    {
      fail_lift_step(mode, "升降杆服务超时: " + lift_step_description_);
      return false;
    }

    if (!lift_step_future_.valid()) {
      fail_lift_step(mode, "升降杆服务 future 无效: " + lift_step_description_);
      return false;
    }

    if (lift_step_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      return false;
    }

    const auto response = lift_step_future_.get();
    if (!response || !response->success) {
      std::string reason = "升降杆动作失败: " + lift_step_description_;
      if (response) {
        reason += " message=" + response->message;
      }
      fail_lift_step(mode, reason);
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "[mission_manager][lift] finished %s step=%s layer=%d height=%.3f message=%s",
      lift_step_description_.c_str(),
      agv_inventory_system::ScanSequenceGenerator::stepTypeToString(step.step_type).c_str(),
      step.layer_index,
      response->estimated_height_m,
      response->message.c_str());
    reset_lift_runtime();
    return true;
  }

  void fail_lift_step(InGapScanRuntimeMode mode, const std::string & reason)
  {
    request_lift_safety_stop("lift failure");
    reset_lift_runtime();
    fail_in_gap_scan_runtime(mode, reason);
  }

  void request_lift_safety_stop(const std::string & reason)
  {
    if (!lift_enabled_) {
      return;
    }
    if (lift_stop_client_ && lift_stop_client_->service_is_ready()) {
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      (void)lift_stop_client_->async_send_request(request);
      RCLCPP_WARN(get_logger(), "[mission_manager][lift] requested stop: %s", reason.c_str());
    } else {
      RCLCPP_WARN(
        get_logger(),
        "[mission_manager][lift] stop service unavailable during %s",
        reason.c_str());
    }
    if (lift_all_off_client_ && lift_all_off_client_->service_is_ready()) {
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      (void)lift_all_off_client_->async_send_request(request);
      RCLCPP_WARN(get_logger(), "[mission_manager][lift] requested all_off: %s", reason.c_str());
    }
  }

  bool execute_inventory_device_step(
    const agv_inventory_system::ScanStep & step,
    InGapScanRuntimeMode mode = InGapScanRuntimeMode::SINGLE_CABINET)
  {
    const bool already_started = inventory_device_step_started(step, mode);
    (void)already_started;

    if (step.step_type == agv_inventory_system::ScanStepType::SCAN_GRID) {
      inventory_scanner_.update();
      if (!inventory_scanner_.is_scan_finished()) {
        return false;
      }
      if (!inventory_scanner_.scan_success()) {
        fail_in_gap_scan_runtime(
          mode,
          "扫描失败: cabinet=" + std::to_string(step.cabinet_id) +
          " layer=" + std::to_string(step.layer_index) +
          " depth=" + std::to_string(step.depth_index));
        return false;
      }
      if (!cache_inventory_result_for_upload(step, mode)) {
        const int level = step.layer_index > 0 ? step.layer_index : 1;
        const int grid = step.depth_index > 0 ? step.depth_index : 1;
        fail_in_gap_scan_runtime(
          mode,
          "缓存扫描结果失败: cabinet=" + std::to_string(step.cabinet_id) +
          " level=" + std::to_string(level) +
          " grid=" + std::to_string(grid));
        return false;
      }
    } else {
      if (!check_lift_step_finished(step, mode)) {
        return false;
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "[mission_manager][%s][scan_runtime] device step finished step_index=%zu/%zu "
      "cabinet=%d layer=%d depth=%d step=%s",
      in_gap_scan_mode_label(mode),
      single_cabinet_scan_step_index_,
      single_cabinet_scan_steps_.size(),
      step.cabinet_id,
      step.layer_index,
      step.depth_index,
      agv_inventory_system::ScanSequenceGenerator::stepTypeToString(step.step_type).c_str());
    single_cabinet_scan_step_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    return true;
  }

  bool single_cabinet_grid_move_step_matches(const agv_inventory_system::ScanStep & step) const
  {
    return
      single_cabinet_grid_move_step_.cabinet_id == step.cabinet_id &&
      single_cabinet_grid_move_step_.layer_index == step.layer_index &&
      single_cabinet_grid_move_step_.depth_index == step.depth_index &&
      single_cabinet_grid_move_step_.step_type == step.step_type;
  }

  bool execute_single_cabinet_grid_move_step(
    const agv_inventory_system::ScanStep & step,
    InGapScanRuntimeMode mode = InGapScanRuntimeMode::SINGLE_CABINET)
  {
    if (!single_cabinet_grid_motion_enabled_) {
      if (mode == InGapScanRuntimeMode::FULL_INVENTORY) {
        fail_full_inventory(
          "full inventory in-gap MOVE_TO_GRID requires single_cabinet_grid_motion_enabled=true");
        return false;
      }
      return true;
    }

    const bool cabinet_changed =
      !single_cabinet_grid_have_previous_depth_ ||
      single_cabinet_grid_previous_cabinet_ != step.cabinet_id;
    const bool layer_changed =
      single_cabinet_grid_have_previous_depth_ &&
      single_cabinet_grid_previous_layer_ != step.layer_index;
    if (cabinet_changed ||
      (layer_changed && single_cabinet_grid_move_return_between_layers_))
    {
      single_cabinet_grid_have_previous_depth_ = true;
      single_cabinet_grid_previous_cabinet_ = step.cabinet_id;
      single_cabinet_grid_previous_layer_ = step.layer_index;
      single_cabinet_grid_previous_depth_ = step.depth_index;
      RCLCPP_INFO(
        get_logger(),
        "[mission_manager][%s][grid_move] step_index=%zu/%zu cabinet=%d layer=%d "
        "target_depth=%d first depth, no move",
        in_gap_scan_mode_label(mode),
        single_cabinet_scan_step_index_,
        single_cabinet_scan_steps_.size(),
        step.cabinet_id,
        step.layer_index,
        step.depth_index);
      publish_in_gap_scan_log(
        mode,
        "[grid_move] cabinet=" + std::to_string(step.cabinet_id) +
        " layer=" + std::to_string(step.layer_index) +
        " depth=" + std::to_string(step.depth_index) +
        " step_index=" + std::to_string(single_cabinet_scan_step_index_) +
        " first depth, no move");
      return true;
    }

    const int delta_depth = step.depth_index - single_cabinet_grid_previous_depth_;
    if (delta_depth == 0) {
      const int previous_depth = single_cabinet_grid_previous_depth_;
      single_cabinet_grid_previous_layer_ = step.layer_index;
      single_cabinet_grid_previous_depth_ = step.depth_index;
      RCLCPP_INFO(
        get_logger(),
        "[mission_manager][%s][grid_move] step_index=%zu/%zu cabinet=%d layer=%d "
        "previous_depth=%d target_depth=%d delta_depth=0 no move",
        in_gap_scan_mode_label(mode),
        single_cabinet_scan_step_index_,
        single_cabinet_scan_steps_.size(),
        step.cabinet_id,
        step.layer_index,
        previous_depth,
        step.depth_index);
      return true;
    }

    const double spacing = std::max(0.0, single_cabinet_grid_spacing_m_);
    const double target_distance = std::abs(delta_depth) * spacing;
    if (target_distance <= 1e-4) {
      const int previous_depth = single_cabinet_grid_previous_depth_;
      single_cabinet_grid_previous_layer_ = step.layer_index;
      single_cabinet_grid_previous_depth_ = step.depth_index;
      RCLCPP_INFO(
        get_logger(),
        "[mission_manager][%s][grid_move] step_index=%zu/%zu cabinet=%d layer=%d "
        "previous_depth=%d target_depth=%d delta_depth=%d distance=%.2f no move",
        in_gap_scan_mode_label(mode),
        single_cabinet_scan_step_index_,
        single_cabinet_scan_steps_.size(),
        step.cabinet_id,
        step.layer_index,
        previous_depth,
        step.depth_index,
        delta_depth,
        target_distance);
      return true;
    }

    const double speed_abs = std::clamp(std::abs(single_cabinet_grid_move_speed_), 0.0, 0.05);
    if (speed_abs <= 1e-4) {
      fail_in_gap_scan_runtime(
        mode,
        "深度格移动速度非法: " + std::to_string(single_cabinet_grid_move_speed_));
      return false;
    }
    const bool moving_deeper = delta_depth > 0;
    const double signed_speed = moving_deeper ?
      apply_entry_motion_direction(speed_abs) : -apply_entry_motion_direction(speed_abs);

    if (!single_cabinet_grid_move_active_ || !single_cabinet_grid_move_step_matches(step)) {
      std::string reason;
      if (!current_odom_ready_for_entry(reason)) {
        fail_in_gap_scan_runtime(mode, "深度格移动前里程计异常: " + reason);
        return false;
      }
      const Pose2D current = current_pose_2d();
      if (!current.valid) {
        fail_in_gap_scan_runtime(mode, "深度格移动前当前位姿无效");
        return false;
      }

      single_cabinet_grid_move_active_ = true;
      single_cabinet_grid_move_step_ = step;
      single_cabinet_grid_move_start_pose_ = current;
      single_cabinet_grid_move_target_distance_ = target_distance;
      single_cabinet_grid_move_cmd_speed_ = signed_speed;
      single_cabinet_grid_move_start_time_ = this->now();
      reset_segment_distance();

      RCLCPP_INFO(
        get_logger(),
        "[mission_manager][%s][grid_move] step_index=%zu/%zu cabinet=%d layer=%d "
        "previous_depth=%d target_depth=%d delta_depth=%d target_distance=%.2f "
        "entry_motion_mode=%s moving_deeper=%d speed=%.3f timeout=%.2f",
        in_gap_scan_mode_label(mode),
        single_cabinet_scan_step_index_,
        single_cabinet_scan_steps_.size(),
        step.cabinet_id,
        step.layer_index,
        single_cabinet_grid_previous_depth_,
        step.depth_index,
        delta_depth,
        single_cabinet_grid_move_target_distance_,
        entry_motion_mode_to_string(entry_motion_mode_).c_str(),
        moving_deeper ? 1 : 0,
        single_cabinet_grid_move_cmd_speed_,
        single_cabinet_grid_move_timeout_sec_);
      RCLCPP_INFO(
        get_logger(),
        "[mission_manager][%s][grid_move] start_pose x=%.3f y=%.3f yaw=%.3f frame=%s",
        in_gap_scan_mode_label(mode),
        current.x,
        current.y,
        current.yaw,
        current.frame_id.c_str());
      publish_in_gap_scan_log(
        mode,
        "[grid_move] cabinet=" + std::to_string(step.cabinet_id) +
        " layer=" + std::to_string(step.layer_index) +
        " previous_depth=" + std::to_string(single_cabinet_grid_previous_depth_) +
        " target_depth=" + std::to_string(step.depth_index) +
        " delta_depth=" + std::to_string(delta_depth) +
        " entry_motion_mode=" + entry_motion_mode_to_string(entry_motion_mode_) +
        " moving_deeper=" + std::string(moving_deeper ? "true" : "false") +
        " target_distance=" + format_fixed(single_cabinet_grid_move_target_distance_, 2) +
        " speed=" + format_fixed(single_cabinet_grid_move_cmd_speed_, 3) +
        " step_index=" + std::to_string(single_cabinet_scan_step_index_));
      return false;
    }

    std::string reason;
    if (!current_odom_ready_for_entry(reason)) {
      publish_stop();
      fail_in_gap_scan_runtime(mode, "深度格移动中里程计异常: " + reason);
      return false;
    }

    const double elapsed = (this->now() - single_cabinet_grid_move_start_time_).seconds();
    const double traveled = segment_distance();
    const double timeout = std::max(0.1, single_cabinet_grid_move_timeout_sec_);
    if (elapsed > timeout) {
      publish_stop();
      RCLCPP_ERROR(
        get_logger(),
        "[mission_manager][%s][grid_move] timeout step_index=%zu/%zu cabinet=%d "
        "layer=%d previous_depth=%d target_depth=%d delta_depth=%d traveled=%.2f "
        "target=%.2f elapsed=%.2f",
        in_gap_scan_mode_label(mode),
        single_cabinet_scan_step_index_,
        single_cabinet_scan_steps_.size(),
        step.cabinet_id,
        step.layer_index,
        single_cabinet_grid_previous_depth_,
        step.depth_index,
        delta_depth,
        traveled,
        single_cabinet_grid_move_target_distance_,
        elapsed);
      fail_in_gap_scan_runtime(
        mode,
        "深度格移动超时: cabinet=" + std::to_string(step.cabinet_id) +
        " layer=" + std::to_string(step.layer_index) +
        " depth=" + std::to_string(step.depth_index));
      return false;
    }

    if (traveled >= single_cabinet_grid_move_target_distance_) {
      publish_stop();
      const int previous_depth = single_cabinet_grid_previous_depth_;
      single_cabinet_grid_previous_layer_ = step.layer_index;
      single_cabinet_grid_previous_depth_ = step.depth_index;
      single_cabinet_grid_move_active_ = false;
      single_cabinet_grid_move_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      RCLCPP_INFO(
        get_logger(),
        "[mission_manager][%s][grid_move] move finished step_index=%zu/%zu cabinet=%d "
        "layer=%d previous_depth=%d target_depth=%d delta_depth=%d traveled=%.2f",
        in_gap_scan_mode_label(mode),
        single_cabinet_scan_step_index_,
        single_cabinet_scan_steps_.size(),
        step.cabinet_id,
        step.layer_index,
        previous_depth,
        step.depth_index,
        delta_depth,
        traveled);
      publish_in_gap_scan_log(
        mode,
        "[grid_move] move finished cabinet=" + std::to_string(step.cabinet_id) +
        " layer=" + std::to_string(step.layer_index) +
        " previous_depth=" + std::to_string(previous_depth) +
        " target_depth=" + std::to_string(step.depth_index) +
        " traveled=" + format_fixed(traveled, 2) +
        " step_index=" + std::to_string(single_cabinet_scan_step_index_));
      return true;
    }

    if (moving_deeper) {
      const auto safety = evaluate_entering_safety();
      if (safety.blocked) {
        publish_stop();
        fail_in_gap_scan_runtime(mode, "深度格深入被安全策略阻塞: " + safety.block_reason);
        return false;
      }
    } else {
      const double ultrasonic_range = min_ultrasonic_range();
      if (std::isfinite(ultrasonic_range) && ultrasonic_range < entry_ultrasonic_stop_distance_) {
        publish_stop();
        fail_in_gap_scan_runtime(
          mode,
          "深度格回浅被超声波安全策略阻塞 range=" + std::to_string(ultrasonic_range));
        return false;
      }
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = single_cabinet_grid_move_cmd_speed_;
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "[mission_manager][%s][grid_move][publish_cmd_vel] cmd_vel_source=GRID_MOVE "
      "step_index=%zu/%zu cabinet=%d "
      "previous_depth=%d target_depth=%d delta_depth=%d traveled=%.2f target=%.2f "
      "elapsed=%.2f entry_motion_mode=%s moving_deeper=%d signed_speed=%.3f "
      "published_cmd_linear_x=%.3f",
      in_gap_scan_mode_label(mode),
      single_cabinet_scan_step_index_,
      single_cabinet_scan_steps_.size(),
      step.cabinet_id,
      single_cabinet_grid_previous_depth_,
      step.depth_index,
      delta_depth,
      traveled,
      single_cabinet_grid_move_target_distance_,
      elapsed,
      entry_motion_mode_to_string(entry_motion_mode_).c_str(),
      moving_deeper ? 1 : 0,
      single_cabinet_grid_move_cmd_speed_,
      cmd.linear.x);
    return false;
  }

  bool execute_single_cabinet_scan_runtime_tick(
    int cabinet_id,
    InGapScanRuntimeMode mode = InGapScanRuntimeMode::SINGLE_CABINET)
  {
    if (!single_cabinet_scan_active_ || single_cabinet_scan_cabinet_ != cabinet_id) {
      if (!begin_single_cabinet_scan_runtime(cabinet_id, mode)) {
        return false;
      }
    }

    if (single_cabinet_scan_step_index_ >= single_cabinet_scan_steps_.size()) {
      reset_single_cabinet_scan_runtime();
      return true;
    }

    const auto & step = single_cabinet_scan_steps_[single_cabinet_scan_step_index_];
    bool step_done = false;
    switch (step.step_type) {
      case agv_inventory_system::ScanStepType::MOVE_TO_GRID:
        step_done = execute_single_cabinet_grid_move_step(step, mode);
        break;
      case agv_inventory_system::ScanStepType::SCAN_GRID:
        step_done = execute_inventory_device_step(step, mode);
        break;
      case agv_inventory_system::ScanStepType::MOVE_LIFT_TO_LEVEL:
      case agv_inventory_system::ScanStepType::MOVE_LIFT_HOME:
        step_done = execute_inventory_device_step(step, mode);
        break;
    }

    if (!step_done) {
      return false;
    }

    ++single_cabinet_scan_step_index_;
    if (single_cabinet_scan_step_index_ >= single_cabinet_scan_steps_.size()) {
      publish_in_gap_scan_log(
        mode,
        "in-gap runtime sequence complete cabinet=" + std::to_string(cabinet_id) +
        " batch cached item_count=" + std::to_string(inventory_upload_batch_.size()) +
        " not uploading until all requested cabinets complete");
      reset_single_cabinet_scan_runtime();
      return true;
    }

    return false;
  }

  void finish_single_cabinet_exit_gap()
  {
    publish_stop();
    robot_inside_gap_ = false;
    const bool finished_exit_entry_yaw_valid =
      entry_turn_start_yaw_valid_ && std::isfinite(entry_turn_start_yaw_);
    const double finished_exit_entry_yaw =
      finished_exit_entry_yaw_valid ? normalize_angle(entry_turn_start_yaw_) : 0.0;
    reset_entry_gap_runtime();
    clear_safe_exit_gap_recovery_context();
    single_cabinet_exit_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    single_cabinet_exit_phase_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    single_cabinet_exit_phase_ = SingleCabinetExitPhase::STRAIGHT_REVERSE;
    single_cabinet_exit_effective_timeout_sec_ = single_cabinet_exit_timeout_sec_;

    if (consume_pending_interrupt_after_exit()) {
      return;
    }

    if (full_inventory_active_) {
      full_inventory_last_exit_entry_yaw_valid_ = finished_exit_entry_yaw_valid;
      full_inventory_last_exit_entry_yaw_ = finished_exit_entry_yaw;
      publish_full_inventory_log(
        "[rear_target] exit gap captured entry_turn_start_yaw_valid=" +
        std::string(full_inventory_last_exit_entry_yaw_valid_ ? "1" : "0") +
        " entry_turn_start_yaw=" + format_fixed(full_inventory_last_exit_entry_yaw_, 4));
      set_state(
        State::FULL_INVENTORY_ADVANCE_NEXT_TARGET,
        "[FULL_INVENTORY] exit gap finished, advance sequence");
      advance_full_inventory_sequence();
      return;
    }

    const auto action = single_cabinet_after_exit_action_;
    single_cabinet_after_exit_action_ = SingleCabinetAfterExitAction::NONE;

    switch (action) {
      case SingleCabinetAfterExitAction::REENTER_ADJUSTED:
        set_single_cabinet_state(
          State::SINGLE_CABINET_REENTER_FOR_ADJUSTED_SCAN,
          "倒退出缝完成，准备再次入缝做临时位置调整");
        return;
      case SingleCabinetAfterExitAction::CORRIDOR_TRANSFER:
        set_single_cabinet_state(
          State::SINGLE_CABINET_CORRIDOR_TRANSFER,
          "倒退出缝完成，准备走廊转移到下一目标柜");
        return;
      case SingleCabinetAfterExitAction::CLOSE_AND_DONE:
        set_single_cabinet_state(State::REQUEST_CLOSE_GAP, "出缝完成，准备请求关柜");
        return;
      case SingleCabinetAfterExitAction::FINAL_CLOSE_AND_DONE:
        if (!single_cabinet_close_gap_after_final_exit_) {
          set_single_cabinet_state(State::DONE, "最终出缝完成，配置为不请求关柜");
          return;
        }
        set_single_cabinet_state(State::REQUEST_CLOSE_GAP, "最终出缝完成，准备请求关柜");
        return;
      case SingleCabinetAfterExitAction::NONE:
      default:
        fail_single_cabinet_motion("出缝完成后缺少下一步动作");
        return;
    }
  }

  void handle_single_cabinet_exit_gap_state()
  {
    std::string mode = single_cabinet_exit_mode_;
    std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    if (mode != "reverse") {
      if (full_inventory_active_) {
        fail_full_inventory("当前只支持 reverse 出缝模式: " + single_cabinet_exit_mode_);
      } else {
        fail_single_cabinet_motion("当前只支持 reverse 出缝模式: " + single_cabinet_exit_mode_);
      }
      return;
    }

    const double speed = std::clamp(std::abs(single_cabinet_exit_speed_), 0.0, 0.50);
    if (speed <= 1e-4) {
      if (full_inventory_active_) {
        fail_full_inventory("倒退出缝速度非法: " + std::to_string(single_cabinet_exit_speed_));
      } else {
        fail_single_cabinet_motion("倒退出缝速度非法: " + std::to_string(single_cabinet_exit_speed_));
      }
      return;
    }

    if (single_cabinet_exit_start_time_.nanoseconds() == 0) {
      const double measured_distance =
        std::isfinite(single_cabinet_last_entering_straight_distance_) &&
        single_cabinet_last_entering_straight_distance_ > 0.05 ?
        single_cabinet_last_entering_straight_distance_ + single_cabinet_exit_extra_distance_m_ :
        single_cabinet_exit_distance_m_;
      single_cabinet_exit_target_distance_ =
        std::max(0.05, std::isfinite(measured_distance) ? measured_distance : single_cabinet_exit_distance_m_);
      const double required_time = single_cabinet_exit_target_distance_ / speed;
      const double configured_timeout =
        std::isfinite(single_cabinet_exit_timeout_sec_) ? single_cabinet_exit_timeout_sec_ : 0.0;
      const double minimum_timeout = required_time + 5.0;
      single_cabinet_exit_effective_timeout_sec_ =
        std::max(std::max(0.1, configured_timeout), minimum_timeout);
      if (configured_timeout < minimum_timeout) {
        RCLCPP_WARN(
          get_logger(),
          "[mission_manager][single_cabinet][exit_gap] warning: timeout too short for distance/speed, "
          "required=%.2f configured=%.2f",
          required_time,
          configured_timeout);
        publish_motion_log(
          "[exit_gap] warning: timeout too short for distance/speed, required=" +
          format_seconds(required_time) +
          ", configured=" + format_seconds(configured_timeout));
      }
      reset_segment_distance();
      single_cabinet_exit_start_time_ = this->now();
      single_cabinet_exit_phase_start_time_ = single_cabinet_exit_start_time_;
      single_cabinet_exit_phase_ = SingleCabinetExitPhase::STRAIGHT_REVERSE;
      RCLCPP_INFO(
        get_logger(),
        "[mission_manager][single_cabinet][exit_gap] start reverse exit phase=%s distance=%.2f "
        "speed=%.3f timeout=%.2f turn_enabled=%s",
        single_cabinet_exit_phase_to_string(single_cabinet_exit_phase_).c_str(),
        single_cabinet_exit_target_distance_,
        speed,
        single_cabinet_exit_effective_timeout_sec_,
        single_cabinet_exit_turn_enabled_ ? "true" : "false");
      publish_motion_log(
        "[exit_gap] start reverse exit phase=" +
        single_cabinet_exit_phase_to_string(single_cabinet_exit_phase_) +
        " distance=" +
        format_seconds(single_cabinet_exit_target_distance_) +
        " speed=" + format_seconds(speed) +
        " timeout=" + format_seconds(single_cabinet_exit_effective_timeout_sec_) +
        " turn_enabled=" + (single_cabinet_exit_turn_enabled_ ? "true" : "false"));
    }

    const double ultrasonic_range = min_ultrasonic_range();
    if (std::isfinite(ultrasonic_range) && ultrasonic_range < entry_ultrasonic_stop_distance_) {
      publish_stop();
      const std::string reason =
        "倒退出缝被超声波安全策略阻塞 range=" + std::to_string(ultrasonic_range);
      if (full_inventory_active_) {
        fail_full_inventory(reason);
      } else {
        fail_single_cabinet_motion(reason);
      }
      return;
    }

    if (single_cabinet_exit_phase_ == SingleCabinetExitPhase::STRAIGHT_REVERSE) {
      const double elapsed = (this->now() - single_cabinet_exit_phase_start_time_).seconds();
      const double traveled = segment_distance();
      if (elapsed > std::max(0.1, single_cabinet_exit_effective_timeout_sec_)) {
        publish_stop();
        RCLCPP_ERROR(
          get_logger(),
          "[mission_manager][single_cabinet][exit_gap] phase=%s timeout traveled=%.2f "
          "target_distance=%.2f elapsed=%.2f",
          single_cabinet_exit_phase_to_string(single_cabinet_exit_phase_).c_str(),
          traveled,
          single_cabinet_exit_target_distance_,
          elapsed);
        publish_motion_log(
          "[exit_gap] phase=" + single_cabinet_exit_phase_to_string(single_cabinet_exit_phase_) +
          " timeout traveled=" + format_seconds(traveled) +
          " target_distance=" + format_seconds(single_cabinet_exit_target_distance_) +
          " elapsed=" + format_seconds(elapsed));
        if (full_inventory_active_) {
          fail_full_inventory("倒退出缝直线阶段超时");
        } else {
          fail_single_cabinet_motion("倒退出缝直线阶段超时");
        }
        return;
      }

      if (traveled >= single_cabinet_exit_target_distance_) {
        publish_stop();
        const double finished_exit_linear_cmd = apply_exit_motion_direction(speed);
        const std::string exit_motion_label =
          exit_motion_label_from_linear_cmd(finished_exit_linear_cmd);
        if (!single_cabinet_exit_turn_enabled_) {
          RCLCPP_INFO(
            get_logger(),
            "[mission_manager][single_cabinet][exit_gap] 出缝完成：%s only traveled=%.2f "
            "entry_motion_mode=%s exit_linear_cmd=%.3f published_cmd_linear_x=%.3f "
            "exit_motion_label=%s",
            exit_motion_label.c_str(),
            traveled,
            entry_motion_mode_to_string(entry_motion_mode_).c_str(),
            finished_exit_linear_cmd,
            finished_exit_linear_cmd,
            exit_motion_label.c_str());
          publish_motion_log(
            "[exit_gap] entry_motion_mode=" + entry_motion_mode_to_string(entry_motion_mode_) +
            " exit_linear_cmd=" + format_fixed(finished_exit_linear_cmd, 3) +
            " published_cmd_linear_x=" + format_fixed(finished_exit_linear_cmd, 3) +
            " exit_motion_label=" + exit_motion_label +
            " only traveled=" + format_seconds(traveled));
          finish_single_cabinet_exit_gap();
          return;
        }

        const double turn_speed = std::isfinite(single_cabinet_exit_turn_angular_speed_) ?
          std::abs(single_cabinet_exit_turn_angular_speed_) : 0.0;
        if (turn_speed <= 1e-4) {
          RCLCPP_WARN(
            get_logger(),
            "[mission_manager][single_cabinet][exit_gap] 原地转回走廊角速度非法，降级为直线出缝完成: %.3f",
            single_cabinet_exit_turn_angular_speed_);
          publish_motion_log(
            "[exit_gap] exit turn angular speed invalid, finish " + exit_motion_label +
            " only entry_motion_mode=" + entry_motion_mode_to_string(entry_motion_mode_) +
            " exit_linear_cmd=" + format_fixed(finished_exit_linear_cmd, 3) +
            " published_cmd_linear_x=" + format_fixed(finished_exit_linear_cmd, 3) +
            " exit_motion_label=" + exit_motion_label);
          finish_single_cabinet_exit_gap();
          return;
        }

        std::string yaw_reason;
        if (!current_odom_ready_for_entry(yaw_reason)) {
          RCLCPP_WARN(
            get_logger(),
            "[mission_manager][single_cabinet][exit_gap] 原地转回走廊无法获取有效yaw，降级完成: %s",
            yaw_reason.c_str());
          publish_motion_log(
            "[exit_gap] yaw invalid before turn_to_corridor, finish " + exit_motion_label +
            " only entry_motion_mode=" + entry_motion_mode_to_string(entry_motion_mode_) +
            " exit_linear_cmd=" + format_fixed(finished_exit_linear_cmd, 3) +
            " published_cmd_linear_x=" + format_fixed(finished_exit_linear_cmd, 3) +
            " exit_motion_label=" + exit_motion_label);
          finish_single_cabinet_exit_gap();
          return;
        }

        const Pose2D current = current_pose_2d();
        if (!current.valid || !std::isfinite(current.yaw) || !std::isfinite(entry_turn_start_yaw_)) {
          RCLCPP_WARN(
            get_logger(),
            "[mission_manager][single_cabinet][exit_gap] 原地转回走廊yaw数据无效，降级完成");
          publish_motion_log(
            "[exit_gap] yaw data invalid before turn_to_corridor, finish " + exit_motion_label +
            " only entry_motion_mode=" + entry_motion_mode_to_string(entry_motion_mode_) +
            " exit_linear_cmd=" + format_fixed(finished_exit_linear_cmd, 3) +
            " published_cmd_linear_x=" + format_fixed(finished_exit_linear_cmd, 3) +
            " exit_motion_label=" + exit_motion_label);
          finish_single_cabinet_exit_gap();
          return;
        }

        const double target_exit_yaw = normalize_angle(entry_turn_start_yaw_);
        const double yaw_error = normalize_angle(target_exit_yaw - current.yaw);
        const double yaw_tolerance =
          std::isfinite(single_cabinet_exit_turn_yaw_tolerance_rad_) ?
          std::max(0.001, std::abs(single_cabinet_exit_turn_yaw_tolerance_rad_)) : 0.08;
        if (std::abs(yaw_error) <= yaw_tolerance) {
          RCLCPP_INFO(
            get_logger(),
            "[mission_manager][single_cabinet][exit_gap] 出缝完成：%s + turn_to_corridor "
            "yaw already aligned current_yaw=%.3f target_exit_yaw=%.3f yaw_error=%.3f "
            "entry_motion_mode=%s exit_linear_cmd=%.3f published_cmd_linear_x=%.3f "
            "exit_motion_label=%s",
            exit_motion_label.c_str(),
            current.yaw,
            target_exit_yaw,
            yaw_error,
            entry_motion_mode_to_string(entry_motion_mode_).c_str(),
            finished_exit_linear_cmd,
            finished_exit_linear_cmd,
            exit_motion_label.c_str());
          publish_motion_log(
            "[exit_gap] entry_motion_mode=" + entry_motion_mode_to_string(entry_motion_mode_) +
            " exit_linear_cmd=" + format_fixed(finished_exit_linear_cmd, 3) +
            " published_cmd_linear_x=" + format_fixed(finished_exit_linear_cmd, 3) +
            " exit_motion_label=" + exit_motion_label +
            " turn_to_corridor yaw already aligned");
          finish_single_cabinet_exit_gap();
          return;
        }

        single_cabinet_exit_phase_ = SingleCabinetExitPhase::TURN_TO_CORRIDOR;
        single_cabinet_exit_phase_start_time_ = this->now();
        RCLCPP_INFO(
          get_logger(),
          "[mission_manager][single_cabinet][exit_gap] switch phase=%s entry_side=%s current_yaw=%.3f "
          "target_exit_yaw=%.3f yaw_error=%.3f",
          single_cabinet_exit_phase_to_string(single_cabinet_exit_phase_).c_str(),
          current_entry_side_.c_str(),
          current.yaw,
          target_exit_yaw,
          yaw_error);
        publish_motion_log(
          "[exit_gap] switch phase=" + single_cabinet_exit_phase_to_string(single_cabinet_exit_phase_) +
          " entry_side=" + current_entry_side_);
        return;
      }

      geometry_msgs::msg::Twist cmd;
      const double exit_linear_cmd = apply_exit_motion_direction(speed);
      cmd.linear.x = exit_linear_cmd;
      cmd.angular.z = 0.0;
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "[mission_manager][single_cabinet][exit_gap] cmd_vel_source=NORMAL_EXIT_GAP "
        "entry_motion_mode=%s exit_linear_cmd=%.3f published_cmd_linear_x=%.3f "
        "published_cmd_angular_z=%.3f exit_distance=%.2f traveled=%.2f "
        "safety_stop=false safety_reason=none phase=%s elapsed=%.2f",
        entry_motion_mode_to_string(entry_motion_mode_).c_str(),
        exit_linear_cmd,
        cmd.linear.x,
        cmd.angular.z,
        single_cabinet_exit_target_distance_,
        traveled,
        single_cabinet_exit_phase_to_string(single_cabinet_exit_phase_).c_str(),
        elapsed);
      cmd_pub_->publish(cmd);
      return;
    }

    if (single_cabinet_exit_phase_ == SingleCabinetExitPhase::TURN_TO_CORRIDOR) {
      std::string yaw_reason;
      if (!current_odom_ready_for_entry(yaw_reason)) {
        publish_stop();
        RCLCPP_WARN(
          get_logger(),
          "[mission_manager][single_cabinet][exit_gap] phase=%s yaw无效，结束出缝避免卡死: %s",
          single_cabinet_exit_phase_to_string(single_cabinet_exit_phase_).c_str(),
          yaw_reason.c_str());
        publish_motion_log("[exit_gap] turn_to_corridor yaw invalid, finish to avoid blocking");
        finish_single_cabinet_exit_gap();
        return;
      }

      const Pose2D current = current_pose_2d();
      if (!current.valid || !std::isfinite(current.yaw) || !std::isfinite(entry_turn_start_yaw_)) {
        publish_stop();
        RCLCPP_WARN(
          get_logger(),
          "[mission_manager][single_cabinet][exit_gap] phase=%s yaw数据无效，结束出缝避免卡死",
          single_cabinet_exit_phase_to_string(single_cabinet_exit_phase_).c_str());
        publish_motion_log("[exit_gap] turn_to_corridor yaw data invalid, finish to avoid blocking");
        finish_single_cabinet_exit_gap();
        return;
      }

      const double target_exit_yaw = normalize_angle(entry_turn_start_yaw_);
      const double yaw_error = normalize_angle(target_exit_yaw - current.yaw);
      const double yaw_tolerance =
        std::isfinite(single_cabinet_exit_turn_yaw_tolerance_rad_) ?
        std::max(0.001, std::abs(single_cabinet_exit_turn_yaw_tolerance_rad_)) : 0.08;
      if (std::abs(yaw_error) <= yaw_tolerance) {
        publish_stop();
        const double finished_exit_linear_cmd = apply_exit_motion_direction(speed);
        const std::string exit_motion_label =
          exit_motion_label_from_linear_cmd(finished_exit_linear_cmd);
        RCLCPP_INFO(
          get_logger(),
          "[mission_manager][single_cabinet][exit_gap] 出缝完成：%s + turn_to_corridor "
          "entry_side=%s current_yaw=%.3f target_exit_yaw=%.3f yaw_error=%.3f "
          "entry_motion_mode=%s exit_linear_cmd=%.3f published_cmd_linear_x=%.3f "
          "exit_motion_label=%s",
          exit_motion_label.c_str(),
          current_entry_side_.c_str(),
          current.yaw,
          target_exit_yaw,
          yaw_error,
          entry_motion_mode_to_string(entry_motion_mode_).c_str(),
          finished_exit_linear_cmd,
          finished_exit_linear_cmd,
          exit_motion_label.c_str());
        publish_motion_log(
          "[exit_gap] entry_motion_mode=" + entry_motion_mode_to_string(entry_motion_mode_) +
          " exit_linear_cmd=" + format_fixed(finished_exit_linear_cmd, 3) +
          " published_cmd_linear_x=" + format_fixed(finished_exit_linear_cmd, 3) +
          " exit_motion_label=" + exit_motion_label +
          " turn_to_corridor");
        finish_single_cabinet_exit_gap();
        return;
      }

      const double elapsed = (this->now() - single_cabinet_exit_phase_start_time_).seconds();
      const double turn_timeout =
        std::isfinite(single_cabinet_exit_turn_timeout_sec_) ?
        std::max(0.1, single_cabinet_exit_turn_timeout_sec_) : 8.0;
      if (elapsed > turn_timeout) {
        publish_stop();
        RCLCPP_WARN(
          get_logger(),
          "[mission_manager][single_cabinet][exit_gap] phase=%s timeout entry_side=%s current_yaw=%.3f "
          "target_exit_yaw=%.3f yaw_error=%.3f elapsed=%.2f",
          single_cabinet_exit_phase_to_string(single_cabinet_exit_phase_).c_str(),
          current_entry_side_.c_str(),
          current.yaw,
          target_exit_yaw,
          yaw_error,
          elapsed);
        publish_motion_log("[exit_gap] turn_to_corridor timeout, finish to avoid blocking");
        finish_single_cabinet_exit_gap();
        return;
      }

      const double turn_speed = std::isfinite(single_cabinet_exit_turn_angular_speed_) ?
        std::abs(single_cabinet_exit_turn_angular_speed_) : 0.0;
      if (turn_speed <= 1e-4) {
        publish_stop();
        RCLCPP_WARN(
          get_logger(),
          "[mission_manager][single_cabinet][exit_gap] phase=%s angular speed invalid, finish to avoid blocking",
          single_cabinet_exit_phase_to_string(single_cabinet_exit_phase_).c_str());
        publish_motion_log("[exit_gap] turn_to_corridor angular speed invalid, finish to avoid blocking");
        finish_single_cabinet_exit_gap();
        return;
      }

      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      cmd.angular.z = std::clamp(yaw_error, -turn_speed, turn_speed);
      cmd_pub_->publish(cmd);
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "[mission_manager][single_cabinet][exit_gap] phase=%s entry_side=%s current_yaw=%.3f "
        "target_exit_yaw=%.3f yaw_error=%.3f cmd.linear.x=%.3f cmd.angular.z=%.3f elapsed=%.2f",
        single_cabinet_exit_phase_to_string(single_cabinet_exit_phase_).c_str(),
        current_entry_side_.c_str(),
        current.yaw,
        target_exit_yaw,
        yaw_error,
        cmd.linear.x,
        cmd.angular.z,
        elapsed);
      return;
    }

    publish_stop();
    RCLCPP_WARN(
      get_logger(),
      "[mission_manager][single_cabinet][exit_gap] unknown exit phase, reset to STRAIGHT_REVERSE");
    single_cabinet_exit_phase_ = SingleCabinetExitPhase::STRAIGHT_REVERSE;
    single_cabinet_exit_phase_start_time_ = this->now();
  }

  void handle_single_cabinet_side_row_scan_finished(int cabinet_id)
  {
    if (!single_cabinet_exit_after_each_scan_) {
      fail_single_cabinet_motion("side-row 流程当前要求 single_cabinet_exit_after_each_scan=true");
      return;
    }

    switch (single_cabinet_side_row_phase_) {
      case SingleCabinetSideRowPhase::FIRST_PRIMARY_SCAN:
        publish_single_cabinet_side_row_log(
          "cabinet=" + std::to_string(cabinet_id) +
          " scan finished, exit and reenter for adjusted cabinet=" +
          std::to_string(single_cabinet_adjusted_scan_cabinet_));
        single_cabinet_side_row_phase_ = SingleCabinetSideRowPhase::FIRST_ADJUSTED_SCAN;
        single_cabinet_current_scan_cabinet_ = single_cabinet_adjusted_scan_cabinet_;
        single_cabinet_after_exit_action_ = SingleCabinetAfterExitAction::REENTER_ADJUSTED;
        set_single_cabinet_state(
          State::SINGLE_CABINET_EXIT_GAP,
          "cabinet=" + std::to_string(cabinet_id) + " 扫描完成，倒退出缝");
        return;

      case SingleCabinetSideRowPhase::FIRST_ADJUSTED_SCAN:
        publish_single_cabinet_side_row_log(
          "cabinet=" + std::to_string(cabinet_id) + " scan finished, exit to corridor");
        if (single_cabinet_side_row_full_sequence_) {
          single_cabinet_after_exit_action_ = SingleCabinetAfterExitAction::CORRIDOR_TRANSFER;
        } else {
          single_cabinet_after_exit_action_ = SingleCabinetAfterExitAction::CLOSE_AND_DONE;
        }
        set_single_cabinet_state(
          State::SINGLE_CABINET_EXIT_GAP,
          "cabinet=" + std::to_string(cabinet_id) + " 扫描完成，倒退出缝");
        return;

      case SingleCabinetSideRowPhase::SECOND_PRIMARY_SCAN:
        publish_single_cabinet_side_row_log(
          "cabinet=" + std::to_string(cabinet_id) +
          " scan finished, exit and reenter for adjusted cabinet=" +
          std::to_string(single_cabinet_adjusted_scan_cabinet_));
        single_cabinet_side_row_phase_ = SingleCabinetSideRowPhase::SECOND_ADJUSTED_SCAN;
        single_cabinet_current_scan_cabinet_ = single_cabinet_adjusted_scan_cabinet_;
        single_cabinet_after_exit_action_ = SingleCabinetAfterExitAction::REENTER_ADJUSTED;
        set_single_cabinet_state(
          State::SINGLE_CABINET_EXIT_GAP,
          "cabinet=" + std::to_string(cabinet_id) + " 扫描完成，倒退出缝");
        return;

      case SingleCabinetSideRowPhase::SECOND_ADJUSTED_SCAN:
        publish_single_cabinet_side_row_log(
          "cabinet=" + std::to_string(cabinet_id) + " scan finished, final exit");
        single_cabinet_side_row_phase_ = SingleCabinetSideRowPhase::COMPLETE;
        single_cabinet_after_exit_action_ = SingleCabinetAfterExitAction::FINAL_CLOSE_AND_DONE;
        set_single_cabinet_state(
          State::SINGLE_CABINET_FINAL_EXIT_GAP,
          "cabinet=" + std::to_string(cabinet_id) + " 扫描完成，最终倒退出缝");
        return;

      case SingleCabinetSideRowPhase::NONE:
      case SingleCabinetSideRowPhase::CORRIDOR_TRANSFER:
      case SingleCabinetSideRowPhase::COMPLETE:
      default:
        fail_single_cabinet_motion(
          "side-row 扫描完成阶段非法 phase=" + side_row_phase_to_string(single_cabinet_side_row_phase_));
        return;
    }
  }

  void handle_single_cabinet_scan_state()
  {
    const int cabinet_id = active_single_cabinet_scan_cabinet();
    if (cabinet_id <= 0) {
      fail_single_cabinet_motion("单柜盘库扫描柜号非法");
      return;
    }

    if (single_cabinet_side_row_active_ &&
      (!single_cabinet_grid_motion_enabled_ || !single_cabinet_scan_active_))
    {
      publish_single_cabinet_side_row_log(
        "gap=" + active_single_cabinet_gap_id() +
        " scan cabinet=" + std::to_string(cabinet_id) +
        " phase=" + side_row_phase_to_string(single_cabinet_side_row_phase_));
    }

    if (!execute_single_cabinet_scan_runtime_tick(cabinet_id)) {
      return;
    }

    if (single_cabinet_side_row_active_) {
      handle_single_cabinet_side_row_scan_finished(cabinet_id);
      return;
    }

    if (!single_cabinet_motion_stop_after_scan_) {
      publish_single_cabinet_log("scan finished, exit gap");
      single_cabinet_after_exit_action_ = SingleCabinetAfterExitAction::CLOSE_AND_DONE;
      set_single_cabinet_state(State::SINGLE_CABINET_FINAL_EXIT_GAP, "单柜扫描完成，倒退出缝");
      return;
    }

    publish_single_cabinet_log("scan finished, stop after scan");
    set_single_cabinet_state(
      State::SINGLE_CABINET_STOP_AFTER_SCAN,
      "scan finished, stop after scan, no exit motion in current step");
  }

  void handle_single_cabinet_reenter_adjusted_scan_state()
  {
    if (!single_cabinet_reentry_for_position_adjustment_) {
      fail_single_cabinet_motion("side-row 调整扫描需要启用 single_cabinet_reentry_for_position_adjustment");
      return;
    }

    // TODO: right-mounted camera orientation adjustment is required before real scanning on the adjusted/opposite scan side.
    publish_single_cabinet_log(
      "[reenter_adjust] reenter for adjusted cabinet=" +
      std::to_string(single_cabinet_current_scan_cabinet_));
    (void)prepare_single_cabinet_target_search(
      single_cabinet_current_scan_cabinet_,
      "准备再次入缝调整扫描 cabinet=" + std::to_string(single_cabinet_current_scan_cabinet_));
  }

  void handle_single_cabinet_corridor_transfer_state()
  {
    if (!single_cabinet_side_row_corridor_transfer_enabled_) {
      fail_single_cabinet_motion("single_cabinet_side_row_corridor_transfer_enabled=false");
      return;
    }
    if (plc_http_enabled_) {
      fail_single_cabinet_motion("side-row corridor transfer 本轮未接入 PLC open/wait，禁止在 plc_http_enabled=true 时直接移动");
      return;
    }

    const std::string previous_gap = active_single_cabinet_gap_id();
    publish_single_cabinet_log("[corridor_transfer] request close gap=" + previous_gap);
    (void)web_api_client_.requestCloseGap(previous_gap);

    single_cabinet_side_row_phase_ = SingleCabinetSideRowPhase::CORRIDOR_TRANSFER;
    single_cabinet_current_scan_cabinet_ = single_cabinet_side_row_corridor_transfer_target_cabinet_;
    single_cabinet_target_recognized_logged_ = false;
    single_cabinet_final_recognition_wait_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

    publish_single_cabinet_log(
      "[corridor_transfer] move toward cabinet 1, target cabinet=" +
      std::to_string(single_cabinet_side_row_corridor_transfer_target_cabinet_) +
      " direction=" + single_cabinet_side_row_corridor_transfer_direction_);

    std::string reason;
    if (!prepare_single_cabinet_target_cabinet(single_cabinet_side_row_corridor_transfer_target_cabinet_, reason)) {
      fail_single_cabinet_motion("准备走廊转移目标失败: " + reason);
      return;
    }

    target_visible_ = false;
    has_distance_ = false;
    latest_distance_ = 0.0;
    tracking_stable_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_target_seen_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    reset_wait_gap_runtime();
    reset_entry_gap_runtime();
    publish_gap_context();

    std::string nav_reason;
    if (!begin_nav_route_for_current_target(
        "[mission_manager][single_cabinet][corridor_transfer] enter real nav target cabinet=" +
        std::to_string(current_target_cabinet_),
        nav_reason,
        State::SINGLE_CABINET_NAV_TO_TARGET))
    {
      fail_single_cabinet_motion("走廊转移启动巡航失败: " + nav_reason);
    }
  }

  void handle_single_cabinet_prepare_next_gap_state()
  {
    if (plc_http_enabled_) {
      fail_single_cabinet_motion("side-row 第二 gap 入口本轮未接入 PLC open/wait，禁止在 plc_http_enabled=true 时直接移动");
      return;
    }

    single_cabinet_active_gap_id_ = single_cabinet_side_row_second_gap_;
    single_cabinet_side_row_phase_ = SingleCabinetSideRowPhase::SECOND_PRIMARY_SCAN;
    single_cabinet_current_scan_cabinet_ = single_cabinet_side_row_second_gap_scan_sequence_.front();
    single_cabinet_adjusted_scan_cabinet_ = single_cabinet_side_row_second_gap_scan_sequence_[1];
    single_cabinet_after_exit_action_ = SingleCabinetAfterExitAction::NONE;

    publish_single_cabinet_side_row_log(
      "prepare next gap=" + single_cabinet_active_gap_id_ +
      " target cabinet=" + std::to_string(single_cabinet_current_scan_cabinet_));
    publish_single_cabinet_side_row_log(
      "enter " + single_cabinet_active_gap_id_ +
      " for cabinet=" + std::to_string(single_cabinet_current_scan_cabinet_));
    publish_single_cabinet_log("request open gap=" + single_cabinet_active_gap_id_);
    if (!web_api_client_.requestOpenGap(single_cabinet_active_gap_id_)) {
      fail_single_cabinet_motion("开柜请求失败: gap=" + single_cabinet_active_gap_id_);
      return;
    }

    set_single_cabinet_state(
      State::SINGLE_CABINET_REENTER_NEXT_GAP,
      "等待第二个缝隙开柜完成 " + format_seconds(open_gap_wait_sec_) + " sec");
  }

  void handle_single_cabinet_reenter_next_gap_state()
  {
    publish_stop();
    if (!state_elapsed(open_gap_wait_sec_)) {
      return;
    }

    (void)prepare_single_cabinet_target_search(
      single_cabinet_current_scan_cabinet_,
      "准备进入第二个缝隙 cabinet=" + std::to_string(single_cabinet_current_scan_cabinet_));
  }

  bool load_configs_and_prepare_current_target(std::string & reason)
  {
    routes_loaded_ = load_route_config(reason);
    if (!routes_loaded_) {
      return false;
    }
    warehouse_layout_loaded_ = load_warehouse_layout_config(reason);
    if (!warehouse_layout_loaded_) {
      return false;
    }

    if (!prepare_current_target(reason)) {
      return false;
    }
    if (!resolve_current_route(reason)) {
      return false;
    }
    if (!configure_current_entry_profile(reason)) {
      return false;
    }
    if (!resolve_current_gap_plan(reason)) {
      return false;
    }
    log_current_target_entry_plan();
    return true;
  }

  const InventoryGapPlan * current_gap_request_plan() const
  {
    if (current_gap_request_index_ >= gap_request_queue_.size()) {
      return nullptr;
    }
    return &gap_request_queue_[current_gap_request_index_];
  }

  bool state_elapsed(double seconds) const
  {
    return (this->now() - state_enter_time_).seconds() >= std::max(0.0, seconds);
  }

  void fail_inventory_flow(const std::string & reason)
  {
    mission_error_reason_ = reason;
    set_flow_state(State::ERROR, reason);
  }

  void stop_single_cabinet_motion_controls()
  {
    request_lift_safety_stop("single cabinet controls stop");
    cancel_nav2_route_goal("单柜盘库停止");
    cancel_nav2_return_goal("单柜盘库停止");
    set_corridor_mode(false, false);
    publish_stop();
    request_recognizer_enable(false);
    set_recognizer_topic_enabled(false, true);
    set_distance_estimator_enabled(false, true);
    set_gap_detector_enabled(false);
  }

  void stop_all_inventory_controls_for_safe_action(const std::string & reason)
  {
    request_lift_safety_stop(reason);
    cancel_nav2_route_goal(reason);
    cancel_nav2_return_goal(reason);
    reset_nav_route_runtime();
    reset_wait_gap_runtime();
    reset_entry_gap_runtime();
    clear_inventory_upload_batch("cancel");
    reset_single_cabinet_scan_runtime();
    set_corridor_mode(false, false);
    request_recognizer_enable(false);
    set_recognizer_topic_enabled(false, true);
    set_distance_estimator_enabled(false, true);
    set_gap_detector_enabled(false);
    clear_current_target_cabinet();
    publish_gap_context();
    publish_stop();
  }

  void fail_single_cabinet_motion(const std::string & reason)
  {
    publish_single_cabinet_log("failed: " + reason);
    clear_inventory_upload_batch("error");
    stop_single_cabinet_motion_controls();
    reset_single_cabinet_scan_runtime();
    mission_active_ = false;
    mission_error_reason_ = reason;
    set_single_cabinet_state(State::ERROR, reason);
  }

  std::vector<int> scan_cabinets_from_request(
    const agv_inventory_system::srv::StartMission::Request & request) const
  {
    std::vector<int> cabinets;
    for (const auto cabinet_id : request.scan_cabinets) {
      cabinets.push_back(static_cast<int>(cabinet_id));
    }
    return cabinets;
  }

  bool find_gap_for_cabinet(int cabinet_id, std::string & gap_id) const
  {
    gap_id.clear();
    for (const auto & item : gap_scan_map_cabinets_by_gap_id_) {
      if (std::find(item.second.begin(), item.second.end(), cabinet_id) != item.second.end()) {
        gap_id = item.first;
        return true;
      }
    }
    return false;
  }

  bool request_contains_formal_inventory_fields(
    const agv_inventory_system::srv::StartMission::Request & request) const
  {
    return request.run_full_inventory ||
      !agv_inventory_system::trim(request.target_gap).empty() ||
      !request.scan_cabinets.empty();
  }

  bool start_single_cabinet_inventory_from_request(
    const agv_inventory_system::srv::StartMission::Request & request,
    std::string & message)
  {
    message.clear();
    if (!single_cabinet_motion_enabled_) {
      message = "单柜盘库未启用";
      return false;
    }
    clear_inventory_upload_batch("new_mission");

    std::string gap_id = agv_inventory_system::trim(request.target_gap);
    std::vector<int> scan_cabinets = scan_cabinets_from_request(request);
    if (scan_cabinets.empty() && request.targets.size() == 1U) {
      int cabinet_id = -1;
      if (parse_target_cabinet(request.targets.front(), cabinet_id)) {
        scan_cabinets.push_back(cabinet_id);
      }
    }
    if (gap_id.empty() && scan_cabinets.size() == 1U && !find_gap_for_cabinet(scan_cabinets.front(), gap_id)) {
      if (scan_cabinets.front() == single_cabinet_motion_target_cabinet_) {
        gap_id = single_cabinet_motion_target_gap_;
      }
    }
    if (scan_cabinets.empty() && !gap_id.empty()) {
      (void)find_configured_gap_cabinets(gap_id, scan_cabinets);
    }

    if (single_cabinet_side_row_enabled_ &&
      is_valid_single_cabinet_side_row_request(gap_id, scan_cabinets, false))
    {
      InventoryGapPlan plan;
      std::string side_row_reason;
      if (!start_single_cabinet_side_row_context(gap_id, scan_cabinets, plan, side_row_reason)) {
        message = side_row_reason;
        publish_single_cabinet_log(message);
        return false;
      }

      clear_safe_exit_gap_recovery_context();
      gap_request_queue_ = {plan};
      current_gap_request_index_ = 0;
      mission_error_reason_.clear();
      inventory_flow_active_ = true;
      single_cabinet_motion_active_ = true;
      single_cabinet_gap_searching_ = false;
      single_cabinet_target_recognized_logged_ = false;
      single_cabinet_close_requested_ = false;
      single_cabinet_final_recognition_wait_start_ =
        rclcpp::Time(0, 0, get_clock()->get_clock_type());

      publish_single_cabinet_log(
        "[side_row] accepted " + single_cabinet_side_row_name_ +
        " gap=" + plan.gap_id +
        " cabinets=" + cabinet_unit_to_string(plan.scan_cabinets));
      set_flow_state(
        State::REQUEST_OPEN_GAP,
        "单柜侧排盘库任务已启动 gap=" + plan.gap_id +
        " cabinets=" + cabinet_unit_to_string(plan.scan_cabinets));

      message = "单柜侧排盘库任务已接收";
      return true;
    }

    if (!is_valid_single_cabinet_motion_request(gap_id, scan_cabinets, false)) {
      message =
        single_cabinet_side_row_enabled_ ? side_row_reject_message() : single_cabinet_reject_message();
      publish_single_cabinet_log(message);
      return false;
    }

    reset_single_cabinet_side_row_context();
    clear_safe_exit_gap_recovery_context();
    InventoryGapPlan plan;
    plan.gap_id = single_cabinet_motion_target_gap_;
    plan.scan_cabinets = {single_cabinet_motion_target_cabinet_};
    gap_request_queue_ = {plan};
    current_gap_request_index_ = 0;
    mission_error_reason_.clear();
    inventory_flow_active_ = true;
    single_cabinet_motion_active_ = true;
    single_cabinet_gap_searching_ = false;
    single_cabinet_target_recognized_logged_ = false;
    single_cabinet_close_requested_ = false;
    single_cabinet_final_recognition_wait_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

    publish_single_cabinet_log(
      "accepted single cabinet inventory gap=" + plan.gap_id +
      " cabinet=" + std::to_string(single_cabinet_motion_target_cabinet_));
    set_flow_state(
      State::REQUEST_OPEN_GAP,
      "单柜盘库任务已启动 gap=" + plan.gap_id +
      " cabinets=" + cabinet_unit_to_string(plan.scan_cabinets));

    message = "单柜盘库任务已接收";
    return true;
  }

  void start_service_callback(
    const std::shared_ptr<agv_inventory_system::srv::StartMission::Request> request,
    std::shared_ptr<agv_inventory_system::srv::StartMission::Response> response)
  {
    std::string auto_recharge_block_reason;
    if (auto_recharge_control_blocks_start_mission(auto_recharge_block_reason)) {
      response->accepted = false;
      response->message =
        "auto recharge is active or canceling; reject start_mission";
      if (!auto_recharge_block_reason.empty()) {
        response->message += " (" + auto_recharge_block_reason + ")";
      }
      return;
    }

    if (mission_active_ || inventory_flow_active_) {
      response->accepted = false;
      response->message = "任务已在运行";
      return;
    }
    clear_inventory_upload_batch("new_mission");

    std::string load_reason;
    inventory_runtime_config_loaded_ = load_inventory_runtime_config(load_reason);
    if (!inventory_runtime_config_loaded_) {
      response->accepted = false;
      response->message = load_reason;
      return;
    }

    if (request->run_full_inventory && !full_inventory_enabled_) {
      response->accepted = false;
      response->message = "全部盘库未启用";
      return;
    }

    if (should_start_full_inventory(*request)) {
      if (request->scan_cabinets.empty()) {
        for (const auto & target : request->targets) {
          TargetMetadata parsed_target;
          std::string parse_reason;
          if (!parse_target_metadata(target, parsed_target, parse_reason)) {
            response->accepted = false;
            response->message = parse_reason;
            return;
          }
        }
      }
      const bool force_map_origin = request->return_home;
      const auto sequence = full_inventory_sequence_from_request(*request);
      std::string full_inventory_reason;
      if (!start_full_inventory_sequence(sequence, force_map_origin, full_inventory_reason)) {
        response->accepted = false;
        response->message = full_inventory_reason;
        publish_full_inventory_log(response->message);
        return;
      }
      response->accepted = true;
      response->message =
        "全部盘库队列已接收 sequence=" + cabinet_unit_to_string(full_inventory_sequence_);
      return;
    }

    if (request_contains_formal_inventory_fields(*request)) {
      std::string start_message;
      if (!start_single_cabinet_inventory_from_request(*request, start_message)) {
        response->accepted = false;
        response->message = start_message;
        return;
      }
      response->accepted = true;
      response->message = start_message;
      return;
    }

    std::vector<std::string> targets = request->targets;
    if (targets.empty()) {
      targets = target_list_param_;
    }
    if (targets.empty()) {
      response->accepted = false;
      response->message = "目标列表为空";
      return;
    }

    for (const auto & target : targets) {
      TargetMetadata parsed_target;
      std::string parse_reason;
      if (!parse_target_metadata(target, parsed_target, parse_reason)) {
        response->accepted = false;
        response->message = parse_reason;
        return;
      }
    }

    targets_ = targets;
    current_target_index_ = 0;
    clear_safe_exit_gap_recovery_context();

    std::string reason;
    if (!load_configs_and_prepare_current_target(reason)) {
      response->accepted = false;
      response->message = reason;
      return;
    }

    mission_active_ = true;
    cancel_requested_ = false;
    mission_force_map_origin_on_finish_ = request->return_home;

    target_visible_ = false;
    has_distance_ = false;
    latest_distance_ = 0.0;

    return_mode_ = ReturnMode::NONE;
    nav2_return_in_progress_ = false;
    nav2_result_ready_ = false;
    reset_wait_gap_runtime();
    reset_entry_gap_runtime();
    publish_gap_context();

    if (plc_http_enabled_) {
      mission_active_ = false;
      request_recognizer_enable(false);
      set_corridor_mode(false, false);
      publish_stop();
      response->accepted = false;
      response->message =
        "legacy targets_ 初始移动入口本轮未接入 PLC open/wait，禁止在 plc_http_enabled=true 时直接移动";
      set_state(State::ERROR, response->message);
      return;
    }

    tracking_stable_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_target_seen_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

    mission_start_distance_ = odom_cumulative_distance_;
    reset_segment_distance();
    mission_start_pose_ = current_pose_2d();
    mission_start_pose_nav2_ = Pose2D{};
    if (!mission_start_pose_.valid) {
      RCLCPP_WARN(get_logger(), "启动任务时未获取到有效里程计位姿，返航将无法使用起点模式");
    } else {
      RCLCPP_INFO(
        get_logger(),
        "记录任务起点：x=%.3f y=%.3f yaw=%.3f frame=%s",
        mission_start_pose_.x,
        mission_start_pose_.y,
        mission_start_pose_.yaw,
        mission_start_pose_.frame_id.c_str());

      std::string tf_error;
      if (transform_pose_2d(mission_start_pose_, nav2_goal_frame_, mission_start_pose_nav2_, tf_error)) {
        RCLCPP_INFO(
          get_logger(),
          "记录Nav2全局起点：x=%.3f y=%.3f yaw=%.3f frame=%s",
          mission_start_pose_nav2_.x,
          mission_start_pose_nav2_.y,
          mission_start_pose_nav2_.yaw,
          mission_start_pose_nav2_.frame_id.c_str());
      } else {
        RCLCPP_WARN(
          get_logger(),
          "记录Nav2全局起点失败，将回退到里程计起点：%s",
          tf_error.c_str());
      }
    }

    std::string nav_route_fail_reason;
    if (!begin_nav_route_for_current_target("开始Nav2巡航路线识别", nav_route_fail_reason)) {
      clear_current_target_cabinet();
      response->accepted = false;
      response->message = nav_route_fail_reason;
      return;
    }

    response->accepted = true;
    response->message = "任务已启动";
  }

  void cancel_service_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    if (inventory_flow_active_) {
      clear_current_target_cabinet();
      if (full_inventory_active_) {
        fail_full_inventory("收到取消请求，停止全部盘库");
        response->message = "全部盘库已请求停止";
      } else if (single_cabinet_motion_active_) {
        fail_single_cabinet_motion("收到取消请求，停止单柜盘库");
        response->message = "单柜盘库已请求停止";
      } else {
        fail_inventory_flow("收到取消请求，停止盘库流程");
        response->message = "盘库流程已请求停止";
      }
      response->success = true;
      return;
    }

    if (!mission_active_ && state_ == State::IDLE) {
      clear_current_target_cabinet();
      publish_stop();
      set_corridor_mode(false, false);
      response->success = true;
      response->message = "当前无运行任务";
      return;
    }

    cancel_requested_ = true;
    mission_active_ = true;
    target_visible_ = false;
    has_distance_ = false;

    request_recognizer_enable(false);
    clear_current_target_cabinet();
    switch_to_returning(ReturnMode::CANCEL_HOME, "收到取消任务，立即停车并返回起点");

    response->success = true;
    response->message = "任务已取消，正在返回起点";
  }

  void return_home_service_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    if (state_ == State::AUTO_RECHARGING) {
      response->success = false;
      response->message = "当前正在自动回充，暂不支持切换回零点。";
      publish_log(response->message);
      publish_state_text(response->message);
      return;
    }

    if (is_in_gap_or_gap_motion_state()) {
      respond_with_pending_interrupt(
        PendingInterruptRequest::RETURN_HOME,
        "当前小车正在缝隙内，不能立即回零点。已记录回零点请求，小车完成出缝后将自动回零点。",
        *response);
      return;
    }

    std::string message;
    response->success = start_return_home_interrupt("收到回零点指令，当前小车不在缝隙内", message);
    response->message = response->success ?
      "已收到回零点指令，当前小车不在缝隙内，正在中断盘库任务并返回零点。" :
      message;
  }

  void return_to_charge_service_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    if (state_ == State::AUTO_RECHARGING) {
      response->success = true;
      response->message = "自动回充流程已在运行中，请勿重复启动。";
      publish_log(response->message);
      publish_state_text(response->message);
      return;
    }

    if (is_in_gap_or_gap_motion_state()) {
      respond_with_pending_interrupt(
        PendingInterruptRequest::AUTO_RECHARGE,
        "当前小车正在缝隙内，不能立即自动充电。已记录自动充电请求，小车完成出缝后将自动启动回充电流程。",
        *response);
      return;
    }

    std::string message;
    response->success =
      start_inventory_auto_recharge("收到自动充电指令，当前小车不在缝隙内", message);
    response->message = response->success ?
      "已收到自动充电指令，当前小车不在缝隙内，正在中断盘库任务并启动自动回充流程。" :
      message;
  }

  void cancel_auto_recharge_service_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    if (!has_active_recharge_or_charging_state()) {
      response->success = true;
      response->message = "当前未处于自动回充状态，无需取消";
      publish_log(response->message);
      return;
    }

    RCLCPP_WARN(
      get_logger(),
      "cancel_auto_recharge redirected to stop_auto_charge_and_depart "
      "(state=%s, status=%s, robot_charging_flag=%s, robot_recharge_flag=%d)",
      state_to_string(state_).c_str(),
      agv_inventory_system::trim(latest_auto_recharge_status_).c_str(),
      latest_auto_recharge_charging_ ? "true" : "false",
      latest_auto_recharge_recharge_flag_);
    publish_log("cancel_auto_recharge redirected to stop_auto_charge_and_depart");

    std::string message;
    response->success = start_stop_auto_charge_and_depart_flow(message);
    response->message = response->success ?
      "cancel_auto_recharge redirected to stop_auto_charge_and_depart，已开始停止充电并离桩流程。" :
      message;
  }

  void safe_exit_gap_service_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    if (state_ == State::AUTO_RECHARGING ||
      state_ == State::STOP_AUTO_CHARGE_AND_DEPART)
    {
      response->success = false;
      response->message = "当前自动回充/离桩流程正在控制底盘，不能执行安全出缝。";
      publish_log(response->message);
      publish_state_text(response->message);
      return;
    }

    if (!robot_inside_gap_ && !is_in_gap_or_gap_motion_state() &&
      !safe_exit_gap_recovery_context_available())
    {
      stop_all_inventory_controls_for_safe_action("收到安全出缝指令但当前不在缝隙内");
      mission_active_ = false;
      inventory_flow_active_ = false;
      pending_interrupt_request_ = PendingInterruptRequest::NONE;
      set_state(State::IDLE, "当前不在缝隙内，已停车并停止任务");
      response->success = false;
      response->message = "当前不在缝内，且缺少可用的出缝恢复上下文，未执行出缝动作。";
      publish_state_text(response->message);
      publish_log(response->message);
      return;
    }

    std::string message;
    response->success = begin_safe_exit_gap_flow(message);
    response->message = response->success ? "已开始安全退出缝隙。" : message;
  }

  void stop_auto_charge_and_depart_service_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    std::string message;
    response->success = start_stop_auto_charge_and_depart_flow(message);
    response->message = response->success ?
      "已开始停止自动充电并离桩流程。" : message;
  }

  void handle_returning_state()
  {
    if (nav2_return_in_progress_) {
      if ((this->now() - nav2_goal_sent_time_).seconds() >
        std::max(0.1, nav2_goal_timeout_sec_))
      {
        cancel_nav2_return_goal("Nav2 返航超时");
        fail_nav2_return("Nav2 返航超时，已取消返航目标");
      }
      return;
    }

    if (nav2_result_ready_) {
      nav2_result_ready_ = false;
      if (nav2_result_success_) {
        publish_log(nav2_result_text_);
        finalize_return_success();
        return;
      }

      const std::string result_text =
        nav2_result_text_.empty() ? "Nav2 返航失败" : nav2_result_text_;
      fail_nav2_return(result_text);
      return;
    }

    fail_nav2_return("RETURNING 状态无活动 Nav2 返航目标");
  }

  void publish_gap_context() const
  {
    if (!gap_context_pub_) {
      return;
    }

    std_msgs::msg::Float32MultiArray msg;
    msg.data.resize(3);
    msg.data[0] = static_cast<float>(wait_gap_failed_cycle_count_);
    msg.data[1] = static_cast<float>(current_adjust_index_);
    msg.data[2] = static_cast<float>(current_adjust_offset_);
    gap_context_pub_->publish(msg);
  }

  void reset_wait_gap_runtime()
  {
    wait_gap_phase_ = WaitGapPhase::IDLE;
    wait_gap_phase_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    wait_gap_detect_cycle_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    wait_gap_failed_cycle_count_ = 0;
    next_adjust_sequence_index_ = 0;
    current_adjust_index_ = -1;
    current_adjust_offset_ = 0.0;
    wait_gap_motion_target_distance_ = 0.0;
    wait_gap_motion_direction_ = 0.0;
  }

  void reset_entry_gap_runtime()
  {
    entry_gap_phase_ = EntryGapPhase::IDLE;
    entry_motion_mode_ = EntryMotionMode::FORWARD_ENTRY;
    entry_gap_phase_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    entry_turn_start_yaw_ = 0.0;
    entry_turn_start_yaw_valid_ = false;
    target_gap_yaw_ = 0.0;
    target_gap_yaw_valid_ = false;
    straight_start_pose_ = Pose2D{};
    entry_last_traveled_ = 0.0;
    entry_turn_completed_ = false;
    entry_turn_yaw_stable_count_ = 0;
    entry_straight_completed_ = false;
    entry_stopped_by_safety_ = false;
  }

  void set_wait_gap_phase(WaitGapPhase phase)
  {
    wait_gap_phase_ = phase;
    wait_gap_phase_start_ = this->now();
  }

  void set_entry_gap_phase(EntryGapPhase phase, const std::string & detail)
  {
    entry_gap_phase_ = phase;
    entry_gap_phase_start_ = this->now();
    publish_log("入缝子阶段切换: " + entry_gap_phase_to_string(phase) + " " + detail);
  }

  void begin_search_gap_flow()
  {
    reset_wait_gap_runtime();
    reset_entry_gap_runtime();
    if (!current_gap_plan_.valid) {
      std::string reason;
      if (!resolve_current_gap_plan(reason)) {
        if (full_inventory_active_) {
          fail_full_inventory("无法生成找缝规划: " + reason);
        } else if (single_cabinet_motion_active_) {
          fail_single_cabinet_motion("无法生成找缝规划: " + reason);
        } else {
          mission_active_ = false;
          publish_stop();
          set_state(State::ERROR, "无法生成找缝规划: " + reason);
        }
        return;
      }
    }
    if (!current_entry_profile_valid_) {
      std::string reason;
      if (!configure_current_entry_profile(reason)) {
        if (full_inventory_active_) {
          fail_full_inventory("无法生成入缝深度规划: " + reason);
        } else if (single_cabinet_motion_active_) {
          fail_single_cabinet_motion("无法生成入缝深度规划: " + reason);
        } else {
          mission_active_ = false;
          publish_stop();
          set_state(State::ERROR, "无法生成入缝深度规划: " + reason);
        }
        return;
      }
      log_current_target_entry_plan();
    }
    std::string timeout_reason;
    if (!validate_search_gap_timeout_config(timeout_reason)) {
      if (full_inventory_active_) {
        fail_full_inventory("SEARCH_GAP超时参数非法: " + timeout_reason);
      } else if (single_cabinet_motion_active_) {
        fail_single_cabinet_motion("SEARCH_GAP超时参数非法: " + timeout_reason);
      } else {
        mission_active_ = false;
        publish_stop();
        set_state(State::ERROR, "SEARCH_GAP超时参数非法: " + timeout_reason);
      }
      return;
    }

    latest_gap_ = agv_inventory_system::msg::GapStatus{};
    search_gap_start_ = this->now();
    reset_segment_distance();
    publish_entry_side();
    request_recognizer_enable(false);
    set_recognizer_topic_enabled(false, true);
    set_distance_estimator_enabled(false, true);
    const SearchDirection effective_direction = effective_gap_search_direction();
    const std::string detail =
      "跟踪稳定，按物理单元找缝 direction=" +
      search_direction_to_string(effective_direction) +
      " gap=" + gap_plan_to_string(current_gap_plan_);
    if (full_inventory_active_) {
      single_cabinet_gap_searching_ = false;
      publish_full_inventory_log(
        "search gap target=" + std::to_string(current_target_cabinet_) +
        " direction=" + search_direction_to_string(effective_direction) +
        " original_direction=" + search_direction_to_string(current_gap_plan_.search_direction) +
        " override_valid=" +
        std::string(full_inventory_gap_search_direction_override_valid_ ? "true" : "false"));
      set_state(State::FULL_INVENTORY_SEARCH_GAP, "[FULL_INVENTORY] " + detail);
    } else if (single_cabinet_motion_active_) {
      single_cabinet_gap_searching_ = true;
      publish_single_cabinet_log("waiting/searching gap=" + active_single_cabinet_gap_id());
      set_single_cabinet_state(State::SINGLE_CABINET_WAITING_GAP, detail);
    } else {
      set_state(State::SEARCH_GAP, detail);
    }
    publish_gap_context();
  }

  void begin_waiting_gap_confirmation_flow(const std::string & detail)
  {
    latest_gap_ = agv_inventory_system::msg::GapStatus{};
    set_wait_gap_phase(WaitGapPhase::STOP_BEFORE_DETECT);
    publish_entry_side();
    if (full_inventory_active_) {
      single_cabinet_gap_searching_ = false;
      begin_full_inventory_post_gap_detect_advance_flow();
      return;
    } else if (single_cabinet_motion_active_) {
      single_cabinet_gap_searching_ = false;
      set_single_cabinet_state(State::SINGLE_CABINET_WAITING_GAP, detail);
    } else {
      set_state(State::WAITING_GAP, detail);
    }
    publish_gap_context();
  }

  void begin_waiting_gap_fallback_flow(const std::string & detail)
  {
    wait_gap_motion_target_distance_ = std::abs(post_track_retreat_distance_);
    wait_gap_motion_direction_ = post_track_retreat_distance_ >= 0.0 ? -1.0 : 1.0;
    latest_gap_ = agv_inventory_system::msg::GapStatus{};
    reset_segment_distance();
    set_wait_gap_phase(WaitGapPhase::RETREATING);
    publish_entry_side();
    if (full_inventory_active_) {
      single_cabinet_gap_searching_ = false;
      fail_full_inventory("SEARCH_GAP legacy fallback bypassed for full_inventory: " + detail);
      return;
    } else if (single_cabinet_motion_active_) {
      single_cabinet_gap_searching_ = false;
      set_single_cabinet_state(State::SINGLE_CABINET_WAITING_GAP, detail);
    } else {
      set_state(State::WAITING_GAP, detail);
    }
    publish_gap_context();
  }

  bool validate_entering_runtime_config(std::string & reason) const
  {
    reason.clear();
    if (!current_entry_profile_valid_) {
      reason = "入缝深度规划尚未生成";
      return false;
    }
    if (!std::isfinite(entry_right_target_yaw_rad_) || !std::isfinite(entry_left_target_yaw_rad_)) {
      reason = "entry_right_target_yaw_rad / entry_left_target_yaw_rad 必须为有限值";
      return false;
    }
    if (!std::isfinite(entry_align_yaw_tolerance_rad_) || entry_align_yaw_tolerance_rad_ <= 0.0) {
      reason = "entry_align_yaw_tolerance_rad 必须为正数";
      return false;
    }
    if (entry_turn_yaw_stable_required_count_ <= 0) {
      reason = "entry_turn_yaw_stable_required_count 必须为正整数";
      return false;
    }
    if (!std::isfinite(entry_turn_angular_speed_) || entry_turn_angular_speed_ <= 0.0) {
      reason = "entry_turn_angular_speed 必须为正数";
      return false;
    }
    if (!std::isfinite(entry_straight_speed_) || entry_straight_speed_ <= 0.0) {
      reason = "entry_straight_speed 必须为正数";
      return false;
    }
    if (!std::isfinite(entry_straight_yaw_kp_) || entry_straight_yaw_kp_ < 0.0) {
      reason = "entry_straight_yaw_kp 不能为负数";
      return false;
    }
    if (!std::isfinite(entry_straight_yaw_deadband_rad_) ||
      entry_straight_yaw_deadband_rad_ < 0.0)
    {
      reason = "entry_straight_yaw_deadband_rad 不能为负数";
      return false;
    }
    if (!std::isfinite(entry_straight_max_angular_speed_) ||
      entry_straight_max_angular_speed_ < 0.0)
    {
      reason = "entry_straight_max_angular_speed 不能为负数";
      return false;
    }
    if (!std::isfinite(entry_side_hold_target_distance_m_) ||
      entry_side_hold_target_distance_m_ <= 0.0)
    {
      reason = "entry_side_hold_target_distance_m 必须为正数";
      return false;
    }
    if (!std::isfinite(entry_side_hold_kp_) || entry_side_hold_kp_ < 0.0) {
      reason = "entry_side_hold_kp 不能为负数";
      return false;
    }
    if (entry_side_hold_min_valid_points_ <= 0) {
      reason = "entry_side_hold_min_valid_points 必须为正整数";
      return false;
    }
    return true;
  }

  bool current_odom_ready_for_entry(std::string & reason) const
  {
    reason.clear();
    if (!latest_odom_ || !has_yaw_) {
      reason = "无可用里程计/航向";
      return false;
    }
    if (latest_odom_time_.nanoseconds() == 0) {
      reason = "尚未收到有效里程计时间戳";
      return false;
    }
    const double max_age = std::max(1.0, 2.0 * odom_primary_timeout_sec_);
    if ((this->now() - latest_odom_time_).seconds() > max_age) {
      reason = "里程计超时";
      return false;
    }
    return true;
  }

  bool current_entering_gap_control_yaw(
    const Pose2D & odom_pose,
    EnteringYawControl & control)
  {
    control = EnteringYawControl{};
    if (!odom_pose.valid || !std::isfinite(odom_pose.yaw)) {
      control.pose_note = "odom pose invalid";
      return false;
    }

    const std::string target_frame = sanitize_frame_id(nav2_goal_frame_.empty() ? "map" : nav2_goal_frame_);
    const std::string source_frame = sanitize_frame_id("base_footprint");
    try {
      const auto tf_msg = tf_buffer_->lookupTransform(
        target_frame,
        source_frame,
        tf2::TimePointZero,
        tf2::durationFromSec(0.05));
      control.yaw = yaw_from_quaternion(tf_msg.transform.rotation);
      control.yaw_frame = target_frame;
      control.pose_note = "tf_latest " + target_frame + "<-" + source_frame;
      control.valid = true;
      return true;
    } catch (const tf2::TransformException & ex) {
      control.yaw = odom_pose.yaw;
      control.yaw_frame = "odom_fallback";
      control.pose_note =
        "tf_error=" + std::string(ex.what()) + ", odom_frame=" + odom_pose.frame_id;
      control.valid = true;
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "entering_gap map yaw unavailable, fallback to odom yaw: %s",
        control.pose_note.c_str());
      return true;
    }
  }

  void fail_entering_gap(const std::string & reason)
  {
    publish_stop();
    if (full_inventory_active_) {
      fail_full_inventory(reason);
      return;
    }
    if (single_cabinet_motion_active_) {
      fail_single_cabinet_motion(reason);
      return;
    }
    mission_active_ = false;
    set_state(State::ERROR, reason);
  }

  void begin_entering_gap_flow(const std::string & detail)
  {
    if (!current_entry_profile_valid_) {
      std::string reason;
      if (!configure_current_entry_profile(reason)) {
        fail_entering_gap("无法生成入缝深度规划: " + reason);
        return;
      }
    }

    std::string reason;
    if (!validate_entering_runtime_config(reason)) {
      fail_entering_gap("入缝参数非法: " + reason);
      return;
    }
    if (!current_odom_ready_for_entry(reason)) {
      fail_entering_gap("入缝前里程计异常: " + reason);
      return;
    }

    const Pose2D current = current_pose_2d();
    if (!current.valid) {
      fail_entering_gap("入缝前当前位姿无效");
      return;
    }
    EnteringYawControl yaw_control;
    if (!current_entering_gap_control_yaw(current, yaw_control)) {
      fail_entering_gap("入缝前当前航向无效: " + yaw_control.pose_note);
      return;
    }

    reset_entry_gap_runtime();
    request_recognizer_enable(false);
    set_recognizer_topic_enabled(false, true);
    set_distance_estimator_enabled(false, true);
    set_gap_detector_enabled(false);
    entry_turn_start_yaw_ = yaw_control.yaw;
    entry_turn_start_yaw_valid_ = true;
    double base_entry_yaw = 0.0;
    if (current_entry_side_ == "right") {
      base_entry_yaw = normalize_angle(entry_right_target_yaw_rad_);
    } else if (current_entry_side_ == "left") {
      base_entry_yaw = normalize_angle(entry_left_target_yaw_rad_);
    } else {
      fail_entering_gap("entry_side 非法，无法确定固定 map Y 入缝目标 yaw: " + current_entry_side_);
      return;
    }
    const SearchDirection entering_gap_search_direction = effective_gap_search_direction();
    entry_motion_mode_ = resolve_entry_motion_mode(entering_gap_search_direction);
    target_gap_yaw_ = entry_motion_mode_ == EntryMotionMode::REVERSE_ENTRY ?
      normalize_angle(base_entry_yaw + M_PI) : base_entry_yaw;
    target_gap_yaw_valid_ = true;
    record_safe_exit_gap_yaw_context("ENTERING_GAP_PREPARE");
    RCLCPP_INFO(
      get_logger(),
      "[mission_manager][entering_gap] fixed map-y target yaw entry_side=%s "
      "entry_motion_mode=%s gap_search_direction=%s base_entry_yaw=%.4f target_gap_yaw=%.4f "
      "entry_right_target_yaw_rad=%.4f entry_left_target_yaw_rad=%.4f "
      "target_yaw_source=fixed_map_y yaw_frame=%s current_yaw=%.4f yaw_error=%.4f pose_note=%s",
      current_entry_side_.c_str(),
      entry_motion_mode_to_string(entry_motion_mode_).c_str(),
      search_direction_to_string(entering_gap_search_direction).c_str(),
      base_entry_yaw,
      target_gap_yaw_,
      entry_right_target_yaw_rad_,
      entry_left_target_yaw_rad_,
      yaw_control.yaw_frame.c_str(),
      yaw_control.yaw,
      normalize_angle(target_gap_yaw_ - yaw_control.yaw),
      yaw_control.pose_note.c_str());
    publish_motion_log(
      "[entering_gap] fixed map-y target yaw entry_side=" + current_entry_side_ +
      " entry_motion_mode=" + entry_motion_mode_to_string(entry_motion_mode_) +
      " gap_search_direction=" + search_direction_to_string(entering_gap_search_direction) +
      " original_gap_search_direction=" + search_direction_to_string(current_gap_plan_.search_direction) +
      " override_valid=" +
      std::string(full_inventory_gap_search_direction_override_valid_ ? "true" : "false") +
      " base_entry_yaw=" + format_fixed(base_entry_yaw, 4) +
      " target_gap_yaw=" + format_fixed(target_gap_yaw_, 4) +
      " entry_right_target_yaw_rad=" + format_fixed(entry_right_target_yaw_rad_, 4) +
      " entry_left_target_yaw_rad=" + format_fixed(entry_left_target_yaw_rad_, 4) +
      " target_yaw_source=fixed_map_y yaw_frame=" + yaw_control.yaw_frame +
      " current_yaw=" + format_fixed(yaw_control.yaw, 4) +
      " yaw_error=" + format_fixed(normalize_angle(target_gap_yaw_ - yaw_control.yaw), 4) +
      " pose_note=" + yaw_control.pose_note);
    reset_segment_distance();
    set_entry_gap_phase(
      EntryGapPhase::ENTERING_TURN,
      "entry_turn_start_yaw=" + std::to_string(entry_turn_start_yaw_) +
      " target_gap_yaw=" + std::to_string(target_gap_yaw_) +
      " base_entry_yaw=" + std::to_string(base_entry_yaw) +
      " entry_motion_mode=" + entry_motion_mode_to_string(entry_motion_mode_) +
      " gap_search_direction=" + search_direction_to_string(current_gap_plan_.search_direction) +
      " target_yaw_source=fixed_map_y" +
      " yaw_frame=" + yaw_control.yaw_frame +
      " current_yaw=" + format_fixed(yaw_control.yaw, 4) +
      " yaw_error=" + format_fixed(normalize_angle(target_gap_yaw_ - yaw_control.yaw), 4) +
      " pose_note=" + yaw_control.pose_note +
      " direction=" + entry_turn_direction_text());
    const std::string state_detail =
      detail + "，开始转入缝隙 target_straight_distance=" +
      std::to_string(target_straight_distance_) + "m";
    if (full_inventory_active_) {
      publish_full_inventory_log(
        "entering gap for cabinet=" + std::to_string(current_target_cabinet_));
      set_state(State::FULL_INVENTORY_ENTERING_GAP, "[FULL_INVENTORY] " + state_detail);
    } else if (single_cabinet_motion_active_) {
      publish_single_cabinet_log("entering gap for cabinet=" + std::to_string(current_target_cabinet_));
      set_single_cabinet_state(State::SINGLE_CABINET_ENTERING_GAP, state_detail);
    } else {
      set_state(State::ENTERING_GAP, state_detail);
    }
  }

  bool run_wait_gap_linear_motion(double direction, double speed, double target_distance)
  {
    if (target_distance <= 1e-4) {
      publish_stop();
      return true;
    }

    if (segment_distance() < target_distance) {
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = direction * std::abs(speed);
      cmd.angular.z = 0.0;
      cmd_pub_->publish(cmd);
      return false;
    }

    publish_stop();
    return true;
  }

  bool start_next_adjust_motion()
  {
    if (next_adjust_sequence_index_ >= gap_adjust_sequence_.size()) {
      return false;
    }

    current_adjust_index_ = static_cast<int>(next_adjust_sequence_index_);
    current_adjust_offset_ = gap_adjust_sequence_[next_adjust_sequence_index_];
    ++next_adjust_sequence_index_;

    wait_gap_motion_target_distance_ = std::abs(current_adjust_offset_);
    wait_gap_motion_direction_ = current_adjust_offset_ >= 0.0 ? 1.0 : -1.0;
    reset_segment_distance();
    set_wait_gap_phase(WaitGapPhase::POSE_ADJUSTING);

    publish_log(
      "缝隙检测失败达到阈值，执行位姿调整 index=" +
      std::to_string(current_adjust_index_) +
      " offset=" + std::to_string(current_adjust_offset_) + "m");
    publish_gap_context();
    return true;
  }

  bool validate_search_gap_timeout_config(std::string & reason) const
  {
    reason.clear();
    if (!std::isfinite(search_gap_forward_timeout_sec_) || search_gap_forward_timeout_sec_ <= 0.0) {
      reason = "search_gap_forward_timeout_sec 必须为正数";
      return false;
    }
    if (!std::isfinite(search_gap_backward_timeout_sec_) || search_gap_backward_timeout_sec_ <= 0.0) {
      reason = "search_gap_backward_timeout_sec 必须为正数";
      return false;
    }
    return true;
  }

  double get_current_search_gap_timeout_sec() const
  {
    if (effective_gap_search_direction() == SearchDirection::BACKWARD) {
      return search_gap_backward_timeout_sec_;
    }
    return search_gap_forward_timeout_sec_;
  }

  void begin_full_inventory_post_gap_detect_advance_flow()
  {
    publish_stop();
    set_gap_detector_enabled(false);
    set_distance_estimator_enabled(false, true);
    request_recognizer_enable(false);
    set_recognizer_topic_enabled(false, true);

    if (!post_gap_detect_advance_enabled_ ||
      !std::isfinite(post_gap_detect_advance_distance_m_) ||
      post_gap_detect_advance_distance_m_ <= 1e-4)
    {
      publish_full_inventory_log(
        "gap detected stable, post advance disabled, entering gap target=" +
        std::to_string(current_target_cabinet_));
      begin_entering_gap_flow("gap detected stable, post advance disabled");
      return;
    }

    full_inventory_post_gap_advance_start_ = this->now();
    reset_segment_distance();
    publish_full_inventory_log(
      "gap detected stable, post advance direction=" +
      search_direction_to_string(effective_gap_search_direction()) +
      " distance=" + format_seconds(post_gap_detect_advance_distance_m_));
    set_state(
      State::FULL_INVENTORY_POST_GAP_DETECT_ADVANCE,
      "[FULL_INVENTORY] gap detected stable, post advance direction=" +
      search_direction_to_string(effective_gap_search_direction()) +
      " distance=" + format_seconds(post_gap_detect_advance_distance_m_));
  }

  void handle_full_inventory_post_gap_detect_advance_state()
  {
    request_recognizer_enable(false);
    set_recognizer_topic_enabled(false);
    set_distance_estimator_enabled(false);
    set_gap_detector_enabled(false);

    if (full_inventory_post_gap_advance_start_.nanoseconds() == 0) {
      full_inventory_post_gap_advance_start_ = this->now();
      reset_segment_distance();
    }

    const double timeout =
      std::isfinite(post_gap_detect_advance_timeout_sec_) ?
      std::max(0.1, post_gap_detect_advance_timeout_sec_) : 8.0;
    const double elapsed = (this->now() - full_inventory_post_gap_advance_start_).seconds();
    if (elapsed >= timeout) {
      publish_stop();
      fail_full_inventory(
        "post gap advance timeout target=" +
        std::to_string(current_target_cabinet_) +
        " elapsed=" + format_seconds(elapsed) +
        " timeout=" + format_seconds(timeout));
      return;
    }

    const double direction =
      effective_gap_search_direction() == SearchDirection::BACKWARD ? -1.0 : 1.0;
    const bool done = run_wait_gap_linear_motion(
      direction,
      post_gap_detect_advance_speed_,
      std::abs(post_gap_detect_advance_distance_m_));
    if (done) {
      full_inventory_post_gap_advance_start_ =
        rclcpp::Time(0, 0, get_clock()->get_clock_type());
      publish_full_inventory_log(
        "post gap advance done, entering gap target=" +
        std::to_string(current_target_cabinet_));
      begin_entering_gap_flow("post gap advance done");
      return;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "[mission_manager][FULL_INVENTORY] post_gap_advance target=%d direction=%s traveled=%.2f/%.2f "
      "speed=%.3f elapsed=%.2f/%.2f",
      current_target_cabinet_,
      search_direction_to_string(effective_gap_search_direction()).c_str(),
      segment_distance(),
      std::abs(post_gap_detect_advance_distance_m_),
      post_gap_detect_advance_speed_,
      elapsed,
      timeout);
  }

  void handle_search_gap_state()
  {
    publish_entry_side();
    publish_gap_context();

    const std::string detected_side = latest_gap_.active_side.empty() ?
      latest_gap_.side : latest_gap_.active_side;
    if (latest_gap_.allow_enter && normalize_entry_side(detected_side) == current_entry_side_) {
      publish_stop();
      if (full_inventory_active_) {
        begin_full_inventory_post_gap_detect_advance_flow();
        return;
      }
      begin_waiting_gap_confirmation_flow("SEARCH_GAP检测到候选间隙，停车确认宽度/稳定性/安全距离");
      return;
    }

    if (search_gap_start_.nanoseconds() == 0) {
      search_gap_start_ = this->now();
    }

    const double timeout_sec = get_current_search_gap_timeout_sec();
    const double elapsed_sec = (this->now() - search_gap_start_).seconds();
    if (elapsed_sec >= std::max(0.1, timeout_sec)) {
      publish_stop();
      if (full_inventory_active_) {
        fail_full_inventory(
          "SEARCH_GAP timeout target=" +
          std::to_string(current_target_cabinet_) +
          " direction=" + search_direction_to_string(effective_gap_search_direction()) +
          " timeout=" + format_seconds(timeout_sec) +
          " elapsed=" + format_seconds(elapsed_sec));
        return;
      }
      begin_waiting_gap_fallback_flow(
        "SEARCH_GAP超时未检测到目标间隙，执行原固定回退序列作为fallback微调 direction=" +
        search_direction_to_string(effective_gap_search_direction()) +
        " timeout=" + format_seconds(timeout_sec) +
        " elapsed=" + format_seconds(elapsed_sec));
      return;
    }

    const double direction =
      effective_gap_search_direction() == SearchDirection::BACKWARD ? -1.0 : 1.0;
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = direction * std::abs(search_gap_speed_);
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "search_gap: target_cabinet=%d target_side=%s entry_side=%s gap_search_direction=%s "
      "original_gap_search_direction=%s override_valid=%s timeout_sec=%.2f elapsed_sec=%.2f "
      "expected_gap=%s cmd.linear.x=%.3f",
      current_target_cabinet_,
      current_target_side_.c_str(),
      current_entry_side_.c_str(),
      search_direction_to_string(effective_gap_search_direction()).c_str(),
      search_direction_to_string(current_gap_plan_.search_direction).c_str(),
      full_inventory_gap_search_direction_override_valid_ ? "true" : "false",
      timeout_sec,
      elapsed_sec,
      gap_plan_to_string(current_gap_plan_).c_str(),
      cmd.linear.x);
  }

  void handle_waiting_gap_state()
  {
    switch (wait_gap_phase_) {
      case WaitGapPhase::RETREATING: {
        const bool done = run_wait_gap_linear_motion(
          wait_gap_motion_direction_,
          retreat_speed_,
          wait_gap_motion_target_distance_);
        if (done) {
          set_wait_gap_phase(WaitGapPhase::STOP_BEFORE_DETECT);
          publish_log("固定回退完成，停车稳定后开始缝隙检测");
        }
        break;
      }

      case WaitGapPhase::STOP_BEFORE_DETECT: {
        publish_stop();
        if ((this->now() - wait_gap_phase_start_).seconds() >= wait_gap_stop_settle_sec_) {
          set_wait_gap_phase(WaitGapPhase::DETECTING_GAP);
          wait_gap_detect_cycle_start_ = this->now();
          publish_log("进入缝隙检测周期");
          publish_gap_context();
        }
        break;
      }

      case WaitGapPhase::DETECTING_GAP: {
        publish_stop();
        publish_gap_context();

        const std::string detected_side = latest_gap_.active_side.empty() ?
          latest_gap_.side : latest_gap_.active_side;
        if (latest_gap_.allow_enter && normalize_entry_side(detected_side) == current_entry_side_) {
          begin_entering_gap_flow("检测到可入缝");
          return;
        }

        if (wait_gap_detect_cycle_start_.nanoseconds() == 0) {
          wait_gap_detect_cycle_start_ = this->now();
          break;
        }

        // “一次完整检测周期失败”定义：在 DETECTING_GAP 连续检测超过 gap_detect_cycle_timeout_sec
        // 且始终未满足 allow_enter，则记为失败 1 次。
        if ((this->now() - wait_gap_detect_cycle_start_).seconds() >= gap_detect_cycle_timeout_sec_) {
          ++wait_gap_failed_cycle_count_;
          publish_log(
            "缝隙检测周期失败 count=" + std::to_string(wait_gap_failed_cycle_count_) +
            "/" + std::to_string(std::max(1, gap_failures_before_adjust_)));
          publish_gap_context();

          wait_gap_detect_cycle_start_ = this->now();

          if (wait_gap_failed_cycle_count_ >= std::max(1, gap_failures_before_adjust_)) {
            if (!start_next_adjust_motion()) {
              mission_active_ = false;
              publish_stop();
              if (full_inventory_active_) {
                fail_full_inventory("缝隙检测失败且调整序列耗尽");
              } else if (single_cabinet_motion_active_) {
                fail_single_cabinet_motion("缝隙检测失败且调整序列耗尽");
              } else {
                set_state(State::ERROR, "缝隙检测失败且调整序列耗尽");
              }
            }
          }
        }
        break;
      }

      case WaitGapPhase::POSE_ADJUSTING: {
        const bool done = run_wait_gap_linear_motion(
          wait_gap_motion_direction_,
          gap_adjust_speed_,
          wait_gap_motion_target_distance_);
        if (done) {
          wait_gap_failed_cycle_count_ = 0;
          wait_gap_detect_cycle_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
          set_wait_gap_phase(WaitGapPhase::STOP_BEFORE_DETECT);
          publish_log("位姿调整完成，重新开始缝隙检测");
          publish_gap_context();
        }
        break;
      }

      case WaitGapPhase::IDLE:
      default: {
        publish_stop();
        set_wait_gap_phase(WaitGapPhase::STOP_BEFORE_DETECT);
        break;
      }
    }
  }

  double entry_straight_traveled(const Pose2D & current) const
  {
    if (!straight_start_pose_.valid) {
      return 0.0;
    }
    return std::hypot(current.x - straight_start_pose_.x, current.y - straight_start_pose_.y);
  }

  double entry_turn_timeout_sec() const
  {
    return entry_turn_timeout_sec_;
  }

  double entry_straight_timeout_sec() const
  {
    return entry_straight_timeout_sec_;
  }

  void log_entering_gap_status(
    const EnteringSafetyEval & safety,
    const Pose2D & current,
    double yaw_error,
    double angular_z,
    double traveled,
    bool turn_done,
    bool straight_done,
    const EntrySideHoldEval & side_hold,
    const std::string & yaw_frame = "odom_fallback",
    const std::string & pose_note = "not provided",
    double entry_linear_cmd = 0.0,
    double final_linear_cmd = 0.0)
  {
    std::ostringstream straight_start_text;
    if (straight_start_pose_.valid) {
      straight_start_text << "x=" << straight_start_pose_.x
                          << ",y=" << straight_start_pose_.y
                          << ",yaw=" << straight_start_pose_.yaw;
    } else {
      straight_start_text << "invalid";
    }
    const std::string straight_start = straight_start_text.str();
    const std::string entry_side_text = side_hold.status == "NOT_EVALUATED" ?
      current_entry_side_ : side_hold.entry_side;

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "entering_gap: phase=%s entry_motion_mode=%s entry_turn_start_yaw=%.3f target_gap_yaw=%.3f "
      "yaw_frame=%s current_yaw=%.3f yaw_error=%.3f pose_note=%s "
      "entry_linear_cmd=%.3f final_linear_cmd=%.3f angular.z=%.3f straight_start_pose=(%s) "
      "traveled=%.3f target_straight_distance=%.3f turn_done=%d straight_done=%d "
      "entry_side=%s left_side_dist=%.3f right_side_dist=%.3f control_side_dist=%.3f "
      "side_error=%.3f yaw_hold_cmd=%.3f side_distance_cmd=%.3f final_angular_cmd=%.3f "
      "side_hold_status=%s side_points[left=%zu,right=%zu] safety_stop=%d safety_reason=%s "
      "motion_direction=%s front=%.3f front_side=%.3f side_dist=%.3f speed_scale=%.2f",
      entry_gap_phase_to_string(entry_gap_phase_).c_str(),
      entry_motion_mode_to_string(entry_motion_mode_).c_str(),
      entry_turn_start_yaw_,
      target_gap_yaw_,
      yaw_frame.c_str(),
      current.yaw,
      yaw_error,
      pose_note.c_str(),
      entry_linear_cmd,
      final_linear_cmd,
      angular_z,
      straight_start.c_str(),
      traveled,
      target_straight_distance_,
      turn_done ? 1 : 0,
      straight_done ? 1 : 0,
      entry_side_text.c_str(),
      side_hold.left_side_dist,
      side_hold.right_side_dist,
      side_hold.control_side_dist,
      side_hold.side_error,
      side_hold.yaw_hold_cmd,
      side_hold.side_distance_cmd,
      side_hold.final_angular_cmd,
      side_hold.status.c_str(),
      side_hold.left_valid_points,
      side_hold.right_valid_points,
      entry_stopped_by_safety_ ? 1 : 0,
      safety.block_reason.c_str(),
      safety.motion_direction.c_str(),
      safety.front_min_dist,
      safety.front_side_min_dist,
      safety.side_min_dist,
      safety.speed_scale);
  }

  void log_entering_gap_status(
    const EnteringSafetyEval & safety,
    const Pose2D & current,
    double yaw_error,
    double angular_z,
    double traveled,
    bool turn_done,
    bool straight_done,
    const std::string & yaw_frame = "odom_fallback",
    const std::string & pose_note = "not provided",
    double entry_linear_cmd = 0.0,
    double final_linear_cmd = 0.0)
  {
    EntrySideHoldEval side_hold;
    log_entering_gap_status(
      safety, current, yaw_error, angular_z, traveled, turn_done, straight_done,
      side_hold, yaw_frame, pose_note, entry_linear_cmd, final_linear_cmd);
  }

  void handle_entering_gap_state()
  {
    std::string reason;
    if (!current_odom_ready_for_entry(reason)) {
      fail_entering_gap("入缝里程计异常: " + reason);
      return;
    }

    Pose2D odom_current = current_pose_2d();
    if (!odom_current.valid || !std::isfinite(odom_current.x) || !std::isfinite(odom_current.y) ||
      !std::isfinite(odom_current.yaw))
    {
      fail_entering_gap("入缝里程计位姿无效");
      return;
    }
    EnteringYawControl yaw_control;
    if (!current_entering_gap_control_yaw(odom_current, yaw_control) ||
      !std::isfinite(yaw_control.yaw))
    {
      fail_entering_gap("入缝当前航向无效: " + yaw_control.pose_note);
      return;
    }
    Pose2D yaw_log_pose = odom_current;
    yaw_log_pose.yaw = yaw_control.yaw;

    if (entry_gap_phase_ == EntryGapPhase::IDLE) {
      begin_entering_gap_flow("ENTERING_GAP运行时补初始化");
      return;
    }

    const auto safety = evaluate_entering_safety();
    const double yaw_error = normalize_angle(target_gap_yaw_ - yaw_control.yaw);
    double traveled = straight_start_pose_.valid ? entry_straight_traveled(odom_current) : 0.0;
    double angular_z = 0.0;
    bool turn_done = entry_turn_completed_;
    bool straight_done = entry_straight_completed_;

    if (safety.blocked) {
      entry_stopped_by_safety_ = true;
      log_entering_gap_status(
        safety, yaw_log_pose, yaw_error, angular_z, traveled, turn_done, straight_done,
        yaw_control.yaw_frame, yaw_control.pose_note);
      fail_entering_gap("入缝被安全策略阻塞: " + safety.block_reason);
      return;
    }

    switch (entry_gap_phase_) {
      case EntryGapPhase::ENTERING_TURN: {
        const double yaw_tolerance = std::max(0.001, std::abs(entry_align_yaw_tolerance_rad_));
        const int required_count = std::max(1, entry_turn_yaw_stable_required_count_);
        if (std::abs(yaw_error) <= yaw_tolerance) {
          ++entry_turn_yaw_stable_count_;
        } else {
          entry_turn_yaw_stable_count_ = 0;
        }
        turn_done = entry_turn_yaw_stable_count_ >= required_count;
        if (turn_done) {
          entry_turn_completed_ = true;
          publish_stop();
          log_entering_gap_status(
            safety, yaw_log_pose, yaw_error, angular_z, traveled, true, false,
            yaw_control.yaw_frame, yaw_control.pose_note);
          set_entry_gap_phase(
            EntryGapPhase::ENTERING_STRAIGHT_ALIGN,
            "转向完成，停止持续转向 yaw_error=" + format_fixed(yaw_error, 4) +
            " tolerance=" + format_fixed(yaw_tolerance, 4) +
            " stable_count=" + std::to_string(entry_turn_yaw_stable_count_) +
            " required_count=" + std::to_string(required_count) +
            " turn_done_reason=stable_yaw_in_tolerance" +
            " yaw_frame=" + yaw_control.yaw_frame +
            " current_yaw=" + format_fixed(yaw_control.yaw, 4) +
            " pose_note=" + yaw_control.pose_note);
          return;
        }

        const double turn_elapsed = (this->now() - entry_gap_phase_start_).seconds();
        const double turn_timeout = entry_turn_timeout_sec();
        if (turn_elapsed > turn_timeout) {
          log_entering_gap_status(
            safety, yaw_log_pose, yaw_error, angular_z, traveled, false, false,
            yaw_control.yaw_frame, yaw_control.pose_note);
          RCLCPP_ERROR(
            get_logger(),
            "[mission_manager][ENTERING_TURN] 入缝转向超时 elapsed=%.2f timeout=%.2f "
            "current_yaw=%.4f target_yaw=%.4f yaw_error=%.4f "
            "entry_turn_angular_speed=%.3f entry_align_yaw_tolerance_rad=%.3f "
            "stable_count=%d required_count=%d yaw_frame=%s pose_note=%s",
            turn_elapsed,
            turn_timeout,
            yaw_control.yaw,
            target_gap_yaw_,
            yaw_error,
            entry_turn_angular_speed_,
            entry_align_yaw_tolerance_rad_,
            entry_turn_yaw_stable_count_,
            required_count,
            yaw_control.yaw_frame.c_str(),
            yaw_control.pose_note.c_str());
          fail_entering_gap("入缝转向超时，未达到目标缝隙航向");
          return;
        }

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        angular_z = std::clamp(
          yaw_error,
          -std::abs(entry_turn_angular_speed_),
          std::abs(entry_turn_angular_speed_)) * safety.speed_scale;
        cmd.angular.z = angular_z;
        cmd_pub_->publish(cmd);
        log_entering_gap_status(
          safety, yaw_log_pose, yaw_error, angular_z, traveled, false, false,
          yaw_control.yaw_frame, yaw_control.pose_note);
        break;
      }

      case EntryGapPhase::ENTERING_STRAIGHT_ALIGN: {
        const double yaw_tolerance = std::max(0.001, std::abs(entry_align_yaw_tolerance_rad_));
        const int required_count = std::max(1, entry_turn_yaw_stable_required_count_);
        publish_stop();
        straight_start_pose_ = odom_current;
        straight_start_pose_.yaw = yaw_control.yaw;
        entry_last_traveled_ = 0.0;
        traveled = 0.0;
        update_safe_exit_gap_recovery_context(
          0.0,
          true,
          entry_motion_mode_ == EntryMotionMode::REVERSE_ENTRY ?
          "MOVING_TO_GRID_CENTER_START_REVERSE_ENTRY" : "MOVING_TO_GRID_CENTER_START_FORWARD_ENTRY");
        log_entering_gap_status(
          safety, yaw_log_pose, yaw_error, angular_z, traveled, true, false,
          yaw_control.yaw_frame, yaw_control.pose_note);
        set_entry_gap_phase(
          EntryGapPhase::MOVING_TO_GRID_CENTER,
          "记录 straight_start_pose，开始直行到深度格中心 yaw_error=" + format_fixed(yaw_error, 4) +
          " tolerance=" + format_fixed(yaw_tolerance, 4) +
          " stable_count=" + std::to_string(entry_turn_yaw_stable_count_) +
          " required_count=" + std::to_string(required_count) +
          " turn_done_reason=stable_yaw_in_tolerance" +
          " yaw_frame=" + yaw_control.yaw_frame +
          " current_yaw=" + format_fixed(yaw_control.yaw, 4) +
          " pose_note=" + yaw_control.pose_note);
        break;
      }

      case EntryGapPhase::MOVING_TO_GRID_CENTER: {
        if (!straight_start_pose_.valid) {
          fail_entering_gap("入缝直行起点无效");
          return;
        }

        traveled = entry_straight_traveled(odom_current);
        if (!std::isfinite(traveled) || traveled < -0.15) {
          log_entering_gap_status(
            safety, yaw_log_pose, yaw_error, angular_z, traveled, true, false,
            yaw_control.yaw_frame, yaw_control.pose_note);
          fail_entering_gap("入缝直行里程异常，traveled=" + std::to_string(traveled));
          return;
        }
        update_safe_exit_gap_recovery_context(
          traveled,
          true,
          entry_motion_mode_ == EntryMotionMode::REVERSE_ENTRY ?
          "MOVING_TO_GRID_CENTER_REVERSE_ENTRY" : "MOVING_TO_GRID_CENTER_FORWARD_ENTRY");

        const double straight_elapsed = (this->now() - entry_gap_phase_start_).seconds();
        const double straight_timeout = entry_straight_timeout_sec();
        const double estimated_speed = straight_elapsed > 1e-3 ? traveled / straight_elapsed : 0.0;
        double yaw_hold_cmd = 0.0;
        if (std::abs(yaw_error) >= entry_straight_yaw_deadband_rad_) {
          yaw_hold_cmd = std::clamp(
            entry_straight_yaw_kp_ * yaw_error,
            -std::abs(entry_straight_max_angular_speed_),
            std::abs(entry_straight_max_angular_speed_));
        }
        EntrySideHoldEval side_hold;
        (void)evaluate_entry_side_distance_hold(side_hold);
        side_hold.yaw_hold_cmd = yaw_hold_cmd;

        const double angular_limit = std::abs(entry_straight_max_angular_speed_);
        const double limited_angular = std::clamp(
          yaw_hold_cmd + side_hold.side_distance_cmd,
          -angular_limit,
          angular_limit);
        const double entry_linear_cmd = apply_entry_motion_direction(entry_straight_speed_);
        const double final_linear_cmd = entry_linear_cmd * safety.speed_scale;
        const double stop_linear_cmd = 0.0;

        if (straight_elapsed > straight_timeout) {
          side_hold.final_angular_cmd = limited_angular * safety.speed_scale;
          log_entering_gap_status(
            safety, yaw_log_pose, yaw_error, angular_z, traveled, true, false, side_hold,
            yaw_control.yaw_frame, yaw_control.pose_note, entry_linear_cmd, stop_linear_cmd);
          publish_stop();
          RCLCPP_ERROR(
            get_logger(),
            "[mission_manager][MOVING_TO_GRID_CENTER] 入缝直行超时 elapsed=%.2f timeout=%.2f "
            "target_straight_distance=%.3f traveled=%.3f entry_straight_speed=%.3f "
            "entry_linear_cmd=%.3f final_linear_cmd=%.3f cmd.linear.x=0.000 "
            "speed_scale=%.2f estimated_speed=%.3f "
            "entry_motion_mode=%s motion_direction=%s "
            "current_pose=(x=%.3f,y=%.3f) current_yaw=%.4f target_yaw=%.4f yaw_error=%.4f "
            "yaw_frame=%s pose_note=%s side_distance_cmd=%.3f side_hold_status=%s",
            straight_elapsed,
            straight_timeout,
            target_straight_distance_,
            traveled,
            entry_straight_speed_,
            entry_linear_cmd,
            stop_linear_cmd,
            safety.speed_scale,
            estimated_speed,
            entry_motion_mode_to_string(entry_motion_mode_).c_str(),
            safety.motion_direction.c_str(),
            odom_current.x,
            odom_current.y,
            yaw_control.yaw,
            target_gap_yaw_,
            yaw_error,
            yaw_control.yaw_frame.c_str(),
            yaw_control.pose_note.c_str(),
            side_hold.side_distance_cmd,
            side_hold.status.c_str());
          fail_entering_gap("入缝直行超时，未到达深度格中心");
          return;
        }

        if (traveled >= target_straight_distance_) {
          entry_straight_completed_ = true;
          straight_done = true;
          publish_stop();
          log_entering_gap_status(
            safety, yaw_log_pose, yaw_error, angular_z, traveled, true, true,
            yaw_control.yaw_frame, yaw_control.pose_note);
          if (full_inventory_active_) {
            single_cabinet_last_entering_straight_distance_ =
              std::max(traveled, target_straight_distance_);
            update_safe_exit_gap_recovery_context(
              single_cabinet_last_entering_straight_distance_,
              true,
              "FULL_INVENTORY_IN_GAP_SCAN");
            set_state(
              State::FULL_INVENTORY_IN_GAP_SCAN,
              "[FULL_INVENTORY] reached depth-grid center, start in-gap scan cabinet=" +
              std::to_string(current_target_cabinet_));
          } else if (single_cabinet_motion_active_) {
            single_cabinet_last_entering_straight_distance_ =
              std::max(traveled, target_straight_distance_);
            update_safe_exit_gap_recovery_context(
              single_cabinet_last_entering_straight_distance_,
              true,
              "SINGLE_CABINET_IN_GAP_SCAN");
            State scan_state = State::SINGLE_CABINET_IN_GAP_SCAN;
            if (single_cabinet_side_row_active_) {
              if (single_cabinet_side_row_phase_ == SingleCabinetSideRowPhase::FIRST_ADJUSTED_SCAN ||
                single_cabinet_side_row_phase_ == SingleCabinetSideRowPhase::SECOND_ADJUSTED_SCAN)
              {
                scan_state = State::SINGLE_CABINET_ADJUSTED_SIDE_SCAN;
              } else if (single_cabinet_side_row_phase_ == SingleCabinetSideRowPhase::SECOND_PRIMARY_SCAN) {
                scan_state = State::SINGLE_CABINET_NEXT_GAP_SCAN;
              }
            }
            set_single_cabinet_state(
              scan_state,
              "已直行到目标深度格中心，开始缝内扫描");
          } else {
            update_safe_exit_gap_recovery_context(
              std::max(traveled, target_straight_distance_),
              true,
              "INVENTORYING");
            set_state(State::INVENTORYING, "已直行到目标深度格中心，盘库流程预留");
          }
          return;
        }

        entry_last_traveled_ = traveled;

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = final_linear_cmd;
        cmd.angular.z = limited_angular * safety.speed_scale;
        angular_z = cmd.angular.z;
        side_hold.final_angular_cmd = angular_z;
        RCLCPP_INFO_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "[mission_manager][MOVING_TO_GRID_CENTER][publish_cmd_vel] entering_gap_phase=%s "
          "entry_motion_mode=%s entry_linear_cmd=%.3f final_linear_cmd=%.3f "
          "published_cmd_linear_x=%.3f published_cmd_angular_z=%.3f "
          "target_gap_yaw=%.4f current_yaw=%.4f yaw_error=%.4f safety_speed_scale=%.2f "
          "safety_stop=%d safety_reason=%s elapsed=%.2f timeout=%.2f "
          "target_straight_distance=%.3f traveled=%.3f entry_straight_speed=%.3f "
          "estimated_speed=%.3f motion_direction=%s current_pose=(x=%.3f,y=%.3f) "
          "yaw_frame=%s pose_note=%s side_distance_cmd=%.3f side_hold_status=%s",
          entry_gap_phase_to_string(entry_gap_phase_).c_str(),
          entry_motion_mode_to_string(entry_motion_mode_).c_str(),
          entry_linear_cmd,
          final_linear_cmd,
          cmd.linear.x,
          cmd.angular.z,
          target_gap_yaw_,
          yaw_control.yaw,
          yaw_error,
          safety.speed_scale,
          safety.blocked ? 1 : 0,
          safety.block_reason.c_str(),
          straight_elapsed,
          straight_timeout,
          target_straight_distance_,
          traveled,
          entry_straight_speed_,
          estimated_speed,
          safety.motion_direction.c_str(),
          odom_current.x,
          odom_current.y,
          yaw_control.yaw_frame.c_str(),
          yaw_control.pose_note.c_str(),
          side_hold.side_distance_cmd,
          side_hold.status.c_str());
        cmd_pub_->publish(cmd);
        log_entering_gap_status(
          safety, yaw_log_pose, yaw_error, angular_z, traveled, true, false, side_hold,
          yaw_control.yaw_frame, yaw_control.pose_note, entry_linear_cmd, cmd.linear.x);
        break;
      }

      case EntryGapPhase::IDLE:
      default:
        begin_entering_gap_flow("ENTERING_GAP运行时补初始化");
        break;
    }
  }

  void handle_target_tracking_state()
  {
    if (!target_visible_ || !recognition_is_fresh() || !latest_recognition_) {
      target_visible_ = false;
      has_distance_ = false;
      set_corridor_mode(false, false);
      publish_stop();
      request_recognizer_enable(false);
      if (full_inventory_active_) {
        fail_full_inventory("跟踪阶段目标丢失，安全停车并终止全部盘库");
      } else if (single_cabinet_motion_active_) {
        fail_single_cabinet_motion("跟踪阶段目标丢失，安全停车并终止单柜盘库");
      } else {
        mission_active_ = false;
        set_state(State::ERROR, "跟踪阶段目标丢失，安全停车并终止任务");
      }
      return;
    }

    if (!has_distance_) {
      return;
    }

    const double distance_error = latest_distance_ - follow_distance_;
    const double heading_error = -static_cast<double>(latest_recognition_->horizontal_offset) /
      std::max(1.0, tracking_offset_scale_px_);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = std::clamp(
      tracking_kp_distance_ * distance_error,
      -tracking_speed_,
      tracking_speed_);
    cmd.angular.z = std::clamp(
      tracking_kp_heading_ * heading_error,
      -tracking_max_angular_,
      tracking_max_angular_);
    cmd_pub_->publish(cmd);

    if (std::abs(distance_error) <= distance_tolerance_) {
      if (tracking_stable_start_.nanoseconds() == 0) {
        tracking_stable_start_ = this->now();
      }
      if ((this->now() - tracking_stable_start_).seconds() >= distance_stable_time_sec_) {
        publish_stop();
        begin_search_gap_flow();
      }
    } else {
      tracking_stable_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    }
  }

  void handle_inventory_flow_state()
  {
    const InventoryGapPlan * plan = current_gap_request_plan();

    switch (state_) {
      case State::REQUEST_OPEN_GAP: {
        if (plan == nullptr) {
          set_flow_state(State::DONE, "没有待执行 gap");
          return;
        }

        if (plc_http_enabled_) {
          const int target_cabinet =
            single_cabinet_motion_active_ ? active_single_cabinet_scan_cabinet() :
            (!plan->scan_cabinets.empty() ? plan->scan_cabinets.front() : current_target_cabinet_);
          if (target_cabinet <= 0) {
            fail_inventory_flow("PLC open 目标柜号非法，无法发送 /open");
            return;
          }
          publish_inventory_flow_log(
            "PLC enabled: requesting open before movement target_cabinet=" +
            std::to_string(target_cabinet) +
            " gap=" + plan->gap_id +
            " cabinets=" + cabinet_unit_to_string(plan->scan_cabinets));
          if (single_cabinet_motion_active_) {
            publish_single_cabinet_log(
              "PLC open before single-cabinet movement target_cabinet=" +
              std::to_string(target_cabinet) +
              " gap=" + plan->gap_id);
          }
          (void)begin_plc_open_wait_for_target(
            target_cabinet,
            PlcOpenContinuation::SINGLE_CABINET_PREPARE_NAV,
            "REQUEST_OPEN_GAP -> SINGLE_CABINET_PREPARE_NAV");
          return;
        }

        publish_inventory_flow_log(
          "start gap=" + plan->gap_id +
          " cabinets=" + cabinet_unit_to_string(plan->scan_cabinets));
        publish_inventory_flow_log("requesting open gap=" + plan->gap_id);
        if (single_cabinet_motion_active_) {
          publish_single_cabinet_log("request open gap=" + plan->gap_id);
        }
        (void)web_api_client_.reportRobotStatus("REQUEST_OPEN_GAP");
        if (!web_api_client_.requestOpenGap(plan->gap_id)) {
          fail_inventory_flow("开柜请求失败: gap=" + plan->gap_id);
          return;
        }

        publish_inventory_flow_log(
          "waiting open ready " + format_seconds(open_gap_wait_sec_) + " sec");
        set_flow_state(
          State::WAIT_OPEN_READY,
          "等待开柜完成 " + format_seconds(open_gap_wait_sec_) + " sec");
        break;
      }

      case State::WAIT_OPEN_READY: {
        const double wait_sec = plc_http_enabled_ ? plc_open_wait_sec_ : open_gap_wait_sec_;
        publish_stop();
        if (!state_elapsed(wait_sec)) {
          break;
        }
        if (plc_http_enabled_) {
          handle_plc_open_wait_done();
          break;
        }
        if (single_cabinet_motion_active_) {
          const int target_cabinet = active_single_cabinet_scan_cabinet();
          set_single_cabinet_state(
            State::SINGLE_CABINET_PREPARE_NAV,
            "prepare nav target_cabinet=" + std::to_string(target_cabinet));
        }
        break;
      }

      case State::REQUEST_CLOSE_GAP: {
        if (plan == nullptr) {
          fail_inventory_flow("当前 gap 为空，无法请求关柜");
          return;
        }

        const std::string close_gap_id =
          single_cabinet_motion_active_ ? active_single_cabinet_gap_id() : plan->gap_id;
        publish_inventory_flow_log("requesting close gap=" + close_gap_id);
        if (single_cabinet_motion_active_) {
          publish_single_cabinet_log("request close gap=" + close_gap_id);
        }
        (void)web_api_client_.reportRobotStatus("REQUEST_CLOSE_GAP");
        if (!web_api_client_.requestCloseGap(close_gap_id)) {
          if (single_cabinet_motion_active_) {
            fail_single_cabinet_motion("关柜请求失败: gap=" + close_gap_id);
          } else {
            fail_inventory_flow("关柜请求失败: gap=" + close_gap_id);
          }
          return;
        }

        publish_inventory_flow_log(
          "waiting close done " + format_seconds(close_gap_wait_sec_) + " sec");
        set_flow_state(
          State::WAIT_CLOSE_DONE,
          "等待关柜完成 " + format_seconds(close_gap_wait_sec_) + " sec");
        break;
      }

      case State::WAIT_CLOSE_DONE: {
        if (!state_elapsed(close_gap_wait_sec_)) {
          break;
        }
        if (plan != nullptr) {
          const std::string finished_gap_id =
            single_cabinet_motion_active_ ? active_single_cabinet_gap_id() : plan->gap_id;
          publish_inventory_flow_log("finished gap=" + finished_gap_id);
        }

        if (current_gap_request_index_ + 1 < gap_request_queue_.size()) {
          ++current_gap_request_index_;
          set_flow_state(State::REQUEST_OPEN_GAP, "开始下一个 gap");
        } else {
          set_flow_state(State::DONE, "全部 gap 已完成");
        }
        break;
      }

      case State::SINGLE_CABINET_PREPARE_NAV: {
        const int target_cabinet = active_single_cabinet_scan_cabinet();
        publish_single_cabinet_log(
          "prepare nav target_cabinet=" + std::to_string(target_cabinet));
        if (single_cabinet_side_row_active_) {
          publish_single_cabinet_side_row_log(
            "first gap=" + single_cabinet_active_gap_id_ +
            " scan cabinet=" + std::to_string(target_cabinet));
        }

        std::string reason;
        if (!prepare_single_cabinet_target_cabinet(target_cabinet, reason)) {
          fail_single_cabinet_motion("准备单柜盘库目标失败: " + reason);
          return;
        }

        mission_active_ = true;
        cancel_requested_ = false;
        target_visible_ = false;
        has_distance_ = false;
        latest_distance_ = 0.0;
        return_mode_ = ReturnMode::NONE;
        nav2_return_in_progress_ = false;
        nav2_result_ready_ = false;
        reset_wait_gap_runtime();
        reset_entry_gap_runtime();
        publish_gap_context();
        tracking_stable_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        last_target_seen_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        mission_start_distance_ = odom_cumulative_distance_;
        reset_segment_distance();
        mission_start_pose_ = current_pose_2d();
        mission_start_pose_nav2_ = Pose2D{};

        publish_single_cabinet_log(
          "enter real nav to target cabinet=" + std::to_string(current_target_cabinet_));
        publish_single_cabinet_log(
          "requested gap=" + active_single_cabinet_gap_id() +
          " existing_gap_plan=" + gap_plan_to_string(current_gap_plan_));

        std::string nav_route_fail_reason;
        if (!begin_nav_route_for_current_target(
            "[mission_manager][single_cabinet] enter real nav to target cabinet=" +
            std::to_string(current_target_cabinet_),
            nav_route_fail_reason,
            State::SINGLE_CABINET_NAV_TO_TARGET))
        {
          fail_single_cabinet_motion(nav_route_fail_reason);
        }
        break;
      }

      case State::SINGLE_CABINET_IN_GAP_SCAN: {
        handle_single_cabinet_scan_state();
        break;
      }

      case State::SINGLE_CABINET_ADJUSTED_SIDE_SCAN: {
        handle_single_cabinet_scan_state();
        break;
      }

      case State::SINGLE_CABINET_NEXT_GAP_SCAN: {
        handle_single_cabinet_scan_state();
        break;
      }

      case State::SINGLE_CABINET_EXIT_GAP:
      case State::SINGLE_CABINET_FINAL_EXIT_GAP: {
        handle_single_cabinet_exit_gap_state();
        break;
      }

      case State::SINGLE_CABINET_REENTER_FOR_ADJUSTED_SCAN: {
        handle_single_cabinet_reenter_adjusted_scan_state();
        break;
      }

      case State::SINGLE_CABINET_CORRIDOR_TRANSFER: {
        handle_single_cabinet_corridor_transfer_state();
        break;
      }

      case State::SINGLE_CABINET_PREPARE_NEXT_GAP: {
        handle_single_cabinet_prepare_next_gap_state();
        break;
      }

      case State::SINGLE_CABINET_REENTER_NEXT_GAP: {
        handle_single_cabinet_reenter_next_gap_state();
        break;
      }

      case State::SINGLE_CABINET_STOP_AFTER_SCAN: {
        if (pending_interrupt_request_ != PendingInterruptRequest::NONE) {
          begin_final_exit_for_pending_after_stop();
          break;
        }
        stop_single_cabinet_motion_controls();
        mission_active_ = false;
        publish_single_cabinet_log("scan finished, stop after scan, no exit motion in current step");
        if (!single_cabinet_close_requested_) {
          publish_single_cabinet_log("request close gap=" + single_cabinet_motion_target_gap_);
          (void)web_api_client_.requestCloseGap(single_cabinet_motion_target_gap_);
          single_cabinet_close_requested_ = true;
        }
        publish_single_cabinet_log("done, back to IDLE");
        set_flow_state(State::DONE, "单柜盘库扫描完成");
        break;
      }

      case State::DONE: {
        if (!flush_inventory_upload_batch_for_mission_complete("all_cabinets_complete")) {
          mission_error_reason_ = "RFID final batch upload failed at mission complete";
          set_state(State::ERROR, mission_error_reason_);
          break;
        }
        (void)web_api_client_.reportRobotStatus("DONE");
        if (full_inventory_active_) {
          publish_full_inventory_log("done, back to IDLE");
        } else if (single_cabinet_motion_active_) {
          publish_single_cabinet_log("done, back to IDLE");
        } else {
          publish_inventory_flow_log("all requested gaps finished");
        }
        mission_active_ = false;
        single_cabinet_motion_active_ = false;
        single_cabinet_gap_searching_ = false;
        single_cabinet_target_recognized_logged_ = false;
        single_cabinet_close_requested_ = false;
        single_cabinet_final_recognition_wait_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        if (single_cabinet_side_row_active_) {
          publish_single_cabinet_log("[return] side-row inventory completed");
        }
        reset_single_cabinet_side_row_context();
        reset_full_inventory_context();
        inventory_flow_active_ = false;
        gap_request_queue_.clear();
        current_gap_request_index_ = 0;
        set_state(State::IDLE, "盘库任务完成，系统待机");
        break;
      }

      case State::ERROR: {
        (void)web_api_client_.reportRobotStatus("ERROR");
        if (full_inventory_active_) {
          stop_single_cabinet_motion_controls();
          publish_full_inventory_log("full inventory failed: " + mission_error_reason_);
        } else if (single_cabinet_motion_active_) {
          stop_single_cabinet_motion_controls();
          publish_single_cabinet_log("single cabinet inventory failed: " + mission_error_reason_);
        } else {
          publish_inventory_flow_log("inventory flow failed: " + mission_error_reason_);
        }
        mission_active_ = false;
        single_cabinet_motion_active_ = false;
        single_cabinet_gap_searching_ = false;
        single_cabinet_target_recognized_logged_ = false;
        single_cabinet_close_requested_ = false;
        single_cabinet_final_recognition_wait_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        reset_single_cabinet_side_row_context();
        reset_full_inventory_context();
        inventory_flow_active_ = false;
        gap_request_queue_.clear();
        current_gap_request_index_ = 0;
        set_state(State::IDLE, "盘库流程已停止，系统待机");
        break;
      }

      default:
        break;
    }
  }

  void on_timer()
  {
    if (state_ == State::SAFE_EXIT_GAP) {
      handle_safe_exit_gap_state();
      return;
    }

    if (state_ == State::STOP_AUTO_CHARGE_AND_DEPART) {
      handle_stop_auto_charge_and_depart_state();
      return;
    }

    if (state_ == State::AUTO_RECHARGING) {
      return;
    }

    if (!mission_active_ && !inventory_flow_active_) {
      return;
    }

    if (inventory_flow_active_ && (state_ == State::DONE || state_ == State::ERROR)) {
      handle_inventory_flow_state();
      return;
    }

    switch (state_) {
      case State::REQUEST_OPEN_GAP:
      case State::WAIT_OPEN_READY:
      case State::REQUEST_CLOSE_GAP:
      case State::WAIT_CLOSE_DONE:
      case State::SINGLE_CABINET_PREPARE_NAV:
      case State::SINGLE_CABINET_IN_GAP_SCAN:
      case State::SINGLE_CABINET_STOP_AFTER_SCAN:
      case State::SINGLE_CABINET_EXIT_GAP:
      case State::SINGLE_CABINET_REENTER_FOR_ADJUSTED_SCAN:
      case State::SINGLE_CABINET_ADJUSTED_SIDE_SCAN:
      case State::SINGLE_CABINET_CORRIDOR_TRANSFER:
      case State::SINGLE_CABINET_PREPARE_NEXT_GAP:
      case State::SINGLE_CABINET_REENTER_NEXT_GAP:
      case State::SINGLE_CABINET_NEXT_GAP_SCAN:
      case State::SINGLE_CABINET_FINAL_EXIT_GAP: {
        handle_inventory_flow_state();
        break;
      }

      case State::FULL_INVENTORY_PREPARE_TARGET: {
        publish_stop();
        break;
      }

      case State::FULL_INVENTORY_NAV_TO_OBSERVE: {
        handle_nav_route_state();
        break;
      }

      case State::FULL_INVENTORY_POST_ROUTE_RECOGNITION_WAIT: {
        handle_full_inventory_post_route_recognition_wait_state();
        break;
      }

      case State::FULL_INVENTORY_RECOGNITION_FALLBACK: {
        handle_full_inventory_recognition_fallback_state();
        break;
      }

      case State::FULL_INVENTORY_TARGET_DISTANCE_ALIGN: {
        handle_full_inventory_target_distance_align_state();
        break;
      }

      case State::FULL_INVENTORY_SEARCH_GAP: {
        handle_search_gap_state();
        break;
      }

      case State::FULL_INVENTORY_POST_GAP_DETECT_ADVANCE: {
        handle_full_inventory_post_gap_detect_advance_state();
        break;
      }

      case State::FULL_INVENTORY_REAR_TARGET_REORIENT: {
        handle_full_inventory_rear_target_reorient_state();
        break;
      }

      case State::FULL_INVENTORY_REAR_TARGET_BACKUP: {
        handle_full_inventory_rear_target_backup_state();
        break;
      }

      case State::FULL_INVENTORY_SAME_SIDE_NEXT_SEARCH: {
        handle_full_inventory_same_side_next_search_state();
        break;
      }

      case State::FULL_INVENTORY_ENTERING_GAP: {
        handle_entering_gap_state();
        break;
      }

      case State::FULL_INVENTORY_IN_GAP_SCAN: {
        handle_full_inventory_scan_state();
        break;
      }

      case State::FULL_INVENTORY_EXIT_GAP: {
        handle_single_cabinet_exit_gap_state();
        break;
      }

      case State::FULL_INVENTORY_ADVANCE_NEXT_TARGET:
      case State::FULL_INVENTORY_COMPLETE: {
        publish_stop();
        break;
      }

      case State::FULL_INVENTORY_AUTO_CHARGE_BETWEEN_SIDES: {
        handle_full_inventory_auto_charge_between_sides_state();
        break;
      }

      case State::SINGLE_CABINET_NAV_TO_TARGET: {
        handle_nav_route_state();
        break;
      }

      case State::SINGLE_CABINET_TARGET_TRACKING: {
        handle_target_tracking_state();
        break;
      }

      case State::SINGLE_CABINET_FINAL_RECOGNITION_WAIT: {
        publish_stop();
        request_recognizer_enable(true);
        if (single_cabinet_final_recognition_wait_start_.nanoseconds() == 0) {
          single_cabinet_final_recognition_wait_start_ = this->now();
        }
        if ((this->now() - single_cabinet_final_recognition_wait_start_).seconds() >=
          std::max(0.0, single_cabinet_final_recognition_wait_sec_))
        {
          fail_single_cabinet_motion(
            "final recognition timeout, target cabinet=" +
            std::to_string(current_target_cabinet_) + " not recognized");
        }
        break;
      }

      case State::SINGLE_CABINET_WAITING_GAP: {
        if (single_cabinet_gap_searching_) {
          handle_search_gap_state();
        } else {
          handle_waiting_gap_state();
        }
        break;
      }

      case State::SINGLE_CABINET_ENTERING_GAP: {
        handle_entering_gap_state();
        break;
      }

      case State::NAV_ROUTE: {
        handle_nav_route_state();
        break;
      }

      case State::TARGET_TRACKING: {
        handle_target_tracking_state();
        break;
      }

      case State::SEARCH_GAP: {
        handle_search_gap_state();
        break;
      }

      case State::WAITING_GAP: {
        handle_waiting_gap_state();
        break;
      }

      case State::ENTERING_GAP: {
        handle_entering_gap_state();
        break;
      }

      case State::INVENTORYING: {
        // 当前版本策略：入缝后不自动推进任务，原地停车等待下一时刻外部指令。
        publish_stop();
        RCLCPP_INFO_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "已进入缝隙，INVENTORYING 状态下原地等待下一时刻指令");
        break;
      }

      case State::RETURNING:
      case State::RETURNING_HOME: {
        handle_returning_state();
        break;
      }

      case State::IDLE:
      case State::DONE:
      case State::ERROR:
        if (full_inventory_active_) {
          fail_full_inventory("正式流程进入 ERROR，停止全部盘库");
        } else if (single_cabinet_motion_active_) {
          fail_single_cabinet_motion("正式流程进入 ERROR，停止单柜盘库");
        }
        break;
      default:
        break;
    }
  }

  std::string recognized_topic_;
  std::string distance_topic_;
  std::string gap_topic_;
  std::string odom_topic_;
  std::string odom_fallback_topic_;
  std::string scan_topic_;

  std::string mission_state_topic_;
  std::string mission_log_topic_;
  std::string cmd_vel_topic_;
  std::string motion_model_{"diff_drive"};
  std::string corridor_enable_topic_;
  std::string corridor_reverse_topic_;
  std::string gap_context_topic_;
  std::string recognizer_enable_topic_;
  std::string gap_detector_enable_topic_;
  std::string entry_side_topic_;
  std::string target_lidar_side_topic_;
  std::string current_target_cabinet_topic_;
  std::string distance_estimator_enable_topic_;

  std::string start_service_name_;
  std::string cancel_service_name_;
  std::string return_home_service_name_;
  std::string return_to_charge_service_name_;
  std::string safe_exit_gap_service_name_;
  std::string stop_auto_charge_and_depart_service_name_;
  std::string inventory_auto_recharge_start_service_name_;
  std::string inventory_auto_recharge_cancel_service_name_;
  std::string cancel_auto_recharge_service_name_;
  std::string auto_recharge_status_topic_;
  std::string auto_recharge_charging_flag_topic_;
  std::string auto_recharge_recharge_flag_topic_;
  std::string recognizer_trigger_service_;

  std::vector<std::string> target_list_param_;
  std::string route_waypoints_file_{"config/route_waypoints.yaml"};
  std::string warehouse_layout_file_{"config/warehouse_layout.yaml"};
  std::string gap_scan_map_file_{"config/gap_scan_map.yaml"};
  std::string route_search_failure_policy_{"error"};
  std::map<std::string, RouteConfig> route_configs_;
  std::map<std::string, std::string> side_route_map_;
  std::map<int, std::string> cabinet_side_map_;
  std::map<int, std::string> cabinet_entry_side_map_;
  std::map<int, SearchDirection> cabinet_gap_search_direction_map_;
  std::map<std::string, WarehouseRowLayout> warehouse_rows_by_side_;
  std::vector<InventoryGapPlan> configured_gap_inventory_plan_{
    InventoryGapPlan{"gap_03_02", std::vector<int>{3, 2, 1}},
    InventoryGapPlan{"gap_04_05", std::vector<int>{4, 5, 6}}};
  std::map<std::string, std::vector<int>> inventory_plan_by_gap_id_{
    {"gap_03_02", std::vector<int>{3, 2, 1}},
    {"gap_04_05", std::vector<int>{4, 5, 6}}};
  std::map<std::string, std::vector<int>> gap_scan_map_cabinets_by_gap_id_{
    {"gap_03_02", std::vector<int>{3, 2, 1}},
    {"gap_04_05", std::vector<int>{4, 5, 6}}};
  RouteConfig current_route_;
  TargetGapPlan current_gap_plan_;
  std::string current_route_name_;
  std::string current_target_side_{"left"};
  std::string current_entry_side_{"left"};
  bool routes_loaded_{false};
  bool warehouse_layout_loaded_{false};
  bool inventory_runtime_config_loaded_{false};
  std::vector<std::string> targets_;
  std::size_t current_target_index_{0};
  int current_target_cabinet_{-1};
  int current_target_level_index_{1};
  int current_target_depth_index_{1};
  bool current_target_depth_defaulted_{false};

  double follow_distance_{0.5};
  double warehouse_length_{12.0};
  double corridor_speed_{0.20};
  double tracking_speed_{0.15};
  double entry_distance_{0.70};
  double turn_speed_{0.40};  // 兼容保留
  bool enable_grid_center_entry_{true};
  double grid_depth_m_{2.4};
  int left_max_depth_index_{4};
  int right_max_depth_index_{3};
  double entry_center_offset_m_{0.0};
  double target_depth_center_m_{1.2};
  double target_straight_distance_{1.2};
  bool current_entry_profile_valid_{false};
  double entry_right_target_yaw_rad_{1.5708};
  double entry_left_target_yaw_rad_{-1.5708};
  double entry_align_yaw_tolerance_rad_{0.08};
  int entry_turn_yaw_stable_required_count_{3};
  double entry_turn_angular_speed_{0.30};
  double entry_turn_timeout_sec_{12.0};
  double entry_straight_speed_{0.08};
  double entry_straight_timeout_sec_{60.0};
  double entry_straight_yaw_kp_{0.8};
  double entry_straight_yaw_deadband_rad_{0.03};
  double entry_straight_max_angular_speed_{0.10};
  double entry_side_hold_target_distance_m_{0.60};
  double entry_side_hold_kp_{0.35};
  int entry_side_hold_min_valid_points_{5};
  double max_dynamic_entry_distance_m_{10.0};
  double enter_stop_distance_{0.30};
  double enter_slow_distance_{0.55};
  double enter_front_sector_start_deg_{-20.0};
  double enter_front_sector_end_deg_{20.0};
  double enter_front_left_sector_start_deg_{20.0};
  double enter_front_left_sector_end_deg_{75.0};
  double enter_left_side_sector_start_deg_{75.0};
  double enter_left_side_sector_end_deg_{120.0};
  double enter_front_right_sector_start_deg_{-75.0};
  double enter_front_right_sector_end_deg_{-20.0};
  double enter_right_side_sector_start_deg_{-120.0};
  double enter_right_side_sector_end_deg_{-75.0};

  double tracking_kp_distance_{0.8};
  double tracking_kp_heading_{1.2};
  double tracking_offset_scale_px_{320.0};
  double tracking_max_angular_{0.8};

  double distance_tolerance_{0.08};
  double target_distance_aligned_min_m_{0.25};
  double target_distance_aligned_max_m_{0.75};
  double target_distance_gap_threshold_m_{1.50};
  int target_distance_gap_confirm_count_{1};
  int target_distance_gap_open_count_{0};
  double distance_stable_time_sec_{1.2};
  double recognition_timeout_sec_{1.0};
  int target_recognition_stable_frames_{3};
  double target_recognition_stable_time_sec_{0.30};
  double odom_primary_timeout_sec_{0.8};
  double post_track_retreat_distance_{0.10};
  double retreat_speed_{0.08};
  double wait_gap_stop_settle_sec_{0.25};
  double search_gap_speed_{0.06};
  double search_gap_forward_timeout_sec_{4.0};
  double search_gap_backward_timeout_sec_{6.0};
  double gap_detect_cycle_timeout_sec_{2.5};
  int gap_failures_before_adjust_{3};
  double gap_adjust_speed_{0.08};
  std::vector<double> gap_adjust_sequence_{-0.10, 0.10, -0.20};

  std::string finish_return_mode_{"auto_charge"};
  double between_side_auto_charge_wait_sec_{10.0};
  double between_side_auto_charge_runtime_sec_{120.0};
  std::string home_pose_frame_id_{"map"};
  double home_pose_x_{0.0};
  double home_pose_y_{0.0};
  double home_pose_yaw_{0.0};
  std::string nav2_action_name_{"navigate_to_pose"};
  std::string nav2_goal_frame_{"map"};
  double auto_recharge_service_timeout_sec_{3.0};
  double auto_recharge_cancel_timeout_sec_{3.0};
  double auto_recharge_cancel_response_timeout_sec_{15.0};
  bool safe_exit_gap_enabled_{true};
  double safe_exit_gap_speed_mps_{0.05};
  double safe_exit_gap_yaw_kp_{0.6};
  double safe_exit_gap_max_angular_z_{0.15};
  double safe_exit_gap_extra_clearance_m_{0.30};
  double safe_exit_gap_timeout_sec_{30.0};
  double stop_auto_charge_depart_distance_m_{0.5};
  double stop_auto_charge_depart_speed_mps_{0.05};
  double stop_auto_charge_depart_yaw_kp_{0.6};
  double stop_auto_charge_depart_max_angular_z_{0.15};
  double stop_auto_charge_depart_timeout_sec_{20.0};
  double stop_auto_charge_depart_stop_before_sec_{0.5};
  double stop_auto_charge_depart_stop_after_sec_{0.5};
  double nav2_server_wait_timeout_sec_{1.0};
  bool nav2_startup_wait_enabled_{true};
  double nav2_startup_wait_timeout_sec_{60.0};
  double nav2_startup_wait_poll_sec_{0.2};
  double nav2_goal_timeout_sec_{120.0};
  double nav2_route_waypoint_timeout_sec_{60.0};
  double nav2_cancel_stop_duration_sec_{0.50};
  bool continue_on_error_{false};
  double control_rate_hz_{10.0};

  bool use_scan_safety_{true};
  double max_scan_age_sec_{0.8};

  bool use_ultrasonic_safety_{true};
  std::vector<std::string> ultrasonic_topics_;
  double entry_ultrasonic_stop_distance_{0.25};
  double max_ultrasonic_age_sec_{0.8};
  std::string web_client_mode_{"local"};
  double open_gap_wait_sec_{5.0};
  double close_gap_wait_sec_{3.0};
  bool scanner_enabled_{true};
  double scan_duration_sec_{2.0};
  double scan_timeout_sec_{5.0};
  int scan_retry_count_{0};
  double scan_result_timeout_sec_{2.0};
  bool lift_enabled_{true};
  double lift_up_duration_sec_{2.0};
  double lift_down_duration_sec_{2.0};
  double lift_service_timeout_sec_{5.0};
  std::string lift_up_service_name_{"/lift/up"};
  std::string lift_down_service_name_{"/lift/down"};
  std::string lift_stop_service_name_{"/lift/stop"};
  std::string lift_all_off_service_name_{"/lift/all_off"};
  std::string lift_home_service_name_{"/lift/home"};
  double grid_motion_duration_sec_{1.0};
  double grid_motion_timeout_sec_{10.0};
  bool single_cabinet_motion_enabled_{false};
  int single_cabinet_motion_target_cabinet_{3};
  std::string single_cabinet_motion_target_gap_{"gap_03_02"};
  bool single_cabinet_motion_stop_after_scan_{true};
  double single_cabinet_final_recognition_wait_sec_{8.0};
  bool single_cabinet_side_row_enabled_{false};
  std::string single_cabinet_side_row_name_{"row_01_02_03_04"};
  std::string single_cabinet_side_row_first_gap_{"gap_02_03_04"};
  std::vector<int> single_cabinet_side_row_first_gap_scan_sequence_{4, 3};
  std::string single_cabinet_side_row_second_gap_{"gap_01_02_03"};
  std::vector<int> single_cabinet_side_row_second_gap_scan_sequence_{2, 1};
  bool single_cabinet_side_row_corridor_transfer_enabled_{true};
  int single_cabinet_side_row_corridor_transfer_target_cabinet_{2};
  std::string single_cabinet_side_row_corridor_transfer_direction_{"toward_cabinet_1"};
  bool single_cabinet_exit_after_each_scan_{true};
  std::string single_cabinet_exit_mode_{"reverse"};
  double single_cabinet_exit_speed_{0.05};
  double single_cabinet_exit_extra_distance_m_{0.10};
  double single_cabinet_exit_timeout_sec_{40.0};
  double single_cabinet_exit_distance_m_{1.20};
  bool single_cabinet_exit_turn_enabled_{true};
  double single_cabinet_exit_turn_angular_speed_{0.25};
  double single_cabinet_exit_turn_yaw_tolerance_rad_{0.08};
  double single_cabinet_exit_turn_timeout_sec_{8.0};
  bool single_cabinet_reentry_for_position_adjustment_{true};
  bool single_cabinet_grid_motion_enabled_{false};
  double single_cabinet_grid_spacing_m_{0.30};
  double single_cabinet_grid_move_speed_{0.04};
  double single_cabinet_grid_move_timeout_sec_{10.0};
  bool single_cabinet_grid_move_return_between_layers_{false};
  bool single_cabinet_close_gap_after_final_exit_{true};
  bool full_inventory_enabled_{true};
  std::vector<int> full_inventory_sequence_{4, 3, 8, 7};
  std::string full_inventory_left_route_{"left_route"};
  std::string full_inventory_right_route_{"right_route"};
  bool recognize_in_idle_{true};
  bool full_inventory_recognize_during_nav_{false};
  bool full_inventory_same_side_next_search_enabled_{true};
  double full_inventory_same_side_search_speed_{0.04};
  double full_inventory_same_side_search_timeout_sec_{20.0};
  bool full_inventory_same_side_pose_hold_enabled_{true};
  double full_inventory_same_side_left_fixed_y_m_{0.575};
  double full_inventory_same_side_left_fixed_yaw_rad_{-3.1400};
  double full_inventory_same_side_right_fixed_y_m_{-0.625};
  double full_inventory_same_side_right_fixed_yaw_rad_{-3.1400};
  double full_inventory_same_side_yaw_kp_{0.40};
  double full_inventory_same_side_yaw_deadband_rad_{0.03};
  double full_inventory_same_side_y_kp_{0.30};
  double full_inventory_same_side_y_deadband_m_{0.03};
  double full_inventory_same_side_y_correction_sign_{1.0};
  double full_inventory_same_side_max_angular_{0.15};
  bool full_inventory_same_side_recognition_delay_enabled_{true};
  double full_inventory_same_side_recognition_delay_distance_m_{1.0};
  bool rear_target_handling_enabled_{true};
  std::string rear_target_handle_mode_{"hold_entry_yaw_backup"};
  double rear_target_turn_yaw_tolerance_rad_{0.08};
  double rear_target_turn_timeout_sec_{10.0};
  bool rear_target_backup_enabled_{true};
  double rear_target_backup_distance_m_{1.50};
  double rear_target_backup_speed_{0.08};
  double rear_target_backup_timeout_sec_{30.0};
  double full_inventory_same_side_active_fixed_y_m_{0.575};
  double full_inventory_same_side_active_fixed_yaw_rad_{-3.1400};
  std::string full_inventory_same_side_active_map_side_{"left"};
  bool full_inventory_same_side_heading_override_valid_{false};
  double full_inventory_same_side_heading_override_yaw_rad_{0.0};
  double full_inventory_final_recognition_wait_sec_{5.0};
  bool full_inventory_recognition_fallback_enabled_{true};
  double full_inventory_recognition_fallback_speed_{0.04};
  double full_inventory_recognition_fallback_wait_sec_{2.0};
  double full_inventory_recognition_fallback_timeout_sec_{20.0};
  std::vector<double> full_inventory_recognition_fallback_sequence_{-0.30, 0.60, -0.30};
  bool post_gap_detect_advance_enabled_{true};
  double post_gap_detect_advance_distance_m_{0.25};
  double post_gap_detect_advance_speed_{0.04};
  double post_gap_detect_advance_timeout_sec_{8.0};
  int scan_layers_{2};
  int scan_depth_count_{3};
  std::string web_base_url_;
  std::string web_open_gap_endpoint_{"/api/gap/open"};
  std::string web_close_gap_endpoint_{"/api/gap/close"};
  std::string web_status_endpoint_{"/api/robot/status"};
  std::string web_result_endpoint_{"/api/inventory/result"};
  bool plc_http_enabled_{false};
  std::string plc_server_url_{"https://58.154.205.27:8099"};
  std::string plc_open_endpoint_{"/http-control-plc/car_open"};
  std::string plc_open_query_param_{"shelfId"};
  std::string plc_close_endpoint_{"/close"};
  std::string plc_stop_endpoint_{"/stop"};
  std::string plc_hello_endpoint_{"/hello"};
  bool plc_verify_tls_{false};
  bool plc_require_body_success_{false};
  double plc_request_timeout_sec_{3.0};
  int plc_retry_count_{1};
  std::string plc_fail_policy_{"error"};
  std::vector<int> plc_supported_cabinets_{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18};
  double plc_open_wait_sec_{5.0};
  bool plc_call_close_on_mission_done_{false};
  bool plc_call_stop_on_error_{false};
  bool rfid_upload_enabled_{true};
  std::string rfid_upload_url_{"https://58.154.205.27:8099/RobotInspection/inventoryAudit"};
  bool rfid_upload_verify_tls_{false};
  double rfid_upload_timeout_sec_{3.0};
  int rfid_upload_retry_count_{2};
  std::string rfid_upload_fail_policy_{"error"};
  bool rfid_local_log_enabled_{true};
  std::string rfid_local_log_path_{
    "/home/wheeltec/wheeltec_ros2/rfid_scan_logs/rfid_scan_records.jsonl"};
  bool rfid_local_log_write_batch_summary_{true};
  std::string rfid_upload_status_path_{"/tmp/agv_inventory_system/rfid_upload_status.json"};
  bool rfid_upload_require_success_{false};
  bool rfid_reader_enabled_{true};
  std::string rfid_reader_mode_{"active_report_serial"};
  std::string rfid_serial_device_{"/dev/ttyUSB0"};
  int rfid_serial_baud_{9600};
  double rfid_scan_duration_sec_{5.0};
  int rfid_frame_min_length_{8};
  int rfid_frame_max_length_{64};
  PlcOpenContinuation plc_open_wait_continuation_{PlcOpenContinuation::NONE};
  int plc_open_wait_target_cabinet_{-1};
  bool plc_open_wait_required_{false};
  std::string plc_open_wait_context_;
  bool inventory_flow_active_{false};
  std::vector<InventoryGapPlan> gap_request_queue_;
  std::size_t current_gap_request_index_{0};
  std::string mission_error_reason_;
  rclcpp::Time state_enter_time_{0, 0, RCL_ROS_TIME};
  bool single_cabinet_motion_active_{false};
  bool single_cabinet_gap_searching_{false};
  bool single_cabinet_target_recognized_logged_{false};
  bool single_cabinet_close_requested_{false};
  rclcpp::Time single_cabinet_final_recognition_wait_start_{0, 0, RCL_ROS_TIME};
  bool single_cabinet_side_row_active_{false};
  bool single_cabinet_side_row_full_sequence_{false};
  std::vector<int> single_cabinet_side_row_requested_sequence_;
  SingleCabinetSideRowPhase single_cabinet_side_row_phase_{SingleCabinetSideRowPhase::NONE};
  SingleCabinetAfterExitAction single_cabinet_after_exit_action_{SingleCabinetAfterExitAction::NONE};
  std::string single_cabinet_active_gap_id_;
  int single_cabinet_current_scan_cabinet_{-1};
  int single_cabinet_adjusted_scan_cabinet_{-1};
  int single_cabinet_next_gap_target_cabinet_{-1};
  double single_cabinet_last_entering_straight_distance_{0.0};
  double single_cabinet_exit_target_distance_{1.20};
  double single_cabinet_exit_effective_timeout_sec_{40.0};
  rclcpp::Time single_cabinet_exit_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time single_cabinet_exit_phase_start_time_{0, 0, RCL_ROS_TIME};
  SingleCabinetExitPhase single_cabinet_exit_phase_{SingleCabinetExitPhase::STRAIGHT_REVERSE};
  std::vector<agv_inventory_system::ScanStep> single_cabinet_scan_steps_;
  std::size_t single_cabinet_scan_step_index_{0};
  std::vector<agv_inventory_system::InventoryUploadItem> inventory_upload_batch_;
  std::set<std::string> inventory_upload_batch_locations_;
  bool inventory_upload_batch_finalized_{false};
  agv_inventory_system::RfidScanLogWriter rfid_scan_log_writer_;
  int single_cabinet_scan_cabinet_{-1};
  bool single_cabinet_scan_active_{false};
  rclcpp::Time single_cabinet_scan_step_start_time_{0, 0, RCL_ROS_TIME};
  bool single_cabinet_grid_have_previous_depth_{false};
  int single_cabinet_grid_previous_cabinet_{-1};
  int single_cabinet_grid_previous_layer_{-1};
  int single_cabinet_grid_previous_depth_{-1};
  bool single_cabinet_grid_move_active_{false};
  agv_inventory_system::ScanStep single_cabinet_grid_move_step_;
  Pose2D single_cabinet_grid_move_start_pose_;
  rclcpp::Time single_cabinet_grid_move_start_time_{0, 0, RCL_ROS_TIME};
  double single_cabinet_grid_move_target_distance_{0.0};
  double single_cabinet_grid_move_cmd_speed_{0.0};
  bool lift_step_active_{false};
  rclcpp::Time lift_step_start_time_{0, 0, RCL_ROS_TIME};
  std::shared_future<agv_inventory_system::srv::LiftMoveTimed::Response::SharedPtr>
  lift_step_future_;
  std::string lift_step_service_name_;
  std::string lift_step_description_;
  bool full_inventory_active_{false};
  std::size_t full_inventory_index_{0};
  int full_inventory_current_target_{-1};
  int full_inventory_next_target_{-1};
  std::string full_inventory_current_side_;
  std::string full_inventory_current_route_;
  bool between_side_auto_charge_active_{false};
  bool between_side_auto_charge_cancel_sent_{false};
  bool between_side_auto_charge_cancel_response_ready_{false};
  bool between_side_auto_charge_cancel_response_success_{false};
  std::string between_side_auto_charge_cancel_response_message_;
  bool between_side_auto_charge_fallback_used_{false};
  std::size_t between_side_auto_charge_target_index_{0};
  int between_side_auto_charge_target_{-1};
  rclcpp::Time between_side_auto_charge_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time between_side_auto_charge_ready_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time between_side_auto_charge_cancel_request_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time full_inventory_same_side_search_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time full_inventory_final_recognition_wait_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time full_inventory_recognition_fallback_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time full_inventory_recognition_fallback_phase_start_{0, 0, RCL_ROS_TIME};
  FullInventoryRecognitionFallbackPhase full_inventory_recognition_fallback_phase_{
    FullInventoryRecognitionFallbackPhase::IDLE};
  std::size_t full_inventory_recognition_fallback_index_{0};
  rclcpp::Time full_inventory_post_gap_advance_start_{0, 0, RCL_ROS_TIME};
  bool full_inventory_rear_target_pending_{false};
  bool full_inventory_rear_target_active_{false};
  int full_inventory_rear_target_finished_cabinet_{-1};
  int full_inventory_rear_target_next_cabinet_{-1};
  double full_inventory_rear_target_yaw_{0.0};
  SearchDirection full_inventory_rear_target_original_gap_direction_{SearchDirection::FORWARD};
  bool full_inventory_last_exit_entry_yaw_valid_{false};
  double full_inventory_last_exit_entry_yaw_{0.0};
  bool full_inventory_gap_search_direction_override_valid_{false};
  SearchDirection full_inventory_gap_search_direction_override_{SearchDirection::FORWARD};
  rclcpp::Time full_inventory_rear_target_turn_start_{0, 0, RCL_ROS_TIME};
  bool full_inventory_rear_target_backup_started_{false};
  Pose2D full_inventory_rear_target_backup_start_pose_;
  double full_inventory_rear_target_backup_fixed_y_{0.0};
  rclcpp::Time full_inventory_rear_target_backup_start_time_{0, 0, RCL_ROS_TIME};
  agv_inventory_system::ScanSequenceGenerator scan_sequence_generator_;
  agv_inventory_system::InventoryScanner inventory_scanner_;
  agv_inventory_system::WebApiClient web_api_client_;

  State state_{State::IDLE};
  ReturnMode return_mode_{ReturnMode::NONE};
  PendingInterruptRequest pending_interrupt_request_{PendingInterruptRequest::NONE};

  bool mission_active_{false};
  bool cancel_requested_{false};
  bool mission_force_map_origin_on_finish_{false};
  std::string latest_auto_recharge_status_;
  rclcpp::Time latest_auto_recharge_status_time_{0, 0, RCL_ROS_TIME};
  bool latest_auto_recharge_charging_{false};
  rclcpp::Time latest_auto_recharge_charging_time_{0, 0, RCL_ROS_TIME};
  int latest_auto_recharge_recharge_flag_{0};
  rclcpp::Time latest_auto_recharge_recharge_flag_time_{0, 0, RCL_ROS_TIME};
  bool robot_inside_gap_{false};
  bool safe_exit_gap_context_valid_{false};
  bool safe_exit_gap_maybe_inside_gap_{false};
  double safe_exit_gap_distance_m_{0.0};
  double safe_exit_gap_yaw_rad_{0.0};
  bool safe_exit_gap_yaw_valid_{false};
  std::string safe_exit_gap_context_source_state_;
  rclcpp::Time safe_exit_gap_context_stamp_{0, 0, RCL_ROS_TIME};
  double safe_exit_gap_start_distance_{0.0};
  double safe_exit_gap_target_distance_{0.0};
  double safe_exit_gap_target_yaw_{0.0};
  rclcpp::Time safe_exit_gap_start_time_{0, 0, RCL_ROS_TIME};
  StopAutoChargeDepartPhase stop_auto_charge_depart_phase_{StopAutoChargeDepartPhase::IDLE};
  StopAutoChargeDepartContinuation stop_auto_charge_depart_continuation_{
    StopAutoChargeDepartContinuation::IDLE};
  double stop_auto_charge_depart_start_distance_{0.0};
  double stop_auto_charge_depart_target_yaw_{0.0};
  rclcpp::Time stop_auto_charge_depart_phase_start_time_{0, 0, RCL_ROS_TIME};
  bool stop_auto_charge_cancel_response_ready_{false};
  bool stop_auto_charge_cancel_response_success_{false};
  std::string stop_auto_charge_cancel_response_message_;
  bool recognizer_enabled_{false};
  bool recognizer_enabled_cmd_{false};
  bool recognizer_enable_initialized_{false};
  bool gap_detector_enabled_cmd_{false};
  bool gap_detector_enable_initialized_{false};
  bool distance_estimator_enabled_cmd_{false};
  bool distance_estimator_enable_initialized_{false};
  bool nav2_return_in_progress_{false};
  bool nav2_result_ready_{false};
  bool nav2_result_success_{false};
  std::string nav2_result_text_;
  rclcpp::Time nav2_goal_sent_time_{0, 0, RCL_ROS_TIME};
  std::size_t current_route_waypoint_index_{0};
  std::uint64_t nav2_route_goal_sequence_{0};
  bool nav2_route_goal_in_progress_{false};
  bool nav2_route_result_ready_{false};
  bool nav2_route_result_success_{false};
  bool nav2_route_cancel_requested_{false};
  bool nav2_route_stop_hold_active_{false};
  bool target_found_pending_{false};
  std::string nav2_route_result_text_;
  rclcpp::Time nav2_route_goal_sent_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time nav2_route_stop_hold_start_{0, 0, RCL_ROS_TIME};

  bool target_visible_{false};
  bool has_distance_{false};
  double latest_distance_{0.0};
  int target_recognition_candidate_id_{-1};
  int target_recognition_stable_count_{0};
  rclcpp::Time target_recognition_first_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time target_recognition_last_time_{0, 0, RCL_ROS_TIME};

  agv_inventory_system::msg::GapStatus latest_gap_;
  agv_inventory_system::msg::RecognizedNumber::SharedPtr latest_recognition_;
  rclcpp::Time latest_recognition_time_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  rclcpp::Time latest_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_scan_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_primary_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_fallback_odom_time_{0, 0, RCL_ROS_TIME};
  bool using_primary_odom_{true};
  bool has_yaw_{false};
  double latest_yaw_{0.0};
  std::string latest_odom_frame_id_;
  Pose2D mission_start_pose_;
  Pose2D mission_start_pose_nav2_;

  bool has_prev_odom_{false};
  double prev_odom_x_{0.0};
  double prev_odom_y_{0.0};
  double odom_cumulative_distance_{0.0};
  double segment_start_distance_{0.0};
  double mission_start_distance_{0.0};
  WaitGapPhase wait_gap_phase_{WaitGapPhase::IDLE};
  rclcpp::Time search_gap_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time wait_gap_phase_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time wait_gap_detect_cycle_start_{0, 0, RCL_ROS_TIME};
  int wait_gap_failed_cycle_count_{0};
  std::size_t next_adjust_sequence_index_{0};
  int current_adjust_index_{-1};
  double current_adjust_offset_{0.0};
  double wait_gap_motion_target_distance_{0.0};
  double wait_gap_motion_direction_{0.0};
  EntryGapPhase entry_gap_phase_{EntryGapPhase::IDLE};
  EntryMotionMode entry_motion_mode_{EntryMotionMode::FORWARD_ENTRY};
  rclcpp::Time entry_gap_phase_start_{0, 0, RCL_ROS_TIME};
  double entry_turn_start_yaw_{0.0};
  bool entry_turn_start_yaw_valid_{false};
  double target_gap_yaw_{0.0};
  bool target_gap_yaw_valid_{false};
  Pose2D straight_start_pose_;
  double entry_last_traveled_{0.0};
  bool entry_turn_completed_{false};
  int entry_turn_yaw_stable_count_{0};
  bool entry_straight_completed_{false};
  bool entry_stopped_by_safety_{false};

  rclcpp::Time tracking_stable_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_target_seen_time_{0, 0, RCL_ROS_TIME};

  std::vector<double> ultrasonic_ranges_;
  std::vector<rclcpp::Time> ultrasonic_stamps_;

  rclcpp::Subscription<agv_inventory_system::msg::RecognizedNumber>::SharedPtr recognized_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_sub_;
  rclcpp::Subscription<agv_inventory_system::msg::GapStatus>::SharedPtr gap_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr auto_recharge_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr auto_recharge_charging_flag_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr auto_recharge_recharge_flag_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_primary_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_fallback_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr> ultrasonic_subs_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_log_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr corridor_enable_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr corridor_reverse_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr gap_context_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr entry_side_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr target_lidar_side_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr current_target_cabinet_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr recognizer_enable_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gap_detector_enable_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr distance_estimator_enable_pub_;

  rclcpp::Service<agv_inventory_system::srv::StartMission>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr return_home_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr return_to_charge_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_auto_recharge_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr safe_exit_gap_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_auto_charge_and_depart_srv_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr recognizer_trigger_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr inventory_auto_recharge_start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr inventory_auto_recharge_cancel_client_;
  rclcpp::Client<agv_inventory_system::srv::LiftMoveTimed>::SharedPtr lift_up_client_;
  rclcpp::Client<agv_inventory_system::srv::LiftMoveTimed>::SharedPtr lift_down_client_;
  rclcpp::Client<agv_inventory_system::srv::LiftMoveTimed>::SharedPtr lift_home_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr lift_stop_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr lift_all_off_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
  std::shared_ptr<NavigateGoalHandle> nav2_goal_handle_;
  std::shared_ptr<NavigateGoalHandle> nav2_route_goal_handle_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
