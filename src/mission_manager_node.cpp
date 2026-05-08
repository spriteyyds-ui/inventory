#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
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
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "wheeltec_inventory_system/id_utils.hpp"
#include "wheeltec_inventory_system/msg/gap_status.hpp"
#include "wheeltec_inventory_system/msg/recognized_number.hpp"
#include "wheeltec_inventory_system/scan_sequence_executor.hpp"
#include "wheeltec_inventory_system/scan_sequence_generator.hpp"
#include "wheeltec_inventory_system/srv/start_mission.hpp"
#include "wheeltec_inventory_system/srv/start_test_gap_scan.hpp"
#include "wheeltec_inventory_system/web_api_client.hpp"
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
    distance_estimator_enable_topic_ =
      declare_parameter<std::string>(
      "distance_estimator_enable_topic", "/inventory/distance_estimator_enable");

    start_service_name_ = declare_parameter<std::string>("start_service_name", "/inventory/start_mission");
    cancel_service_name_ = declare_parameter<std::string>("cancel_service_name", "/inventory/cancel_mission");
    recognizer_trigger_service_ =
      declare_parameter<std::string>("recognizer_trigger_service", "/inventory/trigger_recognition");
    start_test_gap_scan_service_name_ =
      declare_parameter<std::string>("start_test_gap_scan_service_name", "/inventory/start_test_gap_scan");

    target_list_param_ = declare_parameter<std::vector<std::string>>("target_list", std::vector<std::string>{});
    route_waypoints_file_ =
      declare_parameter<std::string>("route_waypoints_file", "config/route_waypoints.yaml");
    warehouse_layout_file_ =
      declare_parameter<std::string>("warehouse_layout_file", "config/warehouse_layout.yaml");
    test_gap_scan_params_file_ =
      declare_parameter<std::string>("test_gap_scan_params_file", "config/test_gap_scan_params.yaml");
    gap_scan_map_file_ =
      declare_parameter<std::string>("gap_scan_map_file", "config/gap_scan_map.yaml");
    route_search_failure_policy_ =
      declare_parameter<std::string>("route_search_failure_policy", "error");

    follow_distance_ = declare_parameter<double>("follow_distance", 0.50);
    warehouse_length_ = declare_parameter<double>("warehouse_length", 12.0);
    corridor_speed_ = declare_parameter<double>("corridor_speed", 0.20);
    tracking_speed_ = declare_parameter<double>("tracking_speed", 0.15);
    entry_speed_ = declare_parameter<double>("entry_speed", 0.08);
    entry_distance_ = declare_parameter<double>("entry_distance", 0.70);
    turn_speed_ = declare_parameter<double>("turn_speed", 0.40);
    enter_linear_speed_ = declare_parameter<double>("enter_linear_speed", 0.08);
    enter_angular_speed_ = declare_parameter<double>("enter_angular_speed", 0.30);
    enter_left_angular_multiplier_ = declare_parameter<double>("enter_left_angular_multiplier", 1.0);
    enter_right_angular_multiplier_ = declare_parameter<double>("enter_right_angular_multiplier", -1.0);
    enable_grid_center_entry_ = declare_parameter<bool>("enable_grid_center_entry", true);
    grid_depth_m_ = declare_parameter<double>("grid_depth_m", 2.4);
    left_max_depth_index_ = declare_parameter<int>("left_max_depth_index", 4);
    right_max_depth_index_ = declare_parameter<int>("right_max_depth_index", 3);
    entry_center_offset_m_ = declare_parameter<double>("entry_center_offset_m", 0.0);
    entry_turn_yaw_delta_rad_ = declare_parameter<double>("entry_turn_yaw_delta_rad", 1.57079632679);
    entry_align_yaw_tolerance_rad_ =
      declare_parameter<double>("entry_align_yaw_tolerance_rad", 0.08);
    entry_turn_linear_speed_ = declare_parameter<double>("entry_turn_linear_speed", enter_linear_speed_);
    entry_turn_angular_speed_ = declare_parameter<double>("entry_turn_angular_speed", enter_angular_speed_);
    entry_straight_speed_ = declare_parameter<double>("entry_straight_speed", enter_linear_speed_);
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

    return_home_on_finish_ = declare_parameter<bool>("return_home_on_finish", false);
    return_target_mode_ = declare_parameter<std::string>("return_target_mode", "start");
    charge_pose_x_ = declare_parameter<double>("charge_pose_x", 0.0);
    charge_pose_y_ = declare_parameter<double>("charge_pose_y", 0.0);
    charge_pose_yaw_ = declare_parameter<double>("charge_pose_yaw", 0.0);
    charge_pose_frame_id_ = declare_parameter<std::string>("charge_pose_frame_id", "odom_combined");
    use_nav2_return_ = declare_parameter<bool>("use_nav2_return", true);
    nav2_action_name_ = declare_parameter<std::string>("nav2_action_name", "navigate_to_pose");
    nav2_goal_frame_ = declare_parameter<std::string>("nav2_goal_frame", "map");
    nav2_server_wait_timeout_sec_ = declare_parameter<double>("nav2_server_wait_timeout_sec", 1.0);
    nav2_goal_timeout_sec_ = declare_parameter<double>("nav2_goal_timeout_sec", 40.0);
    nav2_route_waypoint_timeout_sec_ =
      declare_parameter<double>("nav2_route_waypoint_timeout_sec", 60.0);
    nav2_cancel_stop_duration_sec_ =
      declare_parameter<double>("nav2_cancel_stop_duration_sec", 0.50);
    nav2_enable_for_search_return_ = declare_parameter<bool>("nav2_enable_for_search_return", false);
    fallback_rotate_kp_ = declare_parameter<double>("fallback_rotate_kp", 1.5);
    fallback_rotate_max_angular_ = declare_parameter<double>("fallback_rotate_max_angular", 0.8);
    fallback_rotate_tolerance_rad_ = declare_parameter<double>("fallback_rotate_tolerance_rad", 0.08);
    fallback_rotate_stable_time_sec_ = declare_parameter<double>("fallback_rotate_stable_time_sec", 0.25);
    fallback_drive_speed_ = declare_parameter<double>("fallback_drive_speed", 0.20);
    fallback_heading_kp_ = declare_parameter<double>("fallback_heading_kp", 1.2);
    fallback_drive_max_angular_ = declare_parameter<double>("fallback_drive_max_angular", 0.8);
    fallback_goal_tolerance_m_ = declare_parameter<double>("fallback_goal_tolerance_m", 0.18);

    continue_on_error_ = declare_parameter<bool>("continue_on_error", false);
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 10.0);

    use_scan_safety_ = declare_parameter<bool>("use_scan_safety", true);
    entry_front_window_deg_ = declare_parameter<double>("entry_front_window_deg", 22.0);
    entry_front_stop_distance_ = declare_parameter<double>("entry_front_stop_distance", 0.30);
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
    entry_left_align_distance_ = declare_parameter<double>("entry_left_align_distance", 0.22);
    entry_left_turn_angular_ = declare_parameter<double>("entry_left_turn_angular", 0.35);

    enable_test_gap_scan_ = declare_parameter<bool>("enable_test_gap_scan", true);
    test_web_client_mode_ = declare_parameter<std::string>("web_client_mode", "mock");
    test_open_gap_wait_sec_ = declare_parameter<double>("open_gap_wait_sec", 5.0);
    test_close_gap_wait_sec_ = declare_parameter<double>("close_gap_wait_sec", 3.0);
    test_scan_placeholder_wait_sec_ =
      declare_parameter<double>("scan_placeholder_wait_sec", 2.0);
    test_lift_placeholder_wait_sec_ =
      declare_parameter<double>("lift_placeholder_wait_sec", 3.0);
    test_move_grid_placeholder_wait_sec_ =
      declare_parameter<double>("move_grid_placeholder_wait_sec", 1.0);
    test_motion_mode_ = declare_parameter<std::string>("test_motion_mode", "ackermann_reentry_test");
    test_real_motion_enabled_ = declare_parameter<bool>("test_real_motion_enabled", false);
    test_real_motion_single_cabinet_only_ =
      declare_parameter<bool>("test_real_motion_single_cabinet_only", true);
    test_real_motion_target_cabinet_ =
      declare_parameter<int>("test_real_motion_target_cabinet", 3);
    test_real_motion_target_gap_ =
      declare_parameter<std::string>("test_real_motion_target_gap", "gap_03_02");
    test_real_motion_stop_after_scan_ =
      declare_parameter<bool>("test_real_motion_stop_after_scan", true);
    test_real_final_recognition_wait_sec_ =
      declare_parameter<double>("test_real_final_recognition_wait_sec", 8.0);
    test_real_side_row_enabled_ = declare_parameter<bool>("test_real_side_row_enabled", false);
    test_real_side_row_name_ =
      declare_parameter<std::string>("test_real_side_row_name", "row_01_02_03_04");
    test_real_side_row_first_gap_ =
      declare_parameter<std::string>("test_real_side_row_first_gap", "gap_02_03_04");
    const auto first_gap_scan_sequence_param =
      declare_parameter<std::vector<int64_t>>(
      "test_real_side_row_first_gap_scan_sequence",
      std::vector<int64_t>{4, 3});
    test_real_side_row_first_gap_scan_sequence_.assign(
      first_gap_scan_sequence_param.begin(),
      first_gap_scan_sequence_param.end());
    test_real_side_row_second_gap_ =
      declare_parameter<std::string>("test_real_side_row_second_gap", "gap_01_02_03");
    const auto second_gap_scan_sequence_param =
      declare_parameter<std::vector<int64_t>>(
      "test_real_side_row_second_gap_scan_sequence",
      std::vector<int64_t>{2, 1});
    test_real_side_row_second_gap_scan_sequence_.assign(
      second_gap_scan_sequence_param.begin(),
      second_gap_scan_sequence_param.end());
    test_real_side_row_corridor_transfer_enabled_ =
      declare_parameter<bool>("test_real_side_row_corridor_transfer_enabled", true);
    test_real_side_row_corridor_transfer_target_cabinet_ =
      declare_parameter<int>("test_real_side_row_corridor_transfer_target_cabinet", 2);
    test_real_side_row_corridor_transfer_direction_ =
      declare_parameter<std::string>(
      "test_real_side_row_corridor_transfer_direction",
      "toward_cabinet_1");
    test_real_exit_after_each_scan_ = declare_parameter<bool>("test_real_exit_after_each_scan", true);
    test_real_exit_mode_ = declare_parameter<std::string>("test_real_exit_mode", "reverse");
    test_real_exit_speed_ = declare_parameter<double>("test_real_exit_speed", 0.05);
    test_real_exit_extra_distance_m_ =
      declare_parameter<double>("test_real_exit_extra_distance_m", 0.10);
    test_real_exit_timeout_sec_ = declare_parameter<double>("test_real_exit_timeout_sec", 40.0);
    test_real_exit_distance_m_ = declare_parameter<double>("test_real_exit_distance_m", 1.20);
    test_real_exit_turn_enabled_ = declare_parameter<bool>("test_real_exit_turn_enabled", true);
    test_real_exit_turn_angular_speed_ =
      declare_parameter<double>("test_real_exit_turn_angular_speed", 0.25);
    test_real_exit_turn_yaw_tolerance_rad_ =
      declare_parameter<double>("test_real_exit_turn_yaw_tolerance_rad", 0.08);
    test_real_exit_turn_timeout_sec_ =
      declare_parameter<double>("test_real_exit_turn_timeout_sec", 8.0);
    test_real_reentry_for_position_adjustment_ =
      declare_parameter<bool>("test_real_reentry_for_position_adjustment", true);
    test_real_reentry_comment_ =
      declare_parameter<std::string>(
      "test_real_reentry_comment",
      "Use reverse-exit and re-enter as a temporary replacement for in-gap orientation/position adjustment.");
    test_real_grid_motion_enabled_ =
      declare_parameter<bool>("test_real_grid_motion_enabled", false);
    test_real_grid_spacing_m_ =
      declare_parameter<double>("test_real_grid_spacing_m", 0.30);
    test_real_grid_move_speed_ =
      declare_parameter<double>("test_real_grid_move_speed", 0.04);
    test_real_grid_move_timeout_sec_ =
      declare_parameter<double>("test_real_grid_move_timeout_sec", 10.0);
    test_real_grid_move_return_between_layers_ =
      declare_parameter<bool>("test_real_grid_move_return_between_layers", false);
    test_real_close_gap_after_final_exit_ =
      declare_parameter<bool>("test_real_close_gap_after_final_exit", true);
    overall_test_enabled_ = declare_parameter<bool>("overall_test_enabled", true);
    const auto overall_test_sequence_param =
      declare_parameter<std::vector<int64_t>>(
      "overall_test_sequence",
      std::vector<int64_t>{4, 3, 8, 7});
    overall_test_sequence_.assign(
      overall_test_sequence_param.begin(),
      overall_test_sequence_param.end());
    overall_test_left_route_ =
      declare_parameter<std::string>("overall_test_left_route", "left_route");
    overall_test_right_route_ =
      declare_parameter<std::string>("overall_test_right_route", "right_route");
    overall_test_return_home_between_sides_ =
      declare_parameter<bool>("overall_test_return_home_between_sides", true);
    overall_test_return_home_after_done_ =
      declare_parameter<bool>("overall_test_return_home_after_done", true);
    overall_test_recognize_during_nav_ =
      declare_parameter<bool>("overall_test_recognize_during_nav", false);
    overall_test_same_side_next_search_enabled_ =
      declare_parameter<bool>("overall_test_same_side_next_search_enabled", true);
    overall_test_same_side_search_speed_ =
      declare_parameter<double>("overall_test_same_side_search_speed", 0.04);
    overall_test_same_side_search_timeout_sec_ =
      declare_parameter<double>("overall_test_same_side_search_timeout_sec", 20.0);
    overall_test_same_side_pose_hold_enabled_ =
      declare_parameter<bool>("overall_test_same_side_pose_hold_enabled", true);
    overall_test_same_side_fixed_y_m_ =
      declare_parameter<double>("overall_test_same_side_fixed_y_m", 0.575);
    overall_test_same_side_fixed_yaw_rad_ =
      declare_parameter<double>("overall_test_same_side_fixed_yaw_rad", -3.1400);
    overall_test_same_side_yaw_kp_ =
      declare_parameter<double>("overall_test_same_side_yaw_kp", 0.40);
    overall_test_same_side_yaw_deadband_rad_ =
      declare_parameter<double>("overall_test_same_side_yaw_deadband_rad", 0.03);
    overall_test_same_side_y_kp_ =
      declare_parameter<double>("overall_test_same_side_y_kp", 0.30);
    overall_test_same_side_y_deadband_m_ =
      declare_parameter<double>("overall_test_same_side_y_deadband_m", 0.03);
    overall_test_same_side_y_correction_sign_ =
      declare_parameter<double>("overall_test_same_side_y_correction_sign", 1.0);
    overall_test_same_side_max_angular_ =
      declare_parameter<double>("overall_test_same_side_max_angular", 0.15);
    overall_test_final_recognition_wait_sec_ =
      declare_parameter<double>("overall_test_final_recognition_wait_sec", 5.0);
    overall_test_recognition_fallback_enabled_ =
      declare_parameter<bool>("overall_test_recognition_fallback_enabled", true);
    overall_test_recognition_fallback_speed_ =
      declare_parameter<double>("overall_test_recognition_fallback_speed", 0.04);
    overall_test_recognition_fallback_wait_sec_ =
      declare_parameter<double>("overall_test_recognition_fallback_wait_sec", 2.0);
    overall_test_recognition_fallback_timeout_sec_ =
      declare_parameter<double>("overall_test_recognition_fallback_timeout_sec", 20.0);
    overall_test_recognition_fallback_sequence_ =
      declare_parameter<std::vector<double>>(
      "overall_test_recognition_fallback_sequence_m",
      std::vector<double>{-0.30, 0.60, -0.30});
    post_gap_detect_advance_enabled_ =
      declare_parameter<bool>("post_gap_detect_advance_enabled", true);
    post_gap_detect_advance_distance_m_ =
      declare_parameter<double>("post_gap_detect_advance_distance_m", 0.25);
    post_gap_detect_advance_speed_ =
      declare_parameter<double>("post_gap_detect_advance_speed", 0.04);
    post_gap_detect_advance_timeout_sec_ =
      declare_parameter<double>("post_gap_detect_advance_timeout_sec", 8.0);
    test_scan_layers_ = declare_parameter<int>("scan_layers", 2);
    test_scan_depth_count_ = declare_parameter<int>("scan_depth_count", 3);
    test_default_scan_side_ = declare_parameter<std::string>("default_scan_side", "left");
    test_web_base_url_ = declare_parameter<std::string>("web_base_url", "");
    test_web_open_gap_endpoint_ =
      declare_parameter<std::string>("web_open_gap_endpoint", "/api/gap/open");
    test_web_close_gap_endpoint_ =
      declare_parameter<std::string>("web_close_gap_endpoint", "/api/gap/close");
    test_web_status_endpoint_ =
      declare_parameter<std::string>("web_status_endpoint", "/api/robot/status");
    test_web_result_endpoint_ =
      declare_parameter<std::string>("web_result_endpoint", "/api/inventory/result");

    recognized_sub_ = create_subscription<wheeltec_inventory_system::msg::RecognizedNumber>(
      recognized_topic_,
      10,
      std::bind(&MissionManagerNode::recognized_callback, this, std::placeholders::_1));

    distance_sub_ = create_subscription<std_msgs::msg::Float32>(
      distance_topic_,
      10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        latest_distance_ = static_cast<double>(msg->data);
        has_distance_ = std::isfinite(latest_distance_) && latest_distance_ > 0.0;
      });

    gap_sub_ = create_subscription<wheeltec_inventory_system::msg::GapStatus>(
      gap_topic_,
      10,
      [this](const wheeltec_inventory_system::msg::GapStatus::SharedPtr msg) {
        latest_gap_ = *msg;
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
    recognizer_enable_pub_ =
      create_publisher<std_msgs::msg::Bool>(recognizer_enable_topic_, control_qos);
    gap_detector_enable_pub_ =
      create_publisher<std_msgs::msg::Bool>(gap_detector_enable_topic_, control_qos);
    distance_estimator_enable_pub_ =
      create_publisher<std_msgs::msg::Bool>(distance_estimator_enable_topic_, control_qos);

    start_srv_ = create_service<wheeltec_inventory_system::srv::StartMission>(
      start_service_name_,
      std::bind(
        &MissionManagerNode::start_service_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    cancel_srv_ = create_service<std_srvs::srv::Trigger>(
      cancel_service_name_,
      std::bind(
        &MissionManagerNode::cancel_service_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    start_test_gap_scan_srv_ =
      create_service<wheeltec_inventory_system::srv::StartTestGapScan>(
      start_test_gap_scan_service_name_,
      std::bind(
        &MissionManagerNode::start_test_gap_scan_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    recognizer_trigger_client_ = create_client<std_srvs::srv::SetBool>(
      recognizer_trigger_service_);
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
    std::string test_scan_load_error;
    test_gap_scan_config_loaded_ = load_test_gap_scan_config(test_scan_load_error);
    if (!test_gap_scan_config_loaded_) {
      RCLCPP_WARN(get_logger(), "测试盘库配置加载失败: %s", test_scan_load_error.c_str());
    }

    const double period = 1.0 / std::max(1.0, control_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period)),
      std::bind(&MissionManagerNode::on_timer, this));

    set_state(State::IDLE, "系统待机");
    RCLCPP_INFO(
      get_logger(),
      "任务管理节点已启动，odom主话题=%s，fallback=%s，return_mode=%s，nav2=%s",
      odom_topic_.c_str(),
      odom_fallback_topic_.c_str(),
      return_target_mode_.c_str(),
      use_nav2_return_ ? "on" : "off");
  }

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  enum class State
  {
    IDLE,
    NAV_ROUTE,
    CORRIDOR_NAV,
    TARGET_TRACKING,
    SEARCH_GAP,
    WAITING_GAP,
    ENTERING_GAP,
    INVENTORYING,
    RETURNING,
    DONE,
    ERROR,
    TEST_IDLE,
    TEST_REQUESTING_OPEN_GAP,
    TEST_WAITING_OPEN_DELAY,
    TEST_SCANNING_PLACEHOLDER,
    TEST_REQUESTING_CLOSE_GAP,
    TEST_WAITING_CLOSE_DELAY,
    TEST_NEXT_GAP,
    TEST_DONE,
    TEST_ERROR,
    TEST_REAL_PREPARE_NAV,
    TEST_REAL_NAV_TO_TARGET,
    TEST_REAL_TARGET_TRACKING,
    TEST_REAL_FINAL_RECOGNITION_WAIT,
    TEST_REAL_WAITING_GAP,
    TEST_REAL_ENTERING_GAP,
    TEST_REAL_IN_GAP_SCAN,
    TEST_REAL_STOP_AFTER_SCAN,
    TEST_REAL_EXIT_GAP,
    TEST_REAL_REENTER_FOR_ADJUSTED_SCAN,
    TEST_REAL_ADJUSTED_SIDE_SCAN,
    TEST_REAL_CORRIDOR_TRANSFER,
    TEST_REAL_PREPARE_NEXT_GAP,
    TEST_REAL_REENTER_NEXT_GAP,
    TEST_REAL_NEXT_GAP_SCAN,
    TEST_REAL_FINAL_EXIT_GAP,
    OVERALL_TEST_PREPARE_TARGET,
    OVERALL_TEST_NAV_TO_OBSERVE,
    OVERALL_TEST_POST_ROUTE_RECOGNITION_WAIT,
    OVERALL_TEST_RECOGNITION_FALLBACK,
    OVERALL_TEST_TARGET_DISTANCE_ALIGN,
    OVERALL_TEST_SEARCH_GAP,
    OVERALL_TEST_POST_GAP_DETECT_ADVANCE,
    OVERALL_TEST_ENTERING_GAP,
    OVERALL_TEST_SCAN_PLACEHOLDER,
    OVERALL_TEST_EXIT_GAP,
    OVERALL_TEST_ADVANCE_NEXT_TARGET,
    OVERALL_TEST_SAME_SIDE_NEXT_SEARCH,
    OVERALL_TEST_RETURN_HOME_BETWEEN_SIDES,
    OVERALL_TEST_DONE,
  };

  enum class OverallRecognitionFallbackPhase
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

  enum class ReturnTargetMode
  {
    START,
    CHARGE,
  };

  enum class FallbackDriveMode
  {
    NONE,
    CORRIDOR_DISTANCE,
    DIRECT_POSE,
  };

  enum class FallbackPhase
  {
    IDLE,
    ROTATING,
    DRIVING,
  };

  enum class WaitGapPhase
  {
    IDLE,
    RETREATING,
    STOP_BEFORE_DETECT,
    DETECTING_GAP,
    POSE_ADJUSTING,
  };

  enum class TestRealSideRowPhase
  {
    NONE,
    FIRST_PRIMARY_SCAN,
    FIRST_ADJUSTED_SCAN,
    CORRIDOR_TRANSFER,
    SECOND_PRIMARY_SCAN,
    SECOND_ADJUSTED_SCAN,
    COMPLETE,
  };

  enum class TestRealAfterExitAction
  {
    NONE,
    REENTER_ADJUSTED,
    CORRIDOR_TRANSFER,
    CLOSE_AND_DONE,
    FINAL_CLOSE_AND_DONE,
  };

  enum class TestRealExitPhase
  {
    STRAIGHT_REVERSE,
    ARC_REVERSE,
  };

  enum class OverallTestReturnReason
  {
    NONE,
    BETWEEN_SIDES,
    FINAL_DONE,
  };

  enum class EntryGapPhase
  {
    IDLE,
    ENTERING_TURN,
    ENTERING_STRAIGHT_ALIGN,
    MOVING_TO_GRID_CENTER,
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

  struct TestGapScanPlan
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

  static std::string state_to_string(State s)
  {
    switch (s) {
      case State::IDLE:
        return "IDLE";
      case State::NAV_ROUTE:
        return "NAV_ROUTE";
      case State::CORRIDOR_NAV:
        return "CORRIDOR_NAV";
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
      case State::RETURNING:
        return "RETURNING";
      case State::DONE:
        return "DONE";
      case State::ERROR:
        return "ERROR";
      case State::TEST_IDLE:
        return "TEST_IDLE";
      case State::TEST_REQUESTING_OPEN_GAP:
        return "TEST_REQUESTING_OPEN_GAP";
      case State::TEST_WAITING_OPEN_DELAY:
        return "TEST_WAITING_OPEN_DELAY";
      case State::TEST_SCANNING_PLACEHOLDER:
        return "TEST_SCANNING_PLACEHOLDER";
      case State::TEST_REQUESTING_CLOSE_GAP:
        return "TEST_REQUESTING_CLOSE_GAP";
      case State::TEST_WAITING_CLOSE_DELAY:
        return "TEST_WAITING_CLOSE_DELAY";
      case State::TEST_NEXT_GAP:
        return "TEST_NEXT_GAP";
      case State::TEST_DONE:
        return "TEST_DONE";
      case State::TEST_ERROR:
        return "TEST_ERROR";
      case State::TEST_REAL_PREPARE_NAV:
        return "TEST_REAL_PREPARE_NAV";
      case State::TEST_REAL_NAV_TO_TARGET:
        return "TEST_REAL_NAV_TO_TARGET";
      case State::TEST_REAL_TARGET_TRACKING:
        return "TEST_REAL_TARGET_TRACKING";
      case State::TEST_REAL_FINAL_RECOGNITION_WAIT:
        return "TEST_REAL_FINAL_RECOGNITION_WAIT";
      case State::TEST_REAL_WAITING_GAP:
        return "TEST_REAL_WAITING_GAP";
      case State::TEST_REAL_ENTERING_GAP:
        return "TEST_REAL_ENTERING_GAP";
      case State::TEST_REAL_IN_GAP_SCAN:
        return "TEST_REAL_IN_GAP_SCAN";
      case State::TEST_REAL_STOP_AFTER_SCAN:
        return "TEST_REAL_STOP_AFTER_SCAN";
      case State::TEST_REAL_EXIT_GAP:
        return "TEST_REAL_EXIT_GAP";
      case State::TEST_REAL_REENTER_FOR_ADJUSTED_SCAN:
        return "TEST_REAL_REENTER_FOR_ADJUSTED_SCAN";
      case State::TEST_REAL_ADJUSTED_SIDE_SCAN:
        return "TEST_REAL_ADJUSTED_SIDE_SCAN";
      case State::TEST_REAL_CORRIDOR_TRANSFER:
        return "TEST_REAL_CORRIDOR_TRANSFER";
      case State::TEST_REAL_PREPARE_NEXT_GAP:
        return "TEST_REAL_PREPARE_NEXT_GAP";
      case State::TEST_REAL_REENTER_NEXT_GAP:
        return "TEST_REAL_REENTER_NEXT_GAP";
      case State::TEST_REAL_NEXT_GAP_SCAN:
        return "TEST_REAL_NEXT_GAP_SCAN";
      case State::TEST_REAL_FINAL_EXIT_GAP:
        return "TEST_REAL_FINAL_EXIT_GAP";
      case State::OVERALL_TEST_PREPARE_TARGET:
        return "OVERALL_TEST_PREPARE_TARGET";
      case State::OVERALL_TEST_NAV_TO_OBSERVE:
        return "OVERALL_TEST_NAV_TO_OBSERVE";
      case State::OVERALL_TEST_POST_ROUTE_RECOGNITION_WAIT:
        return "OVERALL_TEST_POST_ROUTE_RECOGNITION_WAIT";
      case State::OVERALL_TEST_RECOGNITION_FALLBACK:
        return "OVERALL_TEST_RECOGNITION_FALLBACK";
      case State::OVERALL_TEST_TARGET_DISTANCE_ALIGN:
        return "OVERALL_TEST_TARGET_DISTANCE_ALIGN";
      case State::OVERALL_TEST_SEARCH_GAP:
        return "OVERALL_TEST_SEARCH_GAP";
      case State::OVERALL_TEST_POST_GAP_DETECT_ADVANCE:
        return "OVERALL_TEST_POST_GAP_DETECT_ADVANCE";
      case State::OVERALL_TEST_ENTERING_GAP:
        return "OVERALL_TEST_ENTERING_GAP";
      case State::OVERALL_TEST_SCAN_PLACEHOLDER:
        return "OVERALL_TEST_SCAN_PLACEHOLDER";
      case State::OVERALL_TEST_EXIT_GAP:
        return "OVERALL_TEST_EXIT_GAP";
      case State::OVERALL_TEST_ADVANCE_NEXT_TARGET:
        return "OVERALL_TEST_ADVANCE_NEXT_TARGET";
      case State::OVERALL_TEST_SAME_SIDE_NEXT_SEARCH:
        return "OVERALL_TEST_SAME_SIDE_NEXT_SEARCH";
      case State::OVERALL_TEST_RETURN_HOME_BETWEEN_SIDES:
        return "OVERALL_TEST_RETURN_HOME_BETWEEN_SIDES";
      case State::OVERALL_TEST_DONE:
        return "OVERALL_TEST_DONE";
      default:
        return "UNKNOWN";
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
    return
      value_deg >= std::min(start_deg, end_deg) &&
      value_deg <= std::max(start_deg, end_deg);
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
    side = wheeltec_inventory_system::trim(side);
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

  static std::string search_direction_to_string(SearchDirection direction)
  {
    return direction == SearchDirection::BACKWARD ? "backward" : "forward";
  }

  static bool try_parse_search_direction(std::string direction, SearchDirection & parsed)
  {
    direction = wheeltec_inventory_system::trim(direction);
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

  static std::string test_real_exit_phase_to_string(TestRealExitPhase phase)
  {
    switch (phase) {
      case TestRealExitPhase::STRAIGHT_REVERSE:
        return "STRAIGHT_REVERSE";
      case TestRealExitPhase::ARC_REVERSE:
        return "ARC_REVERSE";
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

  void publish_test_scan_log(const std::string & text)
  {
    publish_log("[mission_manager][test_scan] " + text);
  }

  void publish_test_real_log(const std::string & text)
  {
    publish_log("[mission_manager][test_real] " + text);
  }

  void publish_overall_test_log(const std::string & text)
  {
    publish_log("[mission_manager][OVERALL_TEST] " + text);
  }

  void publish_motion_test_log(const std::string & text)
  {
    if (overall_test_active_) {
      publish_overall_test_log(text);
    } else {
      publish_test_real_log(text);
    }
  }

  std::string real_motion_reject_message() const
  {
    return
      "real motion test only supports " + test_real_motion_target_gap_ +
      " with scan_cabinets=[" + std::to_string(test_real_motion_target_cabinet_) + "]";
  }

  bool is_nav_route_like_state(State s) const
  {
    return
      s == State::NAV_ROUTE ||
      s == State::TEST_REAL_NAV_TO_TARGET ||
      s == State::OVERALL_TEST_NAV_TO_OBSERVE;
  }

  bool is_target_tracking_like_state(State s) const
  {
    return
      s == State::TARGET_TRACKING ||
      s == State::TEST_REAL_TARGET_TRACKING ||
      s == State::OVERALL_TEST_TARGET_DISTANCE_ALIGN;
  }

  bool is_test_real_recognition_state(State s) const
  {
    return
      test_real_motion_active_ &&
      (s == State::TEST_REAL_NAV_TO_TARGET ||
      s == State::TEST_REAL_TARGET_TRACKING ||
      s == State::TEST_REAL_FINAL_RECOGNITION_WAIT);
  }

  bool is_overall_test_recognition_state(State s) const
  {
    return
      overall_test_active_ &&
      ((s == State::OVERALL_TEST_NAV_TO_OBSERVE && overall_test_recognize_during_nav_) ||
      s == State::OVERALL_TEST_POST_ROUTE_RECOGNITION_WAIT ||
      s == State::OVERALL_TEST_RECOGNITION_FALLBACK ||
      s == State::OVERALL_TEST_SAME_SIDE_NEXT_SEARCH);
  }

  static std::string side_row_phase_to_string(TestRealSideRowPhase phase)
  {
    switch (phase) {
      case TestRealSideRowPhase::FIRST_PRIMARY_SCAN:
        return "FIRST_PRIMARY_SCAN";
      case TestRealSideRowPhase::FIRST_ADJUSTED_SCAN:
        return "FIRST_ADJUSTED_SCAN";
      case TestRealSideRowPhase::CORRIDOR_TRANSFER:
        return "CORRIDOR_TRANSFER";
      case TestRealSideRowPhase::SECOND_PRIMARY_SCAN:
        return "SECOND_PRIMARY_SCAN";
      case TestRealSideRowPhase::SECOND_ADJUSTED_SCAN:
        return "SECOND_ADJUSTED_SCAN";
      case TestRealSideRowPhase::COMPLETE:
        return "COMPLETE";
      case TestRealSideRowPhase::NONE:
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
        std::filesystem::path(ament_index_cpp::get_package_share_directory("wheeltec_inventory_system"));
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
      std::filesystem::current_path() / "src" / "wheeltec_inventory_system" / path;
    if (std::filesystem::exists(source_candidate)) {
      return source_candidate;
    }
    return cwd_candidate;
  }

  void apply_test_gap_scan_runtime_params()
  {
    wheeltec_inventory_system::ScanExecutionParams execution_params;
    execution_params.scan_placeholder_wait_sec = std::max(0.0, test_scan_placeholder_wait_sec_);
    execution_params.lift_placeholder_wait_sec = std::max(0.0, test_lift_placeholder_wait_sec_);
    execution_params.move_grid_placeholder_wait_sec =
      std::max(0.0, test_move_grid_placeholder_wait_sec_);
    scan_sequence_executor_.setParams(execution_params);

    wheeltec_inventory_system::WebApiClientParams web_params;
    web_params.web_client_mode = test_web_client_mode_;
    web_params.web_base_url = test_web_base_url_;
    web_params.web_open_gap_endpoint = test_web_open_gap_endpoint_;
    web_params.web_close_gap_endpoint = test_web_close_gap_endpoint_;
    web_params.web_status_endpoint = test_web_status_endpoint_;
    web_params.web_result_endpoint = test_web_result_endpoint_;
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

      test_gap_scan_map_cabinets_by_gap_id_ = gap_map;
      RCLCPP_INFO(
        get_logger(),
        "已加载测试 gap 扫描映射: %s gaps=%zu",
        map_path.string().c_str(),
        test_gap_scan_map_cabinets_by_gap_id_.size());
      return true;
    } catch (const std::exception & ex) {
      reason = "解析 gap_scan_map.yaml 失败: " + std::string(ex.what());
      return false;
    }
  }

  bool load_test_gap_scan_config(std::string & reason)
  {
    reason.clear();
    apply_test_gap_scan_runtime_params();

    try {
      const auto params_path = resolve_route_config_path(test_gap_scan_params_file_);
      if (!std::filesystem::exists(params_path)) {
        reason = "test_gap_scan_params.yaml 不存在: " + params_path.string();
        return false;
      }

      const YAML::Node root = YAML::LoadFile(params_path.string());
      if (root["enable_test_gap_scan"]) {
        enable_test_gap_scan_ = root["enable_test_gap_scan"].as<bool>();
      }
      if (root["web_client_mode"]) {
        test_web_client_mode_ = root["web_client_mode"].as<std::string>();
      }
      if (root["open_gap_wait_sec"]) {
        test_open_gap_wait_sec_ = root["open_gap_wait_sec"].as<double>();
      }
      if (root["close_gap_wait_sec"]) {
        test_close_gap_wait_sec_ = root["close_gap_wait_sec"].as<double>();
      }
      if (root["scan_placeholder_wait_sec"]) {
        test_scan_placeholder_wait_sec_ = root["scan_placeholder_wait_sec"].as<double>();
      }
      if (root["lift_placeholder_wait_sec"]) {
        test_lift_placeholder_wait_sec_ = root["lift_placeholder_wait_sec"].as<double>();
      }
      if (root["move_grid_placeholder_wait_sec"]) {
        test_move_grid_placeholder_wait_sec_ = root["move_grid_placeholder_wait_sec"].as<double>();
      }
      if (root["test_motion_mode"]) {
        test_motion_mode_ = root["test_motion_mode"].as<std::string>();
      }
      if (root["test_real_motion_enabled"]) {
        test_real_motion_enabled_ = root["test_real_motion_enabled"].as<bool>();
      }
      if (root["test_real_motion_single_cabinet_only"]) {
        test_real_motion_single_cabinet_only_ =
          root["test_real_motion_single_cabinet_only"].as<bool>();
      }
      if (root["test_real_motion_target_cabinet"]) {
        test_real_motion_target_cabinet_ = root["test_real_motion_target_cabinet"].as<int>();
      }
      if (root["test_real_motion_target_gap"]) {
        test_real_motion_target_gap_ = root["test_real_motion_target_gap"].as<std::string>();
      }
      if (root["test_real_motion_stop_after_scan"]) {
        test_real_motion_stop_after_scan_ = root["test_real_motion_stop_after_scan"].as<bool>();
      }
      if (root["test_real_final_recognition_wait_sec"]) {
        test_real_final_recognition_wait_sec_ =
          root["test_real_final_recognition_wait_sec"].as<double>();
      }
      if (root["test_real_side_row_enabled"]) {
        test_real_side_row_enabled_ = root["test_real_side_row_enabled"].as<bool>();
      }
      if (root["test_real_side_row_name"]) {
        test_real_side_row_name_ = root["test_real_side_row_name"].as<std::string>();
      }
      if (root["test_real_side_row_first_gap"]) {
        test_real_side_row_first_gap_ = root["test_real_side_row_first_gap"].as<std::string>();
      }
      if (root["test_real_side_row_first_gap_scan_sequence"]) {
        test_real_side_row_first_gap_scan_sequence_.clear();
        for (const auto cabinet_node : root["test_real_side_row_first_gap_scan_sequence"]) {
          test_real_side_row_first_gap_scan_sequence_.push_back(cabinet_node.as<int>());
        }
      }
      if (root["test_real_side_row_second_gap"]) {
        test_real_side_row_second_gap_ = root["test_real_side_row_second_gap"].as<std::string>();
      }
      if (root["test_real_side_row_second_gap_scan_sequence"]) {
        test_real_side_row_second_gap_scan_sequence_.clear();
        for (const auto cabinet_node : root["test_real_side_row_second_gap_scan_sequence"]) {
          test_real_side_row_second_gap_scan_sequence_.push_back(cabinet_node.as<int>());
        }
      }
      if (root["test_real_side_row_corridor_transfer_enabled"]) {
        test_real_side_row_corridor_transfer_enabled_ =
          root["test_real_side_row_corridor_transfer_enabled"].as<bool>();
      }
      if (root["test_real_side_row_corridor_transfer_target_cabinet"]) {
        test_real_side_row_corridor_transfer_target_cabinet_ =
          root["test_real_side_row_corridor_transfer_target_cabinet"].as<int>();
      }
      if (root["test_real_side_row_corridor_transfer_direction"]) {
        test_real_side_row_corridor_transfer_direction_ =
          root["test_real_side_row_corridor_transfer_direction"].as<std::string>();
      }
      if (root["test_real_exit_after_each_scan"]) {
        test_real_exit_after_each_scan_ = root["test_real_exit_after_each_scan"].as<bool>();
      }
      if (root["test_real_exit_mode"]) {
        test_real_exit_mode_ = root["test_real_exit_mode"].as<std::string>();
      }
      if (root["test_real_exit_speed"]) {
        test_real_exit_speed_ = root["test_real_exit_speed"].as<double>();
      }
      if (root["test_real_exit_extra_distance_m"]) {
        test_real_exit_extra_distance_m_ = root["test_real_exit_extra_distance_m"].as<double>();
      }
      if (root["test_real_exit_timeout_sec"]) {
        test_real_exit_timeout_sec_ = root["test_real_exit_timeout_sec"].as<double>();
      }
      if (root["test_real_exit_distance_m"]) {
        test_real_exit_distance_m_ = root["test_real_exit_distance_m"].as<double>();
      }
      if (root["test_real_exit_turn_enabled"]) {
        test_real_exit_turn_enabled_ = root["test_real_exit_turn_enabled"].as<bool>();
      }
      if (root["test_real_exit_turn_angular_speed"]) {
        test_real_exit_turn_angular_speed_ = root["test_real_exit_turn_angular_speed"].as<double>();
      }
      if (root["test_real_exit_turn_yaw_tolerance_rad"]) {
        test_real_exit_turn_yaw_tolerance_rad_ =
          root["test_real_exit_turn_yaw_tolerance_rad"].as<double>();
      }
      if (root["test_real_exit_turn_timeout_sec"]) {
        test_real_exit_turn_timeout_sec_ = root["test_real_exit_turn_timeout_sec"].as<double>();
      }
      if (root["test_real_reentry_for_position_adjustment"]) {
        test_real_reentry_for_position_adjustment_ =
          root["test_real_reentry_for_position_adjustment"].as<bool>();
      }
      if (root["test_real_reentry_comment"]) {
        test_real_reentry_comment_ = root["test_real_reentry_comment"].as<std::string>();
      }
      if (root["test_real_grid_motion_enabled"]) {
        test_real_grid_motion_enabled_ = root["test_real_grid_motion_enabled"].as<bool>();
      }
      if (root["test_real_grid_spacing_m"]) {
        test_real_grid_spacing_m_ = root["test_real_grid_spacing_m"].as<double>();
      }
      if (root["test_real_grid_move_speed"]) {
        test_real_grid_move_speed_ = root["test_real_grid_move_speed"].as<double>();
      }
      if (root["test_real_grid_move_timeout_sec"]) {
        test_real_grid_move_timeout_sec_ = root["test_real_grid_move_timeout_sec"].as<double>();
      }
      if (root["test_real_grid_move_return_between_layers"]) {
        test_real_grid_move_return_between_layers_ =
          root["test_real_grid_move_return_between_layers"].as<bool>();
      }
      if (root["test_real_close_gap_after_final_exit"]) {
        test_real_close_gap_after_final_exit_ =
          root["test_real_close_gap_after_final_exit"].as<bool>();
      }
      if (root["overall_test_enabled"]) {
        overall_test_enabled_ = root["overall_test_enabled"].as<bool>();
      }
      if (root["overall_test_sequence"]) {
        overall_test_sequence_.clear();
        for (const auto cabinet_node : root["overall_test_sequence"]) {
          overall_test_sequence_.push_back(cabinet_node.as<int>());
        }
      }
      if (root["overall_test_left_route"]) {
        overall_test_left_route_ = root["overall_test_left_route"].as<std::string>();
      }
      if (root["overall_test_right_route"]) {
        overall_test_right_route_ = root["overall_test_right_route"].as<std::string>();
      }
      if (root["overall_test_return_home_between_sides"]) {
        overall_test_return_home_between_sides_ =
          root["overall_test_return_home_between_sides"].as<bool>();
      }
      if (root["overall_test_return_home_after_done"]) {
        overall_test_return_home_after_done_ =
          root["overall_test_return_home_after_done"].as<bool>();
      }
      if (root["overall_test_recognize_during_nav"]) {
        overall_test_recognize_during_nav_ =
          root["overall_test_recognize_during_nav"].as<bool>();
      }
      if (root["overall_test_same_side_next_search_enabled"]) {
        overall_test_same_side_next_search_enabled_ =
          root["overall_test_same_side_next_search_enabled"].as<bool>();
      }
      if (root["overall_test_same_side_search_speed"]) {
        overall_test_same_side_search_speed_ =
          root["overall_test_same_side_search_speed"].as<double>();
      }
      if (root["overall_test_same_side_search_timeout_sec"]) {
        overall_test_same_side_search_timeout_sec_ =
          root["overall_test_same_side_search_timeout_sec"].as<double>();
      }
      if (root["overall_test_same_side_pose_hold_enabled"]) {
        overall_test_same_side_pose_hold_enabled_ =
          root["overall_test_same_side_pose_hold_enabled"].as<bool>();
      }
      if (root["overall_test_same_side_fixed_y_m"]) {
        overall_test_same_side_fixed_y_m_ =
          root["overall_test_same_side_fixed_y_m"].as<double>();
      }
      if (root["overall_test_same_side_fixed_yaw_rad"]) {
        overall_test_same_side_fixed_yaw_rad_ =
          root["overall_test_same_side_fixed_yaw_rad"].as<double>();
      }
      if (root["overall_test_same_side_yaw_kp"]) {
        overall_test_same_side_yaw_kp_ =
          root["overall_test_same_side_yaw_kp"].as<double>();
      }
      if (root["overall_test_same_side_yaw_deadband_rad"]) {
        overall_test_same_side_yaw_deadband_rad_ =
          root["overall_test_same_side_yaw_deadband_rad"].as<double>();
      }
      if (root["overall_test_same_side_y_kp"]) {
        overall_test_same_side_y_kp_ =
          root["overall_test_same_side_y_kp"].as<double>();
      }
      if (root["overall_test_same_side_y_deadband_m"]) {
        overall_test_same_side_y_deadband_m_ =
          root["overall_test_same_side_y_deadband_m"].as<double>();
      }
      if (root["overall_test_same_side_y_correction_sign"]) {
        overall_test_same_side_y_correction_sign_ =
          root["overall_test_same_side_y_correction_sign"].as<double>();
      }
      if (root["overall_test_same_side_max_angular"]) {
        overall_test_same_side_max_angular_ =
          root["overall_test_same_side_max_angular"].as<double>();
      }
      if (root["overall_test_final_recognition_wait_sec"]) {
        overall_test_final_recognition_wait_sec_ =
          root["overall_test_final_recognition_wait_sec"].as<double>();
      }
      if (root["overall_test_recognition_fallback_enabled"]) {
        overall_test_recognition_fallback_enabled_ =
          root["overall_test_recognition_fallback_enabled"].as<bool>();
      }
      if (root["overall_test_recognition_fallback_speed"]) {
        overall_test_recognition_fallback_speed_ =
          root["overall_test_recognition_fallback_speed"].as<double>();
      }
      if (root["overall_test_recognition_fallback_wait_sec"]) {
        overall_test_recognition_fallback_wait_sec_ =
          root["overall_test_recognition_fallback_wait_sec"].as<double>();
      }
      if (root["overall_test_recognition_fallback_timeout_sec"]) {
        overall_test_recognition_fallback_timeout_sec_ =
          root["overall_test_recognition_fallback_timeout_sec"].as<double>();
      }
      if (root["overall_test_recognition_fallback_sequence_m"]) {
        overall_test_recognition_fallback_sequence_.clear();
        for (const auto step_node : root["overall_test_recognition_fallback_sequence_m"]) {
          overall_test_recognition_fallback_sequence_.push_back(step_node.as<double>());
        }
      }
      if (root["post_gap_detect_advance_enabled"]) {
        post_gap_detect_advance_enabled_ = root["post_gap_detect_advance_enabled"].as<bool>();
      }
      if (root["post_gap_detect_advance_distance_m"]) {
        post_gap_detect_advance_distance_m_ =
          root["post_gap_detect_advance_distance_m"].as<double>();
      }
      if (root["post_gap_detect_advance_speed"]) {
        post_gap_detect_advance_speed_ = root["post_gap_detect_advance_speed"].as<double>();
      }
      if (root["post_gap_detect_advance_timeout_sec"]) {
        post_gap_detect_advance_timeout_sec_ =
          root["post_gap_detect_advance_timeout_sec"].as<double>();
      }
      if (root["scan_layers"]) {
        test_scan_layers_ = root["scan_layers"].as<int>();
      }
      if (root["scan_depth_count"]) {
        test_scan_depth_count_ = root["scan_depth_count"].as<int>();
      }
      if (root["default_scan_side"]) {
        test_default_scan_side_ = root["default_scan_side"].as<std::string>();
      }
      if (root["web_base_url"]) {
        test_web_base_url_ = root["web_base_url"].as<std::string>();
      }
      if (root["web_open_gap_endpoint"]) {
        test_web_open_gap_endpoint_ = root["web_open_gap_endpoint"].as<std::string>();
      }
      if (root["web_close_gap_endpoint"]) {
        test_web_close_gap_endpoint_ = root["web_close_gap_endpoint"].as<std::string>();
      }
      if (root["web_status_endpoint"]) {
        test_web_status_endpoint_ = root["web_status_endpoint"].as<std::string>();
      }
      if (root["web_result_endpoint"]) {
        test_web_result_endpoint_ = root["web_result_endpoint"].as<std::string>();
      }

      const YAML::Node plan_root = root["test_inventory_plan"];
      if (!plan_root || !plan_root.IsSequence() || plan_root.size() == 0U) {
        reason = "test_gap_scan_params.yaml 缺少非空 test_inventory_plan";
        return false;
      }

      std::vector<TestGapScanPlan> loaded_plans;
      for (std::size_t i = 0; i < plan_root.size(); ++i) {
        const YAML::Node item = plan_root[i];
        if (!item || !item.IsMap() || !item["gap_id"] || !item["scan_cabinets"]) {
          reason = "test_inventory_plan 条目必须包含 gap_id 和 scan_cabinets";
          return false;
        }

        TestGapScanPlan plan;
        plan.gap_id = item["gap_id"].as<std::string>();
        const YAML::Node cabinets = item["scan_cabinets"];
        if (!cabinets.IsSequence() || cabinets.size() == 0U) {
          reason = "scan_cabinets 必须为非空序列: " + plan.gap_id;
          return false;
        }
        for (std::size_t j = 0; j < cabinets.size(); ++j) {
          plan.scan_cabinets.push_back(cabinets[j].as<int>());
        }
        loaded_plans.push_back(plan);
      }

      configured_test_inventory_plan_ = loaded_plans;
      test_inventory_plan_by_gap_id_.clear();
      for (const auto & plan : configured_test_inventory_plan_) {
        test_inventory_plan_by_gap_id_[plan.gap_id] = plan.scan_cabinets;
      }

      std::string gap_map_reason;
      if (!load_gap_scan_map_config(gap_map_reason)) {
        RCLCPP_WARN(get_logger(), "测试 gap 映射加载失败，先使用 test_inventory_plan: %s", gap_map_reason.c_str());
      }

      apply_test_gap_scan_runtime_params();
      RCLCPP_INFO(
        get_logger(),
        "已加载测试盘库配置: %s enable=%s mode=%s plan_count=%zu layers=%d depth_count=%d",
        params_path.string().c_str(),
        enable_test_gap_scan_ ? "true" : "false",
        test_web_client_mode_.c_str(),
        configured_test_inventory_plan_.size(),
        test_scan_layers_,
        test_scan_depth_count_);
      RCLCPP_INFO(
        get_logger(),
        "测试真实运动配置: enabled=%s target_gap=%s target_cabinet=%d single_only=%s "
        "stop_after_scan=%s final_recognition_wait=%.2f",
        test_real_motion_enabled_ ? "true" : "false",
        test_real_motion_target_gap_.c_str(),
        test_real_motion_target_cabinet_,
        test_real_motion_single_cabinet_only_ ? "true" : "false",
        test_real_motion_stop_after_scan_ ? "true" : "false",
        test_real_final_recognition_wait_sec_);
      RCLCPP_INFO(
        get_logger(),
        "测试侧排流程配置: enabled=%s name=%s first_gap=%s first_seq=%s "
        "second_gap=%s second_seq=%s transfer_target=%d exit_distance=%.2f exit_speed=%.3f "
        "exit_timeout=%.2f exit_turn=%s exit_turn_angular=%.3f exit_turn_tolerance=%.3f "
        "exit_turn_timeout=%.2f grid_motion=%s grid_spacing=%.2f grid_speed=%.3f grid_timeout=%.2f",
        test_real_side_row_enabled_ ? "true" : "false",
        test_real_side_row_name_.c_str(),
        test_real_side_row_first_gap_.c_str(),
        cabinet_unit_to_string(test_real_side_row_first_gap_scan_sequence_).c_str(),
        test_real_side_row_second_gap_.c_str(),
        cabinet_unit_to_string(test_real_side_row_second_gap_scan_sequence_).c_str(),
        test_real_side_row_corridor_transfer_target_cabinet_,
        test_real_exit_distance_m_,
        test_real_exit_speed_,
        test_real_exit_timeout_sec_,
        test_real_exit_turn_enabled_ ? "true" : "false",
        test_real_exit_turn_angular_speed_,
        test_real_exit_turn_yaw_tolerance_rad_,
        test_real_exit_turn_timeout_sec_,
        test_real_grid_motion_enabled_ ? "true" : "false",
        test_real_grid_spacing_m_,
        test_real_grid_move_speed_,
        test_real_grid_move_timeout_sec_);
      RCLCPP_INFO(
        get_logger(),
        "整体盘库测试配置: enabled=%s sequence=%s left_route=%s right_route=%s "
        "return_between_sides=%s return_after_done=%s recognize_during_nav=%s "
        "same_side_search=%s speed=%.3f timeout=%.2f pose_hold=%s fixed_y=%.3f "
        "fixed_yaw=%.4f yaw_kp=%.3f yaw_deadband=%.3f y_kp=%.3f y_deadband=%.3f "
        "y_sign=%.1f max_angular=%.3f final_recognition_wait=%.2f "
        "recognition_fallback=%s fallback_speed=%.3f fallback_wait=%.2f fallback_timeout=%.2f "
        "fallback_sequence=%s post_gap_advance=%s distance=%.2f speed=%.3f timeout=%.2f",
        overall_test_enabled_ ? "true" : "false",
        cabinet_unit_to_string(overall_test_sequence_).c_str(),
        overall_test_left_route_.c_str(),
        overall_test_right_route_.c_str(),
        overall_test_return_home_between_sides_ ? "true" : "false",
        overall_test_return_home_after_done_ ? "true" : "false",
        overall_test_recognize_during_nav_ ? "true" : "false",
        overall_test_same_side_next_search_enabled_ ? "true" : "false",
        overall_test_same_side_search_speed_,
        overall_test_same_side_search_timeout_sec_,
        overall_test_same_side_pose_hold_enabled_ ? "true" : "false",
        overall_test_same_side_fixed_y_m_,
        overall_test_same_side_fixed_yaw_rad_,
        overall_test_same_side_yaw_kp_,
        overall_test_same_side_yaw_deadband_rad_,
        overall_test_same_side_y_kp_,
        overall_test_same_side_y_deadband_m_,
        overall_test_same_side_y_correction_sign_,
        overall_test_same_side_max_angular_,
        overall_test_final_recognition_wait_sec_,
        overall_test_recognition_fallback_enabled_ ? "true" : "false",
        overall_test_recognition_fallback_speed_,
        overall_test_recognition_fallback_wait_sec_,
        overall_test_recognition_fallback_timeout_sec_,
        double_vector_to_string(overall_test_recognition_fallback_sequence_).c_str(),
        post_gap_detect_advance_enabled_ ? "true" : "false",
        post_gap_detect_advance_distance_m_,
        post_gap_detect_advance_speed_,
        post_gap_detect_advance_timeout_sec_);
      return true;
    } catch (const std::exception & ex) {
      reason = "解析 test_gap_scan_params.yaml 失败: " + std::string(ex.what());
      return false;
    }
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
            if (!wheeltec_inventory_system::safe_to_int(cabinet_text, cabinet_id)) {
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
          if (!wheeltec_inventory_system::safe_to_int(cabinet_text, cabinet_id)) {
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

  void set_state(State s, const std::string & detail)
  {
    const bool gap_enabled =
      s == State::SEARCH_GAP ||
      s == State::WAITING_GAP ||
      s == State::TEST_REAL_WAITING_GAP ||
      s == State::OVERALL_TEST_SEARCH_GAP;
    const bool recognizer_enabled =
      s == State::IDLE ||
      s == State::NAV_ROUTE ||
      s == State::TARGET_TRACKING ||
      s == State::TEST_REAL_NAV_TO_TARGET ||
      s == State::TEST_REAL_TARGET_TRACKING ||
      s == State::TEST_REAL_FINAL_RECOGNITION_WAIT ||
      (s == State::OVERALL_TEST_NAV_TO_OBSERVE && overall_test_recognize_during_nav_) ||
      s == State::OVERALL_TEST_POST_ROUTE_RECOGNITION_WAIT ||
      s == State::OVERALL_TEST_RECOGNITION_FALLBACK ||
      s == State::OVERALL_TEST_SAME_SIDE_NEXT_SEARCH;
    const bool distance_enabled =
      s == State::TARGET_TRACKING ||
      s == State::TEST_REAL_TARGET_TRACKING ||
      s == State::OVERALL_TEST_TARGET_DISTANCE_ALIGN;

    set_gap_detector_enabled(gap_enabled);
    set_distance_estimator_enabled(distance_enabled);
    set_recognizer_topic_enabled(recognizer_enabled);
    state_ = s;
    publish_state_text(state_to_string(state_));
    publish_log("[" + state_to_string(state_) + "] " + detail);
  }

  ReturnTargetMode get_return_target_mode() const
  {
    if (return_target_mode_ == "charge" || return_target_mode_ == "charging") {
      return ReturnTargetMode::CHARGE;
    }
    return ReturnTargetMode::START;
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

  bool parse_target_metadata(
    const std::string & code,
    TargetMetadata & target,
    std::string & reason) const
  {
    target = TargetMetadata{};
    reason.clear();

    const std::string cleaned = wheeltec_inventory_system::trim(code);
    if (cleaned.empty()) {
      reason = "目标编号为空";
      return false;
    }

    const auto items = wheeltec_inventory_system::split(cleaned, '-');
    if (items.size() == 1U) {
      if (!wheeltec_inventory_system::safe_to_int(items[0], target.cabinet_id)) {
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
      if (!wheeltec_inventory_system::safe_to_int(items[0], warehouse_id) ||
        !wheeltec_inventory_system::safe_to_int(items[1], target.cabinet_id) ||
        !wheeltec_inventory_system::safe_to_int(items[2], target.level_index))
      {
        reason = "目标编号格式非法: " + code;
        return false;
      }

      if (items.size() == 4U) {
        if (!wheeltec_inventory_system::safe_to_int(items[3], target.depth_index)) {
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

  void switch_test_real_to_waiting_gap_after_recognition(int rec_id)
  {
    const bool in_side_row_transfer =
      test_real_side_row_active_ &&
      test_real_side_row_phase_ == TestRealSideRowPhase::CORRIDOR_TRANSFER;
    if (state_ == State::TEST_REAL_FINAL_RECOGNITION_WAIT) {
      publish_test_real_log(
        "final recognition success cabinet=" + std::to_string(rec_id) +
        (in_side_row_transfer ?
        ", switch to TEST_REAL_PREPARE_NEXT_GAP" : ", switch to TEST_REAL_WAITING_GAP"));
    } else {
      publish_test_real_log(
        "recognized target cabinet=" + std::to_string(rec_id) +
        (in_side_row_transfer ?
        ", switch to TEST_REAL_PREPARE_NEXT_GAP" : ", switch to TEST_REAL_WAITING_GAP"));
    }

    cancel_nav2_route_goal("测试真实运动稳定识别到目标货柜");
    nav2_route_stop_hold_active_ = false;
    nav2_route_cancel_requested_ = false;
    target_found_pending_ = false;
    set_corridor_mode(false, false);
    publish_stop();
    has_distance_ = false;
    tracking_stable_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    if (in_side_row_transfer) {
      publish_test_real_log(
        "[corridor_transfer] recognized target cabinet=" + std::to_string(rec_id));
      publish_test_real_log(
        "[corridor_transfer] switch to next gap=" + test_real_side_row_second_gap_);
      set_test_real_state(
        State::TEST_REAL_PREPARE_NEXT_GAP,
        "走廊转移已稳定识别目标柜号，准备下一个缝隙");
      return;
    }
    begin_search_gap_flow();
  }

  void begin_test_real_final_recognition_wait()
  {
    cancel_nav2_route_goal("测试真实运动巡航路线已走完，进入末端识别等待");
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
    test_real_final_recognition_wait_start_ = this->now();
    publish_test_real_log(
      "route finished, wait final recognition target=" +
      std::to_string(current_target_cabinet_) +
      " timeout=" + format_seconds(test_real_final_recognition_wait_sec_));
    set_test_real_state(
      State::TEST_REAL_FINAL_RECOGNITION_WAIT,
      "route finished, wait final recognition target=" +
      std::to_string(current_target_cabinet_) +
      " timeout=" + format_seconds(test_real_final_recognition_wait_sec_) + " sec");
  }

  bool handle_test_real_recognition(int rec_id)
  {
    if (!is_test_real_recognition_state(state_)) {
      return false;
    }

    const bool matched = rec_id == current_target_cabinet_;
    if (!matched) {
      reset_target_recognition_stability();
      RCLCPP_INFO(
        get_logger(),
        "[mission_manager][test_real][recognition] target_mismatch recognized=%d target=%d",
        rec_id,
        current_target_cabinet_);
      return true;
    }

    const bool stable_ready = update_target_recognition_stability(rec_id);
    const int required_count = std::max(1, target_recognition_stable_frames_);
    RCLCPP_INFO(
      get_logger(),
      "[mission_manager][test_real][recognition] state=%s target=%d recognized=%d "
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
      switch_test_real_to_waiting_gap_after_recognition(rec_id);
    }
    return true;
  }

  void recognized_callback(
    const wheeltec_inventory_system::msg::RecognizedNumber::SharedPtr msg)
  {
    latest_recognition_ = msg;
    latest_recognition_time_ = this->now();
    if (test_real_motion_active_) {
      RCLCPP_INFO(
        get_logger(),
        "[mission_manager][test_real][recognition_rx] state=%s target=%d raw_number=%s "
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
        is_test_real_recognition_state(state_) ||
        is_overall_test_recognition_state(state_))
      {
        reset_target_recognition_stability();
      }
      return;
    }

    int rec_id = -1;
    if (!wheeltec_inventory_system::safe_to_int(msg->number, rec_id)) {
      return;
    }

    if (handle_overall_test_recognition(rec_id)) {
      return;
    }

    if (handle_test_real_recognition(rec_id)) {
      return;
    }

    if (rec_id != current_target_cabinet_) {
      if (is_nav_route_like_state(state_)) {
        reset_target_recognition_stability();
      }
      return;
    }

    // 返航阶段不再做目标识别触发，避免“返航途中重新识别目标”。
    if (state_ == State::RETURNING) {
      return;
    }

    last_target_seen_time_ = this->now();
    target_visible_ = true;

    if (is_nav_route_like_state(state_)) {
      if (update_target_recognition_stability(rec_id)) {
        if (test_real_motion_active_ && !test_real_target_recognized_logged_) {
          publish_test_real_log("recognized target cabinet=" + std::to_string(rec_id));
          test_real_target_recognized_logged_ = true;
        }
        target_found_pending_ = true;
        begin_target_found_stop_hold();
      }
      return;
    }

    if (state_ == State::CORRIDOR_NAV) {
      set_corridor_mode(false, false);
      publish_stop();
      has_distance_ = false;
      tracking_stable_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      set_state(
        State::TARGET_TRACKING,
        "识别到目标柜 target=" + std::to_string(current_target_cabinet_) + "，进入跟踪");
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

    if (use_scan_safety_) {
      if (!latest_scan_ || (this->now() - latest_scan_stamp_).seconds() > max_scan_age_sec_) {
        eval.blocked = true;
        eval.speed_scale = 0.0;
        eval.block_reason = "NO_FRESH_SCAN";
        return eval;
      }

      eval.front_min_dist = min_scan_range_in_sector(
        *latest_scan_, enter_front_sector_start_deg_, enter_front_sector_end_deg_);
      if (current_entry_side_ == "right") {
        eval.front_side_min_dist = min_scan_range_in_sector(
          *latest_scan_, enter_front_right_sector_start_deg_, enter_front_right_sector_end_deg_);
        eval.side_min_dist = min_scan_range_in_sector(
          *latest_scan_, enter_right_side_sector_start_deg_, enter_right_side_sector_end_deg_);
      } else {
        eval.front_side_min_dist = min_scan_range_in_sector(
          *latest_scan_, enter_front_left_sector_start_deg_, enter_front_left_sector_end_deg_);
        eval.side_min_dist = min_scan_range_in_sector(
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
          eval.block_reason = "BLOCKED_FRONT";
        } else if (eval.front_side_min_dist < enter_stop_distance_) {
          eval.block_reason = current_entry_side_ == "right" ? "BLOCKED_FRONT_RIGHT" : "BLOCKED_FRONT_LEFT";
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
    const bool recognizer_during_nav =
      !(overall_test_active_ && nav_state == State::OVERALL_TEST_NAV_TO_OBSERVE &&
      !overall_test_recognize_during_nav_);
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
      if (overall_test_active_) {
        fail_overall_test("启动巡航路线失败: " + fail_reason);
      } else if (test_real_motion_active_) {
        fail_test_real_motion("启动巡航路线失败: " + fail_reason);
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

    if (test_real_motion_active_) {
      fail_test_real_motion(reason);
      return;
    }

    if (overall_test_active_) {
      fail_overall_test(reason);
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
        if (overall_test_active_) {
          publish_overall_test_log(
            "recognized during nav target=" + std::to_string(current_target_cabinet_) +
            ", stop hold finished");
          begin_overall_target_distance_align_after_recognition(
            current_target_cabinet_, "nav recognition stop hold finished");
        } else if (test_real_motion_active_) {
          publish_test_real_log("enter real target tracking cabinet=" + std::to_string(current_target_cabinet_));
          set_test_real_state(State::TEST_REAL_TARGET_TRACKING, "停车完成，进入目标跟踪");
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
      if (overall_test_active_) {
        begin_overall_test_post_route_recognition_wait();
        return;
      }
      if (test_real_motion_active_) {
        begin_test_real_final_recognition_wait();
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
    ReturnTargetMode target_mode = get_return_target_mode();
    if (mode == ReturnMode::SEARCH_TARGET) {
      target_mode = ReturnTargetMode::START;
    }

    if (target_mode == ReturnTargetMode::CHARGE) {
      target_pose.x = charge_pose_x_;
      target_pose.y = charge_pose_y_;
      target_pose.yaw = charge_pose_yaw_;
      target_pose.frame_id = sanitize_frame_id(charge_pose_frame_id_);
      target_pose.valid = true;
      mode_text = "充电位";
      return true;
    }

    if (!mission_start_pose_.valid) {
      mode_text = "起始点无效";
      return false;
    }
    if (use_nav2_return_ && mission_start_pose_nav2_.valid) {
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
    if (!use_nav2_return_) {
      fail_reason = "参数关闭 Nav2 返航";
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
    return_using_nav2_ = true;
    return true;
  }

  bool begin_fallback_return(
    ReturnMode mode, const Pose2D & resolved_target, const std::string & reason)
  {
    (void)reason;
    cancel_nav2_return_goal("切换到兜底返航");
    return_using_nav2_ = false;
    nav2_result_ready_ = false;
    nav2_result_text_.clear();

    fallback_phase_ = FallbackPhase::ROTATING;
    fallback_drive_mode_ = FallbackDriveMode::NONE;
    fallback_rotate_stable_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    request_recognizer_enable(false);

    if (!latest_odom_ || !has_yaw_) {
      if (overall_test_active_) {
        fail_overall_test("返航失败：无可用里程计/航向");
        return false;
      }
      mission_active_ = false;
      set_state(State::ERROR, "返航失败：无可用里程计/航向");
      return false;
    }

    if (mode == ReturnMode::SEARCH_TARGET || get_return_target_mode() == ReturnTargetMode::START) {
      fallback_target_yaw_ = normalize_angle(latest_yaw_ + M_PI);
      fallback_target_distance_ = std::max(0.20, odom_cumulative_distance_ - mission_start_distance_);
      fallback_drive_mode_ = FallbackDriveMode::CORRIDOR_DISTANCE;
      reset_segment_distance();
      publish_log("兜底返航：掉头180度后按走行里程回退");
      return true;
    }

    const Pose2D current = current_pose_2d();
    if (!current.valid) {
      if (overall_test_active_) {
        fail_overall_test("返航失败：当前位姿不可用");
        return false;
      }
      mission_active_ = false;
      set_state(State::ERROR, "返航失败：当前位姿不可用");
      return false;
    }

    if (!resolved_target.frame_id.empty() && !current.frame_id.empty() &&
      resolved_target.frame_id != current.frame_id)
    {
      // 坐标系不一致时，兜底策略退化为“掉头+里程返回”，确保仍可回撤。
      fallback_target_yaw_ = normalize_angle(latest_yaw_ + M_PI);
      fallback_target_distance_ = std::max(0.20, odom_cumulative_distance_ - mission_start_distance_);
      fallback_drive_mode_ = FallbackDriveMode::CORRIDOR_DISTANCE;
      reset_segment_distance();
      publish_log("兜底返航：坐标系不一致，退化为掉头+里程返回");
      return true;
    }

    const double dx = resolved_target.x - current.x;
    const double dy = resolved_target.y - current.y;
    fallback_target_x_ = resolved_target.x;
    fallback_target_y_ = resolved_target.y;
    fallback_target_distance_ = std::hypot(dx, dy);
    fallback_target_yaw_ = fallback_target_distance_ > fallback_goal_tolerance_m_ ?
      std::atan2(dy, dx) : current.yaw;
    fallback_drive_mode_ = FallbackDriveMode::DIRECT_POSE;
    publish_log("兜底返航：按里程计坐标闭环返回目标点");
    return true;
  }

  void finalize_return_success()
  {
    set_corridor_mode(false, false);
    publish_stop();
    return_using_nav2_ = false;
    fallback_phase_ = FallbackPhase::IDLE;
    fallback_drive_mode_ = FallbackDriveMode::NONE;
    reset_wait_gap_runtime();
    reset_entry_gap_runtime();
    publish_gap_context();

    if (overall_test_active_ && overall_test_return_reason_ != OverallTestReturnReason::NONE) {
      on_return_home_finished_continue_overall_test();
      return;
    }

    if (return_mode_ == ReturnMode::CANCEL_HOME) {
      mission_active_ = false;
      cancel_requested_ = false;
      return_mode_ = ReturnMode::NONE;
      set_state(State::IDLE, "取消任务后已返回目标位置");
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

  void run_fallback_return()
  {
    if (fallback_phase_ == FallbackPhase::IDLE) {
      return;
    }

    if (!latest_odom_ || !has_yaw_) {
      publish_stop();
      return;
    }

    if (fallback_phase_ == FallbackPhase::ROTATING) {
      const double yaw_error = normalize_angle(fallback_target_yaw_ - latest_yaw_);
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = 0.0;
      cmd.angular.z = std::clamp(
        fallback_rotate_kp_ * yaw_error,
        -std::abs(fallback_rotate_max_angular_),
        std::abs(fallback_rotate_max_angular_));
      cmd_pub_->publish(cmd);

      if (std::abs(yaw_error) <= fallback_rotate_tolerance_rad_) {
        if (fallback_rotate_stable_start_.nanoseconds() == 0) {
          fallback_rotate_stable_start_ = this->now();
        }
        if ((this->now() - fallback_rotate_stable_start_).seconds() >= fallback_rotate_stable_time_sec_) {
          publish_stop();
          fallback_phase_ = FallbackPhase::DRIVING;
          if (fallback_drive_mode_ == FallbackDriveMode::CORRIDOR_DISTANCE) {
            reset_segment_distance();
            set_corridor_mode(true, false);
          }
        }
      } else {
        fallback_rotate_stable_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      }
      return;
    }

    if (fallback_phase_ == FallbackPhase::DRIVING) {
      if (fallback_drive_mode_ == FallbackDriveMode::CORRIDOR_DISTANCE) {
        if (segment_distance() < fallback_target_distance_) {
          return;
        }
        finalize_return_success();
        return;
      }

      if (fallback_drive_mode_ == FallbackDriveMode::DIRECT_POSE) {
        const Pose2D current = current_pose_2d();
        const double dx = fallback_target_x_ - current.x;
        const double dy = fallback_target_y_ - current.y;
        const double dist = std::hypot(dx, dy);
        if (dist <= fallback_goal_tolerance_m_) {
          finalize_return_success();
          return;
        }

        const double heading_ref = std::atan2(dy, dx);
        const double heading_error = normalize_angle(heading_ref - current.yaw);
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = std::abs(fallback_drive_speed_);
        cmd.angular.z = std::clamp(
          fallback_heading_kp_ * heading_error,
          -std::abs(fallback_drive_max_angular_),
          std::abs(fallback_drive_max_angular_));
        cmd_pub_->publish(cmd);
      }
    }
  }

  void switch_to_returning(ReturnMode mode, const std::string & reason)
  {
    cancel_nav2_route_goal("切换到返航流程");
    set_corridor_mode(false, false);
    publish_stop();
    request_recognizer_enable(false);

    return_mode_ = mode;
    return_using_nav2_ = false;
    nav2_return_in_progress_ = false;
    nav2_result_ready_ = false;
    fallback_phase_ = FallbackPhase::IDLE;
    fallback_drive_mode_ = FallbackDriveMode::NONE;
    fallback_rotate_stable_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    reset_wait_gap_runtime();
    reset_entry_gap_runtime();
    publish_gap_context();

    Pose2D target_pose;
    std::string target_mode_text;
    if (!resolve_return_pose(mode, target_pose, target_mode_text)) {
      if (overall_test_active_) {
        fail_overall_test("无法确定返航目标：" + target_mode_text);
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

    const bool allow_nav2 = use_nav2_return_ && (mode != ReturnMode::SEARCH_TARGET || nav2_enable_for_search_return_);
    bool nav2_started = false;
    bool fallback_started = false;
    std::string nav2_fail_reason;
    if (allow_nav2) {
      nav2_started = begin_nav2_return(target_pose, nav2_fail_reason);
    }
    if (!nav2_started) {
      fallback_started = begin_fallback_return(mode, target_pose, reason);
      if (!fallback_started) {
        return;
      }
    }

    std::string detail = reason + "，返航目标=" + target_mode_text;
    if (nav2_started) {
      detail += "，方式=Nav2";
    } else {
      detail += "，方式=兜底";
      if (!nav2_fail_reason.empty()) {
        detail += "（" + nav2_fail_reason + "）";
      }
    }
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
      std::string nav_route_fail_reason;
      if (!begin_nav_route_for_current_target("切换下一个目标，开始Nav2巡航路线识别", nav_route_fail_reason)) {
        set_state(State::ERROR, nav_route_fail_reason);
      }
      return;
    }

    if (success) {
      if (mission_return_home_on_finish_) {
        request_recognizer_enable(false);
        switch_to_returning(ReturnMode::FINISH_HOME, "全部目标完成，开始返回起点");
      } else {
        mission_active_ = false;
        request_recognizer_enable(false);
        set_corridor_mode(false, false);
        publish_stop();
        set_state(State::DONE, "全部目标处理完成");
      }
    } else {
      mission_active_ = false;
      request_recognizer_enable(false);
      set_corridor_mode(false, false);
      publish_stop();
      set_state(State::ERROR, "存在失败目标且无后续目标");
    }
  }

  void set_test_state(State s, const std::string & detail)
  {
    test_state_enter_time_ = this->now();
    state_ = s;
    publish_state_text(state_to_string(state_));
    publish_log("[" + state_to_string(state_) + "] [mission_manager][test_scan] " + detail);
  }

  void set_test_real_state(State s, const std::string & detail)
  {
    test_state_enter_time_ = this->now();
    set_state(s, "[mission_manager][test_real] " + detail);
  }

  bool find_configured_test_cabinets(
    const std::string & gap_id,
    std::vector<int> & cabinets) const
  {
    cabinets.clear();
    const auto plan_it = test_inventory_plan_by_gap_id_.find(gap_id);
    if (plan_it != test_inventory_plan_by_gap_id_.end()) {
      cabinets = plan_it->second;
      return true;
    }

    const auto map_it = test_gap_scan_map_cabinets_by_gap_id_.find(gap_id);
    if (map_it != test_gap_scan_map_cabinets_by_gap_id_.end()) {
      cabinets = map_it->second;
      return true;
    }

    return false;
  }

  bool validate_test_gap_scan_plan(const TestGapScanPlan & plan, std::string & reason) const
  {
    reason.clear();
    if (wheeltec_inventory_system::trim(plan.gap_id).empty()) {
      reason = "测试 gap_id 为空";
      return false;
    }
    if (plan.scan_cabinets.empty()) {
      reason = "测试 scan_cabinets 为空: gap=" + plan.gap_id;
      return false;
    }
    for (const auto cabinet_id : plan.scan_cabinets) {
      if (cabinet_id <= 0) {
        reason =
          "测试柜号必须大于 0: gap=" + plan.gap_id +
          " cabinet=" + std::to_string(cabinet_id);
        return false;
      }
    }
    return true;
  }

  void reset_test_real_side_row_context()
  {
    test_real_side_row_active_ = false;
    test_real_side_row_full_sequence_ = false;
    test_real_side_row_requested_sequence_.clear();
    test_real_side_row_phase_ = TestRealSideRowPhase::NONE;
    test_real_after_exit_action_ = TestRealAfterExitAction::NONE;
    test_real_active_gap_id_.clear();
    test_real_current_scan_cabinet_ = -1;
    test_real_adjusted_scan_cabinet_ = -1;
    test_real_next_gap_target_cabinet_ = -1;
    test_real_last_entering_straight_distance_ = 0.0;
    test_real_exit_target_distance_ = test_real_exit_distance_m_;
    test_real_exit_effective_timeout_sec_ = test_real_exit_timeout_sec_;
    test_real_exit_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    test_real_exit_phase_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    test_real_exit_phase_ = TestRealExitPhase::STRAIGHT_REVERSE;
    reset_test_real_scan_runtime();
  }

  void reset_overall_test_context()
  {
    overall_test_active_ = false;
    overall_test_index_ = 0;
    overall_test_current_target_ = -1;
    overall_test_next_target_ = -1;
    overall_test_current_side_.clear();
    overall_test_current_route_.clear();
    overall_test_return_reason_ = OverallTestReturnReason::NONE;
    overall_test_waiting_return_home_for_side_switch_ = false;
    overall_test_waiting_return_home_for_done_ = false;
    overall_test_same_side_search_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    overall_test_final_recognition_wait_start_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
    overall_test_recognition_fallback_start_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
    overall_test_recognition_fallback_phase_start_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
    overall_test_recognition_fallback_phase_ = OverallRecognitionFallbackPhase::IDLE;
    overall_test_recognition_fallback_index_ = 0;
    overall_test_post_gap_advance_start_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }

  std::string overall_return_reason_to_string(OverallTestReturnReason reason) const
  {
    switch (reason) {
      case OverallTestReturnReason::BETWEEN_SIDES:
        return "BETWEEN_SIDES";
      case OverallTestReturnReason::FINAL_DONE:
        return "FINAL_DONE";
      case OverallTestReturnReason::NONE:
      default:
        return "NONE";
    }
  }

  bool apply_overall_test_route_overrides(std::string & reason)
  {
    reason.clear();
    if (!overall_test_left_route_.empty()) {
      if (route_configs_.find(overall_test_left_route_) == route_configs_.end()) {
        reason = "overall_test_left_route 不存在: " + overall_test_left_route_;
        return false;
      }
      side_route_map_["left"] = overall_test_left_route_;
    }
    if (!overall_test_right_route_.empty()) {
      if (route_configs_.find(overall_test_right_route_) == route_configs_.end()) {
        reason = "overall_test_right_route 不存在: " + overall_test_right_route_;
        return false;
      }
      side_route_map_["right"] = overall_test_right_route_;
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

  bool validate_overall_test_sequence(
    const std::vector<int> & sequence,
    std::string & reason) const
  {
    reason.clear();
    if (sequence.empty()) {
      reason = "overall_test_sequence 为空";
      return false;
    }
    for (const auto cabinet_id : sequence) {
      if (cabinet_id <= 0) {
        reason = "overall_test_sequence 中货柜号必须大于0: " + std::to_string(cabinet_id);
        return false;
      }
    }
    if (test_scan_layers_ <= 0 || test_scan_depth_count_ <= 0) {
      reason =
        "scan_layers/scan_depth_count 必须为正数: layers=" +
        std::to_string(test_scan_layers_) +
        " depth_count=" + std::to_string(test_scan_depth_count_);
      return false;
    }
    return true;
  }

  std::vector<int> overall_test_sequence_from_request(
    const wheeltec_inventory_system::srv::StartTestGapScan::Request & request) const
  {
    std::vector<int> sequence;
    for (const auto cabinet_id : request.scan_cabinets) {
      sequence.push_back(static_cast<int>(cabinet_id));
    }
    if (!sequence.empty()) {
      return sequence;
    }
    return overall_test_sequence_;
  }

  bool should_start_overall_test(
    const wheeltec_inventory_system::srv::StartTestGapScan::Request & request) const
  {
    if (!overall_test_enabled_) {
      return false;
    }
    const bool gap_empty = wheeltec_inventory_system::trim(request.gap_id).empty();
    return (request.run_all_configured && gap_empty) || (!request.scan_cabinets.empty() && gap_empty);
  }

  void record_overall_test_start_pose()
  {
    mission_start_distance_ = odom_cumulative_distance_;
    reset_segment_distance();
    mission_start_pose_ = current_pose_2d();
    mission_start_pose_nav2_ = Pose2D{};
    if (!mission_start_pose_.valid) {
      RCLCPP_WARN(get_logger(), "[OVERALL_TEST] 启动时未获取到有效里程计位姿，返航将无法使用起点模式");
      return;
    }

    publish_overall_test_log(
      "record start pose x=" + std::to_string(mission_start_pose_.x) +
      " y=" + std::to_string(mission_start_pose_.y) +
      " yaw=" + std::to_string(mission_start_pose_.yaw) +
      " frame=" + mission_start_pose_.frame_id);

    std::string tf_error;
    if (transform_pose_2d(mission_start_pose_, nav2_goal_frame_, mission_start_pose_nav2_, tf_error)) {
      publish_overall_test_log(
        "record Nav2 start pose x=" + std::to_string(mission_start_pose_nav2_.x) +
        " y=" + std::to_string(mission_start_pose_nav2_.y) +
        " yaw=" + std::to_string(mission_start_pose_nav2_.yaw) +
        " frame=" + mission_start_pose_nav2_.frame_id);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "[OVERALL_TEST] 记录Nav2全局起点失败，将回退到里程计起点：%s",
        tf_error.c_str());
    }
  }

  bool prepare_overall_test_target(int cabinet_id, std::string & reason)
  {
    reason.clear();
    if (!routes_loaded_) {
      routes_loaded_ = load_route_config(reason);
      if (!routes_loaded_) {
        return false;
      }
    }
    if (!apply_overall_test_route_overrides(reason)) {
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

    overall_test_current_target_ = cabinet_id;
    overall_test_current_side_ = current_target_side_;
    overall_test_current_route_ = current_route_name_;
    log_current_target_entry_plan();
    publish_entry_side();
    request_recognizer_enable(false);
    set_recognizer_topic_enabled(false, true);
    set_distance_estimator_enabled(false, true);
    set_gap_detector_enabled(false);
    publish_overall_test_log(
      "current_target=" + std::to_string(overall_test_current_target_) +
      " index=" + std::to_string(overall_test_index_) +
      "/" + std::to_string(overall_test_sequence_.size()) +
      " side=" + overall_test_current_side_ +
      " route=" + overall_test_current_route_ +
      " entry_side=" + current_entry_side_ +
      " search_direction=" + search_direction_to_string(current_gap_plan_.search_direction));
    return true;
  }

  bool start_overall_test_target_route(const std::string & context)
  {
    mission_active_ = true;
    cancel_requested_ = false;
    mission_return_home_on_finish_ = false;
    target_visible_ = false;
    has_distance_ = false;
    latest_distance_ = 0.0;
    return_mode_ = ReturnMode::NONE;
    return_target_distance_ = 0.0;
    return_using_nav2_ = false;
    nav2_return_in_progress_ = false;
    nav2_result_ready_ = false;
    fallback_phase_ = FallbackPhase::IDLE;
    fallback_drive_mode_ = FallbackDriveMode::NONE;
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
    publish_overall_test_log(
      "nav observe stage: recognizer=" +
      std::string(overall_test_recognize_during_nav_ ? "on" : "off") +
      " distance=off gap=off target=" + std::to_string(current_target_cabinet_));

    std::string fail_reason;
    if (!begin_nav_route_for_current_target(
        "[OVERALL_TEST] " + context +
        " current_target=" + std::to_string(current_target_cabinet_) +
        " side=" + current_target_side_ +
        " route=" + current_route_name_,
        fail_reason,
        State::OVERALL_TEST_NAV_TO_OBSERVE))
    {
      fail_overall_test("启动目标侧路径失败: " + fail_reason);
      return false;
    }
    return true;
  }

  bool start_overall_test_sequence(
    const std::vector<int> & sequence,
    std::string & reason)
  {
    reason.clear();
    if (!validate_overall_test_sequence(sequence, reason)) {
      return false;
    }

    overall_test_sequence_ = sequence;
    overall_test_active_ = true;
    overall_test_index_ = 0;
    overall_test_current_target_ = -1;
    overall_test_next_target_ =
      overall_test_sequence_.size() > 1U ? overall_test_sequence_[1] : -1;
    overall_test_return_reason_ = OverallTestReturnReason::NONE;
    overall_test_waiting_return_home_for_side_switch_ = false;
    overall_test_waiting_return_home_for_done_ = false;
    overall_test_same_side_search_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    overall_test_final_recognition_wait_start_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
    overall_test_recognition_fallback_start_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
    overall_test_recognition_fallback_phase_start_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
    overall_test_recognition_fallback_phase_ = OverallRecognitionFallbackPhase::IDLE;
    overall_test_recognition_fallback_index_ = 0;
    overall_test_post_gap_advance_start_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());

    test_gap_scan_queue_.clear();
    test_current_gap_index_ = 0;
    test_gap_scan_error_reason_.clear();
    test_gap_scan_active_ = true;
    test_real_motion_active_ = false;
    test_real_gap_searching_ = false;
    test_real_target_recognized_logged_ = false;
    test_real_close_requested_ = false;
    reset_test_real_side_row_context();
    reset_test_real_scan_runtime();
    record_overall_test_start_pose();

    publish_overall_test_log(
      "START sequence=" + cabinet_unit_to_string(overall_test_sequence_));
    set_test_state(
      State::OVERALL_TEST_PREPARE_TARGET,
      "[OVERALL_TEST] prepare first target");
    request_recognizer_enable(false);
    set_recognizer_topic_enabled(false, true);
    set_distance_estimator_enabled(false, true);
    set_gap_detector_enabled(false);

    if (!prepare_overall_test_target(overall_test_sequence_[overall_test_index_], reason)) {
      reset_overall_test_context();
      test_gap_scan_active_ = false;
      return false;
    }
    return start_overall_test_target_route("start target route");
  }

  void fail_overall_test(const std::string & reason)
  {
    publish_overall_test_log("FAILED: " + reason);
    cancel_nav2_route_goal("整体盘库测试失败");
    cancel_nav2_return_goal("整体盘库测试失败");
    set_corridor_mode(false, false);
    publish_stop();
    request_recognizer_enable(false);
    set_recognizer_topic_enabled(false, true);
    set_distance_estimator_enabled(false, true);
    set_gap_detector_enabled(false);
    reset_test_real_scan_runtime();
    test_gap_scan_error_reason_ = reason;
    set_test_state(State::TEST_ERROR, "[OVERALL_TEST] " + reason);
  }

  void begin_overall_test_post_route_recognition_wait()
  {
    cancel_nav2_route_goal("整体盘库测试巡航路线已走完，进入停车识别等待");
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
    overall_test_final_recognition_wait_start_ = this->now();
    publish_overall_test_log(
      "route finished, start recognition wait target=" +
      std::to_string(current_target_cabinet_) +
      " timeout=" + format_seconds(overall_test_final_recognition_wait_sec_));
    set_state(
      State::OVERALL_TEST_POST_ROUTE_RECOGNITION_WAIT,
      "[OVERALL_TEST] route finished, start recognition wait target=" +
      std::to_string(current_target_cabinet_) +
      " timeout=" + format_seconds(overall_test_final_recognition_wait_sec_));
  }

  void begin_overall_target_distance_align_after_recognition(
    int rec_id,
    const std::string & context)
  {
    const bool from_post_route_wait =
      state_ == State::OVERALL_TEST_POST_ROUTE_RECOGNITION_WAIT;
    const bool from_same_side_search =
      state_ == State::OVERALL_TEST_SAME_SIDE_NEXT_SEARCH;
    cancel_nav2_route_goal("整体盘库测试稳定识别到目标货柜");
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
    overall_test_final_recognition_wait_start_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
    overall_test_recognition_fallback_start_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
    overall_test_recognition_fallback_phase_start_ =
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
    overall_test_recognition_fallback_phase_ = OverallRecognitionFallbackPhase::IDLE;
    tracking_stable_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    if (from_post_route_wait) {
      publish_overall_test_log(
        "post-route recognition wait success target=" + std::to_string(rec_id));
    }
    if (from_same_side_search) {
      publish_overall_test_log(
        "same-side next target recognized=" + std::to_string(rec_id) +
        ", recognizer=off, distance=on");
    }
    publish_overall_test_log(
      "recognized target=" + std::to_string(rec_id) +
      ", recognizer=off, distance=on");
    set_state(
      State::OVERALL_TEST_TARGET_DISTANCE_ALIGN,
      "[OVERALL_TEST] recognized target cabinet=" + std::to_string(rec_id) +
      " stable, enter target distance align: " + context);
  }

  bool handle_overall_test_recognition(int rec_id)
  {
    if (!is_overall_test_recognition_state(state_)) {
      return false;
    }

    const bool matched = rec_id == current_target_cabinet_;
    if (!matched) {
      reset_target_recognition_stability();
      RCLCPP_INFO(
        get_logger(),
        "[mission_manager][OVERALL_TEST][recognition] mismatch recognized=%d target=%d",
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
      "[mission_manager][OVERALL_TEST][recognition] state=%s target=%d recognized=%d "
      "stable_count=%d required=%d ready=%s",
      state_to_string(state_).c_str(),
      current_target_cabinet_,
      rec_id,
      target_recognition_stable_count_,
      required_count,
      stable_ready ? "true" : "false");

    if (stable_ready) {
      begin_overall_target_distance_align_after_recognition(
        rec_id, "recognition stable in " + state_to_string(state_));
    }
    return true;
  }

  void start_overall_same_side_next_search()
  {
    overall_test_same_side_search_start_ = this->now();
    reset_target_recognition_stability();
    target_visible_ = false;
    has_distance_ = false;
    latest_distance_ = 0.0;
    request_recognizer_enable(true);
    set_recognizer_topic_enabled(true, true);
    set_distance_estimator_enabled(false, true);
    set_gap_detector_enabled(false);
    set_corridor_mode(false, false);
    publish_overall_test_log(
      "same-side next search start target=" +
      std::to_string(overall_test_current_target_) +
      " recognizer=on distance=off gap=off speed=" +
      format_seconds(overall_test_same_side_search_speed_) +
      " timeout=" + format_seconds(overall_test_same_side_search_timeout_sec_));
    if (overall_test_same_side_pose_hold_enabled_) {
      publish_overall_test_log(
        "same-side pose hold enabled fixed_y=" +
        format_fixed(overall_test_same_side_fixed_y_m_, 3) +
        " fixed_yaw=" + format_fixed(overall_test_same_side_fixed_yaw_rad_, 4));
    }
    set_state(
      State::OVERALL_TEST_SAME_SIDE_NEXT_SEARCH,
      "[OVERALL_TEST] same-side next search target=" +
      std::to_string(overall_test_current_target_));
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

  void handle_overall_same_side_next_search_state()
  {
    request_recognizer_enable(true);
    set_recognizer_topic_enabled(true);
    set_distance_estimator_enabled(false);
    set_gap_detector_enabled(false);
    if (overall_test_same_side_search_start_.nanoseconds() == 0) {
      overall_test_same_side_search_start_ = this->now();
    }
    const double elapsed = (this->now() - overall_test_same_side_search_start_).seconds();
    const double timeout = std::max(0.1, overall_test_same_side_search_timeout_sec_);
    if (elapsed >= timeout) {
      publish_stop();
      fail_overall_test(
        "same side recognition search timeout target=" +
        std::to_string(current_target_cabinet_));
      return;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = overall_test_same_side_search_speed_;
    cmd.angular.z = 0.0;
    Pose2D pose;
    std::string pose_note;
    bool pose_hold_active = false;
    double y_error = 0.0;
    double yaw_error = 0.0;
    double yaw_cmd = 0.0;
    double y_cmd = 0.0;

    if (overall_test_same_side_pose_hold_enabled_) {
      if (current_same_side_pose_hold_pose(pose, pose_note)) {
        pose_hold_active = true;
        y_error = overall_test_same_side_fixed_y_m_ - pose.y;
        yaw_error = normalize_angle(overall_test_same_side_fixed_yaw_rad_ - pose.yaw);
        if (std::abs(yaw_error) >= std::abs(overall_test_same_side_yaw_deadband_rad_)) {
          yaw_cmd = overall_test_same_side_yaw_kp_ * yaw_error;
        }
        if (std::abs(y_error) >= std::abs(overall_test_same_side_y_deadband_m_)) {
          y_cmd = overall_test_same_side_y_kp_ * y_error;
        }
        const double max_angular =
          std::isfinite(overall_test_same_side_max_angular_) ?
          std::max(0.0, std::abs(overall_test_same_side_max_angular_)) : 0.15;
        cmd.angular.z = std::clamp(
          yaw_cmd + overall_test_same_side_y_correction_sign_ * y_cmd,
          -max_angular,
          max_angular);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "[OVERALL_TEST] same-side pose hold unavailable: %s, fallback to open-loop search",
          pose_note.c_str());
      }
    }
    cmd_pub_->publish(cmd);
    if (pose_hold_active) {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "[OVERALL_TEST] same_side_next_search target=%d speed=%.3f current_y=%.3f "
        "fixed_y=%.3f y_error=%.3f current_yaw=%.4f fixed_yaw=%.4f yaw_error=%.4f "
        "angular=%.3f elapsed=%.2f/%.2f pose=%s",
        current_target_cabinet_,
        cmd.linear.x,
        pose.y,
        overall_test_same_side_fixed_y_m_,
        y_error,
        pose.yaw,
        overall_test_same_side_fixed_yaw_rad_,
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
        "[OVERALL_TEST] same_side_next_search target=%d speed=%.3f angular=%.3f "
        "elapsed=%.2f/%.2f open_loop=true",
        current_target_cabinet_,
        cmd.linear.x,
        cmd.angular.z,
        elapsed,
        timeout);
    }
  }

  void begin_overall_recognition_fallback()
  {
    if (!overall_test_recognition_fallback_enabled_) {
      fail_overall_test(
        "post-route recognition wait timeout and recognition fallback disabled target=" +
        std::to_string(current_target_cabinet_));
      return;
    }
    if (overall_test_recognition_fallback_sequence_.empty()) {
      fail_overall_test(
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
    overall_test_recognition_fallback_start_ = this->now();
    overall_test_recognition_fallback_phase_start_ = this->now();
    overall_test_recognition_fallback_index_ = 0;
    overall_test_recognition_fallback_phase_ = OverallRecognitionFallbackPhase::MOVING;
    reset_segment_distance();
    publish_overall_test_log(
      "recognition fallback start sequence=" +
      double_vector_to_string(overall_test_recognition_fallback_sequence_));
    publish_overall_test_log(
      "recognition fallback step=1 move=" +
      format_seconds(overall_test_recognition_fallback_sequence_.front()));
    set_state(
      State::OVERALL_TEST_RECOGNITION_FALLBACK,
      "[OVERALL_TEST] recognition fallback target=" +
      std::to_string(current_target_cabinet_));
  }

  void handle_overall_test_post_route_recognition_wait_state()
  {
    publish_stop();
    request_recognizer_enable(true);
    set_recognizer_topic_enabled(true);
    set_distance_estimator_enabled(false);
    set_gap_detector_enabled(false);
    if (overall_test_final_recognition_wait_start_.nanoseconds() == 0) {
      overall_test_final_recognition_wait_start_ = this->now();
    }

    const double elapsed =
      (this->now() - overall_test_final_recognition_wait_start_).seconds();
    const double timeout = std::max(0.0, overall_test_final_recognition_wait_sec_);
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "[mission_manager][OVERALL_TEST] post-route recognition wait: target=%d elapsed=%.2f/%.2f",
      current_target_cabinet_,
      elapsed,
      timeout);

    if (elapsed >= timeout) {
      publish_overall_test_log(
        "post-route recognition wait timeout: target=" +
        std::to_string(current_target_cabinet_));
      begin_overall_recognition_fallback();
    }
  }

  void handle_overall_recognition_fallback_state()
  {
    request_recognizer_enable(true);
    set_recognizer_topic_enabled(true);
    set_distance_estimator_enabled(false);
    set_gap_detector_enabled(false);

    if (overall_test_recognition_fallback_start_.nanoseconds() == 0) {
      begin_overall_recognition_fallback();
      return;
    }

    const double elapsed_total =
      (this->now() - overall_test_recognition_fallback_start_).seconds();
    const double fallback_step_timeout =
      std::max(0.1, overall_test_recognition_fallback_timeout_sec_);

    if (overall_test_recognition_fallback_index_ >=
      overall_test_recognition_fallback_sequence_.size())
    {
      publish_stop();
      fail_overall_test(
        "recognition fallback sequence exhausted target=" +
        std::to_string(current_target_cabinet_));
      return;
    }

    const double step =
      overall_test_recognition_fallback_sequence_[overall_test_recognition_fallback_index_];
    switch (overall_test_recognition_fallback_phase_) {
      case OverallRecognitionFallbackPhase::MOVING: {
        const double target_distance = std::abs(step);
        const double speed = std::abs(overall_test_recognition_fallback_speed_);
        const double move_elapsed =
          (this->now() - overall_test_recognition_fallback_phase_start_).seconds();
        const double required_time = speed > 1e-4 ? target_distance / speed : fallback_step_timeout;
        const double move_timeout = std::max(fallback_step_timeout, required_time + 2.0);
        if (move_elapsed >= move_timeout) {
          publish_stop();
          fail_overall_test(
            "recognition fallback move timeout target=" +
            std::to_string(current_target_cabinet_) +
            " step=" + std::to_string(overall_test_recognition_fallback_index_ + 1U) +
            " move=" + format_seconds(step) +
            " elapsed=" + format_seconds(move_elapsed) +
            " timeout=" + format_seconds(move_timeout));
          return;
        }
        if (target_distance <= 1e-4 || segment_distance() >= target_distance) {
          publish_stop();
          overall_test_recognition_fallback_phase_ = OverallRecognitionFallbackPhase::WAITING;
          overall_test_recognition_fallback_phase_start_ = this->now();
          break;
        }

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = (step >= 0.0 ? 1.0 : -1.0) *
          std::abs(overall_test_recognition_fallback_speed_);
        cmd.angular.z = 0.0;
        cmd_pub_->publish(cmd);
        RCLCPP_INFO_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "[mission_manager][OVERALL_TEST] recognition_fallback moving step=%zu/%zu "
          "move=%.2f traveled=%.2f/%.2f elapsed=%.2f/%.2f",
          overall_test_recognition_fallback_index_ + 1U,
          overall_test_recognition_fallback_sequence_.size(),
          step,
          segment_distance(),
          target_distance,
          elapsed_total,
          move_timeout);
        break;
      }

      case OverallRecognitionFallbackPhase::WAITING: {
        publish_stop();
        const double wait_elapsed =
          (this->now() - overall_test_recognition_fallback_phase_start_).seconds();
        if (wait_elapsed < std::max(0.0, overall_test_recognition_fallback_wait_sec_)) {
          break;
        }

        ++overall_test_recognition_fallback_index_;
        if (overall_test_recognition_fallback_index_ >=
          overall_test_recognition_fallback_sequence_.size())
        {
          fail_overall_test(
            "recognition fallback sequence exhausted target=" +
            std::to_string(current_target_cabinet_));
          return;
        }

        overall_test_recognition_fallback_phase_ = OverallRecognitionFallbackPhase::MOVING;
        overall_test_recognition_fallback_phase_start_ = this->now();
        reset_segment_distance();
        publish_overall_test_log(
          "recognition fallback step=" +
          std::to_string(overall_test_recognition_fallback_index_ + 1U) +
          " move=" +
          format_seconds(
            overall_test_recognition_fallback_sequence_[overall_test_recognition_fallback_index_]));
        break;
      }

      case OverallRecognitionFallbackPhase::IDLE:
      default:
        overall_test_recognition_fallback_phase_ = OverallRecognitionFallbackPhase::MOVING;
        overall_test_recognition_fallback_phase_start_ = this->now();
        reset_segment_distance();
        break;
    }
  }

  void handle_overall_target_distance_align_state()
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
        "[mission_manager][OVERALL_TEST] target distance align waiting for distance target=%d",
        current_target_cabinet_);
      return;
    }

    const double distance_error = latest_distance_ - follow_distance_;
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = std::clamp(
      tracking_kp_distance_ * distance_error,
      -std::abs(tracking_speed_),
      std::abs(tracking_speed_));
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "[mission_manager][OVERALL_TEST] target distance align target=%d distance=%.2f "
      "follow=%.2f tolerance=%.2f cmd.linear.x=%.3f",
      current_target_cabinet_,
      latest_distance_,
      follow_distance_,
      distance_tolerance_,
      cmd.linear.x);

    if (std::abs(distance_error) <= distance_tolerance_) {
      if (tracking_stable_start_.nanoseconds() == 0) {
        tracking_stable_start_ = this->now();
      }
      if ((this->now() - tracking_stable_start_).seconds() >= distance_stable_time_sec_) {
        publish_stop();
        set_distance_estimator_enabled(false, true);
        publish_overall_test_log("target distance stable, distance=off, gap_detector=on");
        begin_search_gap_flow();
      }
    } else {
      tracking_stable_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    }
  }

  void handle_overall_test_scan_state()
  {
    const int cabinet_id = overall_test_current_target_;
    if (cabinet_id <= 0) {
      fail_overall_test("整体盘库测试扫描柜号非法");
      return;
    }
    publish_stop();
    request_recognizer_enable(false);
    set_recognizer_topic_enabled(false);
    set_distance_estimator_enabled(false);
    set_gap_detector_enabled(false);
    publish_overall_test_log("scan placeholder for cabinet " + std::to_string(cabinet_id));
    if (!execute_placeholder_scan_for_cabinet(cabinet_id, false)) {
      return;
    }
    publish_overall_test_log("scan placeholder finished for cabinet " + std::to_string(cabinet_id));
    set_state(
      State::OVERALL_TEST_EXIT_GAP,
      "[OVERALL_TEST] scan finished, reverse exit gap cabinet=" + std::to_string(cabinet_id));
  }

  bool should_return_home_between_targets(
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
    return overall_test_return_home_between_sides_ && current_side != next_side;
  }

  void start_existing_return_home_for_overall_test(OverallTestReturnReason reason)
  {
    overall_test_return_reason_ = reason;
    overall_test_waiting_return_home_for_side_switch_ =
      reason == OverallTestReturnReason::BETWEEN_SIDES;
    overall_test_waiting_return_home_for_done_ =
      reason == OverallTestReturnReason::FINAL_DONE;
    const std::string reason_text = overall_return_reason_to_string(reason);
    publish_overall_test_log("start existing return home reason=" + reason_text);
    set_test_state(
      State::OVERALL_TEST_RETURN_HOME_BETWEEN_SIDES,
      "[OVERALL_TEST] start return home reason=" + reason_text);
    switch_to_returning(ReturnMode::FINISH_HOME, "[OVERALL_TEST] return home reason=" + reason_text);
  }

  void finish_overall_test_done_after_return()
  {
    publish_overall_test_log("DONE");
    mission_active_ = false;
    test_gap_scan_active_ = false;
    test_real_motion_active_ = false;
    test_real_gap_searching_ = false;
    test_real_target_recognized_logged_ = false;
    test_real_close_requested_ = false;
    overall_test_return_reason_ = OverallTestReturnReason::NONE;
    overall_test_waiting_return_home_for_side_switch_ = false;
    overall_test_waiting_return_home_for_done_ = false;
    set_state(State::OVERALL_TEST_DONE, "[OVERALL_TEST] done");
    reset_overall_test_context();
    test_gap_scan_queue_.clear();
    test_current_gap_index_ = 0;
  }

  void on_return_home_finished_continue_overall_test()
  {
    const auto reason = overall_test_return_reason_;
    return_mode_ = ReturnMode::NONE;
    overall_test_return_reason_ = OverallTestReturnReason::NONE;
    overall_test_waiting_return_home_for_side_switch_ = false;
    overall_test_waiting_return_home_for_done_ = false;

    if (reason == OverallTestReturnReason::FINAL_DONE) {
      finish_overall_test_done_after_return();
      return;
    }

    if (reason != OverallTestReturnReason::BETWEEN_SIDES) {
      fail_overall_test("未知整体盘库测试返航目的");
      return;
    }

    if (overall_test_index_ >= overall_test_sequence_.size()) {
      fail_overall_test("换侧返航后 overall_test_index 越界");
      return;
    }

    const int target = overall_test_sequence_[overall_test_index_];
    std::string prepare_reason;
    if (!prepare_overall_test_target(target, prepare_reason)) {
      fail_overall_test("换侧返航后准备目标失败: " + prepare_reason);
      return;
    }
    publish_overall_test_log(
      "return home finished, switch to side=" + overall_test_current_side_ +
      " route=" + overall_test_current_route_ +
      " target=" + std::to_string(target));
    (void)start_overall_test_target_route("return home finished, switch side");
  }

  void advance_overall_test_sequence()
  {
    if (!overall_test_active_) {
      fail_overall_test("advance requested while overall test inactive");
      return;
    }
    if (overall_test_index_ >= overall_test_sequence_.size()) {
      fail_overall_test("overall_test_index 越界");
      return;
    }

    const int finished_target = overall_test_sequence_[overall_test_index_];

    if (overall_test_index_ + 1U >= overall_test_sequence_.size()) {
      publish_overall_test_log(
        "exit gap done target=" + std::to_string(finished_target) +
        " next_target=-1 same_side=false");
      publish_overall_test_log("all targets completed, final return home");
      if (overall_test_return_home_after_done_) {
        start_existing_return_home_for_overall_test(OverallTestReturnReason::FINAL_DONE);
      } else {
        finish_overall_test_done_after_return();
      }
      return;
    }

    const int next_target = overall_test_sequence_[overall_test_index_ + 1U];
    std::string current_side;
    std::string next_side;
    std::string side_reason;
    const bool return_between =
      should_return_home_between_targets(finished_target, next_target, current_side, next_side, side_reason);
    if (!side_reason.empty()) {
      fail_overall_test("判断目标侧失败: " + side_reason);
      return;
    }
    const bool same_side = current_side == next_side;
    publish_overall_test_log(
      "exit gap done target=" + std::to_string(finished_target) +
      " next_target=" + std::to_string(next_target) +
      " same_side=" + std::string(same_side ? "true" : "false"));

    ++overall_test_index_;
    overall_test_next_target_ =
      overall_test_index_ + 1U < overall_test_sequence_.size() ?
      overall_test_sequence_[overall_test_index_ + 1U] : -1;

    if (return_between) {
      publish_overall_test_log(
        "next_target=" + std::to_string(next_target) +
        " side changed " + current_side + " -> " + next_side +
        ", return home first");
      start_existing_return_home_for_overall_test(OverallTestReturnReason::BETWEEN_SIDES);
      return;
    }

    std::string prepare_reason;
    if (!prepare_overall_test_target(next_target, prepare_reason)) {
      fail_overall_test("准备同侧下一个目标失败: " + prepare_reason);
      return;
    }

    if (same_side && overall_test_same_side_next_search_enabled_) {
      start_overall_same_side_next_search();
      return;
    }

    publish_overall_test_log(
      "next_target=" + std::to_string(next_target) +
      " route restart side=" + next_side);
    (void)start_overall_test_target_route("advance next target");
  }

  bool is_side_row_sequence_request(const std::vector<int> & cabinets) const
  {
    std::vector<int> first_gap_full = test_real_side_row_first_gap_scan_sequence_;
    std::vector<int> full = test_real_side_row_first_gap_scan_sequence_;
    full.insert(
      full.end(),
      test_real_side_row_second_gap_scan_sequence_.begin(),
      test_real_side_row_second_gap_scan_sequence_.end());
    return int_vectors_equal(cabinets, first_gap_full) || int_vectors_equal(cabinets, full);
  }

  bool is_valid_test_real_side_row_request(
    const wheeltec_inventory_system::srv::StartTestGapScan::Request & request) const
  {
    if (!test_real_side_row_enabled_ || request.run_all_configured) {
      return false;
    }
    if (wheeltec_inventory_system::trim(request.gap_id) != test_real_side_row_first_gap_) {
      return false;
    }
    std::vector<int> cabinets;
    for (const auto cabinet_id : request.scan_cabinets) {
      cabinets.push_back(static_cast<int>(cabinet_id));
    }
    return is_side_row_sequence_request(cabinets);
  }

  std::string side_row_reject_message() const
  {
    std::vector<int> full = test_real_side_row_first_gap_scan_sequence_;
    full.insert(
      full.end(),
      test_real_side_row_second_gap_scan_sequence_.begin(),
      test_real_side_row_second_gap_scan_sequence_.end());
    return
      "side row real motion test only supports " + test_real_side_row_name_ +
      " starting from " + test_real_side_row_first_gap_ +
      " with scan_cabinets=" + cabinet_unit_to_string(full);
  }

  std::string active_test_real_gap_id() const
  {
    if (test_real_side_row_active_ && !test_real_active_gap_id_.empty()) {
      return test_real_active_gap_id_;
    }
    return test_real_motion_target_gap_;
  }

  int active_test_real_scan_cabinet() const
  {
    if (test_real_side_row_active_ && test_real_current_scan_cabinet_ > 0) {
      return test_real_current_scan_cabinet_;
    }
    return test_real_motion_target_cabinet_;
  }

  bool prepare_test_real_target_cabinet(int cabinet_id, std::string & reason)
  {
    targets_ = {std::to_string(cabinet_id)};
    current_target_index_ = 0;
    if (!load_configs_and_prepare_current_target(reason)) {
      return false;
    }
    return true;
  }

  bool is_valid_test_real_motion_request(
    const wheeltec_inventory_system::srv::StartTestGapScan::Request & request) const
  {
    if (request.run_all_configured) {
      return false;
    }
    if (wheeltec_inventory_system::trim(request.gap_id) != test_real_motion_target_gap_) {
      return false;
    }
    if (request.scan_cabinets.size() != 1U) {
      return false;
    }
    return request.scan_cabinets[0] == test_real_motion_target_cabinet_;
  }

  bool start_test_real_side_row_context(
    const wheeltec_inventory_system::srv::StartTestGapScan::Request & request,
    TestGapScanPlan & plan,
    std::string & reason)
  {
    reason.clear();
    if (test_real_side_row_first_gap_scan_sequence_.size() < 2U ||
      test_real_side_row_second_gap_scan_sequence_.size() < 2U)
    {
      reason = "side-row 配置至少需要 first_gap/second_gap 各两个扫描柜号";
      return false;
    }
    if (!is_valid_test_real_side_row_request(request)) {
      reason = side_row_reject_message();
      return false;
    }

    reset_test_real_side_row_context();
    for (const auto cabinet_id : request.scan_cabinets) {
      test_real_side_row_requested_sequence_.push_back(static_cast<int>(cabinet_id));
    }

    std::vector<int> full = test_real_side_row_first_gap_scan_sequence_;
    full.insert(
      full.end(),
      test_real_side_row_second_gap_scan_sequence_.begin(),
      test_real_side_row_second_gap_scan_sequence_.end());
    test_real_side_row_full_sequence_ =
      int_vectors_equal(test_real_side_row_requested_sequence_, full);
    test_real_side_row_active_ = true;
    test_real_side_row_phase_ = TestRealSideRowPhase::FIRST_PRIMARY_SCAN;
    test_real_active_gap_id_ = test_real_side_row_first_gap_;
    test_real_current_scan_cabinet_ = test_real_side_row_first_gap_scan_sequence_.front();
    test_real_adjusted_scan_cabinet_ =
      test_real_side_row_first_gap_scan_sequence_.size() > 1U ?
      test_real_side_row_first_gap_scan_sequence_[1] : -1;
    test_real_next_gap_target_cabinet_ = test_real_side_row_corridor_transfer_target_cabinet_;

    plan.gap_id = test_real_active_gap_id_;
    plan.scan_cabinets = test_real_side_row_requested_sequence_;
    return true;
  }

  void publish_test_real_side_row_log(const std::string & text)
  {
    publish_test_real_log("[side_row] " + text);
  }

  bool prepare_test_real_target_search(int cabinet_id, const std::string & context)
  {
    test_real_current_scan_cabinet_ = cabinet_id;
    std::string reason;
    if (!prepare_test_real_target_cabinet(cabinet_id, reason)) {
      fail_test_real_motion(context + "失败: " + reason);
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

  void execute_test_real_grid_move_if_enabled(
    const wheeltec_inventory_system::ScanStep & step)
  {
    if (!test_real_motion_active_) {
      return;
    }
    RCLCPP_INFO(
      get_logger(),
      "[mission_manager][test_real][grid_move] placeholder cabinet=%d layer=%d depth=%d "
      "enabled=%s spacing=%.2f",
      step.cabinet_id,
      step.layer_index,
      step.depth_index,
      test_real_grid_motion_enabled_ ? "true" : "false",
      test_real_grid_spacing_m_);
    if (test_real_grid_motion_enabled_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "[mission_manager][test_real][grid_move] real grid motion is handled by TEST_REAL scan runtime");
    }
  }

  void reset_test_real_scan_runtime()
  {
    test_real_scan_steps_.clear();
    test_real_scan_step_index_ = 0;
    test_real_scan_cabinet_ = -1;
    test_real_scan_active_ = false;
    test_real_scan_step_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

    test_real_grid_have_previous_depth_ = false;
    test_real_grid_previous_cabinet_ = -1;
    test_real_grid_previous_layer_ = -1;
    test_real_grid_previous_depth_ = -1;
    test_real_grid_move_active_ = false;
    test_real_grid_move_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    test_real_grid_move_target_distance_ = 0.0;
    test_real_grid_move_cmd_speed_ = 0.0;
    test_real_grid_move_step_ = wheeltec_inventory_system::ScanStep{};
    test_real_grid_move_start_pose_ = Pose2D{};
  }

  bool begin_test_real_scan_runtime(int cabinet_id)
  {
    test_real_scan_steps_ = scan_sequence_generator_.generateCabinetSnakeSequence(
      cabinet_id,
      test_scan_layers_,
      test_scan_depth_count_);
    if (test_real_scan_steps_.empty()) {
      fail_test_real_motion("生成扫描序列为空: cabinet=" + std::to_string(cabinet_id));
      return false;
    }

    test_real_scan_step_index_ = 0;
    test_real_scan_cabinet_ = cabinet_id;
    test_real_scan_active_ = true;
    test_real_scan_step_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    test_real_grid_have_previous_depth_ = false;
    test_real_grid_previous_cabinet_ = -1;
    test_real_grid_previous_layer_ = -1;
    test_real_grid_previous_depth_ = -1;
    test_real_grid_move_active_ = false;
    test_real_grid_move_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

    publish_test_real_log(
      "in gap, start placeholder scan cabinet=" + std::to_string(cabinet_id));
    RCLCPP_INFO(
      get_logger(),
      "[mission_manager][test_real][scan_runtime] start cabinet=%d step_count=%zu "
      "grid_motion=true spacing=%.2f speed=%.3f timeout=%.2f",
      cabinet_id,
      test_real_scan_steps_.size(),
      test_real_grid_spacing_m_,
      test_real_grid_move_speed_,
      test_real_grid_move_timeout_sec_);
    return true;
  }

  bool test_real_scan_wait_step_started(const wheeltec_inventory_system::ScanStep & step)
  {
    if (test_real_scan_step_start_time_.nanoseconds() != 0) {
      return true;
    }

    test_real_scan_step_start_time_ = this->now();
    RCLCPP_INFO(
      get_logger(),
      "[mission_manager][test_real][scan_runtime] placeholder step=%s cabinet=%d layer=%d depth=%d",
      wheeltec_inventory_system::ScanSequenceGenerator::stepTypeToString(step.step_type).c_str(),
      step.cabinet_id,
      step.layer_index,
      step.depth_index);
    return false;
  }

  bool execute_test_real_scan_wait_step(
    const wheeltec_inventory_system::ScanStep & step,
    double wait_sec)
  {
    const bool already_started = test_real_scan_wait_step_started(step);
    (void)already_started;
    if ((this->now() - test_real_scan_step_start_time_).seconds() < std::max(0.0, wait_sec)) {
      return false;
    }

    if (step.step_type == wheeltec_inventory_system::ScanStepType::SCAN_PLACEHOLDER) {
      if (!web_api_client_.reportInventoryResult(
          step.cabinet_id,
          step.layer_index,
          step.depth_index,
          "placeholder_ok"))
      {
        fail_test_real_motion(
          "占位扫描结果上报失败: cabinet=" + std::to_string(step.cabinet_id) +
          " layer=" + std::to_string(step.layer_index) +
          " depth=" + std::to_string(step.depth_index));
        return false;
      }
    }

    test_real_scan_step_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    return true;
  }

  bool test_real_grid_move_step_matches(const wheeltec_inventory_system::ScanStep & step) const
  {
    return
      test_real_grid_move_step_.cabinet_id == step.cabinet_id &&
      test_real_grid_move_step_.layer_index == step.layer_index &&
      test_real_grid_move_step_.depth_index == step.depth_index &&
      test_real_grid_move_step_.step_type == step.step_type;
  }

  bool execute_test_real_grid_move_step(const wheeltec_inventory_system::ScanStep & step)
  {
    if (!test_real_grid_motion_enabled_) {
      return true;
    }

    const bool cabinet_changed =
      !test_real_grid_have_previous_depth_ ||
      test_real_grid_previous_cabinet_ != step.cabinet_id;
    const bool layer_changed =
      test_real_grid_have_previous_depth_ &&
      test_real_grid_previous_layer_ != step.layer_index;
    if (cabinet_changed ||
      (layer_changed && test_real_grid_move_return_between_layers_))
    {
      test_real_grid_have_previous_depth_ = true;
      test_real_grid_previous_cabinet_ = step.cabinet_id;
      test_real_grid_previous_layer_ = step.layer_index;
      test_real_grid_previous_depth_ = step.depth_index;
      RCLCPP_INFO(
        get_logger(),
        "[mission_manager][test_real][grid_move] cabinet=%d layer=%d depth=%d first depth, no move",
        step.cabinet_id,
        step.layer_index,
        step.depth_index);
      publish_test_real_log(
        "[grid_move] cabinet=" + std::to_string(step.cabinet_id) +
        " layer=" + std::to_string(step.layer_index) +
        " depth=" + std::to_string(step.depth_index) +
        " first depth, no move");
      return true;
    }

    const int delta_depth = step.depth_index - test_real_grid_previous_depth_;
    if (delta_depth == 0) {
      test_real_grid_previous_layer_ = step.layer_index;
      test_real_grid_previous_depth_ = step.depth_index;
      RCLCPP_INFO(
        get_logger(),
        "[mission_manager][test_real][grid_move] cabinet=%d layer=%d depth=%d delta_depth=0 no move",
        step.cabinet_id,
        step.layer_index,
        step.depth_index);
      return true;
    }

    const double spacing = std::max(0.0, test_real_grid_spacing_m_);
    const double target_distance = std::abs(delta_depth) * spacing;
    if (target_distance <= 1e-4) {
      test_real_grid_previous_layer_ = step.layer_index;
      test_real_grid_previous_depth_ = step.depth_index;
      RCLCPP_INFO(
        get_logger(),
        "[mission_manager][test_real][grid_move] cabinet=%d layer=%d depth=%d spacing=0 no move",
        step.cabinet_id,
        step.layer_index,
        step.depth_index);
      return true;
    }

    const double speed_abs = std::clamp(std::abs(test_real_grid_move_speed_), 0.0, 0.05);
    if (speed_abs <= 1e-4) {
      fail_test_real_motion(
        "深度格移动速度非法: " + std::to_string(test_real_grid_move_speed_));
      return false;
    }
    const double signed_speed = delta_depth > 0 ? speed_abs : -speed_abs;

    if (!test_real_grid_move_active_ || !test_real_grid_move_step_matches(step)) {
      std::string reason;
      if (!current_odom_ready_for_entry(reason)) {
        fail_test_real_motion("深度格移动前里程计异常: " + reason);
        return false;
      }
      const Pose2D current = current_pose_2d();
      if (!current.valid) {
        fail_test_real_motion("深度格移动前当前位姿无效");
        return false;
      }

      test_real_grid_move_active_ = true;
      test_real_grid_move_step_ = step;
      test_real_grid_move_start_pose_ = current;
      test_real_grid_move_target_distance_ = target_distance;
      test_real_grid_move_cmd_speed_ = signed_speed;
      test_real_grid_move_start_time_ = this->now();
      reset_segment_distance();

      RCLCPP_INFO(
        get_logger(),
        "[mission_manager][test_real][grid_move] cabinet=%d layer=%d depth=%d "
        "delta_depth=%d distance=%.2f speed=%.3f timeout=%.2f",
        step.cabinet_id,
        step.layer_index,
        step.depth_index,
        delta_depth,
        test_real_grid_move_target_distance_,
        test_real_grid_move_cmd_speed_,
        test_real_grid_move_timeout_sec_);
      RCLCPP_INFO(
        get_logger(),
        "[mission_manager][test_real][grid_move] start_pose x=%.3f y=%.3f yaw=%.3f frame=%s",
        current.x,
        current.y,
        current.yaw,
        current.frame_id.c_str());
      publish_test_real_log(
        "[grid_move] cabinet=" + std::to_string(step.cabinet_id) +
        " layer=" + std::to_string(step.layer_index) +
        " depth=" + std::to_string(step.depth_index) +
        " delta_depth=" + std::to_string(delta_depth) +
        " distance=" + format_seconds(test_real_grid_move_target_distance_) +
        " speed=" + format_seconds(test_real_grid_move_cmd_speed_));
      return false;
    }

    std::string reason;
    if (!current_odom_ready_for_entry(reason)) {
      publish_stop();
      fail_test_real_motion("深度格移动中里程计异常: " + reason);
      return false;
    }

    const double elapsed = (this->now() - test_real_grid_move_start_time_).seconds();
    const double traveled = segment_distance();
    const double timeout = std::max(0.1, test_real_grid_move_timeout_sec_);
    if (elapsed > timeout) {
      publish_stop();
      RCLCPP_ERROR(
        get_logger(),
        "[mission_manager][test_real][grid_move] timeout cabinet=%d layer=%d depth=%d "
        "traveled=%.2f target=%.2f elapsed=%.2f",
        step.cabinet_id,
        step.layer_index,
        step.depth_index,
        traveled,
        test_real_grid_move_target_distance_,
        elapsed);
      fail_test_real_motion(
        "深度格移动超时: cabinet=" + std::to_string(step.cabinet_id) +
        " layer=" + std::to_string(step.layer_index) +
        " depth=" + std::to_string(step.depth_index));
      return false;
    }

    if (traveled >= test_real_grid_move_target_distance_) {
      publish_stop();
      test_real_grid_previous_layer_ = step.layer_index;
      test_real_grid_previous_depth_ = step.depth_index;
      test_real_grid_move_active_ = false;
      test_real_grid_move_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      RCLCPP_INFO(
        get_logger(),
        "[mission_manager][test_real][grid_move] move finished cabinet=%d layer=%d depth=%d "
        "traveled=%.2f",
        step.cabinet_id,
        step.layer_index,
        step.depth_index,
        traveled);
      publish_test_real_log(
        "[grid_move] move finished cabinet=" + std::to_string(step.cabinet_id) +
        " layer=" + std::to_string(step.layer_index) +
        " depth=" + std::to_string(step.depth_index));
      return true;
    }

    if (test_real_grid_move_cmd_speed_ > 0.0) {
      const auto safety = evaluate_entering_safety();
      if (safety.blocked) {
        publish_stop();
        fail_test_real_motion("深度格前进被安全策略阻塞: " + safety.block_reason);
        return false;
      }
    } else {
      const double ultrasonic_range = min_ultrasonic_range();
      if (std::isfinite(ultrasonic_range) && ultrasonic_range < entry_ultrasonic_stop_distance_) {
        publish_stop();
        fail_test_real_motion(
          "深度格后退被超声波安全策略阻塞 range=" + std::to_string(ultrasonic_range));
        return false;
      }
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = test_real_grid_move_cmd_speed_;
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "[mission_manager][test_real][grid_move] moving traveled=%.2f target=%.2f elapsed=%.2f",
      traveled,
      test_real_grid_move_target_distance_,
      elapsed);
    return false;
  }

  bool execute_test_real_scan_runtime_tick(int cabinet_id)
  {
    if (!test_real_scan_active_ || test_real_scan_cabinet_ != cabinet_id) {
      if (!begin_test_real_scan_runtime(cabinet_id)) {
        return false;
      }
    }

    if (test_real_scan_step_index_ >= test_real_scan_steps_.size()) {
      reset_test_real_scan_runtime();
      return true;
    }

    const auto & step = test_real_scan_steps_[test_real_scan_step_index_];
    bool step_done = false;
    switch (step.step_type) {
      case wheeltec_inventory_system::ScanStepType::MOVE_TO_GRID:
        step_done = execute_test_real_grid_move_step(step);
        break;
      case wheeltec_inventory_system::ScanStepType::SCAN_PLACEHOLDER:
        step_done = execute_test_real_scan_wait_step(step, test_scan_placeholder_wait_sec_);
        break;
      case wheeltec_inventory_system::ScanStepType::LIFT_PLACEHOLDER:
      case wheeltec_inventory_system::ScanStepType::LOWER_PLACEHOLDER:
        step_done = execute_test_real_scan_wait_step(step, test_lift_placeholder_wait_sec_);
        break;
    }

    if (!step_done) {
      return false;
    }

    ++test_real_scan_step_index_;
    if (test_real_scan_step_index_ >= test_real_scan_steps_.size()) {
      reset_test_real_scan_runtime();
      return true;
    }

    return false;
  }

  void finish_test_real_exit_gap()
  {
    publish_stop();
    test_real_exit_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    test_real_exit_phase_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    test_real_exit_phase_ = TestRealExitPhase::STRAIGHT_REVERSE;
    test_real_exit_effective_timeout_sec_ = test_real_exit_timeout_sec_;

    if (overall_test_active_) {
      set_state(
        State::OVERALL_TEST_ADVANCE_NEXT_TARGET,
        "[OVERALL_TEST] exit gap finished, advance sequence");
      advance_overall_test_sequence();
      return;
    }

    const auto action = test_real_after_exit_action_;
    test_real_after_exit_action_ = TestRealAfterExitAction::NONE;

    switch (action) {
      case TestRealAfterExitAction::REENTER_ADJUSTED:
        set_test_real_state(
          State::TEST_REAL_REENTER_FOR_ADJUSTED_SCAN,
          "倒退出缝完成，准备再次入缝做临时位置调整");
        return;
      case TestRealAfterExitAction::CORRIDOR_TRANSFER:
        set_test_real_state(
          State::TEST_REAL_CORRIDOR_TRANSFER,
          "倒退出缝完成，准备走廊转移到下一目标柜");
        return;
      case TestRealAfterExitAction::CLOSE_AND_DONE:
        set_test_real_state(State::TEST_REQUESTING_CLOSE_GAP, "出缝完成，准备 mock close gap");
        return;
      case TestRealAfterExitAction::FINAL_CLOSE_AND_DONE:
        if (!test_real_close_gap_after_final_exit_) {
          set_test_real_state(State::TEST_DONE, "最终出缝完成，配置为不 mock close gap");
          return;
        }
        set_test_real_state(State::TEST_REQUESTING_CLOSE_GAP, "最终出缝完成，准备 mock close gap");
        return;
      case TestRealAfterExitAction::NONE:
      default:
        fail_test_real_motion("出缝完成后缺少下一步动作");
        return;
    }
  }

  void handle_test_real_exit_gap_state()
  {
    std::string mode = test_real_exit_mode_;
    std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    if (mode != "reverse") {
      if (overall_test_active_) {
        fail_overall_test("当前只支持 reverse 出缝模式: " + test_real_exit_mode_);
      } else {
        fail_test_real_motion("当前只支持 reverse 出缝模式: " + test_real_exit_mode_);
      }
      return;
    }

    const double speed = std::clamp(std::abs(test_real_exit_speed_), 0.0, 0.05);
    if (speed <= 1e-4) {
      if (overall_test_active_) {
        fail_overall_test("倒退出缝速度非法: " + std::to_string(test_real_exit_speed_));
      } else {
        fail_test_real_motion("倒退出缝速度非法: " + std::to_string(test_real_exit_speed_));
      }
      return;
    }

    if (test_real_exit_start_time_.nanoseconds() == 0) {
      const double measured_distance =
        std::isfinite(test_real_last_entering_straight_distance_) &&
        test_real_last_entering_straight_distance_ > 0.05 ?
        test_real_last_entering_straight_distance_ + test_real_exit_extra_distance_m_ :
        test_real_exit_distance_m_;
      test_real_exit_target_distance_ =
        std::max(0.05, std::isfinite(measured_distance) ? measured_distance : test_real_exit_distance_m_);
      const double required_time = test_real_exit_target_distance_ / speed;
      const double configured_timeout =
        std::isfinite(test_real_exit_timeout_sec_) ? test_real_exit_timeout_sec_ : 0.0;
      const double minimum_timeout = required_time + 5.0;
      test_real_exit_effective_timeout_sec_ =
        std::max(std::max(0.1, configured_timeout), minimum_timeout);
      if (configured_timeout < minimum_timeout) {
        RCLCPP_WARN(
          get_logger(),
          "[mission_manager][test_real][exit_gap] warning: timeout too short for distance/speed, "
          "required=%.2f configured=%.2f",
          required_time,
          configured_timeout);
        publish_motion_test_log(
          "[exit_gap] warning: timeout too short for distance/speed, required=" +
          format_seconds(required_time) +
          ", configured=" + format_seconds(configured_timeout));
      }
      reset_segment_distance();
      test_real_exit_start_time_ = this->now();
      test_real_exit_phase_start_time_ = test_real_exit_start_time_;
      test_real_exit_phase_ = TestRealExitPhase::STRAIGHT_REVERSE;
      RCLCPP_INFO(
        get_logger(),
        "[mission_manager][test_real][exit_gap] start reverse exit phase=%s distance=%.2f "
        "speed=%.3f timeout=%.2f turn_enabled=%s",
        test_real_exit_phase_to_string(test_real_exit_phase_).c_str(),
        test_real_exit_target_distance_,
        speed,
        test_real_exit_effective_timeout_sec_,
        test_real_exit_turn_enabled_ ? "true" : "false");
      publish_motion_test_log(
        "[exit_gap] start reverse exit phase=" +
        test_real_exit_phase_to_string(test_real_exit_phase_) +
        " distance=" +
        format_seconds(test_real_exit_target_distance_) +
        " speed=" + format_seconds(speed) +
        " timeout=" + format_seconds(test_real_exit_effective_timeout_sec_) +
        " turn_enabled=" + (test_real_exit_turn_enabled_ ? "true" : "false"));
    }

    const double ultrasonic_range = min_ultrasonic_range();
    if (std::isfinite(ultrasonic_range) && ultrasonic_range < entry_ultrasonic_stop_distance_) {
      publish_stop();
      const std::string reason =
        "倒退出缝被超声波安全策略阻塞 range=" + std::to_string(ultrasonic_range);
      if (overall_test_active_) {
        fail_overall_test(reason);
      } else {
        fail_test_real_motion(reason);
      }
      return;
    }

    if (test_real_exit_phase_ == TestRealExitPhase::STRAIGHT_REVERSE) {
      const double elapsed = (this->now() - test_real_exit_phase_start_time_).seconds();
      const double traveled = segment_distance();
      if (elapsed > std::max(0.1, test_real_exit_effective_timeout_sec_)) {
        publish_stop();
        RCLCPP_ERROR(
          get_logger(),
          "[mission_manager][test_real][exit_gap] phase=%s timeout traveled=%.2f "
          "target_distance=%.2f elapsed=%.2f",
          test_real_exit_phase_to_string(test_real_exit_phase_).c_str(),
          traveled,
          test_real_exit_target_distance_,
          elapsed);
        publish_motion_test_log(
          "[exit_gap] phase=" + test_real_exit_phase_to_string(test_real_exit_phase_) +
          " timeout traveled=" + format_seconds(traveled) +
          " target_distance=" + format_seconds(test_real_exit_target_distance_) +
          " elapsed=" + format_seconds(elapsed));
        if (overall_test_active_) {
          fail_overall_test("倒退出缝直线阶段超时");
        } else {
          fail_test_real_motion("倒退出缝直线阶段超时");
        }
        return;
      }

      if (traveled >= test_real_exit_target_distance_) {
        publish_stop();
        if (!test_real_exit_turn_enabled_) {
          RCLCPP_INFO(
            get_logger(),
            "[mission_manager][test_real][exit_gap] 出缝完成：straight_reverse only traveled=%.2f",
            traveled);
          publish_motion_test_log(
            "[exit_gap] 出缝完成：straight_reverse only traveled=" + format_seconds(traveled));
          finish_test_real_exit_gap();
          return;
        }

        const double turn_speed = std::isfinite(test_real_exit_turn_angular_speed_) ?
          std::abs(test_real_exit_turn_angular_speed_) : 0.0;
        if (turn_speed <= 1e-4) {
          RCLCPP_WARN(
            get_logger(),
            "[mission_manager][test_real][exit_gap] 弧线出缝角速度非法，降级为直线出缝完成: %.3f",
            test_real_exit_turn_angular_speed_);
          publish_motion_test_log("[exit_gap] exit turn angular speed invalid, finish straight_reverse only");
          finish_test_real_exit_gap();
          return;
        }

        std::string yaw_reason;
        if (!current_odom_ready_for_entry(yaw_reason)) {
          RCLCPP_WARN(
            get_logger(),
            "[mission_manager][test_real][exit_gap] 弧线出缝无法获取有效yaw，降级完成: %s",
            yaw_reason.c_str());
          publish_motion_test_log("[exit_gap] yaw invalid before arc_reverse, finish straight_reverse only");
          finish_test_real_exit_gap();
          return;
        }

        const Pose2D current = current_pose_2d();
        if (!current.valid || !std::isfinite(current.yaw) || !std::isfinite(entry_turn_start_yaw_)) {
          RCLCPP_WARN(
            get_logger(),
            "[mission_manager][test_real][exit_gap] 弧线出缝yaw数据无效，降级完成");
          publish_motion_test_log("[exit_gap] yaw data invalid before arc_reverse, finish straight_reverse only");
          finish_test_real_exit_gap();
          return;
        }

        const double target_exit_yaw = normalize_angle(entry_turn_start_yaw_);
        const double yaw_error = normalize_angle(target_exit_yaw - current.yaw);
        const double yaw_tolerance =
          std::isfinite(test_real_exit_turn_yaw_tolerance_rad_) ?
          std::max(0.001, std::abs(test_real_exit_turn_yaw_tolerance_rad_)) : 0.08;
        if (std::abs(yaw_error) <= yaw_tolerance) {
          RCLCPP_INFO(
            get_logger(),
            "[mission_manager][test_real][exit_gap] 出缝完成：straight_reverse + arc_reverse "
            "yaw already aligned current_yaw=%.3f target_exit_yaw=%.3f yaw_error=%.3f",
            current.yaw,
            target_exit_yaw,
            yaw_error);
          publish_motion_test_log("[exit_gap] 出缝完成：straight_reverse + arc_reverse yaw already aligned");
          finish_test_real_exit_gap();
          return;
        }

        test_real_exit_phase_ = TestRealExitPhase::ARC_REVERSE;
        test_real_exit_phase_start_time_ = this->now();
        RCLCPP_INFO(
          get_logger(),
          "[mission_manager][test_real][exit_gap] switch phase=%s entry_side=%s current_yaw=%.3f "
          "target_exit_yaw=%.3f yaw_error=%.3f",
          test_real_exit_phase_to_string(test_real_exit_phase_).c_str(),
          current_entry_side_.c_str(),
          current.yaw,
          target_exit_yaw,
          yaw_error);
        publish_motion_test_log(
          "[exit_gap] switch phase=" + test_real_exit_phase_to_string(test_real_exit_phase_) +
          " entry_side=" + current_entry_side_);
        return;
      }

      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = -speed;
      cmd.angular.z = 0.0;
      cmd_pub_->publish(cmd);
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "[mission_manager][test_real][exit_gap] phase=%s traveled=%.2f target_distance=%.2f "
        "cmd.linear.x=%.3f cmd.angular.z=%.3f elapsed=%.2f",
        test_real_exit_phase_to_string(test_real_exit_phase_).c_str(),
        traveled,
        test_real_exit_target_distance_,
        cmd.linear.x,
        cmd.angular.z,
        elapsed);
      return;
    }

    if (test_real_exit_phase_ == TestRealExitPhase::ARC_REVERSE) {
      std::string yaw_reason;
      if (!current_odom_ready_for_entry(yaw_reason)) {
        publish_stop();
        RCLCPP_WARN(
          get_logger(),
          "[mission_manager][test_real][exit_gap] phase=%s yaw无效，结束出缝避免卡死: %s",
          test_real_exit_phase_to_string(test_real_exit_phase_).c_str(),
          yaw_reason.c_str());
        publish_motion_test_log("[exit_gap] arc_reverse yaw invalid, finish to avoid blocking");
        finish_test_real_exit_gap();
        return;
      }

      const Pose2D current = current_pose_2d();
      if (!current.valid || !std::isfinite(current.yaw) || !std::isfinite(entry_turn_start_yaw_)) {
        publish_stop();
        RCLCPP_WARN(
          get_logger(),
          "[mission_manager][test_real][exit_gap] phase=%s yaw数据无效，结束出缝避免卡死",
          test_real_exit_phase_to_string(test_real_exit_phase_).c_str());
        publish_motion_test_log("[exit_gap] arc_reverse yaw data invalid, finish to avoid blocking");
        finish_test_real_exit_gap();
        return;
      }

      const double target_exit_yaw = normalize_angle(entry_turn_start_yaw_);
      const double yaw_error = normalize_angle(target_exit_yaw - current.yaw);
      const double yaw_tolerance =
        std::isfinite(test_real_exit_turn_yaw_tolerance_rad_) ?
        std::max(0.001, std::abs(test_real_exit_turn_yaw_tolerance_rad_)) : 0.08;
      const double expected_error_sign = current_entry_side_ == "right" ? 1.0 : -1.0;
      const bool crossed_exit_yaw = expected_error_sign * yaw_error <= 0.0;
      if (std::abs(yaw_error) <= yaw_tolerance || crossed_exit_yaw) {
        publish_stop();
        RCLCPP_INFO(
          get_logger(),
          "[mission_manager][test_real][exit_gap] 出缝完成：straight_reverse + arc_reverse "
          "entry_side=%s current_yaw=%.3f target_exit_yaw=%.3f yaw_error=%.3f crossed=%s",
          current_entry_side_.c_str(),
          current.yaw,
          target_exit_yaw,
          yaw_error,
          crossed_exit_yaw ? "true" : "false");
        publish_motion_test_log("[exit_gap] 出缝完成：straight_reverse + arc_reverse");
        finish_test_real_exit_gap();
        return;
      }

      const double elapsed = (this->now() - test_real_exit_phase_start_time_).seconds();
      const double turn_timeout =
        std::isfinite(test_real_exit_turn_timeout_sec_) ?
        std::max(0.1, test_real_exit_turn_timeout_sec_) : 8.0;
      if (elapsed > turn_timeout) {
        publish_stop();
        RCLCPP_WARN(
          get_logger(),
          "[mission_manager][test_real][exit_gap] phase=%s timeout entry_side=%s current_yaw=%.3f "
          "target_exit_yaw=%.3f yaw_error=%.3f elapsed=%.2f",
          test_real_exit_phase_to_string(test_real_exit_phase_).c_str(),
          current_entry_side_.c_str(),
          current.yaw,
          target_exit_yaw,
          yaw_error,
          elapsed);
        publish_motion_test_log("[exit_gap] arc_reverse timeout, finish to avoid blocking");
        finish_test_real_exit_gap();
        return;
      }

      const double turn_speed = std::isfinite(test_real_exit_turn_angular_speed_) ?
        std::abs(test_real_exit_turn_angular_speed_) : 0.0;
      if (turn_speed <= 1e-4) {
        publish_stop();
        RCLCPP_WARN(
          get_logger(),
          "[mission_manager][test_real][exit_gap] phase=%s angular speed invalid, finish to avoid blocking",
          test_real_exit_phase_to_string(test_real_exit_phase_).c_str());
        publish_motion_test_log("[exit_gap] arc_reverse angular speed invalid, finish to avoid blocking");
        finish_test_real_exit_gap();
        return;
      }

      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = -speed;
      cmd.angular.z = current_entry_side_ == "right" ? turn_speed : -turn_speed;
      cmd_pub_->publish(cmd);
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "[mission_manager][test_real][exit_gap] phase=%s entry_side=%s current_yaw=%.3f "
        "target_exit_yaw=%.3f yaw_error=%.3f cmd.linear.x=%.3f cmd.angular.z=%.3f elapsed=%.2f",
        test_real_exit_phase_to_string(test_real_exit_phase_).c_str(),
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
      "[mission_manager][test_real][exit_gap] unknown exit phase, reset to STRAIGHT_REVERSE");
    test_real_exit_phase_ = TestRealExitPhase::STRAIGHT_REVERSE;
    test_real_exit_phase_start_time_ = this->now();
  }

  void handle_test_real_side_row_scan_finished(int cabinet_id)
  {
    if (!test_real_exit_after_each_scan_) {
      fail_test_real_motion("side-row 流程当前要求 test_real_exit_after_each_scan=true");
      return;
    }

    switch (test_real_side_row_phase_) {
      case TestRealSideRowPhase::FIRST_PRIMARY_SCAN:
        publish_test_real_side_row_log(
          "cabinet=" + std::to_string(cabinet_id) +
          " scan finished, exit and reenter for adjusted cabinet=" +
          std::to_string(test_real_adjusted_scan_cabinet_));
        test_real_side_row_phase_ = TestRealSideRowPhase::FIRST_ADJUSTED_SCAN;
        test_real_current_scan_cabinet_ = test_real_adjusted_scan_cabinet_;
        test_real_after_exit_action_ = TestRealAfterExitAction::REENTER_ADJUSTED;
        set_test_real_state(
          State::TEST_REAL_EXIT_GAP,
          "cabinet=" + std::to_string(cabinet_id) + " 扫描完成，倒退出缝");
        return;

      case TestRealSideRowPhase::FIRST_ADJUSTED_SCAN:
        publish_test_real_side_row_log(
          "cabinet=" + std::to_string(cabinet_id) + " scan finished, exit to corridor");
        if (test_real_side_row_full_sequence_) {
          test_real_after_exit_action_ = TestRealAfterExitAction::CORRIDOR_TRANSFER;
        } else {
          test_real_after_exit_action_ = TestRealAfterExitAction::CLOSE_AND_DONE;
        }
        set_test_real_state(
          State::TEST_REAL_EXIT_GAP,
          "cabinet=" + std::to_string(cabinet_id) + " 扫描完成，倒退出缝");
        return;

      case TestRealSideRowPhase::SECOND_PRIMARY_SCAN:
        publish_test_real_side_row_log(
          "cabinet=" + std::to_string(cabinet_id) +
          " scan finished, exit and reenter for adjusted cabinet=" +
          std::to_string(test_real_adjusted_scan_cabinet_));
        test_real_side_row_phase_ = TestRealSideRowPhase::SECOND_ADJUSTED_SCAN;
        test_real_current_scan_cabinet_ = test_real_adjusted_scan_cabinet_;
        test_real_after_exit_action_ = TestRealAfterExitAction::REENTER_ADJUSTED;
        set_test_real_state(
          State::TEST_REAL_EXIT_GAP,
          "cabinet=" + std::to_string(cabinet_id) + " 扫描完成，倒退出缝");
        return;

      case TestRealSideRowPhase::SECOND_ADJUSTED_SCAN:
        publish_test_real_side_row_log(
          "cabinet=" + std::to_string(cabinet_id) + " scan finished, final exit");
        test_real_side_row_phase_ = TestRealSideRowPhase::COMPLETE;
        test_real_after_exit_action_ = TestRealAfterExitAction::FINAL_CLOSE_AND_DONE;
        set_test_real_state(
          State::TEST_REAL_FINAL_EXIT_GAP,
          "cabinet=" + std::to_string(cabinet_id) + " 扫描完成，最终倒退出缝");
        return;

      case TestRealSideRowPhase::NONE:
      case TestRealSideRowPhase::CORRIDOR_TRANSFER:
      case TestRealSideRowPhase::COMPLETE:
      default:
        fail_test_real_motion(
          "side-row 扫描完成阶段非法 phase=" + side_row_phase_to_string(test_real_side_row_phase_));
        return;
    }
  }

  void handle_test_real_scan_state()
  {
    const int cabinet_id = active_test_real_scan_cabinet();
    if (cabinet_id <= 0) {
      fail_test_real_motion("测试真实运动扫描柜号非法");
      return;
    }

    if (test_real_side_row_active_ &&
      (!test_real_grid_motion_enabled_ || !test_real_scan_active_))
    {
      publish_test_real_side_row_log(
        "gap=" + active_test_real_gap_id() +
        " scan cabinet=" + std::to_string(cabinet_id) +
        " phase=" + side_row_phase_to_string(test_real_side_row_phase_));
    }

    if (test_real_grid_motion_enabled_) {
      if (!execute_test_real_scan_runtime_tick(cabinet_id)) {
        return;
      }
    } else if (!execute_placeholder_scan_for_cabinet(cabinet_id, true)) {
      return;
    }

    if (test_real_side_row_active_) {
      handle_test_real_side_row_scan_finished(cabinet_id);
      return;
    }

    if (!test_real_motion_stop_after_scan_) {
      publish_test_real_log("scan finished, exit gap");
      test_real_after_exit_action_ = TestRealAfterExitAction::CLOSE_AND_DONE;
      set_test_real_state(State::TEST_REAL_FINAL_EXIT_GAP, "单柜扫描完成，倒退出缝");
      return;
    }

    publish_test_real_log("scan finished, stop after scan");
    set_test_real_state(
      State::TEST_REAL_STOP_AFTER_SCAN,
      "scan finished, stop after scan, no exit motion in this test step");
  }

  void handle_test_real_reenter_adjusted_scan_state()
  {
    if (!test_real_reentry_for_position_adjustment_) {
      fail_test_real_motion("side-row 调整扫描需要启用 test_real_reentry_for_position_adjustment");
      return;
    }

    // TODO: right-mounted camera orientation adjustment is required before real scanning on the adjusted/opposite scan side.
    publish_test_real_log(
      "[reenter_adjust] reenter for adjusted cabinet=" +
      std::to_string(test_real_current_scan_cabinet_));
    publish_test_real_log("[reenter_adjust] " + test_real_reentry_comment_);
    (void)prepare_test_real_target_search(
      test_real_current_scan_cabinet_,
      "准备再次入缝调整扫描 cabinet=" + std::to_string(test_real_current_scan_cabinet_));
  }

  void handle_test_real_corridor_transfer_state()
  {
    if (!test_real_side_row_corridor_transfer_enabled_) {
      fail_test_real_motion("test_real_side_row_corridor_transfer_enabled=false");
      return;
    }

    const std::string previous_gap = active_test_real_gap_id();
    publish_test_real_log("[corridor_transfer] mock close gap=" + previous_gap);
    (void)web_api_client_.requestCloseGap(previous_gap);

    test_real_side_row_phase_ = TestRealSideRowPhase::CORRIDOR_TRANSFER;
    test_real_current_scan_cabinet_ = test_real_side_row_corridor_transfer_target_cabinet_;
    test_real_target_recognized_logged_ = false;
    test_real_final_recognition_wait_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

    publish_test_real_log(
      "[corridor_transfer] move toward cabinet 1, target cabinet=" +
      std::to_string(test_real_side_row_corridor_transfer_target_cabinet_) +
      " direction=" + test_real_side_row_corridor_transfer_direction_);

    std::string reason;
    if (!prepare_test_real_target_cabinet(test_real_side_row_corridor_transfer_target_cabinet_, reason)) {
      fail_test_real_motion("准备走廊转移目标失败: " + reason);
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
        "[mission_manager][test_real][corridor_transfer] enter real nav target cabinet=" +
        std::to_string(current_target_cabinet_),
        nav_reason,
        State::TEST_REAL_NAV_TO_TARGET))
    {
      fail_test_real_motion("走廊转移启动巡航失败: " + nav_reason);
    }
  }

  void handle_test_real_prepare_next_gap_state()
  {
    test_real_active_gap_id_ = test_real_side_row_second_gap_;
    test_real_side_row_phase_ = TestRealSideRowPhase::SECOND_PRIMARY_SCAN;
    test_real_current_scan_cabinet_ = test_real_side_row_second_gap_scan_sequence_.front();
    test_real_adjusted_scan_cabinet_ = test_real_side_row_second_gap_scan_sequence_[1];
    test_real_after_exit_action_ = TestRealAfterExitAction::NONE;

    publish_test_real_side_row_log(
      "prepare next gap=" + test_real_active_gap_id_ +
      " target cabinet=" + std::to_string(test_real_current_scan_cabinet_));
    publish_test_real_side_row_log(
      "enter " + test_real_active_gap_id_ +
      " for cabinet=" + std::to_string(test_real_current_scan_cabinet_));
    publish_test_real_log("mock open gap=" + test_real_active_gap_id_);
    if (!web_api_client_.requestOpenGap(test_real_active_gap_id_)) {
      fail_test_real_motion("mock/网页开柜请求失败: gap=" + test_real_active_gap_id_);
      return;
    }

    set_test_real_state(
      State::TEST_REAL_REENTER_NEXT_GAP,
      "等待第二个缝隙开柜延时 " + format_seconds(test_open_gap_wait_sec_) + " sec");
  }

  void handle_test_real_reenter_next_gap_state()
  {
    publish_stop();
    if (!test_state_elapsed(test_open_gap_wait_sec_)) {
      return;
    }

    (void)prepare_test_real_target_search(
      test_real_current_scan_cabinet_,
      "准备进入第二个缝隙 cabinet=" + std::to_string(test_real_current_scan_cabinet_));
  }

  bool build_test_gap_scan_plans(
    const wheeltec_inventory_system::srv::StartTestGapScan::Request & request,
    std::vector<TestGapScanPlan> & plans,
    std::string & reason) const
  {
    plans.clear();
    reason.clear();

    if (test_scan_layers_ <= 0 || test_scan_depth_count_ <= 0) {
      reason =
        "scan_layers/scan_depth_count 必须为正数: layers=" +
        std::to_string(test_scan_layers_) +
        " depth_count=" + std::to_string(test_scan_depth_count_);
      return false;
    }

    if (request.run_all_configured) {
      plans = configured_test_inventory_plan_;
      if (plans.empty()) {
        reason = "test_inventory_plan 为空";
        return false;
      }
    } else {
      TestGapScanPlan plan;
      plan.gap_id = wheeltec_inventory_system::trim(request.gap_id);
      if (plan.gap_id.empty()) {
        reason = "run_all_configured=false 时 gap_id 不能为空";
        return false;
      }

      for (const auto cabinet_id : request.scan_cabinets) {
        plan.scan_cabinets.push_back(static_cast<int>(cabinet_id));
      }

      if (plan.scan_cabinets.empty() &&
        !find_configured_test_cabinets(plan.gap_id, plan.scan_cabinets))
      {
        reason = "未找到 gap_id 对应测试柜号: " + plan.gap_id;
        return false;
      }
      plans.push_back(plan);
    }

    for (const auto & plan : plans) {
      if (!validate_test_gap_scan_plan(plan, reason)) {
        return false;
      }
    }
    return true;
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

  const TestGapScanPlan * current_test_gap_scan_plan() const
  {
    if (test_current_gap_index_ >= test_gap_scan_queue_.size()) {
      return nullptr;
    }
    return &test_gap_scan_queue_[test_current_gap_index_];
  }

  bool test_state_elapsed(double seconds) const
  {
    return (this->now() - test_state_enter_time_).seconds() >= std::max(0.0, seconds);
  }

  void fail_test_gap_scan(const std::string & reason)
  {
    test_gap_scan_error_reason_ = reason;
    set_test_state(State::TEST_ERROR, reason);
  }

  void stop_test_real_motion_controls()
  {
    cancel_nav2_route_goal("测试真实运动停止");
    cancel_nav2_return_goal("测试真实运动停止");
    set_corridor_mode(false, false);
    publish_stop();
    request_recognizer_enable(false);
    set_recognizer_topic_enabled(false, true);
    set_distance_estimator_enabled(false, true);
    set_gap_detector_enabled(false);
  }

  void fail_test_real_motion(const std::string & reason)
  {
    publish_test_real_log("failed: " + reason);
    stop_test_real_motion_controls();
    reset_test_real_scan_runtime();
    mission_active_ = false;
    test_gap_scan_error_reason_ = reason;
    set_test_real_state(State::TEST_ERROR, reason);
  }

  void start_test_gap_scan_callback(
    const std::shared_ptr<wheeltec_inventory_system::srv::StartTestGapScan::Request> request,
    std::shared_ptr<wheeltec_inventory_system::srv::StartTestGapScan::Response> response)
  {
    publish_test_scan_log(
      "received test request run_all_configured=" +
      std::string(request->run_all_configured ? "true" : "false"));

    if (mission_active_) {
      response->success = false;
      response->message = "正式任务正在运行，拒绝测试盘库任务";
      return;
    }
    if (test_gap_scan_active_) {
      response->success = false;
      response->message = "测试盘库任务已在运行";
      return;
    }

    std::string load_reason;
    test_gap_scan_config_loaded_ = load_test_gap_scan_config(load_reason);
    if (!test_gap_scan_config_loaded_) {
      response->success = false;
      response->message = load_reason;
      return;
    }
    if (!enable_test_gap_scan_) {
      response->success = false;
      response->message = "enable_test_gap_scan=false，测试盘库入口未启用";
      return;
    }

    if (should_start_overall_test(*request)) {
      const auto sequence = overall_test_sequence_from_request(*request);
      std::string overall_reason;
      if (!start_overall_test_sequence(sequence, overall_reason)) {
        response->success = false;
        response->message = overall_reason;
        publish_overall_test_log(response->message);
        return;
      }
      response->success = true;
      response->message =
        "整体盘库测试队列已接收 sequence=" + cabinet_unit_to_string(overall_test_sequence_);
      return;
    }

    if (test_real_motion_enabled_) {
      publish_test_real_log("real motion enabled");
      if (test_real_side_row_enabled_ && is_valid_test_real_side_row_request(*request)) {
        TestGapScanPlan plan;
        std::string side_row_reason;
        if (!start_test_real_side_row_context(*request, plan, side_row_reason)) {
          response->success = false;
          response->message = side_row_reason;
          publish_test_real_log(response->message);
          return;
        }

        test_gap_scan_queue_ = {plan};
        test_current_gap_index_ = 0;
        test_gap_scan_error_reason_.clear();
        test_gap_scan_active_ = true;
        test_real_motion_active_ = true;
        test_real_gap_searching_ = false;
        test_real_target_recognized_logged_ = false;
        test_real_close_requested_ = false;
        test_real_final_recognition_wait_start_ =
          rclcpp::Time(0, 0, get_clock()->get_clock_type());

        publish_test_real_log(
          "[side_row] accepted " + test_real_side_row_name_ +
          " gap=" + plan.gap_id +
          " cabinets=" + cabinet_unit_to_string(plan.scan_cabinets));
        set_test_state(
          State::TEST_REQUESTING_OPEN_GAP,
          "测试真实运动侧排任务已启动 gap=" + plan.gap_id +
          " cabinets=" + cabinet_unit_to_string(plan.scan_cabinets));

        response->success = true;
        response->message = "测试真实运动侧排任务已接收";
        return;
      }

      if (!is_valid_test_real_motion_request(*request)) {
        response->success = false;
        response->message =
          test_real_side_row_enabled_ ? side_row_reject_message() : real_motion_reject_message();
        publish_test_real_log(response->message);
        return;
      }

      reset_test_real_side_row_context();
      TestGapScanPlan plan;
      plan.gap_id = test_real_motion_target_gap_;
      plan.scan_cabinets = {test_real_motion_target_cabinet_};
      test_gap_scan_queue_ = {plan};
      test_current_gap_index_ = 0;
      test_gap_scan_error_reason_.clear();
      test_gap_scan_active_ = true;
      test_real_motion_active_ = true;
      test_real_gap_searching_ = false;
      test_real_target_recognized_logged_ = false;
      test_real_close_requested_ = false;
      test_real_final_recognition_wait_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

      publish_test_real_log(
        "accepted single cabinet test gap=" + plan.gap_id +
        " cabinet=" + std::to_string(test_real_motion_target_cabinet_));
      set_test_state(
        State::TEST_REQUESTING_OPEN_GAP,
        "测试真实运动任务已启动 gap=" + plan.gap_id +
        " cabinets=" + cabinet_unit_to_string(plan.scan_cabinets));

      response->success = true;
      response->message = "测试真实运动任务已接收";
      return;
    }

    std::vector<TestGapScanPlan> plans;
    std::string reason;
    if (!build_test_gap_scan_plans(*request, plans, reason)) {
      response->success = false;
      response->message = reason;
      return;
    }

    test_gap_scan_queue_ = plans;
    test_current_gap_index_ = 0;
    test_gap_scan_error_reason_.clear();
    test_gap_scan_active_ = true;

    set_test_state(
      State::TEST_REQUESTING_OPEN_GAP,
      "测试盘库空跑任务已启动 gaps=" + std::to_string(test_gap_scan_queue_.size()));

    response->success = true;
    response->message = "测试盘库空跑任务已接收";
  }

  void start_service_callback(
    const std::shared_ptr<wheeltec_inventory_system::srv::StartMission::Request> request,
    std::shared_ptr<wheeltec_inventory_system::srv::StartMission::Response> response)
  {
    if (mission_active_ || test_gap_scan_active_) {
      response->accepted = false;
      response->message = "任务已在运行";
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

    std::string reason;
    if (!load_configs_and_prepare_current_target(reason)) {
      response->accepted = false;
      response->message = reason;
      return;
    }

    mission_active_ = true;
    cancel_requested_ = false;
    mission_return_home_on_finish_ = request->return_home || return_home_on_finish_;

    target_visible_ = false;
    has_distance_ = false;
    latest_distance_ = 0.0;

    return_mode_ = ReturnMode::NONE;
    return_target_distance_ = 0.0;
    return_using_nav2_ = false;
    nav2_return_in_progress_ = false;
    nav2_result_ready_ = false;
    fallback_phase_ = FallbackPhase::IDLE;
    fallback_drive_mode_ = FallbackDriveMode::NONE;
    reset_wait_gap_runtime();
    reset_entry_gap_runtime();
    publish_gap_context();

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

    if (test_gap_scan_active_) {
      if (overall_test_active_) {
        fail_overall_test("收到取消请求，停止整体盘库测试");
        response->message = "整体盘库测试已请求停止";
      } else if (test_real_motion_active_) {
        fail_test_real_motion("收到取消请求，停止测试真实运动");
        response->message = "测试真实运动已请求停止";
      } else {
        fail_test_gap_scan("收到取消请求，停止测试盘库空跑");
        response->message = "测试盘库空跑已请求停止";
      }
      response->success = true;
      return;
    }

    if (!mission_active_ && state_ == State::IDLE) {
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
    switch_to_returning(ReturnMode::CANCEL_HOME, "收到取消任务，立即停车并返回起点");

    response->success = true;
    response->message = "任务已取消，正在返回起点";
  }

  void handle_returning_state()
  {
    if (return_using_nav2_) {
      if (nav2_return_in_progress_) {
        if ((this->now() - nav2_goal_sent_time_).seconds() > nav2_goal_timeout_sec_) {
          cancel_nav2_return_goal("Nav2 返航超时");
          Pose2D target_pose;
          std::string mode_text;
          if (resolve_return_pose(return_mode_, target_pose, mode_text)) {
            (void)begin_fallback_return(return_mode_, target_pose, "Nav2 超时，切换兜底");
          } else {
            mission_active_ = false;
            set_state(State::ERROR, "Nav2 超时且无法切换兜底");
          }
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

        publish_log(nav2_result_text_ + "，切换兜底返航");
        Pose2D target_pose;
        std::string mode_text;
        if (resolve_return_pose(return_mode_, target_pose, mode_text)) {
          (void)begin_fallback_return(return_mode_, target_pose, "Nav2 失败，切换兜底");
        } else {
          mission_active_ = false;
          set_state(State::ERROR, "Nav2 失败且无法切换兜底");
        }
      }
      return;
    }

    run_fallback_return();
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
    entry_gap_phase_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    entry_turn_start_yaw_ = 0.0;
    target_gap_yaw_ = 0.0;
    straight_start_pose_ = Pose2D{};
    entry_last_traveled_ = 0.0;
    entry_turn_completed_ = false;
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
        if (overall_test_active_) {
          fail_overall_test("无法生成找缝规划: " + reason);
        } else if (test_real_motion_active_) {
          fail_test_real_motion("无法生成找缝规划: " + reason);
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
        if (overall_test_active_) {
          fail_overall_test("无法生成入缝深度规划: " + reason);
        } else if (test_real_motion_active_) {
          fail_test_real_motion("无法生成入缝深度规划: " + reason);
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
      if (overall_test_active_) {
        fail_overall_test("SEARCH_GAP超时参数非法: " + timeout_reason);
      } else if (test_real_motion_active_) {
        fail_test_real_motion("SEARCH_GAP超时参数非法: " + timeout_reason);
      } else {
        mission_active_ = false;
        publish_stop();
        set_state(State::ERROR, "SEARCH_GAP超时参数非法: " + timeout_reason);
      }
      return;
    }

    latest_gap_ = wheeltec_inventory_system::msg::GapStatus{};
    search_gap_start_ = this->now();
    reset_segment_distance();
    publish_entry_side();
    request_recognizer_enable(false);
    set_recognizer_topic_enabled(false, true);
    set_distance_estimator_enabled(false, true);
    const std::string detail =
      "跟踪稳定，按物理单元找缝 direction=" +
      search_direction_to_string(current_gap_plan_.search_direction) +
      " gap=" + gap_plan_to_string(current_gap_plan_);
    if (overall_test_active_) {
      test_real_gap_searching_ = false;
      publish_overall_test_log(
        "search gap target=" + std::to_string(current_target_cabinet_) +
        " direction=" + search_direction_to_string(current_gap_plan_.search_direction));
      set_state(State::OVERALL_TEST_SEARCH_GAP, "[OVERALL_TEST] " + detail);
    } else if (test_real_motion_active_) {
      test_real_gap_searching_ = true;
      publish_test_real_log("waiting/searching gap=" + active_test_real_gap_id());
      set_test_real_state(State::TEST_REAL_WAITING_GAP, detail);
    } else {
      set_state(State::SEARCH_GAP, detail);
    }
    publish_gap_context();
  }

  void begin_waiting_gap_confirmation_flow(const std::string & detail)
  {
    latest_gap_ = wheeltec_inventory_system::msg::GapStatus{};
    set_wait_gap_phase(WaitGapPhase::STOP_BEFORE_DETECT);
    publish_entry_side();
    if (overall_test_active_) {
      test_real_gap_searching_ = false;
      begin_overall_post_gap_detect_advance_flow();
      return;
    } else if (test_real_motion_active_) {
      test_real_gap_searching_ = false;
      set_test_real_state(State::TEST_REAL_WAITING_GAP, detail);
    } else {
      set_state(State::WAITING_GAP, detail);
    }
    publish_gap_context();
  }

  void begin_waiting_gap_fallback_flow(const std::string & detail)
  {
    wait_gap_motion_target_distance_ = std::abs(post_track_retreat_distance_);
    wait_gap_motion_direction_ = post_track_retreat_distance_ >= 0.0 ? -1.0 : 1.0;
    latest_gap_ = wheeltec_inventory_system::msg::GapStatus{};
    reset_segment_distance();
    set_wait_gap_phase(WaitGapPhase::RETREATING);
    publish_entry_side();
    if (overall_test_active_) {
      test_real_gap_searching_ = false;
      fail_overall_test("SEARCH_GAP legacy fallback bypassed for overall_test: " + detail);
      return;
    } else if (test_real_motion_active_) {
      test_real_gap_searching_ = false;
      set_test_real_state(State::TEST_REAL_WAITING_GAP, detail);
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
    if (!std::isfinite(entry_turn_yaw_delta_rad_) || std::abs(entry_turn_yaw_delta_rad_) <= 1e-4) {
      reason = "entry_turn_yaw_delta_rad 必须为非零有限值";
      return false;
    }
    if (!std::isfinite(entry_align_yaw_tolerance_rad_) || entry_align_yaw_tolerance_rad_ <= 0.0) {
      reason = "entry_align_yaw_tolerance_rad 必须为正数";
      return false;
    }
    if (!std::isfinite(entry_turn_linear_speed_) || entry_turn_linear_speed_ < 0.0) {
      reason = "entry_turn_linear_speed 不能为负数";
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

  void fail_entering_gap(const std::string & reason)
  {
    publish_stop();
    if (overall_test_active_) {
      fail_overall_test(reason);
      return;
    }
    if (test_real_motion_active_) {
      fail_test_real_motion(reason);
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

    reset_entry_gap_runtime();
    request_recognizer_enable(false);
    set_recognizer_topic_enabled(false, true);
    set_distance_estimator_enabled(false, true);
    set_gap_detector_enabled(false);
    entry_turn_start_yaw_ = current.yaw;
    const double signed_delta = current_entry_side_ == "right" ?
      -std::abs(entry_turn_yaw_delta_rad_) : std::abs(entry_turn_yaw_delta_rad_);
    target_gap_yaw_ = normalize_angle(entry_turn_start_yaw_ + signed_delta);
    reset_segment_distance();
    set_entry_gap_phase(
      EntryGapPhase::ENTERING_TURN,
      "entry_turn_start_yaw=" + std::to_string(entry_turn_start_yaw_) +
      " target_gap_yaw=" + std::to_string(target_gap_yaw_) +
      " direction=" + entry_turn_direction_text());
    const std::string state_detail =
      detail + "，开始转入缝隙 target_straight_distance=" +
      std::to_string(target_straight_distance_) + "m";
    if (overall_test_active_) {
      publish_overall_test_log(
        "entering gap for cabinet=" + std::to_string(current_target_cabinet_));
      set_state(State::OVERALL_TEST_ENTERING_GAP, "[OVERALL_TEST] " + state_detail);
    } else if (test_real_motion_active_) {
      publish_test_real_log("entering gap for cabinet=" + std::to_string(current_target_cabinet_));
      set_test_real_state(State::TEST_REAL_ENTERING_GAP, state_detail);
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
    if (current_gap_plan_.search_direction == SearchDirection::BACKWARD) {
      return search_gap_backward_timeout_sec_;
    }
    return search_gap_forward_timeout_sec_;
  }

  void begin_overall_post_gap_detect_advance_flow()
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
      publish_overall_test_log(
        "gap detected stable, post advance disabled, entering gap target=" +
        std::to_string(current_target_cabinet_));
      begin_entering_gap_flow("gap detected stable, post advance disabled");
      return;
    }

    overall_test_post_gap_advance_start_ = this->now();
    reset_segment_distance();
    publish_overall_test_log(
      "gap detected stable, post advance direction=" +
      search_direction_to_string(current_gap_plan_.search_direction) +
      " distance=" + format_seconds(post_gap_detect_advance_distance_m_));
    set_state(
      State::OVERALL_TEST_POST_GAP_DETECT_ADVANCE,
      "[OVERALL_TEST] gap detected stable, post advance direction=" +
      search_direction_to_string(current_gap_plan_.search_direction) +
      " distance=" + format_seconds(post_gap_detect_advance_distance_m_));
  }

  void handle_overall_post_gap_detect_advance_state()
  {
    request_recognizer_enable(false);
    set_recognizer_topic_enabled(false);
    set_distance_estimator_enabled(false);
    set_gap_detector_enabled(false);

    if (overall_test_post_gap_advance_start_.nanoseconds() == 0) {
      overall_test_post_gap_advance_start_ = this->now();
      reset_segment_distance();
    }

    const double timeout =
      std::isfinite(post_gap_detect_advance_timeout_sec_) ?
      std::max(0.1, post_gap_detect_advance_timeout_sec_) : 8.0;
    const double elapsed = (this->now() - overall_test_post_gap_advance_start_).seconds();
    if (elapsed >= timeout) {
      publish_stop();
      fail_overall_test(
        "post gap advance timeout target=" +
        std::to_string(current_target_cabinet_) +
        " elapsed=" + format_seconds(elapsed) +
        " timeout=" + format_seconds(timeout));
      return;
    }

    const double direction =
      current_gap_plan_.search_direction == SearchDirection::BACKWARD ? -1.0 : 1.0;
    const bool done = run_wait_gap_linear_motion(
      direction,
      post_gap_detect_advance_speed_,
      std::abs(post_gap_detect_advance_distance_m_));
    if (done) {
      overall_test_post_gap_advance_start_ =
        rclcpp::Time(0, 0, get_clock()->get_clock_type());
      publish_overall_test_log(
        "post gap advance done, entering gap target=" +
        std::to_string(current_target_cabinet_));
      begin_entering_gap_flow("post gap advance done");
      return;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "[mission_manager][OVERALL_TEST] post_gap_advance target=%d direction=%s traveled=%.2f/%.2f "
      "speed=%.3f elapsed=%.2f/%.2f",
      current_target_cabinet_,
      search_direction_to_string(current_gap_plan_.search_direction).c_str(),
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
      if (overall_test_active_) {
        begin_overall_post_gap_detect_advance_flow();
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
      if (overall_test_active_) {
        fail_overall_test(
          "SEARCH_GAP timeout target=" +
          std::to_string(current_target_cabinet_) +
          " direction=" + search_direction_to_string(current_gap_plan_.search_direction) +
          " timeout=" + format_seconds(timeout_sec) +
          " elapsed=" + format_seconds(elapsed_sec));
        return;
      }
      begin_waiting_gap_fallback_flow(
        "SEARCH_GAP超时未检测到目标间隙，执行原固定回退序列作为fallback微调 direction=" +
        search_direction_to_string(current_gap_plan_.search_direction) +
        " timeout=" + format_seconds(timeout_sec) +
        " elapsed=" + format_seconds(elapsed_sec));
      return;
    }

    const double direction =
      current_gap_plan_.search_direction == SearchDirection::BACKWARD ? -1.0 : 1.0;
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = direction * std::abs(search_gap_speed_);
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "search_gap: target_cabinet=%d target_side=%s entry_side=%s gap_search_direction=%s "
      "timeout_sec=%.2f elapsed_sec=%.2f expected_gap=%s cmd.linear.x=%.3f",
      current_target_cabinet_,
      current_target_side_.c_str(),
      current_entry_side_.c_str(),
      search_direction_to_string(current_gap_plan_.search_direction).c_str(),
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
              if (overall_test_active_) {
                fail_overall_test("缝隙检测失败且调整序列耗尽");
              } else if (test_real_motion_active_) {
                fail_test_real_motion("缝隙检测失败且调整序列耗尽");
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
    return
      (current.x - straight_start_pose_.x) * std::cos(target_gap_yaw_) +
      (current.y - straight_start_pose_.y) * std::sin(target_gap_yaw_);
  }

  double entry_turn_timeout_sec() const
  {
    const double min_angular = std::max(0.05, std::abs(entry_turn_angular_speed_));
    return std::max(5.0, 2.5 * std::abs(entry_turn_yaw_delta_rad_) / min_angular + 2.0);
  }

  double entry_straight_timeout_sec() const
  {
    const double min_speed = std::max(0.03, std::abs(entry_straight_speed_));
    return std::max(8.0, 2.5 * target_straight_distance_ / min_speed + 3.0);
  }

  void log_entering_gap_status(
    const EnteringSafetyEval & safety,
    const Pose2D & current,
    double yaw_error,
    double angular_z,
    double traveled,
    bool turn_done,
    bool straight_done,
    const EntrySideHoldEval & side_hold)
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
      "entering_gap: phase=%s entry_turn_start_yaw=%.3f target_gap_yaw=%.3f "
      "current_yaw=%.3f yaw_error=%.3f angular.z=%.3f straight_start_pose=(%s) "
      "traveled=%.3f target_straight_distance=%.3f turn_done=%d straight_done=%d "
      "entry_side=%s left_side_dist=%.3f right_side_dist=%.3f control_side_dist=%.3f "
      "side_error=%.3f yaw_hold_cmd=%.3f side_distance_cmd=%.3f final_angular_cmd=%.3f "
      "side_hold_status=%s side_points[left=%zu,right=%zu] safety_stop=%d safety_reason=%s "
      "front=%.3f front_side=%.3f side_dist=%.3f speed_scale=%.2f",
      entry_gap_phase_to_string(entry_gap_phase_).c_str(),
      entry_turn_start_yaw_,
      target_gap_yaw_,
      current.yaw,
      yaw_error,
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
    bool straight_done)
  {
    EntrySideHoldEval side_hold;
    log_entering_gap_status(
      safety, current, yaw_error, angular_z, traveled, turn_done, straight_done, side_hold);
  }

  void handle_entering_gap_state()
  {
    std::string reason;
    if (!current_odom_ready_for_entry(reason)) {
      fail_entering_gap("入缝里程计异常: " + reason);
      return;
    }

    Pose2D current = current_pose_2d();
    if (!current.valid || !std::isfinite(current.x) || !std::isfinite(current.y) ||
      !std::isfinite(current.yaw))
    {
      fail_entering_gap("入缝里程计位姿无效");
      return;
    }

    if (entry_gap_phase_ == EntryGapPhase::IDLE) {
      begin_entering_gap_flow("ENTERING_GAP运行时补初始化");
      return;
    }

    const auto safety = evaluate_entering_safety();
    const double yaw_error = normalize_angle(target_gap_yaw_ - current.yaw);
    double traveled = straight_start_pose_.valid ? entry_straight_traveled(current) : 0.0;
    double angular_z = 0.0;
    bool turn_done = entry_turn_completed_;
    bool straight_done = entry_straight_completed_;

    if (safety.blocked) {
      entry_stopped_by_safety_ = true;
      log_entering_gap_status(safety, current, yaw_error, angular_z, traveled, turn_done, straight_done);
      fail_entering_gap("入缝被安全策略阻塞: " + safety.block_reason);
      return;
    }

    switch (entry_gap_phase_) {
      case EntryGapPhase::ENTERING_TURN: {
        turn_done =
          std::abs(normalize_angle(current.yaw - target_gap_yaw_)) <
          entry_align_yaw_tolerance_rad_;
        if (turn_done) {
          entry_turn_completed_ = true;
          publish_stop();
          log_entering_gap_status(safety, current, yaw_error, angular_z, traveled, true, false);
          set_entry_gap_phase(EntryGapPhase::ENTERING_STRAIGHT_ALIGN, "转向完成，停止持续转向");
          return;
        }

        if ((this->now() - entry_gap_phase_start_).seconds() > entry_turn_timeout_sec()) {
          log_entering_gap_status(safety, current, yaw_error, angular_z, traveled, false, false);
          fail_entering_gap("入缝转向超时，未达到目标缝隙航向");
          return;
        }

        const double turn_direction = current_entry_side_ == "right" ? -1.0 : 1.0;
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = std::abs(entry_turn_linear_speed_) * safety.speed_scale;
        angular_z = turn_direction * std::abs(entry_turn_angular_speed_) * safety.speed_scale;
        cmd.angular.z = angular_z;
        cmd_pub_->publish(cmd);
        log_entering_gap_status(safety, current, yaw_error, angular_z, traveled, false, false);
        break;
      }

      case EntryGapPhase::ENTERING_STRAIGHT_ALIGN: {
        publish_stop();
        straight_start_pose_ = current;
        straight_start_pose_.yaw = current.yaw;
        entry_last_traveled_ = 0.0;
        traveled = 0.0;
        log_entering_gap_status(safety, current, yaw_error, angular_z, traveled, true, false);
        set_entry_gap_phase(
          EntryGapPhase::MOVING_TO_GRID_CENTER,
          "记录 straight_start_pose，开始直行到深度格中心");
        break;
      }

      case EntryGapPhase::MOVING_TO_GRID_CENTER: {
        if (!straight_start_pose_.valid) {
          fail_entering_gap("入缝直行起点无效");
          return;
        }

        traveled = entry_straight_traveled(current);
        if (!std::isfinite(traveled) || traveled < -0.15) {
          log_entering_gap_status(safety, current, yaw_error, angular_z, traveled, true, false);
          fail_entering_gap("入缝直行里程异常，traveled=" + std::to_string(traveled));
          return;
        }

        if ((this->now() - entry_gap_phase_start_).seconds() > entry_straight_timeout_sec()) {
          log_entering_gap_status(safety, current, yaw_error, angular_z, traveled, true, false);
          fail_entering_gap("入缝直行超时，未到达深度格中心");
          return;
        }

        if (traveled >= target_straight_distance_) {
          entry_straight_completed_ = true;
          straight_done = true;
          publish_stop();
          log_entering_gap_status(safety, current, yaw_error, angular_z, traveled, true, true);
          if (overall_test_active_) {
            test_real_last_entering_straight_distance_ =
              std::max(traveled, target_straight_distance_);
            set_state(
              State::OVERALL_TEST_SCAN_PLACEHOLDER,
              "[OVERALL_TEST] reached depth-grid center, start placeholder scan cabinet=" +
              std::to_string(current_target_cabinet_));
          } else if (test_real_motion_active_) {
            test_real_last_entering_straight_distance_ =
              std::max(traveled, target_straight_distance_);
            State scan_state = State::TEST_REAL_IN_GAP_SCAN;
            if (test_real_side_row_active_) {
              if (test_real_side_row_phase_ == TestRealSideRowPhase::FIRST_ADJUSTED_SCAN ||
                test_real_side_row_phase_ == TestRealSideRowPhase::SECOND_ADJUSTED_SCAN)
              {
                scan_state = State::TEST_REAL_ADJUSTED_SIDE_SCAN;
              } else if (test_real_side_row_phase_ == TestRealSideRowPhase::SECOND_PRIMARY_SCAN) {
                scan_state = State::TEST_REAL_NEXT_GAP_SCAN;
              }
            }
            set_test_real_state(
              scan_state,
              "已直行到目标深度格中心，开始测试占位扫描");
          } else {
            set_state(State::INVENTORYING, "已直行到目标深度格中心，盘库流程预留");
          }
          return;
        }

        entry_last_traveled_ = traveled;
        EntrySideHoldEval side_hold;
        double yaw_hold_cmd = 0.0;
        if (std::abs(yaw_error) >= entry_straight_yaw_deadband_rad_) {
          yaw_hold_cmd = std::clamp(
            entry_straight_yaw_kp_ * yaw_error,
            -std::abs(entry_straight_max_angular_speed_),
            std::abs(entry_straight_max_angular_speed_));
        }
        (void)evaluate_entry_side_distance_hold(side_hold);
        side_hold.yaw_hold_cmd = yaw_hold_cmd;

        const double angular_limit = std::abs(entry_straight_max_angular_speed_);
        const double limited_angular = std::clamp(
          yaw_hold_cmd + side_hold.side_distance_cmd,
          -angular_limit,
          angular_limit);

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = std::abs(entry_straight_speed_) * safety.speed_scale;
        cmd.angular.z = limited_angular * safety.speed_scale;
        angular_z = cmd.angular.z;
        side_hold.final_angular_cmd = angular_z;
        cmd_pub_->publish(cmd);
        log_entering_gap_status(safety, current, yaw_error, angular_z, traveled, true, false, side_hold);
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
      if (overall_test_active_) {
        fail_overall_test("跟踪阶段目标丢失，安全停车并终止整体盘库测试");
      } else if (test_real_motion_active_) {
        fail_test_real_motion("跟踪阶段目标丢失，安全停车并终止测试真实运动");
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

  bool execute_placeholder_scan_for_cabinet(int cabinet_id, bool test_real_log)
  {
    if (test_real_log) {
      publish_test_real_log("in gap, start placeholder scan cabinet=" + std::to_string(cabinet_id));
    } else {
      publish_test_scan_log("scanning cabinet=" + std::to_string(cabinet_id));
    }

    const auto steps = scan_sequence_generator_.generateCabinetSnakeSequence(
      cabinet_id,
      test_scan_layers_,
      test_scan_depth_count_);
    if (steps.empty()) {
      const std::string reason = "生成扫描序列为空: cabinet=" + std::to_string(cabinet_id);
      if (test_real_log) {
        fail_test_real_motion(reason);
      } else {
        fail_test_gap_scan(reason);
      }
      return false;
    }

    bool report_ok = true;
    const bool execute_ok = scan_sequence_executor_.execute(
      steps,
      [this, &report_ok](const wheeltec_inventory_system::ScanStep & step) {
        if (step.step_type == wheeltec_inventory_system::ScanStepType::MOVE_TO_GRID) {
          execute_test_real_grid_move_if_enabled(step);
          return;
        }
        if (step.step_type != wheeltec_inventory_system::ScanStepType::SCAN_PLACEHOLDER) {
          return;
        }
        if (!web_api_client_.reportInventoryResult(
            step.cabinet_id,
            step.layer_index,
            step.depth_index,
            "placeholder_ok"))
        {
          report_ok = false;
        }
      });

    if (!execute_ok || !report_ok) {
      const std::string reason =
        "占位扫描执行或结果上报失败: cabinet=" + std::to_string(cabinet_id);
      if (test_real_log) {
        fail_test_real_motion(reason);
      } else {
        fail_test_gap_scan(reason);
      }
      return false;
    }
    return true;
  }

  void handle_test_gap_scan_state()
  {
    const TestGapScanPlan * plan = current_test_gap_scan_plan();

    switch (state_) {
      case State::TEST_REQUESTING_OPEN_GAP: {
        if (plan == nullptr) {
          set_test_state(State::TEST_DONE, "没有待执行测试 gap");
          return;
        }

        publish_test_scan_log(
          "start gap=" + plan->gap_id +
          " cabinets=" + cabinet_unit_to_string(plan->scan_cabinets));
        publish_test_scan_log("requesting open gap=" + plan->gap_id);
        if (test_real_motion_active_) {
          publish_test_real_log("mock open gap=" + plan->gap_id);
        }
        (void)web_api_client_.reportRobotStatus("TEST_REQUESTING_OPEN_GAP");
        if (!web_api_client_.requestOpenGap(plan->gap_id)) {
          fail_test_gap_scan("mock/网页开柜请求失败: gap=" + plan->gap_id);
          return;
        }

        publish_test_scan_log(
          "waiting open delay " + format_seconds(test_open_gap_wait_sec_) + " sec");
        set_test_state(
          State::TEST_WAITING_OPEN_DELAY,
          "等待开柜延时 " + format_seconds(test_open_gap_wait_sec_) + " sec");
        break;
      }

      case State::TEST_WAITING_OPEN_DELAY: {
        if (test_state_elapsed(test_open_gap_wait_sec_)) {
          if (test_real_motion_active_) {
            const int target_cabinet = active_test_real_scan_cabinet();
            set_test_real_state(
              State::TEST_REAL_PREPARE_NAV,
              "prepare nav target_cabinet=" + std::to_string(target_cabinet));
          } else {
            set_test_state(State::TEST_SCANNING_PLACEHOLDER, "开始占位扫描");
          }
        }
        break;
      }

      case State::TEST_SCANNING_PLACEHOLDER: {
        if (plan == nullptr) {
          fail_test_gap_scan("当前测试 gap 为空");
          return;
        }

        (void)web_api_client_.reportRobotStatus("SCANNING_PLACEHOLDER");
        for (const auto cabinet_id : plan->scan_cabinets) {
          if (!execute_placeholder_scan_for_cabinet(cabinet_id, false)) {
            return;
          }
        }

        set_test_state(State::TEST_REQUESTING_CLOSE_GAP, "当前 gap 占位扫描完成");
        break;
      }

      case State::TEST_REQUESTING_CLOSE_GAP: {
        if (plan == nullptr) {
          fail_test_gap_scan("当前测试 gap 为空，无法请求关柜");
          return;
        }

        const std::string close_gap_id =
          test_real_motion_active_ ? active_test_real_gap_id() : plan->gap_id;
        publish_test_scan_log("requesting close gap=" + close_gap_id);
        if (test_real_motion_active_) {
          publish_test_real_log("mock close gap=" + close_gap_id);
        }
        (void)web_api_client_.reportRobotStatus("TEST_REQUESTING_CLOSE_GAP");
        if (!web_api_client_.requestCloseGap(close_gap_id)) {
          if (test_real_motion_active_) {
            fail_test_real_motion("mock/网页关柜请求失败: gap=" + close_gap_id);
          } else {
            fail_test_gap_scan("mock/网页关柜请求失败: gap=" + close_gap_id);
          }
          return;
        }

        publish_test_scan_log(
          "waiting close delay " + format_seconds(test_close_gap_wait_sec_) + " sec");
        set_test_state(
          State::TEST_WAITING_CLOSE_DELAY,
          "等待关柜延时 " + format_seconds(test_close_gap_wait_sec_) + " sec");
        break;
      }

      case State::TEST_WAITING_CLOSE_DELAY: {
        if (!test_state_elapsed(test_close_gap_wait_sec_)) {
          break;
        }
        if (plan != nullptr) {
          const std::string finished_gap_id =
            test_real_motion_active_ ? active_test_real_gap_id() : plan->gap_id;
          publish_test_scan_log("finished gap=" + finished_gap_id);
        }

        if (test_current_gap_index_ + 1 < test_gap_scan_queue_.size()) {
          set_test_state(State::TEST_NEXT_GAP, "切换下一个测试 gap");
        } else {
          set_test_state(State::TEST_DONE, "全部测试 gap 已完成");
        }
        break;
      }

      case State::TEST_NEXT_GAP: {
        if (test_current_gap_index_ + 1 >= test_gap_scan_queue_.size()) {
          set_test_state(State::TEST_DONE, "没有更多测试 gap");
          return;
        }
        ++test_current_gap_index_;
        set_test_state(State::TEST_REQUESTING_OPEN_GAP, "开始下一个测试 gap");
        break;
      }

      case State::TEST_REAL_PREPARE_NAV: {
        const int target_cabinet = active_test_real_scan_cabinet();
        publish_test_real_log(
          "prepare nav target_cabinet=" + std::to_string(target_cabinet));
        if (test_real_side_row_active_) {
          publish_test_real_side_row_log(
            "first gap=" + test_real_active_gap_id_ +
            " scan cabinet=" + std::to_string(target_cabinet));
        }

        std::string reason;
        if (!prepare_test_real_target_cabinet(target_cabinet, reason)) {
          fail_test_real_motion("准备真实运动目标失败: " + reason);
          return;
        }

        mission_active_ = true;
        cancel_requested_ = false;
        mission_return_home_on_finish_ = false;
        target_visible_ = false;
        has_distance_ = false;
        latest_distance_ = 0.0;
        return_mode_ = ReturnMode::NONE;
        return_target_distance_ = 0.0;
        return_using_nav2_ = false;
        nav2_return_in_progress_ = false;
        nav2_result_ready_ = false;
        fallback_phase_ = FallbackPhase::IDLE;
        fallback_drive_mode_ = FallbackDriveMode::NONE;
        reset_wait_gap_runtime();
        reset_entry_gap_runtime();
        publish_gap_context();
        tracking_stable_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        last_target_seen_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        mission_start_distance_ = odom_cumulative_distance_;
        reset_segment_distance();
        mission_start_pose_ = current_pose_2d();
        mission_start_pose_nav2_ = Pose2D{};

        publish_test_real_log(
          "enter real nav to target cabinet=" + std::to_string(current_target_cabinet_));
        publish_test_real_log(
          "requested gap=" + active_test_real_gap_id() +
          " existing_gap_plan=" + gap_plan_to_string(current_gap_plan_));

        std::string nav_route_fail_reason;
        if (!begin_nav_route_for_current_target(
            "[mission_manager][test_real] enter real nav to target cabinet=" +
            std::to_string(current_target_cabinet_),
            nav_route_fail_reason,
            State::TEST_REAL_NAV_TO_TARGET))
        {
          fail_test_real_motion(nav_route_fail_reason);
        }
        break;
      }

      case State::TEST_REAL_IN_GAP_SCAN: {
        handle_test_real_scan_state();
        break;
      }

      case State::TEST_REAL_ADJUSTED_SIDE_SCAN: {
        handle_test_real_scan_state();
        break;
      }

      case State::TEST_REAL_NEXT_GAP_SCAN: {
        handle_test_real_scan_state();
        break;
      }

      case State::TEST_REAL_EXIT_GAP:
      case State::TEST_REAL_FINAL_EXIT_GAP: {
        handle_test_real_exit_gap_state();
        break;
      }

      case State::TEST_REAL_REENTER_FOR_ADJUSTED_SCAN: {
        handle_test_real_reenter_adjusted_scan_state();
        break;
      }

      case State::TEST_REAL_CORRIDOR_TRANSFER: {
        handle_test_real_corridor_transfer_state();
        break;
      }

      case State::TEST_REAL_PREPARE_NEXT_GAP: {
        handle_test_real_prepare_next_gap_state();
        break;
      }

      case State::TEST_REAL_REENTER_NEXT_GAP: {
        handle_test_real_reenter_next_gap_state();
        break;
      }

      case State::TEST_REAL_STOP_AFTER_SCAN: {
        stop_test_real_motion_controls();
        mission_active_ = false;
        publish_test_real_log("scan finished, stop after scan, no exit motion in this test step");
        if (!test_real_close_requested_) {
          publish_test_real_log("mock close gap=" + test_real_motion_target_gap_);
          (void)web_api_client_.requestCloseGap(test_real_motion_target_gap_);
          test_real_close_requested_ = true;
        }
        publish_test_real_log("done, back to IDLE");
        set_test_state(State::TEST_DONE, "测试真实运动单柜扫描完成");
        break;
      }

      case State::TEST_DONE: {
        (void)web_api_client_.reportRobotStatus("TEST_DONE");
        if (overall_test_active_) {
          publish_overall_test_log("done, back to IDLE");
        } else if (test_real_motion_active_) {
          publish_test_real_log("done, back to IDLE");
        } else {
          publish_test_scan_log("all test gaps finished");
        }
        mission_active_ = false;
        test_real_motion_active_ = false;
        test_real_gap_searching_ = false;
        test_real_target_recognized_logged_ = false;
        test_real_close_requested_ = false;
        test_real_final_recognition_wait_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        if (test_real_side_row_active_) {
          publish_test_real_log("[return] side-row real motion completed");
        }
        reset_test_real_side_row_context();
        reset_overall_test_context();
        test_gap_scan_active_ = false;
        test_gap_scan_queue_.clear();
        test_current_gap_index_ = 0;
        set_state(State::IDLE, "测试盘库任务完成，系统待机");
        break;
      }

      case State::TEST_ERROR: {
        (void)web_api_client_.reportRobotStatus("TEST_ERROR");
        if (overall_test_active_) {
          stop_test_real_motion_controls();
          publish_overall_test_log("overall test failed: " + test_gap_scan_error_reason_);
        } else if (test_real_motion_active_) {
          stop_test_real_motion_controls();
          publish_test_real_log("test real motion failed: " + test_gap_scan_error_reason_);
        } else {
          publish_test_scan_log("test gap scan failed: " + test_gap_scan_error_reason_);
        }
        mission_active_ = false;
        test_real_motion_active_ = false;
        test_real_gap_searching_ = false;
        test_real_target_recognized_logged_ = false;
        test_real_close_requested_ = false;
        test_real_final_recognition_wait_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        reset_test_real_side_row_context();
        reset_overall_test_context();
        test_gap_scan_active_ = false;
        test_gap_scan_queue_.clear();
        test_current_gap_index_ = 0;
        set_state(State::IDLE, "测试盘库空跑已停止，系统待机");
        break;
      }

      case State::TEST_IDLE:
      default:
        break;
    }
  }

  void on_timer()
  {
    if (!mission_active_ && !test_gap_scan_active_) {
      return;
    }

    switch (state_) {
      case State::TEST_IDLE:
      case State::TEST_REQUESTING_OPEN_GAP:
      case State::TEST_WAITING_OPEN_DELAY:
      case State::TEST_SCANNING_PLACEHOLDER:
      case State::TEST_REQUESTING_CLOSE_GAP:
      case State::TEST_WAITING_CLOSE_DELAY:
      case State::TEST_NEXT_GAP:
      case State::TEST_DONE:
      case State::TEST_ERROR:
      case State::TEST_REAL_PREPARE_NAV:
      case State::TEST_REAL_IN_GAP_SCAN:
      case State::TEST_REAL_STOP_AFTER_SCAN:
      case State::TEST_REAL_EXIT_GAP:
      case State::TEST_REAL_REENTER_FOR_ADJUSTED_SCAN:
      case State::TEST_REAL_ADJUSTED_SIDE_SCAN:
      case State::TEST_REAL_CORRIDOR_TRANSFER:
      case State::TEST_REAL_PREPARE_NEXT_GAP:
      case State::TEST_REAL_REENTER_NEXT_GAP:
      case State::TEST_REAL_NEXT_GAP_SCAN:
      case State::TEST_REAL_FINAL_EXIT_GAP: {
        handle_test_gap_scan_state();
        break;
      }

      case State::OVERALL_TEST_PREPARE_TARGET: {
        publish_stop();
        break;
      }

      case State::OVERALL_TEST_NAV_TO_OBSERVE: {
        handle_nav_route_state();
        break;
      }

      case State::OVERALL_TEST_POST_ROUTE_RECOGNITION_WAIT: {
        handle_overall_test_post_route_recognition_wait_state();
        break;
      }

      case State::OVERALL_TEST_RECOGNITION_FALLBACK: {
        handle_overall_recognition_fallback_state();
        break;
      }

      case State::OVERALL_TEST_TARGET_DISTANCE_ALIGN: {
        handle_overall_target_distance_align_state();
        break;
      }

      case State::OVERALL_TEST_SEARCH_GAP: {
        handle_search_gap_state();
        break;
      }

      case State::OVERALL_TEST_POST_GAP_DETECT_ADVANCE: {
        handle_overall_post_gap_detect_advance_state();
        break;
      }

      case State::OVERALL_TEST_SAME_SIDE_NEXT_SEARCH: {
        handle_overall_same_side_next_search_state();
        break;
      }

      case State::OVERALL_TEST_ENTERING_GAP: {
        handle_entering_gap_state();
        break;
      }

      case State::OVERALL_TEST_SCAN_PLACEHOLDER: {
        handle_overall_test_scan_state();
        break;
      }

      case State::OVERALL_TEST_EXIT_GAP: {
        handle_test_real_exit_gap_state();
        break;
      }

      case State::OVERALL_TEST_ADVANCE_NEXT_TARGET:
      case State::OVERALL_TEST_RETURN_HOME_BETWEEN_SIDES:
      case State::OVERALL_TEST_DONE: {
        publish_stop();
        break;
      }

      case State::TEST_REAL_NAV_TO_TARGET: {
        handle_nav_route_state();
        break;
      }

      case State::TEST_REAL_TARGET_TRACKING: {
        handle_target_tracking_state();
        break;
      }

      case State::TEST_REAL_FINAL_RECOGNITION_WAIT: {
        publish_stop();
        request_recognizer_enable(true);
        if (test_real_final_recognition_wait_start_.nanoseconds() == 0) {
          test_real_final_recognition_wait_start_ = this->now();
        }
        if ((this->now() - test_real_final_recognition_wait_start_).seconds() >=
          std::max(0.0, test_real_final_recognition_wait_sec_))
        {
          fail_test_real_motion(
            "final recognition timeout, target cabinet=" +
            std::to_string(current_target_cabinet_) + " not recognized");
        }
        break;
      }

      case State::TEST_REAL_WAITING_GAP: {
        if (test_real_gap_searching_) {
          handle_search_gap_state();
        } else {
          handle_waiting_gap_state();
        }
        break;
      }

      case State::TEST_REAL_ENTERING_GAP: {
        handle_entering_gap_state();
        break;
      }

      case State::NAV_ROUTE: {
        handle_nav_route_state();
        break;
      }

      case State::CORRIDOR_NAV: {
        if (segment_distance() >= warehouse_length_) {
          switch_to_returning(ReturnMode::SEARCH_TARGET, "到达仓库末端未发现目标，开始掉头返回");
        }
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

      case State::RETURNING: {
        handle_returning_state();
        break;
      }

      case State::IDLE:
      case State::DONE:
      case State::ERROR:
        if (overall_test_active_) {
          fail_overall_test("正式流程进入 ERROR，停止整体盘库测试");
        } else if (test_real_motion_active_) {
          fail_test_real_motion("正式流程进入 ERROR，停止测试真实运动");
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
  std::string corridor_enable_topic_;
  std::string corridor_reverse_topic_;
  std::string gap_context_topic_;
  std::string recognizer_enable_topic_;
  std::string gap_detector_enable_topic_;
  std::string entry_side_topic_;
  std::string target_lidar_side_topic_;
  std::string distance_estimator_enable_topic_;

  std::string start_service_name_;
  std::string cancel_service_name_;
  std::string recognizer_trigger_service_;
  std::string start_test_gap_scan_service_name_;

  std::vector<std::string> target_list_param_;
  std::string route_waypoints_file_{"config/route_waypoints.yaml"};
  std::string warehouse_layout_file_{"config/warehouse_layout.yaml"};
  std::string test_gap_scan_params_file_{"config/test_gap_scan_params.yaml"};
  std::string gap_scan_map_file_{"config/gap_scan_map.yaml"};
  std::string route_search_failure_policy_{"error"};
  std::map<std::string, RouteConfig> route_configs_;
  std::map<std::string, std::string> side_route_map_;
  std::map<int, std::string> cabinet_side_map_;
  std::map<int, std::string> cabinet_entry_side_map_;
  std::map<int, SearchDirection> cabinet_gap_search_direction_map_;
  std::map<std::string, WarehouseRowLayout> warehouse_rows_by_side_;
  std::vector<TestGapScanPlan> configured_test_inventory_plan_{
    TestGapScanPlan{"gap_03_02", std::vector<int>{3, 2, 1}},
    TestGapScanPlan{"gap_04_05", std::vector<int>{4, 5, 6}}};
  std::map<std::string, std::vector<int>> test_inventory_plan_by_gap_id_{
    {"gap_03_02", std::vector<int>{3, 2, 1}},
    {"gap_04_05", std::vector<int>{4, 5, 6}}};
  std::map<std::string, std::vector<int>> test_gap_scan_map_cabinets_by_gap_id_{
    {"gap_03_02", std::vector<int>{3, 2, 1}},
    {"gap_04_05", std::vector<int>{4, 5, 6}}};
  RouteConfig current_route_;
  TargetGapPlan current_gap_plan_;
  std::string current_route_name_;
  std::string current_target_side_{"left"};
  std::string current_entry_side_{"left"};
  bool routes_loaded_{false};
  bool warehouse_layout_loaded_{false};
  bool test_gap_scan_config_loaded_{false};
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
  double entry_speed_{0.08};
  double entry_distance_{0.70};
  double turn_speed_{0.40};  // 兼容保留
  double enter_linear_speed_{0.08};
  double enter_angular_speed_{0.30};
  double enter_left_angular_multiplier_{1.0};
  double enter_right_angular_multiplier_{-1.0};
  bool enable_grid_center_entry_{true};
  double grid_depth_m_{2.4};
  int left_max_depth_index_{4};
  int right_max_depth_index_{3};
  double entry_center_offset_m_{0.0};
  double target_depth_center_m_{1.2};
  double target_straight_distance_{1.2};
  bool current_entry_profile_valid_{false};
  double entry_turn_yaw_delta_rad_{1.57079632679};
  double entry_align_yaw_tolerance_rad_{0.08};
  double entry_turn_linear_speed_{0.08};
  double entry_turn_angular_speed_{0.30};
  double entry_straight_speed_{0.08};
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

  bool return_home_on_finish_{false};
  std::string return_target_mode_{"start"};
  double charge_pose_x_{0.0};
  double charge_pose_y_{0.0};
  double charge_pose_yaw_{0.0};
  std::string charge_pose_frame_id_{"odom_combined"};
  bool use_nav2_return_{true};
  std::string nav2_action_name_{"navigate_to_pose"};
  std::string nav2_goal_frame_{"map"};
  double nav2_server_wait_timeout_sec_{1.0};
  double nav2_goal_timeout_sec_{40.0};
  double nav2_route_waypoint_timeout_sec_{60.0};
  double nav2_cancel_stop_duration_sec_{0.50};
  bool nav2_enable_for_search_return_{false};
  double fallback_rotate_kp_{1.5};
  double fallback_rotate_max_angular_{0.8};
  double fallback_rotate_tolerance_rad_{0.08};
  double fallback_rotate_stable_time_sec_{0.25};
  double fallback_drive_speed_{0.20};
  double fallback_heading_kp_{1.2};
  double fallback_drive_max_angular_{0.8};
  double fallback_goal_tolerance_m_{0.18};
  bool continue_on_error_{false};
  double control_rate_hz_{10.0};

  bool use_scan_safety_{true};
  double entry_front_window_deg_{22.0};
  double entry_front_stop_distance_{0.30};
  double max_scan_age_sec_{0.8};

  bool use_ultrasonic_safety_{true};
  std::vector<std::string> ultrasonic_topics_;
  double entry_ultrasonic_stop_distance_{0.25};
  double max_ultrasonic_age_sec_{0.8};
  double entry_left_align_distance_{0.22};
  double entry_left_turn_angular_{0.35};

  bool enable_test_gap_scan_{true};
  std::string test_web_client_mode_{"mock"};
  double test_open_gap_wait_sec_{5.0};
  double test_close_gap_wait_sec_{3.0};
  double test_scan_placeholder_wait_sec_{2.0};
  double test_lift_placeholder_wait_sec_{3.0};
  double test_move_grid_placeholder_wait_sec_{1.0};
  std::string test_motion_mode_{"ackermann_reentry_test"};
  bool test_real_motion_enabled_{false};
  bool test_real_motion_single_cabinet_only_{true};
  int test_real_motion_target_cabinet_{3};
  std::string test_real_motion_target_gap_{"gap_03_02"};
  bool test_real_motion_stop_after_scan_{true};
  double test_real_final_recognition_wait_sec_{8.0};
  bool test_real_side_row_enabled_{false};
  std::string test_real_side_row_name_{"row_01_02_03_04"};
  std::string test_real_side_row_first_gap_{"gap_02_03_04"};
  std::vector<int> test_real_side_row_first_gap_scan_sequence_{4, 3};
  std::string test_real_side_row_second_gap_{"gap_01_02_03"};
  std::vector<int> test_real_side_row_second_gap_scan_sequence_{2, 1};
  bool test_real_side_row_corridor_transfer_enabled_{true};
  int test_real_side_row_corridor_transfer_target_cabinet_{2};
  std::string test_real_side_row_corridor_transfer_direction_{"toward_cabinet_1"};
  bool test_real_exit_after_each_scan_{true};
  std::string test_real_exit_mode_{"reverse"};
  double test_real_exit_speed_{0.05};
  double test_real_exit_extra_distance_m_{0.10};
  double test_real_exit_timeout_sec_{40.0};
  double test_real_exit_distance_m_{1.20};
  bool test_real_exit_turn_enabled_{true};
  double test_real_exit_turn_angular_speed_{0.25};
  double test_real_exit_turn_yaw_tolerance_rad_{0.08};
  double test_real_exit_turn_timeout_sec_{8.0};
  bool test_real_reentry_for_position_adjustment_{true};
  std::string test_real_reentry_comment_{
    "Use reverse-exit and re-enter as a temporary replacement for in-gap orientation/position adjustment."};
  bool test_real_grid_motion_enabled_{false};
  double test_real_grid_spacing_m_{0.30};
  double test_real_grid_move_speed_{0.04};
  double test_real_grid_move_timeout_sec_{10.0};
  bool test_real_grid_move_return_between_layers_{false};
  bool test_real_close_gap_after_final_exit_{true};
  bool overall_test_enabled_{true};
  std::vector<int> overall_test_sequence_{4, 3, 8, 7};
  std::string overall_test_left_route_{"left_route"};
  std::string overall_test_right_route_{"right_route"};
  bool overall_test_return_home_between_sides_{true};
  bool overall_test_return_home_after_done_{true};
  bool overall_test_recognize_during_nav_{false};
  bool overall_test_same_side_next_search_enabled_{true};
  double overall_test_same_side_search_speed_{0.04};
  double overall_test_same_side_search_timeout_sec_{20.0};
  bool overall_test_same_side_pose_hold_enabled_{true};
  double overall_test_same_side_fixed_y_m_{0.575};
  double overall_test_same_side_fixed_yaw_rad_{-3.1400};
  double overall_test_same_side_yaw_kp_{0.40};
  double overall_test_same_side_yaw_deadband_rad_{0.03};
  double overall_test_same_side_y_kp_{0.30};
  double overall_test_same_side_y_deadband_m_{0.03};
  double overall_test_same_side_y_correction_sign_{1.0};
  double overall_test_same_side_max_angular_{0.15};
  double overall_test_final_recognition_wait_sec_{5.0};
  bool overall_test_recognition_fallback_enabled_{true};
  double overall_test_recognition_fallback_speed_{0.04};
  double overall_test_recognition_fallback_wait_sec_{2.0};
  double overall_test_recognition_fallback_timeout_sec_{20.0};
  std::vector<double> overall_test_recognition_fallback_sequence_{-0.30, 0.60, -0.30};
  bool post_gap_detect_advance_enabled_{true};
  double post_gap_detect_advance_distance_m_{0.25};
  double post_gap_detect_advance_speed_{0.04};
  double post_gap_detect_advance_timeout_sec_{8.0};
  int test_scan_layers_{2};
  int test_scan_depth_count_{3};
  std::string test_default_scan_side_{"left"};
  std::string test_web_base_url_;
  std::string test_web_open_gap_endpoint_{"/api/gap/open"};
  std::string test_web_close_gap_endpoint_{"/api/gap/close"};
  std::string test_web_status_endpoint_{"/api/robot/status"};
  std::string test_web_result_endpoint_{"/api/inventory/result"};
  bool test_gap_scan_active_{false};
  std::vector<TestGapScanPlan> test_gap_scan_queue_;
  std::size_t test_current_gap_index_{0};
  std::string test_gap_scan_error_reason_;
  rclcpp::Time test_state_enter_time_{0, 0, RCL_ROS_TIME};
  bool test_real_motion_active_{false};
  bool test_real_gap_searching_{false};
  bool test_real_target_recognized_logged_{false};
  bool test_real_close_requested_{false};
  rclcpp::Time test_real_final_recognition_wait_start_{0, 0, RCL_ROS_TIME};
  bool test_real_side_row_active_{false};
  bool test_real_side_row_full_sequence_{false};
  std::vector<int> test_real_side_row_requested_sequence_;
  TestRealSideRowPhase test_real_side_row_phase_{TestRealSideRowPhase::NONE};
  TestRealAfterExitAction test_real_after_exit_action_{TestRealAfterExitAction::NONE};
  std::string test_real_active_gap_id_;
  int test_real_current_scan_cabinet_{-1};
  int test_real_adjusted_scan_cabinet_{-1};
  int test_real_next_gap_target_cabinet_{-1};
  double test_real_last_entering_straight_distance_{0.0};
  double test_real_exit_target_distance_{1.20};
  double test_real_exit_effective_timeout_sec_{40.0};
  rclcpp::Time test_real_exit_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time test_real_exit_phase_start_time_{0, 0, RCL_ROS_TIME};
  TestRealExitPhase test_real_exit_phase_{TestRealExitPhase::STRAIGHT_REVERSE};
  std::vector<wheeltec_inventory_system::ScanStep> test_real_scan_steps_;
  std::size_t test_real_scan_step_index_{0};
  int test_real_scan_cabinet_{-1};
  bool test_real_scan_active_{false};
  rclcpp::Time test_real_scan_step_start_time_{0, 0, RCL_ROS_TIME};
  bool test_real_grid_have_previous_depth_{false};
  int test_real_grid_previous_cabinet_{-1};
  int test_real_grid_previous_layer_{-1};
  int test_real_grid_previous_depth_{-1};
  bool test_real_grid_move_active_{false};
  wheeltec_inventory_system::ScanStep test_real_grid_move_step_;
  Pose2D test_real_grid_move_start_pose_;
  rclcpp::Time test_real_grid_move_start_time_{0, 0, RCL_ROS_TIME};
  double test_real_grid_move_target_distance_{0.0};
  double test_real_grid_move_cmd_speed_{0.0};
  bool overall_test_active_{false};
  std::size_t overall_test_index_{0};
  int overall_test_current_target_{-1};
  int overall_test_next_target_{-1};
  std::string overall_test_current_side_;
  std::string overall_test_current_route_;
  OverallTestReturnReason overall_test_return_reason_{OverallTestReturnReason::NONE};
  bool overall_test_waiting_return_home_for_side_switch_{false};
  bool overall_test_waiting_return_home_for_done_{false};
  rclcpp::Time overall_test_same_side_search_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time overall_test_final_recognition_wait_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time overall_test_recognition_fallback_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time overall_test_recognition_fallback_phase_start_{0, 0, RCL_ROS_TIME};
  OverallRecognitionFallbackPhase overall_test_recognition_fallback_phase_{
    OverallRecognitionFallbackPhase::IDLE};
  std::size_t overall_test_recognition_fallback_index_{0};
  rclcpp::Time overall_test_post_gap_advance_start_{0, 0, RCL_ROS_TIME};
  wheeltec_inventory_system::ScanSequenceGenerator scan_sequence_generator_;
  wheeltec_inventory_system::ScanSequenceExecutor scan_sequence_executor_;
  wheeltec_inventory_system::WebApiClient web_api_client_;

  State state_{State::IDLE};
  ReturnMode return_mode_{ReturnMode::NONE};

  bool mission_active_{false};
  bool cancel_requested_{false};
  bool mission_return_home_on_finish_{false};
  bool recognizer_enabled_{false};
  bool recognizer_enabled_cmd_{false};
  bool recognizer_enable_initialized_{false};
  bool gap_detector_enabled_cmd_{false};
  bool gap_detector_enable_initialized_{false};
  bool distance_estimator_enabled_cmd_{false};
  bool distance_estimator_enable_initialized_{false};
  bool return_using_nav2_{false};
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

  wheeltec_inventory_system::msg::GapStatus latest_gap_;
  wheeltec_inventory_system::msg::RecognizedNumber::SharedPtr latest_recognition_;
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
  double return_target_distance_{0.0};
  double fallback_target_yaw_{0.0};
  double fallback_target_x_{0.0};
  double fallback_target_y_{0.0};
  double fallback_target_distance_{0.0};
  FallbackDriveMode fallback_drive_mode_{FallbackDriveMode::NONE};
  FallbackPhase fallback_phase_{FallbackPhase::IDLE};
  rclcpp::Time fallback_rotate_stable_start_{0, 0, RCL_ROS_TIME};
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
  rclcpp::Time entry_gap_phase_start_{0, 0, RCL_ROS_TIME};
  double entry_turn_start_yaw_{0.0};
  double target_gap_yaw_{0.0};
  Pose2D straight_start_pose_;
  double entry_last_traveled_{0.0};
  bool entry_turn_completed_{false};
  bool entry_straight_completed_{false};
  bool entry_stopped_by_safety_{false};

  rclcpp::Time tracking_stable_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_target_seen_time_{0, 0, RCL_ROS_TIME};

  std::vector<double> ultrasonic_ranges_;
  std::vector<rclcpp::Time> ultrasonic_stamps_;

  rclcpp::Subscription<wheeltec_inventory_system::msg::RecognizedNumber>::SharedPtr recognized_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_sub_;
  rclcpp::Subscription<wheeltec_inventory_system::msg::GapStatus>::SharedPtr gap_sub_;
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
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr recognizer_enable_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gap_detector_enable_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr distance_estimator_enable_pub_;

  rclcpp::Service<wheeltec_inventory_system::srv::StartMission>::SharedPtr start_srv_;
  rclcpp::Service<wheeltec_inventory_system::srv::StartTestGapScan>::SharedPtr
    start_test_gap_scan_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_srv_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr recognizer_trigger_client_;
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
