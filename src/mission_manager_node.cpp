#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
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
#include "wheeltec_inventory_system/srv/start_mission.hpp"
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

    start_service_name_ = declare_parameter<std::string>("start_service_name", "/inventory/start_mission");
    cancel_service_name_ = declare_parameter<std::string>("cancel_service_name", "/inventory/cancel_mission");
    recognizer_trigger_service_ =
      declare_parameter<std::string>("recognizer_trigger_service", "/inventory/trigger_recognition");

    target_list_param_ = declare_parameter<std::vector<std::string>>("target_list", std::vector<std::string>{});
    route_waypoints_file_ =
      declare_parameter<std::string>("route_waypoints_file", "config/route_waypoints.yaml");
    warehouse_layout_file_ =
      declare_parameter<std::string>("warehouse_layout_file", "config/warehouse_layout.yaml");
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
    search_gap_timeout_sec_ = declare_parameter<double>("search_gap_timeout_sec", 4.0);
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

    recognized_sub_ = create_subscription<wheeltec_inventory_system::msg::RecognizedNumber>(
      recognized_topic_,
      10,
      std::bind(&MissionManagerNode::recognized_callback, this, std::placeholders::_1));

    distance_sub_ = create_subscription<std_msgs::msg::Float32>(
      distance_topic_,
      10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        latest_distance_ = static_cast<double>(msg->data);
        has_distance_ = true;
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
    recognizer_enable_pub_ =
      create_publisher<std_msgs::msg::Bool>(recognizer_enable_topic_, control_qos);
    gap_detector_enable_pub_ =
      create_publisher<std_msgs::msg::Bool>(gap_detector_enable_topic_, control_qos);

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
    std::string target_side{"left"};
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

  bool load_route_config(std::string & reason)
  {
    reason.clear();
    std::map<std::string, RouteConfig> routes;
    std::map<std::string, std::string> side_route_map;
    std::map<int, std::string> cabinet_side_map;

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

      for (const auto map_item : cabinet_side_root) {
        int cabinet_id = -1;
        const std::string cabinet_text = map_item.first.as<std::string>();
        if (!wheeltec_inventory_system::safe_to_int(cabinet_text, cabinet_id)) {
          reason = "cabinet_side_map 货柜号非法: " + cabinet_text;
          return false;
        }

        if (!map_item.second.IsScalar()) {
          reason = "cabinet_side_map 条目必须直接映射到 left/right: " + cabinet_text;
          return false;
        }
        std::string target_side;
        if (!try_normalize_entry_side(map_item.second.as<std::string>(), target_side)) {
          reason = "cabinet_side_map side 必须为 left/right: " + cabinet_text;
          return false;
        }
        cabinet_side_map[cabinet_id] = target_side;
      }

      route_configs_ = routes;
      side_route_map_ = side_route_map;
      cabinet_side_map_ = cabinet_side_map;
      RCLCPP_INFO(
        get_logger(),
        "已加载巡航路线配置: %s routes=%zu side_route_map=%zu cabinet_side_map=%zu",
        route_path.string().c_str(),
        route_configs_.size(),
        side_route_map_.size(),
        cabinet_side_map_.size());
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

      if (cabinet_index + 1 == unit.size() && has_next) {
        plan.search_direction = SearchDirection::FORWARD;
        plan.gap_before_unit = unit;
        plan.gap_after_unit = units[unit_index + 1];
      } else if (cabinet_index == 0 && has_prev) {
        plan.search_direction = SearchDirection::BACKWARD;
        plan.gap_before_unit = units[unit_index - 1];
        plan.gap_after_unit = unit;
      } else if (has_next) {
        plan.search_direction = SearchDirection::FORWARD;
        plan.gap_before_unit = unit;
        plan.gap_after_unit = units[unit_index + 1];
      } else if (has_prev) {
        plan.search_direction = SearchDirection::BACKWARD;
        plan.gap_before_unit = units[unit_index - 1];
        plan.gap_after_unit = unit;
      } else {
        reason = "目标货柜所在物理单元没有相邻间隙: " + std::to_string(current_target_cabinet_);
        return false;
      }

      current_gap_plan_ = plan;
      RCLCPP_INFO(
        get_logger(),
        "目标货柜%d找缝规划: side=%s physical_unit=%s expected_gap=%s search_direction=%s",
        current_target_cabinet_,
        current_gap_plan_.side.c_str(),
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
    return current_target_side_ == "right" ? "right(angular.z<0)" : "left(angular.z>0)";
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
      "目标任务解析: target_cabinet=%d target_side=%s target_level_index=%d "
      "target_depth_index=%d target_depth_center_m=%.3f entry_center_offset_m=%.3f "
      "target_straight_distance=%.3f physical_unit=%s expected_gap=%s "
      "search_direction=%s entry_turn_direction=%s grid_center_entry=%s",
      current_target_cabinet_,
      current_target_side_.c_str(),
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
    msg.data = current_target_side_;
    entry_side_pub_->publish(msg);
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
      enabled ? "ENABLE(IDLE/NAV_ROUTE/TARGET_TRACKING)" : "DISABLE(停止识别)",
      force_publish ? " [force]" : "");
  }

  void set_state(State s, const std::string & detail)
  {
    // SEARCH_GAP/WAITING_GAP 阶段启用 gap 检测，其他状态全部停掉。
    set_gap_detector_enabled(s == State::SEARCH_GAP || s == State::WAITING_GAP);
    // 识别启停严格由状态机控制：
    // IDLE/NAV_ROUTE/TARGET_TRACKING 开启；进入找缝/回退观测后及后续状态关闭。
    set_recognizer_topic_enabled(
      s == State::IDLE || s == State::NAV_ROUTE || s == State::TARGET_TRACKING);
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
    RCLCPP_INFO(
      get_logger(),
      "目标货柜%d解析: route=%s side=%s waypoints=%zu",
      current_target_cabinet_,
      current_route_name_.c_str(),
      current_target_side_.c_str(),
      current_route_.waypoints.size());
    publish_entry_side();
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

  void recognized_callback(
    const wheeltec_inventory_system::msg::RecognizedNumber::SharedPtr msg)
  {
    latest_recognition_ = msg;
    latest_recognition_time_ = this->now();
    if (!mission_active_) {
      return;
    }
    if (!msg->valid) {
      if (state_ == State::NAV_ROUTE) {
        reset_target_recognition_stability();
      }
      return;
    }

    int rec_id = -1;
    if (!wheeltec_inventory_system::safe_to_int(msg->number, rec_id)) {
      return;
    }

    if (rec_id != current_target_cabinet_) {
      if (state_ == State::NAV_ROUTE) {
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

    if (state_ == State::NAV_ROUTE) {
      if (update_target_recognition_stability(rec_id)) {
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
      set_state(State::TARGET_TRACKING, "识别到目标货柜，进入跟踪");
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
    eval.target_side = current_target_side_;
    eval.target_distance = entry_side_hold_target_distance_m_;

    if (!latest_scan_ || (this->now() - latest_scan_stamp_).seconds() > max_scan_age_sec_) {
      eval.status = "NO_FRESH_SCAN";
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "entry side distance hold disabled: target_side=%s reason=%s",
        eval.target_side.c_str(),
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

    const bool target_is_right = eval.target_side == "right";
    const std::size_t target_points = target_is_right ?
      eval.right_valid_points : eval.left_valid_points;
    eval.control_side_dist = target_is_right ? eval.right_side_dist : eval.left_side_dist;
    const int required_points = std::max(1, entry_side_hold_min_valid_points_);

    if (!std::isfinite(eval.control_side_dist) ||
      target_points < static_cast<std::size_t>(required_points))
    {
      eval.status = target_is_right ? "INSUFFICIENT_RIGHT_POINTS" : "INSUFFICIENT_LEFT_POINTS";
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "entry side distance hold disabled: target_side=%s reason=%s left_side_dist=%.3f "
        "right_side_dist=%.3f left_points=%zu right_points=%zu required_points=%d",
        eval.target_side.c_str(),
        eval.status.c_str(),
        eval.left_side_dist,
        eval.right_side_dist,
        eval.left_valid_points,
        eval.right_valid_points,
        required_points);
      return false;
    }

    eval.side_error = eval.control_side_dist - eval.target_distance;
    eval.side_distance_cmd = target_is_right ?
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
    eval.active_side = current_target_side_;

    if (use_scan_safety_) {
      if (!latest_scan_ || (this->now() - latest_scan_stamp_).seconds() > max_scan_age_sec_) {
        eval.blocked = true;
        eval.speed_scale = 0.0;
        eval.block_reason = "NO_FRESH_SCAN";
        return eval;
      }

      eval.front_min_dist = min_scan_range_in_sector(
        *latest_scan_, enter_front_sector_start_deg_, enter_front_sector_end_deg_);
      if (current_target_side_ == "right") {
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
          eval.block_reason = current_target_side_ == "right" ? "BLOCKED_FRONT_RIGHT" : "BLOCKED_FRONT_LEFT";
        } else {
          eval.block_reason = current_target_side_ == "right" ? "BLOCKED_RIGHT_SIDE" : "BLOCKED_LEFT_SIDE";
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

  bool begin_nav_route_for_current_target(const std::string & detail, std::string & fail_reason)
  {
    fail_reason.clear();
    if (current_route_.waypoints.empty()) {
      fail_reason = "当前巡航路线没有waypoints";
      return false;
    }

    reset_nav_route_runtime();
    publish_entry_side();
    set_corridor_mode(false, false);
    request_recognizer_enable(true);
    has_distance_ = false;
    target_visible_ = false;
    set_state(State::NAV_ROUTE, detail);
    set_recognizer_topic_enabled(true, true);

    if (!send_current_route_waypoint(fail_reason)) {
      mission_active_ = false;
      publish_stop();
      request_recognizer_enable(false);
      set_state(State::ERROR, "启动巡航路线失败: " + fail_reason);
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
        set_state(State::TARGET_TRACKING, "停车完成，进入目标跟踪");
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

  void start_service_callback(
    const std::shared_ptr<wheeltec_inventory_system::srv::StartMission::Request> request,
    std::shared_ptr<wheeltec_inventory_system::srv::StartMission::Response> response)
  {
    if (mission_active_) {
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
    routes_loaded_ = load_route_config(reason);
    if (!routes_loaded_) {
      response->accepted = false;
      response->message = reason;
      return;
    }
    warehouse_layout_loaded_ = load_warehouse_layout_config(reason);
    if (!warehouse_layout_loaded_) {
      response->accepted = false;
      response->message = reason;
      return;
    }

    if (!prepare_current_target(reason)) {
      response->accepted = false;
      response->message = reason;
      return;
    }

    if (!resolve_current_route(reason)) {
      response->accepted = false;
      response->message = reason;
      return;
    }
    if (!configure_current_entry_profile(reason)) {
      response->accepted = false;
      response->message = reason;
      return;
    }
    if (!resolve_current_gap_plan(reason)) {
      response->accepted = false;
      response->message = reason;
      return;
    }
    log_current_target_entry_plan();

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
        mission_active_ = false;
        publish_stop();
        set_state(State::ERROR, "无法生成找缝规划: " + reason);
        return;
      }
    }
    if (!current_entry_profile_valid_) {
      std::string reason;
      if (!configure_current_entry_profile(reason)) {
        mission_active_ = false;
        publish_stop();
        set_state(State::ERROR, "无法生成入缝深度规划: " + reason);
        return;
      }
      log_current_target_entry_plan();
    }

    latest_gap_ = wheeltec_inventory_system::msg::GapStatus{};
    search_gap_start_ = this->now();
    reset_segment_distance();
    publish_entry_side();
    set_state(
      State::SEARCH_GAP,
      "跟踪稳定，按物理单元找缝 direction=" +
      search_direction_to_string(current_gap_plan_.search_direction) +
      " gap=" + gap_plan_to_string(current_gap_plan_));
    publish_gap_context();
  }

  void begin_waiting_gap_confirmation_flow(const std::string & detail)
  {
    latest_gap_ = wheeltec_inventory_system::msg::GapStatus{};
    set_wait_gap_phase(WaitGapPhase::STOP_BEFORE_DETECT);
    publish_entry_side();
    set_state(State::WAITING_GAP, detail);
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
    set_state(State::WAITING_GAP, detail);
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
    entry_turn_start_yaw_ = current.yaw;
    const double signed_delta = current_target_side_ == "right" ?
      -std::abs(entry_turn_yaw_delta_rad_) : std::abs(entry_turn_yaw_delta_rad_);
    target_gap_yaw_ = normalize_angle(entry_turn_start_yaw_ + signed_delta);
    reset_segment_distance();
    set_entry_gap_phase(
      EntryGapPhase::ENTERING_TURN,
      "entry_turn_start_yaw=" + std::to_string(entry_turn_start_yaw_) +
      " target_gap_yaw=" + std::to_string(target_gap_yaw_) +
      " direction=" + entry_turn_direction_text());
    set_state(
      State::ENTERING_GAP,
      detail + "，开始转入缝隙 target_straight_distance=" +
      std::to_string(target_straight_distance_) + "m");
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

  void handle_search_gap_state()
  {
    publish_entry_side();
    publish_gap_context();

    const std::string detected_side = latest_gap_.active_side.empty() ?
      latest_gap_.side : latest_gap_.active_side;
    if (latest_gap_.allow_enter && normalize_entry_side(detected_side) == current_target_side_) {
      publish_stop();
      begin_waiting_gap_confirmation_flow("SEARCH_GAP检测到候选间隙，停车确认宽度/稳定性/安全距离");
      return;
    }

    if (search_gap_start_.nanoseconds() == 0) {
      search_gap_start_ = this->now();
    }

    if ((this->now() - search_gap_start_).seconds() >= std::max(0.1, search_gap_timeout_sec_)) {
      publish_stop();
      begin_waiting_gap_fallback_flow(
        "SEARCH_GAP超时未检测到目标间隙，执行原固定回退序列作为fallback微调");
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
      "search_gap: side=%s direction=%s expected_gap=%s speed=%.3f",
      current_target_side_.c_str(),
      search_direction_to_string(current_gap_plan_.search_direction).c_str(),
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
        if (latest_gap_.allow_enter && normalize_entry_side(detected_side) == current_target_side_) {
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
              set_state(State::ERROR, "缝隙检测失败且调整序列耗尽");
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
    const std::string target_side_text = side_hold.status == "NOT_EVALUATED" ?
      current_target_side_ : side_hold.target_side;

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "entering_gap: phase=%s entry_turn_start_yaw=%.3f target_gap_yaw=%.3f "
      "current_yaw=%.3f yaw_error=%.3f angular.z=%.3f straight_start_pose=(%s) "
      "traveled=%.3f target_straight_distance=%.3f turn_done=%d straight_done=%d "
      "target_side=%s left_side_dist=%.3f right_side_dist=%.3f control_side_dist=%.3f "
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
      target_side_text.c_str(),
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

        const double turn_direction = current_target_side_ == "right" ? -1.0 : 1.0;
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
          set_state(State::INVENTORYING, "已直行到目标深度格中心，盘库流程预留");
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

  void on_timer()
  {
    if (!mission_active_) {
      return;
    }

    switch (state_) {
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
        if (!target_visible_ || !recognition_is_fresh() || !latest_recognition_) {
          mission_active_ = false;
          target_visible_ = false;
          has_distance_ = false;
          set_corridor_mode(false, false);
          publish_stop();
          request_recognizer_enable(false);
          set_state(State::ERROR, "跟踪阶段目标丢失，安全停车并终止任务");
          break;
        }

        if (!has_distance_) {
          break;
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

  std::string start_service_name_;
  std::string cancel_service_name_;
  std::string recognizer_trigger_service_;

  std::vector<std::string> target_list_param_;
  std::string route_waypoints_file_{"config/route_waypoints.yaml"};
  std::string warehouse_layout_file_{"config/warehouse_layout.yaml"};
  std::string route_search_failure_policy_{"error"};
  std::map<std::string, RouteConfig> route_configs_;
  std::map<std::string, std::string> side_route_map_;
  std::map<int, std::string> cabinet_side_map_;
  std::map<std::string, WarehouseRowLayout> warehouse_rows_by_side_;
  RouteConfig current_route_;
  TargetGapPlan current_gap_plan_;
  std::string current_route_name_;
  std::string current_target_side_{"left"};
  bool routes_loaded_{false};
  bool warehouse_layout_loaded_{false};
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
  double search_gap_timeout_sec_{4.0};
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
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr recognizer_enable_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gap_detector_enable_pub_;

  rclcpp::Service<wheeltec_inventory_system::srv::StartMission>::SharedPtr start_srv_;
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
