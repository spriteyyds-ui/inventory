#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "wheeltec_inventory_system/msg/gap_status.hpp"

class GapDetectorNode : public rclcpp::Node
{
public:
  GapDetectorNode()
  : Node("gap_detector_node")
  {
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    output_topic_ = declare_parameter<std::string>("output_topic", "/inventory/gap_status");
    context_topic_ = declare_parameter<std::string>("context_topic", "/inventory/gap_context");
    enable_topic_ = declare_parameter<std::string>("enable_topic", "/inventory/gap_detector_enable");
    entry_side_topic_ = declare_parameter<std::string>("entry_side_topic", "/inventory/entry_side");
    enable_on_start_ = declare_parameter<bool>("enable_on_start", false);
    active_side_ = normalize_entry_side(declare_parameter<std::string>("entry_side", "left"));
    base_link_frame_ = declare_parameter<std::string>("base_link_frame", "base_link");

    left_gap_sector_start_deg_ = declare_parameter<double>("gap_sector_start_deg", 40.0);
    left_gap_sector_end_deg_ = declare_parameter<double>("gap_sector_end_deg", 140.0);
    left_front_veto_sector_start_deg_ = declare_parameter<double>(
      "front_left_veto_sector_start_deg", 10.0);
    left_front_veto_sector_end_deg_ = declare_parameter<double>(
      "front_left_veto_sector_end_deg", 70.0);
    left_side_veto_sector_start_deg_ = declare_parameter<double>(
      "left_side_veto_sector_start_deg", 70.0);
    left_side_veto_sector_end_deg_ = declare_parameter<double>(
      "left_side_veto_sector_end_deg", 120.0);
    left_gap_sector_start_deg_ =
      declare_parameter<double>("left_gap_sector_start_deg", left_gap_sector_start_deg_);
    left_gap_sector_end_deg_ =
      declare_parameter<double>("left_gap_sector_end_deg", left_gap_sector_end_deg_);
    left_front_veto_sector_start_deg_ =
      declare_parameter<double>("left_front_veto_sector_start_deg", left_front_veto_sector_start_deg_);
    left_front_veto_sector_end_deg_ =
      declare_parameter<double>("left_front_veto_sector_end_deg", left_front_veto_sector_end_deg_);

    right_gap_sector_start_deg_ =
      declare_parameter<double>("right_gap_sector_start_deg", -120.0);
    right_gap_sector_end_deg_ =
      declare_parameter<double>("right_gap_sector_end_deg", -60.0);
    right_front_veto_sector_start_deg_ =
      declare_parameter<double>("right_front_veto_sector_start_deg", -70.0);
    right_front_veto_sector_end_deg_ =
      declare_parameter<double>("right_front_veto_sector_end_deg", -10.0);
    right_side_veto_sector_start_deg_ =
      declare_parameter<double>("right_side_veto_sector_start_deg", -120.0);
    right_side_veto_sector_end_deg_ =
      declare_parameter<double>("right_side_veto_sector_end_deg", -70.0);

    min_valid_range_ = declare_parameter<double>("min_valid_range", 0.05);
    max_valid_range_ = declare_parameter<double>("max_valid_range", 8.0);

    open_point_min_distance_ = declare_parameter<double>("open_point_min_distance", 0.50);
    near_obstacle_veto_distance_ = declare_parameter<double>("near_obstacle_veto_distance", 0.50);
    required_entry_width_ = declare_parameter<double>("required_entry_width", 0.40);

    open_run_min_points_ = declare_parameter<int>("open_run_min_points", 3);
    min_valid_points_ = declare_parameter<int>("min_valid_points", 8);
    open_ratio_threshold_ = declare_parameter<double>("open_ratio_threshold", 0.10);
    max_adjacent_angle_gap_deg_ = declare_parameter<double>("max_adjacent_angle_gap_deg", 2.0);
    smoothing_window_ = declare_parameter<int>("smoothing_window", 1);

    stable_frames_required_ = declare_parameter<int>("stable_frames_required", 3);
    detect_cycle_timeout_sec_ = declare_parameter<double>("detect_cycle_timeout_sec", 2.5);

    enable_csv_log_ = declare_parameter<bool>("enable_csv_log", true);
    csv_log_dir_ = declare_parameter<std::string>("csv_log_dir", "/tmp/wheeltec_inventory_gap_logs");
    csv_file_prefix_ = declare_parameter<std::string>("csv_file_prefix", "gap_debug");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    status_pub_ = create_publisher<wheeltec_inventory_system::msg::GapStatus>(output_topic_, 10);

    context_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      context_topic_,
      10,
      std::bind(&GapDetectorNode::context_callback, this, std::placeholders::_1));

    auto control_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      enable_topic_,
      control_qos,
      std::bind(&GapDetectorNode::enable_callback, this, std::placeholders::_1));
    entry_side_sub_ = create_subscription<std_msgs::msg::String>(
      entry_side_topic_,
      control_qos,
      std::bind(&GapDetectorNode::entry_side_callback, this, std::placeholders::_1));

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&GapDetectorNode::scan_callback, this, std::placeholders::_1));

    detector_enabled_ = enable_on_start_;
    init_csv_logger();

    RCLCPP_INFO(
      get_logger(),
      "间隙检测节点已启动 scan=%s output=%s enable_topic=%s entry_side_topic=%s enable_on_start=%d base_link=%s active_side=%s open_thr=%.2f width_req=%.2f",
      scan_topic_.c_str(),
      output_topic_.c_str(),
      enable_topic_.c_str(),
      entry_side_topic_.c_str(),
      detector_enabled_ ? 1 : 0,
      base_link_frame_.c_str(),
      active_side_.c_str(),
      open_point_min_distance_,
      required_entry_width_);
  }

private:
  struct SectorPoint
  {
    double angle_deg{0.0};
    double x{0.0};
    double y{0.0};
    double range{0.0};
    bool open{false};
  };

  struct OpenWindowResult
  {
    bool found{false};
    int open_count{0};
    int max_open_run{0};
    int start_idx{-1};
    int end_idx{-1};
    float open_ratio{0.0F};
    float start_deg{0.0F};
    float end_deg{0.0F};
    float width{0.0F};
  };

  struct FrameEval
  {
    bool gap_candidate_frame{false};
    bool gap_detected{false};
    bool entry_width_ok{false};
    bool front_left_safe{false};
    bool left_side_safe{false};
    bool allow_enter{false};

    int valid_points{0};
    int open_points{0};
    int max_open_run{0};
    int stable_frame_count{0};
    int failed_cycle_count{0};

    float open_ratio{0.0F};
    float estimated_gap_width{0.0F};
    float open_window_start_deg{0.0F};
    float open_window_end_deg{0.0F};
    float min_front_left_dist{std::numeric_limits<float>::infinity()};
    float min_left_side_dist{std::numeric_limits<float>::infinity()};

    std::string side{"none"};
    std::string active_side{"left"};
    std::string debug_reason;
  };

  struct SectorConfig
  {
    double gap_start_deg{60.0};
    double gap_end_deg{120.0};
    double front_veto_start_deg{10.0};
    double front_veto_end_deg{70.0};
    double side_veto_start_deg{70.0};
    double side_veto_end_deg{120.0};
  };

  static double deg2rad(double deg)
  {
    return deg * M_PI / 180.0;
  }

  static bool in_range(double value, double start, double end)
  {
    return value >= std::min(start, end) && value <= std::max(start, end);
  }

  static double normalize_deg(double deg)
  {
    while (deg > 180.0) {
      deg -= 360.0;
    }
    while (deg < -180.0) {
      deg += 360.0;
    }
    return deg;
  }

  static std::string normalize_entry_side(std::string side)
  {
    side.erase(side.begin(), std::find_if(side.begin(), side.end(), [](unsigned char c) {
      return std::isspace(c) == 0;
    }));
    side.erase(std::find_if(side.rbegin(), side.rend(), [](unsigned char c) {
      return std::isspace(c) == 0;
    }).base(), side.end());
    std::transform(side.begin(), side.end(), side.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    if (side == "right" || side == "r") {
      return "right";
    }
    return "left";
  }

  static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  std::string sanitize_frame_id(const std::string & frame) const
  {
    if (!frame.empty() && frame.front() == '/') {
      return frame.substr(1);
    }
    return frame;
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

    const std::string target = sanitize_frame_id(base_link_frame_);
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
        tf_msg = tf_buffer_->lookupTransform(target, source, tf2::TimePointZero, tf2::durationFromSec(0.08));
      } else {
        tf_msg = tf_buffer_->lookupTransform(
          target,
          source,
          rclcpp::Time(scan.header.stamp),
          tf2::durationFromSec(0.08));
      }

      tx = tf_msg.transform.translation.x;
      ty = tf_msg.transform.translation.y;
      yaw = yaw_from_quaternion(tf_msg.transform.rotation);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        get_logger(),
        "TF转换失败: %s -> %s, err=%s",
        source.c_str(), target.c_str(), ex.what());
      return false;
    }
  }

  std::vector<SectorPoint> extract_sector_points(
    const sensor_msgs::msg::LaserScan & scan,
    double sector_start_deg,
    double sector_end_deg,
    int & valid_count) const
  {
    std::vector<SectorPoint> points;
    valid_count = 0;

    double tx = 0.0;
    double ty = 0.0;
    double yaw = 0.0;
    if (!resolve_scan_to_base_transform(scan, tx, ty, yaw)) {
      return points;
    }

    points.reserve(scan.ranges.size());
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);

    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
      const double r = scan.ranges[i];
      if (!std::isfinite(r)) {
        continue;
      }
      if (r < min_valid_range_ || r > max_valid_range_) {
        continue;
      }

      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      const double x_scan = r * std::cos(angle);
      const double y_scan = r * std::sin(angle);

      // 将 scan frame 下的点变换到 base_link。
      const double x_base = cy * x_scan - sy * y_scan + tx;
      const double y_base = sy * x_scan + cy * y_scan + ty;
      const double range_base = std::hypot(x_base, y_base);
      if (range_base < min_valid_range_ || range_base > max_valid_range_) {
        continue;
      }

      const double angle_base_deg = normalize_deg(std::atan2(y_base, x_base) * 180.0 / M_PI);
      if (!in_range(angle_base_deg, sector_start_deg, sector_end_deg)) {
        continue;
      }

      SectorPoint p;
      p.angle_deg = angle_base_deg;
      p.x = x_base;
      p.y = y_base;
      p.range = range_base;
      // 仅把“足够远”的点作为开口候选，近点不参与开口窗口。
      p.open = (range_base > open_point_min_distance_);
      points.push_back(p);
      ++valid_count;
    }

    std::sort(
      points.begin(), points.end(),
      [](const SectorPoint & a, const SectorPoint & b) {
        return a.angle_deg < b.angle_deg;
      });

    return points;
  }

  void apply_light_smoothing(std::vector<SectorPoint> & points) const
  {
    if (points.empty() || smoothing_window_ <= 1) {
      return;
    }

    const int w = std::max(1, smoothing_window_);
    std::vector<double> smoothed(points.size(), 0.0);
    for (std::size_t i = 0; i < points.size(); ++i) {
      int cnt = 0;
      double sum = 0.0;
      const int begin = static_cast<int>(i) - w;
      const int end = static_cast<int>(i) + w;
      for (int j = begin; j <= end; ++j) {
        if (j < 0 || j >= static_cast<int>(points.size())) {
          continue;
        }
        sum += points[static_cast<std::size_t>(j)].range;
        ++cnt;
      }
      smoothed[i] = cnt > 0 ? (sum / static_cast<double>(cnt)) : points[i].range;
    }

    for (std::size_t i = 0; i < points.size(); ++i) {
      points[i].range = smoothed[i];
      points[i].open = (points[i].range > open_point_min_distance_);
    }
  }

  OpenWindowResult find_longest_open_window(const std::vector<SectorPoint> & points) const
  {
    OpenWindowResult out;
    if (points.empty()) {
      return out;
    }

    int open_points = 0;
    for (const auto & p : points) {
      if (p.open) {
        ++open_points;
      }
    }

    out.open_count = open_points;
    out.open_ratio = static_cast<float>(
      static_cast<double>(open_points) / static_cast<double>(std::max(1, static_cast<int>(points.size()))));

    int cur_start = -1;
    int cur_len = 0;
    int best_start = -1;
    int best_end = -1;
    int best_len = 0;

    for (std::size_t i = 0; i < points.size(); ++i) {
      const bool open = points[i].open;
      const bool contiguous =
        (i > 0) &&
        std::abs(points[i].angle_deg - points[i - 1].angle_deg) <= max_adjacent_angle_gap_deg_;

      if (open) {
        if (cur_start < 0 || !contiguous) {
          cur_start = static_cast<int>(i);
          cur_len = 1;
        } else {
          ++cur_len;
        }

        if (cur_len > best_len) {
          best_len = cur_len;
          best_start = cur_start;
          best_end = static_cast<int>(i);
        }
      } else {
        cur_start = -1;
        cur_len = 0;
      }
    }

    out.max_open_run = best_len;
    if (best_len <= 0 || best_start < 0 || best_end < 0) {
      return out;
    }

    out.found = true;
    out.start_idx = best_start;
    out.end_idx = best_end;
    out.start_deg = static_cast<float>(points[static_cast<std::size_t>(best_start)].angle_deg);
    out.end_deg = static_cast<float>(points[static_cast<std::size_t>(best_end)].angle_deg);
    out.width = estimate_gap_width(
      points[static_cast<std::size_t>(best_start)],
      points[static_cast<std::size_t>(best_end)]);
    return out;
  }

  float estimate_gap_width(const SectorPoint & start, const SectorPoint & end) const
  {
    // 工程近似：把连续开口窗口的起止点当作缝隙两侧边界，
    // 在 base_link 下直接取二维欧氏距离作为缝宽估计。
    return static_cast<float>(std::hypot(end.x - start.x, end.y - start.y));
  }

  float min_distance_in_sector(const std::vector<SectorPoint> & points) const
  {
    float best = std::numeric_limits<float>::infinity();
    for (const auto & p : points) {
      best = std::min(best, static_cast<float>(p.range));
    }
    return best;
  }

  SectorConfig active_sector_config() const
  {
    SectorConfig cfg;
    if (active_side_ == "right") {
      cfg.gap_start_deg = right_gap_sector_start_deg_;
      cfg.gap_end_deg = right_gap_sector_end_deg_;
      cfg.front_veto_start_deg = right_front_veto_sector_start_deg_;
      cfg.front_veto_end_deg = right_front_veto_sector_end_deg_;
      cfg.side_veto_start_deg = right_side_veto_sector_start_deg_;
      cfg.side_veto_end_deg = right_side_veto_sector_end_deg_;
      return cfg;
    }

    cfg.gap_start_deg = left_gap_sector_start_deg_;
    cfg.gap_end_deg = left_gap_sector_end_deg_;
    cfg.front_veto_start_deg = left_front_veto_sector_start_deg_;
    cfg.front_veto_end_deg = left_front_veto_sector_end_deg_;
    cfg.side_veto_start_deg = left_side_veto_sector_start_deg_;
    cfg.side_veto_end_deg = left_side_veto_sector_end_deg_;
    return cfg;
  }

  FrameEval evaluate_gap_frame(const sensor_msgs::msg::LaserScan & scan)
  {
    FrameEval eval;
    eval.active_side = active_side_;
    const auto sectors = active_sector_config();

    int main_valid_count = 0;
    auto main_sector = extract_sector_points(
      scan,
      sectors.gap_start_deg,
      sectors.gap_end_deg,
      main_valid_count);
    apply_light_smoothing(main_sector);

    int front_left_veto_valid_count = 0;
    auto front_left_veto_sector = extract_sector_points(
      scan,
      sectors.front_veto_start_deg,
      sectors.front_veto_end_deg,
      front_left_veto_valid_count);

    int left_side_veto_valid_count = 0;
    auto left_side_veto_sector = extract_sector_points(
      scan,
      sectors.side_veto_start_deg,
      sectors.side_veto_end_deg,
      left_side_veto_valid_count);

    eval.valid_points = main_valid_count;
    eval.min_front_left_dist = min_distance_in_sector(front_left_veto_sector);
    eval.min_left_side_dist = min_distance_in_sector(left_side_veto_sector);

    const auto window = find_longest_open_window(main_sector);
    eval.open_points = window.open_count;
    eval.max_open_run = window.max_open_run;
    eval.open_ratio = window.open_ratio;
    eval.open_window_start_deg = window.start_deg;
    eval.open_window_end_deg = window.end_deg;
    eval.estimated_gap_width = window.width;

    const bool has_enough_points = eval.valid_points >= std::max(1, min_valid_points_);
    const bool open_run_ok = eval.max_open_run >= std::max(1, open_run_min_points_);
    const bool open_ratio_ok = eval.open_ratio >= static_cast<float>(open_ratio_threshold_);

    eval.gap_candidate_frame = has_enough_points && open_run_ok && open_ratio_ok;
    eval.entry_width_ok = eval.estimated_gap_width >= static_cast<float>(required_entry_width_);
    // 近障碍硬否决：只要扇区内存在 r < near_obstacle_veto_distance 的有效点，直接不安全。
    eval.front_left_safe = !(
      std::isfinite(eval.min_front_left_dist) &&
      eval.min_front_left_dist < static_cast<float>(near_obstacle_veto_distance_));
    eval.left_side_safe = !(
      std::isfinite(eval.min_left_side_dist) &&
      eval.min_left_side_dist < static_cast<float>(near_obstacle_veto_distance_));

    if (eval.gap_candidate_frame) {
      ++stable_frame_count_;
    } else {
      stable_frame_count_ = 0;
    }

    eval.stable_frame_count = stable_frame_count_;
    eval.gap_detected = stable_frame_count_ >= std::max(1, stable_frames_required_);
    eval.allow_enter =
      eval.gap_detected &&
      eval.entry_width_ok &&
      eval.front_left_safe &&
      eval.left_side_safe;
    eval.side = eval.gap_detected ? active_side_ : "none";

    if (detect_cycle_start_.nanoseconds() == 0) {
      detect_cycle_start_ = this->now();
    }
    if (eval.allow_enter) {
      failed_cycle_count_ = 0;
      detect_cycle_start_ = this->now();
    } else if ((this->now() - detect_cycle_start_).seconds() >= detect_cycle_timeout_sec_) {
      ++failed_cycle_count_;
      detect_cycle_start_ = this->now();
    }
    eval.failed_cycle_count = failed_cycle_count_;

    if (!eval.front_left_safe) {
      eval.debug_reason = "BLOCKED_BY_FRONT_LEFT";
    } else if (!eval.left_side_safe) {
      eval.debug_reason = "BLOCKED_BY_LEFT_SIDE";
    } else if (!has_enough_points) {
      eval.debug_reason = "VALID_POINTS_LOW";
    } else if (!open_run_ok) {
      eval.debug_reason = "OPEN_RUN_SHORT";
    } else if (!open_ratio_ok) {
      eval.debug_reason = "OPEN_RATIO_LOW";
    } else if (!eval.entry_width_ok) {
      eval.debug_reason = "BLOCKED_BY_WIDTH";
    } else {
      eval.debug_reason = "OK";
    }

    return eval;
  }

  void publish_status(const FrameEval & eval)
  {
    wheeltec_inventory_system::msg::GapStatus out;
    out.gap_detected = eval.gap_detected;
    out.entry_width_ok = eval.entry_width_ok;
    out.front_left_safe = eval.front_left_safe;
    out.left_side_safe = eval.left_side_safe;
    out.allow_enter = eval.allow_enter;
    out.gap_width = eval.estimated_gap_width;
    out.side = eval.side;
    out.active_side = eval.active_side;
    out.stable_frame_count = eval.stable_frame_count;
    out.failed_cycle_count = std::max(eval.failed_cycle_count, context_failed_cycle_count_);
    out.open_ratio = eval.open_ratio;
    out.max_open_run = eval.max_open_run;
    out.open_window_start_deg = eval.open_window_start_deg;
    out.open_window_end_deg = eval.open_window_end_deg;
    out.min_front_left_dist = eval.min_front_left_dist;
    out.min_left_side_dist = eval.min_left_side_dist;
    out.current_adjust_index = context_adjust_index_;
    out.current_adjust_offset = static_cast<float>(context_adjust_offset_);
    status_pub_->publish(out);
  }

  void append_csv_log(const sensor_msgs::msg::LaserScan & scan, const FrameEval & eval)
  {
    if (!enable_csv_log_ || !csv_file_.is_open()) {
      return;
    }

    const double ts =
      static_cast<double>(scan.header.stamp.sec) +
      static_cast<double>(scan.header.stamp.nanosec) * 1e-9;
    const auto sectors = active_sector_config();

    csv_file_
      << std::fixed << std::setprecision(6)
      << ts << ','
      << sanitize_frame_id(scan.header.frame_id) << ','
      << eval.active_side << ','
      << sectors.gap_start_deg << ','
      << sectors.gap_end_deg << ','
      << eval.valid_points << ','
      << eval.open_points << ','
      << eval.estimated_gap_width << ','
      << eval.stable_frame_count << ','
      << eval.gap_detected << ','
      << eval.entry_width_ok << ','
      << eval.front_left_safe << ','
      << eval.left_side_safe << ','
      << eval.min_front_left_dist << ','
      << eval.min_left_side_dist << ','
      << eval.allow_enter << ','
      << std::max(eval.failed_cycle_count, context_failed_cycle_count_) << ','
      << context_adjust_index_ << ','
      << context_adjust_offset_ << ','
      << eval.debug_reason
      << '\n';
    csv_file_.flush();
  }

  void init_csv_logger()
  {
    if (!enable_csv_log_) {
      return;
    }

    namespace fs = std::filesystem;
    try {
      fs::create_directories(csv_log_dir_);
      const auto now = std::chrono::system_clock::now();
      const std::time_t t = std::chrono::system_clock::to_time_t(now);
      std::tm tm_buf{};
#ifdef _WIN32
      localtime_s(&tm_buf, &t);
#else
      localtime_r(&t, &tm_buf);
#endif
      std::ostringstream oss;
      oss << csv_file_prefix_ << '_' << std::put_time(&tm_buf, "%Y%m%d_%H%M%S") << ".csv";
      const fs::path file_path = fs::path(csv_log_dir_) / oss.str();
      csv_file_.open(file_path.string(), std::ios::out | std::ios::app);
      if (!csv_file_.is_open()) {
        RCLCPP_WARN(get_logger(), "无法打开CSV日志文件: %s", file_path.c_str());
        return;
      }

      csv_file_
        << "timestamp,frame,active_side,gap_sector_start_deg,gap_sector_end_deg,valid_points,open_points,best_window_width,stable_count,"
        << "gap_detected,entry_width_ok,front_left_safe,left_side_safe,min_front_left_dist,min_left_side_dist,"
        << "allow_enter,failed_cycle_count,current_adjust_index,current_adjust_offset,block_reason"
        << '\n';
      csv_file_.flush();

      RCLCPP_INFO(get_logger(), "gap CSV日志: %s", file_path.c_str());
    } catch (const std::exception & ex) {
      RCLCPP_WARN(get_logger(), "初始化CSV日志失败: %s", ex.what());
    }
  }

  void context_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (!msg || msg->data.size() < 3U) {
      return;
    }

    context_failed_cycle_count_ = std::max(0, static_cast<int>(std::lround(msg->data[0])));
    context_adjust_index_ = static_cast<int>(std::lround(msg->data[1]));
    context_adjust_offset_ = static_cast<double>(msg->data[2]);
  }

  void entry_side_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    const std::string next_side = normalize_entry_side(msg->data);
    if (next_side == active_side_) {
      return;
    }

    active_side_ = next_side;
    stable_frame_count_ = 0;
    failed_cycle_count_ = 0;
    detect_cycle_start_ = detector_enabled_ ? this->now() :
      rclcpp::Time(0, 0, get_clock()->get_clock_type());
    RCLCPP_INFO(get_logger(), "gap_detector 检测方向切换为: %s", active_side_.c_str());
  }

  void enable_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    if (detector_enabled_ == msg->data) {
      return;
    }

    detector_enabled_ = msg->data;
    if (!detector_enabled_) {
      // 关闭检测时清空内部稳定态，避免下次启用时沿用旧判定。
      stable_frame_count_ = 0;
      failed_cycle_count_ = 0;
      detect_cycle_start_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      RCLCPP_INFO(get_logger(), "gap_detector 已禁用：停止检测计算与结果发布");
    } else {
      detect_cycle_start_ = this->now();
      RCLCPP_INFO(get_logger(), "gap_detector 已启用：恢复检测计算与结果发布");
    }
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (!detector_enabled_) {
      return;
    }

    const auto eval = evaluate_gap_frame(*msg);
    publish_status(eval);
    append_csv_log(*msg, eval);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "gap: side=%s cand=%d det=%d width=%.3f ratio=%.2f run=%d stable=%d/%d entry_ok=%d "
      "front_left=%.3f safe_fl=%d left_side=%.3f safe_ls=%d allow=%d fail=%d adj_idx=%d adj_off=%.3f reason=%s",
      eval.active_side.c_str(),
      eval.gap_candidate_frame ? 1 : 0,
      eval.gap_detected ? 1 : 0,
      eval.estimated_gap_width,
      eval.open_ratio,
      eval.max_open_run,
      eval.stable_frame_count,
      std::max(1, stable_frames_required_),
      eval.entry_width_ok ? 1 : 0,
      eval.min_front_left_dist,
      eval.front_left_safe ? 1 : 0,
      eval.min_left_side_dist,
      eval.left_side_safe ? 1 : 0,
      eval.allow_enter ? 1 : 0,
      std::max(eval.failed_cycle_count, context_failed_cycle_count_),
      context_adjust_index_,
      context_adjust_offset_,
      eval.debug_reason.c_str());
  }

  std::string scan_topic_;
  std::string output_topic_;
  std::string context_topic_;
  std::string enable_topic_;
  std::string entry_side_topic_;
  std::string base_link_frame_;
  std::string active_side_{"left"};

  double left_gap_sector_start_deg_{40.0};
  double left_gap_sector_end_deg_{140.0};
  double left_front_veto_sector_start_deg_{10.0};
  double left_front_veto_sector_end_deg_{70.0};
  double left_side_veto_sector_start_deg_{70.0};
  double left_side_veto_sector_end_deg_{120.0};
  double right_gap_sector_start_deg_{-120.0};
  double right_gap_sector_end_deg_{-60.0};
  double right_front_veto_sector_start_deg_{-70.0};
  double right_front_veto_sector_end_deg_{-10.0};
  double right_side_veto_sector_start_deg_{-120.0};
  double right_side_veto_sector_end_deg_{-70.0};

  double min_valid_range_{0.05};
  double max_valid_range_{8.0};

  double open_point_min_distance_{0.50};
  double near_obstacle_veto_distance_{0.50};
  double required_entry_width_{0.40};

  int open_run_min_points_{3};
  int min_valid_points_{8};
  double open_ratio_threshold_{0.10};
  double max_adjacent_angle_gap_deg_{2.0};
  int smoothing_window_{1};

  int stable_frames_required_{3};
  int stable_frame_count_{0};

  double detect_cycle_timeout_sec_{2.5};
  int failed_cycle_count_{0};
  rclcpp::Time detect_cycle_start_{0, 0, RCL_ROS_TIME};

  bool enable_csv_log_{true};
  std::string csv_log_dir_{"/tmp/wheeltec_inventory_gap_logs"};
  std::string csv_file_prefix_{"gap_debug"};
  std::ofstream csv_file_;

  int context_failed_cycle_count_{0};
  int context_adjust_index_{-1};
  double context_adjust_offset_{0.0};
  bool detector_enabled_{false};
  bool enable_on_start_{false};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr context_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr entry_side_sub_;
  rclcpp::Publisher<wheeltec_inventory_system::msg::GapStatus>::SharedPtr status_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GapDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
