#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

class DistanceEstimatorNode : public rclcpp::Node
{
public:
  DistanceEstimatorNode()
  : Node("distance_estimator_node")
  {
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    output_topic_ = declare_parameter<std::string>("output_topic", "/inventory/target_distance");
    target_lidar_side_topic_ = declare_parameter<std::string>(
      "target_lidar_side_topic", "/inventory/target_lidar_side");
    default_lidar_side_ = normalize_lidar_side(
      declare_parameter<std::string>("default_lidar_side", "right"));
    if (default_lidar_side_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "default_lidar_side 参数非法，回退为 right");
      default_lidar_side_ = "right";
    }
    active_lidar_side_ = default_lidar_side_;

    left_lidar_center_deg_ = declare_parameter<double>("left_lidar_center_deg", 90.0);
    right_lidar_center_deg_ = declare_parameter<double>("right_lidar_center_deg", -90.0);
    side_lidar_window_deg_ = declare_parameter<double>("side_lidar_window_deg", 10.0);
    min_valid_distance_ = declare_parameter<double>("min_valid_distance", 0.05);
    max_valid_distance_ = declare_parameter<double>("max_valid_distance", 10.0);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);

    if (!std::isfinite(side_lidar_window_deg_) || side_lidar_window_deg_ <= 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "side_lidar_window_deg 参数非法 %.3f，回退为 10.0",
        side_lidar_window_deg_);
      side_lidar_window_deg_ = 10.0;
    }
    if (!std::isfinite(min_valid_distance_) || min_valid_distance_ < 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "min_valid_distance 参数非法 %.3f，回退为 0.05",
        min_valid_distance_);
      min_valid_distance_ = 0.05;
    }
    if (!std::isfinite(max_valid_distance_) || max_valid_distance_ <= min_valid_distance_) {
      RCLCPP_WARN(
        get_logger(),
        "max_valid_distance 参数非法 %.3f，回退为 10.0",
        max_valid_distance_);
      max_valid_distance_ = 10.0;
    }

    distance_pub_ = create_publisher<std_msgs::msg::Float32>(output_topic_, 10);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_scan_ = msg;
      });

    target_lidar_side_sub_ = create_subscription<std_msgs::msg::String>(
      target_lidar_side_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        const std::string side = normalize_lidar_side(msg->data);
        if (side.empty()) {
          RCLCPP_WARN(
            get_logger(),
            "[distance_estimator] ignore invalid target_lidar_side=%s",
            msg->data.c_str());
          return;
        }
        active_lidar_side_ = side;
        RCLCPP_INFO(
          get_logger(),
          "[distance_estimator] active target_lidar_side=%s",
          active_lidar_side_.c_str());
      });

    const double period = 1.0 / std::max(1.0, publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period)),
      std::bind(&DistanceEstimatorNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "距离估算节点已启动 lidar_only side=%s left_center=%.1f right_center=%.1f window=%.1f "
      "valid_range=[%.2f,%.2f]",
      active_lidar_side_.c_str(),
      left_lidar_center_deg_,
      right_lidar_center_deg_,
      side_lidar_window_deg_,
      min_valid_distance_,
      max_valid_distance_);
  }

private:
  struct LidarEstimate
  {
    double distance{std::numeric_limits<double>::quiet_NaN()};
    std::size_t valid_points{0U};
  };

  static double deg2rad(double deg)
  {
    return deg * M_PI / 180.0;
  }

  static double normalize_angle(double rad)
  {
    return std::atan2(std::sin(rad), std::cos(rad));
  }

  static double angular_distance(double a, double b)
  {
    return std::abs(normalize_angle(a - b));
  }

  static std::string normalize_lidar_side(const std::string & side)
  {
    std::string normalized;
    normalized.reserve(side.size());
    for (const char ch : side) {
      normalized.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
    }
    if (normalized == "left" || normalized == "right") {
      return normalized;
    }
    return "";
  }

  double active_lidar_center_deg() const
  {
    return active_lidar_side_ == "left" ? left_lidar_center_deg_ : right_lidar_center_deg_;
  }

  LidarEstimate estimate_side_lidar_distance(const sensor_msgs::msg::LaserScan & scan) const
  {
    LidarEstimate estimate;
    if (scan.ranges.empty()) {
      return estimate;
    }

    const double center_rad = deg2rad(active_lidar_center_deg());
    const double half_window_rad = deg2rad(side_lidar_window_deg_) * 0.5;

    std::vector<double> candidates;
    candidates.reserve(scan.ranges.size());

    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      if (angular_distance(angle, center_rad) > half_window_rad) {
        continue;
      }

      const double range = static_cast<double>(scan.ranges[i]);
      if (!std::isfinite(range)) {
        continue;
      }
      if (range < static_cast<double>(scan.range_min) || range > static_cast<double>(scan.range_max)) {
        continue;
      }
      if (range < min_valid_distance_ || range > max_valid_distance_) {
        continue;
      }

      candidates.push_back(range);
    }

    estimate.valid_points = candidates.size();
    if (candidates.empty()) {
      return estimate;
    }

    std::sort(candidates.begin(), candidates.end());
    const std::size_t count = candidates.size();
    if (count % 2U == 1U) {
      estimate.distance = candidates[count / 2U];
    } else {
      estimate.distance = 0.5 * (candidates[count / 2U - 1U] + candidates[count / 2U]);
    }
    return estimate;
  }

  void on_timer()
  {
    if (!latest_scan_) {
      return;
    }

    const LidarEstimate estimate = estimate_side_lidar_distance(*latest_scan_);
    if (!std::isfinite(estimate.distance) || estimate.distance <= 0.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "[distance_estimator] invalid lidar distance: side=%s valid_points=%zu",
        active_lidar_side_.c_str(),
        estimate.valid_points);
      return;
    }

    std_msgs::msg::Float32 out;
    out.data = static_cast<float>(estimate.distance);
    distance_pub_->publish(out);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1200,
      "target_distance=%.3f lidar_side=%s lidar_center_deg=%.1f window_deg=%.1f valid_points=%zu",
      estimate.distance,
      active_lidar_side_.c_str(),
      active_lidar_center_deg(),
      side_lidar_window_deg_,
      estimate.valid_points);
  }

  std::string scan_topic_;
  std::string output_topic_;
  std::string target_lidar_side_topic_;
  std::string default_lidar_side_{"right"};
  std::string active_lidar_side_{"right"};

  double left_lidar_center_deg_{90.0};
  double right_lidar_center_deg_{-90.0};
  double side_lidar_window_deg_{10.0};
  double min_valid_distance_{0.05};
  double max_valid_distance_{10.0};
  double publish_rate_hz_{10.0};

  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr target_lidar_side_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DistanceEstimatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
