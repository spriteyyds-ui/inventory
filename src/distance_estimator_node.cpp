#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32.hpp"
#include "wheeltec_inventory_system/msg/recognized_number.hpp"

class DistanceEstimatorNode : public rclcpp::Node
{
public:
  DistanceEstimatorNode()
  : Node("distance_estimator_node")
  {
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    recognized_topic_ = declare_parameter<std::string>(
      "recognized_topic", "/inventory/recognized_number");
    output_topic_ = declare_parameter<std::string>("output_topic", "/inventory/target_distance");

    fusion_mode_ = declare_parameter<std::string>("fusion_mode", "hybrid");
    recognition_timeout_sec_ = declare_parameter<double>("recognition_timeout_sec", 0.8);

    visual_min_distance_ = declare_parameter<double>("visual_min_distance", 0.2);
    visual_max_distance_ = declare_parameter<double>("visual_max_distance", 5.0);

    camera_hfov_deg_ = declare_parameter<double>("camera_hfov_deg", 60.0);
    camera_image_width_px_ = declare_parameter<double>("camera_image_width_px", 640.0);
    lidar_window_deg_ = declare_parameter<double>("lidar_window_deg", 10.0);
    min_valid_range_ = declare_parameter<double>("min_valid_range", 0.05);
    max_valid_range_ = declare_parameter<double>("max_valid_range", 8.0);

    tracking_range_sector_start_deg_ =
      declare_parameter<double>("tracking_range_sector_start_deg", -10.0);
    tracking_range_sector_end_deg_ =
      declare_parameter<double>("tracking_range_sector_end_deg", 10.0);
    tracking_min_valid_points_ =
      declare_parameter<int>("tracking_min_valid_points", 5);
    tracking_use_median_ =
      declare_parameter<bool>("tracking_use_median", true);
    tracking_low_percentile_ =
      declare_parameter<double>("tracking_low_percentile", 0.35);
    tracking_distance_filter_alpha_ =
      declare_parameter<double>("tracking_distance_filter_alpha", 0.35);
    tracking_min_valid_distance_ =
      declare_parameter<double>("tracking_min_valid_distance", 0.10);
    tracking_max_valid_distance_ =
      declare_parameter<double>("tracking_max_valid_distance", 5.00);
    tracking_prioritize_lidar_ =
      declare_parameter<bool>("tracking_prioritize_lidar", true);
    tracking_visual_fallback_when_lidar_missing_ =
      declare_parameter<bool>("tracking_visual_fallback_when_lidar_missing", true);

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);

    distance_pub_ = create_publisher<std_msgs::msg::Float32>(output_topic_, 10);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_scan_ = msg;
      });

    recognized_sub_ = create_subscription<wheeltec_inventory_system::msg::RecognizedNumber>(
      recognized_topic_,
      10,
      [this](const wheeltec_inventory_system::msg::RecognizedNumber::SharedPtr msg) {
        latest_recognition_ = msg;
        latest_recognition_time_ = this->now();
      });

    const double period = 1.0 / std::max(1.0, publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period)),
      std::bind(&DistanceEstimatorNode::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "距离估算节点已启动 mode=%s lidar_priority=%d sector=[%.1f,%.1f] min_pts=%d stat=%s q=%.2f alpha=%.2f",
      fusion_mode_.c_str(),
      tracking_prioritize_lidar_ ? 1 : 0,
      tracking_range_sector_start_deg_,
      tracking_range_sector_end_deg_,
      tracking_min_valid_points_,
      tracking_use_median_ ? "median" : "percentile",
      tracking_low_percentile_,
      tracking_distance_filter_alpha_);
  }

private:
  static bool finite_and_positive(double v)
  {
    return std::isfinite(v) && v > 0.0;
  }

  static double deg2rad(double deg)
  {
    return deg * M_PI / 180.0;
  }

  double visual_distance_estimate(const wheeltec_inventory_system::msg::RecognizedNumber & rec) const
  {
    if (!rec.valid || rec.estimated_distance <= 1e-3F) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    return std::clamp(
      static_cast<double>(rec.estimated_distance),
      visual_min_distance_,
      visual_max_distance_);
  }

  double lidar_distance_estimate(
    const sensor_msgs::msg::LaserScan & scan,
    const wheeltec_inventory_system::msg::RecognizedNumber & rec) const
  {
    if (scan.ranges.empty()) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    const double fov_rad = deg2rad(camera_hfov_deg_);
    const double width = std::max(1.0, camera_image_width_px_);
    const double target_center_angle =
      -(static_cast<double>(rec.horizontal_offset) / width) * fov_rad;
    const double start =
      target_center_angle + deg2rad(std::min(tracking_range_sector_start_deg_, tracking_range_sector_end_deg_));
    const double end =
      target_center_angle + deg2rad(std::max(tracking_range_sector_start_deg_, tracking_range_sector_end_deg_));

    std::vector<double> candidates;
    candidates.reserve(scan.ranges.size());
    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      if (angle < start || angle > end) {
        continue;
      }

      const double r = scan.ranges[i];
      if (!std::isfinite(r)) {
        continue;
      }
      if (r < min_valid_range_ || r > max_valid_range_) {
        continue;
      }
      if (r < tracking_min_valid_distance_ || r > tracking_max_valid_distance_) {
        continue;
      }
      candidates.push_back(r);
    }

    if (static_cast<int>(candidates.size()) < std::max(1, tracking_min_valid_points_)) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    std::sort(candidates.begin(), candidates.end());
    if (tracking_use_median_) {
      const std::size_t n = candidates.size();
      if (n % 2 == 1U) {
        return candidates[n / 2U];
      }
      return 0.5 * (candidates[n / 2U - 1U] + candidates[n / 2U]);
    }

    const double q = std::clamp(tracking_low_percentile_, 0.0, 1.0);
    const std::size_t idx = static_cast<std::size_t>(
      std::round(q * static_cast<double>(std::max<std::size_t>(1U, candidates.size() - 1U))));
    return candidates[idx];
  }

  double filter_lidar_distance(double raw)
  {
    if (!finite_and_positive(raw)) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    if (!finite_and_positive(filtered_lidar_distance_)) {
      filtered_lidar_distance_ = raw;
      return filtered_lidar_distance_;
    }

    const double alpha = std::clamp(tracking_distance_filter_alpha_, 0.0, 1.0);
    filtered_lidar_distance_ = alpha * raw + (1.0 - alpha) * filtered_lidar_distance_;
    return filtered_lidar_distance_;
  }

  void on_timer()
  {
    if (!latest_recognition_) {
      return;
    }

    if ((this->now() - latest_recognition_time_).seconds() > recognition_timeout_sec_) {
      return;
    }

    const auto & rec = *latest_recognition_;
    if (!rec.valid) {
      return;
    }

    const double visual_dist = visual_distance_estimate(rec);

    double lidar_dist_raw = std::numeric_limits<double>::quiet_NaN();
    if (latest_scan_) {
      lidar_dist_raw = lidar_distance_estimate(*latest_scan_, rec);
    }
    const double lidar_dist = filter_lidar_distance(lidar_dist_raw);

    double fused = std::numeric_limits<double>::quiet_NaN();
    if (fusion_mode_ == "visual") {
      fused = visual_dist;
    } else if (fusion_mode_ == "lidar") {
      fused = lidar_dist;
    } else {
      if (tracking_prioritize_lidar_) {
        if (finite_and_positive(lidar_dist)) {
          fused = lidar_dist;
        } else if (tracking_visual_fallback_when_lidar_missing_) {
          fused = visual_dist;
        }
      } else {
        if (finite_and_positive(lidar_dist) && finite_and_positive(visual_dist)) {
          fused = 0.7 * lidar_dist + 0.3 * visual_dist;
        } else if (finite_and_positive(lidar_dist)) {
          fused = lidar_dist;
        } else {
          fused = visual_dist;
        }
      }
    }

    if (!finite_and_positive(fused)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "目标距离不可用：lidar_raw=%.3f lidar_filt=%.3f visual=%.3f",
        lidar_dist_raw,
        lidar_dist,
        visual_dist);
      return;
    }

    std_msgs::msg::Float32 out;
    out.data = static_cast<float>(fused);
    distance_pub_->publish(out);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1200,
      "target_distance=%.3f (lidar_raw=%.3f lidar=%.3f visual=%.3f mode=%s)",
      fused,
      lidar_dist_raw,
      lidar_dist,
      visual_dist,
      fusion_mode_.c_str());
  }

  std::string scan_topic_;
  std::string recognized_topic_;
  std::string output_topic_;
  std::string fusion_mode_;

  double recognition_timeout_sec_{0.8};
  double visual_min_distance_{0.2};
  double visual_max_distance_{5.0};

  double camera_hfov_deg_{60.0};
  double camera_image_width_px_{640.0};
  double lidar_window_deg_{10.0};
  double min_valid_range_{0.05};
  double max_valid_range_{8.0};
  double tracking_range_sector_start_deg_{-10.0};
  double tracking_range_sector_end_deg_{10.0};
  int tracking_min_valid_points_{5};
  bool tracking_use_median_{true};
  double tracking_low_percentile_{0.35};
  double tracking_distance_filter_alpha_{0.35};
  double tracking_min_valid_distance_{0.10};
  double tracking_max_valid_distance_{5.0};
  bool tracking_prioritize_lidar_{true};
  bool tracking_visual_fallback_when_lidar_missing_{true};
  double publish_rate_hz_{10.0};

  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  wheeltec_inventory_system::msg::RecognizedNumber::SharedPtr latest_recognition_;
  rclcpp::Time latest_recognition_time_{0, 0, RCL_ROS_TIME};
  double filtered_lidar_distance_{std::numeric_limits<double>::quiet_NaN()};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<wheeltec_inventory_system::msg::RecognizedNumber>::SharedPtr recognized_sub_;
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
