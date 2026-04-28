#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"

class CorridorFollowerNode : public rclcpp::Node
{
public:
  CorridorFollowerNode()
  : Node("corridor_follower_node")
  {
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    forward_speed_ = declare_parameter<double>("forward_speed", 0.20);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 0.60);

    pid_kp_ = declare_parameter<double>("pid_kp", 1.2);
    pid_ki_ = declare_parameter<double>("pid_ki", 0.0);
    pid_kd_ = declare_parameter<double>("pid_kd", 0.15);
    integral_limit_ = declare_parameter<double>("integral_limit", 1.5);
    error_deadband_ = declare_parameter<double>("error_deadband", 0.02);

    left_start_deg_ = declare_parameter<double>("left_start_deg", 60.0);
    left_end_deg_ = declare_parameter<double>("left_end_deg", 120.0);
    right_start_deg_ = declare_parameter<double>("right_start_deg", -120.0);
    right_end_deg_ = declare_parameter<double>("right_end_deg", -60.0);
    min_valid_range_ = declare_parameter<double>("min_valid_range", 0.05);
    max_valid_range_ = declare_parameter<double>("max_valid_range", 8.0);

    enable_control_topic_ = declare_parameter<std::string>(
      "enable_control_topic", "/inventory/corridor_enable");
    reverse_control_topic_ = declare_parameter<std::string>(
      "reverse_control_topic", "/inventory/corridor_reverse");
    set_enable_service_name_ = declare_parameter<std::string>(
      "set_enable_service_name", "/inventory/set_corridor_following");

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&CorridorFollowerNode::scan_callback, this, std::placeholders::_1));

    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      enable_control_topic_,
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        set_enabled(msg->data);
      });

    reverse_sub_ = create_subscription<std_msgs::msg::Bool>(
      reverse_control_topic_,
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        reverse_mode_ = msg->data;
      });

    set_enable_srv_ = create_service<std_srvs::srv::SetBool>(
      set_enable_service_name_,
      [this](
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
      {
        set_enabled(request->data);
        response->success = true;
        response->message = enabled_ ? "corridor follower enabled" : "corridor follower disabled";
      });

    RCLCPP_INFO(get_logger(), "走廊跟随节点已启动，scan=%s", scan_topic_.c_str());
  }

private:
  static double deg2rad(double deg)
  {
    return deg * M_PI / 180.0;
  }

  static bool in_range(double value, double start, double end)
  {
    return value >= std::min(start, end) && value <= std::max(start, end);
  }

  void set_enabled(bool enabled)
  {
    if (enabled_ == enabled) {
      return;
    }
    enabled_ = enabled;
    integral_ = 0.0;
    last_error_ = 0.0;
    has_last_time_ = false;

    if (!enabled_) {
      publish_stop();
    }
  }

  void publish_stop()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);
  }

  double sector_min_range(
    const sensor_msgs::msg::LaserScan & scan,
    double start_deg,
    double end_deg) const
  {
    const double start = deg2rad(start_deg);
    const double end = deg2rad(end_deg);

    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      if (!in_range(angle, start, end)) {
        continue;
      }

      const double r = scan.ranges[i];
      if (!std::isfinite(r)) {
        continue;
      }
      if (r < min_valid_range_ || r > max_valid_range_) {
        continue;
      }
      if (r < best) {
        best = r;
      }
    }
    return best;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (!enabled_) {
      return;
    }

    const double left_min = sector_min_range(*msg, left_start_deg_, left_end_deg_);
    const double right_min = sector_min_range(*msg, right_start_deg_, right_end_deg_);

    if (!std::isfinite(left_min) || !std::isfinite(right_min)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "左右距离无效，停止输出");
      publish_stop();
      return;
    }

    const double error = left_min - right_min;

    const auto now = this->now();
    double dt = 0.02;
    if (has_last_time_) {
      dt = std::max(1e-3, (now - last_time_).seconds());
    }
    last_time_ = now;
    has_last_time_ = true;

    integral_ += error * dt;
    integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);

    const double derivative = (error - last_error_) / dt;
    last_error_ = error;

    double angular = pid_kp_ * error + pid_ki_ * integral_ + pid_kd_ * derivative;
    if (std::abs(error) < error_deadband_) {
      angular = 0.0;
    }
    angular = std::clamp(angular, -max_angular_speed_, max_angular_speed_);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = reverse_mode_ ? -std::abs(forward_speed_) : std::abs(forward_speed_);
    cmd.angular.z = angular;
    cmd_pub_->publish(cmd);
  }

  std::string scan_topic_;
  std::string cmd_vel_topic_;
  std::string enable_control_topic_;
  std::string reverse_control_topic_;
  std::string set_enable_service_name_;

  double forward_speed_{0.20};
  double max_angular_speed_{0.60};

  double pid_kp_{1.2};
  double pid_ki_{0.0};
  double pid_kd_{0.15};
  double integral_limit_{1.5};
  double error_deadband_{0.02};

  double left_start_deg_{60.0};
  double left_end_deg_{120.0};
  double right_start_deg_{-120.0};
  double right_end_deg_{-60.0};
  double min_valid_range_{0.05};
  double max_valid_range_{8.0};

  bool enabled_{false};
  bool reverse_mode_{false};

  double integral_{0.0};
  double last_error_{0.0};
  bool has_last_time_{false};
  rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reverse_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_enable_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CorridorFollowerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
