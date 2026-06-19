// Copyright (c) 2026 郁有冬 <spriteyyds@gmail.com>. All rights reserved.
// yellow_line_test_node.cpp
// 黄线巡线独立测试节点
// 功能：只订阅相机图像，运行黄线识别，发布 debug 图像，打印识别结果。
// 不发布 cmd_vel，不控制小车。用于现场先验证识别效果。

#include <functional>
#include <memory>
#include <string>

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "agv_inventory_system/yellow_line_follower.hpp"

class YellowLineTestNode : public rclcpp::Node
{
public:
  YellowLineTestNode()
  : Node("yellow_line_test_node")
  {
    // 参数声明
    const std::string image_topic =
      declare_parameter<std::string>("yellow_line_image_topic", "/camera/color/image_raw");
    const double roi_y_min = declare_parameter<double>("yellow_line_roi_y_min_ratio", 0.55);
    const double roi_y_max = declare_parameter<double>("yellow_line_roi_y_max_ratio", 0.95);
    const double target_x_ratio = declare_parameter<double>("yellow_line_target_x_ratio", 0.65);
    const double target_x_offset = declare_parameter<double>("yellow_line_target_x_offset_px", 0.0);
    const int h_min = declare_parameter<int>("yellow_h_min", 15);
    const int h_max = declare_parameter<int>("yellow_h_max", 40);
    const int s_min = declare_parameter<int>("yellow_s_min", 80);
    const int s_max = declare_parameter<int>("yellow_s_max", 255);
    const int v_min = declare_parameter<int>("yellow_v_min", 80);
    const int v_max = declare_parameter<int>("yellow_v_max", 255);
    const double min_area = declare_parameter<double>("yellow_line_min_area", 300.0);
    const double lost_timeout = declare_parameter<double>("yellow_line_lost_timeout_sec", 0.5);
    const double kp = declare_parameter<double>("yellow_line_kp", 0.8);
    const double kd = declare_parameter<double>("yellow_line_kd", 0.05);
    const double max_angular = declare_parameter<double>("yellow_line_max_angular", 0.25);
    const bool reverse_invert = declare_parameter<bool>("yellow_line_reverse_invert_angular", true);
    const bool debug_enabled = declare_parameter<bool>("yellow_line_debug_image_enabled", true);
    const std::string debug_topic =
      declare_parameter<std::string>("yellow_line_debug_image_topic", "/yellow_line/debug_image");
    const double linear_x = declare_parameter<double>("test_linear_speed", 0.15);

    // 配置 follower
    agv_inventory_system::YellowLineFollowerConfig cfg;
    cfg.roi_y_min_ratio = roi_y_min;
    cfg.roi_y_max_ratio = roi_y_max;
    cfg.target_x_ratio = target_x_ratio;
    cfg.target_x_offset_px = target_x_offset;
    cfg.yellow_h_min = h_min;
    cfg.yellow_h_max = h_max;
    cfg.yellow_s_min = s_min;
    cfg.yellow_s_max = s_max;
    cfg.yellow_v_min = v_min;
    cfg.yellow_v_max = v_max;
    cfg.min_area = min_area;
    cfg.lost_timeout_sec = lost_timeout;
    cfg.kp = kp;
    cfg.kd = kd;
    cfg.max_angular = max_angular;
    cfg.reverse_invert_angular = reverse_invert;
    follower_.setConfig(cfg);

    linear_x_ = linear_x;

    // Debug image publisher
    if (debug_enabled) {
      debug_pub_ = create_publisher<sensor_msgs::msg::Image>(debug_topic, 10);
    }

    // Image subscriber
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&YellowLineTestNode::on_image, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "[yellow_line_test] started. topic=%s debug=%s test_linear_speed=%.3f",
      image_topic.c_str(), debug_enabled ? "true" : "false", linear_x);
    RCLCPP_INFO(get_logger(),
      "[yellow_line_test] target_x_ratio=%.2f kp=%.2f kd=%.3f max_angular=%.2f",
      target_x_ratio, kp, kd, max_angular);
  }

private:
  void on_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "[yellow_line_test] cv_bridge exception: %s", e.what());
      return;
    }

    const double now_sec = this->now().seconds();
    follower_.processImage(cv_ptr->image, now_sec);

    const auto & r = follower_.getResult();

    // 计算角速度（仅用于打印，不发布 cmd_vel）
    double angular_z = 0.0;
    const bool ok = follower_.getAngularCorrection(linear_x_, angular_z);
    (void)ok;

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "[yellow_line_test] detected=%d line_x=%.1f target_x=%.1f "
      "error_px=%.1f error_norm=%.4f angular_z=%.4f lost=%s",
      r.detected ? 1 : 0,
      r.line_x, r.target_x,
      r.error_px, r.error_norm,
      angular_z,
      follower_.isLostTimeout(now_sec) ? "YES" : "no");

    // Publish debug image
    if (debug_pub_) {
      cv::Mat dbg = follower_.drawDebug(cv_ptr->image, now_sec);
      auto dbg_msg = cv_bridge::CvImage(msg->header, "bgr8", dbg).toImageMsg();
      debug_pub_->publish(*dbg_msg);
    }
  }

  agv_inventory_system::YellowLineFollower follower_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  double linear_x_{0.15};
};

// Copyright (c) 2026 郁有冬 <spriteyyds@gmail.com>. All rights reserved.
static const char* COPYRIGHT_NOTICE =
    "========================================\n"
    " agv_inventory_system\n"
    " Copyright (c) 2026 郁有冬 <spriteyyds@gmail.com>\n"
    " All rights reserved.\n"
    "========================================\n";

int main(int argc, char ** argv)
{
  printf("%s", COPYRIGHT_NOTICE);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YellowLineTestNode>());
  rclcpp::shutdown();
  return 0;
}
