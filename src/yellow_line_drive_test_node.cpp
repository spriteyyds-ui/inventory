// yellow_line_drive_test_node.cpp
// 独立巡线驾驶测试节点
// 功能：订阅前方相机，识别黄线，发布 cmd_vel 做前进/后退巡线。
// 不依赖 mission_manager，不依赖盘库流程。
//
// 用法：
//   ros2 run agv_inventory_system yellow_line_drive_test_node --ros-args --params-file src/agv_inventory_system/config/inventory_system.yaml
//
// 服务控制：
//   ros2 service call /yellow_line_drive_test/start std_srvs/srv/Trigger    # 启动巡线
//   ros2 service call /yellow_line_drive_test/stop std_srvs/srv/Trigger     # 停车
//   ros2 service call /yellow_line_drive_test/forward std_srvs/srv/Trigger  # 切换前进
//   ros2 service call /yellow_line_drive_test/backward std_srvs/srv/Trigger # 切换后退

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "agv_inventory_system/yellow_line_follower.hpp"

class YellowLineDriveTestNode : public rclcpp::Node
{
public:
  YellowLineDriveTestNode()
  : Node("yellow_line_drive_test_node")
  {
    // ===== 参数 =====
    const std::string image_topic =
      declare_parameter<std::string>("yellow_line_image_topic", "/camera/color/image_raw");
    const std::string cmd_vel_topic =
      declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
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
    config_has_reverse_invert_ = reverse_invert;
    linear_speed_ = declare_parameter<double>("test_linear_speed", 0.15);
    backward_linear_speed_ = declare_parameter<double>("test_backward_linear_speed", -1.0);
    const double backward_target_x_ratio = declare_parameter<double>("yellow_line_backward_target_x_ratio", -1.0);
    const double backward_kp = declare_parameter<double>("yellow_line_backward_kp", -1.0);
    const double backward_kd = declare_parameter<double>("yellow_line_backward_kd", -1.0);
    const double backward_max_angular = declare_parameter<double>("yellow_line_backward_max_angular", -1.0);
    const bool debug_enabled = declare_parameter<bool>("yellow_line_debug_image_enabled", true);
    const std::string debug_topic =
      declare_parameter<std::string>("yellow_line_debug_image_topic", "/yellow_line/debug_image");

    // ===== 配置 follower =====
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
    cfg.backward_target_x_ratio = backward_target_x_ratio;
    cfg.backward_kp = backward_kp;
    cfg.backward_kd = backward_kd;
    cfg.backward_max_angular = backward_max_angular;
    follower_.setConfig(cfg);
    lost_timeout_sec_ = lost_timeout;

    // ===== 发布 =====
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    if (debug_enabled) {
      debug_pub_ = create_publisher<sensor_msgs::msg::Image>(debug_topic, 10);
    }

    // ===== 图像订阅 =====
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&YellowLineDriveTestNode::on_image, this, std::placeholders::_1));

    // ===== 服务 =====
    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/start",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr res) {
        running_ = true;
        res->success = true;
        res->message = "巡线已启动，方向=" + std::string(forward_ ? "前进" : "后退");
        RCLCPP_INFO(get_logger(), "[drive_test] START direction=%s speed=%.3f",
          forward_ ? "forward" : "backward", linear_speed_);
        const size_t sub_count = cmd_pub_->get_subscription_count();
        if (sub_count == 0) {
          RCLCPP_WARN(get_logger(),
            "[drive_test] cmd_vel 话题 '%s' 无订阅者！小车不会移动。"
            "请启动底盘驱动: ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py",
            cmd_vel_topic_.c_str());
        } else {
          RCLCPP_INFO(get_logger(),
            "[drive_test] cmd_vel 话题 '%s' 有 %zu 个订阅者", cmd_vel_topic_.c_str(), sub_count);
        }
      });

    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/stop",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr res) {
        running_ = false;
        publish_stop();
        res->success = true;
        res->message = "已停车";
        RCLCPP_INFO(get_logger(), "[drive_test] STOP");
      });

    forward_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/forward",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr res) {
        forward_ = true;
        res->success = true;
        res->message = "已切换为前进";
        RCLCPP_INFO(get_logger(), "[drive_test] direction=forward");
      });

    backward_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/backward",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr res) {
        forward_ = false;
        res->success = true;
        res->message = "已切换为后退";
        RCLCPP_INFO(get_logger(), "[drive_test] direction=backward");
      });

    RCLCPP_INFO(get_logger(),
      "[drive_test] 黄线巡线驾驶测试节点已启动");
    RCLCPP_INFO(get_logger(),
      "[drive_test] 话题: image=%s cmd_vel=%s", image_topic.c_str(), cmd_vel_topic.c_str());
    RCLCPP_INFO(get_logger(),
      "[drive_test] 前进参数: linear_speed=%.3f kp=%.2f kd=%.3f max_angular=%.2f target_x_ratio=%.3f",
      linear_speed_, kp, kd, max_angular, target_x_ratio);
    RCLCPP_INFO(get_logger(),
      "[drive_test] 后退参数: linear_speed=%.3f kp=%.2f kd=%.3f max_angular=%.2f target_x_ratio=%.3f reverse_invert=%s",
      backward_linear_speed_ > 0 ? backward_linear_speed_ : linear_speed_,
      backward_kp > 0 ? backward_kp : kp,
      backward_kd > 0 ? backward_kd : kd,
      backward_max_angular > 0 ? backward_max_angular : max_angular,
      backward_target_x_ratio > 0 ? backward_target_x_ratio : target_x_ratio,
      reverse_invert ? "true" : "false");
    RCLCPP_INFO(get_logger(),
      "[drive_test] 注意: 请确认底盘驱动已启动，运行 'ros2 topic info %s -v' 查看 Subscription count",
      cmd_vel_topic.c_str());
    RCLCPP_INFO(get_logger(),
      "[drive_test] 如果 Subscription count=0，需要启动底盘: "
      "ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py");
    RCLCPP_INFO(get_logger(),
      "[drive_test] 服务:");
    RCLCPP_INFO(get_logger(),
      "  ros2 service call /yellow_line_drive_test_node/start std_srvs/srv/Trigger");
    RCLCPP_INFO(get_logger(),
      "  ros2 service call /yellow_line_drive_test_node/stop std_srvs/srv/Trigger");
    RCLCPP_INFO(get_logger(),
      "  ros2 service call /yellow_line_drive_test_node/forward std_srvs/srv/Trigger");
    RCLCPP_INFO(get_logger(),
      "  ros2 service call /yellow_line_drive_test_node/backward std_srvs/srv/Trigger");
    cmd_vel_topic_ = cmd_vel_topic;
  }

private:
  void publish_stop()
  {
    geometry_msgs::msg::Twist cmd;
    cmd_pub_->publish(cmd);
  }

  void on_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "[drive_test] cv_bridge exception: %s", e.what());
      return;
    }

    const double now_sec = this->now().seconds();

    // 设置方向（影响 target_x_ratio 选择）
    const bool is_forward = forward_;
    follower_.setDirection(is_forward);

    follower_.processImage(cv_ptr->image, now_sec);

    // Publish debug image
    if (debug_pub_) {
      cv::Mat dbg = follower_.drawDebug(cv_ptr->image, now_sec);
      auto dbg_msg = cv_bridge::CvImage(msg->header, "bgr8", dbg).toImageMsg();
      debug_pub_->publish(*dbg_msg);
    }

    if (!running_) {
      return;
    }

    // 计算线速度：后退使用独立速度
    const double eff_backward_speed =
      (backward_linear_speed_ > 0.0) ? backward_linear_speed_ : linear_speed_;
    const double linear_x = is_forward ? linear_speed_ : -eff_backward_speed;

    // 计算角速度
    double angular_z = 0.0;
    const bool line_ok = follower_.getAngularCorrection(linear_x, angular_z);
    const bool lost_timeout = follower_.isLostTimeout(now_sec);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_x;

    if (line_ok) {
      cmd.angular.z = angular_z;
      const auto & r = follower_.getResult();
      // 详细后退调试日志（每 300ms 输出一次）
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 300,
        "[drive_test] ctrl direction=%s linear_x=%.3f error=%.4f d_error=%.4f "
        "final_ang=%.4f reverse_invert=%s line_x=%.1f target_x=%.1f",
        is_forward ? "forward" : "backward",
        linear_x, r.error_norm, 0.0,  // d_error 内部计算，此处用 0 占位
        angular_z,
        (config_has_reverse_invert_ ? "true" : "false"),
        r.line_x, r.target_x);
    } else if (lost_timeout) {
      // 丢线超时 → 停车
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "[drive_test] LOST LINE, STOPPING (timeout=%.2fs)", lost_timeout_sec_);
    } else {
      // 丢线未超时 → 保持直行
      cmd.angular.z = 0.0;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
        "[drive_test] line lost, holding straight...");
    }

    cmd_pub_->publish(cmd);
  }

  // 成员
  agv_inventory_system::YellowLineFollower follower_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr forward_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr backward_srv_;

  std::atomic<bool> running_{false};
  std::atomic<bool> forward_{true};
  double linear_speed_{0.10};
  double backward_linear_speed_{-1.0};
  double lost_timeout_sec_{0.5};
  bool config_has_reverse_invert_{true};
  std::string cmd_vel_topic_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YellowLineDriveTestNode>());
  rclcpp::shutdown();
  return 0;
}
