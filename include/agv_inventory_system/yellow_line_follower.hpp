#ifndef AGV_INVENTORY_SYSTEM__YELLOW_LINE_FOLLOWER_HPP_
#define AGV_INVENTORY_SYSTEM__YELLOW_LINE_FOLLOWER_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace agv_inventory_system
{

struct YellowLineFollowerConfig
{
  // ROI 裁剪比例
  double roi_y_min_ratio{0.55};
  double roi_y_max_ratio{0.95};

  // 目标位置比例（黄线在图像宽度中的目标位置）
  double target_x_ratio{0.65};
  double target_x_offset_px{0.0};

  // 黄色 HSV 阈值
  int yellow_h_min{15};
  int yellow_h_max{40};
  int yellow_s_min{80};
  int yellow_s_max{255};
  int yellow_v_min{80};
  int yellow_v_max{255};

  // 轮廓过滤
  double min_area{300.0};

  // 丢线超时（秒）
  double lost_timeout_sec{0.5};

  // PD 控制参数（前进）
  double kp{0.8};
  double kd{0.05};
  double max_angular{0.25};
  bool reverse_invert_angular{true};

  // 后退独立参数（<0 时使用前进参数的默认值）
  double backward_target_x_ratio{-1.0};  // <0 表示复用前进 target_x_ratio
  double backward_kp{-1.0};              // <0 表示复用前进 kp
  double backward_kd{-1.0};              // <0 表示复用前进 kd
  double backward_max_angular{-1.0};     // <0 表示复用前进 max_angular
};

struct YellowLineDetectResult
{
  bool detected{false};
  double line_x{0.0};
  double target_x{0.0};
  double error_px{0.0};
  double error_norm{0.0};
  double last_error_norm{0.0};
  double last_detect_time_sec{0.0};
  int image_width{0};
  int image_height{0};
};

class YellowLineFollower
{
public:
  explicit YellowLineFollower(const YellowLineFollowerConfig & config = YellowLineFollowerConfig())
  : config_(config) {}

  void setConfig(const YellowLineFollowerConfig & config) {config_ = config;}

  // 设置当前行驶方向（true=前进, false=后退），影响 target_x 选择
  void setDirection(bool forward) {forward_ = forward;}

  void reset()
  {
    result_ = YellowLineDetectResult{};
    last_angular_z_ = 0.0;
    last_time_sec_ = -1.0;
    forward_ = true;
  }

  // 图像识别：处理一帧 BGR 图像，更新内部检测结果
  // 返回 true 表示检测到黄线
  bool processImage(const cv::Mat & bgr_image, double now_sec)
  {
    if (bgr_image.empty()) {
      result_.detected = false;
      return false;
    }

    const int img_w = bgr_image.cols;
    const int img_h = bgr_image.rows;
    result_.image_width = img_w;
    result_.image_height = img_h;

    // 1. 裁剪 ROI
    const int y_min = static_cast<int>(img_h * config_.roi_y_min_ratio);
    const int y_max = static_cast<int>(img_h * config_.roi_y_max_ratio);
    const int roi_y0 = std::max(0, std::min(y_min, img_h - 1));
    const int roi_y1 = std::max(roi_y0 + 1, std::min(y_max, img_h));
    cv::Mat roi = bgr_image(cv::Range(roi_y0, roi_y1), cv::Range(0, img_w));

    // 2. 转 HSV
    cv::Mat hsv;
    cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);

    // 3. 黄色阈值分割
    cv::Mat mask;
    cv::inRange(
      hsv,
      cv::Scalar(config_.yellow_h_min, config_.yellow_s_min, config_.yellow_v_min),
      cv::Scalar(config_.yellow_h_max, config_.yellow_s_max, config_.yellow_v_max),
      mask);

    // 4. 形态学去噪
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    // 5. 查找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 6. 过滤面积，选择最大黄色区域
    double best_area = 0.0;
    int best_idx = -1;
    for (int i = 0; i < static_cast<int>(contours.size()); ++i) {
      const double area = cv::contourArea(contours[i]);
      if (area >= config_.min_area && area > best_area) {
        best_area = area;
        best_idx = i;
      }
    }

    if (best_idx >= 0) {
      // 7. 计算质心
      cv::Moments mu = cv::moments(contours[best_idx]);
      if (mu.m00 > 1e-6) {
        const double cx_roi = mu.m10 / mu.m00;
        result_.line_x = cx_roi;  // 在 ROI 坐标系中，x 与原图一致
        const double active_target_ratio =
          (!forward_ && config_.backward_target_x_ratio >= 0.0)
          ? config_.backward_target_x_ratio : config_.target_x_ratio;
        result_.target_x = img_w * active_target_ratio + config_.target_x_offset_px;
        result_.error_px = result_.line_x - result_.target_x;
        result_.error_norm = result_.error_px / static_cast<double>(img_w);
        result_.detected = true;
        result_.last_detect_time_sec = now_sec;

        // 保存上一次 error_norm 用于 D 项
        if (last_time_sec_ < 0.0) {
          result_.last_error_norm = result_.error_norm;
        } else {
          result_.last_error_norm = last_error_norm_;
        }
        last_error_norm_ = result_.error_norm;
        last_time_sec_ = now_sec;
        return true;
      }
    }

    // 未检测到
    result_.detected = false;
    return false;
  }

  // 获取黄线巡线角速度修正
  // 返回 true 表示黄线可用且输出了有效角速度
  // 返回 false 表示黄线丢失
  bool getAngularCorrection(double linear_x, double & angular_z)
  {
    if (!result_.detected) {
      angular_z = 0.0;
      return false;
    }

    // 选择前进/后退参数
    const bool backward = (linear_x < 0.0);
    const double eff_kp = (backward && config_.backward_kp >= 0.0)
      ? config_.backward_kp : config_.kp;
    const double eff_kd = (backward && config_.backward_kd >= 0.0)
      ? config_.backward_kd : config_.kd;
    const double eff_max = (backward && config_.backward_max_angular >= 0.0)
      ? config_.backward_max_angular : config_.max_angular;

    // PD 控制律
    double d_error_norm = 0.0;
    if (last_time_sec_ > 0.0 && result_.last_detect_time_sec > 0.0) {
      const double dt = result_.last_detect_time_sec - last_time_sec_;
      if (dt > 1e-6 && dt < 2.0) {
        d_error_norm = (result_.error_norm - result_.last_error_norm) / dt;
      }
    }

    double raw_angular = -eff_kp * result_.error_norm - eff_kd * d_error_norm;

    // 后退时翻转角速度方向
    if (backward && config_.reverse_invert_angular) {
      raw_angular *= -1.0;
    }

    // 限幅
    angular_z = std::clamp(raw_angular, -eff_max, eff_max);
    last_angular_z_ = angular_z;
    return true;
  }

  // 检查黄线是否丢失超过超时
  bool isLostTimeout(double now_sec) const
  {
    if (result_.detected) {
      return false;
    }
    if (result_.last_detect_time_sec <= 0.0) {
      return true;  // 从未检测到
    }
    return (now_sec - result_.last_detect_time_sec) >= config_.lost_timeout_sec;
  }

  // 获取最近一次检测结果
  const YellowLineDetectResult & getResult() const {return result_;}

  // 是否最近一次检测到黄线
  bool isDetected() const {return result_.detected;}

  // 获取上次检测时间
  double getLastDetectTime() const {return result_.last_detect_time_sec;}

  // 绘制调试图像
  // 在原图上叠加: ROI 矩形、检测到的 line_x（绿线）、target_x（红线）、error_norm 文字
  cv::Mat drawDebug(const cv::Mat & bgr_image, double now_sec) const
  {
    cv::Mat debug = bgr_image.clone();
    if (debug.empty()) {
      return debug;
    }

    const int img_w = debug.cols;
    const int img_h = debug.rows;
    const int y_min = static_cast<int>(img_h * config_.roi_y_min_ratio);
    const int y_max = static_cast<int>(img_h * config_.roi_y_max_ratio);

    // 画 ROI 矩形（蓝色虚线）
    cv::rectangle(
      debug,
      cv::Point(0, y_min),
      cv::Point(img_w - 1, y_max),
      cv::Scalar(255, 0, 0), 1, cv::LINE_4);

    if (result_.detected) {
      // 画检测到的 line_x（绿色竖线）
      const int lx = static_cast<int>(result_.line_x);
      cv::line(debug, cv::Point(lx, y_min), cv::Point(lx, y_max),
        cv::Scalar(0, 255, 0), 2);

      // 画 target_x（红色竖线）
      const int tx = static_cast<int>(result_.target_x);
      cv::line(debug, cv::Point(tx, y_min), cv::Point(tx, y_max),
        cv::Scalar(0, 0, 255), 2);

      // 标注 error_norm
      char buf[128];
      std::snprintf(buf, sizeof(buf), "err=%.3f ang=%.3f",
        result_.error_norm, last_angular_z_);
      cv::putText(debug, buf, cv::Point(10, y_min - 10),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);

      cv::putText(debug, "DETECTED", cv::Point(10, 20),
        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    } else {
      // 丢失状态
      const double lost_dur = (result_.last_detect_time_sec > 0.0)
        ? (now_sec - result_.last_detect_time_sec) : -1.0;
      char buf[128];
      if (lost_dur >= 0.0) {
        std::snprintf(buf, sizeof(buf), "LOST %.2fs", lost_dur);
      } else {
        std::snprintf(buf, sizeof(buf), "NO DETECT");
      }
      cv::putText(debug, buf, cv::Point(10, 20),
        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
    }

    return debug;
  }

private:
  YellowLineFollowerConfig config_;
  YellowLineDetectResult result_;
  double last_angular_z_{0.0};
  double last_time_sec_{-1.0};
  double last_error_norm_{0.0};
  bool forward_{true};
};

}  // namespace agv_inventory_system

#endif  // AGV_INVENTORY_SYSTEM__YELLOW_LINE_FOLLOWER_HPP_
