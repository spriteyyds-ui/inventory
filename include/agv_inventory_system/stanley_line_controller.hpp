#ifndef WHEELTEC_INVENTORY_SYSTEM__STANLEY_LINE_CONTROLLER_HPP_
#define WHEELTEC_INVENTORY_SYSTEM__STANLEY_LINE_CONTROLLER_HPP_

#include <algorithm>
#include <cmath>

namespace agv_inventory_system
{

struct StanleyLineConfig
{
  double cte_gain{0.30};
  double softening_speed{0.10};
  double max_yaw_offset_rad{0.05};
  double yaw_kp{0.40};
  double yaw_kd{0.0};
  double yaw_deadband_rad{0.02};
  double min_angular{0.015};
  double max_angular{0.15};
};

struct StanleyLineInput
{
  double line_y{0.0};
  double line_yaw{0.0};
  double current_y{0.0};
  double current_yaw{0.0};
  double linear_x{0.0};
};

struct StanleyLineOutput
{
  double angular_z{0.0};
  double cross_track_error{0.0};
  double yaw_offset{0.0};
  double corrected_yaw{0.0};
  double heading_error{0.0};
  double raw_angular{0.0};
  bool min_angular_applied{false};
  double direction_sign{1.0};
};

class StanleyLineController
{
public:
  explicit StanleyLineController(const StanleyLineConfig & config = StanleyLineConfig())
  : config_(config) {}

  void setConfig(const StanleyLineConfig & config) {config_ = config;}

  void reset()
  {
    last_heading_error_ = 0.0;
    last_time_sec_ = -1.0;
  }

  StanleyLineOutput compute(const StanleyLineInput & input, double now_sec)
  {
    StanleyLineOutput out;

    // 1. cross-track error
    out.cross_track_error = config_.cte_gain > 1e-9 ?
      (input.line_y - input.current_y) : 0.0;

    // 2. yaw_offset from Stanley formula, clamped
    const double abs_speed = std::abs(input.linear_x);
    const double denominator = abs_speed + config_.softening_speed;
    out.yaw_offset = std::atan2(
      config_.cte_gain * out.cross_track_error,
      denominator);
    out.yaw_offset = std::clamp(
      out.yaw_offset,
      -config_.max_yaw_offset_rad,
      config_.max_yaw_offset_rad);

    // 3. corrected yaw based on direction
    const double direction_sign = input.linear_x >= 0.0 ? 1.0 : -1.0;
    out.direction_sign = direction_sign;
    out.corrected_yaw = normalize_angle(input.line_yaw + direction_sign * out.yaw_offset);

    // 4. heading error
    out.heading_error = normalize_angle(out.corrected_yaw - input.current_yaw);

    // 5. PD control
    double raw_angular = config_.yaw_kp * out.heading_error;

    // D term
    if (config_.yaw_kd > 1e-9 && last_time_sec_ >= 0.0) {
      const double dt = now_sec - last_time_sec_;
      if (dt > 1e-6 && dt < 2.0) {
        const double heading_error_rate =
          normalize_angle(out.heading_error - last_heading_error_) / dt;
        raw_angular += config_.yaw_kd * heading_error_rate;
      }
    }

    // 6. deadband
    if (std::abs(out.heading_error) < config_.yaw_deadband_rad) {
      raw_angular = 0.0;
    }

    out.raw_angular = raw_angular;

    // 7. min angular enforcement
    out.min_angular_applied = false;
    if (raw_angular != 0.0 && std::abs(raw_angular) < config_.min_angular) {
      const double sign = raw_angular > 0.0 ? 1.0 : -1.0;
      raw_angular = sign * config_.min_angular;
      out.min_angular_applied = true;
    }

    // 8. clamp
    out.angular_z = std::clamp(
      raw_angular,
      -config_.max_angular,
      config_.max_angular);

    // update D state
    last_heading_error_ = out.heading_error;
    last_time_sec_ = now_sec;

    return out;
  }

private:
  StanleyLineConfig config_;
  double last_heading_error_{0.0};
  double last_time_sec_{-1.0};

  static double normalize_angle(double angle)
  {
    while (angle > M_PI) {angle -= 2.0 * M_PI;}
    while (angle < -M_PI) {angle += 2.0 * M_PI;}
    return angle;
  }
};

}  // namespace agv_inventory_system

#endif  // WHEELTEC_INVENTORY_SYSTEM__STANLEY_LINE_CONTROLLER_HPP_
