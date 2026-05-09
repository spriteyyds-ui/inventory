#include "wheeltec_inventory_system/lift_controller.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>

namespace wheeltec_inventory_system
{

void LiftController::configure(const LiftControllerConfig & config)
{
  config_ = config;
  config_.lift_motion_duration_sec = std::max(0.0, config_.lift_motion_duration_sec);
  config_.lift_motion_timeout_sec =
    std::max(config_.lift_motion_duration_sec, config_.lift_motion_timeout_sec);
  config_.lift_level_count = std::max(1, config_.lift_level_count);
  config_.lift_home_level = std::clamp(config_.lift_home_level, 1, config_.lift_level_count);
  current_level_ = std::clamp(current_level_, 1, config_.lift_level_count);
  target_level_ = current_level_;
}

const LiftControllerConfig & LiftController::config() const
{
  return config_;
}

bool LiftController::move_to_level(int level)
{
  const int clamped_level = std::clamp(level, 1, config_.lift_level_count);
  if (!config_.enabled || clamped_level == current_level_) {
    active_ = false;
    finished_ = true;
    success_ = true;
    target_level_ = clamped_level;
    current_level_ = clamped_level;
    std::cout << "[lift] motion finished level=" << current_level_ << std::endl;
    return true;
  }

  active_ = true;
  finished_ = false;
  success_ = false;
  target_level_ = clamped_level;
  start_time_ = Clock::now();

  std::cout << "[lift] moving to level " << target_level_
            << " from " << current_level_ << std::endl;
  return true;
}

void LiftController::update()
{
  if (!active_ || finished_) {
    return;
  }

  const auto now = Clock::now();
  const double elapsed =
    std::chrono::duration<double>(now - start_time_).count();
  if (elapsed > config_.lift_motion_timeout_sec) {
    active_ = false;
    finished_ = true;
    success_ = false;
    std::cout << "[lift] motion failed target_level=" << target_level_
              << " elapsed=" << elapsed << std::endl;
    return;
  }

  if (elapsed >= config_.lift_motion_duration_sec) {
    active_ = false;
    finished_ = true;
    success_ = true;
    current_level_ = target_level_;
    std::cout << "[lift] motion finished level=" << current_level_ << std::endl;
  }
}

bool LiftController::is_motion_finished() const
{
  return finished_;
}

bool LiftController::motion_success() const
{
  return success_;
}

int LiftController::current_level() const
{
  return current_level_;
}

void LiftController::reset()
{
  active_ = false;
  finished_ = false;
  success_ = true;
  current_level_ = config_.lift_home_level;
  target_level_ = current_level_;
  start_time_ = Clock::time_point{};
}

}  // namespace wheeltec_inventory_system
