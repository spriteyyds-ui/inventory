#ifndef WHEELTEC_INVENTORY_SYSTEM__LIFT_CONTROLLER_HPP_
#define WHEELTEC_INVENTORY_SYSTEM__LIFT_CONTROLLER_HPP_

#include <chrono>

namespace wheeltec_inventory_system
{

struct LiftControllerConfig
{
  bool enabled{true};
  double lift_motion_duration_sec{3.0};
  double lift_motion_timeout_sec{6.0};
  int lift_level_count{2};
  int lift_home_level{1};
};

class LiftController
{
public:
  void configure(const LiftControllerConfig & config);
  const LiftControllerConfig & config() const;

  bool move_to_level(int level);
  void update();
  bool is_motion_finished() const;
  bool motion_success() const;
  int current_level() const;
  void reset();

private:
  using Clock = std::chrono::steady_clock;

  LiftControllerConfig config_;
  bool active_{false};
  bool finished_{false};
  bool success_{true};
  int current_level_{1};
  int target_level_{1};
  Clock::time_point start_time_{};
};

}  // namespace wheeltec_inventory_system

#endif  // WHEELTEC_INVENTORY_SYSTEM__LIFT_CONTROLLER_HPP_
