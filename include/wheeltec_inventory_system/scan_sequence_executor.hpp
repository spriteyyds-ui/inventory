#ifndef WHEELTEC_INVENTORY_SYSTEM__SCAN_SEQUENCE_EXECUTOR_HPP_
#define WHEELTEC_INVENTORY_SYSTEM__SCAN_SEQUENCE_EXECUTOR_HPP_

#include <functional>
#include <vector>

#include "wheeltec_inventory_system/scan_sequence_generator.hpp"

namespace wheeltec_inventory_system
{

struct ScanExecutionParams
{
  double scan_placeholder_wait_sec{2.0};
  double lift_placeholder_wait_sec{3.0};
  double move_grid_placeholder_wait_sec{1.0};
};

class ScanSequenceExecutor
{
public:
  explicit ScanSequenceExecutor(const ScanExecutionParams & params = ScanExecutionParams());

  void setParams(const ScanExecutionParams & params);
  const ScanExecutionParams & params() const;

  bool execute(const std::vector<ScanStep> & steps) const;
  bool execute(
    const std::vector<ScanStep> & steps,
    const std::function<void(const ScanStep &)> & step_callback) const;

private:
  static void sleepSeconds(double seconds);

  ScanExecutionParams params_;
};

}  // namespace wheeltec_inventory_system

#endif  // WHEELTEC_INVENTORY_SYSTEM__SCAN_SEQUENCE_EXECUTOR_HPP_
