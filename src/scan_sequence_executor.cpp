#include "wheeltec_inventory_system/scan_sequence_executor.hpp"

#include <chrono>
#include <iostream>
#include <thread>

namespace wheeltec_inventory_system
{

ScanSequenceExecutor::ScanSequenceExecutor(const ScanExecutionParams & params)
: params_(params)
{
}

void ScanSequenceExecutor::setParams(const ScanExecutionParams & params)
{
  params_ = params;
}

const ScanExecutionParams & ScanSequenceExecutor::params() const
{
  return params_;
}

bool ScanSequenceExecutor::execute(const std::vector<ScanStep> & steps) const
{
  return execute(steps, nullptr);
}

bool ScanSequenceExecutor::execute(
  const std::vector<ScanStep> & steps,
  const std::function<void(const ScanStep &)> & step_callback) const
{
  std::cout << "[scan_executor] execute step_count=" << steps.size() << std::endl;

  for (const auto & step : steps) {
    switch (step.step_type) {
      case ScanStepType::MOVE_TO_GRID:
        std::cout << "[scan_executor] move grid cabinet=" << step.cabinet_id
                  << " layer=" << step.layer_index
                  << " depth=" << step.depth_index << std::endl;
        sleepSeconds(params_.grid_motion_duration_sec);
        break;

      case ScanStepType::SCAN_GRID:
        std::cout << "[scan_executor] scan grid cabinet=" << step.cabinet_id
                  << " layer=" << step.layer_index
                  << " depth=" << step.depth_index << std::endl;
        sleepSeconds(params_.scan_duration_sec);
        break;

      case ScanStepType::MOVE_LIFT_TO_LEVEL:
        std::cout << "[scan_executor] lift move level=" << step.layer_index << std::endl;
        sleepSeconds(params_.lift_action_duration_sec);
        break;

      case ScanStepType::MOVE_LIFT_HOME:
        std::cout << "[scan_executor] lift home" << std::endl;
        sleepSeconds(params_.lift_action_duration_sec);
        break;
    }

    if (step_callback) {
      step_callback(step);
    }
  }

  std::cout << "[scan_executor] execute done" << std::endl;
  return true;
}

void ScanSequenceExecutor::sleepSeconds(double seconds)
{
  if (seconds <= 0.0) {
    return;
  }

  std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
}

}  // namespace wheeltec_inventory_system
