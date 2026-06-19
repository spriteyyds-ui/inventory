// Copyright (c) 2026 郁有冬 <spriteyyds@gmail.com>. All rights reserved.
#ifndef AGV_INVENTORY_SYSTEM__SCAN_SEQUENCE_GENERATOR_HPP_
#define AGV_INVENTORY_SYSTEM__SCAN_SEQUENCE_GENERATOR_HPP_

#include <string>
#include <vector>

namespace agv_inventory_system
{

enum class ScanStepType
{
  MOVE_TO_GRID,
  SCAN_GRID,
  MOVE_LIFT_TO_LEVEL,
  MOVE_LIFT_HOME
};

struct ScanStep
{
  int cabinet_id{0};
  int layer_index{0};
  int depth_index{0};
  ScanStepType step_type{ScanStepType::MOVE_TO_GRID};
};

class ScanSequenceGenerator
{
public:
  std::vector<ScanStep> generateCabinetSnakeSequence(
    int cabinet_id,
    int layers,
    int depth_count) const;

  static std::string stepTypeToString(ScanStepType step_type);
  static std::string formatGridLabel(const ScanStep & step);
};

}  // namespace agv_inventory_system

#endif  // AGV_INVENTORY_SYSTEM__SCAN_SEQUENCE_GENERATOR_HPP_
