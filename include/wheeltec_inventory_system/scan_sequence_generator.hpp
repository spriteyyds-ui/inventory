#ifndef WHEELTEC_INVENTORY_SYSTEM__SCAN_SEQUENCE_GENERATOR_HPP_
#define WHEELTEC_INVENTORY_SYSTEM__SCAN_SEQUENCE_GENERATOR_HPP_

#include <string>
#include <vector>

namespace wheeltec_inventory_system
{

enum class ScanStepType
{
  MOVE_TO_GRID,
  SCAN_PLACEHOLDER,
  LIFT_PLACEHOLDER,
  LOWER_PLACEHOLDER
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

}  // namespace wheeltec_inventory_system

#endif  // WHEELTEC_INVENTORY_SYSTEM__SCAN_SEQUENCE_GENERATOR_HPP_
