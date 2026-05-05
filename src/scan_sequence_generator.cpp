#include "wheeltec_inventory_system/scan_sequence_generator.hpp"

#include <iostream>
#include <sstream>

namespace wheeltec_inventory_system
{

std::vector<ScanStep> ScanSequenceGenerator::generateCabinetSnakeSequence(
  int cabinet_id,
  int layers,
  int depth_count) const
{
  std::vector<ScanStep> steps;

  if (cabinet_id <= 0 || layers <= 0 || depth_count <= 0) {
    std::cout << "[scan_sequence_generator] invalid params cabinet=" << cabinet_id
              << " layers=" << layers << " depth_count=" << depth_count << std::endl;
    return steps;
  }

  std::cout << "[scan_sequence_generator] generate snake sequence cabinet=" << cabinet_id
            << " layers=" << layers << " depth_count=" << depth_count << std::endl;

  for (int layer = 1; layer <= layers; ++layer) {
    if (layer > 1) {
      steps.push_back(ScanStep{cabinet_id, layer, 0, ScanStepType::LIFT_PLACEHOLDER});
      std::cout << "[scan_sequence_generator] add lift placeholder cabinet=" << cabinet_id
                << " target_layer=" << layer << std::endl;
    }

    const bool forward = (layer % 2) == 1;
    const int start_depth = forward ? 1 : depth_count;
    const int end_depth = forward ? depth_count : 1;
    const int step_delta = forward ? 1 : -1;

    for (int depth = start_depth;; depth += step_delta) {
      const ScanStep move_step{cabinet_id, layer, depth, ScanStepType::MOVE_TO_GRID};
      const ScanStep scan_step{cabinet_id, layer, depth, ScanStepType::SCAN_PLACEHOLDER};
      steps.push_back(move_step);
      steps.push_back(scan_step);

      std::cout << "[scan_sequence_generator] add grid "
                << formatGridLabel(scan_step) << std::endl;

      if (depth == end_depth) {
        break;
      }
    }
  }

  if (layers > 1) {
    steps.push_back(ScanStep{cabinet_id, 1, 0, ScanStepType::LOWER_PLACEHOLDER});
    std::cout << "[scan_sequence_generator] add lower placeholder cabinet=" << cabinet_id
              << " target_layer=1" << std::endl;
  }

  std::cout << "[scan_sequence_generator] generated step_count=" << steps.size() << std::endl;
  return steps;
}

std::string ScanSequenceGenerator::stepTypeToString(ScanStepType step_type)
{
  switch (step_type) {
    case ScanStepType::MOVE_TO_GRID:
      return "MOVE_TO_GRID";
    case ScanStepType::SCAN_PLACEHOLDER:
      return "SCAN_PLACEHOLDER";
    case ScanStepType::LIFT_PLACEHOLDER:
      return "LIFT_PLACEHOLDER";
    case ScanStepType::LOWER_PLACEHOLDER:
      return "LOWER_PLACEHOLDER";
  }

  return "UNKNOWN";
}

std::string ScanSequenceGenerator::formatGridLabel(const ScanStep & step)
{
  std::ostringstream oss;
  oss << step.cabinet_id << "-" << step.layer_index << "-" << step.depth_index;
  return oss.str();
}

}  // namespace wheeltec_inventory_system
