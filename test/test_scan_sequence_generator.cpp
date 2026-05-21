#include "agv_inventory_system/scan_sequence_generator.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace
{

std::vector<std::string> collectScanGridLabels(
  const std::vector<agv_inventory_system::ScanStep> & steps)
{
  std::vector<std::string> labels;
  for (const auto & step : steps) {
    if (step.step_type == agv_inventory_system::ScanStepType::SCAN_GRID) {
      labels.push_back(agv_inventory_system::ScanSequenceGenerator::formatGridLabel(step));
    }
  }
  return labels;
}

bool expectSequence(
  int cabinet_id,
  const std::vector<std::string> & actual,
  const std::vector<std::string> & expected)
{
  if (actual == expected) {
    std::cout << "[test_scan_sequence_generator] PASS cabinet=" << cabinet_id << std::endl;
    return true;
  }

  std::cout << "[test_scan_sequence_generator] FAIL cabinet=" << cabinet_id << std::endl;
  std::cout << "[test_scan_sequence_generator] expected:";
  for (const auto & item : expected) {
    std::cout << " " << item;
  }
  std::cout << std::endl;

  std::cout << "[test_scan_sequence_generator] actual:";
  for (const auto & item : actual) {
    std::cout << " " << item;
  }
  std::cout << std::endl;
  return false;
}

}  // namespace

int main()
{
  const agv_inventory_system::ScanSequenceGenerator generator;
  bool ok = true;

  ok = expectSequence(
    3,
    collectScanGridLabels(generator.generateCabinetSnakeSequence(3, 2, 3)),
    {"3-1-1", "3-1-2", "3-1-3", "3-2-3", "3-2-2", "3-2-1"}) && ok;

  ok = expectSequence(
    6,
    collectScanGridLabels(generator.generateCabinetSnakeSequence(6, 2, 3)),
    {"6-1-1", "6-1-2", "6-1-3", "6-2-3", "6-2-2", "6-2-1"}) && ok;

  return ok ? 0 : 1;
}
