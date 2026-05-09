#include "wheeltec_inventory_system/inventory_scanner.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <sstream>

namespace wheeltec_inventory_system
{

void InventoryScanner::configure(const InventoryScannerConfig & config)
{
  config_ = config;
  config_.scan_duration_sec = std::max(0.0, config_.scan_duration_sec);
  config_.scan_timeout_sec = std::max(config_.scan_duration_sec, config_.scan_timeout_sec);
  config_.scan_retry_count = std::max(0, config_.scan_retry_count);
  config_.scan_result_timeout_sec = std::max(0.0, config_.scan_result_timeout_sec);
}

const InventoryScannerConfig & InventoryScanner::config() const
{
  return config_;
}

bool InventoryScanner::start_grid_scan(int cabinet_id, int row, int level, int column)
{
  if (!config_.enabled) {
    finished_ = true;
    success_ = true;
    last_result_ = "scanner_disabled";
    std::cout << "[scanner] disabled cabinet=" << cabinet_id
              << " level=" << level
              << " column=" << column << std::endl;
    return true;
  }

  active_ = true;
  finished_ = false;
  success_ = false;
  cabinet_id_ = cabinet_id;
  row_ = row;
  level_ = level;
  column_ = column;
  last_result_.clear();
  start_time_ = Clock::now();

  std::cout << "[scanner] scan started cabinet=" << cabinet_id_
            << " row=" << row_
            << " level=" << level_
            << " column=" << column_ << std::endl;
  return true;
}

void InventoryScanner::update()
{
  if (!active_ || finished_) {
    return;
  }

  const auto now = Clock::now();
  const double elapsed =
    std::chrono::duration<double>(now - start_time_).count();
  if (elapsed > config_.scan_timeout_sec) {
    active_ = false;
    finished_ = true;
    success_ = false;
    last_result_ = "scan_timeout";
    std::cout << "[scanner] scan failed cabinet=" << cabinet_id_
              << " level=" << level_
              << " column=" << column_
              << " elapsed=" << elapsed << std::endl;
    return;
  }

  if (elapsed >= config_.scan_duration_sec) {
    std::ostringstream result;
    result << "scan_ok:cabinet=" << cabinet_id_
           << ",row=" << row_
           << ",level=" << level_
           << ",column=" << column_;
    active_ = false;
    finished_ = true;
    success_ = true;
    last_result_ = result.str();
    std::cout << "[scanner] scan finished cabinet=" << cabinet_id_
              << " level=" << level_
              << " column=" << column_ << std::endl;
  }
}

bool InventoryScanner::is_scan_finished() const
{
  return finished_;
}

bool InventoryScanner::scan_success() const
{
  return success_;
}

const std::string & InventoryScanner::last_scan_result() const
{
  return last_result_;
}

void InventoryScanner::reset()
{
  active_ = false;
  finished_ = false;
  success_ = false;
  cabinet_id_ = 0;
  row_ = 0;
  level_ = 0;
  column_ = 0;
  last_result_.clear();
  start_time_ = Clock::time_point{};
}

}  // namespace wheeltec_inventory_system
