#ifndef WHEELTEC_INVENTORY_SYSTEM__INVENTORY_SCANNER_HPP_
#define WHEELTEC_INVENTORY_SYSTEM__INVENTORY_SCANNER_HPP_

#include <chrono>
#include <string>

namespace agv_inventory_system
{

struct InventoryScannerConfig
{
  bool enabled{true};
  double scan_duration_sec{2.0};
  double scan_timeout_sec{5.0};
  int scan_retry_count{0};
  double scan_result_timeout_sec{2.0};
};

class InventoryScanner
{
public:
  void configure(const InventoryScannerConfig & config);
  const InventoryScannerConfig & config() const;

  bool start_grid_scan(int cabinet_id, int row, int level, int column);
  void update();
  bool is_scan_finished() const;
  bool scan_success() const;
  const std::string & last_scan_result() const;
  void reset();

private:
  using Clock = std::chrono::steady_clock;

  InventoryScannerConfig config_;
  bool active_{false};
  bool finished_{false};
  bool success_{false};
  int cabinet_id_{0};
  int row_{0};
  int level_{0};
  int column_{0};
  std::string last_result_;
  Clock::time_point start_time_{};
};

}  // namespace agv_inventory_system

#endif  // WHEELTEC_INVENTORY_SYSTEM__INVENTORY_SCANNER_HPP_
