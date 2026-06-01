#ifndef WHEELTEC_INVENTORY_SYSTEM__INVENTORY_SCANNER_HPP_
#define WHEELTEC_INVENTORY_SYSTEM__INVENTORY_SCANNER_HPP_

#include "agv_inventory_system/hid_scanner_reader.hpp"

#include <chrono>
#include <string>
#include <vector>

namespace agv_inventory_system
{

struct InventoryScannerConfig
{
  bool enabled{true};
  double scan_duration_sec{2.0};
  double scan_timeout_sec{5.0};
  int scan_retry_count{0};
  double scan_result_timeout_sec{2.0};
  std::string rfid_reader_mode{"hid_keyboard"};
  std::string rfid_hid_device_path{"auto"};
  bool rfid_hid_grab_device{true};
  double rfid_hid_scan_timeout_sec{5.0};
  int rfid_hid_inter_char_timeout_ms{200};
  int rfid_hid_max_tags_per_location{10};
  bool rfid_hid_allow_empty_result{false};
  bool rfid_hid_fallback_to_placeholder{true};
};

enum class InventoryScanOutputSource
{
  PLACEHOLDER_MODE,
  HID_SUCCESS,
  HID_EMPTY_SUCCESS,
  HID_FALLBACK_PLACEHOLDER,
  HID_FAILED
};

struct InventoryScanOutput
{
  InventoryScanOutputSource source{InventoryScanOutputSource::PLACEHOLDER_MODE};
  bool use_real_rfids_result{false};
  bool fallback_to_placeholder{false};
  std::vector<std::string> rfids;
  std::string message;
};

class InventoryScanner
{
public:
  ~InventoryScanner();

  void configure(const InventoryScannerConfig & config);
  const InventoryScannerConfig & config() const;

  bool start_grid_scan(int cabinet_id, int row, int level, int column);
  void update();
  bool is_scan_finished() const;
  bool scan_success() const;
  const std::string & last_scan_result() const;
  const InventoryScanOutput & last_scan_output() const;
  void reset();

private:
  using Clock = std::chrono::steady_clock;

  static std::string normalize_reader_mode(std::string mode);
  void finish_placeholder_scan(const std::string & status);
  void finish_hid_scan_success(const std::vector<std::string> & tags);
  void finish_hid_scan_failure(const std::string & reason);
  void activate_hid_fallback(const std::string & reason);
  void clear_scan_output();

  InventoryScannerConfig config_;
  bool active_{false};
  bool finished_{false};
  bool success_{false};
  bool hid_scan_active_{false};
  bool placeholder_fallback_active_{false};
  int cabinet_id_{0};
  int row_{0};
  int level_{0};
  int column_{0};
  std::string last_result_;
  Clock::time_point start_time_{};
  InventoryScanOutput last_output_;
  HidScannerReader hid_reader_;
};

}  // namespace agv_inventory_system

#endif  // WHEELTEC_INVENTORY_SYSTEM__INVENTORY_SCANNER_HPP_
