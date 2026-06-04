#ifndef WHEELTEC_INVENTORY_SYSTEM__INVENTORY_SCANNER_HPP_
#define WHEELTEC_INVENTORY_SYSTEM__INVENTORY_SCANNER_HPP_

#include <chrono>
#include <set>
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
  bool rfid_reader_enabled{true};
  std::string rfid_reader_mode{"active_report_serial"};
  std::string rfid_serial_device{"/dev/ttyUSB0"};
  int rfid_serial_baud{9600};
  double rfid_scan_duration_sec{5.0};
  std::string rfid_frame_header{"1100EE00"};
  int rfid_frame_length{18};
  int rfid_epc_offset{4};
  int rfid_epc_length{12};
};

enum class InventoryScanOutputSource
{
  ACTIVE_REPORT_SERIAL_SUCCESS,
  ACTIVE_REPORT_SERIAL_EMPTY,
  ACTIVE_REPORT_SERIAL_FAILED
};

struct InventoryScanOutput
{
  InventoryScanOutputSource source{InventoryScanOutputSource::ACTIVE_REPORT_SERIAL_EMPTY};
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
  bool start_active_report_serial_reader(std::string & error_message);
  bool configure_active_report_serial_port(std::string & error_message);
  bool poll_active_report_serial_once(std::string & error_message);
  void parse_active_report_serial_buffer();
  void finish_active_report_serial_scan(bool reader_ok, const std::string & reason);
  void close_active_report_serial_device();
  void reset_active_report_serial_reader();
  void clear_scan_output();

  InventoryScannerConfig config_;
  bool active_{false};
  bool finished_{false};
  bool success_{false};
  bool active_report_serial_active_{false};
  int active_report_serial_fd_{-1};
  int cabinet_id_{0};
  int row_{0};
  int level_{0};
  int column_{0};
  std::string last_result_;
  Clock::time_point start_time_{};
  InventoryScanOutput last_output_;
  std::vector<unsigned char> active_report_serial_buffer_;
  std::vector<std::string> active_report_serial_epcs_;
  std::set<std::string> active_report_serial_seen_epcs_;
};

}  // namespace agv_inventory_system

#endif  // WHEELTEC_INVENTORY_SYSTEM__INVENTORY_SCANNER_HPP_
