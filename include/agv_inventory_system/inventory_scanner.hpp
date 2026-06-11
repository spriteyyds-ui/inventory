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
  // Active report serial mode (old reader)
  bool rfid_reader_enabled{true};
  std::string rfid_reader_mode{"active_report_serial"};
  std::string rfid_serial_device{"/dev/ttyUSB0"};
  int rfid_serial_baud{9600};
  double rfid_scan_duration_sec{5.0};
  int rfid_frame_min_length{8};
  int rfid_frame_max_length{64};
  // UHFReader188 answer mode parameters
  std::string uhf_reader_serial_port{
    "/dev/serial/by-id/usb-Prolific_Technology_Inc._"
    "USB-Serial_Controller_CTA4b2A7N11-if00-port0"};
  int uhf_reader_baudrate{57600};
  int uhf_reader_address{0x00};
  int uhf_reader_q_value{2};
  int uhf_reader_session{0};
  int uhf_reader_scan_rounds_per_cell{5};
  double uhf_reader_scan_interval_sec{0.5};
  double uhf_reader_frame_timeout_sec{2.5};
  bool uhf_reader_collect_follow_frames{true};
  int uhf_reader_max_follow_frames{10};
  bool uhf_reader_fallback_single_cmd{true};
  bool uhf_reader_single_cmd_enabled{true};
  bool uhf_reader_debug_hex_log{false};
};

enum class InventoryScanOutputSource
{
  ACTIVE_REPORT_SERIAL_SUCCESS,
  ACTIVE_REPORT_SERIAL_EMPTY,
  ACTIVE_REPORT_SERIAL_FAILED,
  UHF_READER188_SUCCESS,
  UHF_READER188_EMPTY,
  UHF_READER188_FAILED
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

  // Active report serial mode (old reader)
  bool start_active_report_serial_reader(std::string & error_message);
  bool configure_active_report_serial_port(std::string & error_message);
  bool poll_active_report_serial_once(std::string & error_message);
  void parse_active_report_serial_buffer();
  void finish_active_report_serial_scan(bool reader_ok, const std::string & reason);
  void close_active_report_serial_device();
  void reset_active_report_serial_reader();
  void clear_scan_output();

  // UHFReader188 answer mode
  static bool is_uhf_reader188_mode(const std::string & mode);
  bool start_uhf_reader188(std::string & error_message);
  bool configure_uhf_reader188_port(std::string & error_message);
  bool uhf_reader188_send_cmd(const std::vector<unsigned char> & cmd);
  bool uhf_reader188_read_frame(std::vector<unsigned char> & frame);
  bool uhf_reader188_read_frame_with_timeout(
    std::vector<unsigned char> & frame, double timeout_sec);
  static unsigned short uhf_crc16(const unsigned char * data, std::size_t length);
  static std::vector<unsigned char> uhf_build_cmd(
    int addr, int cmd, const std::vector<unsigned char> & data);
  static std::vector<unsigned char> uhf_build_inventory_short(
    int addr, int q_value, int session);
  static std::vector<unsigned char> uhf_build_inventory_single(int addr);
  static std::vector<unsigned char> uhf_build_reader_info_cmd(int addr);
  static bool uhf_parse_tags(
    const unsigned char * data, std::size_t length,
    std::set<std::string> & seen, std::vector<std::string> & rfids);
  void uhf_collect_inventory_round();
  void uhf_run_cell_scan();
  void finish_uhf_reader188_scan(bool reader_ok, const std::string & reason);
  void close_uhf_reader188_device();
  void reset_uhf_reader188();

  // Shared
  InventoryScannerConfig config_;
  bool active_{false};
  bool finished_{false};
  bool success_{false};

  // Active report serial state
  bool active_report_serial_active_{false};
  int active_report_serial_fd_{-1};
  std::vector<unsigned char> active_report_serial_buffer_;
  std::vector<std::string> active_report_serial_rfids_;
  std::set<std::string> active_report_serial_seen_rfids_;

  // UHFReader188 state
  bool uhf_active_{false};
  int uhf_fd_{-1};
  std::vector<std::string> uhf_rfids_;
  std::set<std::string> uhf_seen_rfids_;
  std::vector<std::string> uhf_error_log_;

  // Scan position
  int cabinet_id_{0};
  int row_{0};
  int level_{0};
  int column_{0};
  std::string last_result_;
  Clock::time_point start_time_{};
  InventoryScanOutput last_output_;
};

}  // namespace agv_inventory_system

#endif  // WHEELTEC_INVENTORY_SYSTEM__INVENTORY_SCANNER_HPP_