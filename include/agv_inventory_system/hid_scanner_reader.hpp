#ifndef WHEELTEC_INVENTORY_SYSTEM__HID_SCANNER_READER_HPP_
#define WHEELTEC_INVENTORY_SYSTEM__HID_SCANNER_READER_HPP_

#include <chrono>
#include <set>
#include <string>
#include <vector>

namespace agv_inventory_system
{

struct HidScannerReaderConfig
{
  std::string device_path;
  bool grab_device{true};
  int inter_char_timeout_ms{200};
  int max_tags_per_location{10};
};

class HidScannerReader
{
public:
  HidScannerReader() = default;
  ~HidScannerReader();

  bool start(const HidScannerReaderConfig & config, std::string & error_message);
  bool poll_once(int timeout_ms, std::string & error_message);
  void finish_pending_tag();
  bool ready_after_quiet() const;
  const std::vector<std::string> & tags() const;
  void reset();

private:
  using Clock = std::chrono::steady_clock;

  bool process_key_event(unsigned short code);
  bool key_code_to_char(unsigned short code, char & ch) const;
  void append_current_tag_if_valid();
  void maybe_finish_pending_tag_by_timeout();
  void close_device();
  void drain_input_events();

  HidScannerReaderConfig config_;
  int fd_{-1};
  bool grabbed_{false};
  bool shift_for_next_key_{false};
  std::string current_tag_;
  std::vector<std::string> tags_;
  std::set<std::string> seen_tags_;
  Clock::time_point last_key_time_{};
  Clock::time_point last_activity_time_{};
};

}  // namespace agv_inventory_system

#endif  // WHEELTEC_INVENTORY_SYSTEM__HID_SCANNER_READER_HPP_
