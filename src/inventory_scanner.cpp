#include "agv_inventory_system/inventory_scanner.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <iostream>
#include <sstream>

namespace agv_inventory_system
{

InventoryScanner::~InventoryScanner()
{
  reset();
}

void InventoryScanner::configure(const InventoryScannerConfig & config)
{
  config_ = config;
  config_.scan_duration_sec = std::max(0.0, config_.scan_duration_sec);
  config_.scan_timeout_sec = std::max(config_.scan_duration_sec, config_.scan_timeout_sec);
  config_.scan_retry_count = std::max(0, config_.scan_retry_count);
  config_.scan_result_timeout_sec = std::max(0.0, config_.scan_result_timeout_sec);
  config_.rfid_reader_mode = normalize_reader_mode(config_.rfid_reader_mode);
  config_.rfid_hid_scan_timeout_sec = std::max(0.1, config_.rfid_hid_scan_timeout_sec);
  config_.rfid_hid_inter_char_timeout_ms =
    std::max(1, config_.rfid_hid_inter_char_timeout_ms);
  config_.rfid_hid_max_tags_per_location =
    std::max(0, config_.rfid_hid_max_tags_per_location);
}

const InventoryScannerConfig & InventoryScanner::config() const
{
  return config_;
}

bool InventoryScanner::start_grid_scan(int cabinet_id, int row, int level, int column)
{
  hid_reader_.reset();
  clear_scan_output();
  placeholder_fallback_active_ = false;
  hid_scan_active_ = false;

  if (!config_.enabled) {
    finished_ = true;
    success_ = true;
    last_result_ = "scanner_disabled";
    last_output_.source = InventoryScanOutputSource::PLACEHOLDER_MODE;
    last_output_.message = last_result_;
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
            << " column=" << column_
            << " reader_mode=" << config_.rfid_reader_mode << std::endl;
  if (config_.rfid_reader_mode == "hid_keyboard") {
    HidScannerReaderConfig reader_config;
    reader_config.device_path = config_.rfid_hid_device_path;
    reader_config.grab_device = config_.rfid_hid_grab_device;
    reader_config.inter_char_timeout_ms = config_.rfid_hid_inter_char_timeout_ms;
    reader_config.max_tags_per_location = config_.rfid_hid_max_tags_per_location;
    std::string error_message;
    if (!hid_reader_.start(reader_config, error_message)) {
      finish_hid_scan_failure(error_message);
    } else {
      hid_scan_active_ = true;
      if (!config_.rfid_hid_grab_device) {
        std::cout << "[scanner][HID] WARNING EVIOCGRAB disabled; scanner input may reach terminal/GUI"
                  << std::endl;
      }
    }
  }
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

  if (hid_scan_active_) {
    std::string error_message;
    if (!hid_reader_.poll_once(0, error_message)) {
      finish_hid_scan_failure(error_message);
      return;
    }

    if (elapsed >= config_.rfid_hid_scan_timeout_sec) {
      hid_reader_.finish_pending_tag();
      const auto tags = hid_reader_.tags();
      hid_reader_.reset();
      hid_scan_active_ = false;
      if (!tags.empty() || config_.rfid_hid_allow_empty_result) {
        finish_hid_scan_success(tags);
      } else {
        finish_hid_scan_failure("HID scan timeout without tag");
      }
      return;
    }

    if (!hid_reader_.tags().empty() && hid_reader_.ready_after_quiet()) {
      hid_reader_.finish_pending_tag();
      const auto tags = hid_reader_.tags();
      hid_reader_.reset();
      hid_scan_active_ = false;
      finish_hid_scan_success(tags);
      return;
    }
    return;
  }

  if (elapsed > config_.scan_timeout_sec) {
    active_ = false;
    finished_ = true;
    success_ = false;
    last_result_ = "scan_timeout";
    last_output_.source = InventoryScanOutputSource::HID_FAILED;
    last_output_.message = last_result_;
    std::cout << "[scanner] scan failed cabinet=" << cabinet_id_
              << " level=" << level_
              << " column=" << column_
              << " elapsed=" << elapsed << std::endl;
    return;
  }

  if (elapsed >= config_.scan_duration_sec) {
    finish_placeholder_scan(placeholder_fallback_active_ ? "hid_fallback_placeholder" : "scan_ok");
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

const InventoryScanOutput & InventoryScanner::last_scan_output() const
{
  return last_output_;
}

void InventoryScanner::reset()
{
  hid_reader_.reset();
  active_ = false;
  finished_ = false;
  success_ = false;
  hid_scan_active_ = false;
  placeholder_fallback_active_ = false;
  cabinet_id_ = 0;
  row_ = 0;
  level_ = 0;
  column_ = 0;
  last_result_.clear();
  start_time_ = Clock::time_point{};
  clear_scan_output();
}

std::string InventoryScanner::normalize_reader_mode(std::string mode)
{
  std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (mode == "hid_keyboard") {
    return mode;
  }
  return "placeholder";
}

void InventoryScanner::finish_placeholder_scan(const std::string & status)
{
  std::ostringstream result;
  result << status << ":cabinet=" << cabinet_id_
         << ",row=" << row_
         << ",level=" << level_
         << ",column=" << column_;
  active_ = false;
  finished_ = true;
  success_ = true;
  last_result_ = result.str();
  last_output_.source = placeholder_fallback_active_ ?
    InventoryScanOutputSource::HID_FALLBACK_PLACEHOLDER :
    InventoryScanOutputSource::PLACEHOLDER_MODE;
  last_output_.use_real_rfids_result = false;
  last_output_.fallback_to_placeholder = placeholder_fallback_active_;
  last_output_.rfids.clear();
  last_output_.message = last_result_;
  std::cout << "[scanner] scan finished cabinet=" << cabinet_id_
            << " level=" << level_
            << " column=" << column_
            << " source=" << (placeholder_fallback_active_ ? "hid_fallback_placeholder" : "placeholder")
            << std::endl;
}

void InventoryScanner::finish_hid_scan_success(const std::vector<std::string> & tags)
{
  active_ = false;
  finished_ = true;
  success_ = true;
  std::ostringstream result;
  result << "hid_scan_ok:cabinet=" << cabinet_id_
         << ",row=" << row_
         << ",level=" << level_
         << ",column=" << column_
         << ",rfid_count=" << tags.size();
  last_result_ = result.str();
  last_output_.source = tags.empty() ?
    InventoryScanOutputSource::HID_EMPTY_SUCCESS :
    InventoryScanOutputSource::HID_SUCCESS;
  last_output_.use_real_rfids_result = true;
  last_output_.fallback_to_placeholder = false;
  last_output_.rfids = tags;
  last_output_.message = last_result_;
  std::cout << "[scanner][HID] scan finished cabinet=" << cabinet_id_
            << " level=" << level_
            << " column=" << column_
            << " rfid_count=" << tags.size() << std::endl;
}

void InventoryScanner::finish_hid_scan_failure(const std::string & reason)
{
  hid_reader_.reset();
  hid_scan_active_ = false;
  if (config_.rfid_hid_fallback_to_placeholder) {
    activate_hid_fallback(reason);
    return;
  }

  active_ = false;
  finished_ = true;
  success_ = false;
  last_result_ = "hid_scan_failed:" + reason;
  last_output_.source = InventoryScanOutputSource::HID_FAILED;
  last_output_.use_real_rfids_result = false;
  last_output_.fallback_to_placeholder = false;
  last_output_.rfids.clear();
  last_output_.message = reason;
  std::cout << "[scanner][HID] scan failed cabinet=" << cabinet_id_
            << " level=" << level_
            << " column=" << column_
            << " reason=\"" << reason << "\"" << std::endl;
}

void InventoryScanner::activate_hid_fallback(const std::string & reason)
{
  hid_reader_.reset();
  hid_scan_active_ = false;
  placeholder_fallback_active_ = true;
  start_time_ = Clock::now();
  std::cout << "[scanner][HID] WARNING fallback to placeholder cabinet=" << cabinet_id_
            << " level=" << level_
            << " column=" << column_
            << " reason=\"" << reason << "\"" << std::endl;
}

void InventoryScanner::clear_scan_output()
{
  last_output_ = InventoryScanOutput{};
}

}  // namespace agv_inventory_system
