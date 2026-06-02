#ifndef WHEELTEC_INVENTORY_SYSTEM__RFID_SCAN_LOG_WRITER_HPP_
#define WHEELTEC_INVENTORY_SYSTEM__RFID_SCAN_LOG_WRITER_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace agv_inventory_system
{

struct RfidScanLogWriterConfig
{
  bool enabled{true};
  std::string path{"/home/wheeltec/wheeltec_ros2/rfid_scan_logs/rfid_scan_records.jsonl"};
  bool write_batch_summary{true};
};

struct RfidScanCellLogRecord
{
  std::string mission_mode;
  int cabinet{0};
  int layer{0};
  int grid{0};
  std::string location_rfid;
  std::vector<std::string> rfids;
  std::string reader_mode;
  std::string source;
  bool fallback_to_placeholder{false};
  bool scan_success{false};
  std::size_t batch_item_count{0};
};

class RfidScanLogWriter
{
public:
  void configure(const RfidScanLogWriterConfig & config);
  const RfidScanLogWriterConfig & config() const;

  bool appendScanCellCached(
    const RfidScanCellLogRecord & record,
    std::string * error_message = nullptr) const;

  bool appendBatchUploadPrepare(
    const std::string & reason,
    std::size_t item_count,
    const std::string & url,
    std::string * error_message = nullptr) const;

  bool appendBatchUploadSuccess(
    std::size_t item_count,
    std::string * error_message = nullptr) const;

  bool appendBatchUploadFailed(
    std::size_t item_count,
    const std::string & fail_policy,
    std::string * error_message = nullptr) const;

  bool appendBatchUploadSkipped(
    const std::string & reason,
    std::size_t item_count,
    std::string * error_message = nullptr) const;

  bool appendJsonLine(
    const std::string & json_object,
    std::string * error_message = nullptr) const;

private:
  static std::string timestampNow();
  static std::string escapeJsonString(const std::string & value);
  static void appendJsonStringArray(
    std::string & json,
    const std::vector<std::string> & values);

  bool shouldWriteSummary() const;
  bool ensureParentDirectory(std::string * error_message) const;

  RfidScanLogWriterConfig config_;
};

}  // namespace agv_inventory_system

#endif  // WHEELTEC_INVENTORY_SYSTEM__RFID_SCAN_LOG_WRITER_HPP_
