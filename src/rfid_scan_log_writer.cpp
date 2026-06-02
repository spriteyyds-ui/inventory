#include "agv_inventory_system/rfid_scan_log_writer.hpp"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace agv_inventory_system
{

void RfidScanLogWriter::configure(const RfidScanLogWriterConfig & config)
{
  config_ = config;
}

const RfidScanLogWriterConfig & RfidScanLogWriter::config() const
{
  return config_;
}

bool RfidScanLogWriter::appendScanCellCached(
  const RfidScanCellLogRecord & record,
  std::string * error_message) const
{
  if (!config_.enabled) {
    return true;
  }

  std::string json;
  json.reserve(384U);
  json += "{\"timestamp\":\"";
  json += escapeJsonString(timestampNow());
  json += "\",\"event\":\"scan_cell_cached\"";
  json += ",\"mission_mode\":\"";
  json += escapeJsonString(record.mission_mode);
  json += "\",\"cabinet\":";
  json += std::to_string(record.cabinet);
  json += ",\"layer\":";
  json += std::to_string(record.layer);
  json += ",\"grid\":";
  json += std::to_string(record.grid);
  json += ",\"locationRfid\":\"";
  json += escapeJsonString(record.location_rfid);
  json += "\",\"rfids\":";
  appendJsonStringArray(json, record.rfids);
  json += ",\"reader_mode\":\"";
  json += escapeJsonString(record.reader_mode);
  json += "\",\"source\":\"";
  json += escapeJsonString(record.source);
  json += "\",\"fallback_to_placeholder\":";
  json += record.fallback_to_placeholder ? "true" : "false";
  json += ",\"scan_success\":";
  json += record.scan_success ? "true" : "false";
  json += ",\"batch_item_count\":";
  json += std::to_string(record.batch_item_count);
  json += "}";

  return appendJsonLine(json, error_message);
}

bool RfidScanLogWriter::appendBatchUploadPrepare(
  const std::string & reason,
  std::size_t item_count,
  const std::string & url,
  std::string * error_message) const
{
  if (!shouldWriteSummary()) {
    return true;
  }

  std::string json;
  json.reserve(192U);
  json += "{\"timestamp\":\"";
  json += escapeJsonString(timestampNow());
  json += "\",\"event\":\"final_batch_upload_prepare\"";
  json += ",\"reason\":\"";
  json += escapeJsonString(reason);
  json += "\",\"item_count\":";
  json += std::to_string(item_count);
  json += ",\"url\":\"";
  json += escapeJsonString(url);
  json += "\"}";

  return appendJsonLine(json, error_message);
}

bool RfidScanLogWriter::appendBatchUploadSuccess(
  std::size_t item_count,
  std::string * error_message) const
{
  if (!shouldWriteSummary()) {
    return true;
  }

  std::string json;
  json.reserve(128U);
  json += "{\"timestamp\":\"";
  json += escapeJsonString(timestampNow());
  json += "\",\"event\":\"final_batch_upload_success\"";
  json += ",\"item_count\":";
  json += std::to_string(item_count);
  json += "}";

  return appendJsonLine(json, error_message);
}

bool RfidScanLogWriter::appendBatchUploadFailed(
  std::size_t item_count,
  const std::string & fail_policy,
  std::string * error_message) const
{
  if (!shouldWriteSummary()) {
    return true;
  }

  std::string json;
  json.reserve(160U);
  json += "{\"timestamp\":\"";
  json += escapeJsonString(timestampNow());
  json += "\",\"event\":\"final_batch_upload_failed\"";
  json += ",\"item_count\":";
  json += std::to_string(item_count);
  json += ",\"fail_policy\":\"";
  json += escapeJsonString(fail_policy);
  json += "\"}";

  return appendJsonLine(json, error_message);
}

bool RfidScanLogWriter::appendBatchUploadSkipped(
  const std::string & reason,
  std::size_t item_count,
  std::string * error_message) const
{
  if (!shouldWriteSummary()) {
    return true;
  }

  std::string json;
  json.reserve(160U);
  json += "{\"timestamp\":\"";
  json += escapeJsonString(timestampNow());
  json += "\",\"event\":\"final_batch_upload_skipped\"";
  json += ",\"reason\":\"";
  json += escapeJsonString(reason);
  json += "\",\"item_count\":";
  json += std::to_string(item_count);
  json += "}";

  return appendJsonLine(json, error_message);
}

bool RfidScanLogWriter::appendJsonLine(
  const std::string & json_object,
  std::string * error_message) const
{
  if (!config_.enabled) {
    return true;
  }
  if (config_.path.empty()) {
    if (error_message) {
      *error_message = "rfid local log path is empty";
    }
    return false;
  }
  if (!ensureParentDirectory(error_message)) {
    return false;
  }

  std::ofstream output(config_.path, std::ios::out | std::ios::app);
  if (!output) {
    if (error_message) {
      *error_message = "open failed path=" + config_.path;
    }
    return false;
  }

  output << json_object << '\n';
  if (!output) {
    if (error_message) {
      *error_message = "write failed path=" + config_.path;
    }
    return false;
  }
  return true;
}

std::string RfidScanLogWriter::timestampNow()
{
  const auto now = std::chrono::system_clock::now();
  const auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
  const auto millis =
    std::chrono::duration_cast<std::chrono::milliseconds>(now - seconds).count();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);

  std::tm tm{};
#if defined(_WIN32)
  gmtime_s(&tm, &time);
#else
  gmtime_r(&time, &tm);
#endif

  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S")
      << '.' << std::setw(3) << std::setfill('0') << millis << 'Z';
  return oss.str();
}

std::string RfidScanLogWriter::escapeJsonString(const std::string & value)
{
  std::ostringstream oss;
  for (const unsigned char ch : value) {
    switch (ch) {
      case '"':
        oss << "\\\"";
        break;
      case '\\':
        oss << "\\\\";
        break;
      case '\b':
        oss << "\\b";
        break;
      case '\f':
        oss << "\\f";
        break;
      case '\n':
        oss << "\\n";
        break;
      case '\r':
        oss << "\\r";
        break;
      case '\t':
        oss << "\\t";
        break;
      default:
        if (ch < 0x20U) {
          oss << "\\u"
              << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(ch)
              << std::dec << std::setfill(' ');
        } else {
          oss << static_cast<char>(ch);
        }
        break;
    }
  }
  return oss.str();
}

void RfidScanLogWriter::appendJsonStringArray(
  std::string & json,
  const std::vector<std::string> & values)
{
  json += "[";
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0U) {
      json += ",";
    }
    json += "\"";
    json += escapeJsonString(values[i]);
    json += "\"";
  }
  json += "]";
}

bool RfidScanLogWriter::shouldWriteSummary() const
{
  return config_.enabled && config_.write_batch_summary;
}

bool RfidScanLogWriter::ensureParentDirectory(std::string * error_message) const
{
  std::error_code ec;
  const std::filesystem::path log_path(config_.path);
  const auto parent = log_path.parent_path();
  if (parent.empty() || std::filesystem::exists(parent, ec)) {
    if (ec) {
      if (error_message) {
        *error_message = "check parent directory failed path=" + parent.string() +
          " error=" + ec.message();
      }
      return false;
    }
    return true;
  }

  if (!std::filesystem::create_directories(parent, ec) && ec) {
    if (error_message) {
      *error_message = "create parent directory failed path=" + parent.string() +
        " error=" + ec.message();
    }
    return false;
  }
  return true;
}

}  // namespace agv_inventory_system
