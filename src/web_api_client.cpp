#include "agv_inventory_system/web_api_client.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <curl/curl.h>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

namespace agv_inventory_system
{
  namespace
  {

    constexpr const char *kDefaultPlcOpenQueryParam = "shelfId";

    std::size_t write_response_body(char *ptr, std::size_t size, std::size_t nmemb, void *userdata)
    {
      auto *body = static_cast<std::string *>(userdata);
      const std::size_t total = size * nmemb;
      body->append(ptr, total);
      return total;
    }

    std::string trim_trailing_slashes(std::string value)
    {
      while (!value.empty() && value.back() == '/')
      {
        value.pop_back();
      }
      return value;
    }

    std::string ensure_leading_slash(std::string value)
    {
      if (value.empty())
      {
        return "/";
      }
      if (value.front() != '/')
      {
        value.insert(value.begin(), '/');
      }
      return value;
    }

    std::string summarize_body(const std::string &body)
    {
      constexpr std::size_t kMaxBodySummary = 200;
      std::string summary = body.substr(0, std::min(body.size(), kMaxBodySummary));
      for (char &ch : summary)
      {
        if (ch == '\r' || ch == '\n' || ch == '\t')
        {
          ch = ' ';
        }
      }
      if (body.size() > kMaxBodySummary)
      {
        summary += "...";
      }
      return summary;
    }

    bool response_body_reports_failure(const std::string &body)
    {
      return body.find("\"success\":false") != std::string::npos ||
             body.find("\"success\": false") != std::string::npos ||
             body.find("\"code\":300") != std::string::npos ||
             body.find("\"code\": 300") != std::string::npos ||
             body.find("\"code\":500") != std::string::npos ||
             body.find("\"code\": 500") != std::string::npos;
    }

    bool response_body_reports_success(const std::string &body)
    {
      return body.find("\"success\":true") != std::string::npos ||
             body.find("\"success\": true") != std::string::npos;
    }

    bool is_http_ok(long http_code)
    {
      return http_code == 200;
    }

    std::string timestamp_now_iso()
    {
      const auto now = std::chrono::system_clock::now();
      const std::time_t time = std::chrono::system_clock::to_time_t(now);

      std::tm tm{};
#if defined(_WIN32)
      gmtime_s(&tm, &time);
#else
      gmtime_r(&time, &tm);
#endif

      std::ostringstream oss;
      oss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
      return oss.str();
    }

    std::string extract_json_string_field(const std::string & body, const std::string & field)
    {
      const std::string key = "\"" + field + "\"";
      const std::size_t key_pos = body.find(key);
      if (key_pos == std::string::npos)
      {
        return "";
      }
      const std::size_t colon_pos = body.find(':', key_pos + key.size());
      if (colon_pos == std::string::npos)
      {
        return "";
      }
      std::size_t value_start = colon_pos + 1U;
      while (value_start < body.size() &&
        std::isspace(static_cast<unsigned char>(body[value_start])) != 0)
      {
        ++value_start;
      }
      if (value_start >= body.size() || body[value_start] != '"')
      {
        return "";
      }
      ++value_start;
      std::string value;
      bool escaped = false;
      for (std::size_t i = value_start; i < body.size(); ++i)
      {
        const char ch = body[i];
        if (escaped)
        {
          value.push_back(ch);
          escaped = false;
          continue;
        }
        if (ch == '\\')
        {
          escaped = true;
          continue;
        }
        if (ch == '"')
        {
          break;
        }
        value.push_back(ch);
      }
      return value;
    }

    bool is_likely_json_object(const std::string & body)
    {
      for (const char ch : body)
      {
        if (std::isspace(static_cast<unsigned char>(ch)) != 0)
        {
          continue;
        }
        return ch == '{';
      }
      return false;
    }

    std::string status_message_from_response(const std::string & body, bool success)
    {
      const std::string message = extract_json_string_field(body, "message");
      if (!message.empty())
      {
        return message;
      }
      const std::string msg = extract_json_string_field(body, "msg");
      if (!msg.empty())
      {
        return msg;
      }
      if (success)
      {
        return "success=true";
      }
      return summarize_body(body);
    }

    std::string normalize_fail_policy(std::string value)
    {
      std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
      });
      if (value == "continue_without_upload")
      {
        return value;
      }
      return "error";
    }

    bool continue_without_upload(const WebApiClientParams &params)
    {
      return normalize_fail_policy(params.rfid_upload_fail_policy) == "continue_without_upload";
    }

    std::string build_location_rfid(int cabinet_id, int layer, int depth)
    {
      return "shelf_" + std::to_string(cabinet_id) + "_" + std::to_string(layer) + "_" +
             std::to_string(depth);
    }

    std::string escape_json_string(const std::string &value);

    std::string build_inventory_results_body(
        const std::vector<InventoryUploadItem> &items)
    {
      std::ostringstream body;
      body << "{\"scanCells\":[";
      for (std::size_t item_index = 0; item_index < items.size(); ++item_index)
      {
        if (item_index > 0)
        {
          body << ",";
        }
        const auto & item = items[item_index];
        body << "{\"locationRfid\":\"" << escape_json_string(item.location_rfid)
             << "\",\"rfids\":[";
        for (std::size_t i = 0; i < item.rfids.size(); ++i)
        {
          if (i > 0)
          {
            body << ",";
          }
          body << "\"" << escape_json_string(item.rfids[i]) << "\"";
        }
        body << "]}";
      }
      body << "]}";
      return body.str();
    }

    std::string join_rfids_for_log(const std::vector<std::string> &rfids)
    {
      std::ostringstream oss;
      for (std::size_t i = 0; i < rfids.size(); ++i)
      {
        if (i > 0)
        {
          oss << ",";
        }
        oss << rfids[i];
      }
      return oss.str();
    }

    std::string escape_json_string(const std::string &value)
    {
      std::ostringstream oss;
      for (const char ch : value)
      {
        switch (ch)
        {
        case '\\':
          oss << "\\\\";
          break;
        case '"':
          oss << "\\\"";
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
          if (static_cast<unsigned char>(ch) < 0x20)
          {
            oss << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                << static_cast<int>(static_cast<unsigned char>(ch));
          }
          else
          {
            oss << ch;
          }
          break;
        }
      }
      return oss.str();
    }

    bool ensure_curl_global_init()
    {
      static const bool initialized = []()
      {
        const CURLcode code = curl_global_init(CURL_GLOBAL_DEFAULT);
        if (code != CURLE_OK)
        {
          std::cout << "[web_api_client][Java] ERROR curl_global_init failed code="
                    << static_cast<int>(code)
                    << " reason=\"" << curl_easy_strerror(code) << "\"" << std::endl;
          return false;
        }
        return true;
      }();
      return initialized;
    }

  } // namespace

  WebApiClient::WebApiClient(const WebApiClientParams &params)
      : params_(params)
  {
  }

  void WebApiClient::setParams(const WebApiClientParams &params)
  {
    params_ = params;
  }

  const WebApiClientParams &WebApiClient::params() const
  {
    return params_;
  }

  bool WebApiClient::requestOpenGap(const std::string &gap_id) const
  {
    if (isLocalMode())
    {
      std::cout << "[web_api_client] local open gap accepted: " << gap_id << std::endl;
      return true;
    }

    return warnLegacyWebHttpUnavailable("open gap");
  }

  bool WebApiClient::requestOpenCabinet(int cabinet_id) const
  {
    const std::string base_url = trim_trailing_slashes(params_.plc_server_url);
    const std::string endpoint = ensure_leading_slash(params_.plc_open_endpoint);
    std::string query_param = params_.plc_open_query_param;
    if (query_param.empty())
    {
      query_param = kDefaultPlcOpenQueryParam;
      std::cout << "[web_api_client][PLC] WARNING plc_open_query_param is empty, fallback to "
                << query_param << std::endl;
    }

    // Map cabinet_id to PLC shelf_id: cabinet 19-36 -> shelfId 25-42
    int plc_shelf_id = cabinet_id;
    if (cabinet_id >= 19 && cabinet_id <= 36) {
      plc_shelf_id = cabinet_id + 6;
    }

    const std::string url = base_url + endpoint + "?" + query_param + "=" +
                            std::to_string(plc_shelf_id);

    std::cout << "[web_api_client][PLC] open cabinet request cabinet=" << cabinet_id
              << " mapped_shelf_id=" << plc_shelf_id
              << " query_param=" << query_param
              << " url=" << url
              << " verify_tls=" << (params_.plc_verify_tls ? "true" : "false")
              << " require_body_success=" << (params_.plc_require_body_success ? "true" : "false")
              << " timeout_sec=" << params_.plc_request_timeout_sec
              << " retry_count=" << params_.plc_retry_count << std::endl;
    return requestHttpGet("PLC open cabinet", url);
  }

  bool WebApiClient::requestPlcClose() const
  {
    const std::string base_url = trim_trailing_slashes(params_.plc_server_url);
    const std::string endpoint = ensure_leading_slash(params_.plc_close_endpoint);
    const std::string url = base_url + endpoint;

    std::cout << "[web_api_client][PLC] close all request"
              << " url=" << url
              << " verify_tls=" << (params_.plc_verify_tls ? "true" : "false")
              << " timeout_sec=" << params_.plc_request_timeout_sec
              << " retry_count=" << params_.plc_retry_count << std::endl;
    return requestHttpGet("PLC close all", url);
  }

  bool WebApiClient::requestCloseGap(const std::string &gap_id) const
  {
    if (isLocalMode())
    {
      std::cout << "[web_api_client] local close gap accepted: " << gap_id << std::endl;
      return true;
    }

    return warnLegacyWebHttpUnavailable("close gap");
  }

  bool WebApiClient::reportRobotStatus(const std::string &state) const
  {
    if (isLocalMode())
    {
      std::cout << "[web_api_client] local status: " << state << std::endl;
      return true;
    }

    return warnLegacyWebHttpUnavailable("report robot status");
  }

  bool WebApiClient::reportInventoryResult(
      int cabinet_id,
      int layer,
      int depth,
      const std::string &result) const
  {
    return reportInventoryResult(cabinet_id, layer, depth, std::vector<std::string>{}, result);
  }

  bool WebApiClient::reportInventoryResult(
      int cabinet_id,
      int layer,
      int depth,
      const std::vector<std::string> &rfids,
      const std::string &result) const
  {
    const int safe_layer = layer > 0 ? layer : 1;
    const int safe_depth = depth > 0 ? depth : 1;
    const std::string location_rfid = build_location_rfid(cabinet_id, safe_layer, safe_depth);

    if (!params_.rfid_upload_enabled)
    {
      std::cout << "[web_api_client][RFID] upload disabled, skip real reader result"
                << " cabinet=" << cabinet_id
                << " level=" << safe_layer
                << " grid=" << safe_depth
                << " locationRfid=" << location_rfid
                << " rfid_count=" << rfids.size()
                << " result_summary=\"" << summarize_body(result) << "\"" << std::endl;
      return true;
    }

    if (params_.rfid_upload_url.empty())
    {
      std::cout << "[web_api_client][RFID] ERROR rfid_upload_url is empty"
                << " cabinet=" << cabinet_id
                << " level=" << safe_layer
                << " grid=" << safe_depth
                << " locationRfid=" << location_rfid
                << " rfid_count=" << rfids.size() << std::endl;
      if (continue_without_upload(params_))
      {
        std::cout << "[web_api_client][RFID] WARNING continue_without_upload after empty upload URL"
                  << " locationRfid=" << location_rfid << std::endl;
        return true;
      }
      return false;
    }

    std::cout << "[web_api_client][RFID] upload result source=active_report_serial"
              << " cabinet=" << cabinet_id
              << " level=" << safe_layer
              << " grid=" << safe_depth
              << " locationRfid=" << location_rfid
              << " rfid_count=" << rfids.size()
              << " rfids=\"" << join_rfids_for_log(rfids) << "\""
              << " result_summary=\"" << summarize_body(result) << "\""
              << " url=" << params_.rfid_upload_url << std::endl;

    InventoryUploadItem item;
    item.location_rfid = location_rfid;
    item.rfids = rfids;
    const bool uploaded = reportInventoryResults(std::vector<InventoryUploadItem>{item}, result);
    if (uploaded)
    {
      return true;
    }

    if (continue_without_upload(params_))
    {
      std::cout << "[web_api_client][RFID] WARNING upload failed but continue_without_upload is set"
                << " cabinet=" << cabinet_id
                << " level=" << safe_layer
                << " grid=" << safe_depth
                << " locationRfid=" << location_rfid << std::endl;
      return true;
    }

    return false;
  }

  bool WebApiClient::reportInventoryResults(
      const std::vector<InventoryUploadItem> &items,
      const std::string &result) const
  {
    const UploadStatus status = reportInventoryResultsWithStatus(items, result);
    return status.success;
  }

  UploadStatus WebApiClient::reportInventoryResultsWithStatus(
      const std::vector<InventoryUploadItem> &items,
      const std::string &result) const
  {
    if (!params_.rfid_upload_enabled)
    {
      std::cout << "[web_api_client][RFID] batch upload disabled, skip result"
                << " item_count=" << items.size()
                << " result_summary=\"" << summarize_body(result) << "\"" << std::endl;
      return UploadStatus{true, false, 0, "上传已跳过: rfid_upload_enabled=false"};
    }

    if (params_.rfid_upload_url.empty())
    {
      std::cout << "[web_api_client][RFID] ERROR rfid_upload_url is empty"
                << " item_count=" << items.size() << std::endl;
      return UploadStatus{false, false, 0, "rfid_upload_url is empty"};
    }

    const std::string body = build_inventory_results_body(items);
    std::cout << "[web_api_client][RFID] batch upload result"
              << " item_count=" << items.size()
              << " result_summary=\"" << summarize_body(result) << "\""
              << " url=" << params_.rfid_upload_url << std::endl;

    return requestHttpPostJsonWithStatus("RFID inventory result batch", params_.rfid_upload_url, body);
  }

  bool WebApiClient::requestHttpGet(const std::string &action, const std::string &url) const
  {
    if (!ensure_curl_global_init())
    {
      return false;
    }

    const int retry_count = std::max(0, params_.plc_retry_count);
    const int total_attempts = retry_count + 1;
    const double timeout_sec = params_.plc_request_timeout_sec > 0.0 ? params_.plc_request_timeout_sec : 10.0;

    for (int attempt = 1; attempt <= total_attempts; ++attempt)
    {
      CURL *curl = curl_easy_init();
      if (curl == nullptr)
      {
        std::cout << "[web_api_client][Java] ERROR " << action
                  << " curl_easy_init failed attempt=" << attempt
                  << "/" << total_attempts << std::endl;
        continue;
      }

      std::string response_body;
      long http_code = 0;
      char error_buffer[CURL_ERROR_SIZE] = {0};

      curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
      curl_easy_setopt(curl, CURLOPT_HTTPGET, 1L);
      curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 0L);
      curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, static_cast<long>(timeout_sec * 1000.0));
      curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, static_cast<long>(timeout_sec * 1000.0));
      curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_response_body);
      curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_body);
      curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, error_buffer);
      curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, params_.plc_verify_tls ? 1L : 0L);
      curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, params_.plc_verify_tls ? 2L : 0L);

      std::cout << "[web_api_client][Java] " << action
                << " attempt=" << attempt << "/" << total_attempts
                << " method=GET url=" << url
                << " timeout_sec=" << timeout_sec << std::endl;

      const CURLcode result = curl_easy_perform(curl);
      curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
      curl_easy_cleanup(curl);

      const std::string body_summary = summarize_body(response_body);
      if (result == CURLE_OK && is_http_ok(http_code))
      {
        if (params_.plc_require_body_success && !response_body_reports_success(response_body))
        {
          std::cout << "[web_api_client][Java] WARNING " << action
                    << " body success required but success=true was not found"
                    << " attempt=" << attempt << "/" << total_attempts
                    << " http_code=" << http_code
                    << " body_summary=\"" << body_summary << "\"" << std::endl;
          continue;
        }

        std::cout << "[web_api_client][Java] " << action
                  << " success http_code=" << http_code
                  << " body_summary=\"" << body_summary << "\"" << std::endl;
        if (!params_.plc_require_body_success && response_body_reports_failure(response_body))
        {
          std::cout << "[web_api_client][Java] WARNING " << action
                    << " body reports failure, but plc_require_body_success=false, continue after fixed wait"
                    << " http_code=" << http_code
                    << " body_summary=\"" << body_summary << "\"" << std::endl;
        }
        return true;
      }

      if (result != CURLE_OK)
      {
        const std::string error_text =
            error_buffer[0] != '\0' ? std::string(error_buffer) : curl_easy_strerror(result);
        std::cout << "[web_api_client][Java] WARNING " << action
                  << " transport failed attempt=" << attempt << "/" << total_attempts
                  << " curl_code=" << static_cast<int>(result)
                  << " reason=\"" << error_text << "\""
                  << " http_code=" << http_code
                  << " body_summary=\"" << body_summary << "\"" << std::endl;
      }
      else
      {
        std::cout << "[web_api_client][Java] WARNING " << action
                  << " unsuccessful response attempt=" << attempt << "/" << total_attempts
                  << " http_code=" << http_code
                  << " body_summary=\"" << body_summary << "\"" << std::endl;
      }
    }

    std::cout << "[web_api_client][Java] ERROR " << action
              << " failed after attempts=" << total_attempts << std::endl;
    return false;
  }

  bool WebApiClient::requestHttpPostJson(
      const std::string &action,
      const std::string &url,
      const std::string &body) const
  {
    const UploadStatus status = requestHttpPostJsonWithStatus(action, url, body);
    return status.success;
  }

  UploadStatus WebApiClient::requestHttpPostJsonWithStatus(
      const std::string &action,
      const std::string &url,
      const std::string &body) const
  {
    if (!ensure_curl_global_init())
    {
      return UploadStatus{false, false, 0, "curl_global_init failed"};
    }

    const int retry_count = std::max(0, params_.rfid_upload_retry_count);
    const int total_attempts = retry_count + 1;
    const double timeout_sec =
        params_.rfid_upload_timeout_sec > 0.0 ? params_.rfid_upload_timeout_sec : 3.0;
    const std::string request_summary = summarize_body(body);
    UploadStatus last_status{false, false, 0, "未发送上传请求"};

    for (int attempt = 1; attempt <= total_attempts; ++attempt)
    {
      CURL *curl = curl_easy_init();
      if (curl == nullptr)
      {
        std::cout << "[web_api_client][RFID] ERROR " << action
                  << " curl_easy_init failed attempt=" << attempt
                  << "/" << total_attempts
                  << " request_summary=\"" << request_summary << "\"" << std::endl;
        last_status = UploadStatus{false, false, 0, "curl_easy_init failed"};
        continue;
      }

      std::string response_body;
      long http_code = 0;
      char error_buffer[CURL_ERROR_SIZE] = {0};
      struct curl_slist *headers = nullptr;
      headers = curl_slist_append(headers, "Content-Type: application/json");
      headers = curl_slist_append(headers, "Accept: application/json");

      curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
      curl_easy_setopt(curl, CURLOPT_POST, 1L);
      curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
      curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
      curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, static_cast<long>(body.size()));
      curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 0L);
      curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT_MS, static_cast<long>(timeout_sec * 1000.0));
      curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, static_cast<long>(timeout_sec * 1000.0));
      curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_response_body);
      curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_body);
      curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, error_buffer);
      curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, params_.rfid_upload_verify_tls ? 1L : 0L);
      curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, params_.rfid_upload_verify_tls ? 2L : 0L);

      std::cout << "[web_api_client][RFID] " << action
                << " attempt=" << attempt << "/" << total_attempts
                << " method=POST url=" << url
                << " timeout_sec=" << timeout_sec
                << " verify_tls=" << (params_.rfid_upload_verify_tls ? "true" : "false")
                << " request_summary=\"" << request_summary << "\"" << std::endl;

      const CURLcode result = curl_easy_perform(curl);
      curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
      curl_easy_cleanup(curl);
      curl_slist_free_all(headers);

      const std::string body_summary = summarize_body(response_body);
      if (result == CURLE_OK && is_http_ok(http_code))
      {
        const bool body_reports_failure = response_body_reports_failure(response_body);
        const bool body_reports_success = response_body_reports_success(response_body);
        const bool likely_json = is_likely_json_object(response_body);
        const bool display_success = !body_reports_failure && body_reports_success;
        const std::string status_message =
          status_message_from_response(response_body, display_success);
        last_status = UploadStatus{display_success, display_success, http_code, status_message};

        if (!likely_json)
        {
          last_status = UploadStatus{
            !params_.rfid_upload_require_success,
            false,
            http_code,
            "响应解析失败: response is not JSON"};
          std::cout << "[web_api_client][RFID] WARNING " << action
                    << " response body is not JSON"
                    << " attempt=" << attempt << "/" << total_attempts
                    << " http_code=" << http_code
                    << " body_summary=\"" << body_summary << "\"" << std::endl;
          if (params_.rfid_upload_require_success)
          {
            continue;
          }
          return last_status;
        }

        if (params_.rfid_upload_require_success && !response_body_reports_success(response_body))
        {
          last_status.success = false;
          last_status.display_success = false;
          std::cout << "[web_api_client][RFID] WARNING " << action
                    << " body success required but success=true was not found"
                    << " attempt=" << attempt << "/" << total_attempts
                    << " http_code=" << http_code
                    << " request_summary=\"" << request_summary << "\""
                    << " body_summary=\"" << body_summary << "\"" << std::endl;
          const std::string message = extract_json_string_field(response_body, "message");
          if (!message.empty())
          {
            std::cout << "[web_api_client][RFID] WARNING " << action
                      << " response message=\"" << message << "\"" << std::endl;
          }
          continue;
        }

        std::cout << "[web_api_client][RFID] " << action
                  << " success http_code=" << http_code
                  << " request_summary=\"" << request_summary << "\""
                  << " body_summary=\"" << body_summary << "\"" << std::endl;
        if (response_body_reports_failure(response_body))
        {
          const std::string message = extract_json_string_field(response_body, "message");
          std::cout << "[web_api_client][RFID] WARNING " << action
                    << " body reports success=false"
                    << " message=\"" << message << "\""
                    << " body_summary=\"" << body_summary << "\"" << std::endl;
        }
        if (body_reports_failure)
        {
          last_status.success = !params_.rfid_upload_require_success;
          last_status.display_success = false;
        }
        return last_status;
      }

      if (result != CURLE_OK)
      {
        const std::string error_text =
            error_buffer[0] != '\0' ? std::string(error_buffer) : curl_easy_strerror(result);
        last_status = UploadStatus{false, false, http_code, error_text};
        std::cout << "[web_api_client][RFID] WARNING " << action
                  << " transport failed attempt=" << attempt << "/" << total_attempts
                  << " curl_code=" << static_cast<int>(result)
                  << " reason=\"" << error_text << "\""
                  << " http_code=" << http_code
                  << " request_summary=\"" << request_summary << "\""
                  << " body_summary=\"" << body_summary << "\"" << std::endl;
      }
      else
      {
        last_status = UploadStatus{
          false,
          false,
          http_code,
          "HTTP 非 200: " + std::to_string(http_code)};
        std::cout << "[web_api_client][RFID] WARNING " << action
                  << " unsuccessful response attempt=" << attempt << "/" << total_attempts
                  << " http_code=" << http_code
                  << " request_summary=\"" << request_summary << "\""
                  << " body_summary=\"" << body_summary << "\"" << std::endl;
      }
    }

    std::cout << "[web_api_client][RFID] ERROR " << action
              << " failed after attempts=" << total_attempts
              << " request_summary=\"" << request_summary << "\"" << std::endl;
    return last_status;
  }

  void WebApiClient::writeRfidUploadStatus(
      bool success,
      long status_code,
      const std::string &message,
      const std::string &source) const
  {
    if (params_.rfid_upload_status_path.empty())
    {
      return;
    }

    try
    {
      const std::filesystem::path status_path(params_.rfid_upload_status_path);
      const auto parent = status_path.parent_path();
      if (!parent.empty())
      {
        std::filesystem::create_directories(parent);
      }

      const std::filesystem::path temp_path =
          status_path.string() + ".tmp";
      std::ofstream output(temp_path, std::ios::out | std::ios::trunc);
      if (!output)
      {
        std::cout << "[web_api_client][RFID] WARNING upload status write open failed path="
                  << temp_path.string() << std::endl;
        return;
      }

      output << "{"
             << "\"last_upload_success\":" << (success ? "true" : "false")
             << ",\"last_upload_time\":\"" << escape_json_string(timestamp_now_iso()) << "\""
             << ",\"last_upload_message\":\"" << escape_json_string(message) << "\""
             << ",\"last_upload_status_code\":";
      if (status_code > 0)
      {
        output << status_code;
      }
      else
      {
        output << "null";
      }
      output << ",\"last_upload_source\":\"" << escape_json_string(source) << "\""
             << "}\n";
      output.close();
      if (!output)
      {
        std::cout << "[web_api_client][RFID] WARNING upload status write failed path="
                  << temp_path.string() << std::endl;
        return;
      }

      std::filesystem::rename(temp_path, status_path);
    }
    catch (const std::exception &exc)
    {
      std::cout << "[web_api_client][RFID] WARNING upload status write exception path="
                << params_.rfid_upload_status_path
                << " error=\"" << exc.what() << "\"" << std::endl;
    }
  }

  bool WebApiClient::isLocalMode() const
  {
    return params_.web_client_mode == "local";
  }

  bool WebApiClient::warnLegacyWebHttpUnavailable(const std::string &action) const
  {
    std::cout << "[web_api_client] WARNING legacy web HTTP endpoint is not configured for action="
              << action << " mode=" << params_.web_client_mode
              << " base_url=" << params_.web_base_url << std::endl;
    return false;
  }

} // namespace agv_inventory_system
