#include "agv_inventory_system/web_api_client.hpp"

#include <algorithm>
#include <curl/curl.h>
#include <iostream>
#include <sstream>

namespace agv_inventory_system
{
namespace
{

constexpr const char * kDefaultPlcOpenQueryParam = "shelfId";

std::size_t write_response_body(char * ptr, std::size_t size, std::size_t nmemb, void * userdata)
{
  auto * body = static_cast<std::string *>(userdata);
  const std::size_t total = size * nmemb;
  body->append(ptr, total);
  return total;
}

std::string trim_trailing_slashes(std::string value)
{
  while (!value.empty() && value.back() == '/') {
    value.pop_back();
  }
  return value;
}

std::string ensure_leading_slash(std::string value)
{
  if (value.empty()) {
    return "/";
  }
  if (value.front() != '/') {
    value.insert(value.begin(), '/');
  }
  return value;
}

std::string summarize_body(const std::string & body)
{
  constexpr std::size_t kMaxBodySummary = 200;
  std::string summary = body.substr(0, std::min(body.size(), kMaxBodySummary));
  for (char & ch : summary) {
    if (ch == '\r' || ch == '\n' || ch == '\t') {
      ch = ' ';
    }
  }
  if (body.size() > kMaxBodySummary) {
    summary += "...";
  }
  return summary;
}

bool response_body_reports_failure(const std::string & body)
{
  return body.find("\"success\":false") != std::string::npos ||
         body.find("\"success\": false") != std::string::npos ||
         body.find("\"code\":300") != std::string::npos ||
         body.find("\"code\": 300") != std::string::npos ||
         body.find("\"code\":500") != std::string::npos ||
         body.find("\"code\": 500") != std::string::npos;
}

bool response_body_reports_success(const std::string & body)
{
  return body.find("\"success\":true") != std::string::npos ||
         body.find("\"success\": true") != std::string::npos;
}

bool is_http_success(long http_code)
{
  return http_code >= 200 && http_code < 300;
}

bool ensure_curl_global_init()
{
  static const bool initialized = []() {
    const CURLcode code = curl_global_init(CURL_GLOBAL_DEFAULT);
    if (code != CURLE_OK) {
      std::cout << "[web_api_client][Java] ERROR curl_global_init failed code="
                << static_cast<int>(code)
                << " reason=\"" << curl_easy_strerror(code) << "\"" << std::endl;
      return false;
    }
    return true;
  }();
  return initialized;
}

}  // namespace

WebApiClient::WebApiClient(const WebApiClientParams & params)
: params_(params)
{
}

void WebApiClient::setParams(const WebApiClientParams & params)
{
  params_ = params;
}

const WebApiClientParams & WebApiClient::params() const
{
  return params_;
}

bool WebApiClient::requestOpenGap(const std::string & gap_id) const
{
  if (isLocalMode()) {
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
  if (query_param.empty()) {
    query_param = kDefaultPlcOpenQueryParam;
    std::cout << "[web_api_client][PLC] WARNING plc_open_query_param is empty, fallback to "
              << query_param << std::endl;
  }
  const std::string url = base_url + endpoint + "?" + query_param + "=" +
    std::to_string(cabinet_id);

  std::cout << "[web_api_client][PLC] open cabinet request cabinet=" << cabinet_id
            << " query_param=" << query_param
            << " url=" << url
            << " verify_tls=" << (params_.plc_verify_tls ? "true" : "false")
            << " require_body_success=" << (params_.plc_require_body_success ? "true" : "false")
            << " timeout_sec=" << params_.plc_request_timeout_sec
            << " retry_count=" << params_.plc_retry_count << std::endl;
  return requestHttpGet("PLC open cabinet", url);
}

bool WebApiClient::requestCloseGap(const std::string & gap_id) const
{
  if (isLocalMode()) {
    std::cout << "[web_api_client] local close gap accepted: " << gap_id << std::endl;
    return true;
  }

  return warnLegacyWebHttpUnavailable("close gap");
}

bool WebApiClient::reportRobotStatus(const std::string & state) const
{
  if (isLocalMode()) {
    std::cout << "[web_api_client] local status: " << state << std::endl;
    return true;
  }

  return warnLegacyWebHttpUnavailable("report robot status");
}

bool WebApiClient::reportInventoryResult(
  int cabinet_id,
  int layer,
  int depth,
  const std::string & result) const
{
  if (isLocalMode()) {
    std::cout << "[web_api_client] local result cabinet=" << cabinet_id
              << " layer=" << layer
              << " depth=" << depth
              << " result=" << result << std::endl;
    return true;
  }

  return warnLegacyWebHttpUnavailable("report inventory result");
}

bool WebApiClient::requestHttpGet(const std::string & action, const std::string & url) const
{
  if (!ensure_curl_global_init()) {
    return false;
  }

  const int retry_count = std::max(0, params_.plc_retry_count);
  const int total_attempts = retry_count + 1;
  const double timeout_sec = params_.plc_request_timeout_sec > 0.0 ?
    params_.plc_request_timeout_sec : 3.0;

  for (int attempt = 1; attempt <= total_attempts; ++attempt) {
    CURL * curl = curl_easy_init();
    if (curl == nullptr) {
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
    if (result == CURLE_OK && is_http_success(http_code)) {
      if (params_.plc_require_body_success && !response_body_reports_success(response_body)) {
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
      if (!params_.plc_require_body_success && response_body_reports_failure(response_body)) {
        std::cout << "[web_api_client][Java] WARNING " << action
                  << " body reports failure, but plc_require_body_success=false, continue after fixed wait"
                  << " http_code=" << http_code
                  << " body_summary=\"" << body_summary << "\"" << std::endl;
      }
      return true;
    }

    if (result != CURLE_OK) {
      const std::string error_text =
        error_buffer[0] != '\0' ? std::string(error_buffer) : curl_easy_strerror(result);
      std::cout << "[web_api_client][Java] WARNING " << action
                << " transport failed attempt=" << attempt << "/" << total_attempts
                << " curl_code=" << static_cast<int>(result)
                << " reason=\"" << error_text << "\""
                << " http_code=" << http_code
                << " body_summary=\"" << body_summary << "\"" << std::endl;
    } else {
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

bool WebApiClient::isLocalMode() const
{
  return params_.web_client_mode == "local";
}

bool WebApiClient::warnLegacyWebHttpUnavailable(const std::string & action) const
{
  std::cout << "[web_api_client] WARNING legacy web HTTP endpoint is not configured for action="
            << action << " mode=" << params_.web_client_mode
            << " base_url=" << params_.web_base_url << std::endl;
  return false;
}

}  // namespace agv_inventory_system
