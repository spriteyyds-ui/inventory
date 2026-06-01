#ifndef WHEELTEC_INVENTORY_SYSTEM__WEB_API_CLIENT_HPP_
#define WHEELTEC_INVENTORY_SYSTEM__WEB_API_CLIENT_HPP_

#include <string>
#include <vector>

namespace agv_inventory_system
{

struct WebApiClientParams
{
  std::string web_client_mode{"local"};
  std::string web_base_url;
  std::string web_open_gap_endpoint{"/api/gap/open"};
  std::string web_close_gap_endpoint{"/api/gap/close"};
  std::string web_status_endpoint{"/api/robot/status"};
  std::string web_result_endpoint{"/api/inventory/result"};
  std::string plc_server_url{"https://58.154.205.27:8099"};
  std::string plc_open_endpoint{"/http-control-plc/car_open"};
  std::string plc_open_query_param{"shelfId"};
  std::string plc_close_endpoint{"/close"};
  std::string plc_stop_endpoint{"/stop"};
  std::string plc_hello_endpoint{"/hello"};
  bool plc_verify_tls{false};
  bool plc_require_body_success{false};
  double plc_request_timeout_sec{3.0};
  int plc_retry_count{1};
  bool rfid_upload_enabled{true};
  std::string rfid_upload_url{"https://58.154.205.27:8099/RobotInspection/inventoryAudit"};
  bool rfid_upload_verify_tls{false};
  double rfid_upload_timeout_sec{3.0};
  int rfid_upload_retry_count{2};
  std::string rfid_upload_fail_policy{"error"};
  bool rfid_placeholder_enabled{true};
  std::string rfid_placeholder_prefix{"RFID_PLACEHOLDER"};
  bool rfid_upload_require_success{false};
};

struct InventoryUploadItem
{
  std::string location_rfid;
  std::vector<std::string> rfids;
};

class WebApiClient
{
public:
  explicit WebApiClient(const WebApiClientParams & params = WebApiClientParams());

  void setParams(const WebApiClientParams & params);
  const WebApiClientParams & params() const;

  bool requestOpenGap(const std::string & gap_id) const;
  bool requestOpenCabinet(int cabinet_id) const;
  bool requestCloseGap(const std::string & gap_id) const;
  bool reportRobotStatus(const std::string & state) const;
  bool reportInventoryResult(
    int cabinet_id,
    int layer,
    int depth,
    const std::string & result) const;
  bool reportInventoryResult(
    int cabinet_id,
    int layer,
    int depth,
    const std::vector<std::string> & rfids,
    const std::string & result) const;
  bool reportInventoryResults(
    const std::vector<InventoryUploadItem> & items,
    const std::string & result) const;

private:
  bool requestHttpGet(const std::string & action, const std::string & url) const;
  bool requestHttpPostJson(
    const std::string & action,
    const std::string & url,
    const std::string & body) const;
  bool isLocalMode() const;
  bool warnLegacyWebHttpUnavailable(const std::string & action) const;

  WebApiClientParams params_;
};

}  // namespace agv_inventory_system

#endif  // WHEELTEC_INVENTORY_SYSTEM__WEB_API_CLIENT_HPP_
