#ifndef WHEELTEC_INVENTORY_SYSTEM__WEB_API_CLIENT_HPP_
#define WHEELTEC_INVENTORY_SYSTEM__WEB_API_CLIENT_HPP_

#include <string>

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
};

class WebApiClient
{
public:
  explicit WebApiClient(const WebApiClientParams & params = WebApiClientParams());

  void setParams(const WebApiClientParams & params);
  const WebApiClientParams & params() const;

  bool requestOpenGap(const std::string & gap_id) const;
  bool requestCloseGap(const std::string & gap_id) const;
  bool reportRobotStatus(const std::string & state) const;
  bool reportInventoryResult(
    int cabinet_id,
    int layer,
    int depth,
    const std::string & result) const;

private:
  bool isLocalMode() const;
  bool warnRealHttpNotImplemented(const std::string & action) const;

  WebApiClientParams params_;
};

}  // namespace agv_inventory_system

#endif  // WHEELTEC_INVENTORY_SYSTEM__WEB_API_CLIENT_HPP_
