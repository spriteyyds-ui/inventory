#include "agv_inventory_system/web_api_client.hpp"

#include <iostream>

namespace agv_inventory_system
{

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

  return warnRealHttpNotImplemented("open gap");
}

bool WebApiClient::requestCloseGap(const std::string & gap_id) const
{
  if (isLocalMode()) {
    std::cout << "[web_api_client] local close gap accepted: " << gap_id << std::endl;
    return true;
  }

  return warnRealHttpNotImplemented("close gap");
}

bool WebApiClient::reportRobotStatus(const std::string & state) const
{
  if (isLocalMode()) {
    std::cout << "[web_api_client] local status: " << state << std::endl;
    return true;
  }

  return warnRealHttpNotImplemented("report robot status");
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

  return warnRealHttpNotImplemented("report inventory result");
}

bool WebApiClient::isLocalMode() const
{
  return params_.web_client_mode == "local";
}

bool WebApiClient::warnRealHttpNotImplemented(const std::string & action) const
{
  std::cout << "[web_api_client] WARNING real HTTP mode not implemented yet action="
            << action << " mode=" << params_.web_client_mode << std::endl;
  return false;
}

}  // namespace agv_inventory_system
