#include "wheeltec_inventory_system/web_api_client.hpp"

#include <iostream>

namespace wheeltec_inventory_system
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
  if (isMockMode()) {
    std::cout << "[web_api_client] MOCK open gap: " << gap_id << std::endl;
    return true;
  }

  return warnRealHttpNotImplemented("open gap");
}

bool WebApiClient::requestCloseGap(const std::string & gap_id) const
{
  if (isMockMode()) {
    std::cout << "[web_api_client] MOCK close gap: " << gap_id << std::endl;
    return true;
  }

  return warnRealHttpNotImplemented("close gap");
}

bool WebApiClient::reportRobotStatus(const std::string & state) const
{
  if (isMockMode()) {
    std::cout << "[web_api_client] MOCK status: " << state << std::endl;
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
  if (isMockMode()) {
    std::cout << "[web_api_client] MOCK result cabinet=" << cabinet_id
              << " layer=" << layer
              << " depth=" << depth
              << " result=" << result << std::endl;
    return true;
  }

  return warnRealHttpNotImplemented("report inventory result");
}

bool WebApiClient::isMockMode() const
{
  return params_.web_client_mode == "mock";
}

bool WebApiClient::warnRealHttpNotImplemented(const std::string & action) const
{
  std::cout << "[web_api_client] WARNING real HTTP mode not implemented yet action="
            << action << " mode=" << params_.web_client_mode << std::endl;
  return false;
}

}  // namespace wheeltec_inventory_system
