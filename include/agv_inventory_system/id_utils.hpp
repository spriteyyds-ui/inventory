// Copyright (c) 2026 郁有冬 <spriteyyds@gmail.com>. All rights reserved.
#ifndef AGV_INVENTORY_SYSTEM__ID_UTILS_HPP_
#define AGV_INVENTORY_SYSTEM__ID_UTILS_HPP_

#include <algorithm>
#include <cctype>
#include <sstream>
#include <string>
#include <vector>

namespace agv_inventory_system
{

inline std::vector<std::string> split(const std::string & input, char delim)
{
  std::vector<std::string> out;
  std::stringstream ss(input);
  std::string item;
  while (std::getline(ss, item, delim)) {
    out.push_back(item);
  }
  return out;
}

inline std::string trim(const std::string & input)
{
  const auto first = std::find_if_not(input.begin(), input.end(), [](unsigned char c) {
    return std::isspace(c) != 0;
  });

  if (first == input.end()) {
    return "";
  }

  const auto last = std::find_if_not(input.rbegin(), input.rend(), [](unsigned char c) {
    return std::isspace(c) != 0;
  }).base();

  return std::string(first, last);
}

inline std::string digits_only(const std::string & input)
{
  std::string out;
  out.reserve(input.size());
  for (const auto ch : input) {
    if (std::isdigit(static_cast<unsigned char>(ch)) != 0) {
      out.push_back(ch);
    }
  }
  return out;
}

inline bool safe_to_int(const std::string & input, int & value)
{
  try {
    const auto cleaned = trim(input);
    if (cleaned.empty()) {
      return false;
    }
    std::size_t idx = 0;
    const int parsed = std::stoi(cleaned, &idx);
    if (idx != cleaned.size()) {
      return false;
    }
    value = parsed;
    return true;
  } catch (...) {
    return false;
  }
}

inline bool parse_target_code(
  const std::string & target_code,
  int & warehouse_id,
  int & cabinet_id,
  int & row_id,
  int & layer_id)
{
  const auto items = split(trim(target_code), '-');
  if (items.size() != 4) {
    return false;
  }

  return safe_to_int(items[0], warehouse_id) &&
         safe_to_int(items[1], cabinet_id) &&
         safe_to_int(items[2], row_id) &&
         safe_to_int(items[3], layer_id);
}

inline bool extract_cabinet_id_from_code(const std::string & code, int & cabinet_id)
{
  int warehouse_id = 0;
  int row_id = 0;
  int layer_id = 0;
  if (parse_target_code(code, warehouse_id, cabinet_id, row_id, layer_id)) {
    (void)warehouse_id;
    (void)row_id;
    (void)layer_id;
    return true;
  }

  // 兼容直接输入货柜号（如 "16"）
  return safe_to_int(code, cabinet_id);
}

inline std::string normalize_cabinet_text(const std::string & raw_text)
{
  auto digits = digits_only(raw_text);
  if (digits.empty()) {
    return "";
  }

  // 去掉前导零，避免 "016" 与 "16" 比较失败
  const auto non_zero_pos = digits.find_first_not_of('0');
  if (non_zero_pos == std::string::npos) {
    return "0";
  }
  return digits.substr(non_zero_pos);
}

inline bool validate_target_code(
  const std::string & target_code,
  int cabinet_min,
  int cabinet_max,
  int row_min,
  int row_max,
  int layer_min,
  int layer_max,
  std::string * reason = nullptr)
{
  int warehouse_id = 0;
  int cabinet_id = 0;
  int row_id = 0;
  int layer_id = 0;

  if (!parse_target_code(target_code, warehouse_id, cabinet_id, row_id, layer_id)) {
    if (reason != nullptr) {
      *reason = "目标编号格式错误，应为 仓库号-货柜号-排号-层号";
    }
    return false;
  }

  if (warehouse_id <= 0) {
    if (reason != nullptr) {
      *reason = "仓库号必须大于 0";
    }
    return false;
  }

  if (cabinet_id < cabinet_min || cabinet_id > cabinet_max) {
    if (reason != nullptr) {
      *reason = "货柜号超出配置范围";
    }
    return false;
  }

  if (row_id < row_min || row_id > row_max) {
    if (reason != nullptr) {
      *reason = "排号超出配置范围";
    }
    return false;
  }

  if (layer_id < layer_min || layer_id > layer_max) {
    if (reason != nullptr) {
      *reason = "层号超出配置范围";
    }
    return false;
  }

  return true;
}

}  // namespace agv_inventory_system

#endif  // AGV_INVENTORY_SYSTEM__ID_UTILS_HPP_
