#ifndef WHEELTEC_INVENTORY_SYSTEM__NUMBER_RECOGNIZER_HPP_
#define WHEELTEC_INVENTORY_SYSTEM__NUMBER_RECOGNIZER_HPP_

#include <string>
#include <vector>

#include "opencv2/core.hpp"
#include "agv_inventory_system/a4_detector.hpp"
#include "agv_inventory_system/digit_classifier.hpp"
#include "agv_inventory_system/digit_segmenter.hpp"

namespace agv_inventory_system
{

/**
 * @brief 数字识别流水线参数。
 */
struct NumberRecognizerParams
{
  A4DetectorParams a4;
  DigitSegmenterParams segmenter;
  DigitClassifierParams classifier;

  int max_attempts{20};
  double known_digit_height_px{40.0};
  double known_distance_m{1.0};
  double distance_focal_length{600.0};
  int cabinet_id_min{1};
  int cabinet_id_max{36};
};

/**
 * @brief 数字识别流水线结果。
 */
struct NumberRecognizerResult
{
  bool success{false};
  bool valid{false};
  std::string number;
  float confidence{0.0F};
  float horizontal_offset{0.0F};
  float estimated_distance{0.0F};
  int attempts{0};
  std::string error_message;

  // 调试输出
  cv::Mat debug_a4_region;
  cv::Mat debug_digits;
  cv::Mat visualization;
};

}  // namespace agv_inventory_system

#endif  // WHEELTEC_INVENTORY_SYSTEM__NUMBER_RECOGNIZER_HPP_
