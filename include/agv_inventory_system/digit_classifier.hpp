// Copyright (c) 2026 郁有冬 <spriteyyds@gmail.com>. All rights reserved.
#ifndef AGV_INVENTORY_SYSTEM__DIGIT_CLASSIFIER_HPP_
#define AGV_INVENTORY_SYSTEM__DIGIT_CLASSIFIER_HPP_

#include <string>
#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/dnn.hpp"

namespace agv_inventory_system
{

/**
 * @brief CNN 分类器参数。
 */
struct DigitClassifierParams
{
  std::string onnx_model_path;
  double min_confidence{0.70};
  bool prefer_cuda{false};
  int input_size{64};
};

/**
 * @brief 单数字分类结果。
 */
struct DigitClassification
{
  bool success{false};
  int digit{-1};
  float confidence{0.0F};
  std::vector<float> probabilities;
  std::string error_message;
};

/**
 * @brief 多数字序列分类结果。
 */
struct SequenceClassification
{
  bool success{false};
  std::string number;
  float min_confidence{0.0F};
  std::vector<DigitClassification> items;
  std::string error_message;
};

/**
 * @brief 基于 OpenCV DNN 的数字分类器。
 */
class DigitClassifier
{
public:
  explicit DigitClassifier(const DigitClassifierParams & params = DigitClassifierParams());

  /**
   * @brief 更新参数。
   */
  void set_params(const DigitClassifierParams & params);

  /**
   * @brief 获取当前参数。
   */
  const DigitClassifierParams & params() const;

  /**
   * @brief 加载 ONNX 模型。
   * @return true 表示加载成功。
   */
  bool load_model(std::string & error_message);

  /**
   * @brief 模型是否可用。
   */
  bool ready() const;

  /**
   * @brief 对单张 28x28 灰度图分类。
   */
  DigitClassification classify_digit(const cv::Mat & digit_gray) const;

  /**
   * @brief 对一组 28x28 灰度图分类并拼接成编号。
   */
  SequenceClassification classify_sequence(const std::vector<cv::Mat> & digits_gray) const;

private:
  static std::vector<float> to_probabilities(const cv::Mat & output);

  DigitClassifierParams params_;
  mutable cv::dnn::Net net_;
  bool ready_{false};
};

}  // namespace agv_inventory_system

#endif  // AGV_INVENTORY_SYSTEM__DIGIT_CLASSIFIER_HPP_
