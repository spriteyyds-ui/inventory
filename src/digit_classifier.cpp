#include "agv_inventory_system/digit_classifier.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <numeric>
#include <utility>

#include "opencv2/imgproc.hpp"

namespace agv_inventory_system
{

DigitClassifier::DigitClassifier(const DigitClassifierParams & params)
: params_(params)
{
}

void DigitClassifier::set_params(const DigitClassifierParams & params)
{
  params_ = params;
  ready_ = false;
  net_ = cv::dnn::Net();
}

const DigitClassifierParams & DigitClassifier::params() const
{
  return params_;
}

bool DigitClassifier::load_model(std::string & error_message)
{
  error_message.clear();
  ready_ = false;
  net_ = cv::dnn::Net();

  if (params_.onnx_model_path.empty()) {
    error_message = "onnx_model_path 为空";
    return false;
  }

  std::ifstream fin(params_.onnx_model_path);
  if (!fin.good()) {
    error_message = "ONNX 模型文件不存在: " + params_.onnx_model_path;
    return false;
  }

  try {
    net_ = cv::dnn::readNetFromONNX(params_.onnx_model_path);
  } catch (const cv::Exception & e) {
    error_message = std::string("加载 ONNX 失败: ") + e.what();
    return false;
  }

  if (net_.empty()) {
    error_message = "加载 ONNX 失败：net 为空";
    return false;
  }

  try {
    if (params_.prefer_cuda) {
      net_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
      net_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    } else {
      net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
      net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
  } catch (const cv::Exception &) {
    // 若 CUDA 不可用则自动回退 CPU。
    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
  }

  ready_ = true;
  return true;
}

bool DigitClassifier::ready() const
{
  return ready_ && !net_.empty();
}

DigitClassification DigitClassifier::classify_digit(const cv::Mat & digit_gray) const
{
  DigitClassification result;

  if (!ready()) {
    result.error_message = "模型未加载";
    return result;
  }

  if (digit_gray.empty()) {
    result.error_message = "输入数字图像为空";
    return result;
  }

  cv::Mat gray;
  if (digit_gray.type() != CV_8UC1) {
    cv::cvtColor(digit_gray, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = digit_gray;
  }

  const int input_size = std::max(8, params_.input_size);
  if (gray.rows != input_size || gray.cols != input_size) {
    cv::resize(gray, gray, cv::Size(input_size, input_size), 0.0, 0.0, cv::INTER_AREA);
  }

  cv::Mat blob = cv::dnn::blobFromImage(
    gray,
    1.0 / 255.0,
    cv::Size(input_size, input_size),
    cv::Scalar(0, 0, 0),
    false,
    false,
    CV_32F);

  cv::Mat output;
  try {
    net_.setInput(blob);
    output = net_.forward();
  } catch (const cv::Exception & e) {
    result.error_message = std::string("DNN 推理失败: ") + e.what();
    return result;
  }

  const std::vector<float> prob = to_probabilities(output);
  if (prob.size() != 10U) {
    result.error_message = "模型输出维度异常，期望10类";
    return result;
  }

  auto it = std::max_element(prob.begin(), prob.end());
  const int pred = static_cast<int>(std::distance(prob.begin(), it));
  const float conf = *it;

  result.digit = pred;
  result.confidence = conf;
  result.probabilities = prob;
  result.success = conf >= static_cast<float>(params_.min_confidence);
  if (!result.success) {
    result.error_message = "分类置信度不足";
  }
  return result;
}

SequenceClassification DigitClassifier::classify_sequence(
  const std::vector<cv::Mat> & digits_gray) const
{
  SequenceClassification sequence;

  if (digits_gray.empty()) {
    sequence.error_message = "输入数字序列为空";
    return sequence;
  }

  float min_conf = 1.0F;
  std::string number;

  for (const auto & digit_img : digits_gray) {
    DigitClassification item = classify_digit(digit_img);
    if (item.digit < 0) {
      sequence.error_message = item.error_message.empty() ? "数字分类失败" : item.error_message;
      sequence.items.push_back(item);
      return sequence;
    }

    min_conf = std::min(min_conf, item.confidence);
    number.push_back(static_cast<char>('0' + item.digit));
    sequence.items.push_back(std::move(item));
  }

  sequence.number = number;
  sequence.min_confidence = min_conf;

  const bool all_items_success = std::all_of(
    sequence.items.begin(), sequence.items.end(),
    [](const DigitClassification & item) { return item.success; });
  sequence.success = all_items_success;

  if (!sequence.success) {
    sequence.error_message = "序列中存在低置信度数字";
  }

  return sequence;
}

std::vector<float> DigitClassifier::to_probabilities(const cv::Mat & output)
{
  if (output.empty()) {
    return {};
  }

  const int total = static_cast<int>(output.total());
  if (total != 10) {
    return {};
  }

  cv::Mat flat = output.reshape(1, 1);
  std::vector<float> values(10, 0.0F);
  for (int i = 0; i < 10; ++i) {
    values[static_cast<std::size_t>(i)] = flat.at<float>(0, i);
  }

  const bool non_negative = std::all_of(
    values.begin(), values.end(), [](float v) { return v >= 0.0F; });
  const float sum = std::accumulate(values.begin(), values.end(), 0.0F);

  if (non_negative && sum > 0.95F && sum < 1.05F) {
    return values;
  }

  // 若模型输出 logits，这里手动做 softmax。
  const float max_v = *std::max_element(values.begin(), values.end());
  float exp_sum = 0.0F;
  for (auto & v : values) {
    v = std::exp(v - max_v);
    exp_sum += v;
  }

  if (exp_sum <= 1e-6F) {
    return {};
  }

  for (auto & v : values) {
    v /= exp_sum;
  }
  return values;
}

}  // namespace agv_inventory_system
