#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "agv_inventory_system/a4_detector.hpp"
#include "agv_inventory_system/digit_classifier.hpp"
#include "agv_inventory_system/digit_segmenter.hpp"
#include "agv_inventory_system/id_utils.hpp"

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::cerr << "用法: offline_test <onnx_model> <image_path> [input_size]" << std::endl;
    return 1;
  }

  const std::string model_path = argv[1];
  const std::string image_path = argv[2];
  const int input_size = (argc >= 4) ? std::max(8, std::stoi(argv[3])) : 64;

  if (!std::filesystem::exists(model_path)) {
    std::cerr << "模型不存在: " << model_path << std::endl;
    return 2;
  }
  if (!std::filesystem::exists(image_path)) {
    std::cerr << "图像不存在: " << image_path << std::endl;
    return 3;
  }

  cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
  if (image.empty()) {
    std::cerr << "读取图像失败: " << image_path << std::endl;
    return 4;
  }

  agv_inventory_system::A4Detector detector;

  agv_inventory_system::DigitSegmenterParams seg_params;
  seg_params.digit_input_size = input_size;
  agv_inventory_system::DigitSegmenter segmenter(seg_params);

  agv_inventory_system::DigitClassifierParams cls_params;
  cls_params.onnx_model_path = model_path;
  cls_params.input_size = input_size;
  cls_params.min_confidence = 0.0;
  agv_inventory_system::DigitClassifier classifier(cls_params);

  std::string err;
  if (!classifier.load_model(err)) {
    std::cerr << "加载模型失败: " << err << std::endl;
    return 5;
  }

  agv_inventory_system::A4DetectionResult a4;
  if (!detector.detect(image, a4)) {
    std::cerr << "A4检测失败: " << a4.error_message << std::endl;
    return 6;
  }

  agv_inventory_system::DigitSegmentationResult seg;
  if (!segmenter.segment(a4.warped_bgr, seg)) {
    std::cerr << "数字分割失败: " << seg.error_message << std::endl;
    return 7;
  }

  std::vector<cv::Mat> digits;
  digits.reserve(seg.digits.size());
  for (const auto & d : seg.digits) {
    digits.push_back(d.digit_gray);
  }

  const auto seq = classifier.classify_sequence(digits);
  if (seq.number.empty()) {
    std::cerr << "数字分类失败: " << seq.error_message << std::endl;
    return 8;
  }

  const std::string normalized = agv_inventory_system::normalize_cabinet_text(seq.number);

  std::cout << "raw_number=" << seq.number << std::endl;
  std::cout << "normalized=" << normalized << std::endl;
  std::cout << "min_conf=" << seq.min_confidence << std::endl;
  std::cout << "digit_count=" << seg.digits.size() << std::endl;

  cv::imwrite("/tmp/offline_a4.jpg", a4.warped_bgr);
  if (!seg.boxed_debug.empty()) {
    cv::imwrite("/tmp/offline_digits_boxed.jpg", seg.boxed_debug);
  }
  if (!seg.montage_debug.empty()) {
    cv::imwrite("/tmp/offline_digits_montage.jpg", seg.montage_debug);
  }

  std::cout << "debug outputs: /tmp/offline_a4.jpg /tmp/offline_digits_boxed.jpg /tmp/offline_digits_montage.jpg" << std::endl;
  return 0;
}
