#ifndef WHEELTEC_INVENTORY_SYSTEM__CIRCLE_MARKER_RECOGNIZER_HPP_
#define WHEELTEC_INVENTORY_SYSTEM__CIRCLE_MARKER_RECOGNIZER_HPP_

#include <array>
#include <string>
#include <vector>

#include "opencv2/core.hpp"
#include "agv_inventory_system/digit_classifier.hpp"

namespace agv_inventory_system
{

struct CircleMarkerParams
{
  bool use_color{false};

  int panel_gaussian_kernel{5};
  int panel_morph_kernel_size{5};
  double panel_min_area_ratio{0.01};
  double panel_max_area_ratio{0.90};
  double panel_approx_epsilon_ratio{0.02};
  double panel_aspect_ratio{1.414};
  double panel_aspect_tolerance{0.35};
  int panel_warp_width{640};
  int panel_warp_height{452};

  double search_x_min_ratio{0.10};
  double search_x_max_ratio{0.90};
  double search_y_min_ratio{0.10};
  double search_y_max_ratio{0.90};

  int dark_threshold{180};
  int bright_threshold{190};
  bool use_otsu{true};

  double circle_min_area_ratio{0.03};
  double circle_max_area_ratio{0.55};
  double circle_min_circularity{0.50};
  double circle_aspect_min{0.70};
  double circle_aspect_max{1.35};

  double digit_min_area_ratio{0.001};
  double digit_max_area_ratio{0.30};
  double digit_padding_ratio{0.22};
  int max_digits{2};
  int min_number{1};
  int max_number{36};
  int digit_input_size{64};
};

struct MarkerPanelDetection
{
  bool success{false};
  std::string error_message;
  double score{0.0};
  double area_ratio{0.0};
  std::array<cv::Point2f, 4> corners{};
  cv::Rect bbox;
  cv::Mat warped_bgr;
  cv::Mat warp_matrix;
  cv::Mat inverse_warp_matrix;
  cv::Mat debug_image;
};

struct CircleMarkerDigitDebugInfo
{
  int width{0};
  int height{0};
  int non_white_pixels{0};
  int total_pixels{0};
  double non_white_ratio{0.0};
};

struct CircleMarkerResult
{
  bool success{false};
  bool valid{false};
  std::string number;
  float confidence{0.0F};
  std::string error_message;
  bool extract_digits_success{false};
  bool classifier_input_ready{false};
  SequenceClassification classification;

  MarkerPanelDetection panel;
  cv::Rect circle_bbox;
  cv::Rect digit_union_bbox;
  cv::Mat circle_mask;
  cv::Mat circle_roi;
  cv::Mat digit_mask;
  std::vector<cv::Rect> digit_boxes;
  std::vector<cv::Mat> digit_images;
  std::vector<CircleMarkerDigitDebugInfo> digit_debug;

  cv::Mat debug_panel;
  cv::Mat debug_digits;
  cv::Mat visualization;
};

class CircleMarkerRecognizer
{
public:
  explicit CircleMarkerRecognizer(const CircleMarkerParams & params = CircleMarkerParams());

  void set_params(const CircleMarkerParams & params);
  const CircleMarkerParams & params() const;

  bool recognize(
    const cv::Mat & bgr,
    const DigitClassifier & classifier,
    CircleMarkerResult & result) const;

  bool detectMarkerPanel(const cv::Mat & bgr, MarkerPanelDetection & result) const;

private:
  struct CircleDetection
  {
    bool success{false};
    std::string error_message;
    cv::Rect bbox;
    cv::Mat mask;
    cv::Mat debug_image;
  };

  bool detectCircle(const cv::Mat & panel_bgr, CircleDetection & result) const;
  bool extractDigits(const cv::Mat & panel_bgr, CircleMarkerResult & result) const;
  bool splitWideDigitMask(const cv::Mat & mask, std::vector<cv::Mat> & out) const;
  cv::Mat normalizeDigitMask(const cv::Mat & binary_digit) const;
  cv::Mat composeDebugDigits(const CircleMarkerResult & result) const;
  cv::Mat drawVisualization(const cv::Mat & bgr, const CircleMarkerResult & result) const;

  static std::array<cv::Point2f, 4> orderCorners(const std::vector<cv::Point> & contour);
  static double pointDistance(const cv::Point2f & a, const cv::Point2f & b);
  static cv::Rect ratioRect(
    int width,
    int height,
    double x_min,
    double x_max,
    double y_min,
    double y_max);
  static cv::Rect expandRect(const cv::Rect & rect, int padding, const cv::Rect & bounds);

  CircleMarkerParams params_;
};

}  // namespace agv_inventory_system

#endif  // WHEELTEC_INVENTORY_SYSTEM__CIRCLE_MARKER_RECOGNIZER_HPP_
