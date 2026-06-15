#ifndef WHEELTEC_INVENTORY_SYSTEM__QR_MARKER_RECOGNIZER_HPP_
#define WHEELTEC_INVENTORY_SYSTEM__QR_MARKER_RECOGNIZER_HPP_

#include <string>
#include <vector>

#include "opencv2/core.hpp"

namespace agv_inventory_system
{

struct QrMarkerParams
{
  int min_number{1};
  int max_number{36};
  int min_side_px{60};
  double known_size_m{0.20};
  double confidence{1.0};
};

struct QrMarkerResult
{
  bool success{false};
  bool valid{false};
  std::string raw_text;
  std::string number;
  float confidence{0.0F};
  float horizontal_offset{0.0F};
  float estimated_distance{0.0F};
  std::string error_message;
  std::vector<cv::Point2f> corners;
  cv::Mat straight_qr;
  cv::Mat debug_image;
  cv::Mat visualization;
};

class QrMarkerRecognizer
{
public:
  explicit QrMarkerRecognizer(const QrMarkerParams & params = QrMarkerParams());

  void set_params(const QrMarkerParams & params);
  const QrMarkerParams & params() const;

  bool recognize(const cv::Mat & bgr, QrMarkerResult & result) const;

private:
  static std::vector<cv::Point2f> extractCorners(const cv::Mat & points);
  static float meanSideLength(const std::vector<cv::Point2f> & corners);
  static bool parsePlainNumber(const std::string & text, int & number);
  cv::Mat drawVisualization(const cv::Mat & bgr, const QrMarkerResult & result) const;

  QrMarkerParams params_;
};

}  // namespace agv_inventory_system

#endif  // WHEELTEC_INVENTORY_SYSTEM__QR_MARKER_RECOGNIZER_HPP_
