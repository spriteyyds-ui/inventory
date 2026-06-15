#include "agv_inventory_system/qr_marker_recognizer.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>

#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect.hpp"

namespace agv_inventory_system
{

QrMarkerRecognizer::QrMarkerRecognizer(const QrMarkerParams & params)
: params_(params)
{
}

void QrMarkerRecognizer::set_params(const QrMarkerParams & params)
{
  params_ = params;
}

const QrMarkerParams & QrMarkerRecognizer::params() const
{
  return params_;
}

bool QrMarkerRecognizer::recognize(const cv::Mat & bgr, QrMarkerResult & result) const
{
  result = QrMarkerResult{};
  if (bgr.empty()) {
    result.error_message = "input image is empty";
    return false;
  }

  cv::QRCodeDetector detector;
  cv::Mat points;
  std::string decoded;
  try {
    decoded = detector.detectAndDecode(bgr, points, result.straight_qr);
  } catch (const cv::Exception & e) {
    result.error_message = e.what();
    result.debug_image = drawVisualization(bgr, result);
    result.visualization = result.debug_image;
    return false;
  }

  result.raw_text = decoded;
  result.corners = extractCorners(points);
  if (result.corners.size() == 4U) {
    float cx = 0.0F;
    for (const auto & p : result.corners) {
      cx += p.x;
    }
    cx /= 4.0F;
    result.horizontal_offset = cx - static_cast<float>(bgr.cols) * 0.5F;

    const float side_px = meanSideLength(result.corners);
    if (side_px > 1.0F) {
      result.estimated_distance = static_cast<float>(
        std::max(0.0, params_.known_size_m * static_cast<double>(bgr.cols) / side_px));
    }
  }

  int number = -1;
  if (decoded.empty()) {
    result.error_message = "QR code not decoded";
  } else if (!parsePlainNumber(decoded, number)) {
    result.error_message = "QR payload is not a plain number: " + decoded;
  } else if (number < params_.min_number || number > params_.max_number) {
    std::ostringstream oss;
    oss << "QR number out of range: " << number;
    result.error_message = oss.str();
  } else {
    result.success = true;
    result.valid = true;
    result.number = std::to_string(number);
    result.confidence = static_cast<float>(std::clamp(params_.confidence, 0.0, 1.0));
  }

  if (result.valid && !result.corners.empty()) {
    const float side_px = meanSideLength(result.corners);
    if (side_px < static_cast<float>(std::max(1, params_.min_side_px))) {
      result.success = false;
      result.valid = false;
      result.number.clear();
      result.confidence = 0.0F;
      std::ostringstream oss;
      oss << "QR side too small: " << side_px << "px";
      result.error_message = oss.str();
    }
  }

  result.debug_image = drawVisualization(bgr, result);
  result.visualization = result.debug_image;
  return result.success;
}

std::vector<cv::Point2f> QrMarkerRecognizer::extractCorners(const cv::Mat & points)
{
  std::vector<cv::Point2f> corners;
  if (points.empty()) {
    return corners;
  }

  cv::Mat reshaped = points.reshape(2, static_cast<int>(points.total()));
  corners.reserve(static_cast<std::size_t>(reshaped.rows));
  for (int i = 0; i < reshaped.rows; ++i) {
    if (reshaped.depth() == CV_32F) {
      const auto p = reshaped.at<cv::Vec2f>(i, 0);
      corners.emplace_back(p[0], p[1]);
    } else if (reshaped.depth() == CV_64F) {
      const auto p = reshaped.at<cv::Vec2d>(i, 0);
      corners.emplace_back(static_cast<float>(p[0]), static_cast<float>(p[1]));
    }
  }

  if (corners.size() > 4U) {
    corners.resize(4U);
  }
  return corners;
}

float QrMarkerRecognizer::meanSideLength(const std::vector<cv::Point2f> & corners)
{
  if (corners.size() != 4U) {
    return 0.0F;
  }
  float sum = 0.0F;
  for (std::size_t i = 0; i < corners.size(); ++i) {
    const auto & a = corners[i];
    const auto & b = corners[(i + 1U) % corners.size()];
    sum += std::hypot(a.x - b.x, a.y - b.y);
  }
  return sum / 4.0F;
}

bool QrMarkerRecognizer::parsePlainNumber(const std::string & text, int & number)
{
  if (text.empty()) {
    return false;
  }

  int value = 0;
  for (const unsigned char ch : text) {
    if (!std::isdigit(ch)) {
      return false;
    }
    value = value * 10 + static_cast<int>(ch - '0');
    if (value > 1000000) {
      return false;
    }
  }

  number = value;
  return true;
}

cv::Mat QrMarkerRecognizer::drawVisualization(
  const cv::Mat & bgr,
  const QrMarkerResult & result) const
{
  cv::Mat out = bgr.clone();
  if (out.empty()) {
    return out;
  }

  const cv::Scalar color = result.valid ? cv::Scalar(0, 220, 0) : cv::Scalar(0, 0, 255);
  if (result.corners.size() == 4U) {
    for (std::size_t i = 0; i < result.corners.size(); ++i) {
      cv::line(
        out,
        cv::Point(
          static_cast<int>(std::round(result.corners[i].x)),
          static_cast<int>(std::round(result.corners[i].y))),
        cv::Point(
          static_cast<int>(std::round(result.corners[(i + 1U) % result.corners.size()].x)),
          static_cast<int>(std::round(result.corners[(i + 1U) % result.corners.size()].y))),
        color,
        2,
        cv::LINE_AA);
    }
    cv::Point2f center(0.0F, 0.0F);
    for (const auto & p : result.corners) {
      center += p;
    }
    center *= 0.25F;
    cv::circle(
      out,
      cv::Point(static_cast<int>(std::round(center.x)), static_cast<int>(std::round(center.y))),
      4,
      color,
      -1,
      cv::LINE_AA);
  }

  std::string label = result.valid ?
    ("QR " + result.number + " conf=" + std::to_string(result.confidence).substr(0, 4)) :
    ("QR invalid: " + result.error_message);
  if (label.size() > 80U) {
    label = label.substr(0, 80U);
  }
  cv::putText(
    out,
    label,
    cv::Point(12, 32),
    cv::FONT_HERSHEY_SIMPLEX,
    0.7,
    color,
    2,
    cv::LINE_AA);

  return out;
}

}  // namespace agv_inventory_system
