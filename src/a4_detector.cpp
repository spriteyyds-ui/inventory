#include "agv_inventory_system/a4_detector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "opencv2/imgproc.hpp"

namespace agv_inventory_system
{

A4Detector::A4Detector(const A4DetectorParams & params)
: params_(params)
{
}

void A4Detector::set_params(const A4DetectorParams & params)
{
  params_ = params;
}

const A4DetectorParams & A4Detector::params() const
{
  return params_;
}

bool A4Detector::detect(const cv::Mat & bgr, A4DetectionResult & result) const
{
  result = A4DetectionResult();

  if (bgr.empty()) {
    result.error_message = "输入图像为空";
    return false;
  }

  cv::Mat gray;
  if (bgr.channels() == 3) {
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = bgr.clone();
  }

  int k = params_.gaussian_kernel;
  if (k % 2 == 0) {
    k += 1;
  }
  k = std::max(3, k);
  cv::GaussianBlur(gray, gray, cv::Size(k, k), 0.0);

  cv::Mat binary;
  cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

  int mk = params_.morph_kernel_size;
  if (mk % 2 == 0) {
    mk += 1;
  }
  mk = std::max(3, mk);
  cv::morphologyEx(
    binary,
    binary,
    cv::MORPH_CLOSE,
    cv::getStructuringElement(cv::MORPH_RECT, cv::Size(mk, mk)));

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  if (contours.empty()) {
    result.error_message = "未检测到任何轮廓";
    return false;
  }

  cv::Mat debug;
  cv::cvtColor(gray, debug, cv::COLOR_GRAY2BGR);

  double best_area = 0.0;
  std::vector<cv::Point> best_quad;

  const double img_area = static_cast<double>(bgr.cols * bgr.rows);
  const double min_area = params_.min_area_ratio * img_area;

  for (const auto & contour : contours) {
    const double area = std::abs(cv::contourArea(contour));
    if (area < min_area) {
      continue;
    }

    const double peri = cv::arcLength(contour, true);
    std::vector<cv::Point> approx;
    cv::approxPolyDP(contour, approx, params_.approx_epsilon_ratio * peri, true);

    if (approx.size() != 4) {
      continue;
    }
    if (!cv::isContourConvex(approx)) {
      continue;
    }

    const cv::Rect br = cv::boundingRect(approx);
    const double aspect = static_cast<double>(br.height) / static_cast<double>(std::max(1, br.width));
    if (aspect < params_.min_aspect_ratio || aspect > params_.max_aspect_ratio) {
      continue;
    }

    if (area > best_area) {
      best_area = area;
      best_quad = approx;
    }

    cv::polylines(debug, approx, true, cv::Scalar(255, 160, 0), 2, cv::LINE_AA);
  }

  if (best_quad.size() != 4) {
    result.error_message = "未检测到满足条件的四边形 A4 区域";
    result.debug_image = debug;
    return false;
  }

  const std::array<cv::Point2f, 4> ordered = order_corners(best_quad);

  const double width_top = point_distance(ordered[0], ordered[1]);
  const double width_bottom = point_distance(ordered[3], ordered[2]);
  const double height_left = point_distance(ordered[0], ordered[3]);
  const double height_right = point_distance(ordered[1], ordered[2]);

  const int warp_w = std::max(20, static_cast<int>(std::round(std::max(width_top, width_bottom))));
  const int warp_h = std::max(20, static_cast<int>(std::round(std::max(height_left, height_right))));

  if (warp_w < 30 || warp_h < 30) {
    result.error_message = "A4 候选区域过小，疑似距离过远";
    result.debug_image = debug;
    return false;
  }

  const std::array<cv::Point2f, 4> dst = {
    cv::Point2f(0.0F, 0.0F),
    cv::Point2f(static_cast<float>(warp_w - 1), 0.0F),
    cv::Point2f(static_cast<float>(warp_w - 1), static_cast<float>(warp_h - 1)),
    cv::Point2f(0.0F, static_cast<float>(warp_h - 1))};

  const cv::Mat m = cv::getPerspectiveTransform(ordered.data(), dst.data());
  cv::Mat warped;
  cv::warpPerspective(bgr, warped, m, cv::Size(warp_w, warp_h));

  result.success = true;
  result.area_ratio = best_area / std::max(1.0, img_area);
  result.corners = ordered;
  result.bbox = cv::boundingRect(best_quad);
  result.warped_bgr = warped;
  result.warp_matrix = m;
  result.inverse_warp_matrix = m.inv();

  const std::vector<cv::Point> best_poly = {
    cv::Point(static_cast<int>(std::round(ordered[0].x)), static_cast<int>(std::round(ordered[0].y))),
    cv::Point(static_cast<int>(std::round(ordered[1].x)), static_cast<int>(std::round(ordered[1].y))),
    cv::Point(static_cast<int>(std::round(ordered[2].x)), static_cast<int>(std::round(ordered[2].y))),
    cv::Point(static_cast<int>(std::round(ordered[3].x)), static_cast<int>(std::round(ordered[3].y)))};
  cv::polylines(debug, best_poly, true, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
  for (std::size_t i = 0; i < ordered.size(); ++i) {
    cv::circle(debug, ordered[i], 4, cv::Scalar(0, 255, 255), -1);
    cv::putText(
      debug,
      "p" + std::to_string(i),
      cv::Point(static_cast<int>(ordered[i].x) + 4, static_cast<int>(ordered[i].y) - 4),
      cv::FONT_HERSHEY_SIMPLEX,
      0.5,
      cv::Scalar(0, 255, 0),
      1,
      cv::LINE_AA);
  }
  result.debug_image = debug;
  return true;
}

std::array<cv::Point2f, 4> A4Detector::order_corners(const std::vector<cv::Point> & contour)
{
  std::array<cv::Point2f, 4> ordered{};
  std::array<cv::Point2f, 4> pts = {
    cv::Point2f(static_cast<float>(contour[0].x), static_cast<float>(contour[0].y)),
    cv::Point2f(static_cast<float>(contour[1].x), static_cast<float>(contour[1].y)),
    cv::Point2f(static_cast<float>(contour[2].x), static_cast<float>(contour[2].y)),
    cv::Point2f(static_cast<float>(contour[3].x), static_cast<float>(contour[3].y))};

  // 按 x+y/x-y 排序顶点，得到稳定顺序：左上、右上、右下、左下。
  float min_sum = std::numeric_limits<float>::max();
  float max_sum = -std::numeric_limits<float>::max();
  float min_diff = std::numeric_limits<float>::max();
  float max_diff = -std::numeric_limits<float>::max();

  cv::Point2f tl, tr, br, bl;
  for (const auto & p : pts) {
    const float sum = p.x + p.y;
    const float diff = p.x - p.y;

    if (sum < min_sum) {
      min_sum = sum;
      tl = p;
    }
    if (sum > max_sum) {
      max_sum = sum;
      br = p;
    }
    if (diff < min_diff) {
      min_diff = diff;
      bl = p;
    }
    if (diff > max_diff) {
      max_diff = diff;
      tr = p;
    }
  }

  ordered[0] = tl;
  ordered[1] = tr;
  ordered[2] = br;
  ordered[3] = bl;
  return ordered;
}

double A4Detector::point_distance(const cv::Point2f & a, const cv::Point2f & b)
{
  const double dx = static_cast<double>(a.x - b.x);
  const double dy = static_cast<double>(a.y - b.y);
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace agv_inventory_system
