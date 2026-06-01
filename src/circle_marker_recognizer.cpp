#include "agv_inventory_system/circle_marker_recognizer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <utility>

#include "opencv2/imgproc.hpp"
#include "agv_inventory_system/id_utils.hpp"

namespace agv_inventory_system
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

int odd_kernel(int value, int min_value)
{
  int out = std::max(min_value, value);
  if (out % 2 == 0) {
    out += 1;
  }
  return out;
}

cv::Mat ensure_bgr(const cv::Mat & image)
{
  if (image.empty()) {
    return {};
  }
  if (image.type() == CV_8UC1) {
    cv::Mat bgr;
    cv::cvtColor(image, bgr, cv::COLOR_GRAY2BGR);
    return bgr;
  }
  return image.clone();
}

void put_label(cv::Mat & image, const std::string & text, const cv::Point & origin, const cv::Scalar & color)
{
  if (image.empty()) {
    return;
  }
  cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, 0.48, color, 1, cv::LINE_AA);
}

void draw_title(cv::Mat & image, const std::string & title)
{
  if (image.empty()) {
    return;
  }
  cv::rectangle(
    image,
    cv::Rect(0, 0, std::min(image.cols, 260), std::min(image.rows, 26)),
    cv::Scalar(0, 0, 0),
    cv::FILLED);
  cv::putText(
    image,
    title,
    cv::Point(6, 18),
    cv::FONT_HERSHEY_SIMPLEX,
    0.48,
    cv::Scalar(0, 255, 255),
    1,
    cv::LINE_AA);
}

double non_white_ratio(const cv::Mat & image, int & non_white_pixels, int & total_pixels)
{
  non_white_pixels = 0;
  total_pixels = 0;
  if (image.empty()) {
    return 0.0;
  }

  cv::Mat gray;
  if (image.type() != CV_8UC1) {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = image;
  }
  cv::Mat non_white;
  cv::threshold(gray, non_white, 254, 255, cv::THRESH_BINARY_INV);
  non_white_pixels = cv::countNonZero(non_white);
  total_pixels = std::max(1, gray.rows * gray.cols);
  return static_cast<double>(non_white_pixels) / static_cast<double>(total_pixels);
}

}  // namespace

CircleMarkerRecognizer::CircleMarkerRecognizer(const CircleMarkerParams & params)
: params_(params)
{
}

void CircleMarkerRecognizer::set_params(const CircleMarkerParams & params)
{
  params_ = params;
}

const CircleMarkerParams & CircleMarkerRecognizer::params() const
{
  return params_;
}

bool CircleMarkerRecognizer::recognize(
  const cv::Mat & bgr,
  const DigitClassifier & classifier,
  CircleMarkerResult & result) const
{
  result = CircleMarkerResult();

  if (bgr.empty()) {
    result.error_message = "输入图像为空";
    return false;
  }
  if (!classifier.ready()) {
    result.error_message = "分类模型未加载";
    return false;
  }

  if (!detectMarkerPanel(bgr, result.panel)) {
    result.error_message = result.panel.error_message.empty() ? "marker panel 检测失败" : result.panel.error_message;
    result.debug_panel = result.panel.debug_image;
    result.visualization = drawVisualization(bgr, result);
    return false;
  }
  result.debug_panel = result.panel.warped_bgr;

  CircleDetection circle;
  if (!detectCircle(result.panel.warped_bgr, circle)) {
    result.error_message = circle.error_message.empty() ? "圆形标靶检测失败" : circle.error_message;
    result.circle_mask = circle.mask;
    result.debug_panel = circle.debug_image.empty() ? result.panel.warped_bgr : circle.debug_image;
    result.debug_digits = composeDebugDigits(result);
    result.visualization = drawVisualization(bgr, result);
    return false;
  }

  result.circle_bbox = circle.bbox;
  result.circle_mask = circle.mask;
  result.debug_panel = circle.debug_image.empty() ? result.panel.warped_bgr : circle.debug_image;
  const cv::Rect panel_bounds(0, 0, result.panel.warped_bgr.cols, result.panel.warped_bgr.rows);
  const cv::Rect bounded_circle = result.circle_bbox & panel_bounds;
  if (bounded_circle.width > 0 && bounded_circle.height > 0) {
    result.circle_roi = result.panel.warped_bgr(bounded_circle).clone();
  }

  result.extract_digits_success = extractDigits(result.panel.warped_bgr, result);
  result.classifier_input_ready = !result.digit_images.empty();
  if (!result.extract_digits_success) {
    result.debug_digits = composeDebugDigits(result);
    result.visualization = drawVisualization(bgr, result);
    return false;
  }

  const SequenceClassification seq = classifier.classify_sequence(result.digit_images);
  result.classification = seq;
  if (!seq.success || seq.number.empty()) {
    result.error_message = seq.error_message.empty() ? "圆内数字分类失败" : seq.error_message;
    result.confidence = seq.min_confidence;
    result.debug_digits = composeDebugDigits(result);
    result.visualization = drawVisualization(bgr, result);
    return false;
  }

  const std::string normalized = normalize_cabinet_text(seq.number);
  int parsed = -1;
  if (!safe_to_int(normalized, parsed)) {
    result.error_message = "识别结果无法转换为货柜号";
    result.confidence = seq.min_confidence;
    result.debug_digits = composeDebugDigits(result);
    result.visualization = drawVisualization(bgr, result);
    return false;
  }

  result.number = normalized;
  result.confidence = seq.min_confidence;
  result.valid = parsed >= params_.min_number && parsed <= params_.max_number;
  result.success = true;
  if (!result.valid) {
    result.error_message = "圆形标靶数字超出配置范围";
  }
  result.debug_digits = composeDebugDigits(result);
  result.visualization = drawVisualization(bgr, result);
  return result.valid;
}

bool CircleMarkerRecognizer::detectMarkerPanel(const cv::Mat & bgr, MarkerPanelDetection & result) const
{
  result = MarkerPanelDetection();

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

  cv::Mat blurred = gray.clone();
  const int blur_k = odd_kernel(params_.panel_gaussian_kernel, 3);
  cv::GaussianBlur(blurred, blurred, cv::Size(blur_k, blur_k), 0.0);

  cv::Mat binary;
  cv::threshold(blurred, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

  const int morph_k = odd_kernel(params_.panel_morph_kernel_size, 3);
  cv::morphologyEx(
    binary,
    binary,
    cv::MORPH_CLOSE,
    cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morph_k, morph_k)));

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  if (contours.empty()) {
    result.error_message = "未检测到白色 panel 轮廓";
    return false;
  }

  cv::Mat debug;
  cv::cvtColor(gray, debug, cv::COLOR_GRAY2BGR);

  const double image_area = static_cast<double>(std::max(1, bgr.cols * bgr.rows));
  const double min_area = std::max(1.0, image_area * std::clamp(params_.panel_min_area_ratio, 0.0, 1.0));
  const double max_area = image_area * std::clamp(params_.panel_max_area_ratio, 0.01, 1.0);
  const cv::Point2f image_center(
    static_cast<float>(bgr.cols) * 0.5F,
    static_cast<float>(bgr.rows) * 0.5F);

  double best_score = -std::numeric_limits<double>::infinity();
  std::vector<cv::Point> best_quad;

  for (const auto & contour : contours) {
    const double area = std::abs(cv::contourArea(contour));
    if (area < min_area || area > max_area) {
      continue;
    }

    const double peri = cv::arcLength(contour, true);
    if (peri <= 1.0) {
      continue;
    }

    std::vector<cv::Point> approx;
    cv::approxPolyDP(contour, approx, std::max(0.001, params_.panel_approx_epsilon_ratio) * peri, true);
    if (approx.size() != 4 || !cv::isContourConvex(approx)) {
      continue;
    }

    const auto ordered = orderCorners(approx);
    const double width_top = pointDistance(ordered[0], ordered[1]);
    const double width_bottom = pointDistance(ordered[3], ordered[2]);
    const double height_left = pointDistance(ordered[0], ordered[3]);
    const double height_right = pointDistance(ordered[1], ordered[2]);
    const double mean_width = 0.5 * (width_top + width_bottom);
    const double mean_height = 0.5 * (height_left + height_right);
    if (mean_width < 10.0 || mean_height < 10.0) {
      continue;
    }

    const double aspect = mean_width / std::max(1.0, mean_height);
    const double target_aspect = std::max(0.05, params_.panel_aspect_ratio);
    const double aspect_error = std::abs(std::log(aspect / target_aspect));
    const double aspect_tolerance = std::max(0.05, params_.panel_aspect_tolerance);
    const double aspect_score = std::exp(-aspect_error / aspect_tolerance);

    const cv::Moments moments = cv::moments(approx);
    cv::Point2f center = image_center;
    if (std::abs(moments.m00) > 1e-6) {
      center.x = static_cast<float>(moments.m10 / moments.m00);
      center.y = static_cast<float>(moments.m01 / moments.m00);
    }
    const double dx = static_cast<double>(center.x - image_center.x) / std::max(1, bgr.cols);
    const double dy = static_cast<double>(center.y - image_center.y) / std::max(1, bgr.rows);
    const double center_score = std::exp(-4.0 * std::sqrt(dx * dx + dy * dy));

    cv::Mat contour_mask = cv::Mat::zeros(gray.size(), CV_8UC1);
    cv::fillConvexPoly(contour_mask, approx, cv::Scalar(255));
    const cv::Scalar mean_scalar = cv::mean(gray, contour_mask);
    const double brightness_score = std::clamp(mean_scalar[0] / 255.0, 0.0, 1.0);

    const double area_score = std::clamp(area / std::max(min_area, image_area * 0.12), 0.0, 1.0);
    const double width_balance =
      1.0 - std::min(1.0, std::abs(width_top - width_bottom) / std::max(1.0, mean_width));
    const double height_balance =
      1.0 - std::min(1.0, std::abs(height_left - height_right) / std::max(1.0, mean_height));
    const double shape_score = std::clamp(0.5 * (width_balance + height_balance), 0.0, 1.0);

    const double score =
      0.30 * brightness_score +
      0.24 * center_score +
      0.20 * aspect_score +
      0.16 * area_score +
      0.10 * shape_score;

    cv::polylines(debug, approx, true, cv::Scalar(255, 160, 0), 1, cv::LINE_AA);
    put_label(
      debug,
      "p " + std::to_string(score).substr(0, 4),
      cv::Point(approx[0].x, std::max(0, approx[0].y - 4)),
      cv::Scalar(255, 160, 0));

    if (score > best_score) {
      best_score = score;
      best_quad = approx;
    }
  }

  if (best_quad.size() != 4) {
    result.error_message = "未检测到满足条件的 marker panel";
    result.debug_image = debug;
    return false;
  }

  const auto ordered = orderCorners(best_quad);
  const int warp_w = std::max(20, params_.panel_warp_width);
  const int warp_h = std::max(20, params_.panel_warp_height);
  const std::array<cv::Point2f, 4> dst = {
    cv::Point2f(0.0F, 0.0F),
    cv::Point2f(static_cast<float>(warp_w - 1), 0.0F),
    cv::Point2f(static_cast<float>(warp_w - 1), static_cast<float>(warp_h - 1)),
    cv::Point2f(0.0F, static_cast<float>(warp_h - 1))};

  const cv::Mat m = cv::getPerspectiveTransform(ordered.data(), dst.data());
  cv::Mat warped;
  cv::warpPerspective(bgr, warped, m, cv::Size(warp_w, warp_h));

  const std::vector<cv::Point> best_poly = {
    cv::Point(static_cast<int>(std::round(ordered[0].x)), static_cast<int>(std::round(ordered[0].y))),
    cv::Point(static_cast<int>(std::round(ordered[1].x)), static_cast<int>(std::round(ordered[1].y))),
    cv::Point(static_cast<int>(std::round(ordered[2].x)), static_cast<int>(std::round(ordered[2].y))),
    cv::Point(static_cast<int>(std::round(ordered[3].x)), static_cast<int>(std::round(ordered[3].y)))};
  cv::polylines(debug, best_poly, true, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
  put_label(debug, "marker_panel", best_poly[0] + cv::Point(4, 16), cv::Scalar(0, 255, 0));

  result.success = true;
  result.score = best_score;
  result.area_ratio = std::abs(cv::contourArea(best_quad)) / image_area;
  result.corners = ordered;
  result.bbox = cv::boundingRect(best_quad);
  result.warped_bgr = warped;
  result.warp_matrix = m;
  result.inverse_warp_matrix = m.inv();
  result.debug_image = debug;
  return true;
}

bool CircleMarkerRecognizer::detectCircle(const cv::Mat & panel_bgr, CircleDetection & result) const
{
  result = CircleDetection();
  if (panel_bgr.empty()) {
    result.error_message = "marker panel 图像为空";
    return false;
  }

  cv::Mat gray;
  if (panel_bgr.channels() == 3) {
    cv::cvtColor(panel_bgr, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = panel_bgr.clone();
  }

  const cv::Rect search_roi = ratioRect(
    gray.cols,
    gray.rows,
    params_.search_x_min_ratio,
    params_.search_x_max_ratio,
    params_.search_y_min_ratio,
    params_.search_y_max_ratio);
  if (search_roi.width <= 0 || search_roi.height <= 0) {
    result.error_message = "圆形搜索 ROI 无效";
    return false;
  }

  cv::Mat dark_mask = cv::Mat::zeros(gray.size(), CV_8UC1);
  cv::Mat threshold_mask;
  cv::threshold(gray, threshold_mask, std::clamp(params_.dark_threshold, 0, 255), 255, cv::THRESH_BINARY_INV);
  threshold_mask(search_roi).copyTo(dark_mask(search_roi));

  if (params_.use_otsu) {
    cv::Mat otsu_local;
    cv::threshold(gray(search_roi), otsu_local, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
    cv::Mat otsu_mask = cv::Mat::zeros(gray.size(), CV_8UC1);
    otsu_local.copyTo(otsu_mask(search_roi));
    cv::bitwise_and(dark_mask, otsu_mask, dark_mask);
  }

  cv::morphologyEx(
    dark_mask,
    dark_mask,
    cv::MORPH_CLOSE,
    cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
  cv::morphologyEx(
    dark_mask,
    dark_mask,
    cv::MORPH_OPEN,
    cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(dark_mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  if (contours.empty()) {
    result.error_message = "未检测到深色圆形候选";
    result.mask = dark_mask;
    return false;
  }

  cv::Mat debug = panel_bgr.clone();
  cv::rectangle(debug, search_roi, cv::Scalar(255, 160, 0), 1);

  const double panel_area = static_cast<double>(std::max(1, gray.cols * gray.rows));
  const double min_area = panel_area * std::clamp(params_.circle_min_area_ratio, 0.0, 1.0);
  const double max_area = panel_area * std::clamp(params_.circle_max_area_ratio, 0.01, 1.0);
  double best_score = -std::numeric_limits<double>::infinity();
  std::vector<cv::Point> best_contour;
  cv::Rect best_bbox;

  for (const auto & contour : contours) {
    const double area = std::abs(cv::contourArea(contour));
    if (area < min_area || area > max_area) {
      continue;
    }
    const double perimeter = cv::arcLength(contour, true);
    if (perimeter <= 1.0) {
      continue;
    }
    const cv::Rect box = cv::boundingRect(contour);
    if (box.width <= 0 || box.height <= 0) {
      continue;
    }
    const double aspect = static_cast<double>(box.width) / static_cast<double>(box.height);
    if (aspect < params_.circle_aspect_min || aspect > params_.circle_aspect_max) {
      continue;
    }
    const cv::Point center(box.x + box.width / 2, box.y + box.height / 2);
    if (!search_roi.contains(center)) {
      continue;
    }
    const double circularity = 4.0 * kPi * area / std::max(1.0, perimeter * perimeter);
    if (circularity < params_.circle_min_circularity) {
      continue;
    }
    const double area_score = std::clamp(area / std::max(1.0, max_area), 0.0, 1.0);
    const double aspect_score = 1.0 - std::min(1.0, std::abs(std::log(aspect)));
    const double score = 0.65 * std::clamp(circularity, 0.0, 1.0) + 0.20 * aspect_score + 0.15 * area_score;

    cv::rectangle(debug, box, cv::Scalar(0, 180, 255), 1);
    if (score > best_score) {
      best_score = score;
      best_contour = contour;
      best_bbox = box;
    }
  }

  if (best_contour.empty()) {
    result.error_message = "未检测到满足条件的圆形标靶";
    result.mask = dark_mask;
    result.debug_image = debug;
    return false;
  }

  cv::Mat circle_mask = cv::Mat::zeros(gray.size(), CV_8UC1);
  std::vector<std::vector<cv::Point>> selected{best_contour};
  cv::drawContours(circle_mask, selected, 0, cv::Scalar(255), cv::FILLED);

  cv::rectangle(debug, best_bbox, cv::Scalar(0, 255, 0), 2);
  put_label(debug, "circle", best_bbox.tl() + cv::Point(0, -4), cv::Scalar(0, 255, 0));

  result.success = true;
  result.bbox = best_bbox;
  result.mask = circle_mask;
  result.debug_image = debug;
  return true;
}

bool CircleMarkerRecognizer::extractDigits(const cv::Mat & panel_bgr, CircleMarkerResult & result) const
{
  if (panel_bgr.empty() || result.circle_mask.empty() || result.circle_bbox.empty()) {
    result.error_message = "圆形标靶区域为空，无法提取数字";
    return false;
  }

  cv::Mat gray;
  if (panel_bgr.channels() == 3) {
    cv::cvtColor(panel_bgr, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = panel_bgr.clone();
  }

  cv::Mat inner_circle;
  cv::erode(
    result.circle_mask,
    inner_circle,
    cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

  cv::Mat bright_mask;
  cv::threshold(gray, bright_mask, std::clamp(params_.bright_threshold, 0, 255), 255, cv::THRESH_BINARY);
  if (params_.use_otsu) {
    cv::Mat otsu;
    cv::threshold(gray(result.circle_bbox), otsu, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    cv::Mat otsu_full = cv::Mat::zeros(gray.size(), CV_8UC1);
    otsu.copyTo(otsu_full(result.circle_bbox));
    cv::bitwise_or(bright_mask, otsu_full, bright_mask);
  }

  cv::bitwise_and(bright_mask, inner_circle, result.digit_mask);
  cv::morphologyEx(
    result.digit_mask,
    result.digit_mask,
    cv::MORPH_OPEN,
    cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2)));
  cv::morphologyEx(
    result.digit_mask,
    result.digit_mask,
    cv::MORPH_CLOSE,
    cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(result.digit_mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  result.digit_contour_count = static_cast<int>(contours.size());
  if (contours.empty()) {
    result.error_message = "圆内未检测到白色数字候选";
    return false;
  }

  const double circle_area = std::max(1.0, static_cast<double>(cv::countNonZero(inner_circle)));
  const double min_area = circle_area * std::clamp(params_.digit_min_area_ratio, 0.0, 1.0);
  const double max_area = circle_area * std::clamp(params_.digit_max_area_ratio, 0.001, 1.0);
  std::vector<cv::Rect> boxes;
  result.digit_candidate_debug.clear();
  result.digit_candidate_debug.reserve(contours.size());
  result.digit_candidate_rejected_count = 0;

  for (std::size_t i = 0; i < contours.size(); ++i) {
    const auto & contour = contours[i];
    CircleMarkerDigitCandidateDebugInfo info;
    info.contour_index = static_cast<int>(i);
    const double area = std::abs(cv::contourArea(contour));
    info.contour_area = area;
    cv::Rect box = cv::boundingRect(contour);
    info.bbox = box;
    info.bbox_area = static_cast<double>(box.area());
    info.area_ratio = area / circle_area;
    info.height_ratio =
      static_cast<double>(box.height) / static_cast<double>(std::max(1, result.circle_bbox.height));
    info.width_ratio =
      static_cast<double>(box.width) / static_cast<double>(std::max(1, result.circle_bbox.width));
    info.aspect_ratio =
      static_cast<double>(box.width) / static_cast<double>(std::max(1, box.height));
    if (area < min_area || area > max_area) {
      info.reject_reason = area < min_area ? "area_ratio too small" : "area_ratio too large";
      result.digit_candidate_rejected_count += 1;
      result.digit_candidate_debug.push_back(info);
      continue;
    }
    if (box.width < 2 || box.height < 4) {
      if (box.width <= 0 || box.height <= 0) {
        info.reject_reason = "bbox invalid";
      } else if (box.height < 4) {
        info.reject_reason = "height_ratio too small";
      } else {
        info.reject_reason = "width_ratio too small";
      }
      result.digit_candidate_rejected_count += 1;
      result.digit_candidate_debug.push_back(info);
      continue;
    }
    const double aspect = static_cast<double>(box.width) / static_cast<double>(box.height);
    if (aspect < 0.08 || aspect > 2.5) {
      info.reject_reason = "aspect_ratio invalid";
      result.digit_candidate_rejected_count += 1;
      result.digit_candidate_debug.push_back(info);
      continue;
    }
    info.accepted = true;
    result.digit_candidate_debug.push_back(info);
    boxes.push_back(box);
  }

  if (boxes.empty()) {
    result.error_message = "圆内数字候选被过滤为空";
    return false;
  }

  std::sort(boxes.begin(), boxes.end(), [](const cv::Rect & a, const cv::Rect & b) {
    return a.x < b.x;
  });

  std::vector<cv::Rect> merged;
  for (const auto & box : boxes) {
    if (merged.empty()) {
      merged.push_back(box);
      continue;
    }
    cv::Rect & last = merged.back();
    const int gap = box.x - (last.x + last.width);
    const int allowed_gap = std::max(2, static_cast<int>(std::round(result.circle_bbox.width * 0.04)));
    const int overlap_y =
      std::max(0, std::min(last.y + last.height, box.y + box.height) - std::max(last.y, box.y));
    const double overlap_ratio =
      static_cast<double>(overlap_y) / static_cast<double>(std::max(1, std::min(last.height, box.height)));
    if (gap <= allowed_gap && overlap_ratio >= 0.25) {
      last |= box;
    } else {
      merged.push_back(box);
    }
  }

  if (static_cast<int>(merged.size()) > std::max(1, params_.max_digits)) {
    std::sort(merged.begin(), merged.end(), [](const cv::Rect & a, const cv::Rect & b) {
      return a.area() > b.area();
    });
    merged.resize(static_cast<std::size_t>(std::max(1, params_.max_digits)));
    std::sort(merged.begin(), merged.end(), [](const cv::Rect & a, const cv::Rect & b) {
      return a.x < b.x;
    });
  }

  const cv::Rect bounds(0, 0, result.digit_mask.cols, result.digit_mask.rows);
  std::vector<cv::Mat> normalized_digits;
  std::vector<cv::Rect> final_boxes;

  if (merged.size() == 1U && params_.max_digits >= 2) {
    const cv::Rect box = merged.front();
    const double wh = static_cast<double>(box.width) / static_cast<double>(std::max(1, box.height));
    if (wh >= 0.85) {
      std::vector<cv::Mat> split_digits;
      if (splitWideDigitMask(result.digit_mask(box).clone(), split_digits) && split_digits.size() == 2U) {
        normalized_digits = split_digits;
        final_boxes.push_back(box);
      }
    }
  }

  if (normalized_digits.empty()) {
    for (const auto & box : merged) {
      const int padding = std::max(
        2,
        static_cast<int>(std::round(std::max(box.width, box.height) * std::max(0.0, params_.digit_padding_ratio))));
      const cv::Rect padded = expandRect(box, padding, bounds);
      cv::Mat norm = normalizeDigitMask(result.digit_mask(padded).clone());
      if (norm.empty()) {
        continue;
      }
      normalized_digits.push_back(norm);
      final_boxes.push_back(padded);
    }
  }

  if (normalized_digits.empty()) {
    result.error_message = "圆内数字归一化失败";
    return false;
  }
  if (static_cast<int>(normalized_digits.size()) > std::max(1, params_.max_digits)) {
    result.error_message = "圆内数字数量超过限制";
    return false;
  }

  result.digit_images = std::move(normalized_digits);
  result.digit_debug.clear();
  result.digit_debug.reserve(result.digit_images.size());
  for (const auto & digit_image : result.digit_images) {
    CircleMarkerDigitDebugInfo info;
    info.width = digit_image.cols;
    info.height = digit_image.rows;
    info.non_white_ratio = non_white_ratio(digit_image, info.non_white_pixels, info.total_pixels);
    result.digit_debug.push_back(info);
  }
  result.digit_boxes = std::move(final_boxes);
  if (!result.digit_boxes.empty()) {
    result.digit_union_bbox = result.digit_boxes.front();
    for (std::size_t i = 1; i < result.digit_boxes.size(); ++i) {
      result.digit_union_bbox |= result.digit_boxes[i];
    }
  }
  return true;
}

bool CircleMarkerRecognizer::splitWideDigitMask(const cv::Mat & mask, std::vector<cv::Mat> & out) const
{
  out.clear();
  if (mask.empty() || mask.cols < 8) {
    return false;
  }

  cv::Mat binary;
  cv::threshold(mask, binary, 0, 255, cv::THRESH_BINARY);
  const int min_x = std::max(1, static_cast<int>(std::round(binary.cols * 0.30)));
  const int max_x = std::min(binary.cols - 2, static_cast<int>(std::round(binary.cols * 0.70)));
  if (min_x >= max_x) {
    return false;
  }

  int best_x = binary.cols / 2;
  int best_count = std::numeric_limits<int>::max();
  int max_left = 0;
  int max_right = 0;
  for (int x = min_x; x <= max_x; ++x) {
    const int count = cv::countNonZero(binary.col(x));
    if (count < best_count) {
      best_count = count;
      best_x = x;
    }
  }
  for (int x = 0; x < best_x; ++x) {
    max_left = std::max(max_left, cv::countNonZero(binary.col(x)));
  }
  for (int x = best_x + 1; x < binary.cols; ++x) {
    max_right = std::max(max_right, cv::countNonZero(binary.col(x)));
  }
  const int side_peak = std::max(1, std::min(max_left, max_right));
  if (static_cast<double>(best_count) / static_cast<double>(side_peak) > 0.35) {
    return false;
  }

  const cv::Rect left_rect(0, 0, best_x, binary.rows);
  const cv::Rect right_rect(best_x, 0, binary.cols - best_x, binary.rows);
  if (cv::countNonZero(binary(left_rect)) < 8 || cv::countNonZero(binary(right_rect)) < 8) {
    return false;
  }

  cv::Mat left = normalizeDigitMask(binary(left_rect).clone());
  cv::Mat right = normalizeDigitMask(binary(right_rect).clone());
  if (left.empty() || right.empty()) {
    return false;
  }
  out.push_back(left);
  out.push_back(right);
  return true;
}

cv::Mat CircleMarkerRecognizer::normalizeDigitMask(const cv::Mat & binary_digit) const
{
  const int canvas_size = std::max(8, params_.digit_input_size);
  if (binary_digit.empty()) {
    return {};
  }

  cv::Mat mask;
  if (binary_digit.type() != CV_8UC1) {
    cv::cvtColor(binary_digit, mask, cv::COLOR_BGR2GRAY);
  } else {
    mask = binary_digit.clone();
  }
  cv::threshold(mask, mask, 0, 255, cv::THRESH_BINARY);

  std::vector<cv::Point> points;
  cv::findNonZero(mask, points);
  if (points.empty()) {
    return {};
  }

  const cv::Rect bb = cv::boundingRect(points);
  cv::Mat glyph = mask(bb).clone();
  const int pad = std::max(2, canvas_size / 8);
  const int target_w = std::max(1, canvas_size - 2 * pad);
  const int target_h = std::max(1, canvas_size - 2 * pad);
  const double scale = std::min(
    static_cast<double>(target_w) / static_cast<double>(std::max(1, glyph.cols)),
    static_cast<double>(target_h) / static_cast<double>(std::max(1, glyph.rows)));
  const int w = std::max(1, static_cast<int>(std::round(glyph.cols * scale)));
  const int h = std::max(1, static_cast<int>(std::round(glyph.rows * scale)));

  cv::Mat resized;
  cv::resize(glyph, resized, cv::Size(w, h), 0.0, 0.0, cv::INTER_NEAREST);
  cv::Mat canvas(canvas_size, canvas_size, CV_8UC1, cv::Scalar(255));
  const int x = (canvas_size - w) / 2;
  const int y = (canvas_size - h) / 2;
  canvas(cv::Rect(x, y, w, h)).setTo(0, resized);
  return canvas;
}

cv::Mat CircleMarkerRecognizer::composeDebugDigits(const CircleMarkerResult & result) const
{
  std::vector<cv::Mat> panels;
  if (!result.circle_mask.empty()) {
    cv::Mat circle_mask = ensure_bgr(result.circle_mask);
    draw_title(circle_mask, "circle_mask");
    panels.push_back(circle_mask);
  }
  if (!result.circle_roi.empty()) {
    cv::Mat circle_roi = ensure_bgr(result.circle_roi);
    draw_title(circle_roi, "circle_roi");
    panels.push_back(circle_roi);
  }
  if (!result.digit_mask.empty()) {
    cv::Mat digit_vis = ensure_bgr(result.digit_mask);
    for (const auto & box : result.digit_boxes) {
      cv::rectangle(digit_vis, box, cv::Scalar(0, 255, 0), 1);
    }
    draw_title(digit_vis, "digit_mask");
    panels.push_back(digit_vis);
  }
  if (!result.digit_images.empty()) {
    const int tile = std::max(16, result.digit_images.front().cols);
    const int margin = 6;
    const int title_h = 26;
    cv::Mat montage(title_h + tile + 2 * margin + 18,
      static_cast<int>(result.digit_images.size()) * (tile + margin) + margin,
      CV_8UC1, cv::Scalar(230));
    draw_title(montage, "classifier_input");
    for (std::size_t i = 0; i < result.digit_images.size(); ++i) {
      cv::Mat resized;
      cv::resize(result.digit_images[i], resized, cv::Size(tile, tile), 0.0, 0.0, cv::INTER_NEAREST);
      const int x = margin + static_cast<int>(i) * (tile + margin);
      const int y = title_h + margin;
      resized.copyTo(montage(cv::Rect(x, y, tile, tile)));
      put_label(montage, "input", cv::Point(x, title_h + tile + margin + 14), cv::Scalar(20));
    }
    panels.push_back(ensure_bgr(montage));
  }
  if (panels.empty()) {
    return {};
  }

  int target_h = 0;
  int total_w = 0;
  for (const auto & panel : panels) {
    target_h = std::max(target_h, panel.rows);
  }
  target_h = std::max(1, std::min(target_h, 360));

  std::vector<cv::Mat> resized_panels;
  for (const auto & panel : panels) {
    const double scale = static_cast<double>(target_h) / static_cast<double>(std::max(1, panel.rows));
    const int w = std::max(1, static_cast<int>(std::round(panel.cols * scale)));
    cv::Mat resized;
    cv::resize(panel, resized, cv::Size(w, target_h), 0.0, 0.0, cv::INTER_NEAREST);
    total_w += w;
    resized_panels.push_back(resized);
  }

  cv::Mat out(target_h, total_w, CV_8UC3, cv::Scalar(30, 30, 30));
  int x = 0;
  for (const auto & panel : resized_panels) {
    panel.copyTo(out(cv::Rect(x, 0, panel.cols, panel.rows)));
    x += panel.cols;
  }
  return out;
}

cv::Mat CircleMarkerRecognizer::drawVisualization(const cv::Mat & bgr, const CircleMarkerResult & result) const
{
  if (bgr.empty()) {
    return {};
  }
  cv::Mat visualization = bgr.clone();
  if (result.panel.success) {
    std::vector<cv::Point> panel_poly;
    for (const auto & c : result.panel.corners) {
      panel_poly.emplace_back(static_cast<int>(std::round(c.x)), static_cast<int>(std::round(c.y)));
    }
    if (panel_poly.size() == 4U) {
      cv::polylines(visualization, panel_poly, true, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }

    cv::Rect marker_box = result.digit_union_bbox.empty() ? result.circle_bbox : result.digit_union_bbox;
    if (!marker_box.empty() && !result.panel.inverse_warp_matrix.empty()) {
      std::vector<cv::Point2f> warped_pts = {
        cv::Point2f(static_cast<float>(marker_box.x), static_cast<float>(marker_box.y)),
        cv::Point2f(static_cast<float>(marker_box.x + marker_box.width), static_cast<float>(marker_box.y)),
        cv::Point2f(static_cast<float>(marker_box.x + marker_box.width), static_cast<float>(marker_box.y + marker_box.height)),
        cv::Point2f(static_cast<float>(marker_box.x), static_cast<float>(marker_box.y + marker_box.height))};
      std::vector<cv::Point2f> src_pts;
      cv::perspectiveTransform(warped_pts, src_pts, result.panel.inverse_warp_matrix);
      if (src_pts.size() == 4U) {
        for (int i = 0; i < 4; ++i) {
          cv::line(
            visualization,
            src_pts[static_cast<std::size_t>(i)],
            src_pts[static_cast<std::size_t>((i + 1) % 4)],
            cv::Scalar(0, 255, 255),
            2,
            cv::LINE_AA);
        }
      }
    }
  }

  const std::string status =
    "circle num=" + (result.number.empty() ? std::string("-") : result.number) +
    " conf=" + std::to_string(result.confidence).substr(0, 4) +
    " valid=" + std::string(result.valid ? "1" : "0");
  cv::putText(
    visualization,
    status,
    cv::Point(20, 40),
    cv::FONT_HERSHEY_SIMPLEX,
    0.75,
    result.valid ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255),
    2,
    cv::LINE_AA);
  if (!result.error_message.empty()) {
    cv::putText(
      visualization,
      result.error_message.substr(0, 36),
      cv::Point(20, 72),
      cv::FONT_HERSHEY_SIMPLEX,
      0.55,
      cv::Scalar(0, 140, 255),
      1,
      cv::LINE_AA);
  }
  return visualization;
}

std::array<cv::Point2f, 4> CircleMarkerRecognizer::orderCorners(const std::vector<cv::Point> & contour)
{
  std::array<cv::Point2f, 4> ordered{};
  if (contour.size() < 4U) {
    return ordered;
  }

  std::array<cv::Point2f, 4> pts = {
    cv::Point2f(static_cast<float>(contour[0].x), static_cast<float>(contour[0].y)),
    cv::Point2f(static_cast<float>(contour[1].x), static_cast<float>(contour[1].y)),
    cv::Point2f(static_cast<float>(contour[2].x), static_cast<float>(contour[2].y)),
    cv::Point2f(static_cast<float>(contour[3].x), static_cast<float>(contour[3].y))};

  float min_sum = std::numeric_limits<float>::max();
  float max_sum = -std::numeric_limits<float>::max();
  float min_diff = std::numeric_limits<float>::max();
  float max_diff = -std::numeric_limits<float>::max();
  for (const auto & p : pts) {
    const float sum = p.x + p.y;
    const float diff = p.x - p.y;
    if (sum < min_sum) {
      min_sum = sum;
      ordered[0] = p;
    }
    if (sum > max_sum) {
      max_sum = sum;
      ordered[2] = p;
    }
    if (diff < min_diff) {
      min_diff = diff;
      ordered[3] = p;
    }
    if (diff > max_diff) {
      max_diff = diff;
      ordered[1] = p;
    }
  }
  return ordered;
}

double CircleMarkerRecognizer::pointDistance(const cv::Point2f & a, const cv::Point2f & b)
{
  const double dx = static_cast<double>(a.x - b.x);
  const double dy = static_cast<double>(a.y - b.y);
  return std::sqrt(dx * dx + dy * dy);
}

cv::Rect CircleMarkerRecognizer::ratioRect(
  int width,
  int height,
  double x_min,
  double x_max,
  double y_min,
  double y_max)
{
  const double x0r = std::clamp(std::min(x_min, x_max), 0.0, 1.0);
  const double x1r = std::clamp(std::max(x_min, x_max), 0.0, 1.0);
  const double y0r = std::clamp(std::min(y_min, y_max), 0.0, 1.0);
  const double y1r = std::clamp(std::max(y_min, y_max), 0.0, 1.0);
  const int x0 = std::clamp(static_cast<int>(std::round(width * x0r)), 0, std::max(0, width - 1));
  const int y0 = std::clamp(static_cast<int>(std::round(height * y0r)), 0, std::max(0, height - 1));
  const int x1 = std::clamp(static_cast<int>(std::round(width * x1r)), x0 + 1, width);
  const int y1 = std::clamp(static_cast<int>(std::round(height * y1r)), y0 + 1, height);
  return cv::Rect(x0, y0, x1 - x0, y1 - y0);
}

cv::Rect CircleMarkerRecognizer::expandRect(const cv::Rect & rect, int padding, const cv::Rect & bounds)
{
  const int p = std::max(0, padding);
  const cv::Rect expanded(rect.x - p, rect.y - p, rect.width + 2 * p, rect.height + 2 * p);
  return expanded & bounds;
}

}  // namespace agv_inventory_system
