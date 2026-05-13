#include "wheeltec_inventory_system/digit_segmenter.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "opencv2/imgproc.hpp"

namespace wheeltec_inventory_system
{
namespace
{

cv::Rect expand_rect(const cv::Rect & rect, int pad, const cv::Rect & bounds)
{
  const cv::Rect expanded(
    rect.x - pad,
    rect.y - pad,
    rect.width + 2 * pad,
    rect.height + 2 * pad);
  return expanded & bounds;
}

cv::Scalar reject_color(const std::string & reason)
{
  if (reason.rfind("edge", 0) == 0) {
    return cv::Scalar(255, 0, 255);  // edge/border
  }
  if (reason == "vertical_line" || reason == "horizontal_line") {
    return cv::Scalar(255, 80, 0);  // thin vertical/horizontal line
  }
  if (reason == "extent_low" || reason == "extent_high") {
    return cv::Scalar(180, 0, 255);  // fill/extent
  }
  return cv::Scalar(0, 0, 255);
}

}  // namespace

DigitSegmenter::DigitSegmenter(const DigitSegmenterParams & params)
: params_(params)
{
}

void DigitSegmenter::set_params(const DigitSegmenterParams & params)
{
  params_ = params;
}

const DigitSegmenterParams & DigitSegmenter::params() const
{
  return params_;
}

bool DigitSegmenter::segment(const cv::Mat & a4_bgr, DigitSegmentationResult & result) const
{
  result = DigitSegmentationResult();

  if (a4_bgr.empty()) {
    result.error_message = "A4 区域图像为空";
    return false;
  }

  cv::Mat gray;
  if (a4_bgr.channels() == 3) {
    cv::cvtColor(a4_bgr, gray, cv::COLOR_BGR2GRAY);
  } else {
    gray = a4_bgr.clone();
  }

  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(
    std::max(0.1, params_.clahe_clip_limit),
    cv::Size(std::max(2, params_.clahe_grid_size), std::max(2, params_.clahe_grid_size)));
  cv::Mat gray_eq;
  clahe->apply(gray, gray_eq);

  int blur_k = params_.pre_blur_kernel;
  if (blur_k % 2 == 0) {
    blur_k += 1;
  }
  blur_k = std::max(1, blur_k);
  if (blur_k >= 3) {
    cv::GaussianBlur(gray_eq, gray_eq, cv::Size(blur_k, blur_k), 0.0);
  }

  int median_k = params_.median_blur_kernel;
  if (median_k % 2 == 0) {
    median_k += 1;
  }
  median_k = std::clamp(median_k, 1, 7);
  if (median_k >= 3) {
    cv::medianBlur(gray_eq, gray_eq, median_k);
  }

  int block = params_.adaptive_thresh_block_size;
  if (block % 2 == 0) {
    block += 1;
  }
  block = std::max(3, block);

  cv::Mat binary_adapt;
  cv::adaptiveThreshold(
    gray_eq,
    binary_adapt,
    255,
    cv::ADAPTIVE_THRESH_GAUSSIAN_C,
    cv::THRESH_BINARY_INV,
    block,
    params_.adaptive_thresh_c);

  cv::Mat binary = binary_adapt;
  if (params_.enable_otsu_fusion) {
    cv::Mat binary_otsu;
    cv::threshold(gray_eq, binary_otsu, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
    cv::bitwise_or(binary_adapt, binary_otsu, binary);
  }

  if (params_.binary_cleanup_enabled && !binary.empty()) {
    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int component_count = cv::connectedComponentsWithStats(
      binary, labels, stats, centroids, 8, CV_32S);
    if (component_count > 1) {
      cv::Mat keep_mask = cv::Mat::zeros(binary.size(), CV_8UC1);
      const int min_component_area = std::max(0, params_.binary_cleanup_min_component_area);
      const int min_component_width = std::max(0, params_.binary_cleanup_min_component_width);
      const int min_component_height = std::max(0, params_.binary_cleanup_min_component_height);
      for (int label = 1; label < component_count; ++label) {
        const int area = stats.at<int>(label, cv::CC_STAT_AREA);
        const int width = stats.at<int>(label, cv::CC_STAT_WIDTH);
        const int height = stats.at<int>(label, cv::CC_STAT_HEIGHT);
        if (area < min_component_area || width < min_component_width ||
          height < min_component_height)
        {
          continue;
        }
        keep_mask.setTo(255, labels == label);
      }
      binary.setTo(0);
      binary.setTo(255, keep_mask);
    }
  }

  int open_k = params_.morph_open_kernel_size > 0 ? params_.morph_open_kernel_size : params_.morph_kernel_size;
  if (open_k % 2 == 0) {
    open_k += 1;
  }
  open_k = std::max(1, open_k);
  if (open_k >= 3) {
    cv::morphologyEx(
      binary,
      binary,
      cv::MORPH_OPEN,
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(open_k, open_k)));
  }

  int close_k = params_.morph_close_kernel_size > 0 ? params_.morph_close_kernel_size : params_.morph_kernel_size;
  if (close_k % 2 == 0) {
    close_k += 1;
  }
  close_k = std::max(1, close_k);
  if (close_k >= 3) {
    cv::morphologyEx(
      binary,
      binary,
      cv::MORPH_CLOSE,
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(close_k, close_k)));
  }

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  auto colorize_binary = [](const cv::Mat & mono) -> cv::Mat {
    if (mono.empty()) {
      return {};
    }
    cv::Mat bgr;
    if (mono.type() == CV_8UC1) {
      cv::cvtColor(mono, bgr, cv::COLOR_GRAY2BGR);
    } else {
      bgr = mono.clone();
    }
    return bgr;
  };

  auto draw_title = [](cv::Mat & img, const std::string & title) {
    if (img.empty()) {
      return;
    }
    cv::rectangle(img, cv::Rect(0, 0, std::min(360, img.cols), 28), cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(
      img,
      title,
      cv::Point(8, 20),
      cv::FONT_HERSHEY_SIMPLEX,
      0.55,
      cv::Scalar(0, 255, 255),
      1,
      cv::LINE_AA);
  };

  auto make_panel = [&](const cv::Mat & bin_mono, const cv::Mat & raw_boxes, const cv::Mat & kept_boxes,
                       const cv::Mat & merged_boxes) -> cv::Mat {
    std::vector<cv::Mat> tiles{
      colorize_binary(bin_mono),
      raw_boxes.clone(),
      kept_boxes.clone(),
      merged_boxes.clone()};

    int tile_w = 320;
    int tile_h = 240;
    for (const auto & t : tiles) {
      if (!t.empty()) {
        tile_w = std::max(tile_w, t.cols);
        tile_h = std::max(tile_h, t.rows);
      }
    }

    for (auto & t : tiles) {
      if (t.empty()) {
        t = cv::Mat(tile_h, tile_w, CV_8UC3, cv::Scalar(20, 20, 20));
      } else if (t.type() == CV_8UC1) {
        cv::cvtColor(t, t, cv::COLOR_GRAY2BGR);
      }
      if (t.cols != tile_w || t.rows != tile_h) {
        cv::resize(t, t, cv::Size(tile_w, tile_h), 0.0, 0.0, cv::INTER_NEAREST);
      }
    }

    draw_title(tiles[0], "binary");
    draw_title(tiles[1], "raw candidates");
    draw_title(tiles[2], "filtered + rejects");
    draw_title(tiles[3], "final + union");

    cv::Mat panel(tile_h * 2, tile_w * 2, CV_8UC3, cv::Scalar(15, 15, 15));
    tiles[0].copyTo(panel(cv::Rect(0, 0, tile_w, tile_h)));
    tiles[1].copyTo(panel(cv::Rect(tile_w, 0, tile_w, tile_h)));
    tiles[2].copyTo(panel(cv::Rect(0, tile_h, tile_w, tile_h)));
    tiles[3].copyTo(panel(cv::Rect(tile_w, tile_h, tile_w, tile_h)));
    return panel;
  };

  if (contours.empty()) {
    result.error_message = "未检测到数字轮廓";
    result.final_zero_reason = "no_contours";
    result.binary_debug = binary;
    result.boxed_debug = make_panel(binary, cv::Mat{}, cv::Mat{}, cv::Mat{});
    return false;
  }

  cv::Mat raw_debug;
  cv::cvtColor(gray_eq, raw_debug, cv::COLOR_GRAY2BGR);
  cv::Mat filtered_debug;
  cv::cvtColor(gray_eq, filtered_debug, cv::COLOR_GRAY2BGR);

  const cv::Rect image_rect(0, 0, binary.cols, binary.rows);
  const int border_margin = std::max(
    std::max(0, params_.a4_border_margin_px),
    static_cast<int>(std::round(
      std::min(binary.cols, binary.rows) * std::max(0.0, params_.a4_border_margin_ratio))));
  cv::Rect content_rect = image_rect;
  if (2 * border_margin < binary.cols && 2 * border_margin < binary.rows) {
    content_rect = cv::Rect(
      border_margin,
      border_margin,
      binary.cols - 2 * border_margin,
      binary.rows - 2 * border_margin);
  }
  const bool has_content_margin =
    content_rect.x != image_rect.x ||
    content_rect.y != image_rect.y ||
    content_rect.width != image_rect.width ||
    content_rect.height != image_rect.height;
  if (params_.reject_border_candidates && has_content_margin) {
    cv::rectangle(filtered_debug, content_rect, cv::Scalar(255, 255, 0), 1);
  }

  std::vector<cv::Rect> boxes;
  boxes.reserve(contours.size());
  std::vector<cv::Rect> removed_by_edge;
  std::vector<cv::Rect> removed_by_line;
  std::vector<cv::Rect> removed_by_extent;
  struct CandidateRecord
  {
    cv::Rect rect;
    double area{0.0};
    double aspect{0.0};
    double extent{0.0};
    double cx_ratio{0.0};
    double cy_ratio{0.0};
    double score{0.0};
    int edge_touch_count{0};
    bool touches_opposite_edges{false};
    std::size_t debug_index{0};
  };
  std::vector<CandidateRecord> relaxed_candidates;

  result.stats.warped_width = binary.cols;
  result.stats.warped_height = binary.rows;
  result.stats.raw_contours = static_cast<int>(contours.size());
  const double image_area = static_cast<double>(std::max(1, binary.cols * binary.rows));
  const double min_area_threshold = std::max<double>(
    std::max(0, params_.min_digit_area),
    image_area * std::max(0.0, params_.min_digit_area_ratio));
  double max_area_threshold = image_area;
  if (params_.max_digit_area > 0 && params_.max_digit_area_ratio > 0.0) {
    max_area_threshold = std::max<double>(
      params_.max_digit_area,
      image_area * params_.max_digit_area_ratio);
  } else if (params_.max_digit_area > 0) {
    max_area_threshold = static_cast<double>(params_.max_digit_area);
  } else if (params_.max_digit_area_ratio > 0.0) {
    max_area_threshold = image_area * params_.max_digit_area_ratio;
  }
  max_area_threshold = std::clamp(max_area_threshold, min_area_threshold, image_area);

  const double min_width_threshold = std::max<double>(
    std::max(1, params_.min_digit_width),
    static_cast<double>(binary.cols) * std::max(0.0, params_.min_digit_width_ratio));
  double max_width_threshold = static_cast<double>(binary.cols);
  if (params_.max_digit_width > 0 && params_.max_digit_width_ratio > 0.0) {
    max_width_threshold = std::max<double>(
      params_.max_digit_width,
      static_cast<double>(binary.cols) * params_.max_digit_width_ratio);
  } else if (params_.max_digit_width > 0) {
    max_width_threshold = static_cast<double>(params_.max_digit_width);
  } else if (params_.max_digit_width_ratio > 0.0) {
    max_width_threshold = static_cast<double>(binary.cols) * params_.max_digit_width_ratio;
  }
  max_width_threshold = std::clamp(max_width_threshold, min_width_threshold, static_cast<double>(binary.cols));

  const double min_height_threshold = std::max<double>(
    std::max(1, params_.min_digit_height),
    static_cast<double>(binary.rows) * std::max(0.0, params_.min_digit_height_ratio));
  double max_height_threshold = static_cast<double>(binary.rows);
  if (params_.max_digit_height > 0 && params_.max_digit_height_ratio > 0.0) {
    max_height_threshold = std::max<double>(
      params_.max_digit_height,
      static_cast<double>(binary.rows) * params_.max_digit_height_ratio);
  } else if (params_.max_digit_height > 0) {
    max_height_threshold = static_cast<double>(params_.max_digit_height);
  } else if (params_.max_digit_height_ratio > 0.0) {
    max_height_threshold = static_cast<double>(binary.rows) * params_.max_digit_height_ratio;
  }
  max_height_threshold = std::clamp(max_height_threshold, min_height_threshold, static_cast<double>(binary.rows));

  const auto digit_score =
    [&image_area, &binary](const CandidateRecord & candidate) -> double {
      const double area_score = std::min(1.0, candidate.area / std::max(1.0, image_area * 0.08));
      const double height_score =
        std::min(1.0, static_cast<double>(candidate.rect.height) / std::max(1.0, binary.rows * 0.35));
      const double width_score =
        std::min(1.0, static_cast<double>(candidate.rect.width) / std::max(1.0, binary.cols * 0.25));
      const double extent_score = std::clamp(candidate.extent, 0.0, 1.0);
      const double aspect_center = 1.7;
      const double aspect_score =
        std::max(0.0, 1.0 - std::abs(candidate.aspect - aspect_center) / std::max(1.0, aspect_center));
      const double center_penalty =
        0.10 * (std::abs(candidate.cx_ratio - 0.5) + std::abs(candidate.cy_ratio - 0.5));
      const double edge_penalty = 0.08 * static_cast<double>(candidate.edge_touch_count);
      return 0.30 * height_score + 0.25 * width_score + 0.20 * area_score +
             0.15 * extent_score + 0.10 * aspect_score -
             center_penalty - edge_penalty;
    };

  for (const auto & contour : contours) {
    const double area = std::abs(cv::contourArea(contour));
    const cv::Rect rect = cv::boundingRect(contour);
    const double aspect = static_cast<double>(rect.height) / static_cast<double>(std::max(1, rect.width));
    const double extent = area / static_cast<double>(std::max(1, rect.area()));
    const double center_x_ratio =
      static_cast<double>(rect.x) + 0.5 * static_cast<double>(rect.width);
    const double center_y_ratio =
      static_cast<double>(rect.y) + 0.5 * static_cast<double>(rect.height);
    const double cx_ratio = center_x_ratio / static_cast<double>(std::max(1, binary.cols));
    const double cy_ratio = center_y_ratio / static_cast<double>(std::max(1, binary.rows));
    const int vertical_line_w = std::max(
      params_.max_vertical_line_width_px,
      static_cast<int>(std::round(binary.cols * std::max(0.0, params_.max_vertical_line_width_ratio))));
    const int horizontal_line_h = std::max(
      params_.max_horizontal_line_height_px,
      static_cast<int>(std::round(binary.rows * std::max(0.0, params_.max_horizontal_line_height_ratio))));

    bool rejected = false;
    std::string reject_reason;
    DigitCandidateDebugInfo debug;
    debug.bbox = rect;
    debug.area = area;
    debug.aspect_ratio = aspect;
    debug.extent = extent;
    debug.center_x_ratio = cx_ratio;
    debug.center_y_ratio = cy_ratio;

    CandidateRecord record{
      rect,
      area,
      aspect,
      extent,
      cx_ratio,
      cy_ratio,
      0.0,
      0,
      false,
      result.candidate_debug.size()};
    record.score = digit_score(record);
    debug.score = record.score;

    if (area < min_area_threshold) {
      rejected = true;
      reject_reason = "area_small";
      ++result.stats.rejected_area_count;
    } else if (area > max_area_threshold) {
      rejected = true;
      reject_reason = "area_large";
      ++result.stats.rejected_area_count;
    }
    if (!rejected) {
      debug.area_ok = true;
      ++result.stats.area_pass_count;
    }
    if (!rejected && rect.width < min_width_threshold) {
      rejected = true;
      reject_reason = "width_small";
      ++result.stats.rejected_size_count;
    } else if (!rejected && rect.height < min_height_threshold) {
      rejected = true;
      reject_reason = "height_small";
      ++result.stats.rejected_size_count;
    } else if (!rejected && rect.width > max_width_threshold) {
      rejected = true;
      reject_reason = "width_large";
      ++result.stats.rejected_size_count;
    } else if (!rejected && rect.height > max_height_threshold) {
      rejected = true;
      reject_reason = "height_large";
      ++result.stats.rejected_size_count;
    }
    if (!rejected && aspect < params_.min_digit_aspect_ratio) {
      rejected = true;
      reject_reason = "aspect_low";
      ++result.stats.rejected_aspect_count;
    } else if (!rejected && aspect > params_.max_digit_aspect_ratio) {
      rejected = true;
      reject_reason = "aspect_high";
      ++result.stats.rejected_aspect_count;
    }
    if (!rejected) {
      debug.shape_ok = true;
      ++result.stats.shape_pass_count;
      if (!result.has_best_shape_candidate || debug.score > result.best_shape_candidate.score) {
        result.best_shape_candidate = debug;
        result.has_best_shape_candidate = true;
      }
    }
    if (!rejected && extent < params_.min_candidate_extent) {
      rejected = true;
      reject_reason = "extent_low";
      ++result.stats.rejected_extent_count;
      removed_by_extent.push_back(rect);
    } else if (!rejected && extent > params_.max_candidate_extent) {
      rejected = true;
      reject_reason = "extent_high";
      ++result.stats.rejected_extent_count;
      removed_by_extent.push_back(rect);
    }
    const bool vertical_line =
      rect.width <= vertical_line_w &&
      aspect >= std::max(1.0, params_.min_vertical_line_aspect_ratio);
    const bool horizontal_line =
      rect.height <= horizontal_line_h &&
      aspect <= std::max(0.01, params_.max_horizontal_line_aspect_ratio);
    if (!rejected && (vertical_line || horizontal_line)) {
      rejected = true;
      reject_reason = vertical_line ? "vertical_line" : "horizontal_line";
      ++result.stats.rejected_line_count;
      removed_by_line.push_back(rect);
    }
    const bool edge_left = rect.x < content_rect.x;
    const bool edge_right = rect.x + rect.width - 1 >= content_rect.x + content_rect.width;
    const bool edge_top = rect.y < content_rect.y;
    const bool edge_bottom = rect.y + rect.height - 1 >= content_rect.y + content_rect.height;
    const bool edge_center_x =
      cx_ratio < params_.min_candidate_center_x_ratio ||
      cx_ratio > params_.max_candidate_center_x_ratio;
    const bool edge_center_y =
      cy_ratio < params_.min_candidate_center_y_ratio ||
      cy_ratio > params_.max_candidate_center_y_ratio;
    const int edge_touch_count =
      (edge_left ? 1 : 0) +
      (edge_right ? 1 : 0) +
      (edge_top ? 1 : 0) +
      (edge_bottom ? 1 : 0);
    const bool touches_opposite_edges =
      (edge_left && edge_right) || (edge_top && edge_bottom);
    record.edge_touch_count = edge_touch_count;
    record.touches_opposite_edges = touches_opposite_edges;
    record.score = digit_score(record);
    debug.score = record.score;
    const bool edge_candidate =
      params_.reject_border_candidates &&
      (edge_left || edge_right || edge_top || edge_bottom || edge_center_x || edge_center_y);
    const bool too_large_for_digit =
      rect.width > max_width_threshold ||
      rect.height > max_height_threshold ||
      area > max_area_threshold;
    const bool strong_digit_candidate =
      area >= min_area_threshold &&
      rect.width >= min_width_threshold &&
      rect.height >= min_height_threshold &&
      aspect >= params_.min_digit_aspect_ratio &&
      aspect <= params_.max_digit_aspect_ratio &&
      extent >= params_.min_candidate_extent &&
      extent <= params_.max_candidate_extent &&
      !too_large_for_digit;
    const bool allow_relaxed_edge_candidate =
      edge_candidate &&
      strong_digit_candidate &&
      !touches_opposite_edges &&
      !(edge_center_x && edge_center_y);

    if (!rejected && edge_candidate) {
      if (allow_relaxed_edge_candidate) {
        ++result.stats.relaxed_edge_pass_count;
      } else {
        rejected = true;
        reject_reason = "edge";
        if (edge_left) {
          reject_reason += "_left";
          ++result.stats.rejected_edge_left_count;
        }
        if (edge_right) {
          reject_reason += "_right";
          ++result.stats.rejected_edge_right_count;
        }
        if (edge_top) {
          reject_reason += "_top";
          ++result.stats.rejected_edge_top_count;
        }
        if (edge_bottom) {
          reject_reason += "_bottom";
          ++result.stats.rejected_edge_bottom_count;
        }
        if (edge_center_x) {
          reject_reason += "_center_x";
          ++result.stats.rejected_edge_center_x_count;
        }
        if (edge_center_y) {
          reject_reason += "_center_y";
          ++result.stats.rejected_edge_center_y_count;
        }
        ++result.stats.rejected_edge_count;
        removed_by_edge.push_back(rect);
      }
    }

    if (!rejected) {
      debug.edge_ok = true;
      ++result.stats.edge_pass_count;
    }
    if (params_.enable_relaxed_digit_fallback &&
      debug.shape_ok &&
      extent >= params_.min_candidate_extent * 0.5 &&
      extent <= std::min(1.0, params_.max_candidate_extent + 0.08) &&
      !vertical_line &&
      !horizontal_line &&
      !too_large_for_digit)
    {
      relaxed_candidates.push_back(record);
    }

    cv::rectangle(raw_debug, rect, rejected ? reject_color(reject_reason) : cv::Scalar(0, 200, 255), 2);
    if (rejected) {
      cv::putText(
        raw_debug,
        reject_reason,
        cv::Point(rect.x, std::max(0, rect.y - 2)),
        cv::FONT_HERSHEY_SIMPLEX,
        0.45,
        reject_color(reject_reason),
        1,
        cv::LINE_AA);
      debug.reject_reason = reject_reason;
      result.candidate_debug.push_back(debug);
      continue;
    }

    debug.reject_reason = "accepted";
    debug.relative_ok = true;
    result.candidate_debug.push_back(debug);

    boxes.push_back(rect);
  }

  if (boxes.empty() && params_.enable_relaxed_digit_fallback && !relaxed_candidates.empty()) {
    const auto best = std::max_element(
      relaxed_candidates.begin(), relaxed_candidates.end(),
      [](const CandidateRecord & a, const CandidateRecord & b) {
        return a.score < b.score;
      });
    if (best != relaxed_candidates.end() && best->score >= params_.relaxed_fallback_min_score) {
      boxes.push_back(best->rect);
      ++result.stats.edge_pass_count;
      ++result.stats.relaxed_edge_pass_count;
      result.stats.relaxed_fallback_used = true;
      result.relaxed_fallback_box = best->rect;
      if (best->debug_index < result.candidate_debug.size()) {
        result.candidate_debug[best->debug_index].reject_reason = "relaxed_fallback";
        result.candidate_debug[best->debug_index].edge_ok = true;
        result.candidate_debug[best->debug_index].relative_ok = true;
      }
    }
  }

  if (boxes.empty()) {
    result.error_message = "轮廓均被候选框阈值过滤";
    result.final_zero_reason = "all_candidates_rejected";
    cv::putText(
      filtered_debug,
      "final=0 " + result.final_zero_reason,
      cv::Point(8, std::min(filtered_debug.rows - 8, 48)),
      cv::FONT_HERSHEY_SIMPLEX,
      0.55,
      cv::Scalar(0, 0, 255),
      1,
      cv::LINE_AA);
    result.binary_debug = binary;
    result.boxed_debug = make_panel(binary, raw_debug, filtered_debug, cv::Mat{});
    return false;
  }

  // 相对最大候选的面积/高度过滤，采用“弱约束”策略，避免误杀细数字（例如 1）。
  double largest_area = 1.0;
  int largest_height = 1;
  int largest_width = 1;
  for (const auto & box : boxes) {
    largest_area = std::max(largest_area, static_cast<double>(box.area()));
    largest_height = std::max(largest_height, box.height);
    largest_width = std::max(largest_width, box.width);
  }

  std::vector<cv::Rect> filtered;
  filtered.reserve(boxes.size());
  std::vector<cv::Rect> removed_by_relative;
  for (const auto & box : boxes) {
    const double area_ratio = static_cast<double>(box.area()) / largest_area;
    const double height_ratio = static_cast<double>(box.height) / static_cast<double>(largest_height);
    const double width_ratio = static_cast<double>(box.width) / static_cast<double>(largest_width);

    bool keep_relative = true;
    if (params_.enable_relative_size_filter) {
      if (params_.relative_filter_use_or) {
        keep_relative =
          area_ratio >= std::max(0.01, params_.min_area_ratio_to_largest) ||
          height_ratio >= std::max(0.05, params_.min_height_ratio_to_largest);
      } else {
        keep_relative =
          area_ratio >= std::max(0.01, params_.min_area_ratio_to_largest) &&
          height_ratio >= std::max(0.05, params_.min_height_ratio_to_largest);
      }
    }

    const bool keep_slender =
      params_.enable_slender_digit_protection &&
      height_ratio >= std::max(0.20, params_.slender_min_height_ratio_to_largest) &&
      width_ratio <= std::max(0.05, params_.slender_max_width_ratio_to_largest);

    if (keep_relative || keep_slender) {
      filtered.push_back(box);
    } else {
      removed_by_relative.push_back(box);
    }
  }

  if (!filtered.empty()) {
    for (const auto & removed : removed_by_relative) {
      for (auto & debug : result.candidate_debug) {
        if (debug.bbox == removed && debug.reject_reason == "accepted") {
          debug.reject_reason = "relative_to_largest";
          debug.relative_ok = false;
          break;
        }
      }
    }
    boxes.swap(filtered);
  } else {
    removed_by_relative.clear();
  }
  result.stats.relative_pass_count = static_cast<int>(boxes.size());

  for (const auto & b : removed_by_edge) {
    cv::rectangle(filtered_debug, b, cv::Scalar(255, 0, 255), 2);
  }
  for (const auto & b : removed_by_line) {
    cv::rectangle(filtered_debug, b, cv::Scalar(255, 80, 0), 2);
  }
  for (const auto & b : removed_by_extent) {
    cv::rectangle(filtered_debug, b, cv::Scalar(180, 0, 255), 1);
  }
  for (const auto & b : boxes) {
    cv::rectangle(filtered_debug, b, cv::Scalar(0, 255, 0), 2);
  }
  for (const auto & b : removed_by_relative) {
    cv::rectangle(filtered_debug, b, cv::Scalar(0, 0, 255), 1);
  }

  std::sort(
    boxes.begin(), boxes.end(),
    [](const cv::Rect & a, const cv::Rect & b) {
      if (a.x == b.x) {
        return a.area() > b.area();
      }
      return a.x < b.x;
    });

  const auto horizontal_overlap_ratio = [](const cv::Rect & a, const cv::Rect & b) -> double {
    const int left = std::max(a.x, b.x);
    const int right = std::min(a.x + a.width, b.x + b.width);
    const int overlap = std::max(0, right - left);
    const int min_w = std::max(1, std::min(a.width, b.width));
    return static_cast<double>(overlap) / static_cast<double>(min_w);
  };

  const auto vertical_gap = [](const cv::Rect & a, const cv::Rect & b) -> int {
    const int top = std::max(a.y, b.y);
    const int bottom = std::min(a.y + a.height, b.y + b.height);
    if (bottom >= top) {
      return 0;
    }
    return top - bottom;
  };

  const auto x_gap_between = [](const cv::Rect & a, const cv::Rect & b) -> int {
    const int gap_ab = b.x - (a.x + a.width);
    const int gap_ba = a.x - (b.x + b.width);
    return std::max(0, std::max(gap_ab, gap_ba));
  };

  const auto rect_ink_extent = [&binary](const cv::Rect & r) -> double {
    const cv::Rect bounded = r & cv::Rect(0, 0, binary.cols, binary.rows);
    if (bounded.empty()) {
      return 0.0;
    }
    return static_cast<double>(cv::countNonZero(binary(bounded))) /
           static_cast<double>(std::max(1, bounded.area()));
  };

  const int dynamic_merge_gap = std::max(
    std::max(0, params_.merge_gap_px),
    static_cast<int>(std::round(binary.cols * std::max(0.0, params_.merge_gap_ratio))));
  const int dynamic_vertical_gap = std::max(
    std::max(0, params_.vertical_merge_gap_px),
    static_cast<int>(std::round(binary.rows * std::max(0.0, params_.merge_gap_ratio))));
  const double y_overlap_required = std::max(
    0.05,
    std::min(
      std::max(0.0, params_.merge_min_y_overlap_ratio),
      std::max(0.0, params_.merge_vertical_overlap_min_ratio)));

  const auto should_merge_digit_fragments =
    [&](const cv::Rect & a, const cv::Rect & b) -> bool {
      const cv::Rect combined = a | b;
      const double combined_aspect =
        static_cast<double>(combined.height) / static_cast<double>(std::max(1, combined.width));
      const double combined_extent = rect_ink_extent(combined);
      const int x_gap = x_gap_between(a, b);
      const int y_gap = vertical_gap(a, b);
      const double y_overlap = has_vertical_overlap(a, b, 0.01) ?
        static_cast<double>(
          std::max(0, std::min(a.y + a.height, b.y + b.height) - std::max(a.y, b.y))) /
        static_cast<double>(std::max(1, std::min(a.height, b.height))) : 0.0;
      const double x_overlap = horizontal_overlap_ratio(a, b);
      const int height_diff = std::abs(a.height - b.height);
      const double height_diff_ratio =
        static_cast<double>(height_diff) / static_cast<double>(std::max(1, std::max(a.height, b.height)));
      const bool close_x = x_gap <= dynamic_merge_gap || x_overlap >= 0.15;
      const bool close_y = y_gap <= dynamic_vertical_gap || y_overlap >= y_overlap_required;
      const bool height_compatible =
        height_diff <= std::max(0, params_.merge_max_height_diff_px) ||
        height_diff_ratio <= std::max(0.05, params_.merge_max_height_diff_ratio) ||
        std::min(a.height, b.height) <= std::max(2, static_cast<int>(0.45 * std::max(a.height, b.height)));
      const bool combined_shape_ok =
        combined_aspect >= std::max(0.10, params_.min_digit_aspect_ratio * 0.60) &&
        combined_aspect <= params_.max_digit_aspect_ratio &&
        combined_extent >= std::max(0.01, params_.min_candidate_extent * 0.35) &&
        combined_extent <= std::min(1.0, params_.max_candidate_extent + 0.10) &&
        combined.width <= max_width_threshold &&
        combined.height <= max_height_threshold &&
        static_cast<double>(combined.area()) <= max_area_threshold;
      return close_x && close_y && height_compatible && combined_shape_ok;
    };

  std::vector<cv::Rect> merged = boxes;
  bool changed = true;
  int merge_pass = 0;
  while (changed && merge_pass < 8) {
    changed = false;
    ++merge_pass;
    for (std::size_t i = 0; i < merged.size() && !changed; ++i) {
      for (std::size_t j = i + 1; j < merged.size(); ++j) {
        if (should_merge_digit_fragments(merged[i], merged[j])) {
          merged[i] = merged[i] | merged[j];
          merged.erase(merged.begin() + static_cast<std::ptrdiff_t>(j));
          changed = true;
          break;
        }
      }
    }
  }

  if (params_.enable_relaxed_digit_fallback && merged.size() > 1U) {
    cv::Rect single_union = merged.front();
    for (std::size_t i = 1; i < merged.size(); ++i) {
      single_union |= merged[i];
    }
    const double width_height_ratio =
      static_cast<double>(single_union.width) / static_cast<double>(std::max(1, single_union.height));
    const double union_extent = rect_ink_extent(single_union);
    if (width_height_ratio < std::max(0.65, params_.union_split_min_width_height_ratio * 0.85) &&
      union_extent >= std::max(0.01, params_.min_candidate_extent * 0.35) &&
      union_extent <= std::min(1.0, params_.max_candidate_extent + 0.10) &&
      single_union.width <= max_width_threshold &&
      single_union.height <= max_height_threshold &&
      static_cast<double>(single_union.area()) <= max_area_threshold)
    {
      merged.clear();
      merged.push_back(single_union);
      result.stats.relaxed_fallback_used = true;
      result.relaxed_fallback_box = single_union;
    }
  }

  if (static_cast<int>(merged.size()) > params_.max_digit_count) {
    const double largest_merged_area = static_cast<double>(std::max_element(
      merged.begin(), merged.end(),
      [](const cv::Rect & a, const cv::Rect & b) { return a.area() < b.area(); })->area());
    const int largest_merged_height = std::max_element(
      merged.begin(), merged.end(),
      [](const cv::Rect & a, const cv::Rect & b) { return a.height < b.height; })->height;
    std::sort(
      merged.begin(), merged.end(),
      [largest_merged_area, largest_merged_height, &content_rect](const cv::Rect & a, const cv::Rect & b) {
        const auto score = [largest_merged_area, largest_merged_height, &content_rect](const cv::Rect & r) {
          const double area_score = static_cast<double>(r.area()) / std::max(1.0, largest_merged_area);
          const double height_score = static_cast<double>(r.height) / static_cast<double>(std::max(1, largest_merged_height));
          const double cx = static_cast<double>(r.x + r.width / 2 - (content_rect.x + content_rect.width / 2));
          const double center_penalty = std::abs(cx) / static_cast<double>(std::max(1, content_rect.width));
          return 0.55 * height_score + 0.35 * area_score - 0.20 * center_penalty;
        };
        return score(a) > score(b);
      });
    merged.resize(static_cast<std::size_t>(params_.max_digit_count));
    std::sort(
      merged.begin(), merged.end(),
      [](const cv::Rect & a, const cv::Rect & b) { return a.x < b.x; });
  }

  cv::Mat boxed;
  cv::cvtColor(gray_eq, boxed, cv::COLOR_GRAY2BGR);

  for (std::size_t i = 0; i < merged.size(); ++i) {
    const int crop_pad = std::max(
      std::max(0, params_.crop_padding_px),
      static_cast<int>(std::round(
        std::max(merged[i].width, merged[i].height) * std::max(0.0, params_.crop_padding_ratio))));
    cv::Rect box = expand_rect(merged[i], crop_pad, content_rect);
    if (box.width <= 0 || box.height <= 0) {
      continue;
    }

    const cv::Mat digit_bin = binary(box).clone();
    cv::Mat digit_norm = normalize_digit_to_canvas(digit_bin, params_.digit_input_size);
    if (digit_norm.empty()) {
      continue;
    }

    result.digits.push_back(DigitCandidate{box, digit_norm});
    for (auto & debug : result.candidate_debug) {
      const cv::Rect overlap = debug.bbox & merged[i];
      if (!overlap.empty() && overlap.area() > 0) {
        debug.final_candidate = true;
        if (debug.reject_reason == "accepted") {
          debug.reject_reason = "final";
        }
      }
    }

    cv::rectangle(boxed, box, cv::Scalar(0, 255, 0), 2);
    cv::putText(
      boxed,
      std::to_string(i),
      cv::Point(box.x, std::max(0, box.y - 4)),
      cv::FONT_HERSHEY_SIMPLEX,
      0.5,
      cv::Scalar(0, 255, 0),
      1,
      cv::LINE_AA);
  }

  if (result.digits.empty()) {
    result.error_message = "数字归一化失败";
    result.final_zero_reason = "normalization_failed";
    cv::putText(
      boxed,
      "final=0 " + result.final_zero_reason,
      cv::Point(8, std::min(boxed.rows - 8, 48)),
      cv::FONT_HERSHEY_SIMPLEX,
      0.55,
      cv::Scalar(0, 0, 255),
      1,
      cv::LINE_AA);
    result.binary_debug = binary;
    result.boxed_debug = make_panel(binary, raw_debug, filtered_debug, boxed);
    return false;
  }

  cv::Rect union_box = result.digits.front().bbox;
  for (std::size_t i = 1; i < result.digits.size(); ++i) {
    union_box |= result.digits[i].bbox;
  }
  const int union_pad = std::max(
    std::max(0, params_.union_fallback_padding_px),
    static_cast<int>(std::round(
      std::max(union_box.width, union_box.height) * std::max(0.0, params_.union_fallback_padding_ratio))));
  result.union_fallback_box = expand_rect(union_box, union_pad, content_rect);
  result.stats.union_fallback_box_valid =
    result.union_fallback_box.width > 0 && result.union_fallback_box.height > 0;
  if (result.stats.union_fallback_box_valid) {
    cv::rectangle(boxed, result.union_fallback_box, cv::Scalar(255, 255, 0), 2);
    cv::putText(
      boxed,
      "union",
      cv::Point(result.union_fallback_box.x, std::max(0, result.union_fallback_box.y - 4)),
      cv::FONT_HERSHEY_SIMPLEX,
      0.5,
      cv::Scalar(255, 255, 0),
      1,
      cv::LINE_AA);
  }
  if (!result.relaxed_fallback_box.empty()) {
    cv::rectangle(boxed, result.relaxed_fallback_box, cv::Scalar(0, 180, 255), 2);
    cv::putText(
      boxed,
      "relaxed",
      cv::Point(result.relaxed_fallback_box.x, std::min(
        boxed.rows - 4, result.relaxed_fallback_box.y + result.relaxed_fallback_box.height + 16)),
      cv::FONT_HERSHEY_SIMPLEX,
      0.5,
      cv::Scalar(0, 180, 255),
      1,
      cv::LINE_AA);
  }
  result.stats.final_count = static_cast<int>(result.digits.size());

  // 构建数字拼接调试图。
  const int tile = std::max(16, params_.digit_input_size);
  const int margin = 6;
  const int width = static_cast<int>(result.digits.size()) * (tile + margin) + margin;
  const int height = tile + 2 * margin + 16;
  cv::Mat montage(height, width, CV_8UC1, cv::Scalar(230));

  for (std::size_t i = 0; i < result.digits.size(); ++i) {
    const int x = margin + static_cast<int>(i) * (tile + margin);
    const int y = margin;
    cv::Mat roi = montage(cv::Rect(x, y, tile, tile));
    result.digits[i].digit_gray.copyTo(roi);
    cv::putText(
      montage,
      result.stats.relaxed_fallback_used ? "relaxed" : "normal",
      cv::Point(x, tile + margin + 12),
      cv::FONT_HERSHEY_SIMPLEX,
      0.38,
      cv::Scalar(20),
      1,
      cv::LINE_AA);
  }

  result.success = true;
  result.binary_debug = binary;
  result.boxed_debug = make_panel(binary, raw_debug, filtered_debug, boxed);
  result.montage_debug = montage;
  return true;
}

bool DigitSegmenter::has_vertical_overlap(const cv::Rect & a, const cv::Rect & b, double min_ratio)
{
  const int top = std::max(a.y, b.y);
  const int bottom = std::min(a.y + a.height, b.y + b.height);
  const int overlap = std::max(0, bottom - top);
  const int min_h = std::max(1, std::min(a.height, b.height));
  const double ratio = static_cast<double>(overlap) / static_cast<double>(min_h);
  return ratio >= min_ratio;
}

cv::Mat DigitSegmenter::normalize_digit_to_canvas(const cv::Mat & binary_digit, int canvas_size)
{
  if (binary_digit.empty() || canvas_size <= 0) {
    return {};
  }

  cv::Mat mask;
  if (binary_digit.type() != CV_8UC1) {
    cv::cvtColor(binary_digit, mask, cv::COLOR_BGR2GRAY);
  } else {
    mask = binary_digit.clone();
  }
  cv::threshold(mask, mask, 0, 255, cv::THRESH_BINARY);

  std::vector<cv::Point> pts;
  cv::findNonZero(mask, pts);
  if (pts.empty()) {
    return {};
  }

  const cv::Rect bb = cv::boundingRect(pts);
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

  cv::Mat roi = canvas(cv::Rect(x, y, w, h));
  roi.setTo(0, resized);
  return canvas;
}

}  // namespace wheeltec_inventory_system
