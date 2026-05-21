#ifndef WHEELTEC_INVENTORY_SYSTEM__DIGIT_SEGMENTER_HPP_
#define WHEELTEC_INVENTORY_SYSTEM__DIGIT_SEGMENTER_HPP_

#include <string>
#include <vector>

#include "opencv2/core.hpp"

namespace agv_inventory_system
{

/**
 * @brief 数字分割参数。
 */
struct DigitSegmenterParams
{
  double clahe_clip_limit{2.0};
  int clahe_grid_size{8};
  int pre_blur_kernel{3};
  int median_blur_kernel{3};
  int adaptive_thresh_block_size{11};
  int adaptive_thresh_c{2};
  bool enable_otsu_fusion{true};
  bool binary_cleanup_enabled{true};
  int binary_cleanup_min_component_area{20};
  int binary_cleanup_min_component_width{3};
  int binary_cleanup_min_component_height{3};
  int morph_kernel_size{3};
  int morph_open_kernel_size{1};
  int morph_close_kernel_size{3};

  int min_digit_area{50};
  int max_digit_area{5000};
  double min_digit_area_ratio{0.0001};
  double max_digit_area_ratio{0.35};
  int min_digit_width{6};
  int min_digit_height{10};
  int max_digit_width{0};
  int max_digit_height{0};
  double min_digit_width_ratio{0.01};
  double max_digit_width_ratio{0.80};
  double min_digit_height_ratio{0.025};
  double max_digit_height_ratio{0.90};
  double min_digit_aspect_ratio{0.20};
  double max_digit_aspect_ratio{12.0};
  double min_candidate_extent{0.06};
  double max_candidate_extent{0.92};
  bool reject_border_candidates{true};
  int a4_border_margin_px{6};
  double a4_border_margin_ratio{0.015};
  double min_candidate_center_x_ratio{0.03};
  double max_candidate_center_x_ratio{0.97};
  double min_candidate_center_y_ratio{0.05};
  double max_candidate_center_y_ratio{0.95};
  int max_vertical_line_width_px{6};
  double max_vertical_line_width_ratio{0.035};
  double min_vertical_line_aspect_ratio{8.0};
  int max_horizontal_line_height_px{6};
  double max_horizontal_line_height_ratio{0.035};
  double max_horizontal_line_aspect_ratio{0.12};
  int crop_padding_px{4};
  double crop_padding_ratio{0.025};
  int union_fallback_padding_px{8};
  double union_fallback_padding_ratio{0.04};
  bool enable_union_split{true};
  double union_split_min_width_height_ratio{0.85};
  double union_split_min_valley_ratio{0.30};
  bool enable_relaxed_digit_fallback{true};
  double relaxed_fallback_min_score{0.20};

  int digit_input_size{28};
  int max_digit_count{4};
  int merge_gap_px{8};
  double merge_gap_ratio{0.04};
  bool enable_relative_size_filter{true};
  bool relative_filter_use_or{true};
  // 相对最大候选的面积/高度过滤阈值（弱约束，避免误杀细数字）。
  double min_area_ratio_to_largest{0.25};
  double min_height_ratio_to_largest{0.35};
  bool enable_slender_digit_protection{true};
  double slender_min_height_ratio_to_largest{0.50};
  double slender_max_width_ratio_to_largest{0.45};
  // 处理同一数字被上下切裂时的纵向合并参数。
  bool enable_horizontal_merge{true};
  bool enable_vertical_split_merge{true};
  double merge_vertical_overlap_min_ratio{0.45};
  double merge_min_y_overlap_ratio{0.30};
  double merge_small_part_area_ratio_max{0.45};
  // 仅当“小碎片”高度显著小于主框时才允许横向合并，避免把独立数字（如 1 和 5）合并。
  double merge_small_part_height_ratio_max{0.72};
  int merge_max_height_diff_px{18};
  double merge_max_height_diff_ratio{0.45};
  int vertical_merge_gap_px{10};
  double min_horizontal_overlap_ratio_for_vertical_merge{0.55};
};

/**
 * @brief 单个数字候选。
 */
struct DigitCandidate
{
  cv::Rect bbox;             // 在 A4 矫正图中的包围框
  cv::Mat digit_gray;  // 统一输入尺寸灰度图（白底黑字）
};

struct DigitSegmentationStats
{
  int warped_width{0};
  int warped_height{0};
  int raw_contours{0};
  int area_pass_count{0};
  int shape_pass_count{0};
  int edge_pass_count{0};
  int relative_pass_count{0};
  int final_count{0};
  int rejected_area_count{0};
  int rejected_size_count{0};
  int rejected_aspect_count{0};
  int rejected_extent_count{0};
  int rejected_line_count{0};
  int rejected_edge_count{0};
  int rejected_edge_left_count{0};
  int rejected_edge_right_count{0};
  int rejected_edge_top_count{0};
  int rejected_edge_bottom_count{0};
  int rejected_edge_center_x_count{0};
  int rejected_edge_center_y_count{0};
  int relaxed_edge_pass_count{0};
  bool relaxed_fallback_used{false};
  bool union_fallback_box_valid{false};
};

struct DigitCandidateDebugInfo
{
  cv::Rect bbox;
  double area{0.0};
  double aspect_ratio{0.0};
  double extent{0.0};
  double center_x_ratio{0.0};
  double center_y_ratio{0.0};
  double score{0.0};
  std::string reject_reason;
  bool area_ok{false};
  bool shape_ok{false};
  bool edge_ok{false};
  bool relative_ok{false};
  bool final_candidate{false};
};

/**
 * @brief 数字分割结果。
 */
struct DigitSegmentationResult
{
  bool success{false};
  std::string error_message;

  // 按 x 从小到大排序后的数字候选。
  std::vector<DigitCandidate> digits;

  // 调试图：二值图 + 候选框 + 拼接结果。
  cv::Mat binary_debug;
  cv::Mat boxed_debug;
  cv::Mat montage_debug;
  cv::Rect union_fallback_box;
  cv::Rect relaxed_fallback_box;
  DigitCandidateDebugInfo best_shape_candidate;
  bool has_best_shape_candidate{false};
  std::string final_zero_reason;
  std::vector<DigitCandidateDebugInfo> candidate_debug;
  DigitSegmentationStats stats;
};

/**
 * @brief A4 矫正图中的数字区域分割器。
 */
class DigitSegmenter
{
public:
  explicit DigitSegmenter(const DigitSegmenterParams & params = DigitSegmenterParams());

  /**
   * @brief 更新参数。
   */
  void set_params(const DigitSegmenterParams & params);

  /**
   * @brief 获取当前参数。
   */
  const DigitSegmenterParams & params() const;

  /**
   * @brief 从矫正后的 A4 图像中切割数字。
   * @param a4_bgr 透视矫正后的 BGR 图。
   * @param result 输出结果。
   * @return true 表示分割成功。
   */
  bool segment(const cv::Mat & a4_bgr, DigitSegmentationResult & result) const;

private:
  static bool has_vertical_overlap(const cv::Rect & a, const cv::Rect & b, double min_ratio);
  static cv::Mat normalize_digit_to_canvas(const cv::Mat & binary_digit, int canvas_size);

  DigitSegmenterParams params_;
};

}  // namespace agv_inventory_system

#endif  // WHEELTEC_INVENTORY_SYSTEM__DIGIT_SEGMENTER_HPP_
