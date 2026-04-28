#ifndef WHEELTEC_INVENTORY_SYSTEM__A4_DETECTOR_HPP_
#define WHEELTEC_INVENTORY_SYSTEM__A4_DETECTOR_HPP_

#include <array>
#include <string>
#include <vector>

#include "opencv2/core.hpp"

namespace wheeltec_inventory_system
{

/**
 * @brief A4 检测器参数。
 */
struct A4DetectorParams
{
  int gaussian_kernel{5};
  int morph_kernel_size{5};
  double min_area_ratio{0.01};
  double approx_epsilon_ratio{0.02};
  double min_aspect_ratio{0.55};
  double max_aspect_ratio{2.20};
};

/**
 * @brief A4 检测输出结果。
 */
struct A4DetectionResult
{
  bool success{false};
  std::string error_message;
  double area_ratio{0.0};

  // 原图中四边形顶点（顺序：左上、右上、右下、左下）
  std::array<cv::Point2f, 4> corners{};

  // 原图中的外接矩形，仅用于可视化。
  cv::Rect bbox;

  // 透视矫正后的 A4 区域（BGR）
  cv::Mat warped_bgr;

  // 透视变换矩阵：原图 -> 矫正图
  cv::Mat warp_matrix;

  // 逆透视矩阵：矫正图 -> 原图
  cv::Mat inverse_warp_matrix;

  // 调试图（原图上绘制候选和最终四边形）
  cv::Mat debug_image;
};

/**
 * @brief A4 白纸检测与透视矫正。
 */
class A4Detector
{
public:
  explicit A4Detector(const A4DetectorParams & params = A4DetectorParams());

  /**
   * @brief 更新参数。
   */
  void set_params(const A4DetectorParams & params);

  /**
   * @brief 获取当前参数。
   */
  const A4DetectorParams & params() const;

  /**
   * @brief 在输入图像中检测 A4 纸并完成透视矫正。
   * @param bgr 输入 BGR 图像。
   * @param result 输出检测结果。
   * @return true 表示检测成功。
   */
  bool detect(const cv::Mat & bgr, A4DetectionResult & result) const;

private:
  static std::array<cv::Point2f, 4> order_corners(const std::vector<cv::Point> & contour);
  static double point_distance(const cv::Point2f & a, const cv::Point2f & b);

  A4DetectorParams params_;
};

}  // namespace wheeltec_inventory_system

#endif  // WHEELTEC_INVENTORY_SYSTEM__A4_DETECTOR_HPP_
