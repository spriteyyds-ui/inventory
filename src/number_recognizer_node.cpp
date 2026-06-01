#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "agv_inventory_system/a4_detector.hpp"
#include "agv_inventory_system/circle_marker_recognizer.hpp"
#include "agv_inventory_system/digit_classifier.hpp"
#include "agv_inventory_system/digit_segmenter.hpp"
#include "agv_inventory_system/id_utils.hpp"
#include "agv_inventory_system/msg/recognized_number.hpp"
#include "yaml-cpp/yaml.h"

class NumberRecognizerNode : public rclcpp::Node
{
public:
  NumberRecognizerNode()
  : Node("number_recognizer_node")
  {
    declare_all_parameters();
    load_parameters();
    // 统一使用节点当前时钟源初始化，避免 ROS time / system time 混用相减导致崩溃。
    last_attempt_time_ = this->now() - rclcpp::Duration::from_seconds(attempt_interval_);
    latest_tracking_distance_stamp_ = this->now() - rclcpp::Duration::from_seconds(3600.0);

    recog_pub_ = create_publisher<agv_inventory_system::msg::RecognizedNumber>(
      recognized_topic_, 10);
    debug_a4_pub_ = create_publisher<sensor_msgs::msg::Image>(debug_a4_topic_, 10);
    debug_digits_pub_ = create_publisher<sensor_msgs::msg::Image>(debug_digits_topic_, 10);
    vis_pub_ = create_publisher<sensor_msgs::msg::Image>(visualization_topic_, 10);

    if (!load_cabinet_entry_side_map()) {
      RCLCPP_WARN(
        get_logger(),
        "cabinet_entry_side_map 加载失败，识别节点将使用 fallback camera_topic=%s",
        camera_topic_.c_str());
    }
    set_image_subscription(camera_topic_, -1, "fallback", true);

    trigger_srv_ = create_service<std_srvs::srv::SetBool>(
      trigger_service_name_,
      [this](
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        trigger_callback(request, response);
      });

    auto control_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      enable_control_topic_,
      control_qos,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        enable_control_callback(msg);
      });

    target_cabinet_sub_ = create_subscription<std_msgs::msg::Int32>(
      target_cabinet_topic_,
      control_qos,
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        target_cabinet_callback(msg);
      });

    distance_sub_ = create_subscription<std_msgs::msg::Float32>(
      distance_overlay_topic_,
      10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        if (!msg) {
          return;
        }
        latest_tracking_distance_ = static_cast<double>(msg->data);
        latest_tracking_distance_stamp_ = this->now();
      });

    param_cb_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        return on_parameters_set(params);
      });

    if (!reload_classifier_model()) {
      RCLCPP_ERROR(get_logger(), "分类模型加载失败，节点将继续运行但识别无效");
    }

    RCLCPP_INFO(get_logger(), "数字识别节点已启动");
    print_parameter_summary();
  }

private:
  void declare_all_parameters()
  {
    camera_topic_ = declare_parameter<std::string>("camera_topic", "/camera/color/image_raw");
    left_camera_topic_ = declare_parameter<std::string>(
      "left_camera_topic", "/c100_left/image_raw");
    right_camera_topic_ = declare_parameter<std::string>(
      "right_camera_topic", "/c100_right/image_raw");
    target_cabinet_topic_ = declare_parameter<std::string>(
      "target_cabinet_topic", "/inventory/current_target_cabinet");
    route_waypoints_file_ = declare_parameter<std::string>(
      "route_waypoints_file", "config/route_waypoints.yaml");
    recognized_topic_ = declare_parameter<std::string>(
      "recognized_topic", "/inventory/recognized_number");
    debug_a4_topic_ = declare_parameter<std::string>(
      "debug_a4_topic", "/inventory/debug_a4_region");
    debug_digits_topic_ = declare_parameter<std::string>(
      "debug_digits_topic", "/inventory/debug_digits");
    visualization_topic_ = declare_parameter<std::string>(
      "visualization_topic", "/inventory/visualization");
    trigger_service_name_ = declare_parameter<std::string>(
      "trigger_service_name", "/inventory/trigger_recognition");
    enable_control_topic_ = declare_parameter<std::string>(
      "enable_control_topic", "/inventory/recognizer_enable");
    distance_overlay_topic_ = declare_parameter<std::string>(
      "distance_overlay_topic", "/inventory/target_distance");

    recognition_target_style_ = declare_parameter<std::string>(
      "recognition_target_style", "circle_marker");
    enable_on_start_ = declare_parameter<bool>("enable_on_start", true);
    min_confidence_ = declare_parameter<double>("min_confidence", 0.70);
    max_attempts_ = declare_parameter<int>("max_attempts", 20);
    attempt_interval_ = declare_parameter<double>("attempt_interval", 0.1);
    distance_overlay_timeout_sec_ = declare_parameter<double>("distance_overlay_timeout_sec", 0.8);
    enable_union_fallback_classification_ =
      declare_parameter<bool>("enable_union_fallback_classification", true);
    circle_marker_debug_log_enabled_ =
      declare_parameter<bool>("circle_marker_debug_log_enabled", true);

    cabinet_id_min_ = declare_parameter<int>("cabinet_id_min", 1);
    cabinet_id_max_ = declare_parameter<int>("cabinet_id_max", 36);

    known_digit_height_px_ = declare_parameter<double>("known_digit_height_px", 40.0);
    known_distance_m_ = declare_parameter<double>("known_distance_m", 1.0);
    distance_focal_length_ = declare_parameter<double>("distance_focal_length", 600.0);

    a4_params_.gaussian_kernel = declare_parameter<int>("a4_gaussian_kernel", 5);
    a4_params_.morph_kernel_size = declare_parameter<int>("a4_morph_kernel_size", 5);
    a4_params_.min_area_ratio = declare_parameter<double>("a4_min_area_ratio", 0.01);
    a4_params_.approx_epsilon_ratio = declare_parameter<double>("a4_approx_epsilon", 0.02);
    a4_params_.min_aspect_ratio = declare_parameter<double>("a4_min_aspect_ratio", 0.55);
    a4_params_.max_aspect_ratio = declare_parameter<double>("a4_max_aspect_ratio", 2.20);

    seg_params_.clahe_clip_limit = declare_parameter<double>("clahe_clip_limit", 2.0);
    seg_params_.clahe_grid_size = declare_parameter<int>("clahe_grid_size", 8);
    seg_params_.pre_blur_kernel = declare_parameter<int>("pre_blur_kernel", 3);
    seg_params_.median_blur_kernel = declare_parameter<int>("median_blur_kernel", 3);
    seg_params_.adaptive_thresh_block_size = declare_parameter<int>("adaptive_thresh_block_size", 11);
    seg_params_.adaptive_thresh_c = declare_parameter<int>("adaptive_thresh_c", 2);
    seg_params_.enable_otsu_fusion = declare_parameter<bool>("enable_otsu_fusion", true);
    seg_params_.binary_cleanup_enabled = declare_parameter<bool>("binary_cleanup_enabled", true);
    seg_params_.binary_cleanup_min_component_area =
      declare_parameter<int>("binary_cleanup_min_component_area", 20);
    seg_params_.binary_cleanup_min_component_width =
      declare_parameter<int>("binary_cleanup_min_component_width", 3);
    seg_params_.binary_cleanup_min_component_height =
      declare_parameter<int>("binary_cleanup_min_component_height", 3);
    seg_params_.morph_kernel_size = declare_parameter<int>("morph_kernel_size", 3);
    seg_params_.morph_open_kernel_size = declare_parameter<int>("morph_open_kernel_size", 1);
    seg_params_.morph_close_kernel_size = declare_parameter<int>("morph_close_kernel_size", 3);
    seg_params_.min_digit_area = declare_parameter<int>("min_digit_area", 50);
    seg_params_.max_digit_area = declare_parameter<int>("max_digit_area", 5000);
    seg_params_.min_digit_area_ratio = declare_parameter<double>("min_digit_area_ratio", 0.0001);
    seg_params_.max_digit_area_ratio = declare_parameter<double>("max_digit_area_ratio", 0.35);
    seg_params_.min_digit_width = declare_parameter<int>("min_digit_width", 6);
    seg_params_.min_digit_height = declare_parameter<int>("min_digit_height", 10);
    seg_params_.max_digit_width = declare_parameter<int>("max_digit_width", 0);
    seg_params_.max_digit_height = declare_parameter<int>("max_digit_height", 0);
    seg_params_.min_digit_width_ratio = declare_parameter<double>("min_digit_width_ratio", 0.01);
    seg_params_.max_digit_width_ratio = declare_parameter<double>("max_digit_width_ratio", 0.80);
    seg_params_.min_digit_height_ratio = declare_parameter<double>("min_digit_height_ratio", 0.025);
    seg_params_.max_digit_height_ratio = declare_parameter<double>("max_digit_height_ratio", 0.90);
    seg_params_.min_digit_aspect_ratio = declare_parameter<double>("min_digit_aspect_ratio", 0.20);
    seg_params_.max_digit_aspect_ratio = declare_parameter<double>("max_digit_aspect_ratio", 12.0);
    seg_params_.min_candidate_extent = declare_parameter<double>("min_candidate_extent", 0.06);
    seg_params_.max_candidate_extent = declare_parameter<double>("max_candidate_extent", 0.92);
    seg_params_.reject_border_candidates =
      declare_parameter<bool>("reject_border_candidates", true);
    seg_params_.a4_border_margin_px = declare_parameter<int>("a4_border_margin_px", 6);
    seg_params_.a4_border_margin_ratio =
      declare_parameter<double>("a4_border_margin_ratio", 0.015);
    seg_params_.min_candidate_center_x_ratio =
      declare_parameter<double>("min_candidate_center_x_ratio", 0.03);
    seg_params_.max_candidate_center_x_ratio =
      declare_parameter<double>("max_candidate_center_x_ratio", 0.97);
    seg_params_.min_candidate_center_y_ratio =
      declare_parameter<double>("min_candidate_center_y_ratio", 0.05);
    seg_params_.max_candidate_center_y_ratio =
      declare_parameter<double>("max_candidate_center_y_ratio", 0.95);
    seg_params_.max_vertical_line_width_px =
      declare_parameter<int>("max_vertical_line_width_px", 6);
    seg_params_.max_vertical_line_width_ratio =
      declare_parameter<double>("max_vertical_line_width_ratio", 0.035);
    seg_params_.min_vertical_line_aspect_ratio =
      declare_parameter<double>("min_vertical_line_aspect_ratio", 8.0);
    seg_params_.max_horizontal_line_height_px =
      declare_parameter<int>("max_horizontal_line_height_px", 6);
    seg_params_.max_horizontal_line_height_ratio =
      declare_parameter<double>("max_horizontal_line_height_ratio", 0.035);
    seg_params_.max_horizontal_line_aspect_ratio =
      declare_parameter<double>("max_horizontal_line_aspect_ratio", 0.12);
    seg_params_.crop_padding_px = declare_parameter<int>("crop_padding_px", 4);
    seg_params_.crop_padding_ratio = declare_parameter<double>("crop_padding_ratio", 0.025);
    seg_params_.union_fallback_padding_px =
      declare_parameter<int>("union_fallback_padding_px", 8);
    seg_params_.union_fallback_padding_ratio =
      declare_parameter<double>("union_fallback_padding_ratio", 0.04);
    seg_params_.enable_union_split = declare_parameter<bool>("enable_union_split", true);
    seg_params_.union_split_min_width_height_ratio =
      declare_parameter<double>("union_split_min_width_height_ratio", 0.85);
    seg_params_.union_split_min_valley_ratio =
      declare_parameter<double>("union_split_min_valley_ratio", 0.30);
    seg_params_.enable_relaxed_digit_fallback =
      declare_parameter<bool>("enable_relaxed_digit_fallback", true);
    seg_params_.relaxed_fallback_min_score =
      declare_parameter<double>("relaxed_fallback_min_score", 0.20);
    seg_params_.digit_input_size = declare_parameter<int>("digit_input_size", 64);
    seg_params_.max_digit_count = declare_parameter<int>("max_digit_count", 4);
    seg_params_.merge_gap_px = declare_parameter<int>("merge_gap_px", 8);
    seg_params_.merge_gap_ratio = declare_parameter<double>("merge_gap_ratio", 0.04);
    seg_params_.enable_relative_size_filter =
      declare_parameter<bool>("enable_relative_size_filter", true);
    seg_params_.relative_filter_use_or =
      declare_parameter<bool>("relative_filter_use_or", true);
    seg_params_.min_area_ratio_to_largest =
      declare_parameter<double>("min_area_ratio_to_largest", 0.08);
    seg_params_.min_height_ratio_to_largest =
      declare_parameter<double>("min_height_ratio_to_largest", 0.28);
    seg_params_.enable_slender_digit_protection =
      declare_parameter<bool>("enable_slender_digit_protection", true);
    seg_params_.slender_min_height_ratio_to_largest =
      declare_parameter<double>("slender_min_height_ratio_to_largest", 0.50);
    seg_params_.slender_max_width_ratio_to_largest =
      declare_parameter<double>("slender_max_width_ratio_to_largest", 0.45);
    seg_params_.enable_horizontal_merge =
      declare_parameter<bool>("enable_horizontal_merge", true);
    seg_params_.enable_vertical_split_merge =
      declare_parameter<bool>("enable_vertical_split_merge", true);
    seg_params_.merge_vertical_overlap_min_ratio =
      declare_parameter<double>("merge_vertical_overlap_min_ratio", 0.45);
    seg_params_.merge_min_y_overlap_ratio =
      declare_parameter<double>("merge_min_y_overlap_ratio", 0.30);
    seg_params_.merge_small_part_area_ratio_max =
      declare_parameter<double>("merge_small_part_area_ratio_max", 0.45);
    seg_params_.merge_small_part_height_ratio_max =
      declare_parameter<double>("merge_small_part_height_ratio_max", 0.72);
    seg_params_.merge_max_height_diff_px =
      declare_parameter<int>("merge_max_height_diff_px", 18);
    seg_params_.merge_max_height_diff_ratio =
      declare_parameter<double>("merge_max_height_diff_ratio", 0.45);
    seg_params_.vertical_merge_gap_px = declare_parameter<int>("vertical_merge_gap_px", 10);
    seg_params_.min_horizontal_overlap_ratio_for_vertical_merge =
      declare_parameter<double>("min_horizontal_overlap_ratio_for_vertical_merge", 0.55);

    cls_params_.onnx_model_path = declare_parameter<std::string>(
      "onnx_model_path", "models/digit_cnn.onnx");
    cls_params_.min_confidence = declare_parameter<double>("classifier_min_confidence", 0.70);
    cls_params_.prefer_cuda = declare_parameter<bool>("prefer_cuda", false);
    cls_params_.input_size = declare_parameter<int>("classifier_input_size", 64);

    circle_params_.use_color = declare_parameter<bool>("circle_marker_use_color", false);
    circle_params_.panel_gaussian_kernel =
      declare_parameter<int>("circle_marker_panel_gaussian_kernel", 5);
    circle_params_.panel_morph_kernel_size =
      declare_parameter<int>("circle_marker_panel_morph_kernel_size", 5);
    circle_params_.panel_min_area_ratio =
      declare_parameter<double>("circle_marker_panel_min_area_ratio", 0.01);
    circle_params_.panel_max_area_ratio =
      declare_parameter<double>("circle_marker_panel_max_area_ratio", 0.90);
    circle_params_.panel_approx_epsilon_ratio =
      declare_parameter<double>("circle_marker_panel_approx_epsilon", 0.02);
    circle_params_.panel_aspect_ratio =
      declare_parameter<double>("circle_marker_panel_aspect_ratio", 1.414);
    circle_params_.panel_aspect_tolerance =
      declare_parameter<double>("circle_marker_panel_aspect_tolerance", 0.35);
    circle_params_.panel_warp_width =
      declare_parameter<int>("circle_marker_panel_warp_width", 640);
    circle_params_.panel_warp_height =
      declare_parameter<int>("circle_marker_panel_warp_height", 452);
    circle_params_.search_x_min_ratio =
      declare_parameter<double>("circle_marker_search_x_min_ratio", 0.10);
    circle_params_.search_x_max_ratio =
      declare_parameter<double>("circle_marker_search_x_max_ratio", 0.90);
    circle_params_.search_y_min_ratio =
      declare_parameter<double>("circle_marker_search_y_min_ratio", 0.10);
    circle_params_.search_y_max_ratio =
      declare_parameter<double>("circle_marker_search_y_max_ratio", 0.90);
    circle_params_.dark_threshold =
      declare_parameter<int>("circle_marker_dark_threshold", 180);
    circle_params_.bright_threshold =
      declare_parameter<int>("circle_marker_bright_threshold", 190);
    circle_params_.use_otsu = declare_parameter<bool>("circle_marker_use_otsu", true);
    circle_params_.circle_min_area_ratio =
      declare_parameter<double>("circle_marker_min_area_ratio", 0.03);
    circle_params_.circle_max_area_ratio =
      declare_parameter<double>("circle_marker_max_area_ratio", 0.55);
    circle_params_.circle_min_circularity =
      declare_parameter<double>("circle_marker_min_circularity", 0.50);
    circle_params_.circle_aspect_min =
      declare_parameter<double>("circle_marker_aspect_min", 0.70);
    circle_params_.circle_aspect_max =
      declare_parameter<double>("circle_marker_aspect_max", 1.35);
    circle_params_.digit_min_area_ratio =
      declare_parameter<double>("circle_marker_digit_min_area_ratio", 0.001);
    circle_params_.digit_max_area_ratio =
      declare_parameter<double>("circle_marker_digit_max_area_ratio", 0.30);
    circle_params_.digit_padding_ratio =
      declare_parameter<double>("circle_marker_digit_padding_ratio", 0.22);
    circle_params_.max_digits = declare_parameter<int>("circle_marker_max_digits", 2);
    circle_params_.min_number = declare_parameter<int>("circle_marker_min_number", 1);
    circle_params_.max_number = declare_parameter<int>("circle_marker_max_number", 36);

    enabled_ = enable_on_start_;
  }

  void load_parameters()
  {
    seg_params_.digit_input_size = std::max(8, seg_params_.digit_input_size);
    cls_params_.input_size = std::max(8, cls_params_.input_size);
    circle_params_.digit_input_size = cls_params_.input_size;
    if (recognition_target_style_ != "circle_marker" &&
      recognition_target_style_ != "legacy_a4_digit")
    {
      RCLCPP_WARN(
        get_logger(),
        "recognition_target_style=%s 不支持，回退为 circle_marker",
        recognition_target_style_.c_str());
      recognition_target_style_ = "circle_marker";
    }
    detector_.set_params(a4_params_);
    segmenter_.set_params(seg_params_);
    circle_recognizer_.set_params(circle_params_);
    classifier_.set_params(cls_params_);
  }

  void print_parameter_summary()
  {
    RCLCPP_INFO(
      get_logger(),
      "camera_topic(fallback)=%s, active_camera_topic=%s, left_camera_topic=%s, right_camera_topic=%s, "
      "target_cabinet_topic=%s, route_waypoints_file=%s, recognized_topic=%s, onnx=%s, min_conf=%.2f, "
      "attempts=%d, input_size=%d",
      camera_topic_.c_str(),
      active_camera_topic_.c_str(),
      left_camera_topic_.c_str(),
      right_camera_topic_.c_str(),
      target_cabinet_topic_.c_str(),
      route_waypoints_file_.c_str(),
      recognized_topic_.c_str(),
      cls_params_.onnx_model_path.c_str(),
      min_confidence_,
      max_attempts_,
      cls_params_.input_size);

    RCLCPP_INFO(
      get_logger(),
      "A4: blur=%d min_area=%.3f eps=%.3f | SEG: blur=%d median=%d block=%d C=%d open=%d close=%d "
      "area=%d..%d ratio=%.5f..%.2f rel=%.2f/%.2f merge_gap=%d/%.3f merge_y=%.2f",
      a4_params_.gaussian_kernel,
      a4_params_.min_area_ratio,
      a4_params_.approx_epsilon_ratio,
      seg_params_.pre_blur_kernel,
      seg_params_.median_blur_kernel,
      seg_params_.adaptive_thresh_block_size,
      seg_params_.adaptive_thresh_c,
      seg_params_.morph_open_kernel_size,
      seg_params_.morph_close_kernel_size,
      seg_params_.min_digit_area,
      seg_params_.max_digit_area,
      seg_params_.min_digit_area_ratio,
      seg_params_.max_digit_area_ratio,
      seg_params_.min_area_ratio_to_largest,
      seg_params_.min_height_ratio_to_largest,
      seg_params_.merge_gap_px,
      seg_params_.merge_gap_ratio,
      seg_params_.merge_min_y_overlap_ratio);

    RCLCPP_INFO(
      get_logger(),
      "SEG_DENOISE: median_blur_kernel=%d binary_cleanup_enabled=%d "
      "min_component_area=%d min_component_size=%dx%d",
      seg_params_.median_blur_kernel,
      seg_params_.binary_cleanup_enabled ? 1 : 0,
      seg_params_.binary_cleanup_min_component_area,
      seg_params_.binary_cleanup_min_component_width,
      seg_params_.binary_cleanup_min_component_height);

    RCLCPP_INFO(
      get_logger(),
      "SEG_FILTER: border=%d margin=%d/%.3f extent=%.2f..%.2f line_v(w<=%d or %.3f, ar>=%.1f) "
      "line_h(h<=%d or %.3f, ar<=%.2f) crop_pad=%d/%.3f union_pad=%d/%.3f",
      seg_params_.reject_border_candidates ? 1 : 0,
      seg_params_.a4_border_margin_px,
      seg_params_.a4_border_margin_ratio,
      seg_params_.min_candidate_extent,
      seg_params_.max_candidate_extent,
      seg_params_.max_vertical_line_width_px,
      seg_params_.max_vertical_line_width_ratio,
      seg_params_.min_vertical_line_aspect_ratio,
      seg_params_.max_horizontal_line_height_px,
      seg_params_.max_horizontal_line_height_ratio,
      seg_params_.max_horizontal_line_aspect_ratio,
      seg_params_.crop_padding_px,
      seg_params_.crop_padding_ratio,
      seg_params_.union_fallback_padding_px,
      seg_params_.union_fallback_padding_ratio);

    RCLCPP_INFO(
      get_logger(),
      "SEG_FILTER_CORE: reject_border_candidates=%d a4_border_margin_px=%d "
      "a4_border_margin_ratio=%.3f center_x=%.3f..%.3f center_y=%.3f..%.3f "
      "min_digit_area=%d max_digit_area=%d candidate_extent=%.3f..%.3f "
      "relaxed_fallback=%d score>=%.2f union_split=%d wh>=%.2f valley<=%.2f",
      seg_params_.reject_border_candidates ? 1 : 0,
      seg_params_.a4_border_margin_px,
      seg_params_.a4_border_margin_ratio,
      seg_params_.min_candidate_center_x_ratio,
      seg_params_.max_candidate_center_x_ratio,
      seg_params_.min_candidate_center_y_ratio,
      seg_params_.max_candidate_center_y_ratio,
      seg_params_.min_digit_area,
      seg_params_.max_digit_area,
      seg_params_.min_candidate_extent,
      seg_params_.max_candidate_extent,
      seg_params_.enable_relaxed_digit_fallback ? 1 : 0,
      seg_params_.relaxed_fallback_min_score,
      seg_params_.enable_union_split ? 1 : 0,
      seg_params_.union_split_min_width_height_ratio,
      seg_params_.union_split_min_valley_ratio);

    RCLCPP_INFO(
      get_logger(),
      "visualization_distance_topic=%s, timeout=%.2fs, union_fallback=%d",
      distance_overlay_topic_.c_str(),
      distance_overlay_timeout_sec_,
      enable_union_fallback_classification_ ? 1 : 0);

    RCLCPP_INFO(
      get_logger(),
      "recognition_target_style=%s | CIRCLE_MARKER: panel_aspect=%.3f tol=%.3f warp=%dx%d "
      "panel_area=%.3f..%.3f search=%.2f..%.2f/%.2f..%.2f dark=%d bright=%d otsu=%d "
      "circle_area=%.3f..%.3f circ>=%.2f digits<=%d range=%d..%d",
      recognition_target_style_.c_str(),
      circle_params_.panel_aspect_ratio,
      circle_params_.panel_aspect_tolerance,
      circle_params_.panel_warp_width,
      circle_params_.panel_warp_height,
      circle_params_.panel_min_area_ratio,
      circle_params_.panel_max_area_ratio,
      circle_params_.search_x_min_ratio,
      circle_params_.search_x_max_ratio,
      circle_params_.search_y_min_ratio,
      circle_params_.search_y_max_ratio,
      circle_params_.dark_threshold,
      circle_params_.bright_threshold,
      circle_params_.use_otsu ? 1 : 0,
      circle_params_.circle_min_area_ratio,
      circle_params_.circle_max_area_ratio,
      circle_params_.circle_min_circularity,
      circle_params_.max_digits,
      circle_params_.min_number,
      circle_params_.max_number);
  }

  std::filesystem::path resolve_config_path(const std::string & raw_path) const
  {
    const std::filesystem::path path(raw_path);
    if (path.is_absolute()) {
      return path;
    }
    const std::string share_dir =
      ament_index_cpp::get_package_share_directory("agv_inventory_system");
    return std::filesystem::path(share_dir) / path;
  }

  static bool normalize_entry_side(std::string side, std::string & normalized)
  {
    side.erase(
      std::remove_if(
        side.begin(), side.end(),
        [](unsigned char c) {
          return std::isspace(c) != 0;
        }),
      side.end());
    std::transform(
      side.begin(), side.end(), side.begin(),
      [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
      });
    if (side != "left" && side != "right") {
      return false;
    }
    normalized = side;
    return true;
  }

  bool load_cabinet_entry_side_map()
  {
    cabinet_entry_side_map_.clear();
    try {
      const auto route_path = resolve_config_path(route_waypoints_file_);
      if (!std::filesystem::exists(route_path)) {
        RCLCPP_ERROR(
          get_logger(),
          "路线配置文件不存在: %s",
          route_path.string().c_str());
        return false;
      }

      const YAML::Node root = YAML::LoadFile(route_path.string());
      const YAML::Node map_root = root["cabinet_entry_side_map"];
      if (!map_root || !map_root.IsMap()) {
        RCLCPP_ERROR(
          get_logger(),
          "route_waypoints.yaml 缺少 cabinet_entry_side_map 映射");
        return false;
      }

      for (const auto map_item : map_root) {
        int cabinet_id = -1;
        const std::string cabinet_text = map_item.first.as<std::string>();
        if (!agv_inventory_system::safe_to_int(cabinet_text, cabinet_id)) {
          RCLCPP_ERROR(
            get_logger(),
            "cabinet_entry_side_map 货柜号非法: %s",
            cabinet_text.c_str());
          return false;
        }
        if (!map_item.second.IsScalar()) {
          RCLCPP_ERROR(
            get_logger(),
            "cabinet_entry_side_map 条目必须直接映射到 left/right: %s",
            cabinet_text.c_str());
          return false;
        }

        const std::string raw_side = map_item.second.as<std::string>();
        std::string side;
        if (!normalize_entry_side(raw_side, side)) {
          RCLCPP_ERROR(
            get_logger(),
            "cabinet_entry_side_map side 必须为 left/right: cabinet=%s side=%s",
            cabinet_text.c_str(),
            raw_side.c_str());
          return false;
        }
        cabinet_entry_side_map_[cabinet_id] = side;
      }

      RCLCPP_INFO(
        get_logger(),
        "已加载 cabinet_entry_side_map: file=%s entries=%zu",
        route_path.string().c_str(),
        cabinet_entry_side_map_.size());
      return true;
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "解析 cabinet_entry_side_map 失败: %s", ex.what());
      return false;
    }
  }

  void set_image_subscription(
    const std::string & topic,
    int target_cabinet,
    const std::string & entry_side,
    bool force = false)
  {
    const std::string next_topic = topic.empty() ? camera_topic_ : topic;
    if (next_topic.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "无法切换相机订阅: target_cabinet=%d entry_side=%s camera_topic 为空",
        target_cabinet,
        entry_side.c_str());
      return;
    }

    if (!force && next_topic == active_camera_topic_) {
      RCLCPP_INFO(
        get_logger(),
        "相机订阅保持: target_cabinet=%d entry_side=%s camera_topic=%s",
        target_cabinet,
        entry_side.c_str(),
        active_camera_topic_.c_str());
      return;
    }

    image_sub_.reset();
    active_camera_topic_ = next_topic;
    attempts_ = 0;
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      active_camera_topic_,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        image_callback(msg);
      });

    RCLCPP_INFO(
      get_logger(),
      "相机订阅切换: target_cabinet=%d entry_side=%s camera_topic=%s",
      target_cabinet,
      entry_side.c_str(),
      active_camera_topic_.c_str());
  }

  void switch_camera_for_target(int target_cabinet)
  {
    current_target_cabinet_ = target_cabinet;
    if (target_cabinet <= 0) {
      set_image_subscription(camera_topic_, target_cabinet, "fallback");
      return;
    }

    const auto side_it = cabinet_entry_side_map_.find(target_cabinet);
    if (side_it == cabinet_entry_side_map_.end()) {
      RCLCPP_WARN(
        get_logger(),
        "target_cabinet=%d 未配置 cabinet_entry_side_map，使用 fallback camera_topic=%s",
        target_cabinet,
        camera_topic_.c_str());
      set_image_subscription(camera_topic_, target_cabinet, "fallback");
      return;
    }

    const std::string & entry_side = side_it->second;
    if (entry_side == "left") {
      set_image_subscription(left_camera_topic_, target_cabinet, entry_side);
      return;
    }
    if (entry_side == "right") {
      set_image_subscription(right_camera_topic_, target_cabinet, entry_side);
      return;
    }

    RCLCPP_WARN(
      get_logger(),
      "target_cabinet=%d entry_side=%s 非 left/right，使用 fallback camera_topic=%s",
      target_cabinet,
      entry_side.c_str(),
      camera_topic_.c_str());
    set_image_subscription(camera_topic_, target_cabinet, "fallback");
  }

  void target_cabinet_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    switch_camera_for_target(msg->data);
  }

  bool reload_classifier_model()
  {
    auto cls_runtime = cls_params_;
    namespace fs = std::filesystem;
    fs::path model_path(cls_runtime.onnx_model_path);
    if (!model_path.is_absolute()) {
      const std::string share_dir =
        ament_index_cpp::get_package_share_directory("agv_inventory_system");
      model_path = fs::path(share_dir) / cls_runtime.onnx_model_path;
    }
    cls_runtime.onnx_model_path = model_path.string();

    classifier_.set_params(cls_runtime);
    std::string err;
    if (!classifier_.load_model(err)) {
      RCLCPP_ERROR(get_logger(), "加载 ONNX 失败: %s", err.c_str());
      return false;
    }

    RCLCPP_INFO(get_logger(), "已加载 ONNX 模型: %s", cls_runtime.onnx_model_path.c_str());
    return true;
  }

  rcl_interfaces::msg::SetParametersResult on_parameters_set(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "ok";

    auto a4_new = a4_params_;
    auto seg_new = seg_params_;
    auto cls_new = cls_params_;
    auto circle_new = circle_params_;
    auto style_new = recognition_target_style_;

    bool reload_model = false;

    for (const auto & p : params) {
      const auto & name = p.get_name();

      if (name == "recognition_target_style") {
        style_new = p.as_string();
      } else if (name == "min_confidence") {
        min_confidence_ = p.as_double();
      } else if (name == "max_attempts") {
        max_attempts_ = p.as_int();
      } else if (name == "attempt_interval") {
        attempt_interval_ = p.as_double();
      } else if (name == "distance_overlay_timeout_sec") {
        distance_overlay_timeout_sec_ = p.as_double();
      } else if (name == "enable_union_fallback_classification") {
        enable_union_fallback_classification_ = p.as_bool();
      } else if (name == "circle_marker_debug_log_enabled") {
        circle_marker_debug_log_enabled_ = p.as_bool();
      } else if (name == "cabinet_id_min") {
        cabinet_id_min_ = p.as_int();
      } else if (name == "cabinet_id_max") {
        cabinet_id_max_ = p.as_int();
      } else if (name == "known_digit_height_px") {
        known_digit_height_px_ = p.as_double();
      } else if (name == "known_distance_m") {
        known_distance_m_ = p.as_double();
      } else if (name == "distance_focal_length") {
        distance_focal_length_ = p.as_double();
      } else if (name == "a4_gaussian_kernel") {
        a4_new.gaussian_kernel = p.as_int();
      } else if (name == "a4_morph_kernel_size") {
        a4_new.morph_kernel_size = p.as_int();
      } else if (name == "a4_min_area_ratio") {
        a4_new.min_area_ratio = p.as_double();
      } else if (name == "a4_approx_epsilon") {
        a4_new.approx_epsilon_ratio = p.as_double();
      } else if (name == "a4_min_aspect_ratio") {
        a4_new.min_aspect_ratio = p.as_double();
      } else if (name == "a4_max_aspect_ratio") {
        a4_new.max_aspect_ratio = p.as_double();
      } else if (name == "clahe_clip_limit") {
        seg_new.clahe_clip_limit = p.as_double();
      } else if (name == "clahe_grid_size") {
        seg_new.clahe_grid_size = p.as_int();
      } else if (name == "pre_blur_kernel") {
        seg_new.pre_blur_kernel = p.as_int();
      } else if (name == "median_blur_kernel") {
        seg_new.median_blur_kernel = p.as_int();
      } else if (name == "adaptive_thresh_block_size") {
        seg_new.adaptive_thresh_block_size = p.as_int();
      } else if (name == "adaptive_thresh_c") {
        seg_new.adaptive_thresh_c = p.as_int();
      } else if (name == "enable_otsu_fusion") {
        seg_new.enable_otsu_fusion = p.as_bool();
      } else if (name == "binary_cleanup_enabled") {
        seg_new.binary_cleanup_enabled = p.as_bool();
      } else if (name == "binary_cleanup_min_component_area") {
        seg_new.binary_cleanup_min_component_area = p.as_int();
      } else if (name == "binary_cleanup_min_component_width") {
        seg_new.binary_cleanup_min_component_width = p.as_int();
      } else if (name == "binary_cleanup_min_component_height") {
        seg_new.binary_cleanup_min_component_height = p.as_int();
      } else if (name == "morph_kernel_size") {
        seg_new.morph_kernel_size = p.as_int();
      } else if (name == "morph_open_kernel_size") {
        seg_new.morph_open_kernel_size = p.as_int();
      } else if (name == "morph_close_kernel_size") {
        seg_new.morph_close_kernel_size = p.as_int();
      } else if (name == "min_digit_area") {
        seg_new.min_digit_area = p.as_int();
      } else if (name == "max_digit_area") {
        seg_new.max_digit_area = p.as_int();
      } else if (name == "min_digit_area_ratio") {
        seg_new.min_digit_area_ratio = p.as_double();
      } else if (name == "max_digit_area_ratio") {
        seg_new.max_digit_area_ratio = p.as_double();
      } else if (name == "min_digit_width") {
        seg_new.min_digit_width = p.as_int();
      } else if (name == "min_digit_height") {
        seg_new.min_digit_height = p.as_int();
      } else if (name == "max_digit_width") {
        seg_new.max_digit_width = p.as_int();
      } else if (name == "max_digit_height") {
        seg_new.max_digit_height = p.as_int();
      } else if (name == "min_digit_width_ratio") {
        seg_new.min_digit_width_ratio = p.as_double();
      } else if (name == "max_digit_width_ratio") {
        seg_new.max_digit_width_ratio = p.as_double();
      } else if (name == "min_digit_height_ratio") {
        seg_new.min_digit_height_ratio = p.as_double();
      } else if (name == "max_digit_height_ratio") {
        seg_new.max_digit_height_ratio = p.as_double();
      } else if (name == "min_digit_aspect_ratio") {
        seg_new.min_digit_aspect_ratio = p.as_double();
      } else if (name == "max_digit_aspect_ratio") {
        seg_new.max_digit_aspect_ratio = p.as_double();
      } else if (name == "min_candidate_extent") {
        seg_new.min_candidate_extent = p.as_double();
      } else if (name == "max_candidate_extent") {
        seg_new.max_candidate_extent = p.as_double();
      } else if (name == "reject_border_candidates") {
        seg_new.reject_border_candidates = p.as_bool();
      } else if (name == "a4_border_margin_px") {
        seg_new.a4_border_margin_px = p.as_int();
      } else if (name == "a4_border_margin_ratio") {
        seg_new.a4_border_margin_ratio = p.as_double();
      } else if (name == "min_candidate_center_x_ratio") {
        seg_new.min_candidate_center_x_ratio = p.as_double();
      } else if (name == "max_candidate_center_x_ratio") {
        seg_new.max_candidate_center_x_ratio = p.as_double();
      } else if (name == "min_candidate_center_y_ratio") {
        seg_new.min_candidate_center_y_ratio = p.as_double();
      } else if (name == "max_candidate_center_y_ratio") {
        seg_new.max_candidate_center_y_ratio = p.as_double();
      } else if (name == "max_vertical_line_width_px") {
        seg_new.max_vertical_line_width_px = p.as_int();
      } else if (name == "max_vertical_line_width_ratio") {
        seg_new.max_vertical_line_width_ratio = p.as_double();
      } else if (name == "min_vertical_line_aspect_ratio") {
        seg_new.min_vertical_line_aspect_ratio = p.as_double();
      } else if (name == "max_horizontal_line_height_px") {
        seg_new.max_horizontal_line_height_px = p.as_int();
      } else if (name == "max_horizontal_line_height_ratio") {
        seg_new.max_horizontal_line_height_ratio = p.as_double();
      } else if (name == "max_horizontal_line_aspect_ratio") {
        seg_new.max_horizontal_line_aspect_ratio = p.as_double();
      } else if (name == "crop_padding_px") {
        seg_new.crop_padding_px = p.as_int();
      } else if (name == "crop_padding_ratio") {
        seg_new.crop_padding_ratio = p.as_double();
      } else if (name == "union_fallback_padding_px") {
        seg_new.union_fallback_padding_px = p.as_int();
      } else if (name == "union_fallback_padding_ratio") {
        seg_new.union_fallback_padding_ratio = p.as_double();
      } else if (name == "enable_union_split") {
        seg_new.enable_union_split = p.as_bool();
      } else if (name == "union_split_min_width_height_ratio") {
        seg_new.union_split_min_width_height_ratio = p.as_double();
      } else if (name == "union_split_min_valley_ratio") {
        seg_new.union_split_min_valley_ratio = p.as_double();
      } else if (name == "enable_relaxed_digit_fallback") {
        seg_new.enable_relaxed_digit_fallback = p.as_bool();
      } else if (name == "relaxed_fallback_min_score") {
        seg_new.relaxed_fallback_min_score = p.as_double();
      } else if (name == "digit_input_size") {
        seg_new.digit_input_size = p.as_int();
      } else if (name == "max_digit_count") {
        seg_new.max_digit_count = p.as_int();
      } else if (name == "merge_gap_px") {
        seg_new.merge_gap_px = p.as_int();
      } else if (name == "merge_gap_ratio") {
        seg_new.merge_gap_ratio = p.as_double();
      } else if (name == "enable_relative_size_filter") {
        seg_new.enable_relative_size_filter = p.as_bool();
      } else if (name == "relative_filter_use_or") {
        seg_new.relative_filter_use_or = p.as_bool();
      } else if (name == "min_area_ratio_to_largest") {
        seg_new.min_area_ratio_to_largest = p.as_double();
      } else if (name == "min_height_ratio_to_largest") {
        seg_new.min_height_ratio_to_largest = p.as_double();
      } else if (name == "enable_slender_digit_protection") {
        seg_new.enable_slender_digit_protection = p.as_bool();
      } else if (name == "slender_min_height_ratio_to_largest") {
        seg_new.slender_min_height_ratio_to_largest = p.as_double();
      } else if (name == "slender_max_width_ratio_to_largest") {
        seg_new.slender_max_width_ratio_to_largest = p.as_double();
      } else if (name == "enable_horizontal_merge") {
        seg_new.enable_horizontal_merge = p.as_bool();
      } else if (name == "enable_vertical_split_merge") {
        seg_new.enable_vertical_split_merge = p.as_bool();
      } else if (name == "merge_vertical_overlap_min_ratio") {
        seg_new.merge_vertical_overlap_min_ratio = p.as_double();
      } else if (name == "merge_min_y_overlap_ratio") {
        seg_new.merge_min_y_overlap_ratio = p.as_double();
      } else if (name == "merge_small_part_area_ratio_max") {
        seg_new.merge_small_part_area_ratio_max = p.as_double();
      } else if (name == "merge_small_part_height_ratio_max") {
        seg_new.merge_small_part_height_ratio_max = p.as_double();
      } else if (name == "merge_max_height_diff_px") {
        seg_new.merge_max_height_diff_px = p.as_int();
      } else if (name == "merge_max_height_diff_ratio") {
        seg_new.merge_max_height_diff_ratio = p.as_double();
      } else if (name == "vertical_merge_gap_px") {
        seg_new.vertical_merge_gap_px = p.as_int();
      } else if (name == "min_horizontal_overlap_ratio_for_vertical_merge") {
        seg_new.min_horizontal_overlap_ratio_for_vertical_merge = p.as_double();
      } else if (name == "classifier_input_size") {
        cls_new.input_size = p.as_int();
      } else if (name == "onnx_model_path") {
        cls_new.onnx_model_path = p.as_string();
        reload_model = true;
      } else if (name == "classifier_min_confidence") {
        cls_new.min_confidence = p.as_double();
      } else if (name == "prefer_cuda") {
        cls_new.prefer_cuda = p.as_bool();
        reload_model = true;
      } else if (name == "circle_marker_use_color") {
        circle_new.use_color = p.as_bool();
      } else if (name == "circle_marker_panel_gaussian_kernel") {
        circle_new.panel_gaussian_kernel = p.as_int();
      } else if (name == "circle_marker_panel_morph_kernel_size") {
        circle_new.panel_morph_kernel_size = p.as_int();
      } else if (name == "circle_marker_panel_min_area_ratio") {
        circle_new.panel_min_area_ratio = p.as_double();
      } else if (name == "circle_marker_panel_max_area_ratio") {
        circle_new.panel_max_area_ratio = p.as_double();
      } else if (name == "circle_marker_panel_approx_epsilon") {
        circle_new.panel_approx_epsilon_ratio = p.as_double();
      } else if (name == "circle_marker_panel_aspect_ratio") {
        circle_new.panel_aspect_ratio = p.as_double();
      } else if (name == "circle_marker_panel_aspect_tolerance") {
        circle_new.panel_aspect_tolerance = p.as_double();
      } else if (name == "circle_marker_panel_warp_width") {
        circle_new.panel_warp_width = p.as_int();
      } else if (name == "circle_marker_panel_warp_height") {
        circle_new.panel_warp_height = p.as_int();
      } else if (name == "circle_marker_search_x_min_ratio") {
        circle_new.search_x_min_ratio = p.as_double();
      } else if (name == "circle_marker_search_x_max_ratio") {
        circle_new.search_x_max_ratio = p.as_double();
      } else if (name == "circle_marker_search_y_min_ratio") {
        circle_new.search_y_min_ratio = p.as_double();
      } else if (name == "circle_marker_search_y_max_ratio") {
        circle_new.search_y_max_ratio = p.as_double();
      } else if (name == "circle_marker_dark_threshold") {
        circle_new.dark_threshold = p.as_int();
      } else if (name == "circle_marker_bright_threshold") {
        circle_new.bright_threshold = p.as_int();
      } else if (name == "circle_marker_use_otsu") {
        circle_new.use_otsu = p.as_bool();
      } else if (name == "circle_marker_min_area_ratio") {
        circle_new.circle_min_area_ratio = p.as_double();
      } else if (name == "circle_marker_max_area_ratio") {
        circle_new.circle_max_area_ratio = p.as_double();
      } else if (name == "circle_marker_min_circularity") {
        circle_new.circle_min_circularity = p.as_double();
      } else if (name == "circle_marker_aspect_min") {
        circle_new.circle_aspect_min = p.as_double();
      } else if (name == "circle_marker_aspect_max") {
        circle_new.circle_aspect_max = p.as_double();
      } else if (name == "circle_marker_digit_min_area_ratio") {
        circle_new.digit_min_area_ratio = p.as_double();
      } else if (name == "circle_marker_digit_max_area_ratio") {
        circle_new.digit_max_area_ratio = p.as_double();
      } else if (name == "circle_marker_digit_padding_ratio") {
        circle_new.digit_padding_ratio = p.as_double();
      } else if (name == "circle_marker_max_digits") {
        circle_new.max_digits = p.as_int();
      } else if (name == "circle_marker_min_number") {
        circle_new.min_number = p.as_int();
      } else if (name == "circle_marker_max_number") {
        circle_new.max_number = p.as_int();
      }
    }

    if (cabinet_id_max_ < cabinet_id_min_) {
      result.successful = false;
      result.reason = "cabinet_id_max 必须 >= cabinet_id_min";
      return result;
    }

    if (seg_new.digit_input_size <= 0 || cls_new.input_size <= 0) {
      result.successful = false;
      result.reason = "输入尺寸参数必须大于0";
      return result;
    }
    if (circle_new.panel_warp_width <= 0 || circle_new.panel_warp_height <= 0 ||
      circle_new.max_digits <= 0)
    {
      result.successful = false;
      result.reason = "circle_marker panel 输入尺寸和最大位数必须大于0";
      return result;
    }
    if (circle_new.max_number < circle_new.min_number) {
      result.successful = false;
      result.reason = "circle_marker_max_number 必须 >= circle_marker_min_number";
      return result;
    }
    if (style_new != "circle_marker" &&
      style_new != "legacy_a4_digit")
    {
      result.successful = false;
      result.reason = "recognition_target_style 仅支持 circle_marker 或 legacy_a4_digit";
      return result;
    }

    a4_params_ = a4_new;
    seg_params_ = seg_new;
    cls_params_ = cls_new;
    circle_params_ = circle_new;
    recognition_target_style_ = style_new;

    seg_params_.digit_input_size = std::max(8, seg_params_.digit_input_size);
    cls_params_.input_size = std::max(8, cls_params_.input_size);
    circle_params_.digit_input_size = cls_params_.input_size;

    detector_.set_params(a4_params_);
    segmenter_.set_params(seg_params_);
    circle_recognizer_.set_params(circle_params_);
    classifier_.set_params(cls_params_);

    if (reload_model && !reload_classifier_model()) {
      result.successful = false;
      result.reason = "模型重载失败";
      return result;
    }

    return result;
  }

  void trigger_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    enabled_ = request->data;
    if (!enabled_) {
      attempts_ = 0;
    }

    response->success = true;
    response->message = enabled_ ? "识别已开启" : "识别已关闭";
  }

  void enable_control_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    if (enabled_ == msg->data) {
      return;
    }

    enabled_ = msg->data;
    if (!enabled_) {
      attempts_ = 0;
      RCLCPP_INFO(get_logger(), "识别使能关闭：停止图像推理与结果发布");
    } else {
      RCLCPP_INFO(get_logger(), "识别使能开启：恢复图像推理");
    }
  }

  void publish_recognition(
    const std::string & number,
    float confidence,
    bool valid,
    float horizontal_offset,
    float estimated_distance)
  {
    agv_inventory_system::msg::RecognizedNumber out;
    out.number = number;
    out.confidence = confidence;
    out.valid = valid;
    out.horizontal_offset = horizontal_offset;
    out.attempts = attempts_;
    out.estimated_distance = estimated_distance;
    recog_pub_->publish(out);
  }

  void publish_image(
    const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr & pub,
    const std_msgs::msg::Header & header,
    const cv::Mat & image,
    const std::string & fallback_encoding = sensor_msgs::image_encodings::BGR8)
  {
    if (!pub || image.empty()) {
      return;
    }

    std::string encoding = fallback_encoding;
    if (image.type() == CV_8UC1) {
      encoding = sensor_msgs::image_encodings::MONO8;
    }

    auto msg = cv_bridge::CvImage(header, encoding, image).toImageMsg();
    pub->publish(*msg);
  }

  static cv::Mat compose_segment_debug_view(
    const agv_inventory_system::DigitSegmentationResult & seg_result,
    const cv::Mat & union_fallback_patch = cv::Mat())
  {
    const auto ensure_bgr = [](const cv::Mat & in) -> cv::Mat {
      if (in.empty()) {
        return {};
      }
      if (in.type() == CV_8UC1) {
        cv::Mat bgr;
        cv::cvtColor(in, bgr, cv::COLOR_GRAY2BGR);
        return bgr;
      }
      return in.clone();
    };

    cv::Mat binary = ensure_bgr(seg_result.binary_debug);
    cv::Mat montage = ensure_bgr(seg_result.montage_debug);
    cv::Mat union_patch = ensure_bgr(union_fallback_patch);
    if (binary.empty() && montage.empty() && union_patch.empty()) {
      return {};
    }
    if (binary.empty() && union_patch.empty()) {
      return montage;
    }
    if (binary.empty()) {
      return union_patch;
    }
    if (montage.empty() && union_patch.empty()) {
      return binary;
    }
    if (montage.empty()) {
      const int h = std::max(binary.rows, union_patch.rows);
      const int w_bin = std::max(
        120,
        static_cast<int>(std::round(
          static_cast<double>(binary.cols) * static_cast<double>(h) / std::max(1, binary.rows))));
      const int w_union = std::max(
        120,
        static_cast<int>(std::round(
          static_cast<double>(union_patch.cols) * static_cast<double>(h) / std::max(1, union_patch.rows))));
      cv::Mat bin_rs;
      cv::Mat union_rs;
      cv::resize(binary, bin_rs, cv::Size(w_bin, h), 0.0, 0.0, cv::INTER_NEAREST);
      cv::resize(union_patch, union_rs, cv::Size(w_union, h), 0.0, 0.0, cv::INTER_NEAREST);
      cv::Mat out(h, w_bin + w_union, CV_8UC3, cv::Scalar(20, 20, 20));
      bin_rs.copyTo(out(cv::Rect(0, 0, w_bin, h)));
      union_rs.copyTo(out(cv::Rect(w_bin, 0, w_union, h)));
      cv::putText(
        out,
        "binary",
        cv::Point(8, 22),
        cv::FONT_HERSHEY_SIMPLEX,
        0.6,
        cv::Scalar(0, 255, 255),
        1,
        cv::LINE_AA);
      cv::putText(
        out,
        "union fallback",
        cv::Point(w_bin + 8, 22),
        cv::FONT_HERSHEY_SIMPLEX,
        0.6,
        cv::Scalar(0, 255, 255),
        1,
        cv::LINE_AA);
      return out;
    }

    const int h = std::max({binary.rows, montage.rows, union_patch.empty() ? 0 : union_patch.rows});
    const int w_bin = std::max(120, static_cast<int>(std::round(
        static_cast<double>(binary.cols) * static_cast<double>(h) / std::max(1, binary.rows))));
    const int w_mtg = std::max(120, static_cast<int>(std::round(
        static_cast<double>(montage.cols) * static_cast<double>(h) / std::max(1, montage.rows))));
    const int w_union = union_patch.empty() ? 0 : std::max(
      120, static_cast<int>(std::round(
          static_cast<double>(union_patch.cols) * static_cast<double>(h) /
          std::max(1, union_patch.rows))));

    cv::Mat bin_rs;
    cv::Mat mtg_rs;
    cv::resize(binary, bin_rs, cv::Size(w_bin, h), 0.0, 0.0, cv::INTER_NEAREST);
    cv::resize(montage, mtg_rs, cv::Size(w_mtg, h), 0.0, 0.0, cv::INTER_NEAREST);
    cv::Mat union_rs;
    if (!union_patch.empty()) {
      cv::resize(union_patch, union_rs, cv::Size(w_union, h), 0.0, 0.0, cv::INTER_NEAREST);
    }

    cv::Mat out(h, w_bin + w_mtg + w_union, CV_8UC3, cv::Scalar(20, 20, 20));
    bin_rs.copyTo(out(cv::Rect(0, 0, w_bin, h)));
    mtg_rs.copyTo(out(cv::Rect(w_bin, 0, w_mtg, h)));
    if (!union_rs.empty()) {
      union_rs.copyTo(out(cv::Rect(w_bin + w_mtg, 0, w_union, h)));
    }

    cv::putText(
      out,
      "binary",
      cv::Point(8, 22),
      cv::FONT_HERSHEY_SIMPLEX,
      0.6,
      cv::Scalar(0, 255, 255),
      1,
      cv::LINE_AA);
    cv::putText(
      out,
      "digit crops",
      cv::Point(w_bin + 8, 22),
      cv::FONT_HERSHEY_SIMPLEX,
      0.6,
      cv::Scalar(0, 255, 255),
      1,
      cv::LINE_AA);
    if (!union_rs.empty()) {
      cv::putText(
        out,
        "union fallback",
        cv::Point(w_bin + w_mtg + 8, 22),
        cv::FONT_HERSHEY_SIMPLEX,
        0.6,
        cv::Scalar(0, 255, 255),
        1,
        cv::LINE_AA);
    }

    return out;
  }

  static cv::Rect union_digit_boxes(const std::vector<agv_inventory_system::DigitCandidate> & digits)
  {
    if (digits.empty()) {
      return cv::Rect();
    }

    cv::Rect box = digits.front().bbox;
    for (std::size_t i = 1; i < digits.size(); ++i) {
      box |= digits[i].bbox;
    }
    return box;
  }

  void log_segmentation_candidates(
    const agv_inventory_system::DigitSegmentationResult & seg_result,
    const std::string & context)
  {
    const rclcpp::Time now = this->now();
    if (last_candidate_debug_log_.nanoseconds() != 0 &&
      (now - last_candidate_debug_log_).seconds() < 1.0)
    {
      return;
    }
    last_candidate_debug_log_ = now;

    if (seg_result.has_best_shape_candidate) {
      const auto & b = seg_result.best_shape_candidate;
      RCLCPP_INFO(
        get_logger(),
        "SEG_DEBUG[%s]: warped=%dx%d raw=%d area_ok=%d shape_ok=%d edge_ok=%d rel_ok=%d final=%d "
        "final_zero=%s best_shape[x=%d y=%d w=%d h=%d area=%.1f ar=%.2f ext=%.3f cx=%.3f cy=%.3f "
        "score=%.3f reason=%s]",
        context.c_str(),
        seg_result.stats.warped_width,
        seg_result.stats.warped_height,
        seg_result.stats.raw_contours,
        seg_result.stats.area_pass_count,
        seg_result.stats.shape_pass_count,
        seg_result.stats.edge_pass_count,
        seg_result.stats.relative_pass_count,
        seg_result.stats.final_count,
        seg_result.final_zero_reason.c_str(),
        b.bbox.x,
        b.bbox.y,
        b.bbox.width,
        b.bbox.height,
        b.area,
        b.aspect_ratio,
        b.extent,
        b.center_x_ratio,
        b.center_y_ratio,
        b.score,
        b.reject_reason.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(),
        "SEG_DEBUG[%s]: warped=%dx%d raw=%d area_ok=%d shape_ok=%d edge_ok=%d rel_ok=%d final=%d final_zero=%s",
        context.c_str(),
        seg_result.stats.warped_width,
        seg_result.stats.warped_height,
        seg_result.stats.raw_contours,
        seg_result.stats.area_pass_count,
        seg_result.stats.shape_pass_count,
        seg_result.stats.edge_pass_count,
        seg_result.stats.relative_pass_count,
        seg_result.stats.final_count,
        seg_result.final_zero_reason.c_str());
    }

    for (std::size_t i = 0; i < seg_result.candidate_debug.size(); ++i) {
      const auto & c = seg_result.candidate_debug[i];
      RCLCPP_INFO(
        get_logger(),
        "SEG_CAND[%zu]: x=%d y=%d w=%d h=%d area=%.1f ar=%.2f ext=%.3f cx=%.3f cy=%.3f "
        "score=%.3f flags(area=%d shape=%d edge=%d rel=%d final=%d) reason=%s",
        i,
        c.bbox.x,
        c.bbox.y,
        c.bbox.width,
        c.bbox.height,
        c.area,
        c.aspect_ratio,
        c.extent,
        c.center_x_ratio,
        c.center_y_ratio,
        c.score,
        c.area_ok ? 1 : 0,
        c.shape_ok ? 1 : 0,
        c.edge_ok ? 1 : 0,
        c.relative_ok ? 1 : 0,
        c.final_candidate ? 1 : 0,
        c.reject_reason.c_str());
    }
  }

  void draw_failure_text(cv::Mat & image, const std::string & error) const
  {
    cv::putText(
      image,
      "FAIL: " + error,
      cv::Point(20, 40),
      cv::FONT_HERSHEY_SIMPLEX,
      0.8,
      cv::Scalar(0, 0, 255),
      2,
      cv::LINE_AA);
  }

  void log_circle_marker_diagnostics(
    const agv_inventory_system::CircleMarkerResult & circle_result,
    bool recognized,
    bool valid,
    bool conf_ok)
  {
    if (!circle_marker_debug_log_enabled_ && recognized && valid) {
      return;
    }

    const rclcpp::Time now = this->now();
    if (last_circle_marker_debug_log_.nanoseconds() != 0 &&
      (now - last_circle_marker_debug_log_).seconds() < 1.0)
    {
      return;
    }
    last_circle_marker_debug_log_ = now;

    const auto & seq = circle_result.classification;
    RCLCPP_INFO(
      get_logger(),
      "CIRCLE_DEBUG: extract_ok=%d classifier_input_ready=%d digit_images=%zu "
      "recognized=%d result_success=%d result_valid=%d node_valid=%d conf_ok=%d "
      "raw_number=%s raw_conf=%.3f seq_success=%d seq_items=%zu err=%s seq_err=%s",
      circle_result.extract_digits_success ? 1 : 0,
      circle_result.classifier_input_ready ? 1 : 0,
      circle_result.digit_images.size(),
      recognized ? 1 : 0,
      circle_result.success ? 1 : 0,
      circle_result.valid ? 1 : 0,
      valid ? 1 : 0,
      conf_ok ? 1 : 0,
      seq.number.c_str(),
      seq.min_confidence,
      seq.success ? 1 : 0,
      seq.items.size(),
      circle_result.error_message.c_str(),
      seq.error_message.c_str());

    for (std::size_t i = 0; i < circle_result.digit_debug.size(); ++i) {
      const auto & info = circle_result.digit_debug[i];
      RCLCPP_INFO(
        get_logger(),
        "CIRCLE_INPUT[%zu]: size=%dx%d non_white=%d/%d ratio=%.4f",
        i,
        info.width,
        info.height,
        info.non_white_pixels,
        info.total_pixels,
        info.non_white_ratio);
    }

    const int accepted_candidates =
      std::max(0, circle_result.digit_contour_count - circle_result.digit_candidate_rejected_count);
    RCLCPP_INFO(
      get_logger(),
      "CIRCLE_DIGIT_CONTOURS: contours=%d rejected=%d accepted=%d digit_images=%zu",
      circle_result.digit_contour_count,
      circle_result.digit_candidate_rejected_count,
      accepted_candidates,
      circle_result.digit_images.size());

    for (const auto & info : circle_result.digit_candidate_debug) {
      const std::string decision = info.accepted ?
        "accept" :
        std::string("reject: ") + info.reject_reason;
      RCLCPP_INFO(
        get_logger(),
        "CIRCLE_DIGIT_CANDIDATE[%d]: bbox=%d,%d,%d,%d contour_area=%.1f "
        "bbox_area=%.1f area_ratio=%.5f height_ratio=%.5f width_ratio=%.5f "
        "aspect_ratio=%.5f %s",
        info.contour_index,
        info.bbox.x,
        info.bbox.y,
        info.bbox.width,
        info.bbox.height,
        info.contour_area,
        info.bbox_area,
        info.area_ratio,
        info.height_ratio,
        info.width_ratio,
        info.aspect_ratio,
        decision.c_str());
    }

    for (std::size_t i = 0; i < seq.items.size(); ++i) {
      const auto & item = seq.items[i];
      std::vector<std::pair<int, float>> top;
      top.reserve(item.probabilities.size());
      for (std::size_t j = 0; j < item.probabilities.size(); ++j) {
        top.emplace_back(static_cast<int>(j), item.probabilities[j]);
      }
      std::sort(
        top.begin(), top.end(),
        [](const auto & a, const auto & b) {
          return a.second > b.second;
        });
      while (top.size() < 3U) {
        top.emplace_back(-1, 0.0F);
      }

      RCLCPP_INFO(
        get_logger(),
        "CIRCLE_CLASS[%zu]: success=%d top1=%d conf=%.3f "
        "top3=(%d:%.3f,%d:%.3f,%d:%.3f) err=%s",
        i,
        item.success ? 1 : 0,
        item.digit,
        item.confidence,
        top[0].first,
        top[0].second,
        top[1].first,
        top[1].second,
        top[2].first,
        top[2].second,
        item.error_message.c_str());
    }
  }

  void runCircleMarker(
    const sensor_msgs::msg::Image::SharedPtr & msg,
    const cv::Mat & image,
    cv::Mat & visualization,
    const rclcpp::Time & now)
  {
    agv_inventory_system::CircleMarkerResult circle_result;
    const bool recognized = circle_recognizer_.recognize(image, classifier_, circle_result);
    const cv::Mat circle_visualization =
      circle_result.visualization.empty() ? visualization : circle_result.visualization;

    float horizontal_offset = 0.0F;
    float estimated_distance = 0.0F;
    const cv::Rect marker_box = circle_result.digit_union_bbox.empty() ?
      circle_result.circle_bbox :
      circle_result.digit_union_bbox;
    if (circle_result.panel.success && !marker_box.empty() &&
      !circle_result.panel.inverse_warp_matrix.empty())
    {
      std::vector<cv::Point2f> warp_pts = {
        cv::Point2f(static_cast<float>(marker_box.x), static_cast<float>(marker_box.y)),
        cv::Point2f(static_cast<float>(marker_box.x + marker_box.width), static_cast<float>(marker_box.y)),
        cv::Point2f(static_cast<float>(marker_box.x + marker_box.width), static_cast<float>(marker_box.y + marker_box.height)),
        cv::Point2f(static_cast<float>(marker_box.x), static_cast<float>(marker_box.y + marker_box.height))};
      std::vector<cv::Point2f> src_pts;
      cv::perspectiveTransform(warp_pts, src_pts, circle_result.panel.inverse_warp_matrix);
      if (src_pts.size() == 4U) {
        float cx = 0.0F;
        float min_y = static_cast<float>(image.rows);
        float max_y = 0.0F;
        for (const auto & p : src_pts) {
          cx += p.x;
          min_y = std::min(min_y, p.y);
          max_y = std::max(max_y, p.y);
        }
        cx /= 4.0F;
        horizontal_offset = cx - static_cast<float>(image.cols) * 0.5F;
        const float pixel_height = std::max(0.0F, max_y - min_y);
        if (pixel_height > 1.0F) {
          const double known_height_m =
            (known_digit_height_px_ * known_distance_m_) /
            std::max(1e-6, distance_focal_length_);
          const double dist = (known_height_m * distance_focal_length_) / pixel_height;
          estimated_distance = static_cast<float>(std::max(0.0, dist));
        }
      }
    }

    const bool conf_ok = circle_result.confidence >= static_cast<float>(min_confidence_);
    const bool valid = recognized && circle_result.valid && conf_ok;
    log_circle_marker_diagnostics(circle_result, recognized, valid, conf_ok);
    if (valid) {
      attempts_ = 0;
    } else if (max_attempts_ > 0 && attempts_ >= max_attempts_) {
      attempts_ = 0;
    }

    publish_recognition(
      valid ? circle_result.number : std::string(""),
      valid ? circle_result.confidence : 0.0F,
      valid,
      horizontal_offset,
      estimated_distance);
    publish_image(
      debug_a4_pub_,
      msg->header,
      circle_result.debug_panel.empty() ? circle_result.panel.debug_image : circle_result.debug_panel);
    publish_image(debug_digits_pub_, msg->header, circle_result.debug_digits);
    publish_image(vis_pub_, msg->header, circle_visualization);

    const bool tracking_dist_fresh =
      std::isfinite(latest_tracking_distance_) &&
      (now - latest_tracking_distance_stamp_).seconds() <=
        std::max(0.05, distance_overlay_timeout_sec_);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1200,
      "circle_marker 识别结果: number=%s conf=%.3f valid=%d recognized=%d offset=%.1fpx "
      "dist_track=%.2fm dist_visual=%.2fm panel=%d circle=%dx%d digits=%zu err=%s",
      circle_result.number.c_str(),
      circle_result.confidence,
      valid ? 1 : 0,
      recognized ? 1 : 0,
      horizontal_offset,
      tracking_dist_fresh ? latest_tracking_distance_ : std::numeric_limits<double>::quiet_NaN(),
      estimated_distance,
      circle_result.panel.success ? 1 : 0,
      circle_result.circle_bbox.width,
      circle_result.circle_bbox.height,
      circle_result.digit_images.size(),
      circle_result.error_message.c_str());
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!enabled_) {
      return;
    }

    const auto now = this->now();
    if ((now - last_attempt_time_).seconds() < attempt_interval_) {
      return;
    }
    last_attempt_time_ = now;

    if (!classifier_.ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "分类模型未就绪，跳过识别");
      return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN(get_logger(), "图像转换失败: %s", e.what());
      return;
    }

    if (cv_ptr->image.empty()) {
      RCLCPP_WARN(get_logger(), "输入图像为空");
      return;
    }

    ++attempts_;

    cv::Mat visualization = cv_ptr->image.clone();
    if (recognition_target_style_ == "circle_marker") {
      runCircleMarker(msg, cv_ptr->image, visualization, now);
      return;
    }

    agv_inventory_system::A4DetectionResult a4_result;
    if (!detector_.detect(cv_ptr->image, a4_result)) {
      draw_failure_text(visualization, a4_result.error_message);
      publish_recognition("", 0.0F, false, 0.0F, 0.0F);
      publish_image(debug_a4_pub_, msg->header, a4_result.debug_image.empty() ? visualization : a4_result.debug_image);
      publish_image(vis_pub_, msg->header, visualization);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1500,
        "A4 检测失败: %s", a4_result.error_message.c_str());
      if (max_attempts_ > 0 && attempts_ >= max_attempts_) {
        attempts_ = 0;
      }
      return;
    }

    agv_inventory_system::DigitSegmentationResult seg_result;
    if (!segmenter_.segment(a4_result.warped_bgr, seg_result)) {
      draw_failure_text(visualization, seg_result.error_message);
      publish_recognition("", 0.0F, false, 0.0F, 0.0F);
      const cv::Mat a4_debug_fail =
        seg_result.boxed_debug.empty() ? a4_result.warped_bgr : seg_result.boxed_debug;
      publish_image(debug_a4_pub_, msg->header, a4_debug_fail);
      publish_image(debug_digits_pub_, msg->header, compose_segment_debug_view(seg_result));
      publish_image(vis_pub_, msg->header, visualization);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1500,
        "数字分割失败: %s seg[warped=%dx%d raw=%d area_ok=%d shape_ok=%d edge_ok=%d rel_ok=%d final=%d "
        "rej(edge=%d line=%d extent=%d area=%d size=%d aspect=%d) "
        "edge_reason(L=%d R=%d T=%d B=%d CX=%d CY=%d relaxed=%d fallback=%d) final_zero=%s]",
        seg_result.error_message.c_str(),
        seg_result.stats.warped_width,
        seg_result.stats.warped_height,
        seg_result.stats.raw_contours,
        seg_result.stats.area_pass_count,
        seg_result.stats.shape_pass_count,
        seg_result.stats.edge_pass_count,
        seg_result.stats.relative_pass_count,
        seg_result.stats.final_count,
        seg_result.stats.rejected_edge_count,
        seg_result.stats.rejected_line_count,
        seg_result.stats.rejected_extent_count,
        seg_result.stats.rejected_area_count,
        seg_result.stats.rejected_size_count,
        seg_result.stats.rejected_aspect_count,
        seg_result.stats.rejected_edge_left_count,
        seg_result.stats.rejected_edge_right_count,
        seg_result.stats.rejected_edge_top_count,
        seg_result.stats.rejected_edge_bottom_count,
        seg_result.stats.rejected_edge_center_x_count,
        seg_result.stats.rejected_edge_center_y_count,
        seg_result.stats.relaxed_edge_pass_count,
        seg_result.stats.relaxed_fallback_used ? 1 : 0,
        seg_result.final_zero_reason.c_str());
      log_segmentation_candidates(seg_result, "segment_fail");
      if (max_attempts_ > 0 && attempts_ >= max_attempts_) {
        attempts_ = 0;
      }
      return;
    }

    std::vector<cv::Mat> digit_images;
    digit_images.reserve(seg_result.digits.size());
    for (const auto & d : seg_result.digits) {
      digit_images.push_back(d.digit_gray);
    }

    const auto seq = classifier_.classify_sequence(digit_images);

    const cv::Rect raw_union_box = union_digit_boxes(seg_result.digits);
    const cv::Rect union_box = (
      seg_result.union_fallback_box.width > 0 && seg_result.union_fallback_box.height > 0) ?
      seg_result.union_fallback_box : raw_union_box;

    struct CandidateOption
    {
      std::string text;
      float confidence{0.0F};
      bool multi_digit{false};
      bool from_union{false};
      double area_ratio{1.0};
    };

    std::vector<CandidateOption> options;
    options.reserve(seg_result.digits.size() * 3 + 4);

    const auto max_digit_area = [&seg_result]() -> double {
      double max_area = 1.0;
      for (const auto & d : seg_result.digits) {
        max_area = std::max(max_area, static_cast<double>(d.bbox.area()));
      }
      return max_area;
    }();

    const auto area_ratio_of_index = [&seg_result, max_digit_area](std::size_t idx) -> double {
      if (idx >= seg_result.digits.size()) {
        return 0.2;
      }
      return std::clamp(
        static_cast<double>(seg_result.digits[idx].bbox.area()) / max_digit_area,
        0.0,
        1.0);
    };

    const auto push_option = [&options](
      const std::string & raw_text, float confidence, bool multi_digit, bool from_union, double area_ratio)
    {
      const std::string normalized = agv_inventory_system::normalize_cabinet_text(raw_text);
      if (normalized.empty()) {
        return;
      }

      CandidateOption opt;
      opt.text = normalized;
      opt.confidence = std::clamp(confidence, 0.0F, 1.0F);
      opt.multi_digit = multi_digit;
      opt.from_union = from_union;
      opt.area_ratio = std::clamp(area_ratio, 0.0, 1.0);
      options.push_back(std::move(opt));
    };

    if (!seq.number.empty()) {
      push_option(
        seq.number,
        seq.min_confidence,
        agv_inventory_system::normalize_cabinet_text(seq.number).size() >= 2U,
        false,
        1.0);
    }

    for (std::size_t i = 0; i < seq.items.size(); ++i) {
      if (seq.items[i].digit < 0) {
        continue;
      }
      push_option(
        std::to_string(seq.items[i].digit),
        seq.items[i].confidence,
        false,
        false,
        area_ratio_of_index(i));
    }

    if (seq.items.size() >= 2U) {
      for (std::size_t i = 0; i + 1 < seq.items.size(); ++i) {
        if (seq.items[i].digit < 0 || seq.items[i + 1].digit < 0) {
          continue;
        }
        const std::string pair_text =
          std::to_string(seq.items[i].digit) + std::to_string(seq.items[i + 1].digit);
        const float pair_conf = std::min(seq.items[i].confidence, seq.items[i + 1].confidence);
        const double pair_area_ratio = 0.5 * (area_ratio_of_index(i) + area_ratio_of_index(i + 1));
        push_option(pair_text, pair_conf, true, false, pair_area_ratio);
      }
    }

    const auto normalize_binary_patch = [](const cv::Mat & binary, int canvas_size) -> cv::Mat {
      if (binary.empty() || canvas_size <= 0) {
        return {};
      }
      cv::Mat mask;
      if (binary.type() != CV_8UC1) {
        cv::cvtColor(binary, mask, cv::COLOR_BGR2GRAY);
      } else {
        mask = binary.clone();
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
      canvas(cv::Rect(x, y, w, h)).setTo(0, resized);
      return canvas;
    };

    struct SplitUnionResult
    {
      std::vector<cv::Mat> patches;
      int split_x{-1};
      std::string reject_reason;
    };

    const auto split_union_patch = [this, &normalize_binary_patch](
      const cv::Mat & union_bin) -> SplitUnionResult
    {
      SplitUnionResult out;
      if (!seg_params_.enable_union_split || union_bin.empty() || union_bin.cols < 2) {
        out.reject_reason = "disabled_or_empty";
        return out;
      }
      if (cabinet_id_max_ < 10) {
        out.reject_reason = "single_digit_range";
        return out;
      }
      const double width_height_ratio =
        static_cast<double>(union_bin.cols) / static_cast<double>(std::max(1, union_bin.rows));
      if (width_height_ratio < std::max(0.1, seg_params_.union_split_min_width_height_ratio)) {
        out.reject_reason = "not_wide_enough";
        return out;
      }

      cv::Mat mask;
      if (union_bin.type() != CV_8UC1) {
        cv::cvtColor(union_bin, mask, cv::COLOR_BGR2GRAY);
      } else {
        mask = union_bin.clone();
      }
      cv::threshold(mask, mask, 0, 255, cv::THRESH_BINARY);

      const int min_x = std::max(1, static_cast<int>(std::round(mask.cols * 0.30)));
      const int max_x = std::min(mask.cols - 2, static_cast<int>(std::round(mask.cols * 0.70)));
      if (min_x >= max_x) {
        out.reject_reason = "bad_search_range";
        return out;
      }

      int best_x = mask.cols / 2;
      int best_count = std::numeric_limits<int>::max();
      int max_left_count = 0;
      int max_right_count = 0;
      for (int x = min_x; x <= max_x; ++x) {
        const int count = cv::countNonZero(mask.col(x));
        if (count < best_count) {
          best_count = count;
          best_x = x;
        }
      }
      for (int x = 0; x < best_x; ++x) {
        max_left_count = std::max(max_left_count, cv::countNonZero(mask.col(x)));
      }
      for (int x = best_x + 1; x < mask.cols; ++x) {
        max_right_count = std::max(max_right_count, cv::countNonZero(mask.col(x)));
      }
      const int side_peak = std::max(1, std::min(max_left_count, max_right_count));
      const double valley_ratio = static_cast<double>(best_count) / static_cast<double>(side_peak);
      if (valley_ratio > std::clamp(seg_params_.union_split_min_valley_ratio, 0.02, 0.95)) {
        out.reject_reason = "no_clear_valley";
        return out;
      }

      const cv::Rect left_rect(0, 0, best_x, mask.rows);
      const cv::Rect right_rect(best_x, 0, mask.cols - best_x, mask.rows);
      if (left_rect.width < seg_params_.min_digit_width || right_rect.width < seg_params_.min_digit_width) {
        out.reject_reason = "side_too_narrow";
        return out;
      }
      const int left_ink = cv::countNonZero(mask(left_rect));
      const int right_ink = cv::countNonZero(mask(right_rect));
      const int min_side_ink = std::max(8, static_cast<int>(std::round(mask.rows * mask.cols * 0.01)));
      if (left_ink < min_side_ink || right_ink < min_side_ink) {
        out.reject_reason = "side_ink_too_low";
        return out;
      }

      cv::Mat left_norm = normalize_binary_patch(mask(left_rect).clone(), seg_params_.digit_input_size);
      cv::Mat right_norm = normalize_binary_patch(mask(right_rect).clone(), seg_params_.digit_input_size);
      if (!left_norm.empty() && !right_norm.empty()) {
        out.patches.push_back(left_norm);
        out.patches.push_back(right_norm);
        out.split_x = best_x;
      } else {
        out.reject_reason = "normalize_failed";
      }
      return out;
    };

    const auto make_union_debug_montage = [](const std::vector<std::pair<std::string, cv::Mat>> & patches) -> cv::Mat {
      if (patches.empty()) {
        return {};
      }
      const int tile = std::max(16, patches.front().second.cols);
      const int margin = 6;
      cv::Mat out(tile + 2 * margin + 18, static_cast<int>(patches.size()) * (tile + margin) + margin,
        CV_8UC1, cv::Scalar(230));
      for (std::size_t i = 0; i < patches.size(); ++i) {
        if (patches[i].second.empty()) {
          continue;
        }
        cv::Mat resized;
        cv::resize(patches[i].second, resized, cv::Size(tile, tile), 0.0, 0.0, cv::INTER_NEAREST);
        resized.copyTo(out(cv::Rect(margin + static_cast<int>(i) * (tile + margin), margin, tile, tile)));
        cv::putText(
          out,
          patches[i].first,
          cv::Point(margin + static_cast<int>(i) * (tile + margin), tile + margin + 14),
          cv::FONT_HERSHEY_SIMPLEX,
          0.45,
          cv::Scalar(20),
          1,
          cv::LINE_AA);
      }
      return out;
    };

    CandidateOption union_option;
    bool has_union_option = false;
    bool union_fallback_used = false;
    cv::Mat union_norm_debug;

    if (!seg_result.binary_debug.empty() && union_box.width > 0 && union_box.height > 0) {
      const cv::Rect bounded_union = union_box & cv::Rect(0, 0, seg_result.binary_debug.cols, seg_result.binary_debug.rows);
      if (bounded_union.width > 0 && bounded_union.height > 0) {
        const cv::Mat union_bin = seg_result.binary_debug(bounded_union).clone();
        const cv::Mat union_norm = normalize_binary_patch(union_bin, seg_params_.digit_input_size);
        std::vector<std::pair<std::string, cv::Mat>> union_debug_patches;
        if (!union_norm.empty()) {
          union_debug_patches.emplace_back("union", union_norm);
          if (enable_union_fallback_classification_) {
            const auto union_cls = classifier_.classify_digit(union_norm);
            if (union_cls.digit >= 0) {
              const std::string normalized_union =
                agv_inventory_system::normalize_cabinet_text(std::to_string(union_cls.digit));
              if (!normalized_union.empty()) {
                union_option.text = normalized_union;
                union_option.confidence = std::clamp(union_cls.confidence, 0.0F, 1.0F);
                union_option.multi_digit = false;
                union_option.from_union = true;
                union_option.area_ratio = 1.0;
                has_union_option = true;
                push_option(normalized_union, union_option.confidence, false, true, 1.0);
              }
            }
          }
        }
        const auto split_result = split_union_patch(union_bin);
        if (enable_union_fallback_classification_ && split_result.patches.size() == 2U) {
          union_debug_patches.emplace_back("split L", split_result.patches[0]);
          union_debug_patches.emplace_back("split R", split_result.patches[1]);
          if (!seg_result.boxed_debug.empty() && split_result.split_x >= 0) {
            const int tile_w = std::max(1, seg_result.boxed_debug.cols / 2);
            const int tile_h = std::max(1, seg_result.boxed_debug.rows / 2);
            const int line_x = tile_w + static_cast<int>(std::round(
              static_cast<double>(bounded_union.x + split_result.split_x) *
              static_cast<double>(tile_w) / static_cast<double>(std::max(1, seg_result.binary_debug.cols))));
            const int line_y0 = tile_h + static_cast<int>(std::round(
              static_cast<double>(bounded_union.y) * static_cast<double>(tile_h) /
              static_cast<double>(std::max(1, seg_result.binary_debug.rows))));
            const int line_y1 = tile_h + static_cast<int>(std::round(
              static_cast<double>(bounded_union.y + bounded_union.height) * static_cast<double>(tile_h) /
              static_cast<double>(std::max(1, seg_result.binary_debug.rows))));
            cv::line(
              seg_result.boxed_debug,
              cv::Point(line_x, line_y0),
              cv::Point(line_x, line_y1),
              cv::Scalar(0, 255, 255),
              2);
            cv::putText(
              seg_result.boxed_debug,
              "split",
              cv::Point(line_x + 3, std::max(tile_h + 12, line_y0 + 14)),
              cv::FONT_HERSHEY_SIMPLEX,
              0.45,
              cv::Scalar(0, 255, 255),
              1,
              cv::LINE_AA);
          }
          const auto union_seq = classifier_.classify_sequence(split_result.patches);
          if (!union_seq.number.empty()) {
            push_option(
              union_seq.number,
              union_seq.min_confidence,
              agv_inventory_system::normalize_cabinet_text(union_seq.number).size() >= 2U,
              true,
              1.0);
          }
        } else if (!split_result.reject_reason.empty() && !seg_result.boxed_debug.empty()) {
          cv::putText(
            seg_result.boxed_debug,
            "split reject: " + split_result.reject_reason,
            cv::Point(8, std::min(seg_result.boxed_debug.rows - 8, 66)),
            cv::FONT_HERSHEY_SIMPLEX,
            0.45,
            cv::Scalar(0, 180, 255),
            1,
            cv::LINE_AA);
        }
        union_norm_debug = make_union_debug_montage(union_debug_patches);
      }
    }

    const auto option_in_range = [this](const CandidateOption & opt) -> bool {
      int id = -1;
      if (!agv_inventory_system::safe_to_int(opt.text, id)) {
        return false;
      }
      return id >= cabinet_id_min_ && id <= cabinet_id_max_;
    };

    const auto score_multi = [](const CandidateOption & opt) -> double {
      return static_cast<double>(opt.confidence) + 0.05 * opt.area_ratio + (opt.from_union ? 0.03 : 0.0);
    };

    const auto score_single = [](const CandidateOption & opt) -> double {
      return static_cast<double>(opt.confidence) + 0.20 * opt.area_ratio + (opt.from_union ? 0.08 : 0.0);
    };

    CandidateOption best;
    bool has_best = false;
    double best_score = -1.0;

    for (const auto & opt : options) {
      if (!opt.multi_digit || !option_in_range(opt)) {
        continue;
      }
      const double score = score_multi(opt);
      if (!has_best || score > best_score) {
        best = opt;
        best_score = score;
        has_best = true;
      }
    }

    if (!has_best) {
      for (const auto & opt : options) {
        if (opt.multi_digit || !option_in_range(opt)) {
          continue;
        }
        const double score = score_single(opt);
        if (!has_best || score > best_score) {
          best = opt;
          best_score = score;
          has_best = true;
        }
      }
    }

    if (!has_best) {
      for (const auto & opt : options) {
        const double score = static_cast<double>(opt.confidence);
        if (!has_best || score > best_score) {
          best = opt;
          best_score = score;
          has_best = true;
        }
      }
    }

    // 单框 union patch 仅作为最后兜底；union split 可参与双数字候选竞争。
    if (!has_best && has_union_option) {
      best = union_option;
      best_score = static_cast<double>(union_option.confidence);
      has_best = true;
      union_fallback_used = true;
    }

    if (!has_best) {
      draw_failure_text(visualization, seq.error_message.empty() ? "分类失败" : seq.error_message);
      publish_recognition("", 0.0F, false, 0.0F, 0.0F);
      publish_image(debug_a4_pub_, msg->header, seg_result.boxed_debug.empty() ? a4_result.warped_bgr : seg_result.boxed_debug);
      publish_image(debug_digits_pub_, msg->header, compose_segment_debug_view(seg_result));
      publish_image(vis_pub_, msg->header, visualization);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1500,
        "CNN 分类失败: %s", seq.error_message.c_str());
      if (max_attempts_ > 0 && attempts_ >= max_attempts_) {
        attempts_ = 0;
      }
      return;
    }

    const std::string normalized = best.text;
    const float conf = best.confidence;
    union_fallback_used = union_fallback_used || best.from_union;

    int cabinet_id = -1;
    const bool in_range =
      agv_inventory_system::safe_to_int(normalized, cabinet_id) &&
      cabinet_id >= cabinet_id_min_ &&
      cabinet_id <= cabinet_id_max_;

    const bool conf_ok = conf >= static_cast<float>(min_confidence_);
    std::vector<cv::Point2f> warp_pts = {
      cv::Point2f(static_cast<float>(union_box.x), static_cast<float>(union_box.y)),
      cv::Point2f(static_cast<float>(union_box.x + union_box.width), static_cast<float>(union_box.y)),
      cv::Point2f(static_cast<float>(union_box.x + union_box.width), static_cast<float>(union_box.y + union_box.height)),
      cv::Point2f(static_cast<float>(union_box.x), static_cast<float>(union_box.y + union_box.height))};

    std::vector<cv::Point2f> src_pts;
    if (!a4_result.inverse_warp_matrix.empty()) {
      cv::perspectiveTransform(warp_pts, src_pts, a4_result.inverse_warp_matrix);
    }

    float horizontal_offset = 0.0F;
    float estimated_distance = 0.0F;

    if (src_pts.size() == 4U) {
      float cx = 0.0F;
      float min_y = static_cast<float>(cv_ptr->image.rows);
      float max_y = 0.0F;

      for (const auto & p : src_pts) {
        cx += p.x;
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
      }

      cx /= 4.0F;
      horizontal_offset = cx - static_cast<float>(cv_ptr->image.cols) * 0.5F;

      const float pixel_height = std::max(0.0F, max_y - min_y);
      if (pixel_height > 1.0F) {
        const double known_height_m =
          (known_digit_height_px_ * known_distance_m_) /
          std::max(1e-6, distance_focal_length_);
        const double dist = (known_height_m * distance_focal_length_) / pixel_height;
        estimated_distance = static_cast<float>(std::max(0.0, dist));
      }

      for (int i = 0; i < 4; ++i) {
        const cv::Point2f p0 = src_pts[static_cast<std::size_t>(i)];
        const cv::Point2f p1 = src_pts[static_cast<std::size_t>((i + 1) % 4)];
        cv::line(visualization, p0, p1, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
      }
    }

    const bool valid = !normalized.empty() && in_range && conf_ok;

    if (valid) {
      attempts_ = 0;
    } else if (max_attempts_ > 0 && attempts_ >= max_attempts_) {
      attempts_ = 0;
    }

    // 绘制 A4 轮廓与文本。
    std::vector<cv::Point> a4_poly;
    for (const auto & c : a4_result.corners) {
      a4_poly.emplace_back(static_cast<int>(std::round(c.x)), static_cast<int>(std::round(c.y)));
    }
    if (a4_poly.size() == 4U) {
      cv::polylines(visualization, a4_poly, true, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }

    const bool tracking_dist_fresh =
      std::isfinite(latest_tracking_distance_) &&
      (now - latest_tracking_distance_stamp_).seconds() <=
        std::max(0.05, distance_overlay_timeout_sec_);
    const double display_distance = tracking_dist_fresh ?
      latest_tracking_distance_ :
      static_cast<double>(estimated_distance);
    const std::string dist_src = tracking_dist_fresh ? "track" : "vision";

    const std::string status_text =
      "num=" + (normalized.empty() ? std::string("-") : normalized) +
      " conf=" + std::to_string(conf).substr(0, 4) +
      " valid=" + std::string(valid ? "1" : "0") +
      " dist=" + std::to_string(display_distance).substr(0, 4) +
      "m src=" + dist_src;

    cv::putText(
      visualization,
      status_text,
      cv::Point(20, 40),
      cv::FONT_HERSHEY_SIMPLEX,
      0.8,
      valid ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255),
      2,
      cv::LINE_AA);

    const std::string dist_debug_text =
      "track=" +
      (tracking_dist_fresh ? std::to_string(latest_tracking_distance_).substr(0, 4) : std::string("na")) +
      " vis=" + std::to_string(estimated_distance).substr(0, 4);
    cv::putText(
      visualization,
      dist_debug_text,
      cv::Point(20, valid ? 70 : 100),
      cv::FONT_HERSHEY_SIMPLEX,
      0.65,
      cv::Scalar(255, 255, 0),
      2,
      cv::LINE_AA);

    if (!valid) {
      std::string reason;
      if (normalized.empty()) {
        reason = "NO_NUMBER";
      } else if (!in_range) {
        reason = "OUT_OF_RANGE";
      } else if (!conf_ok) {
        reason = "LOW_CONF";
      }
      cv::putText(
        visualization,
        "err=" + reason,
        cv::Point(20, 70),
        cv::FONT_HERSHEY_SIMPLEX,
        0.7,
        cv::Scalar(0, 140, 255),
        2,
        cv::LINE_AA);
    }

    publish_recognition(normalized, conf, valid, horizontal_offset, estimated_distance);

    const cv::Mat a4_debug = seg_result.boxed_debug.empty() ? a4_result.warped_bgr : seg_result.boxed_debug;
    publish_image(debug_a4_pub_, msg->header, a4_debug);
    publish_image(debug_digits_pub_, msg->header, compose_segment_debug_view(seg_result, union_norm_debug));
    publish_image(vis_pub_, msg->header, visualization);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1200,
      "识别结果: number=%s conf=%.3f valid=%d offset=%.1fpx dist_track=%.2fm dist_visual=%.2fm "
      "seg[warped=%dx%d raw=%d area_ok=%d shape_ok=%d edge_ok=%d rel_ok=%d final=%d "
      "rej(edge=%d line=%d extent=%d) edge_reason(L=%d R=%d T=%d B=%d CX=%d CY=%d relaxed=%d fallback=%d)] union=%d",
      normalized.c_str(),
      conf,
      valid ? 1 : 0,
      horizontal_offset,
      tracking_dist_fresh ? latest_tracking_distance_ : std::numeric_limits<double>::quiet_NaN(),
      estimated_distance,
      seg_result.stats.warped_width,
      seg_result.stats.warped_height,
      seg_result.stats.raw_contours,
      seg_result.stats.area_pass_count,
      seg_result.stats.shape_pass_count,
      seg_result.stats.edge_pass_count,
      seg_result.stats.relative_pass_count,
      seg_result.stats.final_count,
      seg_result.stats.rejected_edge_count,
      seg_result.stats.rejected_line_count,
      seg_result.stats.rejected_extent_count,
      seg_result.stats.rejected_edge_left_count,
      seg_result.stats.rejected_edge_right_count,
      seg_result.stats.rejected_edge_top_count,
      seg_result.stats.rejected_edge_bottom_count,
      seg_result.stats.rejected_edge_center_x_count,
      seg_result.stats.rejected_edge_center_y_count,
      seg_result.stats.relaxed_edge_pass_count,
      seg_result.stats.relaxed_fallback_used ? 1 : 0,
      union_fallback_used ? 1 : 0);
  }

  std::string camera_topic_;
  std::string active_camera_topic_;
  std::string left_camera_topic_;
  std::string right_camera_topic_;
  std::string target_cabinet_topic_;
  std::string route_waypoints_file_;
  std::string recognized_topic_;
  std::string debug_a4_topic_;
  std::string debug_digits_topic_;
  std::string visualization_topic_;
  std::string trigger_service_name_;
  std::string enable_control_topic_;
  std::string distance_overlay_topic_;
  std::string recognition_target_style_{"circle_marker"};

  bool enable_on_start_{true};
  bool enabled_{true};

  double min_confidence_{0.7};
  int max_attempts_{20};
  double attempt_interval_{0.1};
  double distance_overlay_timeout_sec_{0.8};
  bool enable_union_fallback_classification_{true};
  bool circle_marker_debug_log_enabled_{true};

  int cabinet_id_min_{1};
  int cabinet_id_max_{36};
  int current_target_cabinet_{-1};

  double known_digit_height_px_{40.0};
  double known_distance_m_{1.0};
  double distance_focal_length_{600.0};

  agv_inventory_system::A4DetectorParams a4_params_;
  agv_inventory_system::DigitSegmenterParams seg_params_;
  agv_inventory_system::DigitClassifierParams cls_params_;
  agv_inventory_system::CircleMarkerParams circle_params_;

  agv_inventory_system::A4Detector detector_;
  agv_inventory_system::DigitSegmenter segmenter_;
  agv_inventory_system::DigitClassifier classifier_;
  agv_inventory_system::CircleMarkerRecognizer circle_recognizer_;

  int attempts_{0};
  rclcpp::Time last_attempt_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_candidate_debug_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_circle_marker_debug_log_{0, 0, RCL_ROS_TIME};
  double latest_tracking_distance_{std::numeric_limits<double>::quiet_NaN()};
  rclcpp::Time latest_tracking_distance_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr target_cabinet_sub_;
  rclcpp::Publisher<agv_inventory_system::msg::RecognizedNumber>::SharedPtr recog_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_a4_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_digits_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr vis_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr trigger_srv_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  std::map<int, std::string> cabinet_entry_side_map_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NumberRecognizerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
