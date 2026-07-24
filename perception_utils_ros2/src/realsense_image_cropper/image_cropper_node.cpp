/**
 * Copyright 2026 The realsense_image_cropper authors.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/**
 * @file image_cropper_node.cpp
 * @brief Implements ImageCropperNode.
 */
#include "realsense_image_cropper/image_cropper_node.hpp"

#include <cv_bridge/cv_bridge.hpp>

#include "realsense_image_cropper/crop_ops.hpp"

namespace realsense_image_cropper
{

ImageCropperNode::ImageCropperNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("realsense_image_cropper", options)
{
  const auto color_topic = declare_parameter<std::string>(
    "input.color_topic", "/camera/camera/color/image_raw");
  const auto depth_topic = declare_parameter<std::string>(
    "input.depth_topic", "/camera/camera/aligned_depth_to_color/image_raw");
  const auto camera_info_topic = declare_parameter<std::string>(
    "input.camera_info_topic", "/camera/camera/color/camera_info");

  const auto out_color_topic = declare_parameter<std::string>(
    "output.color_topic", "~/cropped/color");
  const auto out_depth_topic = declare_parameter<std::string>(
    "output.depth_topic", "~/cropped/depth");
  const auto out_camera_info_topic = declare_parameter<std::string>(
    "output.camera_info_topic", "~/cropped/camera_info");

  crop_width_ = declare_parameter<int>("crop.width", crop_width_);
  crop_height_ = declare_parameter<int>("crop.height", crop_height_);
  offset_x_ = declare_parameter<int>("crop.offset_x", offset_x_);
  offset_y_ = declare_parameter<int>("crop.offset_y", offset_y_);

  const int queue_size = declare_parameter<int>("sync.queue_size", 10);
  const bool use_sensor_qos = declare_parameter<bool>("use_sensor_qos", true);

  const auto qos = use_sensor_qos ? rclcpp::SensorDataQoS() : rclcpp::QoS(10);
  const auto rmw_qos = qos.get_rmw_qos_profile();

  color_sub_.subscribe(this, color_topic, rmw_qos);
  depth_sub_.subscribe(this, depth_topic, rmw_qos);
  camera_info_sub_.subscribe(this, camera_info_topic, rmw_qos);
  sync_ = std::make_shared<Synchronizer>(
    SyncPolicy(queue_size), color_sub_, depth_sub_, camera_info_sub_);
  sync_->registerCallback(std::bind(
      &ImageCropperNode::onSynchronizedFrame, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  color_pub_ = create_publisher<sensor_msgs::msg::Image>(out_color_topic, qos);
  depth_pub_ = create_publisher<sensor_msgs::msg::Image>(out_depth_topic, qos);
  camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(out_camera_info_topic, qos);

  RCLCPP_INFO(
    get_logger(), "realsense_image_cropper: %s + %s + %s -> crop %dx%d @ offset(%d,%d)",
    color_topic.c_str(), depth_topic.c_str(), camera_info_topic.c_str(),
    crop_width_, crop_height_, offset_x_, offset_y_);
}

void ImageCropperNode::onSynchronizedFrame(
  const sensor_msgs::msg::Image::ConstSharedPtr & color,
  const sensor_msgs::msg::Image::ConstSharedPtr & depth,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info)
{
  cv_bridge::CvImageConstPtr color_cv;
  cv_bridge::CvImageConstPtr depth_cv;
  try {
    color_cv = cv_bridge::toCvShare(color);
    depth_cv = cv_bridge::toCvShare(depth);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge conversion failed: %s", e.what());
    return;
  }

  const cv::Rect roi = computeRoi(
    color_cv->image.cols, color_cv->image.rows, crop_width_, crop_height_, offset_x_, offset_y_);
  if (roi.width > depth_cv->image.cols || roi.height > depth_cv->image.rows) {
    RCLCPP_ERROR(get_logger(), "depth frame smaller than the computed ROI; skipping frame");
    return;
  }

  const cv::Mat cropped_color = cv::Mat(color_cv->image, roi).clone();
  const cv::Mat cropped_depth = cv::Mat(depth_cv->image, roi).clone();
  const auto cropped_info = adjustCameraInfo(*info, roi);

  color_pub_->publish(*cv_bridge::CvImage(color->header, color->encoding, cropped_color)
    .toImageMsg());
  depth_pub_->publish(*cv_bridge::CvImage(depth->header, depth->encoding, cropped_depth)
    .toImageMsg());
  camera_info_pub_->publish(cropped_info);
}

}  // namespace realsense_image_cropper

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(realsense_image_cropper::ImageCropperNode)
