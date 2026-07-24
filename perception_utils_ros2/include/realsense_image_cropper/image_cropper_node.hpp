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
 * @file image_cropper_node.hpp
 * @brief Crops and republishes a synchronized RealSense color + aligned-depth + CameraInfo
 * stream to a fixed pixel ROI.
 */
#pragma once

#include <memory>

#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/exact_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace realsense_image_cropper
{

/**
 * @brief Node that synchronizes RealSense color + aligned-depth + CameraInfo, crops all three
 * to a common pixel ROI (offset from center, clamped to the image), and republishes them with
 * a corrected CameraInfo so downstream depth deprojection stays valid.
 */
class ImageCropperNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct the node, declare parameters, and wire the synchronized subscription.
   * @param options ROS node options (used for component loading).
   */
  explicit ImageCropperNode(const rclcpp::NodeOptions & options);

private:
  /**
   * @brief Crop the synchronized frame set and publish the cropped color/depth/camera_info.
   * @param color Synchronized color image.
   * @param depth Synchronized aligned-depth image.
   * @param info Synchronized camera info (shared by color and aligned depth).
   */
  void onSynchronizedFrame(
    const sensor_msgs::msg::Image::ConstSharedPtr & color,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info);

  using SyncPolicy = message_filters::sync_policies::ExactTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  message_filters::Subscriber<sensor_msgs::msg::Image> color_sub_;  ///< Color input.
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;  ///< Aligned-depth input.
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub_;  ///< Intrinsics.
  std::shared_ptr<Synchronizer> sync_;  ///< Exact-time synchronizer over the three inputs.

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;  ///< Cropped color output.
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;  ///< Cropped depth output.
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;  ///< Corrected info.

  int crop_width_{256};  ///< Requested ROI width (px); capped to the image size.
  int crop_height_{256};  ///< Requested ROI height (px); capped to the image size.
  int offset_x_{0};  ///< Signed ROI center offset from the image center (px, +x = right).
  int offset_y_{0};  ///< Signed ROI center offset from the image center (px, +y = down).
};

}  // namespace realsense_image_cropper
