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
 * @file crop_ops.hpp
 * @brief Pure (no-ROS-node) helpers for computing and applying a clamped pixel ROI.
 */
#pragma once

#include <opencv2/core.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace realsense_image_cropper
{

/**
 * @brief Compute a pixel ROI centered at the image center plus a signed offset, clamped so
 * the window never exceeds or falls outside the image bounds.
 * @param image_width Source image width (px).
 * @param image_height Source image height (px).
 * @param requested_width Requested ROI width (px); capped to image_width.
 * @param requested_height Requested ROI height (px); capped to image_height.
 * @param offset_x Signed offset (px) of the ROI center from the image center (+x = right).
 * @param offset_y Signed offset (px) of the ROI center from the image center (+y = down).
 * @return ROI rectangle, fully contained within [0, image_width) x [0, image_height).
 */
cv::Rect computeRoi(
  int image_width, int image_height, int requested_width, int requested_height,
  int offset_x, int offset_y);

/**
 * @brief Build a CameraInfo for the cropped output: principal point shifted by the ROI
 * origin, focal lengths unchanged, width/height set to the ROI size.
 * @param info_in Source CameraInfo (uncropped).
 * @param roi ROI applied to the source image.
 * @return CameraInfo valid for the cropped image.
 */
sensor_msgs::msg::CameraInfo adjustCameraInfo(
  const sensor_msgs::msg::CameraInfo & info_in, const cv::Rect & roi);

}  // namespace realsense_image_cropper
