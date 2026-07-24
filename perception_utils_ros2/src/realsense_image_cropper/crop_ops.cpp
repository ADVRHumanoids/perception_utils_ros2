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
 * @file crop_ops.cpp
 * @brief Implements computeRoi and adjustCameraInfo.
 */
#include "realsense_image_cropper/crop_ops.hpp"

#include <algorithm>
#include <cmath>

namespace realsense_image_cropper
{

cv::Rect computeRoi(
  int image_width, int image_height, int requested_width, int requested_height,
  int offset_x, int offset_y)
{
  const int effective_width = std::clamp(requested_width, 1, image_width);
  const int effective_height = std::clamp(requested_height, 1, image_height);

  const double center_x = static_cast<double>(image_width) / 2.0 + offset_x;
  const double center_y = static_cast<double>(image_height) / 2.0 + offset_y;

  int x = static_cast<int>(std::lround(center_x - static_cast<double>(effective_width) / 2.0));
  int y = static_cast<int>(std::lround(center_y - static_cast<double>(effective_height) / 2.0));

  x = std::clamp(x, 0, image_width - effective_width);
  y = std::clamp(y, 0, image_height - effective_height);

  return {x, y, effective_width, effective_height};
}

sensor_msgs::msg::CameraInfo adjustCameraInfo(
  const sensor_msgs::msg::CameraInfo & info_in, const cv::Rect & roi)
{
  sensor_msgs::msg::CameraInfo info_out = info_in;

  info_out.width = static_cast<uint32_t>(roi.width);
  info_out.height = static_cast<uint32_t>(roi.height);

  // K is row-major 3x3: [fx 0 cx; 0 fy cy; 0 0 1]. Shift the principal point by the ROI
  // origin; focal lengths are unaffected by a pure crop (no rescaling).
  info_out.k[2] -= roi.x;
  info_out.k[5] -= roi.y;

  // P is row-major 3x4: [fx 0 cx Tx; 0 fy cy Ty; 0 0 1 0]. Mirror the same principal-point
  // shift; Tx/Ty (baseline terms) are unaffected by cropping.
  info_out.p[2] -= roi.x;
  info_out.p[6] -= roi.y;

  // The output is republished as a standalone full frame, not a sub-window of a larger one.
  info_out.roi.x_offset = 0;
  info_out.roi.y_offset = 0;
  info_out.roi.width = 0;
  info_out.roi.height = 0;
  info_out.roi.do_rectify = false;

  return info_out;
}

}  // namespace realsense_image_cropper
