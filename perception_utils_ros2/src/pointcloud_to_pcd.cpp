/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file pointcloud_to_pcd.cpp
 * @author Extended by Valerio Passamano
 * @brief ROS 2 component that accumulates point clouds and saves them as one PCD file.
 *
 * The node subscribes to a single `PointCloud2` input stream, optionally transforms
 * each received cloud into a fixed TF frame, and appends the points to an internal
 * accumulated cloud. The accumulated data can then be saved automatically on a timer,
 * on demand through a service, or when the node shuts down.
 */

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <string>

#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <perception_utils_ros2/srv/save_map.hpp>

namespace perception_utils
{

class PointCloudToPCD : public rclcpp::Node
{
public:
  /**
   * @brief Construct the accumulation and PCD export component.
   * @param options ROS 2 node options used when loading the component.
   *
   * Declares runtime parameters, creates the point cloud subscription, and
   * optionally starts a save timer when `save_timer_sec` is greater than zero.
   */
  explicit PointCloudToPCD(const rclcpp::NodeOptions & options)
  : rclcpp::Node("pointcloud_to_pcd", options),
    binary_(false),
    compressed_(false),
    rgb_(false),
    use_transform_(false),
    save_on_shutdown_(true),
    fixed_frame_(""),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    save_triggered_(false)
  {
    // Declare parameters
    this->declare_parameter<std::string>("prefix", "");
    this->declare_parameter<std::string>("fixed_frame", "");
    this->declare_parameter<bool>("binary", false);
    this->declare_parameter<bool>("compressed", false);
    this->declare_parameter<bool>("rgb", false);
    this->declare_parameter<bool>("save_on_shutdown", true);
    this->declare_parameter<double>("save_timer_sec", 0.0);       // 0.0 = disabled

    // Retrieve parameter values
    prefix_ = this->get_parameter("prefix").as_string();
    fixed_frame_ = this->get_parameter("fixed_frame").as_string();
    binary_ = this->get_parameter("binary").as_bool();
    compressed_ = this->get_parameter("compressed").as_bool();
    rgb_ = this->get_parameter("rgb").as_bool();
    save_on_shutdown_ = this->get_parameter("save_on_shutdown").as_bool();
    const double save_timer_sec = this->get_parameter("save_timer_sec").as_double();

    RCLCPP_INFO(this->get_logger(), "prefix: %s", prefix_.c_str());
    RCLCPP_INFO(this->get_logger(), "fixed_frame: %s", fixed_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "binary: %s", binary_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "compressed: %s", compressed_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "rgb: %s", rgb_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "save_on_shutdown: %s", save_on_shutdown_ ? "true" : "false");
    if (save_timer_sec > 0.0) {
      RCLCPP_INFO(this->get_logger(), "PCD file will be automatically saved after %.2f seconds (save_timer_sec).", save_timer_sec);
    } else {
      RCLCPP_INFO(this->get_logger(), "PCD file will not be automatically saved. Will be saved on the shutdown of the node.");
    }

    // Create a subscription with reliable and transient local QoS to ensure we receive all clouds
    // even if the node starts after some messages have been published.
    rclcpp::QoS qos(10);
    qos.reliable();
    qos.transient_local();
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input", qos,
      std::bind(&PointCloudToPCD::cloudCb, this, std::placeholders::_1));

    save_map_service_ = this->create_service<perception_utils_ros2::srv::SaveMap>(
      "save_map",
      std::bind(&PointCloudToPCD::saveMapServiceCb, this, std::placeholders::_1, std::placeholders::_2));

    // Create a timer if the user wants a periodic check for saving
    if (save_timer_sec > 0.0) {
      save_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(save_timer_sec),
        std::bind(&PointCloudToPCD::checkAndSave, this));
    }

    RCLCPP_INFO(this->get_logger(), "Initialized PointCloudToPCD node");
  }

  ~PointCloudToPCD() override
  {
    // Optionally save on node shutdown
    if (!save_triggered_ && save_on_shutdown_) {
      RCLCPP_INFO(this->get_logger(), "Node is shutting down; saving accumulated cloud.");
      std::string ignored_filename;
      saveAccumulatedCloud("", false, true, &ignored_filename);
    }
  }

private:
  /// Prefix used to build the output PCD filename.
  std::string prefix_;

  /// When true, save the PCD file in binary format instead of ASCII.
  bool binary_;

  /// When true together with `binary_`, save using compressed binary PCD output.
  bool compressed_;

  /// Selects accumulation of `PointXYZRGB` data instead of `PointXYZ`.
  bool rgb_;

  /// Tracks whether the current callback successfully resolved a TF transform.
  bool use_transform_;

  /// Save the accumulated cloud in the destructor if no earlier automatic save occurred.
  bool save_on_shutdown_;

  /// Optional TF frame into which every input cloud should be transformed.
  std::string fixed_frame_;

  /// TF2 buffer used to query transforms for incoming clouds.
  tf2_ros::Buffer tf_buffer_;

  /// TF2 listener that fills the buffer with transforms from the ROS graph.
  tf2_ros::TransformListener tf_listener_;

  /// Subscription to the input point cloud stream.
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

  /// Service that saves the current accumulated map on demand.
  rclcpp::Service<perception_utils_ros2::srv::SaveMap>::SharedPtr save_map_service_;

  /// Optional timer that triggers automatic saving after a configured interval.
  rclcpp::TimerBase::SharedPtr save_timer_;

  /// Accumulator used when `rgb_` is false.
  pcl::PointCloud<pcl::PointXYZ> accumulated_cloud_xyz_;

  /// Accumulator used when `rgb_` is true.
  pcl::PointCloud<pcl::PointXYZRGB> accumulated_cloud_xyzrgb_;

  /// Prevents duplicate final saves after automatic or shutdown save paths.
  bool save_triggered_;

  /**
   * @brief Consume an input cloud and append it to the internal accumulator.
   * @param cloud_msg Incoming point cloud message from the `input` topic.
   *
   * The callback validates the message, optionally transforms it into
   * `fixed_frame_`, converts it into the selected PCL point type, and appends
   * its points to the corresponding accumulated cloud.
   */
  void cloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
  {
    if (cloud_msg->data.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty point cloud message. Skipping.");
      return;
    }

    // Attempt transform if fixed_frame_ is set
    Eigen::Vector4f translation = Eigen::Vector4f::Zero();
    Eigen::Quaternionf rotation = Eigen::Quaternionf::Identity();
    use_transform_ = false;

    if (!fixed_frame_.empty()) {
      try {
        geometry_msgs::msg::TransformStamped transform =
          tf_buffer_.lookupTransform(fixed_frame_, cloud_msg->header.frame_id, cloud_msg->header.stamp);

        Eigen::Affine3d transform_eigen = tf2::transformToEigen(transform);
        translation.head<3>() = transform_eigen.translation().cast<float>();
        rotation = transform_eigen.rotation().cast<float>();
        use_transform_ = true;
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Transform to frame '%s' failed: %s. Using original frame.",
          fixed_frame_.c_str(), ex.what());
        use_transform_ = false;
      }
    }

    // Convert ROS msg to PCL
    if (rgb_) {
      pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
      pcl::fromROSMsg(*cloud_msg, pcl_cloud);

      if (use_transform_) {
        transformPointCloud(pcl_cloud, translation, rotation);
      }
      accumulated_cloud_xyzrgb_ += pcl_cloud;
      RCLCPP_INFO(this->get_logger(), "Accumulated (XYZRGB) cloud size: %zu", accumulated_cloud_xyzrgb_.size());
    } else {
      pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
      pcl::fromROSMsg(*cloud_msg, pcl_cloud);

      if (use_transform_) {
        transformPointCloud(pcl_cloud, translation, rotation);
      }
      accumulated_cloud_xyz_ += pcl_cloud;
      RCLCPP_INFO(this->get_logger(), "Accumulated (XYZ) cloud size: %zu", accumulated_cloud_xyz_.size());
    }
  }

  /**
   * @brief Apply a rigid transform (translation + rotation) to an XYZ cloud in-place.
   * @param cloud_inout Point cloud to transform.
   * @param translation Translation vector expressed in the target frame.
   * @param rotation Rotation quaternion expressed in the target frame.
   */
  void transformPointCloud(
    pcl::PointCloud<pcl::PointXYZ> & cloud_inout,
    const Eigen::Vector4f & translation,
    const Eigen::Quaternionf & rotation)
  {
    for (auto & pt : cloud_inout.points) {
      Eigen::Vector3f v = rotation * pt.getVector3fMap() + translation.head<3>();
      pt.x = v.x();
      pt.y = v.y();
      pt.z = v.z();
    }
  }

  /**
   * @brief Apply a rigid transform (translation + rotation) to an XYZRGB cloud in-place.
   * @param cloud_inout Point cloud to transform.
   * @param translation Translation vector expressed in the target frame.
   * @param rotation Rotation quaternion expressed in the target frame.
   */
  void transformPointCloud(
    pcl::PointCloud<pcl::PointXYZRGB> & cloud_inout,
    const Eigen::Vector4f & translation,
    const Eigen::Quaternionf & rotation)
  {
    for (auto & pt : cloud_inout.points) {
      Eigen::Vector3f v = rotation * pt.getVector3fMap() + translation.head<3>();
      pt.x = v.x();
      pt.y = v.y();
      pt.z = v.z();
    }
  }

  /**
   * @brief Build the output filename for a manual or automatic save.
   * @param requested_name Requested output filename from the service.
   * @return Resolved path for the output PCD file.
   */
  std::string resolveOutputFilename(const std::string & requested_name) const
  {
    if (requested_name.empty()) {
      const auto now = this->now();
      const auto seconds = now.seconds();
      const auto whole_seconds = static_cast<int64_t>(seconds);
      const auto nanoseconds = static_cast<uint32_t>((seconds - static_cast<double>(whole_seconds)) * 1e9);

      std::ostringstream ss;
      ss << prefix_ << whole_seconds << "_" << std::setw(9) << std::setfill('0')
         << nanoseconds << ".pcd";
      return ss.str();
    }

    std::filesystem::path output_path(requested_name);
    if (output_path.extension() != ".pcd") {
      output_path += ".pcd";
    }

    if (output_path.is_relative()) {
      const std::filesystem::path prefix_path(prefix_);
      const std::filesystem::path prefix_dir = prefix_path.parent_path();
      if (!prefix_dir.empty()) {
        output_path = prefix_dir / output_path;
      }
    }

    return output_path.string();
  }

  /**
   * @brief Ensure that the parent output directory exists before saving.
   * @param filename Final output filename.
   * @return True when the directory exists or was created successfully.
   */
  bool ensureOutputDirectory(const std::string & filename)
  {
    const std::filesystem::path output_path(filename);
    const std::filesystem::path parent_dir = output_path.parent_path();
    if (parent_dir.empty()) {
      return true;
    }

    std::error_code ec;
    std::filesystem::create_directories(parent_dir, ec);
    if (ec) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Could not create output directory '%s': %s",
        parent_dir.string().c_str(),
        ec.message().c_str());
      return false;
    }

    return true;
  }

  /**
   * @brief Timer-based or event-based function to check if we need to save the cloud.
   *
   * If no save has happened yet, this method delegates to `saveAccumulatedCloud()`.
   */
  void checkAndSave()
  {
    if (!save_triggered_) {
      std::string ignored_filename;
      saveAccumulatedCloud("", true, true, &ignored_filename);
    }
  }

  /**
   * @brief Handles manual save requests.
   * @param req Request containing an optional filename.
   * @param res Response with success state and saved path.
   */
  void saveMapServiceCb(
    const perception_utils_ros2::srv::SaveMap::Request::SharedPtr req,
    perception_utils_ros2::srv::SaveMap::Response::SharedPtr res)
  {
    std::string saved_filename;
    const bool success = saveAccumulatedCloud(req->filename, false, false, &saved_filename);
    res->success = success;
    res->saved_path = success ? saved_filename : std::string();
    res->message = success ? "Map saved successfully" : "Failed to save accumulated map";
  }

  /**
   * @brief Writes the accumulated point cloud to disk in a single PCD file.
   *
   * @param requested_name Requested output filename, or empty to use the timestamped default.
   * @param shutdown_after_save If true, shut the node down after a successful save.
   * @param mark_saved If true, suppress later automatic or shutdown saves.
   * @param saved_filename Optional output parameter for the resolved filename.
   * @return True on a successful write.
   */
  bool saveAccumulatedCloud(
    const std::string & requested_name,
    bool shutdown_after_save,
    bool mark_saved,
    std::string * saved_filename)
  {
    const std::string filename = resolveOutputFilename(requested_name);
    if (saved_filename != nullptr) {
      *saved_filename = filename;
    }

    if (!ensureOutputDirectory(filename)) {
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Saving accumulated point cloud to: %s", filename.c_str());

    pcl::PCDWriter writer;

    try {
      if (rgb_) {
        if (accumulated_cloud_xyzrgb_.empty()) {
          RCLCPP_WARN(this->get_logger(), "No points in accumulated XYZRGB cloud to save.");
          return false;
        }

        if (binary_) {
          if (compressed_) {
            writer.writeBinaryCompressed(filename, accumulated_cloud_xyzrgb_);
          } else {
            writer.writeBinary(filename, accumulated_cloud_xyzrgb_);
          }
        } else {
          writer.writeASCII(filename, accumulated_cloud_xyzrgb_, 8);
        }
        RCLCPP_INFO(
          this->get_logger(),
          "Saved %zu XYZRGB points to %s",
          accumulated_cloud_xyzrgb_.size(),
          filename.c_str());
      } else {
        if (accumulated_cloud_xyz_.empty()) {
          RCLCPP_WARN(this->get_logger(), "No points in accumulated XYZ cloud to save.");
          return false;
        }

        if (binary_) {
          if (compressed_) {
            writer.writeBinaryCompressed(filename, accumulated_cloud_xyz_);
          } else {
            writer.writeBinary(filename, accumulated_cloud_xyz_);
          }
        } else {
          writer.writeASCII(filename, accumulated_cloud_xyz_, 8);
        }
        RCLCPP_INFO(
          this->get_logger(),
          "Saved %zu XYZ points to %s",
          accumulated_cloud_xyz_.size(),
          filename.c_str());
      }
    } catch (const pcl::IOException & ex) {
      RCLCPP_ERROR(this->get_logger(), "Failed to save PCD file '%s': %s", filename.c_str(), ex.what());
      return false;
    }

    if (mark_saved) {
      save_triggered_ = true;
    }

    if (shutdown_after_save) {
      RCLCPP_INFO(this->get_logger(), "Shutting down the node.");
      rclcpp::shutdown();
    }

    return true;
  }
};

}  // namespace perception_utils

// Register as a component
RCLCPP_COMPONENTS_REGISTER_NODE(perception_utils::PointCloudToPCD)
