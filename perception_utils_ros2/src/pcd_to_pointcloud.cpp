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
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: pcd_to_pointcloud.cpp 33238 2010-03-11 00:46:58Z rusu $
 *
 */

/**

\author Radu Bogdan Rusu

@b pcd_to_pointcloud is a simple node that loads PCD (Point Cloud Data) files from disk and publishes them as ROS messages on the network.

 **/

#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <chrono>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_components/register_node_macro.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace perception_utils
{
class PCDToPointCloud : public rclcpp::Node
{
protected:
  std::string tf_frame_;

public:
  // ROS messages
  sensor_msgs::msg::PointCloud2 cloud_;

  std::string file_name_, cloud_topic_;
  size_t period_ms_;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  ////////////////////////////////////////////////////////////////////////////////
  explicit PCDToPointCloud(const rclcpp::NodeOptions & options)
  : rclcpp::Node("pcd_to_pointcloud", options), tf_frame_("/base_link")
  {
    // Maximum number of outgoing messages to be queued for delivery to subscribers = 1

    cloud_topic_ = this->declare_parameter<std::string>("cloud_topic", "cloud_pcd");
    tf_frame_ = this->declare_parameter("tf_frame", tf_frame_);
    period_ms_ = this->declare_parameter("publishing_period_ms", 3000);
    file_name_ = this->declare_parameter<std::string>("file_name");
    float downsampling_resolution_ = this->declare_parameter<float>("downsampling_resolution", 1.0);

    if(downsampling_resolution_ <= 0.0 || downsampling_resolution_ > 1.0) {
      RCLCPP_WARN(this->get_logger(), "Invalid downsampling ratio %f. It should be in the range (0.0, 1.0]. Defaulting to 1.0 (no downsampling).", downsampling_resolution_);
      downsampling_resolution_ = 1.0;
    }

    if (file_name_ == "" || pcl::io::loadPCDFile(file_name_, cloud_) == -1) {
      RCLCPP_ERROR(this->get_logger(), "failed to open PCD file");
      throw std::runtime_error{"could not open pcd file"};
    }
    cloud_.header.frame_id = tf_frame_;

    // Apply downsampling
    if (downsampling_resolution_ < 1.0) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(cloud_, *cloud);

      pcl::VoxelGrid<pcl::PointXYZ> sor;
      sor.setInputCloud(cloud);
      sor.setLeafSize(downsampling_resolution_, downsampling_resolution_, downsampling_resolution_);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
      sor.filter(*cloud_filtered);

      pcl::toROSMsg(*cloud_filtered, cloud_);
    }
    int nr_points = cloud_.width * cloud_.height;

    auto fields_list = pcl::getFieldsList(cloud_);
    auto resolved_cloud_topic =
      this->get_node_topics_interface()->resolve_topic_name(cloud_topic_);

    // Enable QoS reconfigurability via parameters
    auto pub_options = rclcpp::PublisherOptions();
    pub_options.qos_overriding_options =
      rclcpp::QosOverridingOptions {{
      rclcpp::QosPolicyKind::History,
      rclcpp::QosPolicyKind::Reliability,
      rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::Depth
    }};
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, 10, pub_options);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms_),
      [this]() {
        this->publish();
      });

    RCLCPP_INFO(
      this->get_logger(),
      "Publishing data with %d points (%s) on topic %s in frame %s.",
      nr_points,
      fields_list.c_str(),
      resolved_cloud_topic.c_str(),
      cloud_.header.frame_id.c_str());
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Publish callback that is periodically called by a timer.
  void publish()
  {
    cloud_.header.stamp = this->get_clock()->now();
    pub_->publish(cloud_);
  }
};
}  // namespace perception_utils

RCLCPP_COMPONENTS_REGISTER_NODE(perception_utils::PCDToPointCloud)