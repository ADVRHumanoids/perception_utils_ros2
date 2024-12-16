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

namespace perception_utils
{

class CombinedPointCloudToPCD : public rclcpp::Node
{
private:
  std::string prefix_;
  bool binary_;
  bool compressed_;
  bool rgb_;
  bool use_transform_;
  bool save_triggered_;
  std::string fixed_frame_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  pcl::PointCloud<pcl::PointXYZ> accumulated_cloud_;

public:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr save_timer_;

  void cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
  {
    if (cloud_msg->data.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Received empty point cloud message!");
      return;
    }

    Eigen::Vector4f v = Eigen::Vector4f::Zero();
    Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
    if (!fixed_frame_.empty()) {
      use_transform_ = false;
      try {
        geometry_msgs::msg::TransformStamped transform;
        transform = tf_buffer_.lookupTransform(
          fixed_frame_, cloud_msg->header.frame_id,
          cloud_msg->header.stamp);

        Eigen::Affine3d transform_eigen = tf2::transformToEigen(transform);
        v = Eigen::Vector4f::Zero();
        v.head<3>() = transform_eigen.translation().cast<float>();
        q = transform_eigen.rotation().cast<float>();
        use_transform_ = true;
      } catch (tf2::LookupException & ex) {
        RCLCPP_WARN(this->get_logger(), "skip transform: %s", ex.what());
      } catch (tf2::TransformException & ex) {
        RCLCPP_ERROR(this->get_logger(), "skip transform: %s", ex.what());
      }
    } else {
      use_transform_ = false;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    if (use_transform_) {
      transformPointCloud(cloud, cloud, v, q);
    }

    // Accumulate the point cloud
    accumulated_cloud_ += cloud;

    RCLCPP_INFO(this->get_logger(), "Accumulated cloud size: %lu", accumulated_cloud_.size());
  }

  template<typename T>
  void transformPointCloud(
    const pcl::PointCloud<T> & cloud_in, pcl::PointCloud<T> & cloud_out,
    const Eigen::Vector4f & v, const Eigen::Quaternionf & q)
  {
    cloud_out = cloud_in;
    for (size_t i = 0; i < cloud_in.size(); ++i) {
      Eigen::Vector3f pt = cloud_in[i].getVector3fMap();
      pt = q * pt + v.head<3>();
      cloud_out[i].x = pt[0];
      cloud_out[i].y = pt[1];
      cloud_out[i].z = pt[2];
    }
  }

  void saveAccumulatedCloud()
  {
    if (save_triggered_) return;

    save_triggered_ = true;  // Prevent multiple saves.

    std::string filename = prefix_ + "combined.pcd";
    RCLCPP_INFO(this->get_logger(), "Saving accumulated point cloud to %s", filename.c_str());

    pcl::PCDWriter writer;
    if (binary_) {
      if (compressed_) {
        writer.writeBinaryCompressed(filename, accumulated_cloud_);
      } else {
        writer.writeBinary(filename, accumulated_cloud_);
      }
    } else {
      writer.writeASCII(filename, accumulated_cloud_, 8);
    }
    save_triggered_ = true;  // Prevent multiple saves.
    RCLCPP_INFO(this->get_logger(), "Saved %lu points to %s", accumulated_cloud_.size(), filename.c_str());

    // Shut down the node after saving the cloud.
    RCLCPP_INFO(this->get_logger(), "Shutting down the node.");
    rclcpp::shutdown();
  }

  void checkAndSave()
  {
    if (!save_triggered_) {  // Example condition to save.
      saveAccumulatedCloud();
    }
  }

  ////////////////////////////////////////////////////////////////////////////////
  CombinedPointCloudToPCD(const rclcpp::NodeOptions & options)
  : rclcpp::Node("pointcloud_to_pcd", options),
    binary_(false), compressed_(false), rgb_(false), save_triggered_(false),
    tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    this->declare_parameter("prefix", prefix_);
    this->declare_parameter("fixed_frame", fixed_frame_);
    this->declare_parameter("binary", binary_);
    this->declare_parameter("compressed", compressed_);
    this->declare_parameter("rgb", rgb_);

    this->get_parameter("prefix", prefix_);
    this->get_parameter("fixed_frame", fixed_frame_);
    this->get_parameter("binary", binary_);
    this->get_parameter("compressed", compressed_);
    this->get_parameter("rgb", rgb_);

    auto sensor_qos = rclcpp::SensorDataQoS();
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input", sensor_qos,
      std::bind(&CombinedPointCloudToPCD::cloud_cb, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Initialized CombinedPointCloudToPCD node");

    // Timer to periodically check for save condition.
    save_timer_ = this->create_wall_timer(
      std::chrono::seconds(10),  // Adjust the time as needed.
      std::bind(&CombinedPointCloudToPCD::checkAndSave, this));

  }

  ~CombinedPointCloudToPCD()
  {
    if (!save_triggered_) {
      saveAccumulatedCloud();
    }
  }
};
}  // namespace perception_utils

RCLCPP_COMPONENTS_REGISTER_NODE(perception_utils::CombinedPointCloudToPCD)