#include <vector>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include "pcl_ros/transforms.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
namespace ira_laser_tools{
class PointCloudMerger : public rclcpp::Node {
public:
    PointCloudMerger(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("pointcloud_merger", options) {
        // Declare and get parameters
        this->declare_parameter<std::string>("destination_frame", "base_link");
        this->declare_parameter<std::string>("cloud_destination_topic", "merged_cloud");
        this->declare_parameter<std::string>("pointcloud_topics", "");

        this->get_parameter("destination_frame", destination_frame_);
        this->get_parameter("cloud_destination_topic", destination_topic_);
        this->get_parameter("pointcloud_topics", pointcloud_topics_);

        if (pointcloud_topics_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No pointcloud topics provided. Exiting.");
            rclcpp::shutdown();
        }

        // Setup TF2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Parse and subscribe to topics
        parse_pointcloud_topics();

        // Create publisher for merged cloud
        pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(destination_topic_, 10);
    }

private:
    void parse_pointcloud_topics() {
        std::istringstream iss(pointcloud_topics_);
        std::vector<std::string> tmp_topics;
        std::string topic;

        // Split topic names
        while (std::getline(iss, topic, ' ')) {
            tmp_topics.push_back(topic);
        }

        if (tmp_topics.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No valid pointcloud topics found.");
            return;
        }

        // Subscribe to each topic
        for (const auto &topic : tmp_topics) {
            auto callback = [this, topic](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                this->pointcloud_callback(msg, topic);
            };
            auto sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(topic, 10, callback);
            subscribers_.push_back(sub);
            clouds_[topic] = pcl::PCLPointCloud2(); // Initialize empty cloud
        }
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, const std::string &topic) {
        pcl::PCLPointCloud2 pcl_cloud;
        //pcl_conversions::toPCL(*msg, pcl_cloud);

        try {
            // Transform cloud to destination frame
            sensor_msgs::msg::PointCloud2 transformed_cloud_msg;
            //auto transform = tf_buffer_->lookupTransform(destination_frame_.c_str(), msg->header.frame_id.c_str(), msg->header.stamp, rclcpp::Duration(1, 0));
            //pcl_ros::transformPointCloud(destination_frame_, transform, *msg, transformed_cloud_msg);
	        
            tf_buffer_->lookupTransform(msg->header.frame_id.c_str(), destination_frame_.c_str(), msg->header.stamp, rclcpp::Duration(1, 0));
            //projector_.transformLaserScanToPointCloud(scan->header.frame_id, *scan, tmpCloud1, *tf_buffer_, range_max);
		    pcl_ros::transformPointCloud(destination_frame_.c_str(), *msg, transformed_cloud_msg, *tf_buffer_);

            // Convert to PCL format and store
            pcl_conversions::toPCL(transformed_cloud_msg, pcl_cloud);
            
            clouds_[topic] = pcl_cloud;

            // Attempt to merge and publish
            merge_and_publish();
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Failed to transform pointcloud from %s to %s: %s",
                        msg->header.frame_id.c_str(), destination_frame_.c_str(), ex.what());
        }
    }

    void merge_and_publish() {
        pcl::PCLPointCloud2 merged_cloud;

        // Merge all available clouds
        for (const auto &[topic, cloud] : clouds_) {
            if (cloud.data.empty()) {
                continue; // Skip empty clouds
            }

#if PCL_VERSION_COMPARE(>=, 1, 10, 0)
            merged_cloud += cloud;
#else
            pcl::concatenatePointCloud(merged_cloud, cloud, merged_cloud);
#endif
        }

        // Publish merged cloud
        if (!merged_cloud.data.empty()) {
            sensor_msgs::msg::PointCloud2 output_cloud;
            pcl_conversions::fromPCL(merged_cloud, output_cloud);
            output_cloud.row_step = output_cloud.width * output_cloud.point_step;
            pointcloud_publisher_->publish(output_cloud);
        }
    }

    // Parameters
    std::string destination_frame_;
    std::string destination_topic_;
    std::string pointcloud_topics_;

    // Data structures
    std::map<std::string, pcl::PCLPointCloud2> clouds_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subscribers_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;

    // TF2
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(ira_laser_tools::PointCloudMerger)
