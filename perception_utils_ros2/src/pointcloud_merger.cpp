#include <string>
#include <sstream>
#include <map>
#include <vector>
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
/**
 * @file pointcloud_merger.cpp
 * @author Extended by Valerio Passamano
 * @brief ROS 2 component that subscribes to multiple PointCloud2 topics, transforms
 * each cloud into a common frame, and republishes a merged cloud.
 *
 * The node keeps the latest transformed cloud received from each configured input
 * topic. Every time a new cloud arrives, it updates that topic's cached cloud,
 * concatenates all currently available clouds, and publishes the result on a single
 * output topic. This makes it suitable for fusing multiple lidar or depth sensors
 * into a combined perception stream.
 */

namespace perception_utils {

/**
 * @class PointCloudMerger
 * @brief Merges multiple `sensor_msgs::msg::PointCloud2` streams into one topic.
 *
 * The node is configured through a whitespace-separated list of input topics,
 * a destination TF frame, and an output topic name. Incoming clouds are
 * transformed into the destination frame with TF2 before being stored and
 * merged. The node always publishes the latest known combination of all
 * non-empty cached clouds.
 */
class PointCloudMerger : public rclcpp::Node {
public:
    /**
     * @brief Construct the point cloud merger component.
     * @param options ROS 2 node options used when loading the component.
     *
     * Declares parameters, creates the TF listener, subscribes to the configured
     * input topics, and initializes the merged cloud publisher.
     */
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
    /**
     * @brief Parse the `pointcloud_topics` parameter and create subscriptions.
     *
     * The parameter is expected to be a whitespace-separated list of topic names.
     * One subscription and one cached `pcl::PCLPointCloud2` entry are created for
     * each parsed topic.
     */
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

    /**
     * @brief Process a new cloud from one of the configured input topics.
     * @param msg Incoming point cloud message.
     * @param topic Name of the subscription topic that produced the message.
     *
     * The callback transforms the cloud into the configured destination frame,
     * converts it into PCL's serialized cloud format, stores it as the latest
     * cloud for that topic, and triggers a merge/publish cycle.
     */
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

    /**
     * @brief Merge all cached clouds and publish the combined result.
     *
     * Empty cache entries are ignored. The published cloud contains the
     * concatenation of all transformed clouds that have been received at least once.
     */
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

    /// Destination TF frame used for all merged point clouds.
    std::string destination_frame_;

    /// Output topic where the merged `PointCloud2` message is published.
    std::string destination_topic_;

    /// Whitespace-separated list of subscribed input point cloud topics.
    std::string pointcloud_topics_;

    /// Latest transformed cloud cached for each input topic.
    std::map<std::string, pcl::PCLPointCloud2> clouds_;

    /// Subscriptions that keep each configured input topic active.
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subscribers_;

    /// Publisher for the merged point cloud output.
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;

    /// TF2 buffer used to look up transforms between source and destination frames.
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    /// TF2 listener that populates the transform buffer from the ROS graph.
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
}  // namespace perception_utils

RCLCPP_COMPONENTS_REGISTER_NODE(perception_utils::PointCloudMerger)
