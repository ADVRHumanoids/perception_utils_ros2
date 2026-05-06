/**
 * @file laserscan_multi_merger.hpp
 * @brief Declaration of a ROS 2 node that merges multiple LaserScan topics.
 *
 * This header declares the `LaserscanMerger` node, which subscribes to several
 * `sensor_msgs::msg::LaserScan` topics, projects each scan into a point cloud,
 * transforms all clouds into a common frame, merges them, and republishes both
 * the merged point cloud and a synthesized merged laser scan.
 */

#include <string.h>
#include <vector>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <laser_geometry/laser_geometry.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "pcl_ros/transforms.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std;
using namespace pcl;

namespace perception_utils{

/**
 * @class LaserscanMerger
 * @brief Fuses multiple laser scan streams into a merged scan and point cloud.
 *
 * The node discovers and subscribes to a configured set of laser scan topics,
 * converts each scan to a point cloud in a common destination frame, waits until
 * fresh data has been received from all configured inputs, and then generates:
 * - a merged `sensor_msgs::msg::PointCloud2` output
 * - a merged `sensor_msgs::msg::LaserScan` output derived from the fused cloud
 */
class LaserscanMerger : public rclcpp::Node
{
public:
	/**
	 * @brief Construct the laser scan merger node.
	 * @param options ROS 2 node options used to initialize the node.
	 */
	LaserscanMerger(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

	/**
	 * @brief Process a new scan from one subscribed topic.
	 * @param scan Incoming laser scan message.
	 * @param topic Topic name associated with the received message.
	 *
	 * The callback projects the scan to a point cloud, transforms it into the
	 * destination frame, stores it in the cache for that topic, and triggers a
	 * merge once all configured inputs have produced fresh data.
	 */
	void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan, std::string topic);

	/**
	 * @brief Convert a merged point set into a single laser scan output.
	 * @param points Matrix view of the merged point cloud.
	 * @param merged_cloud Pointer to the merged PCL cloud whose header is reused.
	 *
	 * For each angular bin, the closest valid point is selected and written into
	 * the outgoing `LaserScan` ranges vector.
	 */
	void pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud);

	/**
	 * @brief Handle dynamic updates to scan-generation parameters.
	 * @param parameters Set of parameters being updated at runtime.
	 * @return Result indicating whether the update was accepted.
	 */
	rcl_interfaces::msg::SetParametersResult reconfigureCallback(const std::vector<rclcpp::Parameter> &parameters);

private:
	/// Handle for the dynamic-parameter callback registration.
	rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

	/// Utility used to project `LaserScan` messages into `PointCloud2`.
	laser_geometry::LaserProjection projector_;

	/// TF buffer used to resolve transforms into the destination frame.
	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

	/// TF listener that populates the TF buffer from the ROS graph.
	std::shared_ptr<tf2_ros::TransformListener> tfListener_;

	/// Active callback registration for runtime parameter updates.
	OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

	/// Publisher for the fused point cloud output.
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;

	/// Publisher for the synthesized merged laser scan.
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher_;

	/// One subscription per configured input laser scan topic.
	std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> scan_subscribers;

	/// Tracks whether each cached cloud has been refreshed since the last merge.
	std::vector<bool> clouds_modified;

	/// Latest transformed point cloud generated from each input scan topic.
	std::vector<pcl::PCLPointCloud2> clouds;

	/// Ordered list of active input scan topics.
	std::vector<string> input_topics;

	/**
	 * @brief Discover configured scan topics and create subscriptions.
	 *
	 * The configured topic list is parsed from the `laserscan_topics` parameter.
	 * Subscriptions and per-topic cache storage are recreated only when the
	 * discovered input topic set changes.
	 */
	void laserscan_topic_parser();

	/// Minimum angle of the merged laser scan.
	double angle_min;

	/// Maximum angle of the merged laser scan.
	double angle_max;

	/// Angular resolution of the merged laser scan.
	double angle_increment;

	/// Time between consecutive range measurements.
	double time_increment;

	/// Full scan duration used in the published scan metadata.
	double scan_time;

	/// Minimum valid range in the published laser scan.
	double range_min;

	/// Maximum valid range and projection distance.
	double range_max;

	/// Common TF frame used for cloud transformation and fusion.
	string destination_frame;

	/// Output topic for the merged point cloud.
	string cloud_destination_topic;

	/// Output topic for the merged laser scan.
	string scan_destination_topic;

	/// Whitespace-separated list of scan topics to subscribe to.
	string laserscan_topics;
};
}
