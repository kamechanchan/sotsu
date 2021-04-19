#ifndef AR_CUBE_POINTCLOUD_EXTRACTOR_H
#define AR_CUBE_POINTCLOUD_EXTRACTOR_H

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

namespace ar_cube_pointcloud_extractor
{
class ARCubePointCloudExtractor
{
public:
  ARCubePointCloudExtractor(ros::NodeHandle& nh, ros::NodeHandle& n);
  void publishPointCloud(void);
  void extractPointCloud(const sensor_msgs::PointCloud2::ConstPtr phoxi_point);
  void tflistener(std::string target_frame, std::string source_frame);
  bool discriminatePoint(pcl::PointXYZ target_point, int index);

public:
  pcl::PointCloud<pcl::PointXYZ> extracted_points_;

private:
  void updateARTransform(void);

private:
  ros::NodeHandle nh_;
  ros::Subscriber point_sub_;
  std::string target_frame_;
  std::string source_frame_;
  XmlRpc::XmlRpcValue ar_indices_;
  std::vector<std::string> ar_indices_vec_;
  std::vector<std::string> ar_link_name_vector_;
  ros::Publisher extracted_pc_pub_;
  std::vector<tf::StampedTransform> ar_tf_vector_;
  tf::TransformListener tf_listener_;
  tf::TransformListener tf_listener__;
  tf::StampedTransform transform_;
  float cubemarker_size_;
  float reserve_area_;
};
struct ExtractorParam
{
  static const float DURATION_TIME;
  static const int START_TIME;
  static const int PUBLISHER_QUEUE_SIZE;
  static const int SUBSCRIBER_QUEUE_SIZE;
  static const int LOOP_RATE_TIME;
};
}  // namespace ar_cube_pointcloud_extractor

#endif  // AR_CUBE_POINTCLOUD_EXTRACTOR_H
