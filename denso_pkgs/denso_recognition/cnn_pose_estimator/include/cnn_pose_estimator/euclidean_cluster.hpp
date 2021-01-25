#ifndef EUCLIDEAN_CLUSTER_H
#define EUCLIDEAN_CLUSTER_H

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/io.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <denso_recognition_msgs/BoundingBoxArray.h>

namespace cluster_cloud
{
class ClusterCloud
{
public:
  ClusterCloud(ros::NodeHandle nh, ros::NodeHandle n);
  void euclideanCallback(const sensor_msgs::PointCloud2::ConstPtr& source_pc);
  void cropBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ min, pcl::PointXYZ max);
  void clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  bool minAreaRect(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int cluster_cnt);
  bool momentOfInertia_AABB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int cluster_cnt);
  denso_recognition_msgs::BoundingBox momentOfInertia_OBB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

private:
  ros::NodeHandle nh_;
  ros::Rate rate_;
  std::string frame_id_;
  ros::Publisher fileterd_cloud_pub_;
  ros::Publisher euclidean_cluster_pub_;
  ros::Subscriber source_pc_sub_;
  tf::TransformListener tf_;
  tf::TransformBroadcaster br_;

  denso_recognition_msgs::BoundingBox box_;

  // Threshold
  double clusterTolerance_;
  int minSize_;
  int maxSize_;

  pcl::PointXYZ crop_min_, crop_max_;
  float min_height_;
};
}  // namespace cluster_cloud

#endif  // EUCLIDEAN_CLUSTER_H
