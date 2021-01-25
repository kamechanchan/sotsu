#ifndef BULK_EXTRACTOR_H
#define BULK_EXTRACTOR_H

#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>

namespace set_initial_cloud
{
class SetInitialCloud
{
public:
  SetInitialCloud(ros::NodeHandle& nh, ros::NodeHandle& n);
  void saveCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

private:
  ros::NodeHandle nh_;
  ros::Subscriber cloud_sub_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr initial_cloud_;  // 出力点群

  double leaf_size_;
  std::string cloud_filename_;
};
}  // namespace bulk_extractor

#endif  // BULK_EXTRACTOR_H
