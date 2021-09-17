#ifndef DOWNSAMPLING_H
#define DOWNSAMPLING_H

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace downsampling
{
class Downsampling
{
public:
  Downsampling(ros::NodeHandle& nh, ros::NodeHandle& n);
  void cloudCB(const sensor_msgs::PointCloud2& input);

protected:
  ros::NodeHandle nh_;
  ros::Subscriber pcl_sub_;
  ros::Publisher pcl_pub_;
  std::string downsample_pc_src_;
  std::string downsample_pc_dst_;
  std::string downsample_frame_id_;
  float downsample_leafsize_x_;
  float downsample_leafsize_y_;
  float downsample_leafsize_z_;
};
}  // namespace downsampling

#endif  // DOWNSAMPLING_H
