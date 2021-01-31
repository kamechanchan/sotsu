#ifndef FILTER_H
#define FILTER_H

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace stat_outlier_removal
{
class Filter
{
public:
  Filter(ros::NodeHandle& nh, ros::NodeHandle& n);
  void cloudCB(const sensor_msgs::PointCloud2& input);

protected:
  ros::NodeHandle nh_;
  ros::Subscriber pcl_sub_;
  ros::Publisher pcl_pub_;
  std::string filter_pc_src_;
  std::string filter_pc_dst_;
  std::string filter_frame_id_;
  int filter_mean_k_;
  float filter_stddev_mul_thresh_;
};
}  // namespace stat_outlier_removal

#endif  // FILTER_H
