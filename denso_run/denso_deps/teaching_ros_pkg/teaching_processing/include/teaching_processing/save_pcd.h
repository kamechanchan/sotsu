#ifndef SAVE_PCD_H
#define SAVE_PCD_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>

namespace save_pcd
{
class SavePCD
{
public:
  SavePCD(ros::NodeHandle& nh);
private:
  void SavePCDCb(const sensor_msgs::PointCloud2& input);
private:
  ros::NodeHandle nh_;
  ros::Subscriber pc_sub_;
  std::string package_name_;
  std::string pc_src_;
  std::string pcd_name_;
};
} // namespace save_pcd

#endif // SAVE_PCD_H