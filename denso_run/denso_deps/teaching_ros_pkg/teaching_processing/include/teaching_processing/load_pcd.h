#ifndef LOAD_PCD_H
#define LOAD_PCD_H

#include <ros/ros.h>

#include <string>

namespace load_pcd
{
class LoadPCD
{
public:
  LoadPCD(ros::NodeHandle& nh);
  void publishPCD(void);
private:
  ros::NodeHandle nh_;
  ros::Publisher pcd_pub_;
  std::string package_name_;
  std::string pcd_name_;
  std::string sensor_frame_;
  std::string output_pc_;
};
} // namespace load_pcd

#endif // LOAD_PCD_H