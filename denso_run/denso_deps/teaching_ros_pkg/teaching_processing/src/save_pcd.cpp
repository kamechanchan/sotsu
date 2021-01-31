#include <teaching_processing/save_pcd.h>

#include <ros/package.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

using save_pcd::SavePCD;

SavePCD::SavePCD(ros::NodeHandle& nh) : nh_(nh)
{
  ros::param::param<std::string>("~package_name", package_name_, "teaching_processing");
  ros::param::param<std::string>("~pcd_name", pcd_name_, "test");
  ros::param::param<std::string>("~pc_src", pc_src_, "/photoneo_center/pointcloud");
  pc_sub_ = nh.subscribe(pc_src_, 1, &SavePCD::SavePCDCb, this);
}

void SavePCD::SavePCDCb(const sensor_msgs::PointCloud2& input)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(input, cloud);

  std::string pcd_file_path;
  pcd_file_path = ros::package::getPath(package_name_);
  pcl::io::savePCDFileASCII(pcd_file_path + "/data/" + pcd_name_ + ".pcd", cloud);
}