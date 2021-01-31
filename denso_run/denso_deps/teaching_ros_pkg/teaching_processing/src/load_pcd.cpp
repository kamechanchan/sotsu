#include <teaching_processing/load_pcd.h>

#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

using load_pcd::LoadPCD;

LoadPCD::LoadPCD(ros::NodeHandle& nh) : nh_(nh)
{
  ros::param::param<std::string>("~pcd_name", pcd_name_, "test");
  ros::param::param<std::string>("~sensor_frame", sensor_frame_, "photoneo_center_optical_frame");
  ros::param::param<std::string>("~output_pc", output_pc_, "pcd_data");
  ros::param::param<std::string>("~package_name", package_name_, "teaching_processing");
  pcd_pub_ = nh.advertise<sensor_msgs::PointCloud2>(output_pc_, 1);
}

void LoadPCD::publishPCD(void)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::PointCloud2 output;

  std::string pcd_file_path;
  pcd_file_path = ros::package::getPath(package_name_);

  pcl::io::loadPCDFile(pcd_file_path + "/data/" + pcd_name_ + ".pcd", cloud);
  pcl::toROSMsg(cloud, output);
  output.header.frame_id = sensor_frame_;

  pcd_pub_.publish(output);
}