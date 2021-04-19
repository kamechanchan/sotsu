#ifndef MAKE_VOXEL_DATA_H
#define MAKE_VOXEL_DATA_H

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <vector>

typedef pcl::PointXYZ PointInT;

namespace make_voxel_data
{
class MakeVoxelData
{
public:
  MakeVoxelData(ros::NodeHandle& nh);
  bool runMakeVoxelData();
private:
  void doMinMax(const pcl::PointCloud<PointInT>::Ptr cloud);
  void makeVoxel(const pcl::PointCloud<PointInT>::Ptr cloud, const float& resol);
private:
  ros::NodeHandle nh_;
  int voxel_n_;
  int data_number_;
  double operation_time_;
  int max_data_;
  double min_x_;
  double min_y_;
  double min_z_;
  double max_x_;
  double max_y_;
  double max_z_;
  std::string package_name_;
  std::string object_name_;
  std::string dataset_name_;
  std::vector<int> voxel_;
};
} // namespace make_voxel_data

#endif // MAKE_VOXEL_DATA_H
