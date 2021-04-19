#ifndef CROP_CLOUD_H
#define CROP_CLOUD_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl/io/io.h>
#include <denso_recognition_msgs/BoundingBoxArray.h>
#include <denso_recognition_msgs/InputCNN.h>
#include <denso_recognition_msgs/InputCNNArray.h>

namespace voxelize_cloud
{
class VoxelizeCloud
{
public:
  VoxelizeCloud(ros::NodeHandle nh, ros::NodeHandle n);
  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& source_pc);
  void boxCallback(const denso_recognition_msgs::BoundingBoxArray box_array);
  void cropBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointXYZRGB min, pcl::PointXYZRGB max);
  void makeVoxel(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float diff);

private:
  ros::NodeHandle nh_;
  ros::Rate rate_;
  std::string frame_id_;
  ros::Publisher filtered_cloud_pub_;
  ros::Publisher voxel_pub_;
  ros::Subscriber source_pc_sub_;
  ros::Subscriber box_sub_;
  tf::TransformListener tf_;
  denso_recognition_msgs::InputCNN input_data_;
  denso_recognition_msgs::InputCNNArray voxels_array_;
  denso_recognition_msgs::BoundingBoxArray box_array_;

  pcl::PointXYZRGB crop_min_, crop_max_;
  int voxel_n_ = 50;
};
}  // namespace voxelize_cloud

#endif  // CROP_CLOUD_H
