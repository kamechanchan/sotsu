#ifndef CROP_CLOUD_H
#define CROP_CLOUD_H

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>

namespace crop_cloud
{
class CropCloud
{
public:
  CropCloud(ros::NodeHandle nh, ros::NodeHandle n);
  void cropCallback(const sensor_msgs::PointCloud2::ConstPtr& source_pc);
  void cropBox(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const pcl::PointXYZRGB min,
               const pcl::PointXYZRGB max);
  void cropBox(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointXYZ min, const pcl::PointXYZ max);
  void run();

private:
  ros::NodeHandle nh_;
  ros::Rate rate_;
  std::string frame_id_;
  ros::Publisher fileterd_cloud_pub_;
  ros::Subscriber source_pc_sub_;
  tf::TransformListener tf_;
  bool use_rgb_;

  pcl::PointXYZRGB crop_min_rgb_, crop_max_rgb_;
  pcl::PointXYZ crop_min_, crop_max_;
};
}  // namespace crop_cloud

#endif  // CROP_CLOUD_H
