#ifndef REGISTRATION_CLOUD_H
#define REGISTRATION_CLOUD_H

#include <eigen_conversions/eigen_msg.h>
#include <math.h>
#include <string>

#include <ros/package.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

namespace registration_cloud
{
class RegistrationCloud
{
public:
  RegistrationCloud(ros::NodeHandle nh, ros::NodeHandle n);
  void getSceneCloud(const sensor_msgs::PointCloud2::ConstPtr& source_pc);
  bool surfaceRemove(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, double threshold, double ratio);
  void downsampleCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output_cloud, float leaf);
  void loadModelCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, std::string direction);
  void transformCloudFrame(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud, std::string from_frame, std::string to_frame);
  void registrateCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr model_cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene_cloud, std::string method);
  void broadcastRegistratedPose();
  float checkUpsideDown();
  void publish();

private:
  ros::NodeHandle nh_;
  ros::Publisher transformed_cloud_pub_;
  ros::Publisher registrated_cloud_pub_;
  ros::Publisher registrated_tf_pub_;
  ros::Publisher center_tf_pub_;
  ros::Subscriber object_cloud_sub_;

  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster br_;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr up_model_cloud_;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr down_model_cloud_;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene_cloud_;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr converted_up_model_;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr converted_down_model_;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr converted_scene_;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr registrated_cloud_;

  std::string model_filename_;
  std::string algorithm_name_;
  std::string estimated_fraime_id_;
  std::string sensor_frame_id_;
  std::string registrated_frame_id_;
  float leaf_scene_;
  float leaf_model_;
  float model_plane_threshold_;  // HV6
  // float plane_threshold_ = 0.016; // HV8
  float scene_plane_threshold_;  // HV6
  // float scene_plane_threshold_ = 0.012; // HV8
  float model_planable_ratio_;
  float scene_planable_ratio_;
  bool cloud_flag_;

  Eigen::Matrix4d registrate_transform_;
};
}  // namespace registration_cloud

#endif // REGISTRATION_CLOUD_H
