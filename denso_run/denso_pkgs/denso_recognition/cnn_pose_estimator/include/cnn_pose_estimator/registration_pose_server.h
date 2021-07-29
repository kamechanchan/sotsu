#ifndef REGISTRATION_POSE_ESTIMATOR_H
#define REGISTRATION_POSE_ESTIMATOR_H

#include <eigen_conversions/eigen_msg.h>
#include <math.h>
#include <string>

#include <ros/package.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/PointCloud2.h>
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

#include <denso_recognition_srvs/RegistrationPose.h>

typedef pcl::PointXYZ PointInT;

namespace registration_pose_server
{
class RegistrationPoseServer
{
public:
  RegistrationPoseServer(ros::NodeHandle nh, ros::NodeHandle n);
  void publishRegistratedCloudAndTF();

private:
  void getSceneCloud(const sensor_msgs::PointCloud2::ConstPtr& source_pc);
  bool surfaceRemove(pcl::PointCloud<PointInT>::Ptr cloud, double threshold, double ratio);
  void downsampleCloud(pcl::PointCloud<PointInT>::Ptr input_cloud, pcl::PointCloud<PointInT>::Ptr output_cloud,
                       float leaf);
  void loadModelCloud(pcl::PointCloud<PointInT>::Ptr cloud, std::string direction);
  void transformCloudFrame(pcl::PointCloud<PointInT>::Ptr input_cloud, pcl::PointCloud<PointInT>::Ptr transformed_cloud,
                           std::string from_frame, std::string to_frame);
  void registrateCloud(pcl::PointCloud<PointInT>::Ptr model_cloud, pcl::PointCloud<PointInT>::Ptr scene_cloud,
                       std::string method);
  bool broadcastRegistratedPose();
  float checkUpsideDown();
  bool registratePose(denso_recognition_srvs::RegistrationPose::Request& req,
                      denso_recognition_srvs::RegistrationPose::Response& res);

private:
  ros::NodeHandle nh_;
  ros::Publisher transformed_cloud_pub_;
  ros::Publisher registrated_cloud_pub_;
  ros::Subscriber object_cloud_sub_;
  ros::ServiceServer registrate_pose_server_;

  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster br_;
  tf2_ros::TransformBroadcaster br2_;

  pcl::PointCloud<PointInT>::Ptr up_model_cloud_;
  pcl::PointCloud<PointInT>::Ptr down_model_cloud_;
  pcl::PointCloud<PointInT>::Ptr scene_cloud_;
  pcl::PointCloud<PointInT>::Ptr converted_up_model_;
  pcl::PointCloud<PointInT>::Ptr converted_down_model_;
  pcl::PointCloud<PointInT>::Ptr converted_scene_;
  pcl::PointCloud<PointInT>::Ptr registrated_cloud_;

  std::string model_filename_;
  std::string algorithm_name_;
  std::string estimated_frame_id_;
  std::string sensor_frame_id_;
  std::string registrated_frame_id_;
  int max_icp_iteration_;
  float leaf_scene_;
  float leaf_model_;
  float model_plane_threshold_;
  float scene_plane_threshold_;
  float model_planable_ratio_;
  float scene_planable_ratio_;
  bool is_ok_;
  bool use_remove_plane_;
  sensor_msgs::PointCloud2 model_cloud_;
  sensor_msgs::PointCloud2 transformed_cloud_;
  geometry_msgs::TransformStamped registrated_frame_transform_stamped_;

  Eigen::Matrix4d registrate_transform_;
};
}  // namespace registration_pose_estimator

#endif  // REGISTRATION_POSE_ESTIMATOR_H
