#ifndef ESTIMATE_OBJECT_POSE_SERVER_H
#define ESTIMATE_OBJECT_POSE_SERVER_H

#include <ros/ros.h>

#include <denso_recognition_srvs/PoseEstimate.h>

namespace estimate_object_pose_server
{
class EstimateObjectPoseServer
{
public:
  EstimateObjectPoseServer(ros::NodeHandle& nh);

private:
  bool estimateObjectPose(denso_recognition_srvs::PoseEstimate::Request& req,
                          denso_recognition_srvs::PoseEstimate::Response& ret);
  bool estimateObjectPoseOnceGetCloud(denso_recognition_srvs::PoseEstimate::Request& req,
                                      denso_recognition_srvs::PoseEstimate::Response& ret);

private:
  ros::NodeHandle nh_;
  ros::ServiceServer estimate_object_pose_server_;
  ros::ServiceServer estimate_object_pose_once_get_cloud_server_;
  ros::ServiceClient get_sensor_frame_client_;
  ros::ServiceClient crop_cloud_client_;
  ros::ServiceClient region_growing_get_top_cluster_client_;
  ros::ServiceClient region_growing_get_cluster_client_;
  ros::ServiceClient region_growing_set_target_cluster_client_;
  ros::ServiceClient make_input_client_;
  ros::ServiceClient cnn_estimator_client_;
  ros::ServiceClient registrate_pose_client_;
  ros::ServiceClient inverse_pose_client_;
  bool use_clustering_;
  bool have_pointcloud_;
  double interval_time_;
};
}  // namespace estimate_object_pose_server

#endif  // ESTIMATE_OBJECT_POSE_SERVER_H