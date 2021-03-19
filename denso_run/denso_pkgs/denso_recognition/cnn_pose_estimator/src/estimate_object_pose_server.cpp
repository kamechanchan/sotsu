#include <cnn_pose_estimator/estimate_object_pose_server.h>

#include <phoxi_camera_srvs/PublishPointCloud.h>
#include <denso_recognition_srvs/CropCloud.h>
#include <denso_recognition_srvs/ExtractObject.h>
#include <denso_recognition_srvs/MakeInput.h>
#include <denso_recognition_srvs/RegistrationPose.h>
#include <denso_recognition_srvs/InversePose.h>

using estimate_object_pose_server::EstimateObjectPoseServer;

EstimateObjectPoseServer::EstimateObjectPoseServer(ros::NodeHandle& nh) : nh_(nh), have_pointcloud_(false)
{
  ros::param::param<bool>("~use_clutering", use_clustering_, true);
  ros::param::param<double>("~interval_time", interval_time_, 2.0);
  estimate_object_pose_server_ = nh.advertiseService("/cnn_pose_estimator/estimate_object_pose",
                                                     &EstimateObjectPoseServer::estimateObjectPose, this);
  estimate_object_pose_once_get_cloud_server_ =
      nh.advertiseService("/cnn_pose_estimator/estimate_object_pose_once_get_cloud",
                          &EstimateObjectPoseServer::estimateObjectPoseOnceGetCloud, this);
  get_sensor_frame_client_ = nh.serviceClient<phoxi_camera_srvs::PublishPointCloud>("/phoxi_bringup/get_pointcloud");
  crop_cloud_client_ = nh.serviceClient<denso_recognition_srvs::CropCloud>("/pointcloud_processing/crop_cloud");
  region_growing_get_top_cluster_client_ =
      nh.serviceClient<denso_recognition_srvs::ExtractObject>("/target_extractor/region_growing/get_top_cluster");
  region_growing_get_cluster_client_ =
      nh.serviceClient<denso_recognition_srvs::ExtractObject>("/target_extractor/region_growing/get_cluster");
  region_growing_set_target_cluster_client_ =
      nh.serviceClient<denso_recognition_srvs::ExtractObject>("/target_extractor/region_growing/set_targer_cluster");
  make_input_client_ = nh.serviceClient<denso_recognition_srvs::MakeInput>("/cnn_pose_estimator/make_input");
  cnn_estimator_client_ =
      nh.serviceClient<denso_recognition_srvs::PoseEstimate>("/cnn_pose_estimator/estimate_object_by_cnn");
  registrate_pose_client_ =
      nh.serviceClient<denso_recognition_srvs::RegistrationPose>("/cnn_pose_estimator/registrate_pose");
  inverse_pose_client_ =
      nh.serviceClient<denso_recognition_srvs::InversePose>("/cnn_pose_estimator/inverse_object_pose");
}

bool EstimateObjectPoseServer::estimateObjectPose(denso_recognition_srvs::PoseEstimate::Request& req,
                                                  denso_recognition_srvs::PoseEstimate::Response& res)
{
  phoxi_camera_srvs::PublishPointCloud publish_pointcloud_srv;
  if (!get_sensor_frame_client_.call(publish_pointcloud_srv))
  {
    ROS_ERROR_STREAM("Failed to get sensor point cloud !!");
    res.success = false;
    return false;
  }

  ROS_INFO_STREAM("Complete to get sensor point cloud !!");

  ros::Duration(interval_time_).sleep();

  denso_recognition_srvs::CropCloud crop_cloud_srv;
  if (!crop_cloud_client_.call(crop_cloud_srv))
  {
    ROS_ERROR_STREAM("Failed to crop cloud !!");
    res.success = false;
    return false;
  }

  ROS_INFO_STREAM("Complete to crop cloud !!");

  ros::Duration(interval_time_).sleep();

  if (use_clustering_)
  {
    denso_recognition_srvs::ExtractObject region_growing_srv;
    if (!region_growing_get_top_cluster_client_.call(region_growing_srv))
    {
      ROS_ERROR_STREAM("Failed to do region growing !!");
      res.success = false;
      return false;
    }

    ROS_INFO_STREAM("Complete to do region growing !!");

    ros::Duration(interval_time_).sleep();
  }

  denso_recognition_srvs::MakeInput make_input_srv;
  if (!make_input_client_.call(make_input_srv))
  {
    ROS_ERROR_STREAM("Failed to make input !!");
    res.success = false;
    return false;
  }

  ROS_INFO_STREAM("Complete to make input !!");

  ros::Duration(interval_time_).sleep();

  denso_recognition_srvs::PoseEstimate object_pose_estimator_srv;
  if (!cnn_estimator_client_.call(object_pose_estimator_srv))
  {
    ROS_ERROR_STREAM("Failed to estimate object pose by cnn result !!");
    res.success = false;
    return false;
  }

  ROS_INFO_STREAM("Complete to estimate object pose by cnn result !!");

  ros::Duration(interval_time_).sleep();

  denso_recognition_srvs::RegistrationPose registrate_pose_srv;
  if (!registrate_pose_client_.call(registrate_pose_srv))
  {
    ROS_ERROR_STREAM("Failed to registrate object pose !!");
    res.success = false;
    return false;
  }

  ROS_INFO_STREAM("Complete to registrate object pose !!");

  ros::Duration(interval_time_).sleep();

  denso_recognition_srvs::InversePose inverse_pose_srv;
  if (!inverse_pose_client_.call(inverse_pose_srv))
  {
    ROS_ERROR_STREAM("Failed to judge inversing object pose !!");
    res.success = false;
    return false;
  }

  ROS_INFO_STREAM("Complete to judge inverse object pose !!");

  res.success = true;
  return true;
}

bool EstimateObjectPoseServer::estimateObjectPoseOnceGetCloud(denso_recognition_srvs::PoseEstimate::Request& req,
                                                              denso_recognition_srvs::PoseEstimate::Response& res)
{
  if (!have_pointcloud_)
  {
    phoxi_camera_srvs::PublishPointCloud publish_pointcloud_srv;
    if (!get_sensor_frame_client_.call(publish_pointcloud_srv))
    {
      ROS_ERROR_STREAM("Failed to get sensor point cloud !!");
      res.success = false;
      return false;
    }

    ROS_INFO_STREAM("Complete to get sensor point cloud !!");

    ros::Duration(interval_time_).sleep();

    denso_recognition_srvs::CropCloud crop_cloud_srv;
    if (!crop_cloud_client_.call(crop_cloud_srv))
    {
      ROS_ERROR_STREAM("Failed to crop cloud !!");
      res.success = false;
      return false;
    }

    ROS_INFO_STREAM("Complete to crop cloud !!");

    ros::Duration(interval_time_).sleep();

    denso_recognition_srvs::ExtractObject region_growing_srv;
    if (!region_growing_get_cluster_client_.call(region_growing_srv))
    {
      ROS_ERROR_STREAM("Failed to do region growing !!");
      res.success = false;
      return false;
    }

    ROS_INFO_STREAM("Complete to do region growing !!");

    ros::Duration(interval_time_).sleep();

    have_pointcloud_ = true;
  }

  denso_recognition_srvs::ExtractObject set_target_cluster_srv;
  if (!region_growing_set_target_cluster_client_.call(set_target_cluster_srv))
  {
    ROS_ERROR_STREAM("Failed to set target extract cluster !!");
    res.success = false;
    return false;
  }

  ROS_INFO_STREAM("Complete to set target extract cluster !!");

  ros::Duration(interval_time_).sleep();

  denso_recognition_srvs::MakeInput make_input_srv;
  if (!make_input_client_.call(make_input_srv))
  {
    ROS_ERROR_STREAM("Failed to make input !!");
    res.success = false;
    return false;
  }

  ROS_INFO_STREAM("Complete to make input !!");

  ros::Duration(interval_time_).sleep();

  denso_recognition_srvs::PoseEstimate object_pose_estimator_srv;
  if (!cnn_estimator_client_.call(object_pose_estimator_srv))
  {
    ROS_ERROR_STREAM("Failed to estimate object pose by cnn result !!");
    res.success = false;
    return false;
  }

  ROS_INFO_STREAM("Complete to estimate object pose by cnn result !!");

  ros::Duration(interval_time_).sleep();

  denso_recognition_srvs::RegistrationPose registrate_pose_srv;
  if (!registrate_pose_client_.call(registrate_pose_srv))
  {
    ROS_ERROR_STREAM("Failed to registrate object pose !!");
    res.success = false;
    return false;
  }

  ROS_INFO_STREAM("Complete to registrate object pose !!");

  ros::Duration(interval_time_).sleep();

  denso_recognition_srvs::InversePose inverse_pose_srv;
  if (!inverse_pose_client_.call(inverse_pose_srv))
  {
    ROS_ERROR_STREAM("Failed to judge inversing object pose !!");
    res.success = false;
    return false;
  }

  ROS_INFO_STREAM("Complete to judge inverse object pose !!");

  res.success = true;
  return true;
}