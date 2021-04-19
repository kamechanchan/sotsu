#include <denso_state_behavior/measurement/measurement_behavior.h>

#include <denso_recognition_srvs/PoseEstimate.h>

using measurement_behavior::MeasurementBehavior;

MeasurementBehavior::MeasurementBehavior(ros::NodeHandle& nh) : nh_(nh), is_ok_(false)
{
  ros::param::param<std::string>("~assemble_part_base_frame_name", assemble_part_base_frame_name_, "assemble_part_base_frame");
  ros::param::param<std::string>("~method_estimate_object_pose", method_estimate_object_pose_, "estimate_object_pose_once_get_cloud");
  estimate_pose_client_ = nh.serviceClient<denso_recognition_srvs::PoseEstimate>("/cnn_pose_estimator/" + method_estimate_object_pose_);
  set_work_point_server_ = nh.advertiseService("/measurement/set_work_point", &MeasurementBehavior::setWorkPoint, this);
  get_work_point_server_ = nh.advertiseService("/measurement/get_work_point", &MeasurementBehavior::getWorkPoint, this);
}

void MeasurementBehavior::broadcastWorkPoint()
{
  if (is_ok_)
  {
    ros::param::set("/measurement_work_point_behavior/is_work_point_broadcast", true);
    grasp_point_transform_stamped_.header.stamp = ros::Time::now();
    assemble_point_transform_stamped_.header.stamp = ros::Time::now();
    assemble_point_base_transform_stamped_.header.stamp = ros::Time::now();
    br_.sendTransform(grasp_point_transform_stamped_);
    br_.sendTransform(assemble_point_transform_stamped_);
    br_.sendTransform(assemble_point_base_transform_stamped_);
  }
}

bool MeasurementBehavior::setWorkPoint(denso_state_srvs::SetWorkPoint::Request& req, denso_state_srvs::SetWorkPoint::Response& res)
{
  is_ok_ = false;
  ros::param::set("/measurement_work_point_behavior/is_work_point_broadcast", false);

  grasp_point_transform_stamped_.transform.translation.x = req.workpoint.grasp.position.x;
  grasp_point_transform_stamped_.transform.translation.y = req.workpoint.grasp.position.y;
  grasp_point_transform_stamped_.transform.translation.z = req.workpoint.grasp.position.z;
  grasp_point_transform_stamped_.transform.rotation.x = req.workpoint.grasp.orientation.x;
  grasp_point_transform_stamped_.transform.rotation.y = req.workpoint.grasp.orientation.y;
  grasp_point_transform_stamped_.transform.rotation.z = req.workpoint.grasp.orientation.z;
  grasp_point_transform_stamped_.transform.rotation.w = req.workpoint.grasp.orientation.w;
  assemble_point_transform_stamped_.transform.translation.x = req.workpoint.grasp.position.x;
  assemble_point_transform_stamped_.transform.translation.y = req.workpoint.grasp.position.y;
  assemble_point_transform_stamped_.transform.translation.z = req.workpoint.grasp.position.z;
  assemble_point_transform_stamped_.transform.rotation.x = req.workpoint.grasp.orientation.x;
  assemble_point_transform_stamped_.transform.rotation.y = req.workpoint.grasp.orientation.y;
  assemble_point_transform_stamped_.transform.rotation.z = req.workpoint.grasp.orientation.z;
  assemble_point_transform_stamped_.transform.rotation.w = req.workpoint.grasp.orientation.w;
  assemble_point_base_transform_stamped_.transform.translation.x = req.workpoint.assemble.position.x;
  assemble_point_base_transform_stamped_.transform.translation.y = req.workpoint.assemble.position.y;
  assemble_point_base_transform_stamped_.transform.translation.z = req.workpoint.assemble.position.z;
  assemble_point_base_transform_stamped_.transform.rotation.x = req.workpoint.assemble.orientation.x;
  assemble_point_base_transform_stamped_.transform.rotation.y = req.workpoint.assemble.orientation.y;
  assemble_point_base_transform_stamped_.transform.rotation.z = req.workpoint.assemble.orientation.z;
  assemble_point_base_transform_stamped_.transform.rotation.w = req.workpoint.assemble.orientation.w;

  res.success = true;
  return true;
}

bool MeasurementBehavior::getWorkPoint(denso_state_srvs::Measurement::Request& req, denso_state_srvs::Measurement::Response& res)
{
  is_ok_ = false;
  ros::param::set("/measurement_work_point_behavior/is_work_point_broadcast", false);
  denso_recognition_srvs::PoseEstimate estimate_pose_srv;
  if (!estimate_pose_client_.call(estimate_pose_srv))
  {
    ROS_ERROR_STREAM("Failed to estimate object pose service !!");
    is_ok_ =false;
    res.success = false;
    return res.success;
  }

  grasp_point_transform_stamped_.header.frame_id = req.grasp_point_name;
  grasp_point_transform_stamped_.child_frame_id = "grasp_point";
  assemble_point_transform_stamped_.header.frame_id = req.assemble_point_name;
  assemble_point_transform_stamped_.child_frame_id = "assemble_point";
  assemble_point_base_transform_stamped_.header.frame_id = assemble_part_base_frame_name_;
  assemble_point_base_transform_stamped_.child_frame_id = req.assemble_point_name;

  is_ok_ = true;

  res.success = true;
  return res.success;
}
