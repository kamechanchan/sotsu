#include <cnn_pose_estimator/inverse_pose_server.h>

#include <iostream>
#include <tf/tf.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>

using inverse_pose_server::InversePoseServer;

InversePoseServer::InversePoseServer(ros::NodeHandle& nh) : nh_(nh), tf_listener_(tf_buffer_), tf_flag_(false)
{
  ros::param::param<std::string>("~object_name", object_name_, "T_pipe");
  ros::param::param<std::string>("~update_objcet_name", update_object_name_, "T_pipe_fix");
  inverse_pose_server_ =
      nh.advertiseService("/cnn_pose_estimator/inverse_object_pose", &InversePoseServer::inverseObjectPose, this);
}

bool InversePoseServer::inverseObjectPose(denso_recognition_srvs::InversePose::Request& req,
                                          denso_recognition_srvs::InversePose::Response& res)
{
  tf_flag_ = false;
  geometry_msgs::TransformStamped transform;
  int time_out = 0;

  while (time_out < 20)
  {
    try
    {
      transform = tf_buffer_.lookupTransform("world", object_name_, ros::Time(0), ros::Duration(1.0));
      break;
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      time_out += 1;
      res.success = false;
      continue;
    }
  }

  if (time_out >= 20)
  {
    return res.success;
  }

  geometry_msgs::Quaternion quat_msg;
  quat_msg.x = transform.transform.rotation.x;
  quat_msg.y = transform.transform.rotation.y;
  quat_msg.z = transform.transform.rotation.z;
  quat_msg.w = transform.transform.rotation.w;
  double roll, pitch, yaw;
  tf::Quaternion quat(quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  double roll_deg = roll * 180.0 / M_PI;
  double pitch_deg = pitch * 180.0 / M_PI;
  double yaw_deg = yaw * 180.0 / M_PI;

  if (roll_deg < -90 || 90 < roll_deg || pitch_deg < -90 || 90 < pitch_deg)
  {
    update_transform_stamped_.transform.translation.x = 0;
    update_transform_stamped_.transform.translation.y = 0;
    update_transform_stamped_.transform.translation.z = 0;
    update_transform_stamped_.transform.rotation.x = 1;
    update_transform_stamped_.transform.rotation.y = 0;
    update_transform_stamped_.transform.rotation.z = 0;
    update_transform_stamped_.transform.rotation.w = 0;
  }
  else
  {
    update_transform_stamped_.transform.translation.x = 0;
    update_transform_stamped_.transform.translation.y = 0;
    update_transform_stamped_.transform.translation.z = 0;
    update_transform_stamped_.transform.rotation.x = 0;
    update_transform_stamped_.transform.rotation.y = 0;
    update_transform_stamped_.transform.rotation.z = 0;
    update_transform_stamped_.transform.rotation.w = 1;
  }

  update_transform_stamped_.header.frame_id = object_name_;
  update_transform_stamped_.child_frame_id = update_object_name_;

  tf_flag_ = true;
  res.success = true;

  return res.success;
}

void InversePoseServer::broadcastInverseTF()
{
  if (tf_flag_)
  {
    update_transform_stamped_.header.stamp = ros::Time::now();
    tf_broadcaster_.sendTransform(update_transform_stamped_);
  }
}