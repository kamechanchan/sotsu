#include <oneshot_calibration_system/ar_oneshot_calibrator.h>

using ar_oneshot_calibrator::AROneshotCalibrator;
using ar_oneshot_calibrator::ARCalibParam;

const int ARCalibParam::SUBSCRIBER_QUEUE_SIZE = 1;

// Class method difinitions
AROneshotCalibrator::AROneshotCalibrator(ros::NodeHandle& nh) : nh_(nh), tfListener_(tfBuffer_)
{
  std::string frame_id, child_frame_id;
  if (nh_.getParam("/ar_oneshot_calibrator/targetA_frame_id", frame_id))
  {
    if (nh_.getParam("/ar_oneshot_calibrator/sensor_frame_id", child_frame_id))
    {
      ar_to_phoxi_.header.frame_id = frame_id;
      ar_to_phoxi_.child_frame_id = child_frame_id;
    }
  }
  if (nh_.getParam("/ar_oneshot_calibrator/origin_frame_id", frame_id))
  {
    if (nh_.getParam("/ar_oneshot_calibrator/targetB_frame_id", child_frame_id))
    {
      base_link_to_true_ar_.header.frame_id = frame_id;
      base_link_to_true_ar_.child_frame_id = child_frame_id;
    }
  }

  // initial transform
  if (nh_.getParam("/ar_oneshot_calibrator/origin_frame_id", frame_id))
  {
    if (nh_.getParam("/ar_oneshot_calibrator/sensor_frame_id", child_frame_id))
    {
      fixed_photoneo_center_.header.frame_id = frame_id;
      fixed_photoneo_center_.child_frame_id = child_frame_id;
      fixed_photoneo_center_.transform.translation.x = 0.0;
      fixed_photoneo_center_.transform.translation.y = 0.0;
      fixed_photoneo_center_.transform.translation.z = 0.0;
      fixed_photoneo_center_.transform.rotation.x = 0.0;
      fixed_photoneo_center_.transform.rotation.y = 0.0;
      fixed_photoneo_center_.transform.rotation.z = 0.0;
      fixed_photoneo_center_.transform.rotation.w = 1.0;
    }
  }

  texture_sub_ = nh.subscribe("/photoneo_center/texture", ARCalibParam::SUBSCRIBER_QUEUE_SIZE, &AROneshotCalibrator::updatePhotoneoTransform, this);
}

void AROneshotCalibrator::broadcast(void)
{
  fixed_photoneo_center_.header.stamp = ros::Time::now();
  br_.sendTransform(fixed_photoneo_center_);
}

void AROneshotCalibrator::getEulerRPY(const geometry_msgs::Quaternion q, double& roll, double& pitch, double& yaw)
{
  tf::Quaternion quat(q.x, q.y, q.z, q.w);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

void AROneshotCalibrator::getQuaternionMsg(double roll, double pitch, double yaw, geometry_msgs::Quaternion& q)
{
  tf::Quaternion quat = tf::createQuaternionFromRPY(roll, pitch, yaw);
  quaternionTFToMsg(quat, q);
}

void AROneshotCalibrator::updatePhotoneoTransform(const sensor_msgs::Image::ConstPtr& img)
{
  // lookupTransform from ar_center to photoneo_center
  lookupTransform(ar_to_phoxi_);

  // convert quatanion to euler RPY
  double ar_to_phoxi_roll, ar_to_phoxi_pitch, ar_to_phoxi_yaw;
  getEulerRPY(ar_to_phoxi_.transform.rotation, ar_to_phoxi_roll, ar_to_phoxi_pitch, ar_to_phoxi_yaw);

  // lookupTransform from base_link to true_ar_center
  lookupTransform(base_link_to_true_ar_);

  // convert quatanion to euler RPY
  double base_to_true_roll, base_to_true_pitch, base_to_true_yaw;
  getEulerRPY(base_link_to_true_ar_.transform.rotation, base_to_true_roll, base_to_true_pitch, base_to_true_yaw);

  // get modify variable
  double modify_x = base_link_to_true_ar_.transform.translation.x + ar_to_phoxi_.transform.translation.x;
  double modify_y = base_link_to_true_ar_.transform.translation.y + ar_to_phoxi_.transform.translation.y;
  double modify_z = base_link_to_true_ar_.transform.translation.z + ar_to_phoxi_.transform.translation.z;
  double modify_roll = base_to_true_roll + ar_to_phoxi_roll;
  double modify_pitch = base_to_true_pitch + ar_to_phoxi_pitch;
  double modify_yaw = base_to_true_yaw + ar_to_phoxi_yaw;

  // euler to quatanion
  geometry_msgs::Quaternion modify_quat;
  getQuaternionMsg(modify_roll, modify_pitch, modify_yaw, modify_quat);

  // modify transform
  fixed_photoneo_center_.transform.translation.x = modify_x;
  fixed_photoneo_center_.transform.translation.y = modify_y;
  fixed_photoneo_center_.transform.translation.z = modify_z;
  fixed_photoneo_center_.transform.rotation.x = modify_quat.x;
  fixed_photoneo_center_.transform.rotation.y = modify_quat.y;
  fixed_photoneo_center_.transform.rotation.z = modify_quat.z;
  fixed_photoneo_center_.transform.rotation.w = modify_quat.w;
}

void AROneshotCalibrator::lookupTransform(geometry_msgs::TransformStamped& transform)
{
  bool tf_flag = true;

  while (tf_flag)
  {
    try
    {
      transform = tfBuffer_.lookupTransform(transform.header.frame_id, transform.child_frame_id, ros::Time(0),
                                            ros::Duration(1.0));
      tf_flag = false;
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      tf_flag = true;
      continue;
    }
  }
}
