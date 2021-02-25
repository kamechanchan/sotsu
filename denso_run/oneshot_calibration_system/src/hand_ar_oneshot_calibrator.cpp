#include <oneshot_calibration_system/hand_ar_oneshot_calibrator.h>

using hand_ar_oneshot_calibrator::HandAROneshotCalibrator;
using hand_ar_oneshot_calibrator::CalibParam;

const float CalibParam::DURATION_TIME = 4.0;
const int CalibParam::START_TIME = 0;
const int CalibParam::PUBLISHER_QUEUE_SIZE = 10;
const int CalibParam::SUBSCRIBER_QUEUE_SIZE = 1;

HandAROneshotCalibrator::HandAROneshotCalibrator(ros::NodeHandle& nh, ros::NodeHandle& n)
  : nh_(nh), tfListener_(tfBuffer_)
{
  ROS_INFO("Constractor Start!!");
  texture_sub_ = nh_.subscribe("/photoneo_center/rgb_texture", CalibParam::SUBSCRIBER_QUEUE_SIZE, &HandAROneshotCalibrator::calibrateSensor, this);

  n.param<std::string>("ar_frame", ar_frame_, "ar_marker_4");
  n.param<std::string>("target_frame", target_frame_, "photoneo_center");
  n.param<std::string>("source_frame", source_frame_, "true_ar_marker_4_center");
  n.param<std::string>("base_frame", base_frame_, "base_link");
  fixed_photoneo_center_.header.frame_id = base_frame_;
  fixed_photoneo_center_.child_frame_id = target_frame_;
  fixed_photoneo_center_.transform.translation.x = 0.0;
  fixed_photoneo_center_.transform.translation.y = 0.0;
  fixed_photoneo_center_.transform.translation.z = 0.0;
  fixed_photoneo_center_.transform.rotation.x = 0.0;
  fixed_photoneo_center_.transform.rotation.y = 0.0;
  fixed_photoneo_center_.transform.rotation.z = 0.0;
  fixed_photoneo_center_.transform.rotation.w = 1.0;
  ar_to_phoxi_.header.frame_id = ar_frame_;
  ar_to_phoxi_.child_frame_id = target_frame_;
  base_link_to_true_ar_.header.frame_id = base_frame_;
  base_link_to_true_ar_.child_frame_id = source_frame_;
  flag_ = false;
  ROS_INFO("Constractor Finished!!");
}

void HandAROneshotCalibrator::getEulerRPY(const geometry_msgs::Quaternion q, double& roll, double& pitch, double& yaw)
{
  tf::Quaternion quat(q.x, q.y, q.z, q.w);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

void HandAROneshotCalibrator::getQuaternionMsg(double roll, double pitch, double yaw, geometry_msgs::Quaternion& q)
{
  tf::Quaternion quat = tf::createQuaternionFromRPY(roll, pitch, yaw);
  quaternionTFToMsg(quat, q);
}

void HandAROneshotCalibrator::calibrateSensor(const sensor_msgs::Image::ConstPtr& img)
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
  double cos = 0.8888;
  double sin = 0.4581;

  cv::Mat mat1 = generateRotationMatrix(base_to_true_roll, base_to_true_pitch, base_to_true_yaw);
  cv::Mat mat2 = generateRotationMatrix(ar_to_phoxi_roll, ar_to_phoxi_pitch, ar_to_phoxi_yaw);
  cv::Mat multiplied_matrix = mat2 * mat1;
  double angle_x;
  double angle_y;
  double angle_z;
  double threshold = 0.001;
  if(std::abs(multiplied_matrix.at<double>(0,2) - 1.0) < threshold){ // R(2,1) = sin(x) = 1の時
    angle_x = atan2(-multiplied_matrix.at<double>(1,0), multiplied_matrix.at<double>(1,1));
    angle_y = -M_PI / 2;
    angle_z = 0;
  }else if(std::abs(multiplied_matrix.at<double>(2,0) + 1.0) < threshold){ // R(2,1) = sin(x) = -1の時
    angle_x = atan2(multiplied_matrix.at<double>(1,0), multiplied_matrix.at<double>(1,1));
    angle_y = M_PI / 2;
    angle_z = 0;
  }else{
    angle_x = atan2(multiplied_matrix.at<double>(1,2), multiplied_matrix.at<double>(2,2));
    angle_y = asin(-multiplied_matrix.at<double>(0,2));
    angle_z = atan2(multiplied_matrix.at<double>(0,1), multiplied_matrix.at<double>(0,0));
  }
  double modify_x = base_link_to_true_ar_.transform.translation.x + (-sin * ar_to_phoxi_.transform.translation.x + cos * ar_to_phoxi_.transform.translation.z);
  double modify_y = base_link_to_true_ar_.transform.translation.y + (cos * ar_to_phoxi_.transform.translation.x + sin * ar_to_phoxi_.transform.translation.z);
  double modify_z = base_link_to_true_ar_.transform.translation.z + ar_to_phoxi_.transform.translation.y;

  double modify_roll = angle_x;
  double modify_pitch = angle_y;
  double modify_yaw =  angle_z;

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
  flag_ = true;
}

cv::Mat HandAROneshotCalibrator::generateRotationMatrix(double roll, double pitch, double yaw)
{
  cv::Mat rot_x = (cv::Mat_<double>(3,3) << 1, 0, 0, 0, std::cos(roll), std::sin(roll), 0, -std::sin(roll), std::cos(roll));
  cv::Mat rot_y = (cv::Mat_<double>(3,3) << std::cos(pitch), 0, -std::sin(pitch), 0, 1, 0, std::sin(pitch), 0, std::cos(pitch));
  cv::Mat rot_z = (cv::Mat_<double>(3,3) << std::cos(yaw), std::sin(yaw), 0, -std::sin(yaw), std::cos(yaw), 0, 0, 0, 1);
  cv::Mat result = rot_x * rot_y * rot_z;
  return result;
}

void HandAROneshotCalibrator::broadcastTF(void)
{
  if (flag_)
  {
      fixed_photoneo_center_.header.stamp = ros::Time::now();
      br_.sendTransform(fixed_photoneo_center_);
  }
}


void HandAROneshotCalibrator::lookupTransform(geometry_msgs::TransformStamped& transform)
{
  bool tf_flag = true;

  while (tf_flag)
  {
    try
    {
      transform = tfBuffer_.lookupTransform(transform.header.frame_id, transform.child_frame_id, ros::Time(0),
                                            ros::Duration(CalibParam::DURATION_TIME));
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
