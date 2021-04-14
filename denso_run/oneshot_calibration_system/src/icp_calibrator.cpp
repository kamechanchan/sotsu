#include <oneshot_calibration_system/icp_calibrator.h>

using icp_calibrator::ICPcalibrator;
using icp_calibrator::ICPcalibParam;

const int ICPcalibParam::PUBLISHER_QUEUE_SIZE = 10;
const int ICPcalibParam::SUBSCRIBER_QUEUE_SIZE = 1;

ICPcalibrator::ICPcalibrator(ros::NodeHandle& nh, ros::NodeHandle& n) : nh_(nh)
{
  ROS_INFO("ICP Calibrator Start!!");
  icp_calib_pub_ = nh.advertise<geometry_msgs::Transform>("pointcloud_calib_transform", ICPcalibParam::PUBLISHER_QUEUE_SIZE);
  icp_result_sub_ = nh.subscribe("input_data", ICPcalibParam::SUBSCRIBER_QUEUE_SIZE, &ICPcalibrator::convertData, this);
  n.param<std::string>("target_frame", target_frame_, "base_link");
  n.param<std::string>("source_frame", source_frame_, "photoneo_center");
  flag = false;
}

void ICPcalibrator::convertData(const std_msgs::Float64MultiArray icp_data)
{
  ROS_INFO("Conversion Start!!");
  tf::StampedTransform base_to_photoneo_by_ar;
  lookupTransform(base_to_photoneo_by_ar);
  Eigen::Affine3d sensor_to_world_transform;
  tf::transformTFToEigen(base_to_photoneo_by_ar, sensor_to_world_transform);
  Eigen::Matrix4d matrix_sensor_to_world = sensor_to_world_transform.matrix();
  Eigen::Matrix4d sensor_to_corrected;
  multiArrayToMat4d(icp_data, sensor_to_corrected);
  Eigen::Matrix4d world_to_corrected_sensor = sensor_to_corrected * matrix_sensor_to_world;
  Eigen::Affine3d eigen_affine3d(world_to_corrected_sensor);
  tf::Transform fixed_sensor_frame;
  tf::transformEigenToTF(eigen_affine3d, fixed_sensor_frame);
  tf::transformTFToMsg(fixed_sensor_frame, base_link_to_photoneo_);
  if (!flag)
    flag = true;
  ROS_INFO("Conversion Finish!!");
}

void ICPcalibrator::publishICPResult(void)
{
  icp_calib_pub_.publish(base_link_to_photoneo_);
}

void ICPcalibrator::multiArrayToMat4d(const std_msgs::Float64MultiArray& icp_array, Eigen::Matrix4d& mat4d)
{
  unsigned int ii = 0;
  for (unsigned int i = 0; i < icp_array.layout.dim[0].size; ++i)
  {
    for (unsigned int j = 0; j < icp_array.layout.dim[1].size; ++j)
    {
      mat4d.coeffRef(i, j) = icp_array.data[ii++];
    }
  }
}

void ICPcalibrator::lookupTransform(tf::StampedTransform& transform)
{
  bool tf_flag = true;

  while (tf_flag)
  {
    try
    {
      tf_.lookupTransform(target_frame_, source_frame_, ros::Time::now(), transform);
      tf_flag = false;
    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      tf_flag = true;
      continue;
    }
  }
}
