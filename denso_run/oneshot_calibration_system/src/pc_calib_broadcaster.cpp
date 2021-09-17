#include <oneshot_calibration_system/pc_calib_broadcaster.h>

using pc_calib_broadcaster::PCCalibBroadcaster;
using pc_calib_broadcaster::CalibParam;

const int CalibParam::SUBSCRIBER_QUEUE_SIZE = 1;

PCCalibBroadcaster::PCCalibBroadcaster(ros::NodeHandle& nh, ros::NodeHandle& n) : nh_(nh)
{
  ROS_INFO("PointCloud Calibration_Broadcast Start!!");
  flag_ = true;
  trans_msgs_sub_ = nh.subscribe("transform_parameter", CalibParam::SUBSCRIBER_QUEUE_SIZE, &PCCalibBroadcaster::sendSetting, this);
  n.param<std::string>("target_frame", target_frame_, "/photoneo_center");
  n.param<std::string>("source_frame", source_frame_, "/base_link");
  n.param<std::string>("node_name", node_name_, "/ar_oneshot_calibrator");
}

void PCCalibBroadcaster::sendSetting(const geometry_msgs::Transform trans)
{
  if (flag_)
  {
    ROS_INFO("Setting start!!");
    tf::transformMsgToTF(trans, output_tf_);
    ROS_INFO("ARmarker Calibration node killed!!");
    std::string command = "rosnode kill " + node_name_;
    flag_ = std::system(command.c_str());
  }
}

void PCCalibBroadcaster::broadcastTF(void)
{
  if (!flag_)
  {
    ROS_INFO("pointcloud calibration broadcaster photoneo_frame!!");
    br_.sendTransform(tf::StampedTransform(output_tf_, ros::Time::now(), source_frame_, target_frame_));
  }
}
