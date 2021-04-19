#ifndef RECORD_EULER_H
#define RECORD_EULER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>

#include <string>

namespace record_euler
{
class RecordEuler
{
public:
  RecordEuler(ros::NodeHandle& nh);
  bool runRecord();
private:
  void receiveCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
  bool savePose();
  bool savePCD(const sensor_msgs::PointCloud2& input_cloud);
  bool savePCDandPose();
private:
  ros::NodeHandle nh_;
  ros::Subscriber cloud_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  sensor_msgs::PointCloud2 receive_cloud_;
  bool is_ok_;
  int max_data_;
  int number_of_data_;
  int timeout_;
  std::string package_name_;
  std::string data_name_;
  std::string optical_sensor_frame_;
  std::string object_name_;
  std::string src_cloud_topic_;
};
} // namespace record_euler

#endif // RECORD_EULER_H
