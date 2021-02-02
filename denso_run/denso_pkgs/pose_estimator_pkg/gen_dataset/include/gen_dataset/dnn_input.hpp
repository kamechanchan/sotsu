#ifndef DNN_INPUT_H
#define DNN_INPUT_H

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl/io/io.h>
#include <pose_estimator_msgs/InputCNN.h>
#include <pose_estimator_msgs/InputData.h>

namespace dnn_input
{
class DnnInput
{
public:
  DnnInput(ros::NodeHandle nh, ros::NodeHandle n);
  void cropCallback(const sensor_msgs::PointCloud2::ConstPtr& source_pc);
  void getVoxelSize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  void downsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output,
                       float leaf);
  void makeVoxel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float diff);

private:
  ros::NodeHandle nh_;
  ros::Rate rate_;
  std::string frame_id_;
  std::string sensor_frame_id_;
  ros::Publisher voxel_pub_;
  ros::Subscriber source_pc_sub_;

  pcl::PointXYZ voxel_min_, voxel_max_;
  pose_estimator_msgs::InputCNN voxel_data_;
  pose_estimator_msgs::InputData input_pub_data_;  // clustering結果を格納配列

  int voxel_n_ = 50;
  float diff_max_ = 0;
};
}  // namespace dnn_input

#endif  // DNN_INPUT_H
