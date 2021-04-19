#ifndef MAKE_INPUT_SERVER_H
#define MAKE_INPUT_SERVER_H

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl/io/io.h>
#include <denso_recognition_msgs/BoundingBoxArray.h>
#include <denso_recognition_msgs/InputCNN.h>
#include <denso_recognition_msgs/InputCNNArray.h>
#include <denso_recognition_srvs/MakeInput.h>

typedef pcl::PointXYZ PointInT;

namespace make_input_server
{
class MakeInputServer
{
public:
  MakeInputServer(ros::NodeHandle nh, ros::NodeHandle n);
  void publishVoxelArray();

private:
  void cropCallback(const sensor_msgs::PointCloud2::ConstPtr& source_pc);
  void getVoxelSize(pcl::PointCloud<PointInT>::Ptr cloud);
  void downsampleCloud(pcl::PointCloud<PointInT>::Ptr cloud, pcl::PointCloud<PointInT>::Ptr output,
                       float leaf);
  void makeVoxel(pcl::PointCloud<PointInT>::Ptr cloud, float diff);
  bool getVoxelArray(denso_recognition_srvs::MakeInput::Request& req, denso_recognition_srvs::MakeInput::Response& res);

private:
  ros::NodeHandle nh_;
  ros::Publisher voxel_pub_;
  ros::Subscriber source_pc_sub_;
  ros::ServiceServer get_voxel_array_server_;
  std::string sensor_frame_id_;
  tf::TransformListener tf_;

  PointInT voxel_min_, voxel_max_;
  sensor_msgs::PointCloud2 input_cloud_;
  denso_recognition_msgs::InputCNN input_data_;
  denso_recognition_msgs::InputCNNArray voxels_array_;  // clustering結果を格納配列

  bool is_ok_;
  int voxel_n_;
  float diff_max_;
};
}  // namespace make_input_server

#endif  // MAKE_INPUT_SERVER_H
