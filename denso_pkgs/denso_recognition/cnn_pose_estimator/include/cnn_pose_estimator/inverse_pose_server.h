#ifndef INVERSE_POSE_SERVER_H
#define INVERSE_POSE_SERVER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <denso_recognition_srvs/InversePose.h>

#include <string>

namespace inverse_pose_server
{
class InversePoseServer
{
public:
  InversePoseServer(ros::NodeHandle& nh);
  void broadcastInverseTF();

private:
  bool inverseObjectPose(denso_recognition_srvs::InversePose::Request& req,
                         denso_recognition_srvs::InversePose::Response& res);

private:
  ros::NodeHandle nh_;
  ros::ServiceServer inverse_pose_server_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  geometry_msgs::TransformStamped update_transform_stamped_;
  bool tf_flag_;
  std::string object_name_;
  std::string update_object_name_;
};
}  // namespace inverse_pose_server

#endif  // INVERSE_POSE_SERVER_H