#ifndef MEASUREMENT_BEHAVIOR_H
#define MEASUREMENT_BEHAVIOR_H

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

#include <denso_state_srvs/SetWorkPoint.h>
#include <denso_state_srvs/Measurement.h>

namespace measurement_behavior
{
class MeasurementBehavior
{
public:
  MeasurementBehavior(ros::NodeHandle& nh);
  void broadcastWorkPoint();
  bool setWorkPoint(denso_state_srvs::SetWorkPoint::Request& req, denso_state_srvs::SetWorkPoint::Response& res);
  bool getWorkPoint(denso_state_srvs::Measurement::Request& req, denso_state_srvs::Measurement::Response& res);

private:
  ros::NodeHandle nh_;
  ros::ServiceServer set_work_point_server_;
  ros::ServiceServer get_work_point_server_;
  ros::ServiceClient estimate_pose_client_;
  tf2_ros::TransformBroadcaster br_;
  geometry_msgs::TransformStamped grasp_point_transform_stamped_;
  geometry_msgs::TransformStamped assemble_point_transform_stamped_;
  geometry_msgs::TransformStamped assemble_point_base_transform_stamped_;
  std::string assemble_part_base_frame_name_;
  std::string method_estimate_object_pose_;
  bool is_ok_;
};
} // namespace measurement_behavior

#endif // MEASUREMENT_BEHAVIOR_H
