#ifndef MEASUREMENT_WORK_POINT_BEHAVIOR_H
#define MEASUREMENT_WORK_POINT_BEHAVIOR_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <denso_state_srvs/MeasurementWorkPoint.h>

namespace measurement_work_point_behavior
{
class MeasurementWorkPointBehavior
{
public:
  MeasurementWorkPointBehavior(ros::NodeHandle& nh);
  bool measureWorkPoint(denso_state_srvs::MeasurementWorkPoint::Request& req, denso_state_srvs::MeasurementWorkPoint::Response& res);

private:
  ros::NodeHandle nh_;
  ros::ServiceServer measure_work_point_server_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};
} // namespace measurement_work_point_behavior

#endif // MEASUREMENT_WORK_POINT_BEHAVIOR_H