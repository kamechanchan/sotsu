#include <denso_state_behavior/measurement/measurement_work_point_behavior.h>

#include <geometry_msgs/TransformStamped.h>

#include <denso_state_msgs/WorkPoint.h>

using measurement_work_point_behavior::MeasurementWorkPointBehavior;

MeasurementWorkPointBehavior::MeasurementWorkPointBehavior(ros::NodeHandle& nh) : nh_(nh), tf_listener_(tf_buffer_)
{
  measure_work_point_server_ = nh.advertiseService("/measurement/measure_work_point", &MeasurementWorkPointBehavior::measureWorkPoint, this);
}

bool MeasurementWorkPointBehavior::measureWorkPoint(denso_state_srvs::MeasurementWorkPoint::Request& req, denso_state_srvs::MeasurementWorkPoint::Response& res)
{
  denso_state_msgs::WorkPoint workpoint;
  geometry_msgs::TransformStamped transform;
  int time_out = 0;
  bool is_ok_ = false;

  while (is_ok_)
  {
    if (!ros::param::get("/measurement_work_point_behavior/is_work_point_broadcast", is_ok_))
    {
      ROS_ERROR_STREAM("Can't get param /measurement_work_point_behavior/is_work_point_broadcast !!");
      res.success = false;
      return res.success;
    }
    time_out += 1;
    if (time_out >= 2000)
    {
      res.success = false;
      return res.success;
    }
  }

  time_out = 0;

  while (time_out < 2000)
  {
    try
    {
      transform = tf_buffer_.lookupTransform("base_link", "grasp_point", ros::Time(0), ros::Duration(1.0));
      break;
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      time_out += 1;
      res.success = false;
      continue;
    }
  }

  if (time_out >= 2000)
  {
    return res.success;
  }

  workpoint.grasp.position.x = transform.transform.translation.x;
  workpoint.grasp.position.y = transform.transform.translation.y;
  workpoint.grasp.position.z = transform.transform.translation.z;
  workpoint.grasp.orientation.x = transform.transform.rotation.x;
  workpoint.grasp.orientation.y = transform.transform.rotation.y;
  workpoint.grasp.orientation.z = transform.transform.rotation.z;
  workpoint.grasp.orientation.w = transform.transform.rotation.w;

  time_out = 0;

  while (time_out < 2000)
  {
    try
    {
      transform = tf_buffer_.lookupTransform("base_link", "assemble_point", ros::Time(0), ros::Duration(1.0));
      break;
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      time_out += 1;
      res.success = false;
      continue;
    }
  }

  if (time_out >= 2000)
  {
    return res.success;
  }

  workpoint.assemble.position.x = transform.transform.translation.x;
  workpoint.assemble.position.y = transform.transform.translation.y;
  workpoint.assemble.position.z = transform.transform.translation.z;
  workpoint.assemble.orientation.x = transform.transform.rotation.x;
  workpoint.assemble.orientation.y = transform.transform.rotation.y;
  workpoint.assemble.orientation.z = transform.transform.rotation.z;
  workpoint.assemble.orientation.w = transform.transform.rotation.w;

  res.success = true;
  res.workpoint = workpoint;

  return res.success;
}