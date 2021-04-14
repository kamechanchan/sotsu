#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <math.h>

class Converter
{
public:
  Converter(void)
  {
    ros::param::param<std::string>("~object_name", object_name_, "HV8");
    joy_sub_ = nh_.subscribe("/ground_truth/" + object_name_ + "/pose", 10, &Converter::odomCallback, this);
  }
  void odomCallback(const nav_msgs::Odometry& msg)
  {
    tf::Transform transform;
    tf::poseMsgToTF(msg.pose.pose, transform);
    br_.sendTransform(tf::StampedTransform(transform, msg.header.stamp, /*msg.header.frame_id*/"world", msg.child_frame_id));
  }
  ros::NodeHandle nh_;
  ros::Subscriber joy_sub_;
  tf::TransformBroadcaster br_;
  std::string object_name_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_tf_converter");
  Converter converter;
  ros::spin();
  return 0;
}
