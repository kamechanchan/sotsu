#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <math.h>

class Converter
{
public:
  Converter(ros::NodeHandle nh, const std::string odom_topic_name) : nh_(nh)
  {
    odom_sub_ = nh_.subscribe(odom_topic_name, 10, &Converter::odomCallback, this);
  }
  void odomCallback(const nav_msgs::Odometry& msg)
  {
    tf::Transform transform;
    tf::poseMsgToTF(msg.pose.pose, transform);
    br_.sendTransform(tf::StampedTransform(transform, msg.header.stamp, msg.header.frame_id, msg.child_frame_id));
  }
  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  tf::TransformBroadcaster br_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sim_odom_tf_converter");
  ros::NodeHandle nh;
  std::string odom_topic_name;
  ros::param::param<std::string>("~odom_topic_name", odom_topic_name, "cardboard_link_00_ground_truth");
  Converter converter(nh, odom_topic_name);
  ros::spin();
  return 0;
}
