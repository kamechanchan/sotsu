#pragma once
#include <ros/ros.h>
#include <cstdio>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class BroadCasterTest
{
public:
    BroadCasterTest();
    void timer_callback(const ros::TimerEvent& e);
private:
    void broadcast_static_tf(void);
    void broadcast_dynamic_tf(void);
    ros::NodeHandle nh_;
    ros::Timer timer_;
    tf2_ros::TransformBroadcaster dynamic_br_;
    tf2_ros::StaticTransformBroadcaster static_br_;
    int counter_;
};