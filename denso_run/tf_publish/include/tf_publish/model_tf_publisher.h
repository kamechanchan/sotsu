#pragma once
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

#include <iostream>
#include <string>
#include <vector>

namespace model_tf_publisher
{
class ModelTf
{
public:
    ModelTf(ros::NodeHandle &nh);
    void broadcastModelTF();
    int getModelId();
    void modelstatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);

private:
    geometry_msgs::TransformStamped tf_stamp_;
    ros::NodeHandle nh_;
    ros::NodeHandle *pnh;
    ros::Subscriber model_state_sub_;
    std::string src_frame_;
    std::string urdf_model_name_;
    std::vector<geometry_msgs::Pose> model_poses_;
    std::vector<std::string> model_names_;
    tf2_ros::TransformBroadcaster br_;
};
}