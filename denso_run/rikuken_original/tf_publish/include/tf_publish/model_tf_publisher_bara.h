#pragma once
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace model_tf_publisher_bara
{
    class Model_bara
    {
    public: 
        Model_bara(ros::NodeHandle&);
        void modelstateCallback(const gazebo_msgs::ModelState::ConstPtr&);
        int getModelId(std::string urdf_model_name);
        void broadcastModelTF();
        
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle *pnh_;
        std::string src_frame_;
        ros::Subscriber model_state_sub_;
        std::vector<std::string> urdf_model_name_, gazebo_model_name_;
        std::vector<geometry_msgs::Pose> gazebo_model_poses_;
        geometry_msgs::Transform transform_stamp_;
        std::string object_name_;
        tf::TransformBroadcaster br_;
    };
}