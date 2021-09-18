#pragma once
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class Arm_Move
{
public:
    Arm_Move();
    void arm_register(std::string);
    void hand_register(std::string);
    void show_arm_joint();
    void show_hand_joint();
    void hand_open();
    void hand_close();
    void set_close_range(double);
    void tf_get(std::string, std::string, geometry_msgs::TransformStamped&);
private:
    moveit::planning_interface::MoveGroupInterface *arm_group_;
    moveit::planning_interface::MoveGroupInterface *hand_group_;
    ros::AsyncSpinner spinner;
    double close_range_;
    template <typename T>
    void show_value(std::vector<T> value){
        for (int i = 0; i < value.size(); i++) {
            std::cout << value[i] << " ";
        }
        std::cout << std::endl;
    }
};