#pragma once
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <estimator/input_data.h>

class GetDataNode{
    private:
        ros::NodeHandle nh_;
        ros::ServiceServer server_;
        std::string save_img_path_;
        sensor_msgs::Image data_;
        ros::Subscriber photoneo_img_sub_;
        void callback_(const sensor_msgs::Image& msg);
    public:
        GetDataNode();
        bool inputData(estimator::input_data::Request &req, estimator::input_data::Response &res);
};