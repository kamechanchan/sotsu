#pragma once
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_ros/buffer.h>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv4/opencv2/opencv.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <estimator/input_data.h>

class GetDataNode{
    private:
        ros::NodeHandle nh_;
        // ros::Subscriber sub_from_sensor_;
        ros::ServiceServer server_;
        // sensor_msgs::Image in_img_;
        std::string save_img_path_;
        // cv::Mat out_img_;
        // cv_bridge::CvImagePtr cv_img_;
        // ishiyama::input_data srv_;
        ros::Subscriber photoneo_img_sub_;
        sensor_msgs::Image data_;
        void callback_(const sensor_msgs::Image& msg);
    public:
        GetDataNode();
        // void callback(const sensor_msgs::Image& in_img);
        // void send_data_server();
        bool inputData(estimator::input_data::Request &req, estimator::input_data::Response &res);
};