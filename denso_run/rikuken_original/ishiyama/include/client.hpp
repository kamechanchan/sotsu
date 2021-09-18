#pragma once
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_ros/buffer.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/opencv.hpp>
#include <geometry_msgs/TransformStamped.h>

class GetDataNode{
    private:
        ros::NodeHandle nh_;
        ros::Subscriber sub_from_sensor_;
        sensor_msgs::Image in_img_;
        std::string save_img_path_;
        cv::Mat out_img_;
        cv_bridge::CvImagePtr cv_img_;
    public:
        GetDataNode();
        void callback(const sensor_msgs::Image& in_img);
        void convertImgFromRostoTensor();
};