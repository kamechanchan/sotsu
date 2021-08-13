#pragma once
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/opencv.hpp>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/TransformStamped.h>

class Annotation_yolo
{
public:
    Annotation_yolo(ros::NodeHandle&);
    void tf_get(std::string, std::string, geometry_msgs::TransformStamped&);
    void InputCallback(sensor_msgs::CameraInfoConstPtr, sensor_msgs::ImageConstPtr);
    void parameter_set();
    void box_get(sensor_msgs::CameraInfo, sensor_msgs::Image, geometry_msgs::TransformStamped, cv::Mat&);
    void box_get(sensor_msgs::CameraInfo, sensor_msgs::Image, std::vector<geometry_msgs::TransformStamped>, cv::Mat&, int);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::Image> Sync_Sub_type;
    cv::Point2d project3d_to_pixel(cv::Point3d, sensor_msgs::CameraInfo);
    void paramter_set_bara(std::string, int);
private:
    message_filters::Synchronizer<Sync_Sub_type> *sensor_sync_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *camera_sub_;
    message_filters::Subscriber<sensor_msgs::Image> *image_sub_;
    ros::NodeHandle nh_;
    ros::NodeHandle *pnh_;
    std::string source_frame_, target_frame_;
    std::vector<std::string> target_frames_;
    std::string camera_topic_name_, image_topic_name_;
    tf2_ros::TransformListener *lister_;
    tf2_ros::Buffer *buffer_;
    cv::Mat draw_image_;
    double f_scale_, cx_scale_, cy_scale_;
    double fx_, fy_, tx_, ty_, cx_, cy_;
    float radious_;
    bool write_is_ok_;
    std::string save_dir_name_, filebasename_, model_name_;
    int save_count_;
    int work_count_;

};