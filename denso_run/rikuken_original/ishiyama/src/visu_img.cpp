#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_ros/buffer.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/opencv.hpp>
#include <geometry_msgs/TransformStamped.h>


void callback(const sensor_msgs::Image& image){
    // ros::NodeHandle nhh;
    // ros::Publisher pub = nhh.advertise<cv::Mat>("ishiyama_tanomu_img", 10);
    cv_bridge::CvImagePtr cv_img;
    cv::Mat out_img;
    std::string img_path = "/home/ericlab/ros_package/denso_ws/src/denso_run/rikuken_original/ishiyama/result/ishiyama.jpg";
    ROS_INFO("ishiyama");
    cv_img = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    out_img = cv_img->image;
    ROS_INFO_STREAM("kamikami" << out_img.size());
    cv::imwrite(img_path, out_img);
    // pub.publish(out_img);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "ishiyama");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/photoneo_center/sensor/image_color", 1000, callback);
    ros::spin();
    return 0;
}