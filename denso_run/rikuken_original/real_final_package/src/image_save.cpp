#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv4/opencv2/opencv.hpp>

// template <typename T>
// void get_one_message(T &img, std::string topic_name, int timeout)
// {
//     boost::shared_ptr<const T> share;
//     share = ros::topic::waitForMessage<T>(topic_name, ros::Duration(timeout));
//     cv_bridge::CvImagePtr input_bridge;
//     if (share != NULL) {
//         input_bridge = cv_bridge::toCvCopy(share, sensor_msgs::image_encodings::BGR8);
//     }

// }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tsuchida_");
    ros::NodeHandle pnh("~");
    
    std::string image_topic;
    int timeout;
    std::string save_image_file;
    pnh.getParam("timeout", timeout);
    pnh.getParam("image_topic", image_topic);
    pnh.getParam("save_image_file", save_image_file);
    sensor_msgs::ImageConstPtr share;
    share = ros::topic::waitForMessage<sensor_msgs::Image>(image_topic, ros::Duration(timeout));
    cv_bridge::CvImagePtr input_bridge;
    if (share != NULL) {
        input_bridge = cv_bridge::toCvCopy(share, sensor_msgs::image_encodings::BGR8);
    }
    else {
        ROS_ERROR_STREAM("Fail to get image!!");
        return 0;
    }
    cv::Mat img;
    img = input_bridge->image;
    cv::imshow("mask_show", img);
    cv::waitKey(3000);
    cv::imwrite(save_image_file, img);
    cv::destroyAllWindows();
    share.reset();
    return 0;
    


}