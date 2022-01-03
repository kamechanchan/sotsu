#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <time.h>
#include <fstream>
#include <cloud_practice/img_cloud_common.h>


CloudLoader::CloudLoader(ros::NodeHandle &nh, const std::string &pub_topic_name, const std::string &pcd_file_name = ""):
    nh_(nh),
    cloud_pub_(nh_.advertise<sensor_msgs::PointCloud2>(pub_topic_name, 1))
{
    if (pcd_file_name == "")
            return;
        file_path_ = ros::package::getPath("cloud_practice") + "/data/" + pcd_file_name;
}

void CloudLoader::load()
{
    if (file_path_ == "")
        return;
    std::cout << file_path_ << std::endl;
    pcl::io::loadPCDFile(file_path_, cloud_pcl_);
}

void CloudLoader::convertPCLtoROS()
{
    pcl::toROSMsg(cloud_pcl_, cloud_ros_);
    cloud_ros_.header.frame_id = "base_link";
}

void CloudLoader::publish()
{
    cloud_pub_.publish(cloud_ros_);
}


void CloudOperator::setInputCloud(const sensor_msgs::PointCloud2 &cloud_input_ros, const sensor_msgs::Image &img_input_ros)
    {
        cloud_input_ros_ = cloud_input_ros;
        img_input_ros_ = img_input_ros;
    }


CloudOperationHandler::CloudOperationHandler(ros::NodeHandle &nh,
                            CloudOperator *cloud_operator,
                            const std::string &cloud_topic_name, 
                            const std::string &img_topic_name, 
                            int timespan) :
        cloud_operator_(cloud_operator),
        count(0)
{
    // out_file = new std::ofstream("/home/ericlab/planar.txt");
    ros::WallTime start = ros::WallTime::now();
    get_one_message(cloud_input_ros_, cloud_topic_name, nh, timespan);
    get_one_message(img_input_ros_, img_topic_name, nh, timespan);
    cloud_operator_->setInputCloud(cloud_input_ros_, img_input_ros_);
    cloud_operator_->operate();
    // std::cout << "ukerta" << std::to_string(count++) << std::endl;
    cloud_operator_->publish();
    ros::WallTime end = ros::WallTime::now();
    ros::WallDuration take_time = end - start;
    double measure_time = take_time.toSec();
    // *out_file << count << "ループ目の前処理の時間は" << measure_time << "秒" << std::endl;
}