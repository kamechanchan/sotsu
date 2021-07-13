#pragma once
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <time.h>
#include <fstream>


class CloudLoader
{
public:
    CloudLoader(){}
    CloudLoader(ros::NodeHandle &nh, const std::string &pub_topic_name,
        const std::string &pcd_file_name = ""):
        nh_(nh),
        cloud_pub_(nh_.advertise<sensor_msgs::PointCloud2>(pub_topic_name, 1))
    {
        if (pcd_file_name == "")
            return;
        file_path_ = ros::package::getPath("cloud_practice") + "/data/" + pcd_file_name;
    }

    void load()
    {
        if (file_path_ == "")
            return;
        std::cout << file_path_ << std::endl;
        pcl::io::loadPCDFile(file_path_, cloud_pcl_);
    }

    void convertPCLtoROS()
    {
        pcl::toROSMsg(cloud_pcl_, cloud_ros_);
        cloud_ros_.header.frame_id = "base_link";
    }

    void publish()
    {
        cloud_pub_.publish(cloud_ros_);
    }
    pcl::PointCloud<pcl::PointXYZI> cloud_pcl_;

private:
    ros::NodeHandle nh_;
    ros::Publisher cloud_pub_;
    sensor_msgs::PointCloud2 cloud_ros_;
    std::string file_path_;
};

class CloudOperator
{
public:
    void setInputCloud(const sensor_msgs::PointCloud2 &cloud_input_ros)
    {
        cloud_input_ros_ = cloud_input_ros;
    }
    virtual void operate() = 0;
    virtual void publish() = 0;
protected:
    sensor_msgs::PointCloud2 cloud_input_ros_;
};

class CloudOperationHandler
{
public:
    CloudOperationHandler(ros::NodeHandle &nh,
                            CloudOperator *cloud_operator,
                            const std::string &sub_topic_name) :
        cloud_operator_(cloud_operator),
        cloud_sub_(nh.subscribe(sub_topic_name, 10, &CloudOperationHandler::operateCB, this)),
        count(0)
    {
        out_file = new std::ofstream("/home/ericlab/planar.txt");
    }

    void operateCB(const sensor_msgs::PointCloud2& cloud_input_ros)
    {
        ros::WallTime start = ros::WallTime::now();
        cloud_operator_->setInputCloud(cloud_input_ros);
        cloud_operator_->operate();
        std::cout << "ukerta" << std::to_string(count++) << std::endl;
        cloud_operator_->publish();
        ros::WallTime end = ros::WallTime::now();
        ros::WallDuration take_time = end - start;
        double measure_time = take_time.toSec();
        *out_file << count << "ループ目の前処理の時間は" << measure_time << "秒" << std::endl;
    }

       

protected:
    CloudOperator *cloud_operator_;
    ros::Subscriber cloud_sub_;
    std::ofstream *out_file;
    int count;
    
};
