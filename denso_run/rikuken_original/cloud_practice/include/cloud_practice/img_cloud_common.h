#pragma once
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <time.h>
#include <fstream>


template <typename T>
void get_one_message(T &final_message, std::string message_name, ros::NodeHandle nh, int timespan)
{
    boost::shared_ptr<const T> share;
    share = ros::topic::waitForMessage<T>(message_name, nh, ros::Duration(timespan));
    if (share != NULL) {
        final_message = *share;
    }
    share.reset();
}

class CloudLoader
{
public:
    CloudLoader(ros::NodeHandle &nh, const std::string &pub_topic_name,const std::string &pcd_file_name);
    void load();
    void convertPCLtoROS();
    void publish();
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
    void setInputCloud(const sensor_msgs::PointCloud2 &cloud_input_ros, const sensor_msgs::Image &img_input_ros);
    virtual void operate() = 0;
    virtual void publish() = 0;
protected:
    sensor_msgs::PointCloud2 cloud_input_ros_;
    sensor_msgs::Image img_input_ros_;
};

class CloudOperationHandler
{
public:
    CloudOperationHandler(ros::NodeHandle &nh,
                            CloudOperator *cloud_operator,
                            const std::string &cloud_topic_name, 
                            const std::string &img_topic_name, 
                            int timespan);
    CloudOperator *cloud_operator_;
    // std::ofstream *out_file;
    sensor_msgs::PointCloud2 cloud_input_ros_;
    sensor_msgs::Image img_input_ros_;
    int count;

// protected:
//     CloudOperator *cloud_operator_;
//     // std::ofstream *out_file;
//     const sensor_msgs::PointCloud2& cloud_input_ros_;
//     const sensor_msgs::Image& img_input_ros_;
//     int count;
    
};