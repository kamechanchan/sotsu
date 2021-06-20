#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

ros::NodeHandle *pnh;
std::string pcd_dir_name;
std::string sensor_topic_name;
static int count = 0;

void callback(sensor_msgs::PointCloud2ConstPtr msg)
{
    bool hantei;
    pnh->getParam("/is_record_kekkyoku/ok", hantei);
    ROS_INFO_STREAM("hantei = " << hantei);
    if (hantei) {
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::fromROSMsg(*msg, cloud);
        std::string path = pcd_dir_name + "save_" + std::to_string(count++) + ".pcd";
        pcl::io::savePCDFile(path, cloud);
        ROS_INFO_STREAM("save file num is " << count);
        ROS_INFO_STREAM(path);
        pnh->setParam("/is_record_kekkyoku/ok", false);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "save_pcd");
    ros::NodeHandle nh;
    pnh = new ros::NodeHandle("~");
    pnh->getParam("pcd_dir_name", pcd_dir_name);
    pnh->getParam("sensor_topic_name", sensor_topic_name);
    ros::Subscriber sub;
    sub = nh.subscribe(sensor_topic_name, 10, callback);
    ros::spin();
    return 0;

}