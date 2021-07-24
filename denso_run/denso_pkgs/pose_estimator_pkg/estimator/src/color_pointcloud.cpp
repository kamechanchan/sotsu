#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <color_cloud_bridge/dummy_pcl.h>
#include <color_cloud_bridge/out_segmentation.h>

ros::NodeHandle *pnh;
std::string dummy_topic_name;
std::string publish_topic_name;
std::string instance_topic_name;
std::string frame_id_;

ros::Publisher dummy_pub;
ros::Publisher instance_pub;

void callback(color_cloud_bridge::out_segmentationConstPtr msg)
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    ROS_INFO_STREAM("get msg");
    ROS_INFO_STREAM("cloud size is " << msg->x.size());
    for (int i = 0; i < msg->x.size(); i++) {
        pcl::PointXYZRGB color;
        color.x = msg->x[i];
        color.y = msg->y[i];
        color.z = msg->z[i];
        if (msg->instance[i] == 0)
        {
            color.r = 255;
            color.g = 0;
            color.b = 0;
        }
        else if (msg->instance[i] == 1)
        {
            color.r = 0;
            color.g = 255;
            color.b = 0;
        }
        else if (msg->instance[i] == 2)
        {
            color.r = 0;
            color.g = 0;
            color.b = 255;
        }
        else if (msg->instance[i] == 3)
        {
            color.r = 255;
            color.g = 255;
            color.b = 0;
        }
        else if (msg->instance[i] == 4)
        {
            color.r = 0;
            color.g = 255;
            color.b = 255;
        }
        else if (msg->instance[i] == 5)
        {
            color.r = 255;
            color.g = 0;
            color.b = 255;
        }
        else if (msg->instance[i] == 6)
        {
            color.r = 50;
            color.g = 149;
            color.b = 255;
        }
        cloud.push_back(color);
    }
    cloud.header.frame_id = frame_id_;
    sensor_msgs::PointCloud2 ros_msg;
    pcl::toROSMsg(cloud, ros_msg);
    instance_pub.publish(ros_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "inti");
    ros::NodeHandle nh;
    pnh = new ros::NodeHandle("~");
    pnh->getParam("dummy_topic_name", dummy_topic_name);
    pnh->getParam("instance_topic_name", instance_topic_name);
    pnh->getParam("frame_id", frame_id_);
    instance_pub = nh.advertise<sensor_msgs::PointCloud2>(instance_topic_name, 10);
    ros::Subscriber sub;
    sub = nh.subscribe(dummy_topic_name, 10, callback);
    ros::spin();
    return 0;
}