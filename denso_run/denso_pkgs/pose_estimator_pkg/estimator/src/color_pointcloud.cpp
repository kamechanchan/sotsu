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
    // pcl::PointCloud<pcl::PointXYZ> cloud;
    ROS_INFO_STREAM("get msg");
    ROS_INFO_STREAM("cloud size is " << msg->x.size());
    ROS_INFO_STREAM("are");
    int color_0 = 0;
    int color_1 = 0;
    for (int i = 0; i < msg->x.size(); i++) {
        // pcl::PointXYZRGB color;
        pcl::PointXYZRGB color;
        

        color.x = msg->x[i];
        color.y = msg->y[i];
        color.z = msg->z[i];
        // for (int j = 0; j < 25; j++){
        //     if (msg->instance[i] == j){
        //         color.r = 255 / j+1;
        //         color.g = 10.2 * j+1;
        //         color.b = 255 / j+1;
        //     }
        // }
        if (msg->instance[i] == 0)
        {
            color.r = 255;
            color.g = 0;
            color.b = 0;
            color_0++;
        }
        else if (msg->instance[i] == 1)
        {
            color.r = 0;
            color.g = 255;
            color.b = 0;
            color_1++;
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
            color.r = 30;
            color.g = 100;
            color.b = 200;
        }
        else if (msg->instance[i] == 7)
        {
            color.r = 200;
            color.g = 0;
            color.b = 100;
        }
        else if (msg->instance[i] == 8)
        {
            color.r = 200;
            color.g = 150;
            color.b = 0;
        }
        else if (msg->instance[i] == 9)
        {
            color.r = 75;
            color.g = 75;
            color.b = 75;
        }
        else if (msg->instance[i] == 10)
        {
            color.r = 50;
            color.g = 0;
            color.b = 150;
        }
        else if (msg->instance[i] == 11)
        {
            color.r = 100;
            color.g = 0;
            color.b = 150;
        }
        else if (msg->instance[i] == 12)
        {
            color.r = 150;
            color.g = 0;
            color.b = 150;
        }
        else if (msg->instance[i] == 13)
        {
            color.r = 200;
            color.g = 0;
            color.b = 150;
        }
        else if (msg->instance[i] == 14)
        {
            color.r = 255;
            color.g = 0;
            color.b = 150;
        }
        else if (msg->instance[i] == 15)
        {
            color.r = 0;
            color.g = 50;
            color.b = 150;
        }
        else if (msg->instance[i] == 16)
        {
            color.r = 0;
            color.g = 100;
            color.b = 150;
        }
        else if (msg->instance[i] == 17)
        {
            color.r = 0;
            color.g = 150;
            color.b = 150;
        }
        else if (msg->instance[i] == 18)
        {
            color.r = 0;
            color.g = 200;
            color.b = 150;
        }
        else if (msg->instance[i] == 19)
        {
            color.r = 0;
            color.g = 255;
            color.b = 150;
        }
        else if (msg->instance[i] == 20)
        {
            color.r = 150;
            color.g = 0;
            color.b = 50;
        }
        else if (msg->instance[i] == 21)
        {
            color.r = 150;
            color.g = 0;
            color.b = 100;
        }
        else if (msg->instance[i] == 22)
        {
            color.r = 150;
            color.g = 0;
            color.b = 150;
        }
        else if (msg->instance[i] == 23)
        {
            color.r = 150;
            color.g = 0;
            color.b = 200;
        }
        else if (msg->instance[i] == 24)
        {
            color.r = 150;
            color.g = 0;
            color.b = 255;
        }
        else {
            color.r = 255;
            color.g = 255;
            color.b = 255;
        }
        cloud.push_back(color);
    }
    cloud.header.frame_id = frame_id_;
    sensor_msgs::PointCloud2 ros_msg;
    pcl::toROSMsg(cloud, ros_msg);
    instance_pub.publish(ros_msg);
    ROS_INFO_STREAM("instance 0: " << color_0 << "  1: " << color_1);
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