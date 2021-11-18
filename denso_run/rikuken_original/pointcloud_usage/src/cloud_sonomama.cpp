#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types_conversion.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher ros_pub;

void callback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    sensor_msgs::PointCloud2 ros_msg;
    cloud.header.frame_id = msg->header.frame_id;
    ROS_INFO_STREAM(cloud.header.frame_id);
    pcl::toROSMsg(cloud, ros_msg);
    ros_pub.publish(ros_msg);  
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_init");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    std::string input_topic_name, output_topic_name;
    pnh.getParam("input_topic_name", input_topic_name);
    pnh.getParam("output_topic_name", output_topic_name);
    ros_pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic_name, 10);
    ros::Subscriber sub;
    sub = nh.subscribe(input_topic_name, 10, callback);
    ros::spin();
}