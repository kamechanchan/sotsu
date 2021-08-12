#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <color_cloud_bridge/dummy_pcl.h>
#include <color_cloud_bridge/sensor_and_index.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef sensor_msgs::PointCloud2ConstPtr yes;
std::string sensor_topic_name;
std::string publish_topic_name;
std::string instance_topic_name;
static int count = 0;
ros::Publisher dummy_pub;
ros::Publisher instance_pub;
ros::NodeHandle *pnh;

typedef sensor_msgs::PointCloud2 po2;
typedef message_filters::sync_policies::ApproximateTime<po2, po2, po2, po2, po2, po2, po2> Sensor_Sync_Sub_Type;
void callback(yes msg1, yes msg2, yes msg3, yes msg4, yes msg5, yes msg6, yes msg7)
{
    
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "eier");
    ros::NodeHandle nh;
    pnh = new ros::NodeHandle("~");
    pnh->getParam("sensor_topic_name", sensor_topic_name);
    pnh->getParam("publish_topic_name", publish_topic_name);
    dummy_pub = nh.advertise<color_cloud_bridge::dummy_pcl>(publish_topic_name, 10);
    instance_pub = nh.advertise<color_cloud_bridge::sensor_and_index>(instance_topic_name, 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sens_1(nh, sensor_topic_name+'_'+std::to_string(0), 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sens_2(nh, sensor_topic_name+'_'+std::to_string(1), 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sens_3(nh, sensor_topic_name+'_'+std::to_string(2), 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sens_4(nh, sensor_topic_name+'_'+std::to_string(3), 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sens_5(nh, sensor_topic_name+'_'+std::to_string(4), 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sens_6(nh, sensor_topic_name+'_'+std::to_string(5), 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sens_7(nh, sensor_topic_name+'_'+std::to_string(6), 10);
    message_filters::Synchronizer<Sensor_Sync_Sub_Type> sync(Sensor_Sync_Sub_Type(10), sens_1, sens_2, sens_3, sens_4, sens_5, sens_6, sens_7);
    sync.registerCallback(boost::bind(callback, _1, _2, _3, _4, _5, _6, _7));
    ros::Subscriber sub;
    
    ros::spin();
}

