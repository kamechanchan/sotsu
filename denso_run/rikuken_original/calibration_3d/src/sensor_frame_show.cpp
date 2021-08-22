#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

void callback(const sensor_msgs::PointCloud2ConstPtr msg)
{
    ROS_INFO_STREAM(msg->header.frame_id);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "init");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    std::string tengun_topic;
    pnh.getParam("topic_name", tengun_topic);
    ros::Subscriber sub;
    sub = nh.subscribe(tengun_topic, 10, callback);
    ros::spin();
    return 0;
}