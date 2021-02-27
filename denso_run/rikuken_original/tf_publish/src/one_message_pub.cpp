#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "init");
    ros::NodeHandle nh;
    ros::Publisher pub;
    pub = nh.advertise<std_msgs::String>("shinya", 10);
    std_msgs::String msg;
    std::cin >> msg.data;
    pub.publish(msg);
    ROS_INFO_STREAM(msg.data);
    return 0;
    
}