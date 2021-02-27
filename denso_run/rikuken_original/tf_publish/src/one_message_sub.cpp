#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "zen_int");
    std_msgs::String msgs;
    ros::NodeHandle nh;
    boost::shared_ptr<const std_msgs::String> msgs_baikai;
    msgs_baikai = ros::topic::waitForMessage<std_msgs::String>("shinya", nh, ros::Duration(5));
    if (msgs_baikai != NULL)
    {
        ROS_INFO_STREAM(*msgs_baikai);
    }
    return 0;
}