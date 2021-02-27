#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <math.h>

class Converter
{
public:
    Converter(void)
    {
        ros::param::param<std::string>("~object", object_name_);
        pnh = new ros::NodeHandle("~");
        pnh->getParam("object_name", object_name_);
        joy_sub_= nh_.subscribe("/groud_truth/" + object_name_ + "/pose", 10, &Converter::odomCallback, this);

    }

    void odomCallback(const nav_msgs::OdometryConstPtr &msg)
    {
        tf::Transform transform;
        tf::poseMsgToTF(msg->pose.pose, transform);
        br_.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "world", msg->child_frame_id));
    }

    ros::NodeHandle nh_;
    ros::Subscriber joy_sub_;
    tf::TransformBroadcaster br_;
    std::string object_name_;
    ros::NodeHandle *pnh;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_tf_converter");
    Converter converter;
    ros::spin();
    return 0;
}