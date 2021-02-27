#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_tf2");
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tfListener(tf_buffer);
    ros::NodeHandle pnh("~");
    std::string target = "dynamic_link";
    std::string source = "source_link";
    pnh.getParam("target", target);
    pnh.getParam("source", source);

    ros::Rate rate(10.0);
    while (ros::ok())
    {
        geometry_msgs::TransformStamped ts;
        try {
            ts = tf_buffer.lookupTransform(target, source, ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        rate.sleep();
    }
    return 0;
}