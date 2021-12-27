#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <geometry_msgs/TransformStamped.h>

tf2_ros::TransformBroadcaster *br;
void callback(const geometry_msgs::TransformStampedConstPtr msg)
{
    ros::Rate lop(10);
    int count = 0;
    while (count <= 10) {
        br->sendTransform(*msg);
        lop.sleep();
        count++;
    }
}
    

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tsu");
    br = new tf2_ros::TransformBroadcaster();
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("tsuchida_tf_pub", 10, callback);
    ros::spin();
}