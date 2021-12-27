#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "static_tf_broadcaster");
    geometry_msgs::TransformStamped static_transform;
    tf2_ros::TransformBroadcaster tf_br_;
    tf2_ros::StaticTransformBroadcaster st_br_;
    
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    float x, y, z, roll, pitch, yaw;
    std::string parent_frame, child_frame;
    pnh.getParam("x", x);
    pnh.getParam("y", y);
    pnh.getParam("z", z);
    pnh.getParam("roll", roll);
    pnh.getParam("pitch", pitch);
    pnh.getParam("yaw", yaw);
    pnh.getParam("parent_frame", parent_frame);
    pnh.getParam("child_frame", child_frame);
    static_transform.header.stamp = ros::Time::now();
    static_transform.header.frame_id = parent_frame;
    static_transform.child_frame_id = child_frame;
    static_transform.transform.translation.x = x;
    static_transform.transform.translation.y = y;
    static_transform.transform.translation.z = z;
    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);
    static_transform.transform.rotation.x = quat.x();
    static_transform.transform.rotation.y = quat.y();
    static_transform.transform.rotation.z = quat.z();
    static_transform.transform.rotation.w = quat.w();
    ros::Rate loop(10);
    while (ros::ok())
    {
        tf_br_.sendTransform(static_transform);
        loop.sleep();
    }
    
    return 0;
    

}