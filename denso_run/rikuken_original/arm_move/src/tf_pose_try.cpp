#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>

geometry_msgs::TransformStamped set_tf(std::string child, 
    double x, double y, double z, tf2::Quaternion quat)
{
    geometry_msgs::Transform tf_value;
    geometry_msgs::TransformStamped tfyes;
    tfyes.child_frame_id = child;
    tfyes.header.frame_id = "world";
    tfyes.header.stamp = ros::Time::now();
    tf_value.translation.x = x;
    tf_value.translation.y = y;
    tf_value.translation.z = z;
    tf_value.rotation.x = quat[0];
    tf_value.rotation.y = quat[1];
    tf_value.rotation.z = quat[2];
    tf_value.rotation.w = quat[3];
    tfyes.transform = tf_value;
    return tfyes;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tsuchida");
    tf2::Quaternion quat_start, q_moto;
    geometry_msgs::TransformStamped static_tf, dynamic_tf;
    quat_start.setRPY(M_PI / 4, M_PI / 10, M_PI / 3);
    static_tf = set_tf("tsuchida_moto", 0, 1, 0.5, quat_start);
    tf2_ros::StaticTransformBroadcaster sta_br;
    sta_br.sendTransform(static_tf);
    

}