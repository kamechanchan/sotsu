#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tsuchida");
    tf2::Quaternion quat_start, q_moto;
    geometry_msgs::TransformStamped static_tf, dynamic_tf;
    geometry_msgs::Transform tf_value;
    static_tf.child_frame_id = "tsuchida_static";
    static_tf.header.frame_id = "world";
    tf_value.translation.x = 0;
    tf_value.translation.y = 1;
    tf_value.translation.z = 0.5;
    quat_start.setRPY(M_PI / 4, M_PI / 10, M_PI / 3);
    tf_value.rotation.x = quat_start[0];
    tf_value.rotation.y = quat_start[1];
    tf_value.rotation.z = quat_start[2];
    tf_value.rotation.w = quat_start[3];
    tf2_ros::StaticTransformBroadcaster sta_br;
    static_tf.transform = tf_value;
    static_tf.header.stamp = ros::Time::now();
    sta_br.sendTransform(static_tf);
    

}