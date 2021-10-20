#include <arm_move/arm_move.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tsuchida_arm");
    Arm_Move arm_hand;
    ros::NodeHandle pnh("~");
    std::string target_frame;
    double  z_br;
    pnh.getParam("target_frame", target_frame);
    pnh.getParam("tf_z", z_br);
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::TransformStamped>("tsuchida_tf_pub", 10);
    arm_hand.arm_register("manipulator");
    arm_hand.hand_register("hand");
    arm_hand.set_close_range(0.01953);
    arm_hand.show_arm_joint();
    arm_hand.show_hand_joint();
    // arm_hand.hand_close();
    // arm_hand.hand_open();
    // arm_hand.move_end_effector(0, 0, 0.1, 0.001);
    
    // arm_hand.return_home();
    
    geometry_msgs::Point pon1, pon2, pon;
    pon1 = arm_hand.get_pose_tf("body_link", "world");
    pon2 = arm_hand.get_pose_tf(target_frame, "world");

    pon.x = pon1.x - pon2.x;
    pon.y = pon1.y - pon2.y;
    // pon.z = pon1.z - pon2.z;

    geometry_msgs::TransformStamped final_tf;
    final_tf = arm_hand.get_pose_tf(target_frame, "world", z_br);
    final_tf.header.frame_id = "world";
    final_tf.child_frame_id = "tsuchida_Tf";
    final_tf.header.stamp = ros::Time::now();
    pub.publish(final_tf);
    // geometry_msgs::Transform final_yes, final_yes_moto;
    // final_yes = arm_hand.transform_to_target_point(final_tf.transform);
    tf2_ros::TransformBroadcaster br;
    // final_yes = final_tf.transform;
    // final_yes.translation.x = pon.x;
    // final_yes.translation.y = pon.y;
    // final_yes.translation.z = pon.z;
    // final_tf.transform.translation.x = pon.x;
    // final_tf.transform.translation.y = pon.y;
    // final_tf.transform.translation.z = pon.z + 0.08;

    
    ros::Rate loop(2), loop_tf(10);
    int count = 0;

    // while (ros::ok())
    // {
    //     final_tf.header.frame_id = "world";
    //     final_tf.child_frame_id = "tsuchida_Tf";
    //     final_tf.header.stamp = ros::Time::now();
    //     br.sendTransform(final_tf);
    //     loop_tf.sleep();
    // }
    // arm_hand.move_end_effector(pon.x, pon.y, pon.z + 0.08, 0.0005);
    arm_hand.move_end_effector_set_tf(final_tf, 0.001, pon.x, pon.y);
    arm_hand.hand_close();
    while (count <= 2) {
        count++;
        loop.sleep();
    }
    arm_hand.move_end_effector(0, 0, 0.2, 0.001);
    // arm_hand.move_end_effector(0.0, -0.2, 0, 0.001);
    // arm_hand.move_end_effector(0, 0, -0.1, 0.001);
    arm_hand.hand_open();
    arm_hand.return_home();


    return 0;
}