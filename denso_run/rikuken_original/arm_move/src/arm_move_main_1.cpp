#include <arm_move/arm_move.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tsuchida_arm");
    Arm_Move arm_hand;
    arm_hand.arm_register("manipulator");
    arm_hand.hand_register("hand");
    arm_hand.set_close_range(0.0197);
    arm_hand.show_arm_joint();
    arm_hand.show_hand_joint();
    // arm_hand.hand_close();
    // arm_hand.hand_open();
    // arm_hand.move_end_effector(0, 0, 0.1, 0.001);
    
    // arm_hand.return_home();
    
    geometry_msgs::Point pon1, pon2, pon;
    pon1 = arm_hand.get_pose_tf("body_link", "world");
    pon2 = arm_hand.get_pose_tf("HV8_0", "world");
    pon.x = pon1.x - pon2.x;
    pon.y = pon1.y - pon2.y;
    pon.z = pon1.z - pon2.z;
    
    std::cout << "x: " << pon.x << "  y: " << pon.y << "   z: " << pon.z << std::endl;
    
    // arm_hand.move_end_effector(pon.x, 0, 0, 0.001);
    // arm_hand.move_end_effector(0, pon.y, 0, 0.001);
    ros::Rate loop(2);
    int count = 0;
    arm_hand.move_end_effector(pon.x, pon.y, pon.z + 0.08, 0.0005);
    arm_hand.hand_close();
    while (count <= 2) {
        count++;
        loop.sleep();
    }
    arm_hand.move_end_effector(0, 0, 0.2, 0.001);
    arm_hand.move_end_effector(0, -0.3, 0, 0.001);
    arm_hand.move_end_effector(0, 0, -0.2, 0.001);
    arm_hand.hand_open();
    arm_hand.return_home();


    return 0;
}