#include <arm_move/arm_move.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tsuchida_arm");
    Arm_Move arm_hand;
    arm_hand.arm_register("manipulator");
    arm_hand.hand_register("hand");
    arm_hand.set_close_range(0.017);
    arm_hand.show_arm_joint();
    arm_hand.show_hand_joint();
    arm_hand.hand_close();
    arm_hand.hand_open();
    return 0;
}