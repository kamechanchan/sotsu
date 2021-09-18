#include <arm_move/arm_move.hpp>

Arm_Move::Arm_Move()
: spinner(1)
{
    spinner.start();
}

void Arm_Move::arm_register(std::string arm_group)
{
    arm_group_ = new moveit::planning_interface::MoveGroupInterface(arm_group);
    show_value<std::string>(arm_group_->getJointNames());
}

void Arm_Move::hand_register(std::string hand_group)
{
    hand_group_ = new moveit::planning_interface::MoveGroupInterface(hand_group);
    show_value<std::string>(hand_group_->getJointNames());
}

void Arm_Move::show_hand_joint()
{
    show_value<double>(hand_group_->getCurrentJointValues());
}

void Arm_Move::show_arm_joint()
{
    show_value<double>(arm_group_->getCurrentJointValues());
}

void Arm_Move::hand_close()
{
    std::vector<double> joint_value;
    joint_value = hand_group_->getCurrentJointValues();
    joint_value[0] = close_range_;
    hand_group_->setJointValueTarget(joint_value);
    hand_group_->move();
}

void Arm_Move::set_close_range(double range)
{
    close_range_ = range;
}

void Arm_Move::hand_open()
{
    std::vector<double> joint_value;
    joint_value = hand_group_->getCurrentJointValues();
    joint_value[0] = 0;
    hand_group_->setJointValueTarget(joint_value);
    hand_group_->move();
}

void Arm_Move::tf