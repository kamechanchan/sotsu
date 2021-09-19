#include <arm_move/arm_move.hpp>

Arm_Move::Arm_Move()
: spinner(1)
, tfBuffer_()
, tfListener_(tfBuffer_)
, dulation_(0.1)
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

void Arm_Move::tf_get(std::string source_frame, std::string target_frame, geometry_msgs::TransformStamped &transform)
{
    try
    {
        transform = tfBuffer_.lookupTransform(source_frame, target_frame, ros::Time(0));
        ROS_INFO_ONCE("I got a transform");
    }  
    catch (tf2::TransformException &e)
    {
        ROS_WARN_STREAM(e.what());
        tfBuffer_.clear();
        ros::Duration(dulation_).sleep();
        return;
    }
}

void Arm_Move::show_tf_value(std::string source_frame, std::string target_frame)
{
    tf_get(source_frame, target_frame, transform_);
    point_ = transform_to_target_point(transform_);
    std::cout << "x: " << point_.x << "  y: " << point_.y << "   z: " << point_.z << std::endl;
    // std::cout << "rx: " << trans.rotation.x << "   ry: " << trans.rotation.y << "   rz: " << trans.rotation.z << "   rw: " << trans.rotation.w << std::endl;
}

geometry_msgs::Point Arm_Move::get_pose_tf(std::string source, std::string target)
{
    tf_get(source, target, transform_);
    return transform_to_target_point(transform_);
}

geometry_msgs::Point Arm_Move::transform_to_target_point(geometry_msgs::TransformStamped transform)
{
    tf2::Quaternion q_moto(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z, 0);
    tf2::Quaternion q_rot;
    tf2::convert(transform.transform.rotation, q_rot);
    tf2::Quaternion q_after = q_rot.inverse() * q_moto * q_rot;
    geometry_msgs::Point point;
    point.x = q_after.x();
    point.y = q_after.y();
    point.z = q_after.z();
    return point;
}

void Arm_Move::move_end_effector(double x1, double y1, double z1, double eef_step)
{
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose wpose = arm_group_->getCurrentPose().pose;
    wpose.position.x += x1;
    wpose.position.y += y1;
    wpose.position.z += z1;
    waypoints.push_back(wpose);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_thresh = 0.0;
    double fraction = arm_group_->computeCartesianPath(waypoints, eef_step
                                , jump_thresh, trajectory);
    arm_group_->execute(trajectory);
}