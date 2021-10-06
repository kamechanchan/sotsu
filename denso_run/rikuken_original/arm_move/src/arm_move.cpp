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
    arm_group_->setMaxVelocityScalingFactor(1.0);
    
}

void Arm_Move::hand_register(std::string hand_group)
{
    hand_group_ = new moveit::planning_interface::MoveGroupInterface(hand_group);
    moveit_msgs::MotionPlanRequest res;
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
    double move_range = close_range_ - joint_value[0];
    double sampling_range = move_range / 20;
    ros::Rate loop(20);
    for (int i = 0; i < 20; i++) {
        joint_value[0] += sampling_range;
        hand_group_->setJointValueTarget(joint_value);
        hand_group_->move();
        loop.sleep();
    }
    // joint_value[0] = close_range_;
    
    // hand_group_->setJointValueTarget(joint_value);
    // hand_group_->setMaxVelocityScalingFactor(0.01);
    // hand_group_->move();
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

geometry_msgs::TransformStamped Arm_Move::get_pose_tf(std::string source, std::string target, double z_temae)
{
    tf_get(target, source, transform_);
    tf2::Quaternion q_zero(0, 0, z_temae, 0);
    geometry_msgs::Transform trans = transform_.transform;
    tf2::Quaternion q_moto(trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w);
    tf2::Quaternion q_convert;
    q_convert.setRPY(0, M_PI, 0);
    tf2::Quaternion trans_ato, rotate_at;
    trans_ato = q_moto * q_zero * q_moto.inverse();
    rotate_at = q_convert * q_moto;
    q_convert.setRPY(0, 0, M_PI);
    rotate_at = q_convert * rotate_at;
    geometry_msgs::TransformStamped final_pose;
    final_pose.transform.translation.x = trans_ato[0] + trans.translation.x;
    final_pose.transform.translation.y = trans_ato[1] + trans.translation.y;
    final_pose.transform.translation.z = trans_ato[2] + trans.translation.z;
    // final_pose.transform.translation.x = trans_ato[0];
    // final_pose.transform.translation.y = trans_ato[1];
    // final_pose.transform.translation.z = trans_ato[2];
    final_pose.transform.rotation.x = rotate_at[0];
    final_pose.transform.rotation.y = rotate_at[1];
    final_pose.transform.rotation.z = rotate_at[2];
    final_pose.transform.rotation.w = rotate_at[3];
    return final_pose;
    
    

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


geometry_msgs::Transform Arm_Move::transform_to_target_point(geometry_msgs::Transform transform)
{
    tf2::Quaternion q_moto(transform.translation.x, transform.translation.y, transform.translation.z, 0);
    tf2::Quaternion q_rot;
    tf2::convert(transform.rotation, q_rot);
    tf2::Quaternion q_after = q_rot.inverse() * q_moto * q_rot;
    tf2::Quaternion q_convert, q_final;
    q_convert.setRPY(0, M_PI, 0);
    q_final = q_convert * q_rot;
    geometry_msgs::Transform point;
    point.translation.x = q_after.x();
    point.translation.y = q_after.y();
    point.translation.z = q_after.z();
    point.rotation.x = q_final.x();
    point.rotation.y = q_final.y();
    point.rotation.z = q_final.z();
    point.rotation.w = q_final.w();
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

void Arm_Move::move_end_effector_set_tf(double x1, double y1, double z1, double roll, double pitch, double yaw, double eef_step)
{
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose wpose = arm_group_->getCurrentPose().pose;
    wpose.position.x = x1;
    wpose.position.y = y1;
    wpose.position.z = z1;
    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);
    tf2::convert(quat, wpose.orientation);
    waypoints.push_back(wpose);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_thresh = 0.0;
    double fraction = arm_group_->computeCartesianPath(waypoints, eef_step,
                                            jump_thresh, trajectory);
    arm_group_->execute(trajectory);                                                                                        
}

void Arm_Move::move_end_effector_set_tf(double x1, double y1, double z1, tf2::Quaternion quat, double eef_step)
{
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose wpose = arm_group_->getCurrentPose().pose;
    wpose.position.x = x1;
    wpose.position.y = y1;
    wpose.position.z = z1;
    tf2::convert(quat, wpose.orientation);
    waypoints.push_back(wpose);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_thresh = 0.0;
    double fraction = arm_group_->computeCartesianPath(waypoints, eef_step,
                                            jump_thresh, trajectory);
    arm_group_->execute(trajectory);
}

void Arm_Move::return_home()
{
    std::vector<double> joint_value = {0.220046, -1.6059, 1.24065, -1.14717, -1.5684, -1.57072};
    arm_group_->setJointValueTarget(joint_value);
    arm_group_->move();
}

void seteuler(double &kakudo)
{
    if (kakudo >= M_PI / 2) {
        kakudo = M_PI - kakudo;
    }
    else if (kakudo <= -M_PI / 2) {
        kakudo = -M_PI - kakudo;
    }
    std::cout << "M_PI: " << M_PI << "   kakudo: " << kakudo << std::endl;
    if (kakudo >= M_PI / 4) {
        kakudo = kakudo - M_PI / 4;
    }
    else if (kakudo <= -M_PI / 4) {
        kakudo = kakudo + M_PI / 4;
    }
}

void Arm_Move::move_end_effector_set_tf(geometry_msgs::TransformStamped trans, double eef_step)
{
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose wpose = arm_group_->getCurrentPose().pose;
    wpose.position.x = trans.transform.translation.x;
    wpose.position.y = trans.transform.translation.y;
    wpose.position.z = trans.transform.translation.z;
    tf2::Quaternion quat, q_moto, q_ato, q_object, q_zero, q_object_c;
    q_zero.setRPY(0, 0, 0);
    // quat.setRPY(-M_PI / 10, 0, 0);
    tf2::convert(trans.transform.rotation, q_object);
    q_object_c =  q_zero.inverse() * q_object;
    double yaw, roll, pitch;
    tf2::getEulerYPR(q_object, yaw, pitch, roll);
    std::cout << "roll: " << roll << "   pitch: " << pitch << "  yaw: " << yaw << std::endl;
    quat.setRPY(0, 0, yaw);
    q_object = quat.inverse() * q_object * quat;
    q_object_c =  q_zero.inverse() * q_object;
    
    tf2::getEulerYPR(q_object_c, yaw, pitch, roll);
    std::cout << "roll: " << roll << "   pitch: " << pitch << "  yaw: " << yaw << std::endl;
    seteuler(roll);
    seteuler(pitch);
    std::cout << "roll: " << roll << "   pitch: " << pitch << std::endl;
    quat.setRPY(-roll, -pitch, 0);

    tf2::convert(wpose.orientation, q_moto);

    q_ato = quat * q_moto;
    
    wpose.orientation.x = q_ato[0];
    wpose.orientation.y = q_ato[1];
    wpose.orientation.z = q_ato[2];
    wpose.orientation.w = q_ato[3];
    // wpose.orientation.x = trans.transform.rotation.x;
    // wpose.orientation.y = trans.transform.rotation.y;
    // wpose.orientation.z = trans.transform.rotation.z;
    // wpose.orientation.w = trans.transform.rotation.w;
    waypoints.push_back(wpose);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_thresh = 0.0;
    double fraction = arm_group_->computeCartesianPath(waypoints, eef_step,
                                            jump_thresh, trajectory);
    arm_group_->execute(trajectory);
}

void Arm_Move::move_end_effector_set_tf(geometry_msgs::Transform trans, double eef_step)
{
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose wpose = arm_group_->getCurrentPose().pose;
    wpose.position.x += trans.translation.x;
    wpose.position.y += trans.translation.y;
    wpose.position.z += trans.translation.z;
    wpose.orientation.x = trans.rotation.x;
    wpose.orientation.y = trans.rotation.y;
    wpose.orientation.z = trans.rotation.z;
    wpose.orientation.w = trans.rotation.w;
    waypoints.push_back(wpose);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_thresh = 0.0;
    double fraction = arm_group_->computeCartesianPath(waypoints, eef_step,
                                            jump_thresh, trajectory);
    arm_group_->execute(trajectory);
}