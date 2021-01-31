#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState


class MHandSim(object):
    def __init__(self):
        self.robot_name_ = rospy.get_param("~robot_name", default="vs087")
        self.robot_controller_ = "/" + self.robot_name_ + "/gripper_controller"
        self.hand_joints_state_ = JointTrajectoryControllerState()
        self.hand_joints_command_ = JointTrajectory()
        self.hand_joints_ = rospy.get_param(self.robot_controller_ + "/joints")
        self.joints_position_limits_min = {}
        self.joints_position_limits_max = {}
        self.hand_sub_ = rospy.Subscriber(
            self.robot_controller_ + "/state",
            JointTrajectoryControllerState,
            self.read_hand_callback_,
            queue_size=1)
        self.hand_pub_ = rospy.Publisher(
            self.robot_controller_ +
            "/command",
            JointTrajectory,
            queue_size=1)
        rospy.loginfo("Initialize MHandSim Class")

    def register_joints_(self):
        initial_point = JointTrajectoryPoint()
        self.hand_joints_command_.header.stamp = rospy.Time.now()
        self.hand_joints_command_.header.frame_id = "world"

        for i in range(len(self.hand_joints_)):
            self.hand_joints_command_.joint_names.append(self.hand_joints_[i])
            initial_point.positions.append(0)

        self.hand_joints_command_.points.append(initial_point)

        for i in range(len(self.hand_joints_command_.points)):
            self.hand_joints_command_.points[i].time_from_start = rospy.Duration.from_sec(
                0.01)

        for i in range(len(self.hand_joints_)):
            self.joints_position_limits_min[self.hand_joints_[i]] = rospy.get_param(
                self.robot_name_ + "/joints_limit/gripper/" + self.hand_joints_[i] + "/min_position")
            self.joints_position_limits_max[self.hand_joints_[i]] = rospy.get_param(
                self.robot_name_ + "/joints_limit/gripper/" + self.hand_joints_[i] + "/max_position")

    def read_hand_callback_(self, msg):
        self.hand_joints_state_ = msg

    def is_hand_open_(self):
        actual_state = {}

        for i in range(len(self.hand_joints_state_.joint_names)):
            actual_state[self.hand_joints_state_.joint_names[i]
                         ] = self.hand_joints_state_.actual.positions[i]

        open_flag = False

        for i in range(len(self.hand_joints_state_.joint_names)):
            div = abs((self.joints_position_limits_min[self.hand_joints_state_.joint_names[i]
                                                       ] - actual_state[self.hand_joints_state_.joint_names[i]]))
            if (div < 0.01):
                open_flag = True
                continue
            else:
                open_flag = False
                break

        return open_flag

    def is_hand_close_(self):
        actual_state = {}

        for i in range(len(self.hand_joints_state_.joint_names)):
            actual_state[self.hand_joints_state_.joint_names[i]
                         ] = self.hand_joints_state_.actual.positions[i]

        close_flag = False

        for i in range(len(self.hand_joints_state_.joint_names)):
            div = abs((self.joints_position_limits_max[self.hand_joints_state_.joint_names[i]
                                                       ] - actual_state[self.hand_joints_state_.joint_names[i]]))
            if (div < 0.01):
                close_flag = True
                continue
            else:
                close_flag = False
                break

        return close_flag

    def open_hand_(self):
        for i in range(len(self.hand_joints_command_.joint_names)):
            self.hand_joints_command_.points[0].positions[
                i] = self.joints_position_limits_min[self.hand_joints_command_.joint_names[i]]

        self.hand_pub_.publish(self.hand_joints_command_)

    def close_hand_(self):
        for i in range(len(self.hand_joints_command_.joint_names)):
            self.hand_joints_command_.points[0].positions[
                i] = self.joints_position_limits_max[self.hand_joints_command_.joint_names[i]]

        self.hand_pub_.publish(self.hand_joints_command_)

    def get_position_(self):
        return self.hand_joints_state_.joint_names, self.hand_joints_state_.actual.positions

    def set_position_(self, value):
        min = self.joints_position_limits_min["finger_A_joint"]
        max = self.joints_position_limits_max["finger_A_joint"]
        if (value < min):
            value = min
            rospy.logwarn("joints limit from " + str(min) +
                          " to " + str(max) + " !!")
        if (value > max):
            value = max
            rospy.logwarn("joints limit from " + str(min) +
                          " to " + str(max) + " !!")

        for i in range(len(self.hand_joints_command_.joint_names)):
            self.hand_joints_command_.points[0].positions[i] = value

        self.hand_pub_.publish(self.hand_joints_command_)
