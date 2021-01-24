#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState


class TerceroSim(object):
    def __init__(self):
        self.robot_name_ = rospy.get_param("~robot_name", default="vs087")
        self.robot_controller_ = "/" + self.robot_name_ + "/gripper_controller"
        self.hand_joints_state_ = JointTrajectoryControllerState()
        self.hand_joints_command_ = JointTrajectory()
        self.hand_joints_ = rospy.get_param(self.robot_controller_ + "/joints")
        self.hand_frame_ = "world"
        self.tolerance = 0.001
        self.chack_name_ = ["finger_R_joint", "finger_L_joint"]
        self.pusher_name_ = "finger_3rd_joint"
        self.joints_position_limits_min = {}
        self.joints_position_limits_max = {}
        self.hand_sub_ = rospy.Subscriber(
            self.robot_controller_ + "/state",
            JointTrajectoryControllerState,
            self.read_hand_callback,
            queue_size=1)
        self.hand_pub_ = rospy.Publisher(
            self.robot_controller_ +
            "/command",
            JointTrajectory,
            queue_size=1)
        rospy.loginfo("Initialize TerceroSim Class")

    def register_joints(self):
        initial_point = JointTrajectoryPoint()
        self.hand_joints_command_.header.stamp = rospy.Time.now()
        self.hand_joints_command_.header.frame_id = self.hand_frame_

        for i in range(len(self.hand_joints_)):
            self.hand_joints_command_.joint_names.append(self.hand_joints_[i])
            initial_point.positions.append(0)

        self.hand_joints_command_.points.append(initial_point)

        for i in range(len(self.hand_joints_command_.points)):
            self.hand_joints_command_.points[i].time_from_start = rospy.Duration.from_sec(
                self.tolerance)

        for i in range(len(self.hand_joints_)):
            self.joints_position_limits_min[self.hand_joints_[i]] = rospy.get_param(
                self.robot_name_ + "/joints_limit/gripper/" + self.hand_joints_[i] + "/min_position")
            self.joints_position_limits_max[self.hand_joints_[i]] = rospy.get_param(
                self.robot_name_ + "/joints_limit/gripper/" + self.hand_joints_[i] + "/max_position")

    def read_hand_callback(self, msg):
        self.hand_joints_state_ = msg

    def is_chack_open(self):
        actual_state = {}

        for i in range(len(self.hand_joints_state_.joint_names)):
            actual_state[self.hand_joints_state_.joint_names[i]
                         ] = self.hand_joints_state_.actual.positions[i]

        open_flag = False

        for i in range(len(self.hand_joints_state_.joint_names)):
            if (self.hand_joints_state_.joint_names[i] in self.chack_name_):
                div = abs(self.joints_position_limits_min[self.hand_joints_state_.joint_names[i]
                                                          ] - actual_state[self.hand_joints_state_.joint_names[i]])
                if (div < self.tolerance):
                    open_flag = True
                    continue
                else:
                    open_flag = False
                    break
            else:
                continue

        return open_flag

    def is_chack_close(self):
        actual_state = {}

        for i in range(len(self.hand_joints_state_.joint_names)):
            actual_state[self.hand_joints_state_.joint_names[i]
                         ] = self.hand_joints_state_.actual.positions[i]

        close_flag = False

        for i in range(len(self.hand_joints_state_.joint_names)):
            if (self.hand_joints_state_.joint_names[i] in self.chack_name_):
                div = abs(self.joints_position_limits_max[self.hand_joints_state_.joint_names[i]
                                                          ] - actual_state[self.hand_joints_state_.joint_names[i]])
                if (div < self.tolerance):
                    close_flag = True
                    continue
                else:
                    close_flag = False
                    break
            else:
                continue

        return close_flag

    def is_pusher_on(self):
        actual_state = {}

        for i in range(len(self.hand_joints_state_.joint_names)):
            actual_state[self.hand_joints_state_.joint_names[i]
                         ] = self.hand_joints_state_.actual.positions[i]

        on_flag = False

        for i in range(len(self.hand_joints_state_.joint_names)):
            if (self.hand_joints_state_.joint_names[i] == self.pusher_name_):
                div = abs(self.joints_position_limits_max[self.hand_joints_state_.joint_names[i]
                                                          ] - actual_state[self.hand_joints_state_.joint_names[i]])
                if (div < self.tolerance):
                    on_flag = True
                    continue
                else:
                    on_flag = False
                    break
            else:
                continue

        return on_flag

    def is_pusher_off(self):
        actual_state = {}

        for i in range(len(self.hand_joints_state_.joint_names)):
            actual_state[self.hand_joints_state_.joint_names[i]
                         ] = self.hand_joints_state_.actual.positions[i]

        off_flag = False

        for i in range(len(self.hand_joints_state_.joint_names)):
            if (self.hand_joints_state_.joint_names[i] == self.pusher_name_):
                div = abs(self.joints_position_limits_min[self.hand_joints_state_.joint_names[i]
                                                          ] - actual_state[self.hand_joints_state_.joint_names[i]])
                if (div < self.tolerance):
                    off_flag = True
                    continue
                else:
                    off_flag = False
                    break
            else:
                continue

        return off_flag

    def open_chack(self):
        for i in range(len(self.hand_joints_command_.joint_names)):
            if (self.hand_joints_command_.joint_names[i] in self.chack_name_):
                self.hand_joints_command_.points[0].positions[
                    i] = self.joints_position_limits_min[self.hand_joints_command_.joint_names[i]]
            else:
                self.hand_joints_command_.points[0].positions[i] = self.hand_joints_state_.actual.positions[i]

        self.hand_pub_.publish(self.hand_joints_command_)

    def close_chack(self):
        for i in range(len(self.hand_joints_command_.joint_names)):
            if (self.hand_joints_command_.joint_names[i] in self.chack_name_):
                self.hand_joints_command_.points[0].positions[
                    i] = self.joints_position_limits_max[self.hand_joints_command_.joint_names[i]]
            else:
                self.hand_joints_command_.points[0].positions[i] = self.hand_joints_state_.actual.positions[i]

        self.hand_pub_.publish(self.hand_joints_command_)

    def on_pusher(self):
        for i in range(len(self.hand_joints_command_.joint_names)):
            if (self.hand_joints_command_.joint_names[i] == self.pusher_name_):
                self.hand_joints_command_.points[0].positions[
                    i] = self.joints_position_limits_max[self.hand_joints_command_.joint_names[i]]
            else:
                self.hand_joints_command_.points[0].positions[i] = self.hand_joints_state_.actual.positions[i]

        self.hand_pub_.publish(self.hand_joints_command_)

    def off_pusher(self):
        for i in range(len(self.hand_joints_command_.joint_names)):
            if (self.hand_joints_command_.joint_names[i] == self.pusher_name_):
                self.hand_joints_command_.points[0].positions[
                    i] = self.joints_position_limits_min[self.hand_joints_command_.joint_names[i]]
            else:
                self.hand_joints_command_.points[0].positions[i] = self.hand_joints_state_.actual.positions[i]

        self.hand_pub_.publish(self.hand_joints_command_)

    def get_chack_position(self):
        chack_joint_name = []
        chack_joint_position = []

        for i in range(len(self.hand_joints_command_.joint_names)):
            if (self.hand_joints_command_.joint_names[i] in self.chack_name_):
                chack_joint_name.append(
                    self.hand_joints_command_.joint_names[i])
                chack_joint_position.append(
                    self.hand_joints_state_.actual.positions[i])
            else:
                continue

        return chack_joint_name, chack_joint_position

    def get_pusher_position(self):
        pusher_joint_name = []
        pusher_joint_position = []

        for i in range(len(self.hand_joints_command_.joint_names)):
            if (self.hand_joints_command_.joint_names[i] == self.pusher_name_):
                pusher_joint_name.append(
                    self.hand_joints_command_.joint_names[i])
                pusher_joint_position.append(
                    self.hand_joints_state_.actual.positions[i])
            else:
                continue

        return pusher_joint_name, pusher_joint_position

    def set_chack_position(self, value):
        min = self.joints_position_limits_min[self.chack_name_[0]]
        max = self.joints_position_limits_max[self.chack_name_[0]]
        if (value < min):
            value = min
            rospy.logwarn(
                "joints limit from " +
                str(min) +
                " to " +
                str(max) +
                " !!")
        if (value > max):
            value = max
            rospy.logwarn(
                "joints limit from " +
                str(min) +
                " to " +
                str(max) +
                " !!")

        for i in range(len(self.hand_joints_command_.joint_names)):
            if (self.hand_joints_command_.joint_names[i]
                    != self.pusher_name_):
                self.hand_joints_command_.points[0].positions[i] = value
            else:
                self.hand_joints_command_.points[0].positions[i] = self.hand_joints_state_.actual.positions[i]
                continue

        self.hand_pub_.publish(self.hand_joints_command_)

    def set_pusher_position(self, value):
        min = self.joints_position_limits_min[self.pusher_name_]
        max = self.joints_position_limits_max[self.pusher_name_]
        if (value < min):
            value = min
            rospy.logwarn(
                "joints limit from " +
                str(min) +
                " to " +
                str(max) +
                " !!")
        if (value > max):
            value = max
            rospy.logwarn(
                "joints limit from " +
                str(min) +
                " to " +
                str(max) +
                " !!")

        for i in range(len(self.hand_joints_command_.joint_names)):
            if (self.hand_joints_command_.joint_names[i]
                    == self.pusher_name_):
                self.hand_joints_command_.points[0].positions[i] = value
            else:
                self.hand_joints_command_.points[0].positions[i] = self.hand_joints_state_.actual.positions[i]
                continue

        self.hand_pub_.publish(self.hand_joints_command_)
