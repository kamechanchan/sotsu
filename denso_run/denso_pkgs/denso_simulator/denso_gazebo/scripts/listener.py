#!/usr/bin/env python3
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '/home/tsuchidashinya/ros_package/denso_ws/src/denso_run/denso_pkgs/denso_simulator/denso_gazebo/scripts'))
sys.path.append(os.path.join(os.path.dirname(__file__), '/home/tsuchidashinya/ros_package/denso_ws/devel/.private/denso_gazebo/lib/denso_gazebo/'))
import rospy

from denso_gazebo_srvs.srv import Pose, PoseResponse
from denso_gazebo_srvs.srv import GetJointsValue, GetJointsValueResponse
from denso_gazebo_srvs.srv import SetJointsValue, SetJointsValueResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState

#from driver import DensoRobotArmSim

class DensoRobotArmSim(object):
    def __init__(self):
        self.robot_name_ = rospy.get_param("~robot_name", default="vs087")
        self.robot_controller_ = "/" + self.robot_name_ + "/arm_controller"
        self.arm_joints_state_ = JointTrajectoryControllerState()
        self.arm_joints_command_ = JointTrajectory()
        self.arm_joints_ = rospy.get_param(self.robot_controller_ + "/joints", default="/vs087/arm_controller/jonints")
        self.arm_joint_pos_min_ = list()
        self.arm_joint_pos_max_ = list()
        self.arm_sub_ = rospy.Subscriber(
            self.robot_controller_ + "/state",
            JointTrajectoryControllerState,
            self.read_arm_callback_,
            queue_size=1)
        self.arm_pub_ = rospy.Publisher(
            self.robot_controller_ +
            "/command",
            JointTrajectory,
            queue_size=1)
        rospy.loginfo("Initialize DensoRobotArmSim Class")

    def register_joints_(self):
        initial_point = JointTrajectoryPoint()
        self.arm_joints_command_.header.stamp = rospy.Time.now()
        self.arm_joints_command_.header.frame_id = "world"

        for i in range(len(self.arm_joints_)):
            self.arm_joints_command_.joint_names.append(self.arm_joints_[i])
            initial_point.positions.append(0)
            pos_min = rospy.get_param(
                "/" +
                self.robot_name_ +
                "/joints_limit/arm/" +
                self.arm_joints_[i] +
                "/min_position", default="")
            self.arm_joint_pos_min_.append(pos_min)
            pos_max = rospy.get_param(
                "/" +
                self.robot_name_ +
                "/joints_limit/arm/" +
                self.arm_joints_[i] +
                "/max_position", default="")
            self.arm_joint_pos_max_.append(pos_max)

        self.arm_joints_command_.points.append(initial_point)

        for i in range(len(self.arm_joints_command_.points)):
            self.arm_joints_command_.points[i].time_from_start = rospy.Duration.from_sec(
                0.01)

    def read_arm_callback_(self, msg):
        self.arm_joints_state_ = msg

    def set_default_position_(self):
        default_pos = [-0.0001745329, -0.35203291,
                       2.27294228, 0.0003490659, 1.22173, 3.14159]

        for i in range(len(self.arm_joints_command_.joint_names)):
            self.arm_joints_command_.points[0].positions[i] = default_pos[i]

        self.arm_pub_.publish(self.arm_joints_command_)

    def set_straight_position_(self):
        for i in range(len(self.arm_joints_command_.joint_names)):
            self.arm_joints_command_.points[0].positions[i] = 0

        self.arm_pub_.publish(self.arm_joints_command_)

    def get_position_(self):
        return self.arm_joints_state_.joint_names, self.arm_joints_state_.actual.positions

    def set_position_(self, values):
        if (len(values) != len(self.arm_joints_command_.joint_names)):
            rospy.logerr(
                "The number of require positions and joints do not match !!")
            return False

        for i in range(len(self.arm_joints_command_.joint_names)):
            if (values[i] < self.arm_joint_pos_min_[i]):
                self.arm_joints_command_.points[0].positions[i] = self.arm_joint_pos_min_[
                    i]
                rospy.logwarn(
                    self.arm_joints_command_.joint_names[i] +
                    ": The lower limit is limited to " +
                    str(
                        self.arm_joint_pos_min_[i]))
                rospy.logwarn("Set " +
                              self.arm_joints_command_.joint_names[i] +
                              " to " +
                              str(self.arm_joint_pos_min_[i]))
            elif (self.arm_joint_pos_max_[i] < values[i]):
                self.arm_joints_command_.points[0].positions[i] = self.arm_joint_pos_max_[
                    i]
                rospy.logwarn(
                    self.arm_joints_command_.joint_names[i] +
                    ": The upper limit is limited to " +
                    str(
                        self.arm_joint_pos_max_[i]))
                rospy.logwarn("Set " +
                              self.arm_joints_command_.joint_names[i] +
                              " to " +
                              str(self.arm_joint_pos_max_[i]))
            else:
                self.arm_joints_command_.points[0].positions[i] = values[i]

        self.arm_pub_.publish(self.arm_joints_command_)

        return True

class DensoRobotArmSimListener(object):
    def __init__(self):
        self.robot_name = rospy.get_param("~robot_name", default="vs087")
        self.arm_sim = DensoRobotArmSim()
        self.arm_sim.register_joints_()
        self.set_arm_default_srv = rospy.Service(
            self.robot_name + "/set_arm_default", Pose, self.set_arm_default)
        self.set_arm_straight_srv = rospy.Service(
            self.robot_name + "/set_arm_straight", Pose, self.set_arm_straight)
        self.get_arm_position_srv = rospy.Service(
            self.robot_name +
            "/get_arm_position",
            GetJointsValue,
            self.get_arm_position)
        self.set_arm_position_srv = rospy.Service(
            self.robot_name +
            "/set_arm_position",
            SetJointsValue,
            self.set_arm_position)
        rospy.loginfo("Initialize denso_robot_arm_sim_listerner_node")

    def set_arm_default(self, req):
        ret = PoseResponse()
        self.arm_sim.set_default_position_()
        ret.success = True
        return ret

    def set_arm_straight(self, req):
        ret = PoseResponse()
        self.arm_sim.set_straight_position_()
        ret.success = True
        return ret

    def get_arm_position(self, req):
        ret = GetJointsValueResponse()
        ret.type = "joint_position"
        joint_names, positions = self.arm_sim.get_position_()

        for i in range(len(joint_names)):
            ret.joint_names.append(joint_names[i])
            ret.values.append(positions[i])

        return ret

    def set_arm_position(self, req):
        ret = SetJointsValueResponse()
        ret.success = self.arm_sim.set_position_(req.values)
        return ret


if __name__ == '__main__':
    rospy.init_node('denso_robot_arm_sim_listener_node')
    denso_robot_arm_sim_listener = DensoRobotArmSimListener()
    rospy.spin()
