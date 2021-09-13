#!/usr/bin/env python3
import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), '/home/ericlabshinya/ros_package/denso_ws/src/denso_run/denso_pkgs/denso_simulator/denso_gazebo/scripts'))
sys.path.append(os.path.join(os.path.dirname(__file__), '/home/ericlabshinya/ros_package/denso_ws/devel/.private/denso_gazebo/lib/denso_gazebo/'))
sys.path.append(os.path.join(os.path.dirname(__file__), '/home/ericlabshinya/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/trainer/options'))
import rospy
from test_options import TestOptions
from denso_gazebo_srvs.srv import Pose, PoseResponse
from denso_gazebo_srvs.srv import GetJointsValue, GetJointsValueResponse
from denso_gazebo_srvs.srv import SetJointsValue, SetJointsValueResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState

#from driver import DensoRobotArmSim


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
PointNet_Pose