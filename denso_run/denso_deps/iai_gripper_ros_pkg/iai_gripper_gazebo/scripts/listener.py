#!/usr/bin/env python

import rospy

from iai_gripper_gazebo_srvs.srv import GetStatus, GetStatusResponse
from iai_gripper_gazebo_srvs.srv import Move, MoveResponse
from iai_gripper_gazebo_srvs.srv import GetJointsValue, GetJointsValueResponse
from iai_gripper_gazebo_srvs.srv import SetJointsValue, SetJointsValueResponse

from driver import Iai_GripperSim


class Iai_GripperSimListener(object):
    def __init__(self):
        self.robot_name = rospy.get_param("~robot_name", default="vs087")
        self.iai_gripper_sim = Iai_GripperSim()
        self.iai_gripper_sim.register_joints_()
        self.get_hand_status_srv = rospy.Service(
            '/iai_gripper/get_hand_status', GetStatus, self.get_hand_status)
        self.get_hand_open_srv = rospy.Service(
            '/iai_gripper/hand_open', Move, self.hand_open)
        self.get_hand_close_srv = rospy.Service(
            '/iai_gripper/hand_close', Move, self.hand_close)
        self.get_hand_position_srv = rospy.Service(
            '/iai_gripper/get_hand_position',
            GetJointsValue,
            self.get_hand_position)
        self.set_hand_position_srv = rospy.Service(
            '/iai_gripper/set_hand_position',
            SetJointsValue,
            self.set_hand_position)
        rospy.loginfo("Initialize iai_gripper_sim_listener_node")

    def get_hand_status(self, req):
        ret = GetStatusResponse()
        if self.iai_gripper_sim.is_hand_open_():
            ret.status = 'open'
        elif self.iai_gripper_sim.is_hand_close_():
            ret.status = 'close'
        else:
            ret.status = 'middle'

        return ret

    def hand_open(self, req):
        ret = MoveResponse()
        self.iai_gripper_sim.open_hand_()
        ret.success = True
        return ret

    def hand_close(self, req):
        ret = MoveResponse()
        self.iai_gripper_sim.close_hand_()
        ret.success = True
        return ret

    def get_hand_position(self, req):
        ret = GetJointsValueResponse()
        ret.type = "joint_position"
        joint_names, positions = self.iai_gripper_sim.get_position_()

        for i in range(len(joint_names)):
            ret.joint_names.append(joint_names[i])
            ret.values.append(positions[i])

        return ret

    def set_hand_position(self, req):
        ret = SetJointsValueResponse()
        self.iai_gripper_sim.set_position_(req.value)
        ret.success = True
        return ret


if __name__ == '__main__':
    rospy.init_node('iai_gripper_sim_listener_node')
    iai_gripper_sim_listener = Iai_GripperSimListener()
    rospy.spin()
