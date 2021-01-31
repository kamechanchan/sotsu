#!/usr/bin/env python

import rospy

from mhand_gazebo_srvs.srv import GetStatus, GetStatusResponse
from mhand_gazebo_srvs.srv import Move, MoveResponse
from mhand_gazebo_srvs.srv import GetJointsValue, GetJointsValueResponse
from mhand_gazebo_srvs.srv import SetJointsValue, SetJointsValueResponse

from driver import MHandSim


class MHandSimListener(object):
    def __init__(self):
        self.robot_name = rospy.get_param("~robot_name", default="vs087")
        self.mhand_sim = MHandSim()
        self.mhand_sim.register_joints_()
        self.get_hand_status_srv = rospy.Service(
            '/mhand/get_hand_status', GetStatus, self.get_hand_status)
        self.get_hand_open_srv = rospy.Service(
            '/mhand/hand_open', Move, self.hand_open)
        self.get_hand_close_srv = rospy.Service(
            '/mhand/hand_close', Move, self.hand_close)
        self.get_hand_position_srv = rospy.Service(
            '/mhand/get_hand_position', GetJointsValue, self.get_hand_position)
        self.set_hand_position_srv = rospy.Service(
            '/mhand/set_hand_position', SetJointsValue, self.set_hand_position)
        rospy.loginfo("Initialize mhand_sim_listener_node")

    def get_hand_status(self, req):
        ret = GetStatusResponse()
        if self.mhand_sim.is_hand_open_():
            ret.status = 'open'
        elif self.mhand_sim.is_hand_close_():
            ret.status = 'close'
        else:
            ret.status = 'middle'

        return ret

    def hand_open(self, req):
        ret = MoveResponse()
        self.mhand_sim.open_hand_()
        ret.success = True
        return ret

    def hand_close(self, req):
        ret = MoveResponse()
        self.mhand_sim.close_hand_()
        ret.success = True
        return ret

    def get_hand_position(self, req):
        ret = GetJointsValueResponse()
        ret.type = "joint_position"
        joint_names, positions = self.mhand_sim.get_position_()

        for i in range(len(joint_names)):
            ret.joint_names.append(joint_names[i])
            ret.values.append(positions[i])

        return ret

    def set_hand_position(self, req):
        ret = SetJointsValueResponse()
        self.mhand_sim.set_position_(req.value)
        ret.success = True
        return ret


if __name__ == '__main__':
    rospy.init_node('mhand_sim_listener_node')
    mhand_sim_listener = MHandSimListener()
    rospy.spin()
