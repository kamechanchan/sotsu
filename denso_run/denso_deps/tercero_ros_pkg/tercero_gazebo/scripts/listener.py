#!/usr/bin/env python

import rospy

from tercero_gazebo_srvs.srv import GetStatus, GetStatusResponse
from tercero_gazebo_srvs.srv import Move, MoveResponse
from tercero_gazebo_srvs.srv import GetJointsValue, GetJointsValueResponse
from tercero_gazebo_srvs.srv import SetJointsValue, SetJointsValueResponse

from driver import TerceroSim


class TerceroSimListener(object):
    def __init__(self):
        self.robot_name = rospy.get_param("~robot_name", default="vs087")
        self.tercero_sim = TerceroSim()
        self.tercero_sim.register_joints()
        self.get_chack_state_srv = rospy.Service(
            '/tercero/get_chack_state', GetStatus, self.get_chack_state)
        self.get_pusher_state_srv = rospy.Service(
            '/tercero/get_pusher_state', GetStatus, self.get_pusher_state)
        self.get_chack_open_srv = rospy.Service(
            '/tercero/chack_open', Move, self.chack_open)
        self.pusher_on_srv = rospy.Service(
            '/tercero/pusher_on', Move, self.pusher_on)
        self.chack_close_srv = rospy.Service(
            '/tercero/chack_close', Move, self.chack_close)
        self.pusher_off_srv = rospy.Service(
            '/tercero/pusher_off', Move, self.pusher_off)
        self.get_chack_position_srv = rospy.Service(
            '/tercero/get_chack_position', GetJointsValue, self.get_chack_position)
        self.get_pusher_position_srv = rospy.Service(
            '/tercero/get_pusher_position', GetJointsValue, self.get_pusher_position)
        self.set_chack_position_srv = rospy.Service(
            '/tercero/set_chack_position',
            SetJointsValue,
            self.set_chack_position)
        self.set_pusher_position_srv = rospy.Service(
            '/tercero/set_pusher_position',
            SetJointsValue,
            self.set_pusher_position)
        rospy.loginfo("Initialize tercero_sim_listener_node")

    def get_chack_state(self, req):
        ret = GetStatusResponse()
        if self.tercero_sim.is_chack_open():
            ret.status = 'open'
        elif self.tercero_sim.is_chack_close():
            ret.status = 'close'
        else:
            ret.status = 'middle'

        return ret

    def get_pusher_state(self, req):
        ret = GetStatusResponse()
        if self.tercero_sim.is_pusher_on():
            ret.status = 'on'
        elif self.tercero_sim.is_pusher_off():
            ret.status = 'off'
        else:
            ret.status = 'middle'

        return ret

    def chack_open(self, req):
        ret = MoveResponse()
        self.tercero_sim.open_chack()
        ret.success = True
        return ret

    def pusher_on(self, req):
        ret = MoveResponse()
        self.tercero_sim.on_pusher()
        ret.success = True
        return ret

    def chack_close(self, req):
        ret = MoveResponse()
        self.tercero_sim.close_chack()
        ret.success = True
        return ret

    def pusher_off(self, req):
        ret = MoveResponse()
        self.tercero_sim.off_pusher()
        ret.success = True
        return ret

    def get_chack_position(self, req):
        ret = GetJointsValueResponse()
        ret.type = "joint_position"
        joint_names, positions = self.tercero_sim.get_chack_position()

        for i in range(len(joint_names)):
            ret.joint_names.append(joint_names[i])
            ret.values.append(positions[i])

        return ret

    def get_pusher_position(self, req):
        ret = GetJointsValueResponse()
        ret.type = "joint_position"
        joint_names, positions = self.tercero_sim.get_pusher_position()

        for i in range(len(joint_names)):
            ret.joint_names.append(joint_names[i])
            ret.values.append(positions[i])

        return ret

    def set_chack_position(self, req):
        ret = SetJointsValueResponse()
        self.tercero_sim.set_chack_position(req.value)
        ret.success = True
        return ret

    def set_pusher_position(self, req):
        ret = SetJointsValueResponse()
        self.tercero_sim.set_pusher_position(req.value)
        ret.success = True
        return ret


if __name__ == '__main__':
    rospy.init_node('tercero_sim_listener_node')
    Tercero_sim_listener = TerceroSimListener()
    rospy.spin()
