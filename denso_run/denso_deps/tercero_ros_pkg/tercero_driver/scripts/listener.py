#!/usr/bin/env python

import rospy

from tercero_srvs.srv import GetHandReady, GetHandReadyResponse
from tercero_srvs.srv import GetHandState, GetHandStateResponse
from tercero_srvs.srv import GetJointPosition, GetJointPositionResponse
from tercero_srvs.srv import GetTorque, GetTorqueResponse
from tercero_srvs.srv import Move, MoveResponse
from tercero_srvs.srv import SetTorque, SetTorqueResponse

from driver import Tercero


class TerceroListener(object):
    def __init__(self):
        self.check_chack_service_ = rospy.Service(
            '/tercero/chack_check', GetHandReady, self.chack_check)
        self.check_pusher_service_ = rospy.Service(
            '/tercero/pusher_check', GetHandReady, self.pusher_check)
        self.get_chack_state_service_ = rospy.Service(
            '/tercero/get_chack_state', GetHandState, self.get_chack_state)
        self.get_pusher_state_service_ = rospy.Service(
            '/tercero/get_pusher_state', GetHandState, self.get_pusher_state)
        self.get_chack_position_service_ = rospy.Service(
            '/tercero/get_chack_position', GetJointPosition, self.get_chack_position)
        self.get_pusher_position_service_ = rospy.Service(
            '/tercero/get_pusher_position', GetJointPosition, self.get_pusher_position)
        self.get_chack_torque_service_ = rospy.Service(
            '/tercero/get_chack_torque', GetTorque, self.get_chack_torque)
        self.get_pusher_torque_service_ = rospy.Service(
            '/tercero/get_pusher_torque', GetTorque, self.get_pusher_torque)
        self.chack_open_service_ = rospy.Service(
            '/tercero/chack_open', Move, self.chack_open)
        self.chack_close_service_ = rospy.Service(
            '/tercero/chack_close', Move, self.chack_close)
        self.pusher_on_service_ = rospy.Service(
            '/tercero/pusher_on', Move, self.pusher_on)
        self.pusher_off_service_ = rospy.Service(
            '/tercero/pusher_off', Move, self.pusher_off)
        self.set_chack_torque_service_ = rospy.Service(
            '/tercero/set_chack_torque', SetTorque, self.set_chack_torque)
        self.set_pusher_torque_service_ = rospy.Service(
            '/tercero/set_pusher_torque', SetTorque, self.set_pusher_torque)
        self.grasp_service_ = rospy.Service('/tercero/grasp', Move, self.grasp)
        self.release_service_ = rospy.Service(
            '/tercero/release', Move, self.release)
        self.tercero = Tercero()

    def chack_check(self, req):
        ret = GetHandReadyResponse()
        if (self.tercero.is_chack_ok()):
            ret.state = "Hand Chack is OK !!"
            ret.ready = True
        else:
            ret.state = "Hand Chack is Error !!"
            ret.ready = False

        return ret

    def pusher_check(self, req):
        ret = GetHandReadyResponse()
        if (self.tercero.is_pusher_ok()):
            ret.state = "Hand Pusher is OK !!"
            ret.ready = True
        else:
            ret.state = "Hand Pusher is Error !!"
            ret.ready = False

        return ret

    def get_chack_state(self, req):
        ret = GetHandStateResponse()
        ret.state = self.tercero.get_chack_state()
        return ret

    def get_pusher_state(self, req):
        ret = GetHandStateResponse()
        ret.state = self.tercero.get_pusher_state()
        return ret

    def get_chack_position(self, req):
        ret = GetJointPositionResponse()
        ret.position = self.tercero.get_chack_joint_position()
        return ret

    def get_pusher_position(self, req):
        ret = GetJointPositionResponse()
        ret.position = self.tercero.get_pusher_joint_position()
        return ret

    def get_chack_torque(self, req):
        ret = GetTorqueResponse()
        ret.torque = self.tercero.get_chack_torque()
        return ret

    def get_pusher_torque(self, req):
        ret = GetTorqueResponse()
        ret.torque = self.tercero.get_pusher_torque()
        return ret

    def chack_open(self, req):
        ret = MoveResponse()
        ret.success = self.tercero.open_chack()
        return ret

    def chack_close(self, req):
        ret = MoveResponse()
        ret.success = self.tercero.close_chack()
        return ret

    def pusher_on(self, req):
        ret = MoveResponse()
        ret.success = self.tercero.down_pusher()
        return ret

    def pusher_off(self, req):
        ret = MoveResponse()
        ret.success = self.tercero.up_pusher()
        return ret

    def set_chack_torque(self, req):
        ret = SetTorqueResponse()
        ret.success = self.tercero.set_chack_torque(req.torque)
        return ret

    def set_pusher_torque(self, req):
        ret = SetTorqueResponse()
        ret.success = self.tercero.set_pusher_torque(req.torque)
        return ret

    def grasp(self, req):
        ret = MoveResponse()
        if (not self.tercero.close_chack()):
            ret.success = False
            return ret
        else:
            ret.success = self.tercero.down_pusher()
            return ret

    def release(self, req):
        ret = MoveResponse()
        if (not self.tercero.up_pusher()):
            ret.success = False
            return ret
        else:
            ret.success = self.tercero.open_chack()
            return ret


if __name__ == '__main__':
    rospy.init_node('tercero_node')
    tercero_listener = TerceroListener()
    rospy.spin()
