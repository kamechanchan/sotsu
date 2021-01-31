#!/usr/bin/env python

import rospy

from mhand_srvs.srv import GetStatus, GetStatusResponse
from mhand_srvs.srv import Move, MoveResponse
from mhand_srvs.srv import GetTorque, GetTorqueResponse
from mhand_srvs.srv import SetTorque, SetTorqueResponse

from driver import MHand

class MHandListener(object):
    def __init__(self):
        self.mhand = MHand()
        self.get_hand_status_service = rospy.Service('/mhand/get_hand_status', GetStatus, self.get_hand_status)
        self.hand_open_service = rospy.Service('/mhand/hand_open', Move, self.hand_open)
        self.hand_close_service = rospy.Service('/mhand/hand_close', Move, self.hand_close)
        self.get_torque_service = rospy.Service('/mhand/get_torque', GetTorque, self.get_torque)
        self.set_torque_service = rospy.Service('/mhand/set_torque', SetTorque, self.set_torque)
        print("Ready to MHand services")

    def get_hand_status(self, req):
        ret = GetStatusResponse()
        if self.mhand.is_hand_open():
            ret.status = 'open'
        elif self.mhand.is_hand_close():
            ret.status = 'close'
        else:
            ret.status = 'unknown'
        return ret

    def hand_open(self, req):
        ret = MoveResponse()
        self.mhand.open_hand()
        ret.success = True
        return ret

    def hand_close(self, req):
        ret = MoveResponse()
        self.mhand.close_hand()
        ret.success = True
        return ret

    def get_torque(self, req):
        ret = GetTorqueResponse()
        ret.torque = self.mhand.get_torque()
        return ret

    def set_torque(self, req):
        ret = SetTorqueResponse()
        self.mhand.set_torque(req.torque)
        ret.success = True
        return ret


if __name__ == '__main__':
    rospy.init_node('mhand_listener')
    mhand_listener = MHandListener()
    rospy.spin()
