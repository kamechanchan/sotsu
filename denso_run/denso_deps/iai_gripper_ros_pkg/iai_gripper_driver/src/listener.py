#!/usr/bin/env python

import rospy

from iai_gripper_srvs.srv import OriginReturn, OriginReturnResponse
from iai_gripper_srvs.srv import Move, MoveResponse
from iai_gripper_srvs.srv import Pause, PauseResponse
from iai_gripper_srvs.srv import Servo, ServoResponse
from iai_gripper_srvs.srv import GetPosition, GetPositionResponse
from iai_gripper_srvs.srv import Reset, ResetResponse

from driver import IAIGripper

class IAIGripperListener(object):
    def __init__(self):
        self.iai_gripper = IAIGripper()
        self.move_gripper_service = rospy.Service('/iai_gripper/move_gripper', Move, self.move_gripper)
        self.origin_return_service = rospy.Service('/iai_gripper/origin_return', OriginReturn, self.origin_return)
        self.pause_service = rospy.Service('/iai_gripper/pause', Pause, self.pause)
        self.servo_service = rospy.Service('/iai_gripper/servo', Servo, self.servo)
        self.get_position_service = rospy.Service('/iai_gripper/get_position', GetPosition, self.get_position)
        self.reset_service = rospy.Service('/iai_gripper/reset', Reset, self.reset)
        print("Ready to IAIgripper services")

    def move_gripper(self, req):
        ret = MoveResponse()
        self.iai_gripper.move(req.position)
        ret.success = True
        return ret

    def origin_return(self, req):
        ret = OriginReturnResponse()
        self.iai_gripper.return_origin()
        ret.success = True
        return ret

    def pause(self, req):
        ret = PauseResponse()
        self.iai_gripper.pause(req.pause)
        ret.success = True
        return ret

    def servo(self, req):
        ret = ServoResponse()
        self.iai_gripper.servo(req.servo_on)
        ret.success = True
        return ret

    def get_position(self, req):
        ret = GetPositionResponse()
        ret.position = self.iai_gripper.get_position()
        ret.success = True
        return ret

    def reset(self, req):
        ret = ResetResponse()
        ret.success = self.iai_gripper.reset()
        return ret

if __name__ == '__main__':
    rospy.init_node('iai_gripper_listener')
    iai_gripper_listener = IAIGripperListener()
    rospy.spin()
