#!/usr/bin/env python

import rospy
from iai_gripper_srvs.srv import Servo, ServoRequest
from iai_gripper_srvs.srv import Pause, PauseRequest
from iai_gripper_srvs.srv import Reset, ResetRequest

from time import sleep

if __name__ == '__main__':
    rospy.init_node('iai_gripper_activate')
    rospy.loginfo('waiting for iai_gripper services are activated.')
    # rospy.wait_for_service('/iai_gripper/servo')
    # rospy.wait_for_service('/iai_gripper/pause')
    rospy.wait_for_service('/iai_gripper/reset')

    sleep(10)

    try:
        # servo_service = rospy.ServiceProxy('/iai_gripper/servo', Servo)
        # req = ServoRequest()
        # req.servo_on = True
        # ret = servo_service(req)
        # pause_service = rospy.ServiceProxy('/iai_gripper/pause', Pause)
        # req = PauseRequest()
        # req.pause = False
        # ret = pause_service(req)
        reset_service = rospy.ServiceProxy('/iai_gripper/reset', Reset)
        req = ResetRequest()
        ret = reset_service(req)
        rospy.loginfo('iai_gripper activation sequence is completed.')
    except rospy.ServiceException as e:
        rospy.logerr('iai_gripper activation failed:{}'.format(e))

