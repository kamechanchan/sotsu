#!/usr/bin/env python

import rospy
# from tercero_gazebo_srvs.srv import Move, MoveRequest
from tercero_srvs.srv import Move, MoveRequest

from .behavior import Behavior


class GraspingBehavior(Behavior):
    """define grasping transition behavior"""

    def __init__(self):
        # self.grasping_srv = rospy.ServiceProxy('/tercero/hand_close', Move)
        self.grasping_srv = rospy.ServiceProxy('/tercero/chack_close', Move)

    def execute_impl(self):
        try:
            ret = self.grasping_srv()
        except rospy.ServiceException as e:
            rospy.logerr('grasping behavior service call failed:{}'.format(e))
        if ret.success:
            return 'success'
        else:
            return 'failed'
