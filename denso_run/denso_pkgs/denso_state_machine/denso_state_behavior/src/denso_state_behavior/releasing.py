#!/usr/bin/env python

import rospy
# from tercero_gazebo_srvs.srv import Move, MoveRequest
from tercero_srvs.srv import Move, MoveRequest
from .behavior import Behavior
from tercero_gazebo_srvs.srv import Move  # RealRobot
# from denso_gazebo_srvs.srv import Pose  # Simulator


class ReleasingBehavior(Behavior):
    """define releasing transition behavior"""

    def __init__(self):
        # self.release_srv = rospy.ServiceProxy('/tercero/hand_open', Move)
        self.release_srv = rospy.ServiceProxy('/tercero/chack_open', Move)

    def execute_impl(self):
        try:
            ret = self.release_srv()
        except rospy.ServiceException as e:
            rospy.logerr('release behavior service call failed:{}'.format(e))
        if ret.success:
            return 'success'
        else:
            return 'failed'
