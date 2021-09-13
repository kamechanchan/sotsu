#!/usr/bin/env python

import copy
import rospy
from geometry_msgs.msg import PoseStamped
# from tercero_gazebo_srvs.srv import Move
from tercero_srvs.srv import Move
from denso_state_srvs.srv import GetFixedJointValues, GetFixedJointValuesRequest
from .behavior import Behavior
from .moving import MovingBehavior


class AssemblingBehavior(Behavior):
    """define Assembling transition behavior"""

    def __init__(self, num_generate_traj=0, z_offset=0.10):
        """
        Parameters
        ----------
        num_generate_traj : int, default 0(planner default)
            number of generate candidate paths when planning
        z_offset : float, default 0.10[m]
            z-axis offset value for approach/depart moving
        """

        if isinstance(num_generate_traj, int):
            self.num_generate_traj = num_generate_traj
        else:
            raise TypeError('num_generate_traj must be a type of int')

        if isinstance(z_offset, float):
            self.z_offset = z_offset
        else:
            raise TypeError('z_offset must be a type of float')

        # self.release_srv = rospy.ServiceProxy('/tercero/hand_open', Move)
        self.release_srv = rospy.ServiceProxy('/tercero/chack_open', Move)
        self.get_joint_value_srv = rospy.ServiceProxy(
            '/fix_eef_joint_value/get_assemble_point', GetFixedJointValues)

    def execute_impl(self):
        req = GetFixedJointValuesRequest()
        req.offset = self.z_offset
        ret = self.get_joint_value_srv(req)
        if not ret.success:
            rospy.logerr('get assemble joint value failed')
            return 'failed'

        approach_moving_obj = MovingBehavior(
            joint_names=ret.joint_names,
            target_joint_values=ret.joint_values,
            num_generate_traj=self.num_generate_traj)

        depart_moving_obj = MovingBehavior(
            joint_names=ret.joint_names,
            target_joint_values=ret.joint_values_offset,
            num_generate_traj=self.num_generate_traj)

        ret = approach_moving_obj.execute_impl()
        if not ret == 'success':
            rospy.logerr('approach moving failed')
            return 'failed'

        try:
            ret = self.release_srv()
        except rospy.ServiceException as e:
            rospy.logerr('release behavior service call failed:{}'.format(e))

        if not ret.success:
            rospy.logerr('releasing failed')
            return 'failed'

        ret = depart_moving_obj.execute_impl()
        if not ret == 'success':
            rospy.logerr('depart moving failed')
            return 'failed'

        return 'success'
