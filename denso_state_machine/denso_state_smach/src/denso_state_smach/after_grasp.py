#!/usr/bin/env python

import copy
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from smach import State
from denso_state_behavior import MovingBehavior
from denso_state_srvs.srv import GetFixedJointValues, GetFixedJointValuesRequest


class AfterGrasp(State):
    """define AfterGrasp state"""

    def __init__(
            self,
            z_offset=0.10,
            num_moving_generate_traj=0,
            num_depart_generate_traj=0):
        """
        Parameters
        ----------
        z_offset : float, default 0.10[m]
            z-axis offset value for approach/depart moving
        num_moving_generate_traj : int, default 0(planner default)
            number of generate candidate paths when moving planning
        num_depart_generate_traj : int, default 0(planner default)
            number of generate candidate paths when depart planning
        """
        State.__init__(self, outcomes=['success', 'failed'])
        if isinstance(num_moving_generate_traj, int):
            self.num_moving_generate_traj = num_moving_generate_traj
        else:
            raise TypeError(
                'num_moving_generate_traj must be a type of int')
        if isinstance(num_depart_generate_traj, int):
            self.num_depart_generate_traj = num_depart_generate_traj
        else:
            raise TypeError(
                'num_depart_generate_traj must be a type of int')
        self.z_offset = z_offset
        self.fixed_joint_values_g = rospy.ServiceProxy(
            '/fix_eef_joint_value/get_grasp_point', GetFixedJointValues)
        self.fixed_joint_values_a = rospy.ServiceProxy(
            '/fix_eef_joint_value/get_assemble_point', GetFixedJointValues)

    def execute(self, userdata):
        req = GetFixedJointValuesRequest()
        req.offset = self.z_offset
        res = self.fixed_joint_values_g.call(req)
        if not res.success:
            rospy.logerr('get grasp joint value failed')
            return 'failed'

        depart_moving_obj = MovingBehavior(
            joint_names=res.joint_names,
            target_joint_values=res.joint_values_offset,
            num_generate_traj=self.num_depart_generate_traj)

        res = self.fixed_joint_values_a.call(req)
        if not res.success:
            rospy.logerr('get assemble joint value failed')
            return 'failed'

        offset_moving_obj = MovingBehavior(
            joint_names=res.joint_names,
            target_joint_values=res.joint_values_offset,
            num_generate_traj=self.num_moving_generate_traj)

        ret = depart_moving_obj.execute_impl()
        if ret == 'failed':
            return 'failed'

        ret = offset_moving_obj.execute_impl()
        if ret == 'failed':
            return 'failed'

        return 'success'
