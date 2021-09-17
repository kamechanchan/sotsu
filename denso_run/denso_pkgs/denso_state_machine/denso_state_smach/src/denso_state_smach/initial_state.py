#!/usr/bin/env python

import copy
from geometry_msgs.msg import PoseStamped
from smach import State
from denso_state_behavior import MovingBehavior
from denso_state_behavior import ReleasingBehavior
from denso_state_srvs.srv import GetFixedJointValues, GetFixedJointValuesRequest
import rospy


class InitialState(State):
    """define InitialState state"""

    def __init__(
            self,
            z_offset=0.10,
            num_moving_generate_traj=0,
            num_approach_generate_traj=0):
        """
        Parameters
        ----------
        z_offset : float, default 0.10[m]
            z-axis offset value for approach/depart moving
        num_moving_generate_traj : int, default 0(planner default)
            number of generate candidate paths when moving planning
        num_approach_generate_traj : int, default 0(planner default)
            number of generate candidate paths when approach planning
        """
        State.__init__(self, outcomes=['success', 'failed'])
        if isinstance(num_moving_generate_traj, int):
            self.num_moving_generate_traj = num_moving_generate_traj
        else:
            raise TypeError(
                'num_moving_generate_traj must be a type of int')
        if isinstance(num_approach_generate_traj, int):
            self.num_approach_generate_traj = num_approach_generate_traj
        else:
            raise TypeError(
                'num_approach_generate_traj must be a type of int')
        self.z_offset = z_offset
        self.fixed_joint_values = rospy.ServiceProxy(
            '/fix_eef_joint_value/get_grasp_point', GetFixedJointValues)

    def execute(self, userdata):
        req = GetFixedJointValuesRequest()
        req.offset = self.z_offset
        res = self.fixed_joint_values.call(req)
        if not res.success:
            rospy.logerr('get grasp joint value failed')
            return 'failed'

        init_moving_obj = MovingBehavior(
            target_pose=self._get_initial_pose(),
            num_generate_traj=1)
        moving_obj = MovingBehavior(
            joint_names=res.joint_names,
            target_joint_values=res.joint_values,
            num_generate_traj=self.num_moving_generate_traj)
        offset_moving_obj = MovingBehavior(
            joint_names=res.joint_names,
            target_joint_values=res.joint_values_offset,
            num_generate_traj=self.num_approach_generate_traj)
        releasing_obj = ReleasingBehavior()

        ret = init_moving_obj.execute_impl()
        if ret == 'failed':
            return 'failed'

        ret = releasing_obj.execute_impl()
        if ret == 'failed':
            return 'failed'

        ret = offset_moving_obj.execute_impl()
        if ret == 'failed':
            return 'failed'

        ret = moving_obj.execute_impl()
        if ret == 'failed':
            return 'failed'

        return 'success'

    def _get_initial_pose(self):
        initial_pose = PoseStamped()
        initial_pose.pose.orientation.w = 0.0
        initial_pose.pose.orientation.x = 0.707
        initial_pose.pose.orientation.y = 0.707
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.position.x = 0.28727
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.position.z = 0.604027

        return initial_pose
