#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped
from smach import State
from denso_state_behavior import MovingBehavior


class AfterAssemble(State):
    """define AfterAssemble state"""

    def __init__(self, num_moving_generate_traj=0):
        """
        Parameters
        ----------
        num_moving_generate_traj : int, default 0(planner default)
            number of generate candidate paths when moving planning
        """
        State.__init__(self, outcomes=['success', 'failed'])
        if isinstance(num_moving_generate_traj, int):
            self.num_moving_generate_traj = num_moving_generate_traj
        else:
            raise TypeError(
                'num_moving_generate_traj must be a type of int')

    def execute(self, userdata):
        initial_pose_moving_obj = MovingBehavior(
            joint_names=self._get_joint_names(),
            target_joint_values=self._get_initial_joint_value(),
            num_generate_traj=self.num_moving_generate_traj)

        ret = initial_pose_moving_obj.execute_impl()
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
        initial_pose.pose.position.y = 4.20769e-06
        initial_pose.pose.position.z = 0.604027

    def _get_initial_joint_value(self):
        return [-0.0001, -0.352033, 2.2729, 0.0003, 1.2217, 1.570795]

    def _get_joint_names(self):
        return [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6"]
