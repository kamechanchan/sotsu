#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped
from smach import State
from denso_state_behavior import AssemblingBehavior
from third_2019.srv import GetFixedTarget, GetFixedTargetRequest


class BeforeAssemble(State):
    """define BeforeAssemble state"""

    def __init__(self, num_approach_generate_traj=0, z_offset=0.10):
        """
        Parameters
        ----------
        num_approach_generate_traj : int, default 0(planner default)
            number of generate candidate paths when approach planning
        z_offset : float, default 0.10[m]
            z-axis offset value for approach/depart moving
        """
        State.__init__(self, outcomes=['success', 'failed'])
        if isinstance(num_approach_generate_traj, int):
            self.num_approach_generate_traj = num_approach_generate_traj
        else:
            raise TypeError(
                'num_approach_generate_traj must be a type of int')

        if isinstance(z_offset, float):
            self.z_offset = z_offset
        else:
            raise TypeError('z_offset must be a type of float')

    def execute(self, userdata):
        assemble_obj = AssemblingBehavior(
            num_generate_traj=self.num_approach_generate_traj,
            z_offset=self.z_offset)

        ret = assemble_obj.execute_impl()
        if ret == 'failed':
            return 'failed'

        return 'success'
