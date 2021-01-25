#!/usr/bin/env python

from smach import State
from denso_state_behavior import GraspingBehavior


class BeforeGrasp(State):
    """define BeforeGrasp state"""

    def __init__(self):
        State.__init__(self, outcomes=['success', 'failed'])

    def execute(self, userdata):
        grasping_obj = GraspingBehavior()

        ret = grasping_obj.execute_impl()
        if ret == 'failed':
            return 'failed'

        return 'success'
