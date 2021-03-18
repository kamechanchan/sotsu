#!/usr/bin/env python

import rospy
from smach import State


class FinalState(State):
    """define FinalState state"""

    def __init__(self):
        State.__init__(self, outcomes=['success', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Task done')
        return 'success'
