#!/usr/bin/env python

import rospy

from smach import State
from denso_state_behavior import MeasurementBehavior
from denso_state_msgs.msg import WorkPoint
from denso_state_srvs.srv import SetWorkPoint, SetWorkPointRequest


class Measurement(State):
    """define Measurement state"""

    def __init__(self, object_name, assemble_point_name, relative_workpoint):
        """
        Parameters
        ----------
        object_name : string
            grasp target object name
        assemble_point_name : string
            assemble point name
        relative_workpoint : denso_state_msgs/WorkPoint
            relative workpoint
        """
        State.__init__(self, outcomes=['success', 'failed'])
        if not isinstance(object_name, str):
            raise TypeError(
                'object_name must be a type of string')
        if not isinstance(assemble_point_name, str):
            raise TypeError(
                'assemble_point_name must be a type of string')
        if not isinstance(relative_workpoint, WorkPoint):
            raise TypeError(
                'relative_workpoint must be a type of denso_state_msgs/WorkPoint')
        self.object_name_ = object_name
        self.assemble_point_name_ = assemble_point_name
        self.relative_workpoint = relative_workpoint
        self.set_work_point_srv_ = rospy.ServiceProxy(
            '/fix_eef_joint_value/set_workpoint', SetWorkPoint)

    def execute(self, userdata):
        measurement_behavior = MeasurementBehavior(
            self.object_name_, self.assemble_point_name_, self.relative_workpoint)

        ret, workpoint = measurement_behavior.execute_impl()
        if not ret:
            return 'failed'
        else:
            req = SetWorkPointRequest()
            req.workpoint = workpoint
            try:
                ret = self.set_work_point_srv_.call(req)
            except rospy.ServiceException as e:
                rospy.logerr('set work point service call failed:{}'.format(e))

        if ret:
            return 'success'
        else:
            return 'failed'
