#!/usr/bin/env python

import rospy
from .behavior import Behavior
from denso_state_msgs.msg import WorkPoint
from denso_state_srvs.srv import SetWorkPoint, SetWorkPointRequest
from denso_state_srvs.srv import Measurement, MeasurementRequest
from denso_state_srvs.srv import MeasurementWorkPoint, MeasurementWorkPointRequest


class MeasurementBehavior(Behavior):
    """define measurement behavior"""

    def __init__(
            self,
            object_name,
            assemble_point_name,
            relative_workpoint):
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

        if isinstance(object_name, str):
            self.object_name_ = object_name
        else:
            raise TypeError('object_name must be a type of str')

        if isinstance(assemble_point_name, str):
            self.assemble_point_name_ = assemble_point_name
        else:
            raise TypeError('assemble_point_name be a type of str')

        if not isinstance(relative_workpoint, WorkPoint):
            raise TypeError(
                'relative_workpoint must be a type of denso_state_msgs/WorkPoint')

        self.relative_workpoint = relative_workpoint
        self.set_workpoint_srv_ = rospy.ServiceProxy(
            '/measurement/set_work_point', SetWorkPoint)
        self.measurement_srv_ = rospy.ServiceProxy(
            '/measurement/get_work_point', Measurement)
        self.measurement_work_point_srv_ = rospy.ServiceProxy(
            '/measurement/measure_work_point', MeasurementWorkPoint)

    def execute_impl(self):
        workpoint = WorkPoint()

        req = SetWorkPointRequest()
        req.workpoint = self.relative_workpoint
        try:
            ret = self.set_workpoint_srv_.call(req)
            if not ret.success:
                rospy.logerr(
                    'measurement behavior service call failed:{}'.format(e))
        except rospy.ServiceException as e:
            rospy.logerr(
                'measurement behavior service call failed:{}'.format(e))

        req = MeasurementRequest()
        req.grasp_point_name = self.object_name_
        req.assemble_point_name = self.assemble_point_name_
        try:
            ret = self.measurement_srv_.call(req)
        except rospy.ServiceException as e:
            rospy.logerr(
                'measurement behavior service call failed:{}'.format(e))

        if not ret.success:
            return False, workpoint

        rate = rospy.Rate(1)
        rate.sleep()

        req = MeasurementWorkPointRequest()

        ret = self.measurement_work_point_srv_.call(req)

        if ret.success:
            workpoint = ret.workpoint
            return True, workpoint
        else:
            return False, workpoint
