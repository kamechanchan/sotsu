#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from denso_state_srvs.srv import Moving
from denso_state_srvs.srv import MovingRequest
from .behavior import Behavior


class MovingBehavior(Behavior):
    """define moving transition behavior"""

    def __init__(
            self,
            target_pose=None,
            joint_names=None,
            target_joint_values=None,
            num_generate_traj=0):
        """
        Parameters
        ----------
        target_pose : geometry_msgs/PoseStamped, default None
            target point of robot
        joint_names : list of str, default None
            list of robot joint names. specify with target_joint_values
        target_joint_values : list of float, default None
            target joint value of robot. specify with joint_names
        num_generate_traj : int, default 0(planner default)
            number of generate candidate paths when planning
        """

        if ((target_pose is None) and (joint_names is None)
                and (target_joint_values is None)):
            raise RuntimeError('Missing required arguments')

        if target_pose:
            if not isinstance(target_pose, PoseStamped):
                raise TypeError(
                    'target_pose must be a geometry_msgs/PoseStamped')
        self.target_pose = target_pose

        if joint_names:
            print(joint_names)
            if (not isinstance(joint_names, list)) or (
                    not isinstance(joint_names[0], str)):
                raise TypeError('joint_names must be a list of str')
        self.joint_names = joint_names

        if target_joint_values:
            print(target_joint_values)
            if (not (isinstance(target_joint_values, list) or isinstance(
                    target_joint_values, tuple)) or (not isinstance(target_joint_values[0], float))):
                raise TypeError('target_joint_values must be a list of float')
        self.target_joint_values = target_joint_values

        if isinstance(num_generate_traj, int):
            self.num_generate_traj = num_generate_traj
        else:
            raise TypeError(
                'num_generate_traj must be a type of int')
        self.moving_srv = rospy.ServiceProxy(
            '/state_behavior/moving', Moving)

    def execute_impl(self):
        req = MovingRequest()
        if (self.joint_names is not None) and (
                self.target_joint_values is not None):
            req.joint_names = self.joint_names
            req.target_joint_values = self.target_joint_values
        else:
            req.target_pose = self.target_pose
        req.num_generate_traj = self.num_generate_traj
        try:
            ret = self.moving_srv(req)
        except rospy.ServiceException as e:
            rospy.logerr('moving behavior service call failed:{}'.format(e))
        if ret.success:
            return 'success'
        else:
            return 'failed'
