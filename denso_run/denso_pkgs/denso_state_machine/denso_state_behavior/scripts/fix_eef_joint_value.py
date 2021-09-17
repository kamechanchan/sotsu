#!/usr/bin/env python

import rospy
import moveit_commander
import tf
import tf2_ros
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from denso_state_msgs.msg import WorkPoint
from denso_state_srvs.srv import SetWorkPoint, SetWorkPointResponse
from denso_state_srvs.srv import GetFixedJointValues, GetFixedJointValuesResponse
import re
import math
import copy


class FixEEFJointValue(object):
    def __init__(self):
        self.dof = rospy.get_param("~dof", 6)

        # to solve FK and IK
        self.robot = moveit_commander.RobotCommander()
        self.arm = moveit_commander.MoveGroupCommander("arm")
        rospy.wait_for_service('/compute_ik')
        self.ik_service = rospy.ServiceProxy('/compute_ik', GetPositionIK)
        self.compute_limit = 10000

        self._workpoint = None

        rospy.Service(
            '/fix_eef_joint_value/set_workpoint',
            SetWorkPoint,
            self.set_workpoint)
        rospy.Service(
            '/fix_eef_joint_value/get_grasp_point',
            GetFixedJointValues,
            self.get_grasp_point)
        rospy.Service(
            '/fix_eef_joint_value/get_assemble_point',
            GetFixedJointValues,
            self.get_assemble_point)

    def _get_link_name(self):
        robot_description_semantic = rospy.get_param(
            "/robot_description_semantic")
        pre_pattern = r"(.*)link\ name=(.*)"
        post_pattern = "(?<=\").*?(?=\")"
        linkname_list = re.findall(post_pattern, "\n".join(
            map(str, re.findall(pre_pattern, robot_description_semantic))))
        return linkname_list[:self.dof]

    def _get_joint_name(self):
        robot_description_semantic = rospy.get_param(
            "/robot_description_semantic")
        pre_pattern = r"(.*)joint\ name=(.*)"
        post_pattern = "(?<=\").*?(?=\")"
        jointname_list = re.findall(post_pattern, "\n".join(
            map(str, re.findall(pre_pattern, robot_description_semantic))))
        return jointname_list[:self.dof]

    def _compute_ik(self, pose):
        robot_state = self.robot.get_current_state()

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.pose = copy.deepcopy(pose)

        req = PositionIKRequest()
        req.group_name = "arm"
        req.robot_state = robot_state
        req.avoid_collisions = True
        req.pose_stamped = pose_stamped

        try:
            timeout = 0
            while True:
                ret = self.ik_service(req)
                if ret.error_code.val == 1:
                    break
                timeout += 1
                if timeout >= self.compute_limit:
                    rospy.logerr("Could not obtein valid IK solution")
                    break
        except rospy.ServiceException as e:
            rospy.logerr("compute IK service failed:{}".format(e))
            return False

        if not ret.solution.joint_state:
            rospy.logerr('Could not obtein valid IK solution')
            return False
        else:
            return list(ret.solution.joint_state.position)

    def _fix_value_into_range_of_motion(self, workpoint):
        theta_g = self._compute_ik(workpoint.grasp)
        theta_a = self._compute_ik(workpoint.assemble)
        delta_theta = theta_a[self.dof - 1] - theta_g[self.dof - 1]

        # if delta_theta >= 0:
        #     if abs(delta_theta) <= math.pi:
        #         margin = (math.pi - delta_theta) / 2.0
        #         theta_g[self.dof - 1] = 0 + margin
        #         theta_a[self.dof - 1] = delta_theta + margin
        #     else:
        #         margin = (delta_theta - math.pi) / 2.0
        #         theta_g[self.dof - 1] = math.pi - margin
        #         theta_a[self.dof - 1] = delta_theta - math.pi - margin
        # else:
        #     if abs(delta_theta) <= math.pi:
        #         margin = (delta_theta + math.pi) / 2.0
        #         theta_g[self.dof - 1] = math.pi - margin
        #         theta_a[self.dof - 1] = delta_theta + math.pi - margin
        #     else:
        #         margin = (math.pi - (delta_theta + (2 * math.pi))) / 2.0
        #         theta_g[self.dof - 1] = 0 + margin
        #         theta_a[self.dof - 1] = delta_theta + (2 * math.pi) + margin

        print("===THETA G | THETA A===")
        print(theta_g[self.dof - 1], theta_a[self.dof - 1])
        print("=====")

        return theta_g[:self.dof], theta_a[:self.dof]

    def set_workpoint(self, req):
        res = SetWorkPointResponse()
        self._workpoint = copy.deepcopy(req.workpoint)
        res.success = True
        return res

    def get_grasp_point(self, req):
        res = GetFixedJointValuesResponse()

        if self._workpoint is None:
            rospy.logerr("WorkPoint is not initialized")
            res.success = False
            return res

        workpoint = copy.deepcopy(self._workpoint)
        workpoint_offset = copy.deepcopy(workpoint)

        workpoint_offset.grasp.position.z += req.offset
        workpoint_offset.assemble.position.z += req.offset

        theta_g, _ = self._fix_value_into_range_of_motion(workpoint)
        theta_g_offset, _ = self._fix_value_into_range_of_motion(
            workpoint_offset)

        res.joint_names = self._get_joint_name()
        res.joint_values = theta_g
        res.joint_values_offset = theta_g_offset
        res.success = True

        return res

    def get_assemble_point(self, req):
        res = GetFixedJointValuesResponse()

        if self._workpoint is None:
            rospy.logerr("WorkPoint is not initialized")
            res.success = False
            return res

        workpoint = copy.deepcopy(self._workpoint)
        workpoint_offset = copy.deepcopy(workpoint)

        workpoint_offset.grasp.position.z += req.offset
        workpoint_offset.assemble.position.z += req.offset

        _, theta_a = self._fix_value_into_range_of_motion(workpoint)
        _, theta_a_offset = self._fix_value_into_range_of_motion(
            workpoint_offset)

        res.joint_names = self._get_joint_name()
        res.joint_values = theta_a
        res.joint_values_offset = theta_a_offset
        res.success = True

        return res


if __name__ == "__main__":
    rospy.init_node("fix_eef_joint_value")
    fix_eef_joint_value = FixEEFJointValue()
    rospy.spin()
