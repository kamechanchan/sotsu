#!/usr/bin/env python

import rospy
import moveit_commander
import tf
import tf2_ros
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from denso_state_msgs.msg import WorkPoint
from third_2019.srv import GetFixedTarget, GetFixedTargetResponse
import re
import math
import copy


class TargetValueGenerator(object):
    def __init__(self):
        self.dof = 6

        # to solve FK and IK
        self.robot = moveit_commander.RobotCommander()
        self.arm = moveit_commander.MoveGroupCommander("arm")
        rospy.wait_for_service('/compute_fk')
        rospy.wait_for_service('/compute_ik')
        self.fk_service = rospy.ServiceProxy('/compute_fk', GetPositionFK)
        self.ik_service = rospy.ServiceProxy('/compute_ik', GetPositionIK)
        self.compute_limit = 10000

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.point_names = ["grasp_point", "assemble_point"]
        self.source_frame = "world"

        self.target_value_srv = rospy.Service("/get_fixed_target", GetFixedTarget, self.get_target_value)

    def _get_link_name(self):
        robot_description_semantic = rospy.get_param(
            "/robot_description_semantic")
        pre_pattern = r"(.*)link\ name=(.*)"
        post_pattern = "(?<=\").*?(?=\")"
        linkname_list = re.findall(post_pattern, "\n".join(
            map(str, re.findall(pre_pattern, robot_description_semantic))))
        return linkname_list[:self.dof]

    def _compute_fk(self, joint_values):
        req = GetPositionFKRequest()
        robot_state = self.robot.get_current_state()
        robot_state.joint_state.position = joint_values
        req.header.stamp = rospy.Time.now()
        req.header.frame_id = "base_link"
        req.fk_link_names = self._get_link_name()
        req.robot_state = robot_state

        try:
            timeout = 0
            while True:
                ret = self.fk_service(req)
                if ret.error_code.val == 1:
                    break
                timeout += 1
                if timeout >= self.compute_limit:
                    rospy.logerr("Could not obtein valid FK solution")
                    break
        except rospy.ServiceException as e:
            rospy.logerr("compute FK service failed:{}".format(e))
            return False

        if ret.error_code.val == 1:
            return ret.pose_stamped[self.dof - 1]
        else:
            rospy.logerr("compute FK service failed")
            return False

    def _compute_ik(self, trans):
        robot_state = self.robot.get_current_state()

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.pose.position = trans.transform.translation
        pose_stamped.pose.orientation = trans.transform.rotation

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

    def _fix_value_into_range_of_motion(
            self, grasp_target_trans, assemble_target_trans):
        fixed_grasp_target_trans = copy.deepcopy(grasp_target_trans)
        fixed_assemble_target_trans = copy.deepcopy(assemble_target_trans)

        theta_g = self._compute_ik(grasp_target_trans)
        theta_a = self._compute_ik(assemble_target_trans)
        delta_theta = theta_a[self.dof - 1] - theta_g[self.dof - 1]

        if delta_theta >= 0:
            if abs(delta_theta) <= math.pi:
                margin = (math.pi - delta_theta) / 2.0
                theta_g[self.dof - 1] = 0 + margin
                theta_a[self.dof - 1] = delta_theta + margin
            else:
                margin = (delta_theta - math.pi) / 2.0
                theta_g[self.dof - 1] = math.pi - margin
                theta_a[self.dof - 1] = delta_theta - math.pi - margin
        else:
            if abs(delta_theta) <= math.pi:
                margin = (delta_theta + math.pi) / 2.0
                theta_g[self.dof - 1] = math.pi - margin
                theta_a[self.dof - 1] = delta_theta + math.pi - margin
            else:
                margin = (math.pi - (delta_theta + (2 * math.pi))) / 2.0
                theta_g[self.dof - 1] = 0 + margin
                theta_a[self.dof - 1] = delta_theta + (2 * math.pi) + margin

        print("===THETA G | THETA A===")
        print(theta_g[self.dof - 1], theta_a[self.dof - 1])
        print("=====")
        grasp_pose = self._compute_fk(theta_g)
        assemble_pose = self._compute_fk(theta_a)

        fixed_grasp_target_trans.transform.translation = grasp_pose.pose.position
        fixed_grasp_target_trans.transform.rotation = grasp_pose.pose.orientation
        fixed_assemble_target_trans.transform.translation = assemble_pose.pose.position
        fixed_assemble_target_trans.transform.rotation = assemble_pose.pose.orientation

        return fixed_grasp_target_trans, fixed_assemble_target_trans

    def get_target_value(self, req):
        while not rospy.is_shutdown():
            try:
                grasp_target_trans = self.tf_buffer.lookup_transform(
                    self.source_frame, self.point_names[0], rospy.Time(0), rospy.Duration(1.0))
                assemble_target_trans = self.tf_buffer.lookup_transform(
                    self.source_frame, self.point_names[1], rospy.Time(0), rospy.Duration(1.0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn("{} retry...".format(e))
                rospy.sleep(1.0)
                continue

        fixed_grasp_target_trans, fixed_assemble_target_trans = self._fix_value_into_range_of_motion(
            grasp_target_trans, assemble_target_trans)

        workpoint = WorkPoint()
        workpoint.header = fixed_grasp_target_trans.header
        workpoint.grasp.position = fixed_grasp_target_trans.transform.translation
        workpoint.grasp.orientation = fixed_grasp_target_trans.transform.rotation
        workpoint.assemble.position = fixed_assemble_target_trans.transform.translation
        workpoint.assemble.orientation = fixed_assemble_target_trans.transform.rotation

        res = GetFixedTargetResponse()
        res.success = True
        res.workpoint = workpoint
        return res


if __name__ == "__main__":
    rospy.init_node("target_value_generator")
    generator = TargetValueGenerator()
    rospy.spin()
