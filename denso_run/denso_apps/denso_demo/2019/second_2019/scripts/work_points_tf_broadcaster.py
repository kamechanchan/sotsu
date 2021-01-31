#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

from denso_state_msgs.msg import WorkPoint


class WorkPointsTFBroadcaster:
    def __init__(self):
        self.work_points_sub_ = rospy.Subscriber(
            "/relative_workpoint",
            WorkPoint,
            self.broadcastWorkPointsTF,
            queue_size=1)
        self.parts_list = ["HV8", "jig_HV6"]
        self.point_names = [
            "grasp_point",
            "assemble_part_point",
            "assemble_point"]
        self.grasp_point_transform = TransformStamped()
        self.assemble_part_point_transform = TransformStamped()
        self.assemble_point_transform = TransformStamped()
        self.grasp_point_tf = tf2_ros.TransformBroadcaster()
        self.assemble_part_point_tf = tf2_ros.TransformBroadcaster()
        self.assemble_point_tf = tf2_ros.TransformBroadcaster()

    def setTFinfo(self, pose, parts_name, points_name, index=None):
        t = TransformStamped()
        if index is not None:
            t.header.frame_id = parts_name + "_" + str(index)
        else:
            t.header.frame_id = parts_name
        t.child_frame_id = points_name
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation.x = pose.orientation.x
        t.transform.rotation.y = pose.orientation.y
        t.transform.rotation.z = pose.orientation.z
        t.transform.rotation.w = pose.orientation.w
        return t

    def broadcastWorkPointsTF(self, msg):
        relative_grasp_point = msg.grasp
        relative_assemble_point = msg.assemble

        self.grasp_point_transform = self.setTFinfo(
            relative_grasp_point, self.parts_list[0], self.point_names[0], 0)
        self.assemble_part_point_transform = self.setTFinfo(
            relative_assemble_point, self.parts_list[1], self.point_names[1], 0)
        self.assemble_point_transform = self.setTFinfo(
            relative_grasp_point, self.point_names[1], self.point_names[2])

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.grasp_point_transform.header.stamp = rospy.Time.now()
            self.assemble_part_point_transform.header.stamp = rospy.Time.now()
            self.assemble_point_transform.header.stamp = rospy.Time.now()
            self.grasp_point_tf.sendTransform(self.grasp_point_transform)
            self.assemble_part_point_tf.sendTransform(
                self.assemble_part_point_transform)
            self.assemble_point_tf.sendTransform(self.assemble_point_transform)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node("work_points_tf_broadcaster_node")
    br = WorkPointsTFBroadcaster()
    rospy.spin()
