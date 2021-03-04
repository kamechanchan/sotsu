#!/usr/bin/env python

import rospy
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped

from third_2019.srv import WorkPointTFBroadcast, WorkPointTFBroadcastResponse

class WorkPointsTFBroadcaster:
    def __init__(self):
        self.parts_list = ["bulk_HV8", "jig_HV6_0"]
        self.target_object = ""
        self.point_names = [
            "grasp_point",
            "assemble_part_point",
            "assemble_point"]
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.grasp_point_transform = TransformStamped()
        self.assemble_part_point_transform = TransformStamped()
        self.assemble_point_transform = TransformStamped()
        self.grasp_point_tf = tf2_ros.TransformBroadcaster()
        self.assemble_part_point_tf = tf2_ros.TransformBroadcaster()
        self.assemble_point_tf = tf2_ros.TransformBroadcaster()
        self.x_min = 0.1
        self.x_max = 1.0
        self.y_min = 0.1
        self.y_max = 1.0
        self.z_min = 0.0
        self.z_max = 1.0
        self.relative_grasp_point = None
        self.relative_assemble_point = None
        self.is_data_arrived = False
        self.work_points_tf_broadcaster_srv = rospy.Service("/work_points_tf_broadcast", WorkPointTFBroadcast, self.updateRelativePoint)

    def updateRelativePoint(self, msg):
        res = WorkPointTFBroadcastResponse()
        index = 0
        timeout = rospy.Duration(1.0)
        while not rospy.is_shutdown():
            try:
                trans = self.tf_buffer.lookup_transform("base_link", self.parts_list[0]+"_"+str(index), rospy.Time(0), timeout=timeout)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                res.success = False
                return res

            if self.isInlier(trans):
                res.success = True
                self.target_object = self.parts_list[0]+"_"+str(index)
                self.relative_grasp_point = msg.relative_point.grasp
                self.relative_assemble_point = msg.relative_point.assemble
                self.is_data_arrived = True
                return res
            else:
                index += 1

    def isInlier(self, trans):
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        z = trans.transform.translation.z

        if (x < self.x_min) or (x > self.x_max):
            return False
        if (y < self.y_min) or (y > self.y_max):
            return False
        if (z < self.z_min) or (z > self.z_max):
            return False

        return True


    def setTFinfo(self, pose, target_object_name, points_name):
        t = TransformStamped()
        t.header.frame_id = target_object_name
        t.child_frame_id = points_name
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation.x = pose.orientation.x
        t.transform.rotation.y = pose.orientation.y
        t.transform.rotation.z = pose.orientation.z
        t.transform.rotation.w = pose.orientation.w
        return t

    def broadcastWorkPointsTF(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.is_data_arrived is False:
                rate.sleep()
                continue
            self.grasp_point_transform = self.setTFinfo(
                self.relative_grasp_point, self.target_object, self.point_names[0])
            self.assemble_part_point_transform = self.setTFinfo(
                self.relative_assemble_point, self.parts_list[1], self.point_names[1])
            self.assemble_point_transform = self.setTFinfo(
                self.relative_grasp_point, self.point_names[1], self.point_names[2])

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
    br.broadcastWorkPointsTF()
    rospy.spin()
