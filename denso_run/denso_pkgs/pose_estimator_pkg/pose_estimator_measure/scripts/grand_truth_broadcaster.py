#! /usr/bin/env python
import rospy
import tf
import tf2_ros
import geometry_msgs.msg
import math



class GT_Broadcaster(object):
    def __init__(self, x=0, y=0.24, z=0.024):
        self.x = x
        self.y = y
        self.z = z
        self.br_all = tf2_ros.StaticTransformBroadcaster()
        self.bbox_pose_center = geometry_msgs.msg.TransformStamped()
        self.bbox_pose_L = geometry_msgs.msg.TransformStamped()
        self.bbox_pose_H = geometry_msgs.msg.TransformStamped()

    def set_bbox(self, bbox_L, bbox_H):
        self.bbox_pose_L.header.stamp = rospy.Time.now()
        self.bbox_pose_L.header.frame_id = "grand_truth"
        self.bbox_pose_L.child_frame_id = "BBox_L"

        self.bbox_pose_H.header.stamp = rospy.Time.now()
        self.bbox_pose_H.header.frame_id = "grand_truth"
        self.bbox_pose_H.child_frame_id = "BBox_H"

        self.bbox_pose_H.transform.translation.x = bbox_H.position.x
        self.bbox_pose_H.transform.translation.y = bbox_H.position.y
        self.bbox_pose_H.transform.translation.z = bbox_H.position.z
        quat = tf.transformations.quaternion_from_euler(math.pi/2, math.pi, 0)
        self.bbox_pose_H.transform.rotation.x = quat[0]
        self.bbox_pose_H.transform.rotation.y = quat[1]
        self.bbox_pose_H.transform.rotation.z = quat[2]

        self.bbox_pose_L.transform.translation.x = bbox_L.position.x
        self.bbox_pose_L.transform.translation.y = bbox_L.position.y
        self.bbox_pose_L.transform.translation.z = bbox_L.position.z
        quat = tf.transformations.quaternion_from_euler(-math.pi/2, math.pi, -math.pi/2)
        self.bbox_pose_L.transform.rotation.x = quat[0]
        self.bbox_pose_L.transform.rotation.y = quat[1]
        self.bbox_pose_L.transform.rotation.z = quat[2]
        self.bbox_pose_L.transform.rotation.w = quat[3]

    def br_publisher(self):
        tf_list = []
        self.bbox_pose_center.header.stamp = rospy.Time.now()
        self.bbox_pose_center.header.frame_id = "J6"
        self.bbox_pose_center.child_frame_id = "grand_truth"

        self.bbox_pose_center.transform.translation.x = self.x
        self.bbox_pose_center.transform.translation.y = self.y
        self.bbox_pose_center.transform.translation.z = self.z

        # quat = tf.transformations.quaternion_from_euler(0, math.pi, -math.pi/2 + 1.22)
        quat = tf.transformations.quaternion_from_euler(0, math.pi, -math.pi/2)

        self.bbox_pose_center.transform.rotation.x = quat[0]
        self.bbox_pose_center.transform.rotation.y = quat[1]
        self.bbox_pose_center.transform.rotation.z = quat[2]
        self.bbox_pose_center.transform.rotation.w = quat[3]

        tf_list.append(self.bbox_pose_L)
        tf_list.append(self.bbox_pose_H)
        tf_list.append(self.bbox_pose_center)

        self.br_all.sendTransform(tf_list)
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('measure_position_broadcaster')

    node = GT_Broadcaster(x=-0.001, y=0.16149, z=0.171)  ##for HV8
    # node = GT_Broadcaster(x=-0.001, y=0.160, z=0.169)    ##for HV6
    # node = GT_Broadcaster(x=-0.001, y=0.160, z=0.167)    ##for HV6
    # node = GT_Broadcaster(x=0.001, y=0.161, z=0.167)    ##for HV6

    # node = GT_Broadcaster(x=-0.001, y=0.16149, z=0.174)  ##for HV8
    #node = GT_Broadcaster(x=-0.001, y=0.160, z=0.169)    ##for HV6
    # node = GT_Broadcaster(x=-0.001, y=0.160, z=0.169)    ##for HV6

    Bbox_L = geometry_msgs.msg.Pose()
    Bbox_H = geometry_msgs.msg.Pose()

    Bbox_L.position.x = 0.05
    Bbox_L.position.y = -0.05
    Bbox_L.position.z = -0.025

    Bbox_H.position.x = -0.05
    Bbox_H.position.y = 0.05
    Bbox_H.position.z = 0.025

    node.set_bbox(Bbox_L, Bbox_H)
    node.br_publisher()
