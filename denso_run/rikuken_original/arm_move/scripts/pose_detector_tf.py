#!/usr/bin/env python3
import rospy
import tf2_ros
import math
from geometry_msgs.msg import *
import tf_conversions
import tf
from tf.transformations import quaternion_from_euler
import numpy

def tf_get(tfbuffer, source, target):
    rate = rospy.Rate(10)
    while 1:
        try:
            trans = tfbuffer.lookup_transform(source, target, rospy.Time())
            break
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
    return trans

if __name__ == '__main__':
    rospy.init_node("tsuchida")
    tfBuffer = tf2_ros.Buffer()
    tfLisner = tf2_ros.TransformListener(tfBuffer)
    br = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        msg = TransformStamped()
        msg = tf_get(tfBuffer, "world", "HV8")
        pose_a = msg.transform
        pose_b = msg.transform
        q_trans = numpy.array([pose_a.translation.x, pose_a.translation.y, pose_a.translation.z, 0])
        q_trans_zero = numpy.array([0, 0, 0.1, 0])
        # pose_b.translation.x = pose_a.translation.x
        # pose_b.translation.z = pose_a.translation.z + 0.1
        q = tf_conversions.transformations.quaternion_from_euler(0, math.pi, 0)
        # q = quaternion_from_euler(0, math.pi, 0)
        
        # print(type(q))
        q_moto = numpy.array([pose_a.rotation.x, pose_a.rotation.y, pose_a.rotation.z, pose_a.rotation.w])  
        # q_moto = Quaternion(pose_a.rotation.x, pose_a.rotation.y, pose_a.rotation.z, pose_a.rotation.w)
        # q_moto = numpy.array(q_moto)
        # print(type(q_moto))
        q_trans_new = tf_conversions.transformations.quaternion_multiply(q_moto, q_trans_zero)
        q_trans_new = tf_conversions.transformations.quaternion_multiply(q_moto, q_trans_new)
        # q_trans_new = tf_conversions.transformations.quaternion_multiply(q_trans, q_moto)
        q_new = tf_conversions.transformations.quaternion_multiply(q_moto, q)

        
        # q_new = tf.transformations.quaternion_multiply(q_moto, q)
        # q_trans_new = q_trans
        pose_b.translation.x = q_trans_new[0] + q_trans[0]
        pose_b.translation.y = q_trans_new[1] + q_trans[1]
        pose_b.translation.z = q_trans_new[2] + q_trans[2]
        # pose_b.translation.x = q_trans_new[0] 
        # pose_b.translation.y = q_trans_new[1]
        # pose_b.translation.z = q_trans_new[2] 
        pose_b.rotation.x = q_new[0]
        pose_b.rotation.y = q_new[1]
        pose_b.rotation.z = q_new[2]
        pose_b.rotation.w = q_new[3]
        # pose_b.rotation.x = 0
        # pose_b.rotation.y = 0
        # pose_b.rotation.z = 0
        # pose_b.rotation.w = 1
        send_tf = TransformStamped()
        send_tf.header.stamp = rospy.Time.now()
        send_tf.header.frame_id = "world"
        send_tf.child_frame_id = "tsuchida_tf"
        send_tf.transform = pose_b
        br.sendTransform(send_tf)
        rate.sleep()

    


