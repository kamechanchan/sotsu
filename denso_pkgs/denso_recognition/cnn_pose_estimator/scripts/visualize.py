#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


class TFDisplay:

    def __init__(self):
        rospy.init_node('sample_node')
        self.frame1_pose = [0.5, 0.5, 0.5,
                            0.232182, -0.166374, -0.65957, 0.695254]
        self.frame2_pose = [0.5, 0.5, 0.5, 0.21339089161, -
                            0.15271546082, -0.67449055154, 0.69007463441]

    def getTF(self, frame_name, pose):
        sample_tf = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = frame_name
        t.transform.translation.x = pose[0]
        t.transform.translation.y = pose[1]
        t.transform.translation.z = pose[2]
        t.transform.rotation.x = pose[3]
        t.transform.rotation.y = pose[4]
        t.transform.rotation.z = pose[5]
        t.transform.rotation.w = pose[6]
        sample_tf.sendTransform(t)

    def sendTF(self):
        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            self.getTF("real_value", self.frame1_pose)
            self.getTF("estimated_value", self.frame2_pose)
            rate.sleep()


if __name__ == '__main__':
    tf_sample = TFDisplay()
    try:
        tf_sample.sendTF()
    except rospy.ROSInternalException:
        pass
