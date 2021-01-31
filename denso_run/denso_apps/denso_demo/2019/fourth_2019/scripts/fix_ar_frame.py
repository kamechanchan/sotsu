#!/usr/bin/env python
import rospy
import tf2_ros


class RenamingTF:

    def __init__(self):
        rospy.init_node('renaming_ar_tf', anonymous=False)
        self.base_frame_name = "world"
        self.ar_frame_name = "ar_marker_3"
        self.new_frame_name = "assemble_part_base_frame"
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

    def get_ar_trans(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.base_frame_name,
                self.ar_frame_name,
                rospy.Time(0),
                rospy.Duration(1.0))
            return trans
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("AR frame is not found.")
            return False

    def renameFrame(self, trans):
        loop_rate = rospy.Rate(10)
        trans.child_frame_id = self.new_frame_name
        trans.header.frame_id = self.base_frame_name
        trans.transform.translation.x = trans.transform.translation.x + 0.055  
        trans.transform.translation.y = trans.transform.translation.y + 0.003
        trans.transform.translation.z = trans.transform.translation.z + 0.03
        while not rospy.is_shutdown():
            trans.header.stamp = rospy.Time.now()
            self.tf_broadcaster.sendTransform(trans)
            loop_rate.sleep()


if __name__ == "__main__":
    rename_tf = RenamingTF()
    while True:
        trans = rename_tf.get_ar_trans()
        if not (trans is False):
            break
    rename_tf.renameFrame(trans)
