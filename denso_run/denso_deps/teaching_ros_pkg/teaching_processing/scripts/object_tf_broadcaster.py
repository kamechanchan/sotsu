#!/usr/bin/env python

import rospy
import rospkg
import yaml
import tf2_ros
from geometry_msgs.msg import TransformStamped

class ObjectTFBroadcaster(object):
    def __init__(self):
        self.yaml_file = rospy.get_param("~object_tf_file", "scene_1.yaml")
        self.object_config = rospy.get_param("/objects_list", "jig_and_HV6.yaml")
        self.sensor_frame = rospy.get_param("~sensor_frame", "photoneo_center")
        self.br = tf2_ros.TransformBroadcaster()
        self.rate = 10
        self.record_dict = {}

    def load_yaml_data(self):
        rospack = rospkg.RosPack()
        yaml_path = rospack.get_path('teaching_processing') + '/data/' + self.yaml_file
        with open(yaml_path, "r") as f:
            self.record_dict = yaml.load(f)

    def set_tf_config(self, trans):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.sensor_frame + "_optical_frame"
        t.child_frame_id = trans['object_name']
        t.transform.translation.x = trans['x']
        t.transform.translation.y = trans['y']
        t.transform.translation.z = trans['z']
        t.transform.rotation.x = trans['qx']
        t.transform.rotation.y = trans['qy']
        t.transform.rotation.z = trans['qz']
        t.transform.rotation.w = trans['w']
        return t

    def broadcast_tf(self):
        self.load_yaml_data()
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            for value in self.object_config:
                t = self.set_tf_config(self.record_dict[value])
                self.br.sendTransform(t)

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node("object_tf_broadcaster_node")
    objectbroadcaster = ObjectTFBroadcaster()
    objectbroadcaster.broadcast_tf()
