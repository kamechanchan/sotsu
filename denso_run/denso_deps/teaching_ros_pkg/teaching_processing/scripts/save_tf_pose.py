#!/usr/bin/env python

import rospy
import rospkg
import yaml
from photoneo_localization.msg import TransformStampedArray

class SaveTFPose(object):
    def __init__(self):
        rospy.Subscriber("/localization_result_tf", TransformStampedArray, self.tf_pose_callback, queue_size=1)
        self.transform = TransformStampedArray()
        self.yaml_file = rospy.get_param("~object_tf_file", "scene_1.yaml")

    def tf_pose_callback(self, msg):
        self.transform = msg
        rospack = rospkg.RosPack()
        yaml_path = rospack.get_path(
            'teaching_processing') + '/data/' + self.yaml_file
        yaml_dict = {}
        for trans in self.transform.transforms:
            s_yaml_dict = {"object_name": trans.child_frame_id,
                           "x": trans.transform.translation.x,
                           "y": trans.transform.translation.y,
                           "z": trans.transform.translation.z,
                           "qx": trans.transform.rotation.x,
                           "qy": trans.transform.rotation.y,
                           "qz": trans.transform.rotation.z,
                           "w": trans.transform.rotation.w}
            yaml_dict[trans.child_frame_id] = s_yaml_dict

        with open(yaml_path, "w") as f:
            yaml.dump(yaml_dict, f, default_flow_style=False)

        rospy.loginfo("Finish to save object tf pose")


if __name__ == '__main__':
    rospy.init_node("save_tf_pose_node")
    savepose = SaveTFPose()
    rospy.spin()
