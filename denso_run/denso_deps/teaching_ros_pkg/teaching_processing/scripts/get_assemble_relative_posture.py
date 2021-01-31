#!/usr/bin/env python

import rospy
import rospkg
import tf
import tf2_ros
import yaml

class GetAssembleRelativePosture(object):
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.save_file = rospy.get_param("~assemble_posture_file", "assemble_result.yaml")
        self.grasp_part = rospy.get_param("~grasp_object", "HV8_0")
        self.assembled_part = rospy.get_param("~assembled_src_object", "jig_HV6_0")

    def save_ralative_posture(self, trans):
        rospack = rospkg.RosPack()
        file_path = rospack.get_path('execute_robot') + '/data/' + self.save_file
        yaml_dict = {}
        s_yaml_dict = {"src_object": self.assembled_part,
                       "point_name": "assemble_point",
                       "x": trans.transform.translation.x,
                       "y": trans.transform.translation.y,
                       "z": trans.transform.translation.z,
                       "qx": trans.transform.rotation.x,
                       "qy": trans.transform.rotation.y,
                       "qz": trans.transform.rotation.z,
                       "w": trans.transform.rotation.w}

        yaml_dict["assemble_posture_result"] = s_yaml_dict

        with open(file_path, "w") as f:
            yaml.dump(yaml_dict, f, default_flow_style=False)

        rospy.loginfo("Finish to save relative assembled posture")

    def get_ralative_posture(self):
        while not rospy.is_shutdown():
            try:
                assemble_target_trans = self.tf_buffer.lookup_transform(
                    self.assembled_part, self.grasp_part, rospy.Time(0), rospy.Duration(1.0))
                break
            except:
                rospy.logwarn("frame is not found !!")
                continue

        return assemble_target_trans

    def run(self):
        trans = self.get_ralative_posture()
        self.save_ralative_posture(trans)

if __name__ == '__main__':
    rospy.init_node("get_assemble_relative_posture_node")
    relativeposture = GetAssembleRelativePosture()
    relativeposture.run()
