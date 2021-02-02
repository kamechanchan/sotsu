#!/usr/bin/env python

import sys
import random, time
import tf
from tf.transformations import quaternion_from_euler
from math import *
from sensor_msgs.msg import PointCloud2

import ros_numpy
from util import *
import open3d as o3d
import pcl
import rospy
from gazebo_msgs.srv import *
from gazebo_msgs.msg import *

class RandomMove(object):
    def __init__(self):
        self.pos_up_ = ModelState()
        self.pos_down_ = ModelState()
        self.num_ =0
        self.sensor_parent_frame_ = rospy.get_param("~sensor_parent_frame", "photoneo_center_optical_frame")
        self.pos_up_.model_name = rospy.get_param("~object_up", "HV8_up")
        self.pos_down_.model_name = rospy.get_param("~object_down", "HV8_down")
        self.sub_ = rospy.Subscriber("/photoneo_center/sensor/points", PointCloud2, self.record_data)
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            self.set_model_ = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def random_state_make(self):
        self.pos_up_.pose.position.x = random.uniform(-0.05, 0.05)
        self.pos_up_.pose.position.y = random.uniform(-0.3, -0.1)
        self.pos_up_.pose.position.z = random.uniform(0.0001, 0.1)

        self.pos_down_.pose.position.x = random.uniform(-0.05, 0.05)
        self.pos_down_.pose.position.y = random.uniform(0.1, 0.3)
        self.pos_down_.pose.position.z = random.uniform(0.0001, 0.1)

        roll = random.uniform(-3.14, 3.14)
        pitch = random.uniform(-3.14, 3.14)
        yaw = random.uniform(-3.14, 3.14)

        quat = quaternion_from_euler(roll, pitch, yaw)
        self.pos_up_.pose.orientation.x = quat[0]
        self.pos_up_.pose.orientation.y = quat[1]
        self.pos_up_.pose.orientation.z = quat[2]
        self.pos_up_.pose.orientation.w = quat[3]

        self.pos_down_.pose.orientation.x = quat[0]
        self.pos_down_.pose.orientation.y = quat[1]
        self.pos_down_.pose.orientation.z = quat[2]
        self.pos_down_.pose.orientation.w = quat[3]

        return self.set_model_(self.pos_up_), self.set_model_(self.pos_down_)

    def record_data(self, data):
        header = data.header
        pc = ros_numpy.numpify(data)
        height = pc.shape[0]
        width = pc.shape[1]
        np_points = np.zeros((height * width, 3), dtype=np.float32)
        np_points[:, 0] = np.resize(pc['x'], height * width)
        np_points[:, 1] = np.resize(pc['y'], height * width)
        np_points[:, 2] = np.resize(pc['z'], height * width)
        p = pcl.PointCloud(np.array(np_points, dtype=np.float32))
        self.random_state_make()
        self.num_ += 1
        print(self.num_)



def main():
    rospy.init_node("random_state_maker_node", anonymous=False)
    random_state_maker = RandomMove()
    rospy.spin()

if __name__ == '__main__':
    main()
