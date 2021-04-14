#!/usr/bin/env python3

import sys
import random
import rospy
import tf
from tf.transformations import quaternion_from_euler
from math import *
from gazebo_msgs.msg import *
import tf2_ros
from time import *

class RandomMoveEuler(object):
    def __init__(self):
        self.set_model_state_pub_ = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
        self.pos_ = ModelState()
        self.tf_buffer_ = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_)
        self.sensor_parent_frame_ = rospy.get_param("~sensor_parent_frame", "world")
        self.pos_.model_name = rospy.get_param("~object_name", "HV8")
        self.object_name = rospy.get_param("~object_name", "HV8")
        self.init_x = rospy.get_param("~init_x", 0)
        self.receive_ok = rospy.set_param("/" + self.object_name + "/receive_cloud/is_ok", False)
        self.record_ok = rospy.set_param("/" + self.object_name + "/record_cloud/is_ok", False)

    def isReadyMove(self):
        try:
            self.tf_buffer_.lookup_transform("world", self.pos_.model_name, rospy.Time(0), rospy.Duration(1.0))
            return True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception):
            return False


    def init_state_make(self):
        #self.pos_.pose.position.x = random.uniform(-0.04 + self.init_x, 0.04 + self.init_x)
        #self.pos_.pose.position.y = random.uniform(-0.4, 0.4)
        #self.pos_.pose.position.z = random.uniform(0.05, 0.25)
        self.pos_.pose.position.x = random.uniform(-0.4, 0.4)
        self.pos_.pose.position.y = random.uniform(-0.4, 0.4)
        self.pos_.pose.position.z = random.uniform(0.01, 0.2)

        roll = random.uniform(-0.5, 0.5)
        pitch = random.uniform(-0.5, 0.5)
        yaw = random.uniform(-3.14, 3.14)
        #roll = random.uniform(-3.14, 3.14)
        #pitch = random.uniform(-3.14, 3.14)
        #yaw = random.uniform(-3.14, 3.14)

        quat = quaternion_from_euler(roll, pitch, yaw)
        self.pos_.pose.orientation.x = quat[0]
        self.pos_.pose.orientation.y = quat[1]
        self.pos_.pose.orientation.z = quat[2]
        self.pos_.pose.orientation.w = quat[3]

        self.set_model_state_pub_.publish(self.pos_)
        return True

    def random_state_make(self):
        self.record_ok = rospy.get_param("/" + self.object_name + "/record_cloud/is_ok", False)
        if self.record_ok:
            self.pos_.pose.position.x = random.uniform(-0.4, 0.4)
            self.pos_.pose.position.y = random.uniform(-0.4, 0.4)
            self.pos_.pose.position.z = random.uniform(0.01, 0.2)

            roll = random.uniform(-0.5, 0.5)
            pitch = random.uniform(-0.5, 0.5)
            yaw = random.uniform(-3.14, 3.14)

            quat = quaternion_from_euler(roll, pitch, yaw)
            self.pos_.pose.orientation.x = quat[0]
            self.pos_.pose.orientation.y = quat[1]
            self.pos_.pose.orientation.z = quat[2]
            self.pos_.pose.orientation.w = quat[3]
            self.set_model_state_pub_.publish(self.pos_)

            rospy.set_param("/" + self.object_name + "/record_cloud/is_ok", False)
            rospy.set_param("/" + self.object_name + "/receive_cloud/is_ok", True)
        else:
            self.set_model_state_pub_.publish(self.pos_)

        return True


def main():
    rospy.init_node("random_state_maker_node", anonymous=False)
    random_state_maker = RandomMoveEuler()
    random_state_maker.init_state_make()
    while not random_state_maker.isReadyMove():
        rospy.logwarn("Not ready model ...")

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        if not random_state_maker.random_state_make():
            rospy.logwarn("Failed to move object !!")
        rate.sleep()

if __name__ == '__main__':
    main()
