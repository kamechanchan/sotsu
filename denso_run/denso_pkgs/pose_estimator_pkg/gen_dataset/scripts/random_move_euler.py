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
from pose_estimator_srvs.srv import PoseEstimate
from pose_estimator_srvs.srv import range1, range1Request, range1Response
gen = range1Request()
gen.pi

gen_dataset.srv.
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

    def parameter_make(self, req):
        self.xsmall = req.xsmall
        self.xlarge = req.xlarge
        self.ysmall = req.ysmall
        self.ylarge = req.ylarge
        self.zsmall = req.zsmall
        self.zlarge = req.zlarge
        self.rollsmall = req.rollsmall
        self.rolllarge = req.rolllarge
        self.pitchsmall = req.pitchsmall
        self.pitchlarge = req.pitchlarge
        self.yawsmall = req.yawsmall
        self.yawlarge = req.yawlarge
        
    def init_state_make(self):

        self.pos_.pose.position.x = random.uniform(-0.2, 0.2)
        self.pos_.pose.position.y = random.uniform(-0.2, 0.2)
        self.pos_.pose.position.z = random.uniform(0.1, 0.25)


        
        roll = random.uniform(-pi, pi)
        pitch = random.uniform(-pi, pi)
        yaw = random.uniform(-pi, pi)

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
            self.pos_.pose.position.x = random.uniform(-0.2, 0.2)
            self.pos_.pose.position.y = random.uniform(-0.2, 0.2)
            self.pos_.pose.position.z = random.uniform(0.1, 0.25)

            roll = random.uniform(-pi/4, pi/4)
            pitch = random.uniform(-pi/4, pi/4)
            yaw = random.uniform(-pi/4, pi/4)

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
    s = rospy.Service('range_decision', range1, random_state_maker.parameter_make)

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
