#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

from time import sleep

#####################################################################
# NOTE:This program assumes send format is 288(HandI/O and MiniI/O) #
#      and recv format 290(HandI/O and MiniI/O)                     #
#####################################################################

class MHand(object):
    def __init__(self, torque=10, grasp_time=1):
        self.robot_name = rospy.get_param("/robot_name", default="/vs087")
        self._hand_io_status = 0
        self._mini_io_status = 0
        self._hand_io_sub = rospy.Subscriber(self.robot_name+"/Read_HandIO", Int32, self._read_hand_io_callback, queue_size=1)
        self._mini_io_sub = rospy.Subscriber(self.robot_name+"/Read_MiniIO", Int32, self._read_mini_io_callback, queue_size=1)
        self._hand_io_pub = rospy.Publisher(self.robot_name+"/Write_HandIO", Int32, queue_size=1)
        self._mini_io_pub = rospy.Publisher(self.robot_name+"/Write_MiniIO", Int32, queue_size=1)
        self.grasp_time = grasp_time
        print('Ready to control MHand')

    def _read_hand_io_callback(self, msg):
        self._hand_io_status = msg.data

    def _read_mini_io_callback(self, msg):
        self._mini_io_status = msg.data

    def is_hand_open(self):
        if (self._hand_io_status & 0x00000002):
            return True
        else:
            return False

    def is_hand_close(self):
        if (self._hand_io_status & 0x00000004):
            return True
        else:
            return False

    def open_hand(self):
        msg = Int32()
        msg.data = self._hand_io_status | 0x00010000
        self._hand_io_pub.publish(msg)
        sleep(self.grasp_time)
        msg.data = self._hand_io_status & 0xfffeffff
        self._hand_io_pub.publish(msg)

    def close_hand(self):
        msg = Int32()
        msg.data = self._hand_io_status | 0x00020000
        self._hand_io_pub.publish(msg)
        sleep(self.grasp_time)
        msg.data = self._hand_io_status & 0xfffdffff
        self._hand_io_pub.publish(msg)

    def get_torque(self):
        return self._mini_io_status >> 25

    def set_torque(self, torque):
        msg = Int32()
        if torque >= 50:
            torque = 50
        msg.data = (torque << 25)
        self._mini_io_pub.publish(msg)
