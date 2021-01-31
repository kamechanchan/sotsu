#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

from time import sleep

#####################################################################
# NOTE:This program assumes send format is 288(HandI/O and MiniI/O) #
#      and recv format 290(HandI/O and MiniI/O)                     #
#####################################################################

class IAIGripper(object):
    def __init__(self, torque=10, sleep_time=0.1):
        self.robot_name = rospy.get_param("/robot_name", default="/vs087")
        self._gripper_move_pos = 4
        self._hand_io_status = 0
        self._mini_io_status = 0
        self._hand_io_sub = rospy.Subscriber(self.robot_name+"/Read_HandIO", Int32, self._read_hand_io_callback, queue_size=1)
        self._mini_io_sub = rospy.Subscriber(self.robot_name+"/Read_MiniIO", Int32, self._read_mini_io_callback, queue_size=1)
        self._hand_io_pub = rospy.Publisher(self.robot_name+"/Write_HandIO", Int32, queue_size=1)
        self._mini_io_pub = rospy.Publisher(self.robot_name+"/Write_MiniIO", Int32, queue_size=1)
        self.sleep_time = sleep_time
        print('Ready to control IAIgripper')

    def _read_hand_io_callback(self, msg):
        self._hand_io_status = msg.data

    def _read_mini_io_callback(self, msg):
        self._mini_io_status = msg.data

    def servo(self, servo_on):
        msg = Int32()
        if servo_on is True:
            msg.data = self._hand_io_status | 0x00100000
            self._hand_io_pub.publish(msg)

        if servo_on is False:
            msg.data = self._hand_io_status & 0xffefffff
            self._hand_io_pub.publish(msg)

    def pause(self, pause):
        msg = Int32()
        if pause is True:
            msg.data = self._hand_io_status & 0xfffdffff
            self._hand_io_pub.publish(msg)

        if pause is False:
            msg.data = self._hand_io_status | 0x00020000
            self._hand_io_pub.publish(msg)

    def move(self, position):
        # set position
        msg = Int32()
        if position < 0:
            position = 0
        if position > self._gripper_move_pos:
            position = self._gripper_move_pos
        msg.data = (position << 25)
        self._mini_io_pub.publish(msg)

        sleep(self.sleep_time)

        # move gripper
        msg.data = self._hand_io_status | 0x00040000
        self._hand_io_pub.publish(msg)
        sleep(self.sleep_time)
        msg.data = self._hand_io_status & 0xfffbffff
        self._hand_io_pub.publish(msg)

        # wait for moving
        while (self._hand_io_status & 0x00000004) == 0:
            pass

    def return_origin(self):
        msg = Int32()
        msg.data = self._hand_io_status | 0x00010000
        self._hand_io_pub.publish(msg)
        sleep(self.sleep_time)
        # wait for moving
        while (self._hand_io_status & 0x00000002) == 0:
            pass
        msg.data = self._hand_io_status & 0xfffeffff
        self._hand_io_pub.publish(msg)

    def get_position(self):
        return (self._mini_io_status & 0x0000fc00) >> 10

    def reset(self):
        msg = Int32()
        msg.data = self._hand_io_status | 0x00080000
        self._hand_io_pub.publish(msg)
        sleep(self.sleep_time)
        msg.data = self._hand_io_status & 0xfff7ffff
        self._hand_io_pub.publish(msg)
        sleep(self.sleep_time)
        msg.data = self._hand_io_status | 0x00100000
        self._hand_io_pub.publish(msg)
        sleep(self.sleep_time)
        msg.data = self._hand_io_status | 0x00020000
        self._hand_io_pub.publish(msg)
        return True

