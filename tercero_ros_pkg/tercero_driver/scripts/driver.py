#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt16MultiArray

from time import sleep

MIN_TORQUE = 0
MAX_TORQUE = 100

STATE_CHACK_READY_CH = 0
STATE_CHACK_MOVE_CH = 1
STATE_CHACK_POSITION_CH = 2
STATE_CHACK_TORQUE_CH = 6
STATE_PUSHER_READY_CH = 16
STATE_PUSHER_MOVE_CH = 17
STATE_PUSHER_POSITION_CH = 18
STATE_PUSHER_TORQUE_CH = 22

STATE_READY_ERROR = 0
STATE_READY_OK = 1
STATE_NOT_ORIGIN = 0
STATE_MOVE_OPEN = 1
STATE_MOVE_CLOSE = 2
STATE_MOVE_UP = 1
STATE_MOVE_DOWN = 2

ORDER_DATA_SIZE = 13
ORDER_CHACK_ACTION_CH = 0
ORDER_CHACK_TORQUE_CH = 1
ORDER_PUSHER_ACTION_CH = 10
ORDER_PUSHER_TORQUE_CH = 11

ORDER_PULSE_STOP = 0
ORDER_ACTION_OPEN = 1
ORDER_ACTION_CLOSE = 2
ORDER_ACTION_UP = 1
ORDER_ACTION_DOWN = 2


class Tercero(object):
    def __init__(self):
        self.hand_pub_ = rospy.Publisher(
            "hand_order", UInt16MultiArray, queue_size=1)
        self.hand_sub_ = rospy.Subscriber(
            "hand_state", UInt16MultiArray, self.receive_state, queue_size=1)
        self.hand_order_ = [0] * ORDER_DATA_SIZE
        self.hand_state_ = []
        self.grasp_time = 1

    def receive_state(self, msg):
        self.hand_state_ = msg.data

    def is_chack_ok(self):
        if self.hand_state_[STATE_CHACK_READY_CH] == STATE_READY_ERROR:
            return False
        elif self.hand_state_[STATE_CHACK_READY_CH] == STATE_READY_OK:
            return True
        else:
            rospy.logerr("Invalid Number")
            return False

    def is_pusher_ok(self):
        if self.hand_state_[STATE_PUSHER_READY_CH] == STATE_READY_ERROR:
            return False
        elif self.hand_state_[STATE_PUSHER_READY_CH] == STATE_READY_OK:
            return True
        else:
            rospy.logerr("Invalid Number")
            return False

    def get_chack_state(self):
        if self.hand_state_[STATE_CHACK_MOVE_CH] == STATE_NOT_ORIGIN:
            return "Not Origin Position"
        elif self.hand_state_[STATE_CHACK_MOVE_CH] == STATE_MOVE_OPEN:
            return "open"
        elif self.hand_state_[STATE_CHACK_MOVE_CH] == STATE_MOVE_CLOSE:
            return "close"
        else:
            return "Invalid Number"

    def get_pusher_state(self):
        if self.hand_state_[STATE_PUSHER_MOVE_CH] == STATE_NOT_ORIGIN:
            return "Not Origin Position"
        elif self.hand_state_[STATE_PUSHER_MOVE_CH] == STATE_MOVE_UP:
            return "up"
        elif self.hand_state_[STATE_PUSHER_MOVE_CH] == STATE_MOVE_DOWN:
            return "down"
        else:
            return "Invalid Number"

    def get_chack_joint_position(self):
        return self.hand_state_[STATE_CHACK_POSITION_CH] / 10.0

    def get_pusher_joint_position(self):
        return self.hand_state_[STATE_PUSHER_POSITION_CH] / 10.0

    def get_chack_torque(self):
        return self.hand_state_[STATE_CHACK_TORQUE_CH]

    def get_pusher_torque(self):
        return self.hand_state_[STATE_PUSHER_TORQUE_CH]

    def open_chack(self):
        if (self.is_chack_ok()):
            msg = UInt16MultiArray()
            self.hand_order_[ORDER_CHACK_ACTION_CH] = ORDER_ACTION_OPEN
            msg.data = self.hand_order_
            self.hand_pub_.publish(msg)
            sleep(self.grasp_time)
            self.hand_order_[ORDER_CHACK_ACTION_CH] = ORDER_PULSE_STOP
            self.hand_pub_.publish(msg)
            return True
        else:
            rospy.logerr("Error chack state")
            return False

    def close_chack(self):
        if (self.is_chack_ok()):
            msg = UInt16MultiArray()
            self.hand_order_[ORDER_CHACK_ACTION_CH] = ORDER_ACTION_CLOSE
            msg.data = self.hand_order_
            self.hand_pub_.publish(msg)
            sleep(self.grasp_time)
            self.hand_order_[ORDER_CHACK_ACTION_CH] = ORDER_PULSE_STOP
            self.hand_pub_.publish(msg)
            return True
        else:
            rospy.logerr("Error chack state")
            return False

    def up_pusher(self):
        if (self.is_pusher_ok()):
            msg = UInt16MultiArray()
            self.hand_order_[ORDER_PUSHER_ACTION_CH] = ORDER_ACTION_UP
            msg.data = self.hand_order_
            self.hand_pub_.publish(msg)
            sleep(self.grasp_time)
            self.hand_order_[ORDER_PUSHER_ACTION_CH] = ORDER_PULSE_STOP
            self.hand_pub_.publish(msg)
            return True
        else:
            rospy.logerr("Error pusher state")
            return False

    def down_pusher(self):
        if (self.is_pusher_ok()):
            msg = UInt16MultiArray()
            self.hand_order_[ORDER_PUSHER_ACTION_CH] = ORDER_ACTION_DOWN
            msg.data = self.hand_order_
            self.hand_pub_.publish(msg)
            sleep(self.grasp_time)
            self.hand_order_[ORDER_PUSHER_ACTION_CH] = ORDER_PULSE_STOP
            self.hand_pub_.publish(msg)
            return True
        else:
            rospy.logerr("Error pusher state")
            return False

    def set_chack_torque(self, torque):
        msg = UInt16MultiArray()
        if torque < MIN_TORQUE or torque > MAX_TORQUE:
            rospy.logerr("torque from " + str(MIN_TORQUE) +
                         " to " + str(MAX_TORQUE) + " !!")
            return False

        if (self.is_chack_ok()):
            self.hand_order_[ORDER_CHACK_TORQUE_CH] = torque
            msg.data = self.hand_order_
            self.hand_pub_.publish(msg)
            return True
        else:
            rospy.logerr("Error chack state")
            return False

    def set_pusher_torque(self, torque):
        msg = UInt16MultiArray()
        if torque < MIN_TORQUE or torque > MAX_TORQUE:
            rospy.logerr("torque from " + str(MIN_TORQUE) +
                         " to " + str(MAX_TORQUE) + " !!")
            return False

        if (self.is_pusher_ok()):
            self.hand_order_[ORDER_PUSHER_TORQUE_CH] = torque
            msg.data = self.hand_order_
            self.hand_pub_.publish(msg)
            return True
        else:
            rospy.logerr("Error pusher state")
            return False
