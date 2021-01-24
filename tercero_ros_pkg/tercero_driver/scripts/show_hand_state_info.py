#!/usr/bin/env python

import rospy

from std_msgs.msg import UInt16MultiArray

from time import sleep

ZERO_TORQUE = 0
REQUIRED_MIN_TORQUE = 10

STATE_MIN_DATA_SIZE = 24
STATE_CHACK_READY_CH = 0
STATE_CHACK_MOVE_CH = 1
STATE_CHACK_POSITION_CH = 2
STATE_CHACK_TORQUE_CH = 6
STATE_PUSHER_READY_CH = 16
STATE_PUSHER_MOVE_CH = 17
STATE_PUSHER_POSITION_CH = 18
STATE_PUSHER_TORQUE_CH = 22

STATE_NOT_ORIGIN = 0
STATE_MOVE_OPEN = 1
STATE_MOVE_CLOSE = 2
STATE_MOVE_UP = 1
STATE_MOVE_DOWN = 2


class Color:
    BLACK = '\033[30m'
    RED = '\033[31m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    BLUE = '\033[34m'
    PURPLE = '\033[35m'
    CYAN = '\033[36m'
    WHITE = '\033[37m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    END = '\033[0m'
    BOLD = '\033[1m'
    ACCENT = '\033[01m'
    UNDERLINE = '\033[4m'
    INVISIBLE = '\033[08m'
    REVERCE = '\033[07m'


class ShowHandStateInfo(object):
    def __init__(self):
        self.hand_sub_ = rospy.Subscriber(
            'hand_state', UInt16MultiArray, self.receive_hand_state, queue_size=1)
        self.hand_state_ = []
        self.chack_ready_ = 0
        self.chack_state_ = 0
        self.chack_position_ = 0
        self.chack_torque_ = 0
        self.pusher_ready_ = 0
        self.pusher_state_ = 0
        self.pusher_position_ = 0
        self.pusher_torque_ = 0
        self.color = Color()

    def receive_hand_state(self, msg):
        self.hand_state_ = msg.data

    def update_state(self):
        if (len(self.hand_state_) > STATE_MIN_DATA_SIZE):
            self.chack_ready_ = self.hand_state_[STATE_CHACK_READY_CH]
            self.chack_state_ = self.hand_state_[STATE_CHACK_MOVE_CH]
            self.chack_position_ = self.hand_state_[STATE_CHACK_POSITION_CH]
            self.chack_torque_ = self.hand_state_[STATE_CHACK_TORQUE_CH]
            self.pusher_ready_ = self.hand_state_[STATE_PUSHER_READY_CH]
            self.pusher_state_ = self.hand_state_[STATE_PUSHER_MOVE_CH]
            self.pusher_position_ = self.hand_state_[STATE_PUSHER_POSITION_CH]
            self.pusher_torque_ = self.hand_state_[STATE_PUSHER_TORQUE_CH]
        else:
            rospy.logwarn("Not Receive hand state !!")

    def is_ready(self, ready):
        if (ready):
            return self.color.GREEN + "OK" + self.color.END
        else:
            return self.color.ACCENT + self.color.FAIL + "Error" + self.color.END

    def get_chack_state(self, chack_state):
        if (chack_state == STATE_NOT_ORIGIN):
            return self.color.ACCENT + self.color.FAIL + "not origin position" + self.color.END
        elif (chack_state == STATE_MOVE_OPEN):
            return "open"
        elif (chack_state == STATE_MOVE_CLOSE):
            return "close"
        else:
            return self.color.ACCENT + self.color.FAIL + "Invalid number" + self.color.END

    def get_pusher_state(self, pusher_state):
        if (pusher_state == STATE_NOT_ORIGIN):
            return self.color.ACCENT + self.color.FAIL + "not origin position" + self.color.END
        elif (pusher_state == STATE_MOVE_UP):
            return "up"
        elif (pusher_state == STATE_MOVE_DOWN):
            return "down"
        else:
            return self.color.ACCENT + self.color.FAIL + "Invalid number" + self.color.END

    def get_position(self, position):
        return position / 10.0

    def show_hand_state(self):
        self.update_state()

        print("")
        print(self.color.GREEN +
              "================== Tercero Hand State ==================" + self.color.END)
        print("")
        now = rospy.get_rostime()
        print("Receive Time: " + str(now.secs) +
              " secs, " + str(now.nsecs) + " nsecs")
        print("")
        print(self.color.BOLD + "Chack: " + self.color.END)
        print("Move Ready: " + self.is_ready(self.chack_ready_) +
              " (" + str(self.chack_ready_) + ")")
        print("State: " + self.get_chack_state(self.chack_state_) +
              " (" + str(self.chack_state_) + ")")
        print("Position: " + str(self.get_position(self.chack_position_)
                                 ) + " mm (" + str(self.chack_position_) + ")")
        if (self.chack_torque_ == ZERO_TORQUE):
            print("Torque: " + self.color.ACCENT + self.color.FAIL + str(self.chack_torque_) +
                  self.color.END + "% (" + str(self.chack_torque_) + ")" + self.color.FAIL + " : [ERROR] Set chack torque !!" + self.color.END)
        elif (self.chack_torque_ < REQUIRED_MIN_TORQUE):
            print("Torque: " + self.color.YELLOW + str(self.chack_torque_) +
                  self.color.END + "% (" + str(self.chack_torque_) + ")" + self.color.YELLOW + " : [WARNING] Chack torque is too low !!" + self.color.END)
        else:
            print("Torque: " + str(self.chack_torque_) +
                  "% (" + str(self.chack_torque_) + ")")
        print("")
        print(self.color.BOLD + "Pusher: " + self.color.END)
        print("Move Ready: " + self.is_ready(self.pusher_ready_) +
              " (" + str(self.pusher_ready_) + ")")
        print("State: " + self.get_pusher_state(self.pusher_state_) +
              " (" + str(self.pusher_state_) + ")")
        print("Position: " + str(self.get_position(self.pusher_position_)
                                 ) + " mm (" + str(self.pusher_position_) + ")")
        if (self.pusher_torque_ == ZERO_TORQUE):
            print("Torque: " + self.color.ACCENT + self.color.FAIL + str(self.pusher_torque_) +
                  self.color.END + "% (" + str(self.pusher_torque_) + ")" + self.color.FAIL + " : [ERROR] Set pusher torque !!" + self.color.END)
        elif (self.pusher_torque_ < REQUIRED_MIN_TORQUE):
            print("Torque: " + self.color.YELLOW + str(self.pusher_torque_) +
                  self.color.END + "% (" + str(self.pusher_torque_) + ")" + self.color.YELLOW + " : [WARNING] Pusher torque is too low !!" + self.color.END)
        else:
            print("Torque: " + str(self.pusher_torque_) +
                  "% (" + str(self.pusher_torque_) + ")")
        print("")
        print(self.color.GREEN +
              "========================================================" + self.color.END)
        print("")


if __name__ == '__main__':
    rospy.init_node("show_hand_state_info_node")

    while (not rospy.get_param("/set_default_torque_node/finish_initialize", False)):
        continue

    sleep(1)
    show_hand_state_info = ShowHandStateInfo()

    rospy.set_param("/set_default_torque_node/finish_initialize", False)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            show_hand_state_info.show_hand_state()
            rate.sleep()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting Down show_hand_state_info_node.")
