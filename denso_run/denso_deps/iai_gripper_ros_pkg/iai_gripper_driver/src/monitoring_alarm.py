#!/usr/bin/env python

import rospy

from std_msgs.msg import Int32

class IAIGripperAlarmMonitor(object):
    def __init__(self):
        self.robot_name = rospy.get_param("/robot_name", default="/vs087")
        self._hand_io_sub = rospy.Subscriber(self.robot_name+"/Read_HandIO", Int32, self.monitoring_alarm, queue_size=1)

    def monitoring_alarm(self, msg):
        if (msg.data & 0x00000020) is not 0x00000020:
            rospy.logerr('iai_gripper alarm signal detected')
        if (msg.data & 0x00000010) is not 0x00000010:
            rospy.logerr('iai_gripper emergency stop signal detected')

if __name__ == '__main__':
    rospy.init_node('iai_gripper_alarm_monitor')
    alarm_monitor = IAIGripperAlarmMonitor()
    rospy.spin()

