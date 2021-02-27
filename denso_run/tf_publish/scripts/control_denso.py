#!/usr/bin/env python3
# license removed for brevity
import rospy
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def talker():
    rospy.init_node('joint_trajectory_publisher', anonymous=True)
    pub = rospy.Publisher('/vs087/arm_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(0.5)
    msg = JointTrajectory()
    msg.header.stamp = rospy.Time.now()
    msg.joint_names = [ "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" ]
    msg.points = [JointTrajectoryPoint() for i in range(1)]
    msg.points[0].positions = [math.pi / 2, 0.26, math.pi / 4, 0, 0, 0];
    msg.points[0].time_from_start = rospy.Time(1.0)
    
    pub.publish(msg)
    #rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass