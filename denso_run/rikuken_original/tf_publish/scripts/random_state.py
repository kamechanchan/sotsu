#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import *
import tf2_ros
from time import *
import random 
import tf
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose


if __name__=='__main__':
    rospy.init_node('random_state')
    pose_data =ModelState()
    pose_data.model_name = rospy.get_param('~object_name', 'HV8_0')
    model_state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
    various_pose = Pose()
    late = rospy.Rate(10)
    time_start = time()
    record_ok = rospy.get_param("/HV8/record_cloud/is_ok", False)
    while not rospy.is_shutdown():
        time_end = time()
        #if time_end - time_start > 1.0:
        ls = input()
        various_pose.position.x = random.uniform(-0.4, 0.4)
        various_pose.position.y = random.uniform(-0.4, 0.4)
        various_pose.position.z = random.uniform(0.01, 0.2)
        roll = random.uniform(-0.5, 0.5)
        pitch = random.uniform(-0.5, 0.5)
        yaw = random.uniform(-3.14, 3.14)
        quat = quaternion_from_euler(roll, pitch, yaw)
        various_pose.orientation.x = quat[0]
        various_pose.orientation.y = quat[1]
        various_pose.orientation.z = quat[2]
        various_pose.orientation.w = quat[3]
        pose_data.pose = various_pose
        model_state_pub.publish(pose_data)
        late.sleep()
        pose_data.pose = various_pose
        model_state_pub.publish(pose_data)
        time_start = time()
            
        pose_data.pose = various_pose
        rospy.set_param("/HV8/receive_cloud/is_ok", True)
        model_state_pub.publish(pose_data)
        late.sleep()




    