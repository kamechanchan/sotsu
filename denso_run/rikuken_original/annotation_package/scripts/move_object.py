#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import *
import random
from rospy.timer import Rate
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose

class annotation_environment(object):
    def __init__(self):
        cont = 0
        self.model_state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
        self.execute()
            
    def object_move(self, name):
        loop = rospy.Rate(20)
        com = 0
        while com < 1:
            com = com + 1
            pose_list = []
            for i in range(7):
                pose_data = ModelState()
                pose_data.model_name =  name + '_' + str(i)
                pose_data.pose.position.x = random.uniform(-0.12, 0.12)
                pose_data.pose.position.y = random.uniform(-0.12, 0.12)
                pose_data.pose.position.z = i*0.1
                roll = random.uniform(-0.5, 0.5)
                pitch = random.uniform(-0.5, 0.5)
                yaw = random.uniform(-3.14, 3.14)
                quat = quaternion_from_euler(roll, pitch, yaw)
                pose_data.pose.orientation.x = quat[0]
                pose_data.pose.orientation.y = quat[1]
                pose_data.pose.orientation.z = quat[2]
                pose_data.pose.orientation.w = quat[3]
                print(pose_data.model_name)
                pose_list.append(pose_data)
            for i in range(7):
                self.model_state_pub.publish(pose_list[i])
                loop.sleep()
            loop.sleep()
    
    def execute(self):
        loop = rospy.Rate(1)
        while not rospy.is_shutdown():
            hantei = rospy.get_param("/is_move/ok", True)
            if hantei:
                self.object_move("HV8")
                rospy.set_param("/is_record/ok", True)
                rospy.set_param("/is_move/ok", False)
            

if __name__=='__main__':
    rospy.init_node('random_state')
    annotation_environment()
        
    


