
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
        loop = rospy.Rate(40)
        com = 0
        pose_list = []
        for i in range(20):
                pose_data = ModelState()
                pose_data.model_name =  name + '_' + str(i)
                if i == 0:
                    pose_data.pose.position.x = random.uniform(-0.22, -0.18)
                    pose_data.pose.position.y = random.uniform(-0.22, -0.18)
                    pose_data.pose.position.z = 0.06
                elif i == 1:
                    pose_data.pose.position.x = random.uniform(-0.12, -0.10)
                    pose_data.pose.position.y = random.uniform(0.06, 0.1)
                    pose_data.pose.position.z = 0.2
                elif i == 2:
                    pose_data.pose.position.x = random.uniform(-0.12, -0.10)
                    pose_data.pose.position.y = random.uniform(-0.26, -0.22)
                    pose_data.pose.position.z = 0.3
                elif i == 3:
                    pose_data.pose.position.x = random.uniform(-0.18, -0.22)
                    pose_data.pose.position.y = random.uniform(-0.12, -0.16)
                    pose_data.pose.position.z = 0.1
                elif i == 4:
                    pose_data.pose.position.x = random.uniform(0.02, 0.06)
                    pose_data.pose.position.y = random.uniform(0.12, 0.16)
                    pose_data.pose.position.z = 0.3
                elif i == 5:
                    pose_data.pose.position.x = random.uniform(0.26, 0.20)
                    pose_data.pose.position.y = random.uniform(0.06, 0.10)
                    pose_data.pose.position.z = 0.4
                elif i == 6:
                    pose_data.pose.position.x = random.uniform(0.23, 0.27)
                    pose_data.pose.position.y = random.uniform(-0.10, -0.06)
                    pose_data.pose.position.z = 0.24
                elif i == 7:
                    pose_data.pose.position.x = random.uniform(-0.2, -0.16)
                    pose_data.pose.position.y = random.uniform(-0.10, -0.06)
                    pose_data.pose.position.z = 0.09
                elif i == 8:
                    pose_data.pose.position.x = random.uniform(-0.25, -0.30)
                    pose_data.pose.position.y = random.uniform(0.12, 0.16)
                    pose_data.pose.position.z = 0.12
                elif i == 9:
                    pose_data.pose.position.x = random.uniform(-0.12, -0.08)
                    pose_data.pose.position.y = random.uniform(0.26, 0.22)
                    pose_data.pose.position.z = 0.20
                elif i == 10:
                    pose_data.pose.position.x = random.uniform(-0.08, -0.04)
                    pose_data.pose.position.y = random.uniform(-0.22, -0.18)
                    pose_data.pose.position.z = 0.05
                elif i == 11:
                    pose_data.pose.position.x = random.uniform(-0.04, 0.00)
                    pose_data.pose.position.y = random.uniform(-0.10, -0.06)
                    pose_data.pose.position.z = 0.15
                elif i == 12:
                    pose_data.pose.position.x = random.uniform(0.20, 0.16)
                    pose_data.pose.position.y = random.uniform(-0.12, -0.06)
                    pose_data.pose.position.z = 0.05
                elif i == 13:
                    pose_data.pose.position.x = random.uniform(0.04, 0.08)
                    pose_data.pose.position.y = random.uniform(-0.22, -0.18)
                    pose_data.pose.position.z = 0.18
                elif i == 14:
                    pose_data.pose.position.x = random.uniform(0.08, 0.12)
                    pose_data.pose.position.y = random.uniform(-0.06, -0.02)
                    pose_data.pose.position.z = 0.14
                elif i == 15:
                    pose_data.pose.position.x = random.uniform(0.12, 0.16)
                    pose_data.pose.position.y = random.uniform(0.02, 0.06)
                    pose_data.pose.position.z = 0.15
                elif i == 16:
                    pose_data.pose.position.x = random.uniform(0.16, 0.20)
                    pose_data.pose.position.y = random.uniform(0.06, 0.10)
                    pose_data.pose.position.z = 0.22
                elif i == 17:
                    pose_data.pose.position.x = random.uniform(-0.08, -0.04)
                    pose_data.pose.position.y = random.uniform(0.22, 0.26)
                    pose_data.pose.position.z = 0.30
                elif i == 18:
                    pose_data.pose.position.x = random.uniform(-0.25, -0.20)
                    pose_data.pose.position.y = random.uniform(-0.28, -0.24)
                    pose_data.pose.position.z = 0.06
                elif i == 19:
                    pose_data.pose.position.x = random.uniform(-0.12, -0.08)
                    pose_data.pose.position.y = random.uniform(0.06, 0.010)
                    pose_data.pose.position.z = 0.13



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
        while com < 2:
            com = com + 1
            
            
            for i in range(20):
                self.model_state_pub.publish(pose_list[i])
                loop.sleep()
            loop.sleep()
    
    def execute(self):
        loop = rospy.Rate(1)
        # while not rospy.is_shutdown():
        #     hantei = rospy.get_param("/is_move/ok", True)
        #     if hantei:
        #         self.object_move("HV8")
        #         rospy.set_param("/is_record/ok", True)
        #         rospy.set_param("/is_move/ok", False)
        self.object_move("HV8")
            

if __name__=='__main__':
    rospy.init_node('random_state')
    annotation_environment()