#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import *
import random
from rospy.timer import Rate
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose
from estimator.srv import six_input
from estimator.srv import six_inputResponse


class annotation_environment(object):
    def __init__(self):
        cont = 0
        self.model_state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
        self.kaisuu = 0
        self.service_name = rospy.get_param("~service_name", "six_input")
        self.object_name = rospy.get_param("~object_name", "HV8")
        
        self.z_coordinamte = 0
        self.first_function()
        print("nazeda")

        rospy.Service(self.service_name, six_input, self.callback)
        rospy.spin()


    def first_function(self):
        self.model_state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
        self.box_move()
    
    def box_move(self):
        print("itteruhazu")
        pose_list = []
        loop = rospy.Rate(10)
        pose_data = ModelState()
        pose_data.model_name = "object_box"
        pose_data.pose.position.x = 0
        pose_data.pose.position.y = 0
        pose_data.pose.position.z = self.z_coordinamte
        roll = 0
        pitch = 0
        yaw = 0
        quat = quaternion_from_euler(roll, pitch, yaw)
        pose_data.pose.orientation.x = quat[0]
        pose_data.pose.orientation.y = quat[1]
        pose_data.pose.orientation.z = quat[2]
        pose_data.pose.orientation.w = quat[3]
        pose_list.append(pose_data)
        for i in range(6):
            self.model_state_pub.publish(pose_list[0])
            loop.sleep()

    def callback(self, req):
        self.object_move(self.object_name)
        self.delay.sleep()
        res = six_inputResponse()
        res.content = True
        print("one_sleep")
        self.delay.sleep()
        self.delay.sleep()

        # print("third_sleep")
        # self.delay.sleep()
        # print("fou_sleeep")
        # self.delay.sleep()
        # self.delay.sleep()
        # self.delay.sleep()
        # self.delay.sleep()
        # self.delay.sleep()
        # self.delay.sleep()
        # self.delay.sleep()
        return res
    

    def object_move(self, name):
        loop = rospy.Rate(100)
        self.delay = rospy.Rate(1)
        com = 0
        pose_list = []
        a_1 = 0.14
        a_2 = 0.07
        a_3 = 0
        a_4 = -0.07
        a_5 = -0.14
        for i in range(25):
                pose_data = ModelState()
                pose_data.model_name =  name + '_' + str(i)
                if i == 0:
                    # pose_data.pose.position.x = random.uniform(a_1, a_5)
                    # pose_data.pose.position.y = random.uniform(a_1, a_5)
                    pose_data.pose.position.z = random.uniform(0.1, 0.2)
                    pose_data.pose.position.x = a_1
                    pose_data.pose.position.y = a_1
                    # pose_data.pose.position.z = 0.00
                elif i == 1:
                    # pose_data.pose.position.x = random.uniform(a_1, a_5)
                    # pose_data.pose.position.y = random.uniform(a_1, a_5)
                    pose_data.pose.position.z = random.uniform(0.1, 0.2)
                    # pose_data.pose.position.x = random.uniform(-0.02, -0.06)
                    # pose_data.pose.position.y = random.uniform(0.06, 0.1)
                    # pose_data.pose.position.z = 0.4
                    pose_data.pose.position.x = a_1
                    pose_data.pose.position.y = a_2
                    # pose_data.pose.position.z = 0.4
                elif i == 2:
                    # pose_data.pose.position.x = random.uniform(a_1, a_5)
                    # pose_data.pose.position.y = random.uniform(a_1, a_5)
                    pose_data.pose.position.z = random.uniform(0.1, 0.16)
                    # pose_data.pose.position.x = random.uniform(-0.02, 0.02)
                    # pose_data.pose.position.y = random.uniform(-0.06, -0.02)
                    # pose_data.pose.position.z = 0.55
                    pose_data.pose.position.x = a_1
                    pose_data.pose.position.y = a_3
                    # pose_data.pose.position.z = 0.45
                elif i == 3:
                    # pose_data.pose.position.x = random.uniform(a_1, a_5)
                    # pose_data.pose.position.y = random.uniform(a_1, a_5)
                    pose_data.pose.position.z = random.uniform(0.1, 0.2)
                    # pose_data.pose.position.x = random.uniform(-0.04, -0.08)
                    # pose_data.pose.position.y = random.uniform(-0.02, -0.06)
                    # pose_data.pose.position.z = 0.25
                    pose_data.pose.position.x = a_1
                    pose_data.pose.position.y = a_4
                    # pose_data.pose.position.z = 0.25
                elif i == 4:
                    # pose_data.pose.position.x = random.uniform(a_1, a_5)
                    # pose_data.pose.position.y = random.uniform(a_1, a_5)
                    pose_data.pose.position.z = random.uniform(0.1, 0.2)
                    # pose_data.pose.position.x = random.uniform(0.02, 0.06)
                    # pose_data.pose.position.y = random.uniform(0.03, 0.07)
                    # pose_data.pose.position.z = 0.3
                    pose_data.pose.position.x = a_1
                    pose_data.pose.position.y = a_5
                    # pose_data.pose.position.z = 0.3
                elif i == 5:
                    # pose_data.pose.position.x = random.uniform(a_1, a_5)
                    # pose_data.pose.position.y = random.uniform(a_1, a_5)
                    pose_data.pose.position.z = random.uniform(0.1, 0.2)
                    # pose_data.pose.position.x = random.uniform(0.00, 0.04)
                    # pose_data.pose.position.y = random.uniform(0.00, 0.04)
                    # pose_data.pose.position.z = 0.05
                    pose_data.pose.position.x = a_2
                    pose_data.pose.position.y = a_1
                    # pose_data.pose.position.z = 0.05
                elif i == 6:
                    # pose_data.pose.position.x = random.uniform(a_1, a_5)
                    # pose_data.pose.position.y = random.uniform(a_1, a_5)
                    pose_data.pose.position.z = random.uniform(0.08, 0.13)
                    # pose_data.pose.position.x = random.uniform(-0.01, -0.05)
                    # pose_data.pose.position.y = random.uniform(-0.02, -0.06)
                    # pose_data.pose.position.z = 0.25
                    pose_data.pose.position.x = a_2
                    pose_data.pose.position.y = a_2
                    # pose_data.pose.position.z = 0.25
                elif i == 7:
                    # pose_data.pose.position.x = random.uniform(a_1, a_5)
                    # pose_data.pose.position.y = random.uniform(a_1, a_5)
                    pose_data.pose.position.z = random.uniform(0.15, 0.2)
                    # pose_data.pose.position.x = random.uniform(-0.02, -0.06)
                    # pose_data.pose.position.y = random.uniform(-0.02, -0.06)
                    # pose_data.pose.position.z = 0.53
                    pose_data.pose.position.x = a_2
                    pose_data.pose.position.y = a_3
                    # pose_data.pose.position.z = 0.53
                elif i == 8:
                    # pose_data.pose.position.x = random.uniform(a_1, a_5)
                    # pose_data.pose.position.y = random.uniform(a_1, a_5)
                    pose_data.pose.position.z = random.uniform(0.15, 0.2)
                    # pose_data.pose.position.x = random.uniform(0.00, 0.04)
                    # pose_data.pose.position.y = random.uniform(0.04, 0.08)
                    # pose_data.pose.position.z = 0.15
                    pose_data.pose.position.x = a_2
                    pose_data.pose.position.y = a_4 
                    # pose_data.pose.position.z = 0.15
                elif i == 9:
                    # pose_data.pose.position.x = random.uniform(a_1, a_5)
                    # pose_data.pose.position.y = random.uniform(a_1, a_5)
                    pose_data.pose.position.z = random.uniform(0.1, 0.16)
                    # pose_data.pose.position.x = random.uniform(-0.08, -0.04)
                    # pose_data.pose.position.y = random.uniform(-0.02, -0.06)
                    # pose_data.pose.position.z = 0.46
                    pose_data.pose.position.x = a_2
                    pose_data.pose.position.y = a_5
                    # pose_data.pose.position.z = 0.46
                elif i == 10:
                    # pose_data.pose.position.x = random.uniform(a_1, a_5)
                    # pose_data.pose.position.y = random.uniform(a_1, a_5)
                    pose_data.pose.position.z = random.uniform(0.15, 0.2)
                    # pose_data.pose.position.x = random.uniform(-0.04, 0.00)
                    # pose_data.pose.position.y = random.uniform(-0.02, -0.06)
                    # pose_data.pose.position.z = 0.38
                    pose_data.pose.position.x = a_3
                    pose_data.pose.position.y = a_1
                    # pose_data.pose.position.z = 0.38
                elif i == 11:
                    # pose_data.pose.position.x = random.uniform(a_1, a_5)
                    # pose_data.pose.position.y = random.uniform(a_1, a_5)
                    pose_data.pose.position.z = random.uniform(0.08, 0.14)
                    # pose_data.pose.position.x = random.uniform(-0.04, 0.00)
                    # pose_data.pose.position.y = random.uniform(-0.10, -0.06)
                    # pose_data.pose.position.z = 0.29
                    pose_data.pose.position.x = a_3
                    pose_data.pose.position.y = a_2
                    # pose_data.pose.position.z = 0.29
                elif i == 12:
                    # pose_data.pose.position.x = random.uniform(a_1, a_5)
                    # pose_data.pose.position.y = random.uniform(a_1, a_5)
                    pose_data.pose.position.z = random.uniform(0.15, 0.2)
                    # pose_data.pose.position.x = random.uniform(-0.02, 0.02)
                    # pose_data.pose.position.y = random.uniform(-0.02, -0.06)
                    # pose_data.pose.position.z = 0.43
                    pose_data.pose.position.x = a_3
                    pose_data.pose.position.y = a_3
                    # pose_data.pose.position.z = 0.43
                elif i == 13:
                    # pose_data.pose.position.x = random.uniform(a_1, a_5)
                    # pose_data.pose.position.y = random.uniform(a_1, a_5)
                    pose_data.pose.position.z = random.uniform(0.13, 0.2)
                    # pose_data.pose.position.x = random.uniform(-0.04, -0.08)
                    # pose_data.pose.position.y = random.uniform(-0.04, -0.08)
                    # pose_data.pose.position.z = 0.19
                    pose_data.pose.position.x = a_3
                    pose_data.pose.position.y = a_4
                    # pose_data.pose.position.z = 0.19
                elif i == 14:
                    # pose_data.pose.position.x = random.uniform(a_1, a_5)
                    # pose_data.pose.position.y = random.uniform(a_1, a_5)
                    pose_data.pose.position.z = random.uniform(0.08, 0.12)
                    # pose_data.pose.position.x = random.uniform(0.06, 0.10)
                    # pose_data.pose.position.y = random.uniform(-0.06, -0.02)
                    # pose_data.pose.position.z = 0.36
                    pose_data.pose.position.x = a_3
                    pose_data.pose.position.y = a_5
                    # pose_data.pose.position.z = 0.36
                elif i == 15:
                    # pose_data.pose.position.x = random.uniform(a_1, a_5)
                    # pose_data.pose.position.y = random.uniform(a_1, a_5)
                    pose_data.pose.position.z = random.uniform(0.07, 0.16)
                    # pose_data.pose.position.x = random.uniform(0.04, 0.08)
                    # pose_data.pose.position.y = random.uniform(0.02, 0.06)
                    # pose_data.pose.position.z = 0.50
                    pose_data.pose.position.x = a_4
                    pose_data.pose.position.y = a_1
                    # pose_data.pose.position.z = 0.50
                elif i == 16:
                    # pose_data.pose.position.x = random.uniform(a_1, a_5)
                    # pose_data.pose.position.y = random.uniform(a_1, a_5)
                    pose_data.pose.position.z = random.uniform(0.15, 0.2)
                    # pose_data.pose.position.x = random.uniform(-0.02, 0.02)
                    # pose_data.pose.position.y = random.uniform(0.02, 0.06)
                    # pose_data.pose.position.z = 0.42
                    pose_data.pose.position.x = a_4
                    pose_data.pose.position.y = a_2
                    # pose_data.pose.position.z = 0.42
                elif i == 17:
                    # pose_data.pose.position.x = random.uniform(a_1, a_5)
                    # pose_data.pose.position.y = random.uniform(a_1, a_5)
                    pose_data.pose.position.z = random.uniform(0.1, 0.15)
                    # pose_data.pose.position.x = random.uniform(-0.08, -0.04)
                    # pose_data.pose.position.y = random.uniform(0.02, 0.06)
                    # pose_data.pose.position.z = 0.32
                    pose_data.pose.position.x = a_4
                    pose_data.pose.position.y = a_3
                    # pose_data.pose.position.z = 0.32
                elif i == 18:
                    # pose_data.pose.position.x = random.uniform(a_1, a_5)
                    # pose_data.pose.position.y = random.uniform(a_1, a_5)
                    pose_data.pose.position.z = random.uniform(0.16, 0.2)
                    # pose_data.pose.position.x = random.uniform(-0.02, -0.06)
                    # pose_data.pose.position.y = random.uniform(-0.02, -0.06)
                    # pose_data.pose.position.z = 0.20
                    pose_data.pose.position.x = a_4
                    pose_data.pose.position.y = a_4
                    # pose_data.pose.position.z = 0.20
                elif i == 19:
                    # pose_data.pose.position.x = random.uniform(a_1, a_5)
                    # pose_data.pose.position.y = random.uniform(a_1, a_5)
                    pose_data.pose.position.z = random.uniform(0.12, 0.17)
                    # pose_data.pose.position.x = random.uniform(-0.04, -0.08)
                    # pose_data.pose.position.y = random.uniform(0.06, 0.010)
                    # pose_data.pose.position.z = 0.16
                    pose_data.pose.position.x = a_4
                    pose_data.pose.position.y = a_5
                    # pose_data.pose.position.z = 0.16
                elif i == 20:
                    pose_data.pose.position.x = a_5
                    pose_data.pose.position.y = a_1
                    # pose_data.pose.position.z = 0.06
                    # pose_data.pose.position.x = random.uniform(a_1, a_5)
                    # pose_data.pose.position.y = random.uniform(a_1, a_5)
                    pose_data.pose.position.z = random.uniform(0.13, 0.2)
                elif i == 21:
                    pose_data.pose.position.x = a_5
                    pose_data.pose.position.y = a_2
                    # pose_data.pose.position.z = 0.36
                    # pose_data.pose.position.x = random.uniform(a_1, a_5)
                    # pose_data.pose.position.y = random.uniform(a_1, a_5)
                    pose_data.pose.position.z = random.uniform(0.1, 0.14)
                elif i == 22:
                    pose_data.pose.position.x = a_5
                    pose_data.pose.position.y = a_3
                    # pose_data.pose.position.z = 0.26
                    # pose_data.pose.position.x = random.uniform(a_1, a_5)
                    # pose_data.pose.position.y = random.uniform(a_1, a_5)
                    pose_data.pose.position.z = random.uniform(0.1, 0.14)
                elif i == 23:
                    pose_data.pose.position.x = a_5
                    pose_data.pose.position.y = a_4
                    # pose_data.pose.position.z = 0.16
                    # pose_data.pose.position.x = random.uniform(a_1, a_5)
                    # pose_data.pose.position.y = random.uniform(a_1, a_5)
                    pose_data.pose.position.z = random.uniform(0.13, 0.2)
                elif i == 24:
                    pose_data.pose.position.x = a_5 
                    pose_data.pose.position.y = a_5
                    # pose_data.pose.position.z = 0.12
                    # pose_data.pose.position.x = random.uniform(a_1, a_5)
                    # pose_data.pose.position.y = random.uniform(a_1, a_5)
                    pose_data.pose.position.z = random.uniform(0.1, 0.18)



                roll = random.uniform(-1.2, 1.2)
                pitch = random.uniform(-1.2, 1.2)
                yaw = random.uniform(-0.01, 0.03)
                quat = quaternion_from_euler(roll, pitch, yaw)
                pose_data.pose.orientation.x = quat[0]
                pose_data.pose.orientation.y = quat[1]
                pose_data.pose.orientation.z = quat[2]
                pose_data.pose.orientation.w = quat[3]
                #print(pose_data.model_name)
                pose_list.append(pose_data)
        while com < 2:
            com = com + 1
            
            
            for i in range(25):
                self.model_state_pub.publish(pose_list[i])
                loop.sleep()
        
        # delay.sleep()
        
            #loop.sleep()
    
    def execute(self):
        loop = rospy.Rate(10)
        naibu_loop = rospy.Rate(1)
        #count = 0
        while not rospy.is_shutdown():
            
            hantei = rospy.get_param("/is_move/ok", True)
            if hantei:
                self.object_move("HV8")
                
                # count = 0
                # while count <= 2: 
                #     count = count + 1
                #     print("move_count" + str(count))
                #     naibu_loop.sleep()
                rospy.set_param("/is_record/ok", True)
                rospy.set_param("/is_move/ok", False)
                count = 0

        naibu_loop.sleep()    
            
        # self.object_move("HV8")
            

if __name__=='__main__':
    rospy.init_node('random_state')
    annotation_environment()