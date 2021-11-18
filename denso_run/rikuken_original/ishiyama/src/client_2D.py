#!/usr/bin/env python3
import rospy
from ishiyama.srv import input_data
from sensor_msgs.msg import Image

def get_data():
    rospy.init_node("client", anonymous=True)
    rospy.Subscriber("/photoneo_center/sensor/image_color", Image, callback)
    rospy.spin()

def callback(data):
    rospy.wait_for_service("ishiyama_input_data")
    try:
        tem_data = rospy.ServiceProxy("ishiyama_input_data", input_data)
        out_data = tem_data(data)
    except rospy.ServiceException:
        print("service call failed")

if __name__ == "__main__":
    get_data()