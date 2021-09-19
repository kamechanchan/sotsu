#!/usr/bin/env python3
import rospy
from estimator.srv import input_data
from sensor_msgs.msg import Image

class Get_data:
    def __init__(self):
        rospy.init_node("client", anonymous=True)
        rospy.Subscriber("/photoneo_center/sensor/image_color", Image, self.callback)
        self.pub = rospy.Publisher("ishiyama_pub_data", Image, queue_size=10)
        rospy.spin()

    def callback(self, data):
        rospy.wait_for_service("ishiyama_input_data")
        try:
            tem_data = rospy.ServiceProxy("ishiyama_input_data", input_data)
            out_data = tem_data(data)
            self.pub.publish(out_data.out_img)
            # print(out_data.out_img)
        except rospy.ServiceException:
            print("service call failed")

if __name__ == "__main__":
    Get_data()