#!/usr/bin/env python3
from cv2 import BRISK
from numpy.core.fromnumeric import trace
import rospy
from estimator.srv import input_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from torchvision import transforms 

class Get_data:
    def __init__(self):
        rospy.init_node("client", anonymous=True)
        rospy.Subscriber("/photoneo_center/sensor/image_color", Image, self.callback)
        # self.pub = rospy.Publisher("ishiyama_pub_data", Image, queue_size=10)
        rospy.spin()

    def callback(self, data):
        rospy.wait_for_service("ishiyama_input_data")
        try:
            tem_data = rospy.ServiceProxy("ishiyama_input_data", input_data)
            out_data = tem_data(data)
            input = self.data_transformation(out_data)
            self.to_yolo(input)
            # self.pub.publish(out_data.out_img)
            # print(out_data.out_img)
        except rospy.ServiceException:
            print("service call failed")
    
    def data_transformation(self, data):
        try:
            bridge = CvBridge()
            out_data = bridge.imgmsg_to_cv2(data, "bgr8")
            out_data = transforms.ToTensor()(out_data)
            return out_data
        except CvBridgeError as e:
            print(e)
            return e

    def to_yolo(self, data):
        


if __name__ == "__main__":
    Get_data()