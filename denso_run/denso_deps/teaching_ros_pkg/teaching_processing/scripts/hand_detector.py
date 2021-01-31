#!/usr/bin/env python

from __future__ import division
import rospy
import rospkg
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

class HandDetectNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/photoneo_center/rgb_texture", Image, self.get_hand_point, queue_size=1, buff_size=2**24)
        self.image_pub = rospy.Publisher(
            "/hand_keypoint_image", Image, queue_size=1)
        rospack = rospkg.RosPack()
        protoFile = rospack.get_path(
            'teaching_processing') + '/hand/' + 'pose_deploy.prototxt'
        weightsFile = rospack.get_path(
            'teaching_processing') + '/hand/' + 'pose_iter_102000.caffemodel'
        self.net = cv2.dnn.readNetFromCaffe(protoFile, weightsFile)
        self.nPoints = 22
        self.threshold = 0.1
        rospy.loginfo("Initialized HandDetectNode.")

    # def imgCb(self, data):
    #     try:
    #         self.raw_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
    #     except CvBridgeError as e:
    #         print(e)

    def get_hand_point(self, data):
        try:
            self.raw_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv_image = self.raw_img
        ymin = 250
        ymax = 650
        xmin = 400
        xmax = 700
        roi_image = cv_image[ymin:ymax, xmin:xmax]
        frameWidth = roi_image.shape[1]
        frameHeight = roi_image.shape[0]
        aspect_ratio = frameWidth / frameHeight
        inHeight = 368
        inWidth = int(((aspect_ratio*inHeight)*8)//8)
        inpBlob = cv2.dnn.blobFromImage(
            roi_image, 1.0 / 255, (inWidth, inHeight), (0, 0, 0), swapRB=False, crop=False)
        self.net.setInput(inpBlob)
        output = self.net.forward()

        for i in range(self.nPoints):
            probMap = output[0, i, :, :]
            probMap = cv2.resize(probMap, (frameWidth, frameHeight))

            minVal, prob, minLoc, point = cv2.minMaxLoc(probMap)

            if prob > self.threshold:
                cv2.circle(roi_image, (int(point[0]), int(
                    point[1])), 6, (0, 255, 255), thickness=-1, lineType=cv2.FILLED)
                print("key_point_" + str(i) + ": point0: " + str(point[0]) + ", point1: " + str(point[1]))
                # cv2.putText(cv_image, "{}".format(i), (int(point[0]), int(
                #     point[1])), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 1, lineType=cv2.LINE_AA)

        cv_image[ymin:ymax, xmin:xmax] = roi_image
        cv2.rectangle(cv_image, (xmin, ymin), (xmax, ymax), (0, 200, 0), 4)

        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

def main():
    rospy.init_node('hand_detect_node')
    HandDetectNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down HandDetectNode.")

if __name__ == '__main__':
    main()
