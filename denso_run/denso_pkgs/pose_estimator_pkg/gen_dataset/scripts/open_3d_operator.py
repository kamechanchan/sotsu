#! /usr/bin/env python

import rospy
import time
from sensor_msgs.msg import PointCloud2

from util import *
from open3d import *
import glob, os

class Open3dNode(object):
    def __init__(self, sub_topic_name, pub_topic_name):
        self.sub_topic_name = sub_topic_name
        self.pub_topic_name = pub_topic_name
        self.sub = rospy.Subscriber(self.sub_topic_name, PointCloud2, self.callback)
        self.pub = rospy.Publisher(self.pub_topic_name, PointCloud2, queue_size=1)
        self.obj_datas = None
        self.load_ply()

    def load_ply(self):
        files_list = [f for f in glob.glob("*.pcd")]
        for path in files_list:
            path = os.path.join(os.getcwd(), path)
            self.obj_datas = get_cropped(path)

    def Publisher(self, data):
        self.pub.publish(data)

    def callback(self, data):

        header = data.header
        data = convertCloudFromRosToOpen3d(data)
        if not self.obj_datas:
            obj_data = crop_geometry(data)
        else:
            obj_data = self.obj_datas
        self.Publisher(convertCloudFromOpen3dToRos(obj_data, header))


if __name__ == "__main__":
    rospy.init_node("Open3dNode", anonymous=True)
    node = Open3dNode("/photoneo/cloud_without_segmented", "/edited_cloud")

    while not rospy.is_shutdown():
        rospy.sleep(0.1)

