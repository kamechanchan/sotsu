#!/usr/bin/env python

import rospy, rospkg, tf, random, time, sys
from tf.transformations import quaternion_from_euler
from math import *
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped

import ros_numpy, pcl, h5py, os
from util import *
from gazebo_msgs.srv import *
from gazebo_msgs.msg import *
from tqdm import tqdm

class RandomMove(object):
    def __init__(self):
        self.pos_up_ = ModelState()
        rospack = rospkg.RosPack()
        self.num_ =-5
        self.sensor_parent_frame_ = rospy.get_param("~sensor_parent_frame")
        self.lis = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.topic_name = rospy.get_param("~topic_name")
        self.object_name = rospy.get_param("~object_name")
        self.num_dataset = rospy.get_param("~num_dataset", 50000)
        self.bar = tqdm(total=self.num_dataset)
        self.bar.set_description("Progress rate")
        self.package_path_ = rospack.get_path("gen_dataset")
        self.save_file_path = self.package_path_ + "/../datasets/" + self.sensor_parent_frame_ + "_" + self.object_name
        self.pos_up_.model_name = self.object_name
        self.numpy_pose = np.zeros(7)
        self.pose_optical = PoseStamped
        self.sub_ = rospy.Subscriber("/" + self.topic_name + "/sensor/points", PointCloud2, self.record_data)
        self.init_x = rospy.get_param("~init_x")
        self.init_hdf5(self.save_file_path)
        rospy.wait_for_service('/gazebo/set_model_state')
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            self.set_model_ = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            self.get_model_ = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def init_hdf5(self, file_path):
        file_path = file_path + ".hdf5"
        self.hdf5_file_ = h5py.File(file_path, 'w')

    def random_state_make(self):
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = random.uniform(-0.05 + self.init_x, 0.05 + self.init_x)
        pose.pose.position.y = random.uniform(-0.2, 0.2)
        pose.pose.position.z = random.uniform(0.1, 0.5)

        roll = random.uniform(-3.14, 3.14)
        pitch = random.uniform(-3.14, 3.14)
        yaw = random.uniform(-3.14, 3.14)
        quat = quaternion_from_euler(roll, pitch, yaw)

        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        self.pos_up_.pose = pose.pose
        self.pose_optical = self.lis.transformPose(self.sensor_parent_frame_, pose)
        return self.set_model_(self.pos_up_)

    def init_state_make(self):
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = self.init_x
        pose.pose.position.y = 0
        pose.pose.position.z = 0.1

        quat = quaternion_from_euler(0, 0, 0)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        self.pos_up_.pose = pose.pose
        return self.set_model_(self.pos_up_)

    def get_pose_array(self, position):
        x = position.pose.position.x
        y = position.pose.position.y
        z = position.pose.position.z
        ori_x = position.pose.orientation.x
        ori_y = position.pose.orientation.y
        ori_z = position.pose.orientation.z
        ori_w = position.pose.orientation.w
        return np.array([x, y, z, ori_x, ori_y, ori_z, ori_w])

    def br_pose(self):
        res_pose = PoseStamped()
        pose_optical = self.lis.transformPose(self.sensor_parent_frame_, res_pose)
        self.numpy_pose = self.get_pose_array(pose_optical)
        self.br.sendTransform((self.numpy_pose[0], self.numpy_pose[1], self.numpy_pose[2]), (self.numpy_pose[3], self.numpy_pose[4], self.numpy_pose[5], self.numpy_pose[6]), res_pose.header.stamp, self.object_name, self.sensor_parent_frame_)
        return True

    def record_data(self, data):
        if self.num_ <= 0:
            print("Initialize dataset model! Wait for a wile")
            self.num_ += 1
            self.init_state_make()
        elif self.br_pose():
            header = data.header
            pc = ros_numpy.numpify(data)
            height = pc.shape[0]
            width = pc.shape[1]
            np_points = np.zeros((height * width, 3), dtype=np.float32)
            np_points[:, 0] = np.resize(pc['x'], height * width)
            np_points[:, 1] = np.resize(pc['y'], height * width)
            np_points[:, 2] = np.resize(pc['z'], height * width)
            pcd = np_points[~np.any(np.isnan(np_points), axis=1)]
            p = pcl.PointCloud(np.array(pcd, dtype=np.float32))

            if (self.num_ != self.num_dataset+1 and self.num_ > 0):
 
                data_g = self.hdf5_file_.create_group("data_" + str(self.num_))
                data_g.create_dataset("pcl", data=p, compression="lzf")
                print(self.numpy_pose)
                data_g.create_dataset("pose", data=self.numpy_pose, compression="lzf")
                self.num_ += 1
                self.hdf5_file_.flush()
                self.bar.update(1)
                self.random_state_make()
            else:
                self.hdf5_file_.flush()
                self.hdf5_file_.close()
                print("Finish Clealy")
                os._exit(10)
        else:
            print("Error!")

def main():
    rospy.init_node("random_state_maker_node", anonymous=False)
    random_state_maker = RandomMove()
    try:
        rospy.spin()
    except:
        print("Shutting down")

if __name__ == '__main__':
    main()

