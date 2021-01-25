#!/usr/bin/env python

import rospy

from denso_recognition_msgs.msg import InputCNNArray
from denso_recognition_srvs.srv import PoseEstimate, PoseEstimateResponse

from object_pose_estimator import ObjectPoseEstimator

import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped


class ObjectPoseEstimatorServer(object):
    def __init__(self):
        self.tf_broadcaster_ = tf2_ros.TransformBroadcaster()
        self.tf_transform_stamped_ = TransformStamped()
        self.tf_transform_stamped_.transform.rotation.w = 1.0
        self.is_ok_ = False
        self.use_gpu_ = rospy.get_param("~use_gpu", True)
        self.division_ = rospy.get_param("~division", 50)
        self.orientation_ = rospy.get_param("~orientation", "rotation_matrix")
        self.model_ = rospy.get_param("~model", "hv7_euler2.model")
        self.voxels_array_ = InputCNNArray()
        self.sensor_frame_id_ = rospy.get_param(
            "~sensor_frame_id", "photoneo_center_optical_frame")
        self.estimate_frame_id_ = rospy.get_param(
            "~estimate_frame_id", "estimate_tf")
        self.object_pose_estimator_ = ObjectPoseEstimator(
            self.use_gpu_, self.division_, self.orientation_, self.model_, self.sensor_frame_id_)
        self.object_pose_estimator_.setParameter()
        self.input_data_sub_ = rospy.Subscriber(
            "/input_data", InputCNNArray, self.receiveInputDataCallback, queue_size=1)
        self.pose_estimete_service_ = rospy.Service(
            '/cnn_pose_estimator/estimate_object_by_cnn', PoseEstimate, self.estimate_pose)
        rospy.loginfo("Create ObjectPoseEstimaterServer instatnce !!")

    def receiveInputDataCallback(self, voxels_array):
        self.voxels_array_ = voxels_array

    def estimate_pose(self, req):
        self.is_ok_ = False
        voxels = self.voxels_array_
        ret = PoseEstimateResponse()
        success, pose = self.object_pose_estimator_.estimateObjectPose(voxels)
        if success:
            ret.success = True
            self.tf_transform_stamped_.transform.translation = pose.pose.position
            self.tf_transform_stamped_.transform.rotation = pose.pose.orientation
            self.tf_transform_stamped_.header.frame_id = self.sensor_frame_id_
            self.tf_transform_stamped_.child_frame_id = self.estimate_frame_id_
            self.is_ok_ = True
        else:
            ret.success = False
            self.is_ok_ = False

        return ret.success

    def broadcastEstimateTF(self):
        if self.is_ok_:
            self.tf_transform_stamped_.header.stamp = rospy.Time.now()
            self.tf_broadcaster_.sendTransform(self.tf_transform_stamped_)


def main():
    rospy.init_node("object_pose_estimate_server", anonymous=False)
    object_pose_estimator_server = ObjectPoseEstimatorServer()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        object_pose_estimator_server.broadcastEstimateTF()
        rate.sleep()


if __name__ == '__main__':
    main()
