import rospy
import tf2_ros
import tf
import tf2_msgs
import numpy as np


def space_(space_num):
    for i in range(space_num):
        print('')

def print_3(n1, n2, n3):
    print(str(n1) + '   ' + str(n2) + '    ' + str(n3))
if __name__=='__main__':
    rospy.init_node('error_node')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            estimation = tfBuffer.lookup_transform('estimated_tf', 'photoneo_center_optical_frame', rospy.Time(0))
            groud_truth = tfBuffer.lookup_transform('HV8', 'photoneo_center_optical_frame', rospy.Time(0))
            esti_rot = estimation.transform.rotation
            groud_rot = groud_truth.transform.rotation
            esti_matrix = tf.transformations.quaternion_matrix(np.array([esti_rot.x, esti_rot.y, esti_rot.z, esti_rot.w]))
            groud_matrix = tf.transformations.quaternion_matrix(np.array([groud_rot.x, groud_rot.y, groud_rot.z, groud_rot.w]))
            esti_matrix = np.asarray(esti_matrix)
            groud_matrix = np.asarray(groud_matrix)
            error_matrix = np.dot(esti_matrix, groud_matrix.T)
            print('error matrix is')
            print(error_matrix[0:3, 0:3])
            euler = tf.transformations.euler_from_matrix(error_matrix)
            for i in range(3):
                print(euler[i])
            space_(2)
            print(str(tf.transformations.rotation_from_matrix(error_matrix)))
            space_(4)
            rate.sleep()

            

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

            