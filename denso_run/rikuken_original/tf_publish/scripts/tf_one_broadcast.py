#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs

if __name__=='__main__':
    rospy.init_node('inti')
    broad = tf2_ros.TransformBroadcaster()
    trans = geometry_msgs.msg.TransformStamped()
    trans.header.stamp = rospy.Time.now()
    trans.header.frame_id = 'world'
    trans.child_frame_id = 'shiny'
    jin = geometry_msgs.msg.Transform()
    jin.translation.x = 43
    jin.translation.y = 2
    jin.translation.z = 23
    jin.rotation.x = 0
    jin.rotation.y = 0
    jin.rotation.z = 0
    jin.rotation.w = 1
    trans.transform.translation.x = 23
    trans.transform.translation.y = 34
    trans.transform.translation.z = 12
    trans.transform.rotation.x = 0
    trans.transform.rotation.y = 0
    trans.transform.rotation.z = 0
    trans.transform.rotation.w = 1
    late = rospy.Rate(10)
    '''while not rospy.is_shutdown():
        broad.sendTransform(trans)
        print(str(trans))
        late.sleep()'''

    rer = input()
    broad.sendTransform(trans)
    brad_1 = tf2_ros.TransformBroadcaster()
    trans.header.frame_id = "world"
    trans.child_frame_id = "naoki"
    trans.transform = jin
    brad_1.sendTransform(trans)
    print(str(trans))
    
    
