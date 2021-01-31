#!/usr/bin/env python

import rospy
from mhand_srvs.srv import SetTorque, SetTorqueRequest

if __name__ == '__main__':
    rospy.init_node('mhand_set_default_torque')
    torque = rospy.get_param('/mhand/default_torque', 10)
    rospy.loginfo('waiting for mhand services are activated.')
    rospy.wait_for_service('/mhand/set_torque')
    try:
        set_torque = rospy.ServiceProxy('/mhand/set_torque', SetTorque)
        req = SetTorqueRequest()
        req.torque = torque
        ret = set_torque(req)
        rospy.loginfo('mhand set default torque {}% complete.')
    except rospy.ServiceException as e:
        rospy.logerr('Set torque service failed:{}'.format(e))

