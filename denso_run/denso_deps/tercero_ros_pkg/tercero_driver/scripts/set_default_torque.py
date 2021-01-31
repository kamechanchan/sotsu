#!/usr/bin/env python

import rospy
from tercero_srvs.srv import GetHandReady
from tercero_srvs.srv import SetTorque, SetTorqueRequest

from time import sleep

DEFAULT_CHACK_TORQUE = 10
TIME_OUT = 60

if __name__ == '__main__':
    rospy.init_node('set_default_torque_node')
    chack_default_torque = rospy.get_param(
        '~default_chack_torque', DEFAULT_CHACK_TORQUE)
    pusher_default_torque = rospy.get_param(
        '~default_pusher_torque', DEFAULT_CHACK_TORQUE)
    finish_initialize = rospy.get_param('~finish_initialize', False)
    flag = False
    rospy.loginfo('Waiting for tercero services are activated.')

    rospy.wait_for_service('/tercero/chack_check')
    try:
        chack_check = rospy.ServiceProxy('/tercero/chack_check', GetHandReady)
        ret = chack_check()
        sec = 0
        while (not ret.ready):
            rospy.loginfo("Waiting ready to set tercero torque...")
            ret = chack_check()
            sleep(1)
            sec += 1
            if (sec == TIME_OUT):
                rospy.logerr('Failed Ethernet/IP or tercero state')
                rospy.logerr('Not Set chack torque')
                flag = False
                break

        flag = ret.ready

        if (flag):
            rospy.wait_for_service('/tercero/set_chack_torque')
            try:
                set_chack_torque = rospy.ServiceProxy(
                    '/tercero/set_chack_torque', SetTorque)
                req = SetTorqueRequest()
                req.torque = chack_default_torque
                ret = set_chack_torque(req)
                rospy.loginfo('tercero set default chack torque {}% complete.')
            except rospy.ServiceException as e:
                rospy.logerr('Set chack torque service failed:{}'.format(e))

    except rospy.ServiceException as e:
        rospy.logerr('Get chack state service failed:{}'.format(e))

    if (flag):
        rospy.wait_for_service('/tercero/pusher_check')
        try:
            pusher_check = rospy.ServiceProxy(
                '/tercero/pusher_check', GetHandReady)
            ret = pusher_check()

            if (ret.ready):
                rospy.wait_for_service('/tercero/set_pusher_torque')
                try:
                    set_pusher_torque = rospy.ServiceProxy(
                        '/tercero/set_pusher_torque', SetTorque)
                    req = SetTorqueRequest()
                    req.torque = pusher_default_torque
                    ret = set_pusher_torque(req)
                    rospy.loginfo(
                        'tercero set default pusher torque {}% complete.')
                    rospy.set_param('~finish_initialize', True)
                except rospy.ServiceException as e:
                    rospy.logerr(
                        'Set pusher torque service failed:{}'.format(e))
            else:
                rospy.logerr("Not Set pusher torque")

        except rospy.ServiceException as e:
            rospy.logerr('Get pusher state service failed:{}'.format(e))
    else:
        rospy.logerr("Not set tercero torque !!")
