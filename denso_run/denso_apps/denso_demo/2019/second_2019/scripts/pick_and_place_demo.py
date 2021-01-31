#!/usr/bin/env python

import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer
from geometry_msgs.msg import PoseStamped
from denso_state_msgs.msg import WorkPoint
from denso_state_smach import *


class StateMachineConstructor:
    def __init__(self):
        rospy.init_node('state_machine_p_and_p_example')
        self.target_sub = rospy.Subscriber(
            '/target_workpoint', WorkPoint, self.callback)
        self.num_moving_generate_traj = rospy.get_param(
            '~num_moving_generate_traj', 25)
        self.num_approach_generate_traj = rospy.get_param(
            '~num_approach_generate_traj', 3)

    def callback(self, workpoint):
        grasp_object_pose = PoseStamped()
        assemble_object_pose = PoseStamped()
        grasp_object_pose.pose = workpoint.grasp
        assemble_object_pose.pose = workpoint.assemble

        sm = StateMachine(outcomes=['success', 'failed'])
        sis = IntrospectionServer('p_and_p_example', sm, '/SM_ROOT')
        sis.start()
        with sm:
            StateMachine.add(
                "InitialState",
                InitialState(
                    grasp_object_pose,
                    num_moving_generate_traj=self.num_moving_generate_traj,
                    num_approach_generate_traj=self.num_approach_generate_traj),
                transitions={
                    'success': 'BeforeGrasp',
                    'failed': 'failed'})
            StateMachine.add(
                "BeforeGrasp",
                BeforeGrasp(),
                transitions={
                    'success': 'AfterGrasp',
                    'failed': 'failed'})
            StateMachine.add(
                "AfterGrasp",
                AfterGrasp(
                    assemble_object_pose,
                    num_moving_generate_traj=self.num_moving_generate_traj,
                    num_depart_generate_traj=self.num_approach_generate_traj),
                transitions={
                    'success': 'BeforeAssemble',
                    'failed': 'failed'})
            StateMachine.add(
                "BeforeAssemble",
                BeforeAssemble(
                    assemble_object_pose,
                    num_approach_generate_traj=self.num_approach_generate_traj),
                transitions={
                    'success': 'AfterAssemble',
                    'failed': 'failed'})
            StateMachine.add(
                "AfterAssemble",
                AfterAssemble(
                    num_moving_generate_traj=self.num_moving_generate_traj),
                transitions={
                    'success': 'FinalState',
                    'failed': 'failed'})
            StateMachine.add(
                "FinalState",
                FinalState(),
                transitions={
                    'success': 'success',
                    'failed': 'failed'})

        sm.execute()


if __name__ == '__main__':
    sm_constructor = StateMachineConstructor()
    rospy.spin()
