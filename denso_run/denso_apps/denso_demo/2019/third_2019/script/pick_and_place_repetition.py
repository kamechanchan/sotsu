#!/usr/bin/env python

import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer
from geometry_msgs.msg import PoseStamped
from denso_state_smach import *
from denso_state_srvs.srv import WorkPointServiceProvider


class StateMachineConstructor:
    def __init__(self):
        rospy.init_node('state_machine_p_and_p_example')
        rospy.wait_for_service('/test')
        self.relative_work_point_srv = rospy.ServiceProxy(
            '/test', WorkPointServiceProvider)
        self.num_moving_generate_traj = rospy.get_param(
            '~num_moving_generate_traj', 25)
        self.num_approach_generate_traj = rospy.get_param(
            '~num_approach_generate_traj', 3)

    def create_statemachine(self):
        try:
            response = self.relative_work_point_srv()
        except rospy.ServiceException as e:
            rospy.logerr('servive call failed:{}'.format(e))

        relative_grasp_pose = PoseStamped()
        relative_grasp_pose2 = PoseStamped()
        relative_grasp_pose3 = PoseStamped()
        relative_grasp_pose4 = PoseStamped()
        relative_grasp_pose5 = PoseStamped()
        relative_grasp_pose6 = PoseStamped()
        relative_assemble_pose = PoseStamped()
        relative_assemble_pose2 = PoseStamped()
        relative_assemble_pose3 = PoseStamped()
        relative_assemble_pose4 = PoseStamped()
        relative_assemble_pose5 = PoseStamped()
        relative_assemble_pose6 = PoseStamped()

        relative_grasp_pose.pose = response.grasp[0]
        relative_grasp_pose2.pose = response.grasp[1]
        relative_grasp_pose3.pose = response.grasp[2]
        relative_grasp_pose4.pose = response.grasp[3]
        relative_grasp_pose5.pose = response.grasp[4]
        relative_grasp_pose6.pose = response.grasp[5]
        relative_assemble_pose.pose = response.assemble[0]
        relative_assemble_pose2.pose = response.assemble[1]
        relative_assemble_pose3.pose = response.assemble[2]
        relative_assemble_pose4.pose = response.assemble[3]
        relative_assemble_pose5.pose = response.assemble[4]
        relative_assemble_pose6.pose = response.assemble[5]

        sm_top = StateMachine(outcomes=['success', 'failed'])
        with sm_top:
            StateMachine.add(
                "InitialState",
                InitialState(
                    relative_grasp_pose,
                    1,
                    num_moving_generate_traj=self.num_moving_generate_traj,
                    num_approach_generate_traj=self.num_approach_generate_traj),
                transitions={
                    'success': 'SUB1',
                    'failed': 'failed'})

            sm_sub1 = StateMachine(outcomes=['success', 'failed'])
            with sm_sub1:
                StateMachine.add(
                    "BeforeGrasp",
                    BeforeGrasp(),
                    transitions={
                        'success': 'AfterGrasp',
                        'failed': 'failed'})
                StateMachine.add(
                    "AfterGrasp",
                    AfterGrasp(
                        relative_assemble_pose,
                        num_moving_generate_traj=self.num_moving_generate_traj,
                        num_approach_generate_traj=self.num_approach_generate_traj),
                    transitions={
                        'success': 'BeforeAssemble',
                        'failed': 'failed'})
                StateMachine.add(
                    "BeforeAssemble",
                    BeforeAssemble(
                        relative_assemble_pose,
                        num_approach_generate_traj=self.num_approach_generate_traj),
                    transitions={
                        'success': 'AfterAssembleRepetition',
                        'failed': 'failed'})
                StateMachine.add(
                    "AfterAssembleRepetition",
                    AfterAssembleRepetition(
                        relative_grasp_pose2,
                        2,
                        num_moving_generate_traj=self.num_moving_generate_traj,
                        num_approach_generate_traj=self.num_approach_generate_traj),
                    transitions={
                        'success': 'success',
                        'failed': 'failed'})
            StateMachine.add(
                "SUB1", sm_sub1, transitions={
                    'success': 'SUB2', 'failed': 'failed'})

            sm_sub2 = StateMachine(outcomes=['success', 'failed'])
            with sm_sub2:
                StateMachine.add(
                    "BeforeGrasp",
                    BeforeGrasp(),
                    transitions={
                        'success': 'AfterGrasp',
                        'failed': 'failed'})
                StateMachine.add(
                    "AfterGrasp",
                    AfterGrasp(
                        relative_assemble_pose2,
                        num_moving_generate_traj=self.num_moving_generate_traj,
                        num_approach_generate_traj=self.num_approach_generate_traj),
                    transitions={
                        'success': 'BeforeAssemble',
                        'failed': 'failed'})
                StateMachine.add(
                    "BeforeAssemble",
                    BeforeAssemble(
                        relative_assemble_pose2,
                        3,
                        num_approach_generate_traj=self.num_approach_generate_traj),
                    transitions={
                        'success': 'AfterAssembleRepetition',
                        'failed': 'failed'})
                StateMachine.add(
                    "AfterAssembleRepetition",
                    AfterAssembleRepetition(
                        relative_grasp_pose3,
                        num_moving_generate_traj=self.num_moving_generate_traj,
                        num_approach_generate_traj=self.num_approach_generate_traj),
                    transitions={
                        'success': 'success',
                        'failed': 'failed'})
            StateMachine.add(
                "SUB2", sm_sub2, transitions={
                    'success': 'SUB3', 'failed': 'failed'})

            sm_sub3 = StateMachine(outcomes=['success', 'failed'])
            with sm_sub3:
                StateMachine.add(
                    "BeforeGrasp",
                    BeforeGrasp(),
                    transitions={
                        'success': 'AfterGrasp',
                        'failed': 'failed'})
                StateMachine.add(
                    "AfterGrasp",
                    AfterGrasp(
                        relative_assemble_pose3,
                        num_moving_generate_traj=self.num_moving_generate_traj,
                        num_approach_generate_traj=self.num_approach_generate_traj),
                    transitions={
                        'success': 'BeforeAssemble',
                        'failed': 'failed'})
                StateMachine.add(
                    "BeforeAssemble",
                    BeforeAssemble(
                        relative_assemble_pose3,
                        num_approach_generate_traj=self.num_approach_generate_traj),
                    transitions={
                        'success': 'AfterAssembleRepetition',
                        'failed': 'failed'})
                StateMachine.add(
                    "AfterAssembleRepetition",
                    AfterAssembleRepetition(
                        relative_grasp_pose4,
                        4,
                        num_moving_generate_traj=self.num_moving_generate_traj,
                        num_approach_generate_traj=self.num_approach_generate_traj),
                    transitions={
                        'success': 'success',
                        'failed': 'failed'})
            StateMachine.add(
                "SUB3", sm_sub3, transitions={
                    'success': 'SUB4', 'failed': 'failed'})

            sm_sub4 = StateMachine(outcomes=['success', 'failed'])
            with sm_sub4:
                StateMachine.add(
                    "BeforeGrasp",
                    BeforeGrasp(),
                    transitions={
                        'success': 'AfterGrasp',
                        'failed': 'failed'})
                StateMachine.add(
                    "AfterGrasp",
                    AfterGrasp(
                        relative_assemble_pose4,
                        num_moving_generate_traj=self.num_moving_generate_traj,
                        num_approach_generate_traj=self.num_approach_generate_traj),
                    transitions={
                        'success': 'BeforeAssemble',
                        'failed': 'failed'})
                StateMachine.add(
                    "BeforeAssemble",
                    BeforeAssemble(
                        relative_assemble_pose4,
                        num_approach_generate_traj=self.num_approach_generate_traj),
                    transitions={
                        'success': 'AfterAssembleRepetition',
                        'failed': 'failed'})
                StateMachine.add(
                    "AfterAssembleRepetition",
                    AfterAssembleRepetition(
                        relative_grasp_pose5,
                        5,
                        num_moving_generate_traj=self.num_moving_generate_traj,
                        num_approach_generate_traj=self.num_approach_generate_traj),
                    transitions={
                        'success': 'success',
                        'failed': 'failed'})
            StateMachine.add(
                "SUB4", sm_sub4, transitions={
                    'success': 'SUB5', 'failed': 'failed'})

            sm_sub5 = StateMachine(outcomes=['success', 'failed'])
            with sm_sub5:
                StateMachine.add(
                    "BeforeGrasp",
                    BeforeGrasp(),
                    transitions={
                        'success': 'AfterGrasp',
                        'failed': 'failed'})
                StateMachine.add(
                    "AfterGrasp",
                    AfterGrasp(
                        relative_assemble_pose5,
                        num_moving_generate_traj=self.num_moving_generate_traj,
                        num_approach_generate_traj=self.num_approach_generate_traj),
                    transitions={
                        'success': 'BeforeAssemble',
                        'failed': 'failed'})
                StateMachine.add(
                    "BeforeAssemble",
                    BeforeAssemble(
                        relative_assemble_pose5,
                        num_approach_generate_traj=self.num_approach_generate_traj),
                    transitions={
                        'success': 'AfterAssembleRepetition',
                        'failed': 'failed'})
                StateMachine.add(
                    "AfterAssembleRepetition",
                    AfterAssembleRepetition(
                        relative_grasp_pose6,
                        6,
                        num_moving_generate_traj=self.num_moving_generate_traj,
                        num_approach_generate_traj=self.num_approach_generate_traj),
                    transitions={
                        'success': 'success',
                        'failed': 'failed'})
            StateMachine.add(
                "SUB5", sm_sub5, transitions={
                    'success': 'SUB6', 'failed': 'failed'})

            sm_sub6 = StateMachine(outcomes=['success', 'failed'])
            with sm_sub6:
                StateMachine.add(
                    "BeforeGrasp",
                    BeforeGrasp(),
                    transitions={
                        'success': 'AfterGrasp',
                        'failed': 'failed'})
                StateMachine.add(
                    "AfterGrasp",
                    AfterGrasp(
                        relative_assemble_pose6,
                        num_moving_generate_traj=self.num_moving_generate_traj,
                        num_approach_generate_traj=self.num_approach_generate_traj),
                    transitions={
                        'success': 'BeforeAssemble',
                        'failed': 'failed'})
                StateMachine.add(
                    "BeforeAssemble",
                    BeforeAssemble(
                        relative_assemble_pose6,
                        num_approach_generate_traj=self.num_approach_generate_traj),
                    transitions={
                        'success': 'AfterAssemble',
                        'failed': 'failed'})
                StateMachine.add(
                    "AfterAssemble",
                    AfterAssemble(
                        num_moving_generate_traj=self.num_moving_generate_traj),
                    transitions={
                        'success': 'success',
                        'failed': 'failed'})
            StateMachine.add(
                "SUB6",
                sm_sub6,
                transitions={
                    'success': 'FinalState',
                    'failed': 'failed'})

            StateMachine.add(
                "FinalState",
                FinalState(),
                transitions={
                    'success': 'success',
                    'failed': 'failed'})

        sis = IntrospectionServer('p_and_p_example', sm_top, '/SM_ROOT')
        sis.start()

        sm_top.execute()
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('state_machine_p_and_p_example')
    sm = StateMachineConstructor()
    sm.create_statemachine()
    rospy.spin()
