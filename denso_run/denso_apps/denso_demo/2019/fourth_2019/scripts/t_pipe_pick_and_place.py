#!/usr/bin/env python

import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer
from geometry_msgs.msg import PoseStamped
from denso_state_smach import *
from denso_state_srvs.srv import WorkPointServiceProvider
from denso_state_msgs.msg import WorkPoint


class TpipeStateMachineConstructor:
    def __init__(self):
        rospy.init_node('t_pipe_p_and_p')
        rospy.wait_for_service('/test')
        self.relative_work_point_srv = rospy.ServiceProxy(
            '/test', WorkPointServiceProvider)
        self.num_moving_generate_traj = rospy.get_param(
            '~num_moving_generate_traj', 25)
        self.num_approach_generate_traj = rospy.get_param(
            '~num_approach_generate_traj', 3)

    def create_statemachine(self):
        try:
            res = self.relative_work_point_srv()
        except rospy.ServiceException as e:
            rospy.logerr('servive call failed:{}'.format(e))

        relative_workpoint1 = WorkPoint()
        relative_workpoint2 = WorkPoint()
        relative_workpoint3 = WorkPoint()

        relative_workpoint1.grasp = res.grasp[0]
        relative_workpoint1.assemble = res.assemble[0]
        relative_workpoint2.grasp = res.grasp[1]
        relative_workpoint2.assemble = res.assemble[1]
        relative_workpoint3.grasp = res.grasp[2]
        relative_workpoint3.assemble = res.assemble[2]

        sm_top = StateMachine(outcomes=['success', 'failed'])
        with sm_top:
            StateMachine.add(
                "Measurement",
                Measurement(
                    "T_pipe_fix",
                    "jig",
                    relative_workpoint1),
                transitions={
                    'success': 'InitialState',
                    'failed': 'failed'})
            StateMachine.add(
                "InitialState",
                InitialState(
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
                        num_moving_generate_traj=self.num_moving_generate_traj,
                        num_depart_generate_traj=self.num_approach_generate_traj),
                    transitions={
                        'success': 'BeforeAssemble',
                        'failed': 'failed'})
                StateMachine.add(
                    "BeforeAssemble",
                    BeforeAssemble(
                        num_approach_generate_traj=self.num_approach_generate_traj),
                    transitions={
                        'success': 'Measurement',
                        'failed': 'failed'})
                StateMachine.add(
                    "Measurement",
                    Measurement(
                        "T_pipe_fix",
                        "jig",
                        relative_workpoint2),
                    transitions={
                        'success': 'AfterAssembleRepetition',
                        'failed': 'failed'})
                StateMachine.add(
                    "AfterAssembleRepetition",
                    AfterAssembleRepetition(
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
                        num_moving_generate_traj=self.num_moving_generate_traj,
                        num_depart_generate_traj=self.num_approach_generate_traj),
                    transitions={
                        'success': 'BeforeAssemble',
                        'failed': 'failed'})
                StateMachine.add(
                    "BeforeAssemble",
                    BeforeAssemble(
                        num_approach_generate_traj=self.num_approach_generate_traj),
                    transitions={
                        'success': 'Measurement',
                        'failed': 'failed'})
                StateMachine.add(
                    "Measurement",
                    Measurement(
                        "T_pipe_fix",
                        "jig",
                        relative_workpoint3),
                    transitions={
                        'success': 'AfterAssembleRepetition',
                        'failed': 'failed'})
                StateMachine.add(
                    "AfterAssembleRepetition",
                    AfterAssembleRepetition(
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
                        num_moving_generate_traj=self.num_moving_generate_traj,
                        num_depart_generate_traj=self.num_approach_generate_traj),
                    transitions={
                        'success': 'BeforeAssemble',
                        'failed': 'failed'})
                StateMachine.add(
                    "BeforeAssemble",
                    BeforeAssemble(
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
                "SUB3", sm_sub3, transitions={
                    'success': 'FinalState', 'failed': 'failed'})

            StateMachine.add(
                "FinalState",
                FinalState(),
                transitions={
                    'success': 'success',
                    'failed': 'failed'})

        sis = IntrospectionServer('t_pipe_p_and_p', sm_top, '/SM_ROOT')
        sis.start()

        sm_top.execute()
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('t_pipe_p_and_p')
    sm = TpipeStateMachineConstructor()
    sm.create_statemachine()
    rospy.spin()
