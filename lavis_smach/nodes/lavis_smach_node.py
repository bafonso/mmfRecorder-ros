#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
from __future__ import print_function,division

import rospy

import smach
import smach_ros

from smach_ros import SimpleActionState


from zaber_stage.msg import EmptyAction
from zaber_stage.msg import MoveAction,MoveGoal


class SetupExperiment(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SETUP_EXPERIMENT')
        return 'succeeded'


class FinishExperiment(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted','preempted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FINISH_EXPERIMENT')
        return 'succeeded'


class LavisSmach(object):
    def __init__(self):

        # Create a SMACH state machine
        self.sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

        # Open the container
        with self.sm:
            # Add states to the container
            smach.StateMachine.add('SETUP_EXPERIMENT', SetupExperiment(),
                                   transitions={'succeeded':'HOME_STAGE',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})
            smach.StateMachine.add('HOME_STAGE',
                                   SimpleActionState('home',
                                                     EmptyAction),
                                   transitions={'succeeded':'CENTER_STAGE',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})
            move_goal = MoveGoal()
            move_goal.pose.position.x = 225
            move_goal.pose.position.y = 225
            smach.StateMachine.add('CENTER_STAGE',
                                   SimpleActionState('move_absolute',
                                                     MoveAction,
                                                     goal=move_goal),
                                   transitions={'succeeded':'FINISH_EXPERIMENT',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})
            # smach.StateMachine.add('WAIT_TO_SAVE_BACKGROUND_IMAGES',
            #                        smach_ros.MonitorState("/faa_controls/save_background_images",
            #                                               Empty,
            #                                               monitor_cb),
            #                        transitions={'invalid':'SAVE_BACKGROUND_IMAGES',
            #                                     'valid':'aborted',
            #                                     'preempted':'preempted'})
            # smach.StateMachine.add('SAVE_BACKGROUND_IMAGES',
            #                        SimpleActionState('/faa_actionlib/save_background_images',
            #                                          EmptyAction),
            #                        transitions={'succeeded':'GET_EXPERIMENT_PARAMETERS',
            #                                     'aborted':'aborted',
            #                                     'preempted':'preempted'})
            # smach.StateMachine.add('GET_EXPERIMENT_PARAMETERS',
            #                        smach_ros.MonitorState("/faa_controls/parameters_initialized",
            #                                               Empty,
            #                                               monitor_cb),
            #                        transitions={'invalid':'SAVE_EXPERIMENT_PARAMETERS',
            #                                     'valid':'aborted',
            #                                     'preempted':'preempted'})
            # smach.StateMachine.add('SAVE_EXPERIMENT_PARAMETERS', SaveExperimentParameters(),
            #                        transitions={'succeeded':'WAIT_TO_LOAD_FLIES',
            #                                     'aborted':'aborted',
            #                                     'preempted':'preempted'})
            # smach.StateMachine.add('WAIT_TO_LOAD_FLIES',
            #                        smach_ros.MonitorState("/faa_controls/load_flies",
            #                                               Empty,
            #                                               monitor_cb),
            #                        transitions={'invalid':'LOAD_FLIES',
            #                                     'valid':'aborted',
            #                                     'preempted':'preempted'})
            # smach.StateMachine.add('LOAD_FLIES',
            #                        SimpleActionState('/faa_actionlib/load_flies',
            #                                          EmptyAction),
            #                        transitions={'succeeded':'WAIT_TO_RUN_EXPERIMENT',
            #                                     'aborted':'aborted',
            #                                     'preempted':'preempted'})
            # smach.StateMachine.add('WAIT_TO_RUN_EXPERIMENT',
            #                        smach_ros.MonitorState("/faa_controls/run_experiment",
            #                                               Empty,
            #                                               monitor_cb),
            #                        transitions={'invalid':'ACCLIMATE_FLIES',
            #                                     'valid':'aborted',
            #                                     'preempted':'preempted'})
            # smach.StateMachine.add('ACCLIMATE_FLIES',
            #                        SimpleActionState('/faa_actionlib/acclimate_flies',
            #                                          EmptyAction),
            #                        transitions={'succeeded':'RUN_EXPERIMENT',
            #                                     'aborted':'aborted',
            #                                     'preempted':'preempted'})

            # self.sm_experiment = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

            # with self.sm_experiment:
                # smach.StateMachine.add('CHECK_PRETRIAL', CheckPretrial(),
                #                        transitions={'run_normally':'WALK_TO_START',
                #                                     'no_acclimation':'CHECK_TRIAL',
                #                                     'aborted':'aborted',
                #                                     'preempted':'preempted'})
                # smach.StateMachine.add('WALK_TO_START',
                #                        SimpleActionState('/faa_actionlib/walk_to_start',
                #                                          EmptyAction),
                #                        transitions={'succeeded':'CATCH_IN_START',
                #                                     'aborted':'aborted',
                #                                     'preempted':'preempted'})
                # smach.StateMachine.add('CATCH_IN_START', CatchInStart(),
                #                        transitions={'succeeded':'WAIT_IN_START',
                #                                     'aborted':'aborted',
                #                                     'preempted':'preempted'})
                # smach.StateMachine.add('WAIT_IN_START',
                #                        SimpleActionState('/faa_actionlib/wait_in_start',
                #                                          EmptyAction),
                #                        transitions={'succeeded':'CHECK_TRIAL',
                #                                     'aborted':'aborted',
                #                                     'preempted':'preempted'})
                # smach.StateMachine.add('CHECK_TRIAL', CheckTrial(),
                #                        transitions={'run_trial':'WALK_TO_END',
                #                                     'end_experiment':'succeeded',
                #                                     'aborted':'aborted',
                #                                     'preempted':'preempted'})
                # smach.StateMachine.add('WALK_TO_END',
                #                        SimpleActionState('/faa_actionlib/walk_to_end',
                #                                          EmptyAction),
                #                        transitions={'succeeded':'CATCH_IN_END',
                #                                     'aborted':'aborted',
                #                                     'preempted':'preempted'})
                # smach.StateMachine.add('CATCH_IN_END', CatchInEnd(),
                #                        transitions={'succeeded':'WAIT_IN_END',
                #                                     'aborted':'aborted',
                #                                     'preempted':'preempted'})
                # smach.StateMachine.add('WAIT_IN_END',
                #                        SimpleActionState('/faa_actionlib/wait_in_end',
                #                                          EmptyAction),
                #                        transitions={'succeeded':'FINISH_TRIAL',
                #                                     'aborted':'aborted',
                #                                     'preempted':'preempted'})
                # smach.StateMachine.add('FINISH_TRIAL', FinishTrial(),
                #                        transitions={'succeeded':'WALK_TO_START',
                #                                     'aborted':'aborted',
                #                                     'preempted':'preempted'})

            # smach.StateMachine.add('RUN_EXPERIMENT', self.sm_experiment,
            #                        transitions={'succeeded':'FINISH_EXPERIMENT',
            #                                     'aborted':'aborted',
            #                                     'preempted':'preempted'})

            smach.StateMachine.add('FINISH_EXPERIMENT', FinishExperiment(),
                                   transitions={'succeeded':'succeeded',
                                                'aborted':'aborted',
                                                'preempted':'preempted'})

    def execute(self):
        outcome = self.sm.execute()


if __name__ == '__main__':
    rospy.init_node('lavis_smach_node')
    ls = LavisSmach()
    sis = smach_ros.IntrospectionServer('lavis_smach', ls.sm, '/SM_ROOT')
    sis.start()
    ls.execute()
    sis.stop()
