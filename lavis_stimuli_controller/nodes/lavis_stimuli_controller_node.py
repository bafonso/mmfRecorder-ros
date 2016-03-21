#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
from __future__ import print_function,division

import rospy

from std_msgs.msg import Empty

from larvae_behavior_classifier.msg import Behaviors

from mightex_controller.msg import CmdCurrent,CmdChannel

from pyaudio_controller.msg import Tone


class LavisStimuliController(object):
    def __init__(self,*args,**kwargs):
        rospy.loginfo('Initializing lavis_stimuli_controller_node...')
        self._initialized = False

        self._behaviors_sub = rospy.Subscriber('behaviors',Behaviors,self._behaviors_callback)

        self._cmd_light_current_pub = rospy.Publisher('cmd_current', CmdCurrent, queue_size=2)
        self._cmd_light_off_pub = rospy.Publisher('cmd_off', CmdChannel, queue_size=2)
        self._cmd_all_lights_off_pub = rospy.Publisher('cmd_all_off', Empty, queue_size=2)
        self._play_tone_pub = rospy.Publisher('play_tone', Tone, queue_size=2)

        self._feedback_period = rospy.get_param('~feedback_period')

        self._start_sub = rospy.Subscriber('~start',Empty,self._start_callback)
        self._stop_sub = rospy.Subscriber('~stop',Empty,self._stop_callback)

        self._last_update_time = 0
        self._running = False

        self._test_current = 0
        self._test_current_inc = 100
        self._test_current_max = 1000

        self._test_freq = 0
        self._test_freq_inc = 1000
        self._test_freq_max = 10000
        self._test_duration = 250

        rospy.loginfo('lavis_stimuli_controller_node initialized!')
        self._initialized = True

    def _behaviors_callback(self,data):
        if self._initialized and self._running:
            time_now = rospy.get_time()
            if (time_now - self._last_update_time) >= self._feedback_period:
                self._last_update_time = time_now
                self._test_current = (self._test_current + self._test_current_inc) % self._test_current_max
                cmd_current = CmdCurrent()
                cmd_current.channel = 1
                cmd_current.current = self._test_current
                self._cmd_light_current_pub.publish(cmd_current)

                self._test_freq = (self._test_freq + self._test_freq_inc) % self._test_freq_max
                tone = Tone()
                tone.frequency = self._test_freq
                tone.duration = self._test_duration
                self._play_tone_pub.publish(tone)

    def _start_callback(self,data):
        self._running = True

    def _stop_callback(self,data):
        self._running = False
        self._cmd_all_lights_off_pub.publish(Empty())


if __name__ == '__main__':
    try:
        rospy.init_node('lavis_stimuli_controller_node')
        lsc = LavisStimuliController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
