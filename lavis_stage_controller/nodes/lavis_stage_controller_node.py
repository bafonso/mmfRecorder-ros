#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
from __future__ import print_function,division

import rospy

from blob_tracker.msg import Blobs

from std_msgs.msg import Empty

from geometry_msgs.msg import Twist


class LavisStageController(object):
    def __init__(self,*args,**kwargs):
        rospy.loginfo('Initializing lavis_stage_controller_node...')
        self._initialized = False

        self._blobs_sub = rospy.Subscriber('blobs',Blobs,self._blobs_callback)
        self._stage_pub = rospy.Publisher('cmd_vel', Twist, queue_size=2)

        self._feedback_period = rospy.get_param('~feedback_period')
        self._image_x_stage_x_gain = rospy.get_param('~image_x_stage_x_gain')
        self._image_x_stage_y_gain = rospy.get_param('~image_x_stage_y_gain')
        self._image_y_stage_x_gain = rospy.get_param('~image_y_stage_x_gain')
        self._image_y_stage_y_gain = rospy.get_param('~image_y_stage_y_gain')

        self._start_sub = rospy.Subscriber('~start',Empty,self._start_callback)
        self._stop_sub = rospy.Subscriber('~stop',Empty,self._stop_callback)

        self._last_update_time = 0
        self._running = False

        rospy.loginfo('lavis_stage_controller_node initialized!')
        self._initialized = True

    def _blobs_callback(self,data):
        if self._initialized and self._running:
            time_now = rospy.get_time()
            if (time_now - self._last_update_time) >= self._feedback_period:
                self._last_update_time = time_now
                blob = None
                blob_count = len(data.blobs)
                if blob_count == 0:
                    return
                elif blob_count == 1:
                    blob = data.blobs[0]
                else:
                    area_max = 0
                    i_max = None
                    for i in xrange(blob_count):
                        area = data.blobs[i].area
                        if area > area_max:
                            area_max = area
                            i_max = i
                    blob = data.blobs[i_max]
                x_offset = blob.x - data.image_width/2
                y_offset = blob.y - data.image_height/2
                twist = Twist()
                twist.linear.x = (x_offset/100)*self._image_x_stage_x_gain + (y_offset/100)*self._image_y_stage_x_gain
                twist.linear.y = (x_offset/100)*self._image_x_stage_y_gain + (y_offset/100)*self._image_y_stage_y_gain
                self._stage_pub.publish(twist)

    def _start_callback(self,data):
        self._running = True

    def _stop_callback(self,data):
        self._running = False
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        self._stage_pub.publish(twist)


if __name__ == '__main__':
    try:
        rospy.init_node('lavis_stage_controller_node')
        lsc = LavisStageController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
