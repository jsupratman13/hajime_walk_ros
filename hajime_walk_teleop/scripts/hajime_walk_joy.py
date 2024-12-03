#!/usr/bin/env python
# -*- coding:utf-8 -*-

# Copyright (c) 2024 Joshua Supratman
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import rospy
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty

from hajime_walk_msgs.msg import HajimeMotionAction
from hajime_walk_msgs.msg import HajimeMotionGoal
from hajime_walk_msgs.msg import HajimeWalk


class HajimeWalkJoy(object):
    def __init__(self) -> None:
        self.__walk_linear = rospy.get_param('axes_linear')
        self.__walk_scale = rospy.get_param('scale_linear')
        self.__walk_angular = rospy.get_param('buttons_angular')
        self.__walk_angular_scale = rospy.get_param('scale_angular')

        self.__continuous_mode = rospy.get_param('button_enable_continuous_mode')
        self.__discrete_mode = rospy.get_param('button_disable_continuous_mode')
        self.__enable_continuous_mode = False

        self.__motions = {}
        for key, value in rospy.get_param('button_motions').items():
            self.__motions[key] = value

        rospy.Subscriber('joy', Joy, self._joy_callback, queue_size=1)
        self.__walk_pub = rospy.Publisher('hajime_walk/walk', HajimeWalk, queue_size=1)
        self.__cancel_pub = rospy.Publisher('hajime_walk/cancel', Empty, queue_size=1)
        self.__motion_client = SimpleActionClient('hajime_walk/motion', HajimeMotionAction)

        rospy.loginfo('Hajime Walk Joy is ready')

    def _joy_callback(self, joy_msg: Joy) -> None:
        # wait for motion to finish
        if self.__motion_client.get_state() == GoalStatus.ACTIVE:
            return

        # motion
        for name, params in self.__motions.items():
            if 'button' in params:
                if joy_msg.buttons[params['button']]:
                    goal = HajimeMotionGoal()
                    goal.motion_id = int(params['motion_id'])
                    self.__motion_client.send_goal(goal)
                    rospy.loginfo(f'execute motion {name}')
                    return
            elif 'axis' in params:
                if joy_msg.axes[params['axis']] >= params['axis_threshold'] > 0 or \
                        joy_msg.axes[params['axis']] <= params['axis_threshold'] < 0:
                    goal = HajimeMotionGoal()
                    goal.motion_id = int(params['motion_id'])
                    self.__motion_client.send_goal(goal)
                    rospy.loginfo(f'execute motion {name}')
                    return

        # enable/disable continuous mode
        if joy_msg.buttons[self.__continuous_mode] and not self.__enable_continuous_mode:
            self.__enable_continuous_mode = True
        elif joy_msg.buttons[self.__discrete_mode] and self.__enable_continuous_mode:
            self.__cancel_pub.publish()
            self.__enable_continuous_mode = False

        # walk
        if joy_msg.axes[self.__walk_linear['x']] or joy_msg.axes[self.__walk_linear['y']] or \
                joy_msg.buttons[self.__walk_angular['left']] or joy_msg.buttons[self.__walk_angular['right']]:
            walk_msg = HajimeWalk()
            walk_msg.stride_x = int(joy_msg.axes[self.__walk_linear['x']]) * self.__walk_scale['x']
            walk_msg.stride_y = int(joy_msg.axes[self.__walk_linear['y']]) * self.__walk_scale['y']
            if joy_msg.buttons[self.__walk_angular['left']] and not joy_msg.buttons[self.__walk_angular['right']]:
                dir = 1
            elif joy_msg.buttons[self.__walk_angular['right']] and not joy_msg.buttons[self.__walk_angular['left']]:
                dir = -1
            else:
                dir = 0
            walk_msg.stride_th = dir * self.__walk_angular_scale
            self.__walk_pub.publish(walk_msg)
            return

        # stop
        if self.__enable_continuous_mode:
            self.__walk_pub.publish(HajimeWalk())
            return
        self.__cancel_pub.publish()


if __name__ == '__main__':
    rospy.init_node('hajime_walk_joy')
    hajime_walk_joy = HajimeWalkJoy()
    rospy.spin()
