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
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty

from hajime_walk_msgs.msg import HajimeMotion
from hajime_walk_msgs.msg import HajimeWalk


class HajimeWalkJoy(object):
    def __init__(self) -> None:
        rospy.Subscriber('joy', Joy, self._joy_callback, queue_size=1)
        self.__motion_pub = rospy.Publisher('hajime_walk/motion', HajimeMotion, queue_size=1)
        self.__walk_pub = rospy.Publisher('hajime_walk/walk', HajimeWalk, queue_size=1)
        self.__cancel_pub = rospy.Publisher('hajime_walk/cancel', Empty, queue_size=1)
        self.__motion_flag = False
        self.__walk_test = 1

    def _joy_callback(self, joy_msg: Joy) -> None:
        # walk test
        if joy_msg.buttons[-2]:
            self.__walk_test *= -1
            if self.__walk_test == 1:
                self.__cancel_pub.publish()
        if self.__walk_test == -1:
            self.__motion_flag = False
            self.__walk_pub.publish(HajimeWalk())
            return

        # Motion
        if joy_msg.buttons[0] or joy_msg.buttons[1] or \
                joy_msg.buttons[2] or joy_msg.buttons[3] or \
                joy_msg.buttons[6] or joy_msg.buttons[7]:
            motion_msg = HajimeMotion()
            self.__cancel_pub.publish()
            if joy_msg.buttons[0]:
                motion_msg.motion_id = 4
                self.__motion_pub.publish(motion_msg)
            elif joy_msg.buttons[1]:
                motion_msg.motion_id = 7
                self.__motion_pub.publish(motion_msg)
            elif joy_msg.buttons[2]:
                motion_msg.motion_id = 3
                self.__motion_pub.publish(motion_msg)
            elif joy_msg.buttons[3]:
                motion_msg.motion_id = 6
                self.__motion_pub.publish(motion_msg)
            elif joy_msg.buttons[6] or joy_msg.axes[2] < 0:
                motion_msg.motion_id = 30
                self.__motion_pub.publish(motion_msg)
            elif joy_msg.buttons[7] or joy_msg.axes[5] < 0:
                motion_msg.motion_id = 31
                self.__motion_pub.publish(motion_msg)
            self.__motion_flag = True

        # Walk or cancel
        if joy_msg.axes[-1] or joy_msg.axes[-2] or joy_msg.buttons[4] or joy_msg.buttons[5]:
            self.__motion_flag = False
            walk_msg = HajimeWalk()
            walk_msg.stride_x = int(joy_msg.axes[-1]) * 10
            walk_msg.stride_y = int(joy_msg.axes[-2]) * 12
            if joy_msg.buttons[4] and not joy_msg.buttons[5]:
                dir = 1
            elif joy_msg.buttons[5] and not joy_msg.buttons[4]:
                dir = -1
            else:
                dir = 0
            walk_msg.stride_th = dir * 10
            self.__walk_pub.publish(walk_msg)
        elif not self.__motion_flag:
            self.__cancel_pub.publish()


if __name__ == '__main__':
    rospy.init_node('hajime_walk_joy')
    hajime_walk_joy = HajimeWalkJoy()
    rospy.spin()
