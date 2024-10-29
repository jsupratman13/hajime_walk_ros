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

import typing

from .servo import Servo


class MotionFrame(object):
    def __init__(self, frame_id: int, servos: typing.List[Servo]) -> None:
        self.m_id = frame_id
        self.m_time = 0
        self.m_servos = servos
        self.m_vm_servo_angles = [0] * 31
        self.m_servo_angles = [0] * 31

    @classmethod
    def from_motion_frame(cls, new_frame_id: int, rhs: 'MotionFrame') -> None:
        new_instance = cls(new_frame_id.rhs.m_servos)
        new_instance.m_servo_angles = rhs.m_servo_angles[:]
        new_instance.m_vm_servo_angles = rhs.m_vm_servo_angles[:]
        new_instance.m_time = rhs.m_time
        return new_instance
