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

import copy
import typing

from .motion_frame import MotionFrame
from .servo import Servo

MAX_FRAME_NUM = 16


class Motion(object):
    def __init__(self, default_servo_status: typing.List[Servo]) -> None:
        self.__m_default_servo_status = default_servo_status
        self.__m_frames = []
        self.__m_playback_frame = []
        self.__m_timer = 0.0
        self.__m_frame_count = 0
        self.__m_motion_start_flag = False
        self.__m_wait_count = 0
        self.__m_play_id = 0
        self.__m_end_id = 0

    def save_to(self, motion_file: str, vmotion_file: str) -> None:
        with open(motion_file, 'w') as f:
            for i, frame in enumerate(self.__m_frames):
                if i > MAX_FRAME_NUM:
                    print(f'frame {i} is not saved, exceeding max frame number')
                    break
                f.write(f'{frame.m_time / 10},')
                for j, servo_angles in enumerate(frame.m_servo_angles):
                    if j == 29 or j == 30:
                        f.write(f'{i + 2},')
                    else:
                        f.write(f'{servo_angles / 10},')
                f.write('0x0\n')

    def load_from(self, motion_file: str, vmotion_file: str) -> None:
        # TODO: check if motion file is valid
        with open(motion_file, 'r') as f:
            for i, line in enumerate(f):
                if not self.add_frame():
                    print(f'frame {i} is not added, exceeding max frame number')
                    return

                # process time
                m_frame = self.__m_frames[i]
                m_frame.m_time = int(float(line.split(",", 1)[0])) * 10

                # remove time segment
                line = line.split(",", 1)[1]

                for i in range(31):
                    m_value = float(line.split(",", 1)[0])
                    m_frame.m_servo_angles[i] = int(m_value * 10)

                    # remove processed segment
                    line = line.split(",", 1)[1]

    def play_back(self, start_id: int, end_id: int) -> None:
        if not self.__m_motion_start_flag:
            self.__m_play_id = start_id
            self.__m_end_id = end_id
            self.__m_motion_start_flag = True
            # TODO: onPlayBack

    def add_frame(self) -> bool:
        if len(self.__m_frames) < MAX_FRAME_NUM:
            self.__m_frame_count += 1
            self.__m_frames.append(MotionFrame(self.__m_frame_count, self.__m_default_servo_status))
            return True
        return False

    def frame_count(self) -> int:
        return self.__m_frame_count

    def delete_frame(self, frame_id: int) -> bool:
        for i, frame in enumerate(self.__m_frames):
            if frame.m_id == frame_id:
                del self.__m_frames[i]
                self.__m_frame_count -= 1
                new_id = 0
                for m in range(self.__m_frame_count):
                    self.__m_frames[i].m_id = new_id
                    new_id += 1
                return True
        print(f'frame id {frame_id} does not exist')
        return False

    def set_frame_wait_time(self, frame_id: int, wait_time: int) -> bool:
        for i, frame in enumerate(self.__m_frames):
            if frame.m_id == frame_id:
                self.__m_frames[i].m_time = wait_time
                return True
        return False

    def frame_wait_time(self, frame_id: int) -> int:
        for i, frame in enumerate(self.__m_frames):
            if frame.m_id == frame_id:
                return frame.m_time
        return -1

    def frame_servo_angle(self, frame_id: int, servo_num: int) -> int:
        for i, frame in enumerate(self.__m_frames):
            if frame.m_id == frame_id:
                return frame.m_servo_angles[servo_num]
        return None

    def set_frame_servo_angle(self, frame_id: int, servo_num: int, servo_angle: int) -> bool:
        for i, frame in enumerate(self.__m_frames):
            if frame.m_id == frame_id:
                self.__m_frames[i].m_servo_angles[servo_num] = servo_angle
                return True
        return False

    def frame_vmservo_angle(self, frame_id: int, servo_num: int, servo_angle: int) -> bool:
        for i, frame in enumerate(self.__m_frames):
            if frame.m_id == frame_id:
                return frame.m_vm_servo_angles[servo_num]
        return None

    def set_frame_vmservo_angle(self, frame_id: int, servo_num: int, servo_angle: int) -> bool:
        for i, frame in enumerate(self.__m_frames):
            if frame.m_id == frame_id:
                self.__m_frames[i].m_vm_servo_angles[servo_num] = servo_angle
                return True
        return False

    def copy_frame(self, src_id: int, dst_id: int) -> bool:
        for i, frame in enumerate(self.__m_frames):
            if frame.m_id == src_id:
                src_frame = copy.deepcopy(frame)
                break
        else:
            return False
        for i, frame in enumerate(self.__m_frames):
            if frame.m_id == dst_id:
                self.__m_frames[i] = src_frame
                return True
        return False
