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

import os

import numpy as np
import rospkg
import rospy
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QFileDialog
from python_qt_binding.QtWidgets import QListWidgetItem
from python_qt_binding.QtWidgets import QWidget
from qt_gui.plugin_context import PluginContext
from rqt_gui_py.plugin import Plugin
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from .motion import Motion
from .servo import Servo


class HajimeMotionCreator(Plugin):
    def __init__(self, context: PluginContext) -> None:
        super().__init__(context)
        self.setObjectName(self.__class__.__name__)

        self._widget = QWidget()

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_hajime_motion_creator'), 'resource', 'hajime_motion_creator.ui')
        loadUi(ui_file, self._widget)
        context.add_widget(self._widget)

        # connect signals
        self._widget.pushDelete.clicked[bool].connect(self._on_delete_list)
        self._widget.pushMake.clicked[bool].connect(self._add_list)
        self._widget.listFrame.currentItemChanged.connect(self._show_current_frame)
        self._widget.spinSpd.valueChanged.connect(self._on_write_frame_spd)
        self._widget.pushSend.clicked[bool].connect(self._on_push_send_angles)
        self._widget.pushCopy.clicked[bool].connect(self._on_push_copy_frame)
        self._widget.pushPaste.clicked[bool].connect(self._on_push_paste_frame)
        self._widget.doubleStep.valueChanged.connect(self._on_spin_step_changed)
        self._widget.pushInterchanging.clicked[bool].connect(self._on_push_interchanging)
        self._widget.pushSave.clicked[bool].connect(self._on_save_motion_file)
        self._widget.pushLoad.clicked[bool].connect(self._on_load_motion_file)

        # TODO
        self._widget.pushSend.setEnabled(False)
        self._widget.pushCopy.setEnabled(False)
        self._widget.pushPaste.setEnabled(False)
        self._widget.pushInterchanging.setEnabled(False)

        self.__spin_boxes = []
        self.__servo_labels = []
        for i in range(31):
            try:
                spin_box = getattr(self._widget, f'doubleServo{i}')
                spin_box.valueChanged.connect(self._on_servo_angle_changed)
                self.__spin_boxes.append(spin_box)
                labels = getattr(self._widget, f'lineServo{i}')
                labels.setText("0.0")
                self.__servo_labels.append(labels)
            except AttributeError:
                continue

        self.__servo_to_frame_map = {
            'doubleServo7': 0,
            'doubleServo8': 1,
            'doubleServo9': 2,
            'doubleServo10': 3,
            'doubleServo11': 4,
            'doubleServo12': 5,
            'doubleServo19': 6,
            'doubleServo20': 7,
            'doubleServo21': 8,
            'doubleServo1': 9,
            'doubleServo2': 10,
            'doubleServo3': 11,
            'doubleServo4': 12,
            'doubleServo5': 13,
            'doubleServo6': 14,
            'doubleServo15': 15,
            'doubleServo16': 16,
            'doubleServo17': 17,
        }

        servo_list = []
        for i in range(24):
            servo_list.append(Servo(i, True, 200))

        self.__m_motion = Motion(servo_list)

        self.__pub = rospy.Publisher(
            'joint_group_position_controller/command', Float64MultiArray, queue_size=10)
        self.__sub = rospy.Subscriber('joint_states', JointState, self._on_update_servo_status)

    def shutdown_plugin(self) -> None:
        self.__sub.unregister()
        self.__pub.unregister()

    def _on_delete_list(self) -> None:
        delete_frames = self._widget.listFrame.selectedItems()
        if len(delete_frames) == 0:
            return

        for item in delete_frames:
            if self._widget.listFrame.count() != 1:
                frame_id = self._widget.listFrame.row(item) + 1
                ret = self.__m_motion.delete_frame(frame_id)
                if not ret:
                    return
                self._widget.listFrame.takeItem(self._widget.listFrame.row(item))
        # for i, frame_num in enumerate(range(1, self.__m_motion.frame_count() + 1)):
        #     self._widget.listFrame.item(i).setText(f'Frame{frame_num}')

        self._show_current_frame(self._widget.listFrame.currentItem())

    def _add_list(self) -> None:
        if self.__m_motion.add_frame():
            QListWidgetItem(self._widget.listFrame)
            current_frame_num = self._widget.listFrame.count()
            last_frame_num = current_frame_num - 1
            item = self._widget.listFrame.item(last_frame_num)
            item.setText(f'Frame{current_frame_num}')
            self._widget.listFrame.setCurrentRow(last_frame_num)

    def _show_current_frame(self, item: QListWidgetItem) -> None:
        self.show_servo_angles(self._widget.listFrame.row(item) + 1)
        self.show_frame_speed(self._widget.listFrame.row(item) + 1)

    def _on_push_send_angles(self) -> None:
        print('Send not implemented')

    def _on_servo_angle_changed(self, value: float) -> None:
        msg = Float64MultiArray()
        # left
        msg.data.append(-np.deg2rad(self._widget.doubleServo15.value()))
        msg.data.append(np.deg2rad(self._widget.doubleServo16.value()))
        msg.data.append(-np.deg2rad(self._widget.doubleServo17.value()))
        msg.data.append(-np.deg2rad(self._widget.doubleServo6.value()))
        msg.data.append(-np.deg2rad(self._widget.doubleServo5.value()))
        msg.data.append(-np.deg2rad(self._widget.doubleServo4.value()))
        msg.data.append(-np.deg2rad(self._widget.doubleServo3.value()))
        msg.data.append(-np.deg2rad(self._widget.doubleServo2.value()))
        msg.data.append(-np.deg2rad(self._widget.doubleServo1.value()))

        # right
        msg.data.append(-np.deg2rad(self._widget.doubleServo19.value()))
        msg.data.append(np.deg2rad(self._widget.doubleServo20.value()))
        msg.data.append(-np.deg2rad(self._widget.doubleServo21.value()))
        msg.data.append(-np.deg2rad(self._widget.doubleServo12.value()))
        msg.data.append(-np.deg2rad(self._widget.doubleServo11.value()))
        msg.data.append(-np.deg2rad(self._widget.doubleServo10.value()))
        msg.data.append(-np.deg2rad(self._widget.doubleServo9.value()))
        msg.data.append(-np.deg2rad(self._widget.doubleServo8.value()))
        msg.data.append(-np.deg2rad(self._widget.doubleServo7.value()))

        self.__pub.publish(msg)

        for spin_box in self.__spin_boxes:
            if spin_box == self.sender():
                iangle = int(value * 10)
                selected_items = self._widget.listFrame.selectedItems()
                if not selected_items:
                    print('No frame selected, no update')
                    return
                index = self.__servo_to_frame_map.get(spin_box.objectName())
                old_angle = self.__m_motion.frame_servo_angle(self._widget.listFrame.row(selected_items[0]) + 1, index)
                if old_angle is None:
                    return
                diff = iangle - old_angle
                for frame in selected_items:
                    frame_id = self._widget.listFrame.row(frame) + 1
                    if self._widget.listFrame.currentItem() == frame:
                        self.__m_motion.set_frame_servo_angle(frame_id, index, iangle)
                    else:
                        old_angle = self.__m_motion.frame_servo_angle(frame_id, index)
                        if old_angle is None:
                            return
                        self.__m_motion.set_frame_servo_angle(frame_id, index, old_angle + diff)

    def _on_vm_servo_angle_changed(self, value: float) -> None:
        pass

    def _on_write_frame_spd(self, value: int) -> None:
        selected_frame = self._widget.listFrame.selectedItems()
        if not selected_frame:
            print('No frame selected, cannot set frame speed')
            return
        for frame in selected_frame:
            frame_id = self._widget.listFrame.row(frame) + 1
            self.__m_motion.set_frame_wait_time(frame_id, value * 10)

    def _on_update_servo_status(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            if name == 'head_yaw_joint':
                continue
            elif name == 'left_ankle_roll_joint':
                self._widget.lineServo1.setText(str(np.round(np.rad2deg(position), 1)))
            elif name == 'left_ankle_pitch_joint':
                self._widget.lineServo2.setText(str(np.round(np.rad2deg(position), 1)))
            elif name == 'left_knee_pitch_joint':
                self._widget.lineServo3.setText(str(np.round(np.rad2deg(position), 1)))
            elif name == 'left_waist_pitch_joint':
                self._widget.lineServo4.setText(str(np.round(np.rad2deg(position), 1)))
            elif name == 'left_waist_roll_joint':
                self._widget.lineServo5.setText(str(np.round(np.rad2deg(position), 1)))
            elif name == 'left_waist_yaw_joint':
                self._widget.lineServo6.setText(str(np.round(np.rad2deg(position), 1)))
            elif name == 'right_ankle_roll_joint':
                self._widget.lineServo7.setText(str(np.round(np.rad2deg(position), 1)))
            elif name == 'right_ankle_pitch_joint':
                self._widget.lineServo8.setText(str(np.round(np.rad2deg(position), 1)))
            elif name == 'right_knee_pitch_joint':
                self._widget.lineServo9.setText(str(np.round(np.rad2deg(position), 1)))
            elif name == 'right_waist_pitch_joint':
                self._widget.lineServo10.setText(str(np.round(np.rad2deg(position), 1)))
            elif name == 'right_waist_roll_joint':
                self._widget.lineServo11.setText(str(np.round(np.rad2deg(position), 1)))
            elif name == 'right_waist_yaw_joint':
                self._widget.lineServo12.setText(str(np.round(np.rad2deg(position), 1)))
            elif name == 'left_shoulder_pitch_joint':
                self._widget.lineServo15.setText(str(np.round(np.rad2deg(position), 1)))
            elif name == 'left_shoulder_roll_joint':
                self._widget.lineServo16.setText(str(np.round(np.rad2deg(position), 1)))
            elif name == 'left_elbow_pitch_joint':
                self._widget.lineServo17.setText(str(np.round(np.rad2deg(position), 1)))
            elif name == 'right_shoulder_pitch_joint':
                self._widget.lineServo19.setText(str(np.round(np.rad2deg(position), 1)))
            elif name == 'right_shoulder_roll_joint':
                self._widget.lineServo20.setText(str(np.round(np.rad2deg(position), 1)))
            elif name == 'right_elbow_pitch_joint':
                self._widget.lineServo21.setText(str(np.round(np.rad2deg(position), 1)))

    def _set_spin_value_from_current_servo_angles(self) -> None:
        pass

    def _on_load_motion_file(self) -> None:
        filename, _ = QFileDialog.getOpenFileName(self._widget, 'Open Motion File', '', 'Motion Files (*.txt)')
        if filename:
            print(f'Load motion file: {filename}')
            servo_list = []
            for i in range(24):
                servo_list.append(Servo(i, True, 200))
            self.__m_motion = Motion(servo_list)
            self.__m_motion.load_from(filename, '')
            self._widget.listFrame.clear()
            self._show_current_frame(self._widget.listFrame.currentItem())
            for i in range(self.__m_motion.frame_count()):
                QListWidgetItem(self._widget.listFrame)
                item = self._widget.listFrame.item(i)
                item.setText(f'Frame{i + 1}')

    def _on_save_motion_file(self) -> None:
        filename, _ = QFileDialog.getSaveFileName(self._widget, 'Save Motion File', '', 'Motion Files (*.txt)')
        if filename:
            print(f'Save motion file: {filename}')
            self.__m_motion.save_to(filename, '')

    def _on_push_copy_frame(self) -> None:
        print('Copy frame not implemented')

    def _on_push_paste_frame(self) -> None:
        print('Paste frame not implemented')

    def _on_spin_step_changed(self, value: float) -> None:
        for spin_box in self.__spin_boxes:
            spin_box.setSingleStep(value)

    def _on_push_interchanging(self) -> None:
        print('interchanging not implemented')

    def show_servo_angles(self, frame_id: int) -> None:
        for spin_box in self.__spin_boxes:
            name = spin_box.objectName()
            index = self.__servo_to_frame_map.get(name)
            if index is None:
                continue
            servo_angle = self.__m_motion.frame_servo_angle(frame_id, index)
            if servo_angle is None:
                rospy.logerr(f'frame {frame_id} is not valid?')
                return
            spin_box.blockSignals(True)
            spin_box.setValue(servo_angle / 10.0)
            spin_box.blockSignals(False)

    def show_frame_speed(self, frame_id: int) -> None:
        self._widget.spinSpd.blockSignals(True)
        self._widget.spinSpd.setValue(self.__m_motion.frame_wait_time(frame_id) * 0.1)
        self._widget.spinSpd.blockSignals(False)
