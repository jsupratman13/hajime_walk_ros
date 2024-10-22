/*********************************************************************
 * Copyright (c) 2024 Joshua Supratman
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ********************************************************************/
// gyro sensor

#ifndef GYRO_HPP_
#define GYRO_HPP_

#include <memory>
#include "acc.hpp"
#include "kine.hpp"
#include "joy.hpp"
#include "serv.hpp"
#include "sq_walk.hpp"

namespace hr46
{
// forward declaration
class Acc;
class Joy;
class Kine;
class Serv;
class SqWalk;

struct st_xp_gyro
{
  float kp1_foot;            // gain (roll)
  float kp2_foot;            // gain (pitch)
  float kp1_hip;             // gain (roll)
  float kp2_hip;             // gain (pitch)
  float kp1_arm;             // gain (roll)
  float kp2_arm;             // gain (pitch)
  float kp2_waist;           // gain (pitch)
  float kp3_waist;           // gain (yaw)
  float gyro_k1;             // [(deg/sec)/V]
  float gyro_k2;             // [(deg/sec)/V]
  float gyro_k3;             // [(deg/sec)/V]
  float ad_volt_offset1;     // [deg/sec]
  float ad_volt_offset2;     // [deg/sec]
  float ad_volt_offset3;     // [deg/sec]
  float t1;                  // [msec]
  float t2;                  // [msec]
  float gyro_data3_flt2_t1;  // [msec]
  float yaw_cntl_gain;       // [-]
  float yaw_cntl_dead;       // [deg]
  float yaw_cntl_theta;      // [deg]
  float gyro_omega;          // [1/rad]
  float fall_roll_deg1;      // fall check [deg]
  float fall_pitch_deg1;     // fall check [deg]
};

struct st_xv_gyro
{
  float gyro_data1;       // [deg/sec]
  float gyro_data2;       // [deg/sec]
  float gyro_data3;       // [deg/sec]
  float gyro_data1_d;     // [deg/sec/sec]
  float gyro_data2_d;     // [deg/sec/sec]
  float gyro_data3_d;     // [deg/sec/sec]
  float gyro_data1_flt;   // [deg/sec]
  float gyro_data2_flt;   // [deg/sec]
  float gyro_data3_flt;   // [deg/sec]
  float gyro_data3_flt2;  // [deg/sec]
  float gyro_roll;        // roll [deg]
  float gyro_pitch;       // pitch [deg]
  float gyro_yaw;         // yaw [deg]
  float gyro_yaw2;        // yaw [deg]
  float gyro_roll2;       // roll [deg]
  float gyro_pitch2;      // pitch [deg]
  float deg_foot_roll;    // [deg]
  float deg_foot_pitch;   // [deg]
  float deg_hip_roll;     // [deg]
  float deg_hip_pitch;    // [deg]
  float deg_arm_roll;     // [deg]
  float deg_arm_pitch;    // [deg]
  float deg_waist_pitch;  // [deg]
  float deg_waist_yaw;    // [deg]
  float yaw_cntl_ref;     // reference [deg]
  float yaw_cntl_fb;      // feedback [deg]
  float quaternion[4];    // quaternion (w, x, y, z)
};

struct st_flag_gyro
{
  short vib;        // vibraion feedback
  short vib_auto;   // auto/manual
  short vib_manu;   // manual setting
  short zero;       // offset clear
  short yaw_cntl;   // yaw control
  short fall_on;    // fall status
  short fall_cntl;  // fall servo off
  short fall;       // fall
};

class Gyro
{
public:
  st_xp_gyro xp_gyro_;
  st_xv_gyro xv_gyro_{ 0.0 };
  st_flag_gyro flag_gyro_;
  short flag_auto_gyro_offset_;

  void gyro_fun();
  void gyro_init();
  void gyro_cntr_fun();
  void gyro_yaw_cntr_fun();
  void gyro_offset_tune();
  void PostureControl();

  short w_pulse1_ = 0;
  float count_ = 0.0f;

  // AccSharedPtr acc_;
  std::shared_ptr<Acc> acc_;
  // JoySharedPtr joy_;
  std::shared_ptr<Joy> joy_;
  // KineSharedPtr kine_;
  std::shared_ptr<Kine> kine_;
  // ServSharedPtr serv_;
  std::shared_ptr<Serv> serv_;
  // SqWalkSharedPtr sq_walk_;
  std::shared_ptr<SqWalk> sq_walk_;
};

using GyroSharedPtr = std::shared_ptr<Gyro>;
}  // namespace hr46

#endif  // GYRO_HPP_
