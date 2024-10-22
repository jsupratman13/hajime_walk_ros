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
// acceration sensor
#ifndef ACC_HPP_
#define ACC_HPP_

#include <memory>
#include <thread>
#include "calc_mv.hpp"
#include "motion.hpp"

namespace hr46
{
// forward declaration
class CalcMv;
class Motion;

#define ACC_SENSOR  // use acc sensor

enum
{
  STANDUP_FWD = 1,
  STANDUP_BWD = 2,
  STANDUP_RIGHT = 3,
  STANDUP_LEFT = 4
};

struct st_xp_acc
{
  float acc_k1;              // x[G/bit]
  float acc_k2;              // y[G/bit]
  float acc_k3;              // z[G/bit]
  float ad_volt_offset1;     // x[G]
  float ad_volt_offset2;     // y[G]
  float ad_volt_offset3;     // z[G]
  float t1;                  // filter time constant [msec]
  float t2;                  // filter time constant [msec]
  float fall_fwd;            // fall forward check [deg]
  float fall_bwd;            // fall backward check [deg]
  float fall_right;          // fall right check [deg]
  float fall_left;           // fall left check [deg]
  float fall_check_time;     // fall check time [bit]
  float fall_pitch;          // fall pitch [deg]
  float fall_roll;           // fall roll [deg]
  float fall_pitch_oblique;  // fall pitch [deg]
  float fall_roll_oblique;   // fall roll [deg]
};

struct st_xv_acc
{
  float acc_data1;        // x[G]
  float acc_data2;        // y[G]
  float acc_data3;        // z[G]
  float acc_data1_d;      // x[G/sec]
  float acc_data2_d;      // y[G/sec]
  float acc_data3_d;      // z[G/sec]
  float acc_data1_flt;    // filer x[G]
  float acc_data2_flt;    // filter y[G]
  float acc_data3_flt;    // filter z[G]
  float acc_pitch;        // pitch angle [deg]
  float acc_roll;         // roll angle [deg]
  float acc_pitch_d;      // pitch angle velocity [deg/s]
  float acc_roll_d;       // roll angle velocity [deg/s]
  float acc_pitch2;       // pitch angle [deg]
  float acc_roll2;        // roll angle [deg]
  short fall_fwd_work;    // [bit]
  short fall_bwd_work;    // [bit]
  short fall_right_work;  // [bit]
  short fall_left_work;   // [bit]
};

struct st_flag_acc
{
  short zero;            // offset clear
  short fall_fwd_on;     // fall forward
  short fall_bwd_on;     // fall backward
  short fall_right_on;   // fall right
  short fall_left_on;    // fall left
  short fall;            // fall
  short standup_select;  // standup selection
};

class Acc
{
public:
  st_xp_acc xp_acc_;
  st_xv_acc xv_acc_;
  st_flag_acc flag_acc_;
  short flag_restart_;
  short flag_ukemi_finished_;
  short flag_ukemi_start_;
  short flag_servo_restart_;
  short flag_restart_work_;
  short flag_ukemi_;
  short flag_ukemi_finished_work_;
  short flag_motion_accept_;
  short flag_motion_accept_work_;
  float touchdown_gain_;

  void acc_init();
  void acc_func();

  // CalcMvSharedPtr calc_mv_;
  // MotionSharedPtr motion_;
  std::shared_ptr<CalcMv> calc_mv_;
  std::shared_ptr<Motion> motion_;
};

using AccSharedPtr = std::shared_ptr<Acc>;

}  // namespace hr46

#endif  // ACC_HPP_
