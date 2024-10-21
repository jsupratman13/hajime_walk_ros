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

// acceleration sensor
#include <algorithm>
#include <cmath>
#include "hr46/acc.hpp"
#include "hr46/func.hpp"
#include "hr46/var.hpp"

namespace hr46
{

void Acc::acc_func()
{
  // low pass filter
  xv_acc_.acc_data1_d = hr46::diff(xv_acc_.acc_data1, xp_acc_.t1, xp_acc_.t2, &xv_acc_.acc_data1_flt);
  xv_acc_.acc_data2_d = hr46::diff(xv_acc_.acc_data2, xp_acc_.t1, xp_acc_.t2, &xv_acc_.acc_data2_flt);
  xv_acc_.acc_data3_d = hr46::diff(xv_acc_.acc_data3, xp_acc_.t1, xp_acc_.t2, &xv_acc_.acc_data3_flt);

  // calculate angle from acc
  xv_acc_.acc_pitch = hr46::rad2deg(std::asin(std::clamp(xv_acc_.acc_data1_flt, -1.0f, 1.0f)));
  xv_acc_.acc_roll = hr46::rad2deg(std::asin(std::clamp(xv_acc_.acc_data2_flt, -1.0f, 1.0f)));
  xv_acc_.acc_pitch_d = hr46::rad2deg(std::asin(std::clamp(xv_acc_.acc_data1_d, -1.0f, 1.0f)));
  xv_acc_.acc_roll_d = hr46::rad2deg(std::asin(std::clamp(xv_acc_.acc_data2_d, -1.0f, 1.0f)));

  // check fall down for servo off
  xv_acc_.acc_pitch2 = hr46::rad2deg(std::asin(std::clamp(xv_acc_.acc_data1, -1.0f, 1.0f)));
  xv_acc_.acc_roll2 = hr46::rad2deg(std::asin(std::clamp(xv_acc_.acc_data2, -1.0f, 1.0f)));

#ifdef ACC_SENSOR
  // check fall down for auto standing up
  hr46::TIMER(xv_acc_.acc_pitch >= xp_acc_.fall_fwd, flag_acc_.fall_fwd_on, xp_acc_.fall_check_time,
              xv_acc_.fall_fwd_work);
  hr46::TIMER(xv_acc_.acc_pitch <= xp_acc_.fall_bwd, flag_acc_.fall_bwd_on, xp_acc_.fall_check_time,
              xv_acc_.fall_bwd_work);
  hr46::TIMER(xv_acc_.acc_roll >= xp_acc_.fall_right, flag_acc_.fall_right_on, xp_acc_.fall_check_time,
              xv_acc_.fall_right_work);
  hr46::TIMER(xv_acc_.acc_roll <= xp_acc_.fall_left, flag_acc_.fall_left_on, xp_acc_.fall_check_time,
              xv_acc_.fall_left_work);

  // auto standing up after fall down
  if (flag_acc_.fall_fwd_on == ON && flag_acc_.fall_bwd_on == OFF)
  {
    flag_acc_.standup_select = STANDUP_FWD;
  }
  else if (flag_acc_.fall_bwd_on == ON && flag_acc_.fall_fwd_on == OFF)
  {
    flag_acc_.standup_select = STANDUP_BWD;
  }
  else if (flag_acc_.fall_right_on == ON && flag_acc_.fall_left_on == OFF)
  {
    flag_acc_.standup_select = STANDUP_RIGHT;
  }
  else if (flag_acc_.fall_left_on == ON && flag_acc_.fall_right_on == OFF)
  {
    flag_acc_.standup_select = STANDUP_LEFT;
  }
  else
  {
    flag_acc_.standup_select = OFF;
  }

  // check fall down
  if (xv_acc_.acc_roll >= xp_acc_.fall_roll || xv_acc_.acc_roll <= -xp_acc_.fall_roll)
  {
    if (xv_acc_.acc_pitch >= 0)
      flag_acc_.fall = 3;
    else
      flag_acc_.fall = 4;
  }
  else if ((xv_acc_.acc_pitch >= xp_acc_.fall_pitch) || ((xv_acc_.acc_pitch >= xp_acc_.fall_pitch_oblique) &&
                                                         (std::fabs(xv_acc_.acc_roll) >= xp_acc_.fall_roll_oblique)))
  {
    flag_acc_.fall = 1;
  }
  else if ((xv_acc_.acc_pitch <= -xp_acc_.fall_pitch) || ((xv_acc_.acc_pitch <= -xp_acc_.fall_pitch_oblique) &&
                                                          (std::fabs(xv_acc_.acc_roll) >= xp_acc_.fall_roll_oblique)))
  {
    flag_acc_.fall = 2;
  }
  else if ((std::fabs(xv_acc_.acc_pitch) < 0.5 * xp_acc_.fall_pitch) &&
           (std::fabs(xv_acc_.acc_roll) < 0.5 * xp_acc_.fall_roll))
  {
    flag_acc_.fall = 0;
    flag_motion_accept_ = ON;
    // if (flag_ukemi_start_ != ON)
    //   flag_motion_accept_ = ON;
  }

  hr46::TIMER(flag_ukemi_start_, flag_restart_, 1000, flag_restart_work_);
  if (flag_restart_ == ON)
  {
    flag_restart_ = OFF;
    flag_ukemi_start_ = OFF;
    flag_servo_restart_ = OFF;
    flag_restart_work_ = 0;
    flag_motion_accept_work_ = 0;
    flag_motion_accept_ = ON;
  }
  else if (flag_ukemi_ == 1)
  {
    hr46::TIMER(flag_ukemi_start_, flag_ukemi_finished_, 100, flag_ukemi_finished_work_);
    hr46::TIMER(flag_ukemi_start_, flag_motion_accept_, 300, flag_motion_accept_work_);
    if ((flag_ukemi_finished_ == ON) && (flag_servo_restart_ == OFF))
    {
      flag_ukemi_finished_ = OFF;
      flag_servo_restart_ = ON;
      motion_->sq_flag_.start = ON;
      calc_mv_->xv_mv_.count = 0;
      flag_ukemi_finished_work_ = 0;
    }
    else if (flag_acc_.fall_fwd_on || flag_acc_.fall_bwd_on || flag_acc_.fall_right_on || flag_acc_.fall_left_on)
    {
      if (flag_ukemi_start_ == OFF)
      {
        flag_ukemi_start_ = ON;
        // TODO: flag_servo_off_ = ON;
        flag_motion_accept_ = OFF;
      }
    }
  }
#endif
}

void Acc::acc_init()
{
  // acc sensor = ADXL300
  xp_acc_.acc_k1 = 3.1f;             // 3.03 [G/V] = 0.33V/G
  xp_acc_.acc_k2 = -3.1f;            // 3.03 [G/V] = 0.33V/G
  xp_acc_.acc_k3 = 3.1f;             // 3.03 [G/V] = 0.33V/G
  xp_acc_.ad_volt_offset1 = -1.65f;  // -1.65[V]
  xp_acc_.ad_volt_offset2 = -1.65f;  // -1.65[V]
  xp_acc_.ad_volt_offset3 = -1.65f;  // -1.65[V]
  xp_acc_.t1 = 100.f;                // [msec]
  xp_acc_.t2 = 300.f;                // diff T2=3*T1 [msec]

  xp_acc_.fall_fwd = 40.0f;                   // [deg]
  xp_acc_.fall_bwd = -40.0f;                  // [deg]
  xp_acc_.fall_right = 40.0f;                 // [deg]
  xp_acc_.fall_left = -40.0f;                 // [deg]
  xp_acc_.fall_check_time = 1.0f * RTC_TICK;  // 1 [sec]
  xp_acc_.fall_pitch = 40.0f;                 // [deg]
  xp_acc_.fall_roll = 40.0f;                  // [deg]
  xp_acc_.fall_pitch_oblique = 30.0f;         // [deg]
  xp_acc_.fall_roll_oblique = 40.0f;          // [deg]

  xv_acc_.acc_data1 = 0.0f;
  xv_acc_.acc_data2 = 0.0f;
  xv_acc_.acc_data3 = 0.0f;
  xv_acc_.acc_data1_d = 0.0f;
  xv_acc_.acc_data2_d = 0.0f;
  xv_acc_.acc_data3_d = 0.0f;
  xv_acc_.acc_data1_flt = 0.0f;
  xv_acc_.acc_data2_flt = 0.0f;
  xv_acc_.acc_data3_flt = 0.0f;
  xv_acc_.acc_pitch = 0.0f;
  xv_acc_.acc_roll = 0.0f;
  xv_acc_.acc_pitch_d = 0.0f;
  xv_acc_.acc_roll_d = 0.0f;
  xv_acc_.acc_pitch2 = 0.0f;
  xv_acc_.acc_roll2 = 0.0f;
  xv_acc_.fall_bwd_work = 0;
  xv_acc_.fall_fwd_work = 0;
  xv_acc_.fall_right_work = 0;
  xv_acc_.fall_left_work = 0;

  flag_restart_work_ = 0;
  flag_ukemi_finished_work_ = 0;
  flag_motion_accept_work_ = 0;

  flag_acc_.zero = OFF;
  flag_acc_.fall_fwd_on = OFF;
  flag_acc_.fall_bwd_on = OFF;
  flag_acc_.fall_right_on = OFF;
  flag_acc_.fall_left_on = OFF;
  flag_acc_.fall = OFF;
  flag_acc_.standup_select = OFF;

  flag_restart_ = OFF;
  flag_ukemi_finished_ = OFF;
  flag_ukemi_start_ = OFF;
  flag_servo_restart_ = OFF;
  flag_motion_accept_ = ON;

  touchdown_gain_ = 0.0f;
}
}  // namespace hr46
