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
#include "hr46/sq_start.hpp"
#include "hr46/var.hpp"

namespace hr46
{

void SqStart::sq_start_init()
{
  flag_md_start_end_ = OFF;  // mode end flag
  mode_sq_start_prev_ = -1;
  mode_sq_start_ = 0;
  servo_period_ = 0;
}

int SqStart::sq_start()
{
  if (mode_sq_start_ != mode_sq_start_prev_)
  {
    switch (mode_sq_start_)
    {
      case 0:
        // servo_reset
        // B3MAllReset( B3M_RESET_AFTER_TIME );
        mode_sq_time_ = 0.06f;
        break;

      case 1:
        // servo_free_mode
        // write_servo_rs_all( B3M_SERVO_SERVO_MODE, &xp_servo_rs.free_mode[0], 1 );
        mode_sq_time_ = 0.02f;
        break;

      case 2:
        // write_servo_rs_all( B3M_SYSTEM_DEADBAND_WIDTH, &xp_servo_rs.deadband_width[0], 2 );
        mode_sq_time_ = 0.02f;
        break;

      case 3:
        // servo control mode
        // write_servo_rs_all( B3M_SERVO_SERVO_MODE, &xp_servo_rs.run_or_control[0], 1 );
        mode_sq_time_ = 0.02f;
        break;

      case 4:
        // servo_trajectory_even
        // write_servo_rs_all( B3M_SERVO_RUN_MODE, &xp_servo_rs.trajectory_even[0], 1 );
        mode_sq_time_ = 0.02f;
        break;

      case 5:
        // write_servo_normal_mode
        // write_servo_rs_all( B3M_SERVO_SERVO_MODE, &xp_servo_rs.normal_mode[0], 1 );
        mode_sq_time_ = 0.02f;
        break;

      case 6:
        // write_servo_rs_all( B3M_CONTROL_KP0, &xp_servo_rs.control_kp0[0], 2 );
        mode_sq_time_ = 0.02f;
        break;

      case 7:
        // write_servo_rs_all( B3M_CONTROL_KD0, &xp_servo_rs.control_kd1[0], 2 );
        mode_sq_time_ = 0.02f;
        break;

      case 8:
        // write_servo_rs_all( B3M_CONTROL_KI0, &xp_servo_rs.control_ki0[0], 2 );
        mode_sq_time_ = 0.02f;
        break;

      case 9:
        // write_servo_rs_all( B3M_CONTROL_STATIC_FRICTION0, &xp_servo_rs.control_static_friction0[0], 2 );
        mode_sq_time_ = 0.02f;
        break;

      case 10:
        // write_servo_rs_all( B3M_CONTROL_DYNAMIC_FRICTION0, &xp_servo_rs.control_dynamic_friction0[0], 2 );
        mode_sq_time_ = 0.02f;
        break;

      case 11:
        // write_servo_rs_all( B3M_CONTROL_KP1, &xp_servo_rs.control_kp1[0], 2 );
        mode_sq_time_ = 0.02f;
        break;

      case 12:
        // write_servo_rs_all( B3M_CONTROL_KD1, &xp_servo_rs.control_kd1[0], 2 );
        mode_sq_time_ = 0.02f;
        break;

      case 13:
        // write_servo_rs_all( B3M_CONTROL_KI1, &xp_servo_rs.control_ki1[0], 2 );
        mode_sq_time_ = 0.02f;
        break;

      case 14:
        // write_servo_rs_all(B3M_CONTROL_STATIC_FRICTION1, &xp_servo_rs.control_static_friction1[0], 2);
        mode_sq_time_ = 0.02f;
        break;

      case 15:
        // write_servo_rs_all( B3M_CONTROL_DYNAMIC_FRICTION1   , &xp_servo_rs.control_dynamic_friction1[0], 2 );
        mode_sq_time_ = 0.02f;
        break;

      case 16:
        // write_servo_rs_all( B3M_CONTROL_KP2, &xp_servo_rs.control_kp2[0], 2 );
        mode_sq_time_ = 0.02f;
        break;

      case 17:
        // write_servo_rs_all( B3M_CONTROL_KD2, &xp_servo_rs.control_kd2[0], 2 );
        mode_sq_time_ = 0.02f;
        break;

      case 18:
        // write_servo_rs_all( B3M_CONTROL_KI2, &xp_servo_rs.control_ki2[0], 2 );
        mode_sq_time_ = 0.02f;
        break;

      case 19:
        // write_servo_rs_all( B3M_CONTROL_STATIC_FRICTION2, &xp_servo_rs.control_static_friction2[0], 2 );
        mode_sq_time_ = 0.02f;
        break;

      case 20:
        // write_servo_rs_all( B3M_CONTROL_DYNAMIC_FRICTION2, &xp_servo_rs.control_dynamic_friction2[0], 2 );
        mode_sq_time_ = 0.02f;
        break;

      case 21:
        // write servo gain change
        // write_servo_rs_all( B3M_CONTROL_GAIN_PRESETNO, &xp_servo_rs.control_gain_presetno[0], 1 );
        mode_sq_time_ = 0.02f;
        break;

      case 22:
        mode_sq_time_ = 3.0f;
        break;

      case 23:
        // servo trajectory normal
        // write_servo_rs_all(B3M_SERVO_RUN_MODE, &xp_servo_rs.trajectory_normal[0], 1);
        mode_sq_time_ = 0.02f;
        break;

      case 24:
        motion_->sq_flag_.start = OFF;
        flag_md_start_end_ = ON;
        break;

      default:
        break;
    }
  }
  mode_sq_start_prev_ = mode_sq_start_;

  if ((!flag_md_start_end_) && (mode_sq_time_ > EPS_TIME))
  {
    mode_sq_start_++;
  }
  else
  {
    mode_sq_time_ -= RTC_TIME_SEC;
  }

  return flag_md_start_end_;  // flag to check if the mode is finished
}

}  // namespace hr46
