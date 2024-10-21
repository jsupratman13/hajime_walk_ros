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
#include <algorithm>

#include "hr46/gyro.hpp"

#include "hr46/func.hpp"
#include "hr46/var.hpp"

namespace hr46
{

void Gyro::gyro_fun()
{
  float _w_gyro_roll2;
  float _w_gyro_pitch2;
  const float wakeup_time = 0.5;

  // low pass filter
  xv_gyro_.gyro_data1_d = hr46::diff(xv_gyro_.gyro_data1, xp_gyro_.t1, xp_gyro_.t2, &xv_gyro_.gyro_data1_flt);
  xv_gyro_.gyro_data2_d = hr46::diff(xv_gyro_.gyro_data2, xp_gyro_.t1, xp_gyro_.t2, &xv_gyro_.gyro_data2_flt);
  xv_gyro_.gyro_data3_d = hr46::diff(xv_gyro_.gyro_data3, xp_gyro_.t1, xp_gyro_.t2, &xv_gyro_.gyro_data3_flt);
  xv_gyro_.gyro_data3_flt2 =
      hr46::filterf(xv_gyro_.gyro_data3_flt, xv_gyro_.gyro_data3_flt2, xp_gyro_.gyro_data3_flt2_t1);

  // calculate angle from gyro sensor
  // xv_gyro_.gyro_roll = hr46::integrator_f(xv_gyro_.gyro_data1_flt / RTC_TIME, xv_gyro_.gyro_roll, 180.f);
  // xv_gyro_.gyro_pitch = hr46::integrator_f(xv_gyro_.gyro_data2_flt / RTC_TIME, xv_gyro_.gyro_pitch, 180.f);
  // xv_gyro_.gyro_yaw = hr46::integrator_f(xv_gyro_.gyro_data3_flt / RTC_TIME, xv_gyro_.gyro_yaw, 180.f);
  // xv_gyro_.gyro_yaw2 = hr46::integrator_f(xv_gyro_.gyro_data3_flt / RTC_TIME, xv_gyro_.gyro_yaw2, 180.f);  //
  // 1800->180 130430

  // check fall down for servo off
  //   xv_gyro_.gyro_roll2 = (1.0f - xp_gyro_.gyro_omega * RTC_TIME_SEC) * xv_gyro_.gyro_roll2 +
  //                        xp_gyro_.gyro_omega * RTC_TIME_SEC * xv_acc.acc_roll2 - RTC_TIME_SEC * xv_gyro_.gyro_data1;

  //   xv_gyro_.gyro_pitch2 = (1.0f - xp_gyro_.gyro_omega * RTC_TIME_SEC) * xv_gyro_.gyro_pitch2 +
  //                        xp_gyro_.gyro_omega * RTC_TIME_SEC * xv_acc.acc_pitch2 - RTC_TIME_SEC * xv_gyro_.gyro_data2;

  _w_gyro_roll2 = std::fabs(xv_gyro_.gyro_roll2);
  _w_gyro_pitch2 = std::fabs(xv_gyro_.gyro_pitch2);

  if (_w_gyro_roll2 > xp_gyro_.fall_roll_deg1 || _w_gyro_pitch2 > xp_gyro_.fall_pitch_deg1)
  {
    flag_gyro_.fall_on = ON;
  }
  else
  {
    flag_gyro_.fall_on = OFF;
  }

  if (hr46::pulse1((flag_gyro_.fall_cntl && flag_gyro_.fall_on), &w_pulse1_))
  {
    // TODO: flag_servo_off = ON;
  }

  // check fall down
  if (xv_gyro_.gyro_roll2 >= xp_gyro_.fall_roll_deg1)
  {
    count_ += RTC_TIME_SEC;
    if (count_ > wakeup_time)
    {
      flag_gyro_.fall = STANDUP_LEFT;
    }
  }
  else if (xv_gyro_.gyro_roll2 <= -xp_gyro_.fall_roll_deg1)
  {
    count_ += RTC_TIME_SEC;
    if (count_ > wakeup_time)
    {
      flag_gyro_.fall = STANDUP_RIGHT;
    }
  }
  else if (xv_gyro_.gyro_pitch2 >= xp_gyro_.fall_pitch_deg1)
  {
    count_ += RTC_TIME_SEC;
    if (count_ > wakeup_time)
    {
      flag_gyro_.fall = STANDUP_BWD;
    }
  }
  else if (xv_gyro_.gyro_pitch2 <= -xp_gyro_.fall_pitch_deg1)
  {
    count_ += RTC_TIME_SEC;
    if (count_ > wakeup_time)
    {
      flag_gyro_.fall = STANDUP_FWD;
    }
  }
  else if ((std::fabs(xv_gyro_.gyro_roll2) < 0.5 * xp_gyro_.fall_roll_deg1) &&
           (std::fabs(xv_gyro_.gyro_pitch2) < 0.5 * xp_gyro_.fall_pitch_deg1))
  {
    flag_gyro_.fall = 0;
    acc_->flag_motion_accept_ = ON;
    count_ = 0.0f;
  }
}

void Gyro::gyro_init()
{
  xp_gyro_.kp1_foot = 0.0f;
  xp_gyro_.kp2_foot = 0.0f;
  xp_gyro_.kp1_hip = 0.0f;
  xp_gyro_.kp2_hip = 0.0f;
  xp_gyro_.kp1_arm = 0.1f;
  xp_gyro_.kp2_arm = 0.1f;
  xp_gyro_.kp2_waist = 0.0f;
  xp_gyro_.kp3_waist = 0.0f;
  xp_gyro_.yaw_cntl_gain = 1.0f;
  xp_gyro_.yaw_cntl_dead = 2.0f;
  xp_gyro_.yaw_cntl_theta = 5.0f;

  // IDG500
  xp_gyro_.gyro_k1 = 1000.0f;          // ? [deg/sec]
  xp_gyro_.gyro_k2 = 1000.0f;          // ? [deg/sec]
  xp_gyro_.ad_volt_offset1 = -1.350f;  // -1.35[V]
  xp_gyro_.ad_volt_offset2 = -1.350f;  // -1.35[V]

  // CRS03-04
  xp_gyro_.gyro_k3 = 100.0f;         // [deg/sec]
  xp_gyro_.ad_volt_offset3 = -2.5f;  // -2.5[V]

  xp_gyro_.t1 = 5.f;                     // [msec]
  xp_gyro_.t2 = 15.f;                    // diff T2=3*T1 [msec]
  xp_gyro_.gyro_data3_flt2_t1 = 1000.f;  // [msec]

  xp_gyro_.gyro_omega = 0.3f;        // 1/T : time constant = 3[s]
  xp_gyro_.fall_roll_deg1 = 50.0f;   // fall check [deg]
  xp_gyro_.fall_pitch_deg1 = 55.0f;  // fall check [deg]

  xv_gyro_.gyro_data1 = 0.0f;
  xv_gyro_.gyro_data2 = 0.0f;
  xv_gyro_.gyro_data3 = 0.0f;
  xv_gyro_.gyro_data1_d = 0.0f;
  xv_gyro_.gyro_data2_d = 0.0f;
  xv_gyro_.gyro_data3_d = 0.0f;
  xv_gyro_.gyro_data1_flt = 0.0f;
  xv_gyro_.gyro_data2_flt = 0.0f;
  xv_gyro_.gyro_data3_flt = 0.0f;
  xv_gyro_.gyro_data3_flt2 = 0.0f;
  xv_gyro_.gyro_roll = 0.0f;
  xv_gyro_.gyro_pitch = 0.0f;
  xv_gyro_.gyro_yaw = 0.0f;
  xv_gyro_.gyro_yaw2 = 0.0f;
  xv_gyro_.gyro_roll2 = 0.0f;
  xv_gyro_.gyro_pitch2 = 0.0f;
  xv_gyro_.deg_foot_roll = 0.0f;
  xv_gyro_.deg_foot_pitch = 0.0f;
  xv_gyro_.deg_hip_roll = 0.0f;
  xv_gyro_.deg_hip_pitch = 0.0f;
  xv_gyro_.deg_arm_roll = 0.0f;
  xv_gyro_.deg_arm_pitch = 0.0f;
  xv_gyro_.deg_waist_pitch = 0.0f;
  xv_gyro_.deg_waist_yaw = 0.0f;
  xv_gyro_.yaw_cntl_ref = 0.0f;
  xv_gyro_.yaw_cntl_fb = 0.0f;

  flag_gyro_.vib = OFF;
  flag_gyro_.vib_auto = ON;
  flag_gyro_.vib_manu = OFF;
  flag_gyro_.zero = OFF;
  flag_gyro_.yaw_cntl = OFF;
  flag_gyro_.fall_on = OFF;
  flag_gyro_.fall_cntl = OFF;
  flag_auto_gyro_offset_ = ON;

  xv_gyro_.quaternion[0] = 1.0f;
  xv_gyro_.quaternion[1] = 0.0f;
  xv_gyro_.quaternion[2] = 0.0f;
  xv_gyro_.quaternion[3] = 0.0f;
}

void Gyro::gyro_cntr_fun()
{
  float w;
  float w2;
  float w3;

  w = xp_gyro_.kp1_foot * xv_gyro_.gyro_data1_flt;
  xv_gyro_.deg_foot_roll = std::clamp(w, -20.0f, 20.0f);

  w = xp_gyro_.kp2_foot * xv_gyro_.gyro_data2_flt;
  xv_gyro_.deg_foot_pitch = std::clamp(w, -20.0f, 20.0f);

  // add touchdown point tuning
  // if( flag_moving == STATE_WALKING )
  // w = xp_gyro_.kp1_hip * xv_gyro_.gyro_data1_flt - touchdown_gain * xv_acc.acc_roll;
  // else
  w = xp_gyro_.kp1_hip * xv_gyro_.gyro_data1_flt;
  xv_gyro_.deg_hip_roll = std::clamp(w, -10.0f, 10.0f);

  // if( flag_moving == STATE_WALKING )
  // w = xp_gyro_.kp2_hip * xv_gyro_.gyro_data2_flt - touchdown_gain * xv_acc.acc_pitch;
  // else
  w = xp_gyro_.kp2_hip * xv_gyro_.gyro_data2_flt;
  xv_gyro_.deg_hip_pitch = std::clamp(w, -10.0f, 10.0f);
  // end touchdown point tuning

  w = xp_gyro_.kp1_arm * xv_gyro_.gyro_data1_flt;
  xv_gyro_.deg_arm_roll = std::clamp(w, -30.0f, 30.0f);

  w = xp_gyro_.kp2_arm * xv_gyro_.gyro_data2_flt;
  xv_gyro_.deg_arm_pitch = std::clamp(w, -30.0f, 30.0f);

  w = xp_gyro_.kp2_waist * xv_gyro_.gyro_data2_flt;
  xv_gyro_.deg_waist_pitch = std::clamp(w, -20.0f, 20.0f);

  w = xp_gyro_.kp3_waist * xv_gyro_.gyro_data3_flt;
  xv_gyro_.deg_waist_yaw = std::clamp(w, -20.0f, 20.0f);

  if (flag_gyro_.vib_auto == ON && flag_gyro_.vib == ON)
  {
    float len = std::sqrt(joy_->xv_joy_.walk_x_percent * joy_->xv_joy_.walk_x_percent +
                          joy_->xv_joy_.walk_y_percent * joy_->xv_joy_.walk_y_percent);
    float ratio = 1.0;
    if (len != 0.0)
    {
      ratio = std::fabs(joy_->xv_joy_.walk_x_percent / len);
    }
    kine_->xv_kine_[0].foot_r -= xv_gyro_.deg_foot_roll * ratio;
    kine_->xv_kine_[1].foot_r -= xv_gyro_.deg_foot_roll * ratio;
    serv_->xv_ref_.d_ref[FOOT_ROLL_R] -= xv_gyro_.deg_foot_roll * ratio;
    serv_->xv_ref_.d_ref[FOOT_ROLL_L] -= xv_gyro_.deg_foot_roll * ratio;

    kine_->xv_kine_[0].hip_r -= xv_gyro_.deg_hip_roll * ratio;
    kine_->xv_kine_[1].hip_r -= xv_gyro_.deg_hip_roll * ratio;
    serv_->xv_ref_.d_ref[LEG_ROLL_R] -= xv_gyro_.deg_hip_roll * ratio;
    serv_->xv_ref_.d_ref[LEG_ROLL_L] -= xv_gyro_.deg_hip_roll * ratio;

    // knee feedback
    kine_->xv_kine_[0].knee -= xv_gyro_.deg_hip_pitch;
    kine_->xv_kine_[1].knee -= xv_gyro_.deg_hip_pitch;
    serv_->xv_ref_.d_ref[KNEE_R2] -= xv_gyro_.deg_hip_pitch;
    serv_->xv_ref_.d_ref[KNEE_L2] -= xv_gyro_.deg_hip_pitch;

    w = kine_->xv_kine_[0].foot_p - xv_gyro_.deg_foot_pitch;
    w2 = std::clamp(w, -90.0f, kine_->xv_kine_[0].knee);
    kine_->xv_kine_[0].foot_p = w2;
    serv_->xv_ref_.d_ref[KNEE_R1] -= xv_gyro_.deg_foot_pitch;

    w = kine_->xv_kine_[1].foot_p - xv_gyro_.deg_foot_pitch;
    w2 = std::clamp(w, -90.f, kine_->xv_kine_[1].knee);
    kine_->xv_kine_[1].foot_p = w2;
    serv_->xv_ref_.d_ref[KNEE_L1] -= xv_gyro_.deg_foot_pitch;

    if (xv_gyro_.deg_arm_roll < 0.0f)
    {
      serv_->xv_ref_.d_ref[ARM_ROLL_R] -= xv_gyro_.deg_arm_roll;
    }
    else if (xv_gyro_.deg_arm_roll > 0.0f)
    {
      serv_->xv_ref_.d_ref[ARM_ROLL_L] -= xv_gyro_.deg_arm_roll;
    }

    serv_->xv_ref_.d_ref[ARM_PITCH_R] -= xv_gyro_.deg_arm_pitch;
    serv_->xv_ref_.d_ref[ARM_PITCH_L] -= xv_gyro_.deg_arm_pitch;

    kine_->xv_kine_[0].leg -= xv_gyro_.deg_waist_pitch;
    kine_->xv_kine_[1].leg -= xv_gyro_.deg_waist_pitch;
  }
  else if (flag_gyro_.vib_auto == ON && flag_gyro_.vib == ON2)
  {
    w = xv_gyro_.deg_foot_roll / 2.0f;
    kine_->xv_kine_[0].foot_r -= w;
    kine_->xv_kine_[1].foot_r -= w;
    serv_->xv_ref_.d_ref[FOOT_ROLL_R] -= w;
    serv_->xv_ref_.d_ref[FOOT_ROLL_L] -= w;

    w = xv_gyro_.deg_hip_roll / 2.0f;
    kine_->xv_kine_[0].hip_r -= w;
    kine_->xv_kine_[1].hip_r -= w;
    serv_->xv_ref_.d_ref[LEG_ROLL_R] -= w;
    serv_->xv_ref_.d_ref[LEG_ROLL_L] -= w;

    // knee feedback
    w = xv_gyro_.deg_hip_pitch / 2.0f;
    kine_->xv_kine_[0].knee -= w;
    kine_->xv_kine_[1].knee -= w;
    serv_->xv_ref_.d_ref[KNEE_R2] -= w;
    serv_->xv_ref_.d_ref[KNEE_L2] -= w;

    w3 = xv_gyro_.deg_foot_pitch / 2.0f;
    w = kine_->xv_kine_[0].foot_p - w3;
    w2 = std::clamp(w, -90.0f, kine_->xv_kine_[0].knee);
    kine_->xv_kine_[0].foot_p = w2;
    serv_->xv_ref_.d_ref[KNEE_R1] -= w3;

    w = kine_->xv_kine_[1].foot_p - w3;
    w2 = std::clamp(w, -90.0f, kine_->xv_kine_[1].knee);
    kine_->xv_kine_[1].foot_p = w2;
    serv_->xv_ref_.d_ref[KNEE_L1] -= w3;

    w = xv_gyro_.deg_arm_roll / 2.0f;
    if (w < 0.0f)
    {
      serv_->xv_ref_.d_ref[ARM_ROLL_R] -= w;
    }
    else if (w > 0.0f)
    {
      serv_->xv_ref_.d_ref[ARM_ROLL_L] -= w;
    }

    serv_->xv_ref_.d_ref[ARM_PITCH_R] -= xv_gyro_.deg_arm_pitch;
    serv_->xv_ref_.d_ref[ARM_PITCH_L] -= xv_gyro_.deg_arm_pitch;

    kine_->xv_kine_[0].leg -= xv_gyro_.deg_waist_pitch;
    kine_->xv_kine_[1].leg -= xv_gyro_.deg_waist_pitch;
  }
  else if (flag_gyro_.vib_auto == ON && flag_gyro_.vib == ON3)
  {
    w = xv_gyro_.deg_foot_roll * 1.3f;
    kine_->xv_kine_[0].foot_r -= w;
    kine_->xv_kine_[1].foot_r -= w;
    serv_->xv_ref_.d_ref[FOOT_ROLL_R] -= w;
    serv_->xv_ref_.d_ref[FOOT_ROLL_L] -= w;

    w = xv_gyro_.deg_hip_roll * 1.3f;
    kine_->xv_kine_[0].hip_r -= w;
    kine_->xv_kine_[1].hip_r -= w;
    serv_->xv_ref_.d_ref[LEG_ROLL_R] -= w;
    serv_->xv_ref_.d_ref[LEG_ROLL_L] -= w;

    // knee feedback
    w = xv_gyro_.deg_hip_pitch * 1.3f;
    kine_->xv_kine_[0].knee -= w;
    kine_->xv_kine_[1].knee -= w;
    serv_->xv_ref_.d_ref[KNEE_R2] -= w;
    serv_->xv_ref_.d_ref[KNEE_L2] -= w;

    w3 = xv_gyro_.deg_foot_pitch * 1.3f;

    w = kine_->xv_kine_[0].foot_p - w3;
    w2 = std::clamp(w, -90.0f, kine_->xv_kine_[0].knee);
    kine_->xv_kine_[0].foot_p = w2;
    serv_->xv_ref_.d_ref[KNEE_R1] -= w3;

    w = kine_->xv_kine_[1].foot_p - w3;
    w2 = std::clamp(w, -90.0f, kine_->xv_kine_[1].knee);
    kine_->xv_kine_[1].foot_p = w2;
    serv_->xv_ref_.d_ref[KNEE_L1] -= w3;

    w = xv_gyro_.deg_arm_roll * 2.0f;
    if (w < 0.0f)
    {
      serv_->xv_ref_.d_ref[ARM_ROLL_R] -= w;
    }
    else if (w > 0.0f)
    {
      serv_->xv_ref_.d_ref[ARM_ROLL_L] -= w;
    }

    serv_->xv_ref_.d_ref[ARM_PITCH_R] -= xv_gyro_.deg_arm_pitch;
    serv_->xv_ref_.d_ref[ARM_PITCH_L] -= xv_gyro_.deg_arm_pitch;

    kine_->xv_kine_[0].leg -= xv_gyro_.deg_waist_pitch;
    kine_->xv_kine_[1].leg -= xv_gyro_.deg_waist_pitch;
  }
}

void Gyro::gyro_yaw_cntr_fun()
{
  float _err, _dead;

  // gyro yaw control on
  if (flag_gyro_.yaw_cntl)
  {
    _err = xv_gyro_.yaw_cntl_ref - xv_gyro_.gyro_yaw;
    _dead = xp_gyro_.yaw_cntl_gain * hr46::deadband(_err, xp_gyro_.yaw_cntl_dead);
    xv_gyro_.yaw_cntl_fb = std::clamp(_dead, -xp_gyro_.yaw_cntl_theta, xp_gyro_.yaw_cntl_theta);

    if (xv_gyro_.yaw_cntl_fb > EPS_DATA)
    {
      sq_walk_->xv_mv_walk_.theta_percent_dlim = xv_gyro_.yaw_cntl_fb / sq_walk_->xp_mv_walk_.theta;
    }
    else if (xv_gyro_.yaw_cntl_fb < -EPS_DATA)
    {
      sq_walk_->xv_mv_walk_.theta_percent_dlim = xv_gyro_.yaw_cntl_fb / sq_walk_->xp_mv_walk_.theta;
    }
    else
    {
      sq_walk_->xv_mv_walk_.theta_percent_dlim = 0.0f;
    }
  }
}

void Gyro::PostureControl()
{
  float w, w1, w2;
  float hip_pitch_fd;
  float knee_pitch_fd;
  float foot_pitch_fd;
  float gain = 0.85;
  const float thre_control = 3.0;

  // Hip Pitch Control
  w = gain * xv_gyro_.gyro_pitch;
  hip_pitch_fd = std::clamp(w, -25.0f, 25.0f);
  kine_->xv_kine_[0].leg -= hip_pitch_fd;
  kine_->xv_kine_[1].leg -= hip_pitch_fd;
  serv_->xv_ref_.d_ref[LEG_PITCH_R] -= hip_pitch_fd;
  serv_->xv_ref_.d_ref[LEG_PITCH_L] -= hip_pitch_fd;
  if (xv_gyro_.gyro_pitch < -thre_control)
  {
    w1 = gain * xv_gyro_.gyro_pitch;
    knee_pitch_fd = std::clamp(w1, -18.0f, 18.0f);
    kine_->xv_kine_[0].knee -= knee_pitch_fd;
    kine_->xv_kine_[1].knee -= knee_pitch_fd;
    serv_->xv_ref_.d_ref[LEG_PITCH_L] -= knee_pitch_fd;
    serv_->xv_ref_.d_ref[LEG_PITCH_R] -= knee_pitch_fd;
  }
  else if (thre_control < xv_gyro_.gyro_pitch)
  {
    w2 = gain * xv_gyro_.gyro_pitch;
    foot_pitch_fd = std::clamp(w2, -18.0f, 18.0f);
    kine_->xv_kine_[0].foot_p -= foot_pitch_fd;
    kine_->xv_kine_[1].foot_r -= foot_pitch_fd;
    serv_->xv_ref_.d_ref[LEG_PITCH_L] -= foot_pitch_fd;
    serv_->xv_ref_.d_ref[LEG_PITCH_R] -= foot_pitch_fd;
  }
}

}  // namespace hr46
