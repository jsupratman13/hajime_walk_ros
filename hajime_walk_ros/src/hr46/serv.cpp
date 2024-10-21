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

#include "hr46/func.hpp"
#include "hr46/serv.hpp"

namespace hr46
{

Serv::Serv()
{
  sw_.ref_d = FOOT_XYZ;  // foot xyz position control (inv. kinematics)

  for (int i = 0; i < SERV_NUM; i++)
  {
    xv_pv_.temp[i] = 0;
    xv_pv_.deg[i] = 0.0f;
    xv_pv_.current[i] = 0;
  }
  xp_ref_d_lim_ = 360.0f / RTC_TICK;  // dlimit for leg joint = 360 [deg/s]

  for (int i = 0; i < SERV_NUM; i++)
  {
    xp_sv_[i].deg_lim_h = 180;
    xp_sv_[i].deg_lim_l = -180;

    // RS405CB
    // -150deg=0xFA24(-1500),0deg=0x0000(0),150deg=0x05DC(1500) => 10.00 bit/deg
    xp_sv_[i].deg2pls = 1000;
    xp_sv_[i].deg_lim_offset = 0;

    xp_sv_[i].deg_sign = 1;
    xp_sv_[i].deg_offset = 0;

    xv_sv_[i].d = 0;
    xv_sv_[i].deg_sw_out = 0;
    xv_sv_[i].deg_sv = 0;
    xv_sv_[i].deg_lim = 0;
    xv_sv_[i].pls = 0;
    xv_sv_[i].pls_out = 0;  // analogue servo

    xv_ref_.d[i] = 0.0f;
    xv_ref_.d_ref[i] = 0.0f;
  }

  xp_sv_[0].deg_sign = -1;  // left  foot (roll)
  xp_sv_[0].deg_offset = 0;
  xp_sv_[1].deg_sign = 1;  // left  foot (pitch)
  xp_sv_[1].deg_offset = 0;
  xp_sv_[2].deg_sign = 1;  // left  knee
  xp_sv_[2].deg_offset = 0;
  xp_sv_[3].deg_sign = 1;  // left  leg
  xp_sv_[3].deg_offset = 0;
  xp_sv_[4].deg_sign = 1;  // left  hip
  xp_sv_[4].deg_offset = 0;
  xp_sv_[5].deg_sign = -1;  // left  waist
  xp_sv_[5].deg_offset = 0;
  xp_sv_[6].deg_sign = 1;  // right foot (roll)
  xp_sv_[6].deg_offset = 0;
  xp_sv_[7].deg_sign = -1;  // right foot (pitch)
  xp_sv_[7].deg_offset = 0;
  xp_sv_[8].deg_sign = -1;  // right knee
  xp_sv_[8].deg_offset = 0;

  xp_sv_[9].deg_sign = -1;  // right leg
  xp_sv_[9].deg_offset = 0;
  xp_sv_[10].deg_sign = -1;  // right hip
  xp_sv_[10].deg_offset = 0;
  xp_sv_[11].deg_sign = -1;  // right waist
  xp_sv_[11].deg_offset = 0;
  xp_sv_[12].deg_sign = -1;  // body (pitch)
  xp_sv_[12].deg_offset = 0;
  xp_sv_[13].deg_sign = 1;  // body (yaw)
  xp_sv_[13].deg_offset = 0;
  xp_sv_[14].deg_sign = -1;  // left  shoulder (pitch)
  xp_sv_[14].deg_offset = 0;
  xp_sv_[15].deg_sign = -1;  // left  shoulder (roll)
  xp_sv_[15].deg_offset = 0;
  xp_sv_[16].deg_sign = -1;  // left  elbow (yaw)
  xp_sv_[16].deg_offset = 0;
  xp_sv_[17].deg_sign = 1;  // left  elbow (pitch)
  xp_sv_[17].deg_offset = 0;

  xp_sv_[18].deg_sign = 1;  // right shoulder (pitch)
  xp_sv_[18].deg_offset = 0;
  xp_sv_[19].deg_sign = 1;  // right shoulder (roll)
  xp_sv_[19].deg_offset = 0;
  xp_sv_[20].deg_sign = 1;  // right elbow (yaw)
  xp_sv_[20].deg_offset = 0;
  xp_sv_[21].deg_sign = 1;  // right elbow (pitch)
  xp_sv_[21].deg_offset = 0;

  xp_sv_[22].deg_sign = 1;  // head (yaw)
  xp_sv_[22].deg_offset = 0;
  xp_sv_[23].deg_sign = 1;  // head (pitch)(tilt)
  xp_sv_[23].deg_offset = 0;
  xp_sv_[24].deg_sign = 1;  // spare
  xp_sv_[24].deg_offset = 0;
  xp_sv_[25].deg_sign = 1;  // spare
  xp_sv_[25].deg_offset = 0;
  xp_sv_[26].deg_sign = 1;  // spare
  xp_sv_[26].deg_offset = 0;
  xp_sv_[27].deg_sign = 1;  // spare
  xp_sv_[27].deg_offset = 0;
  xp_sv_[28].deg_sign = 1;  // spare
  xp_sv_[28].deg_offset = 0;
}

void Serv::serv()
{
  short i;
  long offset, hlim, llim;
  float w;

  xv_ref_.d[LEG_ROLL_R] =
      hr46::sw2(sw_.ref_d, kine_->xv_kine_[0].hip_r, xv_ref_.d_ref[LEG_ROLL_R]);  // right  hip (roll)
  xv_ref_.d[LEG_PITCH_R] = hr46::sw2(sw_.ref_d, kine_->xv_kine_[0].leg, xv_ref_.d_ref[LEG_PITCH_R]);  // right leg
  xv_ref_.d[KNEE_R2] = hr46::sw2(sw_.ref_d, kine_->xv_kine_[0].knee, xv_ref_.d_ref[KNEE_R2]);         // right knee
  xv_ref_.d[KNEE_R1] = hr46::sw2(sw_.ref_d, kine_->xv_kine_[0].foot_p, xv_ref_.d_ref[KNEE_R1]);  // right foot (pitch)
  xv_ref_.d[FOOT_ROLL_R] =
      hr46::sw2(sw_.ref_d, kine_->xv_kine_[0].foot_r, xv_ref_.d_ref[FOOT_ROLL_R]);  // right foot (roll)

  xv_ref_.d[LEG_ROLL_L] =
      hr46::sw2(sw_.ref_d, kine_->xv_kine_[1].hip_r, xv_ref_.d_ref[LEG_ROLL_L]);  // left  hip (roll)
  xv_ref_.d[LEG_PITCH_L] = hr46::sw2(sw_.ref_d, kine_->xv_kine_[1].leg, xv_ref_.d_ref[LEG_PITCH_L]);  // left leg
  xv_ref_.d[KNEE_L2] = hr46::sw2(sw_.ref_d, kine_->xv_kine_[1].knee, xv_ref_.d_ref[KNEE_L2]);         // left knee
  xv_ref_.d[KNEE_L1] = hr46::sw2(sw_.ref_d, kine_->xv_kine_[1].foot_p, xv_ref_.d_ref[KNEE_L1]);  // left foot (pitch)
  xv_ref_.d[FOOT_ROLL_L] =
      hr46::sw2(sw_.ref_d, kine_->xv_kine_[1].foot_r, xv_ref_.d_ref[FOOT_ROLL_L]);  // left foot (roll)

  xv_ref_.d[LEG_YAW_R] = (xv_ref_.d[LEG_YAW_R] + kine_->xv_posture_.yaw);  // right hip (yaw)
  xv_ref_.d[LEG_YAW_L] = (xv_ref_.d[LEG_YAW_L] + kine_->xv_posture_.yaw);  // left hip (yaw)

  xv_ref_.d[SPARE12] = xv_ref_.d_ref[SPARE12];  // body (pitch)
  xv_ref_.d[SPARE13] = xv_ref_.d_ref[SPARE13];  // body (yaw)

  xv_ref_.d[ARM_PITCH_L] = xv_ref_.d_ref[ARM_PITCH_L];      // left shoulder (pitch)
  xv_ref_.d[ARM_ROLL_L] = xv_ref_.d_ref[ARM_ROLL_L];        // left shoulder (roll)
  xv_ref_.d[ELBOW_PITCH_L] = xv_ref_.d_ref[ELBOW_PITCH_L];  // left elbow (yaw)
  xv_ref_.d[SPARE17] = xv_ref_.d_ref[SPARE17];              // left elbow (pitch)

  xv_ref_.d[ARM_PITCH_R] = xv_ref_.d_ref[ARM_PITCH_R];      // right shoulder (pitch)
  xv_ref_.d[ARM_ROLL_R] = xv_ref_.d_ref[ARM_ROLL_R];        // right shoulder (roll)
  xv_ref_.d[ELBOW_PITCH_R] = xv_ref_.d_ref[ELBOW_PITCH_R];  // right elbow (yaw)
  xv_ref_.d[SPARE21] = xv_ref_.d_ref[SPARE21];              // right   elbow (pitch)

  xv_ref_.d[HEAD_YAW] = xv_ref_.d_ref[HEAD_YAW];      // head (yaw)(pan)
  xv_ref_.d[HEAD_PITCH] = xv_ref_.d_ref[HEAD_PITCH];  // head (pitch)(tilt)
  xv_ref_.d[SPARE24] = xv_ref_.d_ref[SPARE24];        // spare
  xv_ref_.d[SPARE25] = xv_ref_.d_ref[SPARE25];        // spare
  xv_ref_.d[SPARE26] = xv_ref_.d_ref[SPARE26];        // spare
  xv_ref_.d[SPARE27] = xv_ref_.d_ref[SPARE27];        // spare
  xv_ref_.d[SPARE28] = xv_ref_.d_ref[SPARE28];        // spare
}

void Serv::set_sw_ref_d(int n)
{
  int i;
  /* 1: foot xyz position control	*/
  /* 2: joint angle control	*/

  if (n == JOINT_ANGLE)  // joint angle control
  {
    for (i = 0; i < SERV_NUM; i++)
    {
      // tracking
      xv_ref_.d_ref[i] = xv_ref_.d[i];
      calc_mv_->xv_mvdata_d_[i].amp = 0.0f;
      calc_mv_->xv_mvdata_d_[i].start = calc_mv_->xv_mvdata_d_[i].out_old = calc_mv_->xv_mvdata_d_[i].pos_old =
          xv_ref_.d[i];

      calc_mv_->xv_data_d_[i].time = 0.1f;
      calc_mv_->xv_data_d_[i].pos = xv_ref_.d[i];
    }
    sw_.ref_d = JOINT_ANGLE;
  }
  else  // foot xyz position control
  {
    for (i = 0; i < SERV_NUM; i++)
    {
      if (i != ARM_PITCH_R && i != ARM_ROLL_R && i != ELBOW_PITCH_R && i != ARM_PITCH_L && i != ARM_ROLL_L &&
          i != ELBOW_PITCH_L && i != HEAD_YAW && i != HEAD_PITCH)
        calc_mv_->xv_data_d_[i].pos = 0.0f;
    }
    sw_.ref_d = FOOT_XYZ;
  }
}

short Serv::check_sw_ref_d()
{
  return sw_.ref_d;
}

}  // namespace hr46
