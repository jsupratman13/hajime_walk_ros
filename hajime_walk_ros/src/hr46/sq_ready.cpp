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

#include "hr46/sq_ready.hpp"
#include "hr46/var.hpp"

namespace hr46
{
SqReady::SqReady()
{
  // ready posture
  xp_mv_ready_.time = 1.5f;               // time to ready posture
  xp_mv_ready_.z3 = Z3_LIMIT_H;           // waist height: lower the waist by 5%
  xp_mv_ready_.arm_sh_pitch = 0.0f;       // shoulder pitch degree
  xp_mv_ready_.arm_sh_roll = 0.0f;        // shoulder roll degree
  xp_mv_ready_.arm_el_yaw = 0.0f;         // elbow yaw degree
  xp_mv_ready_.arm_el_pitch = 0.0f;       // elbow pitch degree
  xp_mv_ready_.pitch = 0.0f;              // waist pitch degree
  flag_ready_gyro_ = OFF;                 // turn off gyro in ready posture
  xv_mv_ready_time_ = xp_mv_ready_.time;  // time to ready posture
}
void SqReady::sq_ready_init(int slow_mode)
{
  flag_md_ready_end_ = OFF;
  sq_walk_->flag_walk_.upleg = OFF;

  if (slow_mode)
  {
    xv_mv_ready_time_ = xp_mv_ready_.time; /* slow */
    flag_ready_gyro_ = ON;
  }
  else
  {
    xv_mv_ready_time_ = 0.04f; /* fast */
    flag_ready_gyro_ = ON3;
  }

  if (serv_->check_sw_ref_d() == JOINT_ANGLE)
  {
    kine_->trk_kine();
    serv_->set_sw_ref_d(FOOT_XYZ);  // foot xyz position control
  }

  /* status */
  mode_sq_ready_ = SQ_READY;
}

int SqReady::sq_ready(short flag_face_control)
{
  gyro_->flag_gyro_.vib = ON;  // normal gain
  switch (mode_sq_ready_)
  {
    case SQ_READY:  // ready position
      /* action */
      calc_mv_->xv_data_x_r_.time = calc_mv_->xv_data_y_r_.time = calc_mv_->xv_data_y_r2_.time =
          calc_mv_->xv_data_z_r_.time = calc_mv_->xv_data_x_l_.time = calc_mv_->xv_data_y_l_.time =
              calc_mv_->xv_data_y_l2_.time = calc_mv_->xv_data_z_l_.time = xv_mv_ready_time_;

      calc_mv_->xv_data_x_r_.pos = 0.0f;
      calc_mv_->xv_data_y_r_.pos = sq_walk_->xp_mv_walk_.y_wide;
      calc_mv_->xv_data_y_r2_.pos = 0.0f;
      calc_mv_->xv_data_z_r_.pos = xp_mv_ready_.z3;
      calc_mv_->xv_data_x_l_.pos = 0.0f;
      calc_mv_->xv_data_y_l_.pos = -sq_walk_->xp_mv_walk_.y_wide;
      calc_mv_->xv_data_y_l2_.pos = 0.0f;
      calc_mv_->xv_data_z_l_.pos = xp_mv_ready_.z3;

      /* status */
      mode_sq_ready_ = SQ_READY2;

      break;

    case SQ_READY2:  // ready position
      /* action */
      calc_mv_->xv_data_pitch_.time = calc_mv_->xv_data_roll2_.time = xv_mv_ready_time_;

      calc_mv_->xv_data_roll2_.pos = 0.0f;
      calc_mv_->xv_data_pitch_.pos = xp_mv_ready_.pitch;

      /* status */
      mode_sq_ready_ = SQ_READY3;

      break;

    case SQ_READY3:  // ready position
      /* action */
      calc_mv_->xv_data_d_[LEG_YAW_L].time = calc_mv_->xv_data_d_[LEG_YAW_R].time = calc_mv_->xv_data_d_[12].time =
          calc_mv_->xv_data_d_[13].time = xv_mv_ready_time_;

      calc_mv_->xv_data_d_[LEG_YAW_L].pos = calc_mv_->xv_data_d_[LEG_YAW_R].pos = calc_mv_->xv_data_d_[12].pos =
          calc_mv_->xv_data_d_[13].pos = 0.0f;

      calc_mv_->xv_data_d_[ELBOW_PITCH_L].time = calc_mv_->xv_data_d_[ELBOW_PITCH_R].time = xv_mv_ready_time_;
      calc_mv_->xv_data_d_[ARM_PITCH_L].time = calc_mv_->xv_data_d_[ARM_PITCH_R].time = xv_mv_ready_time_;
      calc_mv_->xv_data_d_[ELBOW_PITCH_L].pos = xp_mv_ready_.arm_el_pitch;
      calc_mv_->xv_data_d_[ELBOW_PITCH_R].pos = xp_mv_ready_.arm_el_pitch;
      calc_mv_->xv_data_d_[ARM_PITCH_L].pos = xp_mv_ready_.arm_sh_pitch;
      calc_mv_->xv_data_d_[ARM_PITCH_R].pos = xp_mv_ready_.arm_sh_pitch;

      /* status */
      mode_sq_ready_ = SQ_READY4;
      break;

    case SQ_READY4:  // ready position
      /* action */
      if (!flag_face_control)
      {
        calc_mv_->xv_data_d_[HEAD_YAW].time = calc_mv_->xv_data_d_[HEAD_PITCH].time = xv_mv_ready_time_;
        calc_mv_->xv_data_d_[HEAD_YAW].pos = 0.0f;
        calc_mv_->xv_data_d_[HEAD_PITCH].pos = 0.0f;
      }
      calc_mv_->xv_data_d_[ARM_ROLL_L].time = calc_mv_->xv_data_d_[ARM_ROLL_R].time = xv_mv_ready_time_;

      calc_mv_->xv_data_d_[ARM_ROLL_L].pos = xp_mv_ready_.arm_sh_roll;
      calc_mv_->xv_data_d_[ARM_ROLL_R].pos = xp_mv_ready_.arm_sh_roll;

      /* status */
      mode_sq_ready_ = SQ_READY_END;
      break;

    case SQ_READY_END:  // end
      if (mode_sq_time_ >= (xv_mv_ready_time_ - EPS_TIME))
      {
        gyro_->flag_gyro_.vib = ON2;  // low gain
        motion_->sq_flag_.ready = OFF;
        flag_md_ready_end_ = ON;
        mode_sq_ready_ = SQ_READY_INIT;
        mode_sq_time_ = 0.0f;

        flag_ready_gyro_ = OFF;

        serv_->set_sw_ref_d(FOOT_XYZ);  // foot xyz position control
      }
      break;

    default:
      break;
  }

  mode_sq_time_ += RTC_TIME_SEC;
  return flag_md_ready_end_;
}

}  // namespace hr46
