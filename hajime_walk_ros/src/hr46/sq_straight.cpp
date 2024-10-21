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

#include "hr46/sq_straight.hpp"
#include "hr46/var.hpp"
#include "hr46/motion.hpp"

namespace hr46
{

SqStraight::SqStraight()
{
  // upright posture
  xp_mv_straight_.time = 1.0f;          // time to stand upright (constant)
  xp_mv_straight_.z3 = Z3_LIMIT_H;      // waist height (constant)
  xp_mv_straight_.arm_sh_pitch = 0.0f;  // shoulder pitch deg (constant)
  xp_mv_straight_.arm_sh_roll = 0.0f;   // shoulder roll deg (constant)
  xp_mv_straight_.arm_el_yaw = 0.0f;    // elbow yaw deg (constant)
  xp_mv_straight_.arm_el_pitch = 0.0f;  // elbow pitch deg (constant)
  xv_mv_straight_time_ = 1.0f;          // time to stand upright (constant)
}

void SqStraight::sq_straight_init()
{
  flag_md_straight_end_ = OFF;
  xv_mv_straight_time_ = xp_mv_straight_.time; /* slow */
  // if (!flag_servo_on && motion_->mode_motion__ != MOTION_START)
  if (motion_->mode_motion_ != MOTION_START)
  {
    xv_mv_straight_time_ = 0.2f; /* fast */
  }

  if (serv_->check_sw_ref_d() == JOINT_ANGLE)
  {
    kine_->trk_kine();
  }

  // status
  mode_sq_straight_ = SQ_STRAIGHT;
}

int SqStraight::sq_straight(short flag_face_control)
{
  gyro_->flag_gyro_.vib = ON;  // normal gain
  switch (mode_sq_straight_)
  {
    case SQ_STRAIGHT:
      calc_mv_->xv_data_x_r_.time = calc_mv_->xv_data_y_r_.time = calc_mv_->xv_data_y_r2_.time =
          calc_mv_->xv_data_z_r_.time = calc_mv_->xv_data_x_l_.time = calc_mv_->xv_data_y_l_.time =
              calc_mv_->xv_data_y_l2_.time = calc_mv_->xv_data_z_l_.time = xv_mv_straight_time_;

      calc_mv_->xv_data_x_r_.pos = 0.f;
      calc_mv_->xv_data_y_r_.pos = 0.f;
      calc_mv_->xv_data_y_r2_.pos = 0.f;
      calc_mv_->xv_data_z_r_.pos = xp_mv_straight_.z3;
      calc_mv_->xv_data_x_l_.pos = 0.f;
      calc_mv_->xv_data_y_l_.pos = 0.f;
      calc_mv_->xv_data_y_l2_.pos = 0.f;
      calc_mv_->xv_data_z_l_.pos = xp_mv_straight_.z3;

      // status
      mode_sq_straight_ = SQ_STRAIGHT2;
      break;

    case SQ_STRAIGHT2:
      calc_mv_->xv_data_pitch_.time = calc_mv_->xv_data_roll2_.time = xv_mv_straight_time_;

      calc_mv_->xv_data_pitch_.pos = 0.f;
      calc_mv_->xv_data_roll2_.pos = 0.f;

      // status
      mode_sq_straight_ = SQ_STRAIGHT3;
      break;

    case SQ_STRAIGHT3:
      calc_mv_->xv_data_d_[0].time = xv_mv_straight_time_;
      calc_mv_->xv_data_d_[1].time = xv_mv_straight_time_;
      calc_mv_->xv_data_d_[2].time = xv_mv_straight_time_;
      calc_mv_->xv_data_d_[3].time = xv_mv_straight_time_;
      calc_mv_->xv_data_d_[4].time = xv_mv_straight_time_;
      calc_mv_->xv_data_d_[5].time = xv_mv_straight_time_;
      calc_mv_->xv_data_d_[6].time = xv_mv_straight_time_;
      calc_mv_->xv_data_d_[7].time = xv_mv_straight_time_;

      calc_mv_->xv_data_d_[0].pos = 0.0f;
      calc_mv_->xv_data_d_[1].pos = 0.0f;
      calc_mv_->xv_data_d_[2].pos = 0.0f;
      calc_mv_->xv_data_d_[3].pos = 0.0f;
      calc_mv_->xv_data_d_[4].pos = 0.0f;
      calc_mv_->xv_data_d_[5].pos = 0.0f;
      calc_mv_->xv_data_d_[6].pos = 0.0f;
      calc_mv_->xv_data_d_[7].pos = 0.0f;

      // status
      mode_sq_straight_ = SQ_STRAIGHT4;
      break;

    case SQ_STRAIGHT4:
      calc_mv_->xv_data_d_[8].time = xv_mv_straight_time_;
      calc_mv_->xv_data_d_[9].time = xv_mv_straight_time_;
      calc_mv_->xv_data_d_[10].time = xv_mv_straight_time_;
      calc_mv_->xv_data_d_[11].time = xv_mv_straight_time_;
      calc_mv_->xv_data_d_[12].time = xv_mv_straight_time_;
      calc_mv_->xv_data_d_[13].time = xv_mv_straight_time_;

      calc_mv_->xv_data_d_[8].pos = 0.0f;
      calc_mv_->xv_data_d_[9].pos = 0.0f;
      calc_mv_->xv_data_d_[10].pos = 0.0f;
      calc_mv_->xv_data_d_[11].pos = 0.0f;
      calc_mv_->xv_data_d_[12].pos = 0.0f;
      calc_mv_->xv_data_d_[13].pos = 0.0f;

      calc_mv_->xv_data_d_[ARM_PITCH_L].time = calc_mv_->xv_data_d_[ARM_PITCH_R].time =
          calc_mv_->xv_data_d_[ARM_ROLL_L].time = calc_mv_->xv_data_d_[ARM_ROLL_R].time = xv_mv_straight_time_;
      calc_mv_->xv_data_d_[ARM_PITCH_L].pos = calc_mv_->xv_data_d_[ARM_PITCH_R].pos = xp_mv_straight_.arm_sh_pitch;
      calc_mv_->xv_data_d_[ARM_ROLL_L].pos = -xp_mv_straight_.arm_sh_roll;
      calc_mv_->xv_data_d_[ARM_ROLL_R].pos = xp_mv_straight_.arm_sh_roll;

      // status
      mode_sq_straight_ = SQ_STRAIGHT5;
      break;

    case SQ_STRAIGHT5:
      if (!flag_face_control)
      {
        calc_mv_->xv_data_d_[HEAD_YAW].time = calc_mv_->xv_data_d_[HEAD_PITCH].time = xv_mv_straight_time_;
        calc_mv_->xv_data_d_[HEAD_YAW].pos = 0.0f;
        calc_mv_->xv_data_d_[HEAD_PITCH].pos = 0.0f;
      }

      calc_mv_->xv_data_d_[ELBOW_PITCH_L].time = calc_mv_->xv_data_d_[SPARE17].time =
          calc_mv_->xv_data_d_[ELBOW_PITCH_R].time = calc_mv_->xv_data_d_[SPARE21].time = xv_mv_straight_time_;

      calc_mv_->xv_data_d_[ELBOW_PITCH_L].pos = xp_mv_straight_.arm_el_pitch;
      calc_mv_->xv_data_d_[SPARE17].pos = xp_mv_straight_.arm_el_yaw;
      calc_mv_->xv_data_d_[ELBOW_PITCH_R].pos = xp_mv_straight_.arm_el_pitch;
      calc_mv_->xv_data_d_[SPARE21].pos = xp_mv_straight_.arm_el_yaw;

      // status
      mode_sq_straight_ = SQ_STRAIGHT_END;
      mode_sq_time_ = 0.0f;
      break;

    case SQ_STRAIGHT_END:
      if (mode_sq_time_ >= (xv_mv_straight_time_ - EPS_TIME))
      {
        gyro_->flag_gyro_.vib = ON2;  // low gain

        serv_->set_sw_ref_d(FOOT_XYZ);  // foot xyz position control

        // status
        motion_->sq_flag_.straight = OFF;
        flag_md_straight_end_ = ON;

        mode_sq_time_ = 0.0f;
      }
      break;

    default:
      break;
  }

  mode_sq_time_ += RTC_TIME_SEC;
  return flag_md_straight_end_;
}

}  // namespace hr46
