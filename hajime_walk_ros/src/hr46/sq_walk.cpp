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
#include <algorithm>

#include "hr46/sq_walk.hpp"
#include "hr46/func.hpp"
#include "hr46/var.hpp"

#include <iostream>
namespace hr46
{

SqWalk::SqWalk()
{
  mode_sq_walk_ = SQ_WALK_INIT;
  flag_walk_.upleg = OFF;
  flag_walk_.upleg_last = OFF;
  flag_walk_.y = STRAIGHT;
  flag_walk_.turn = STRAIGHT;
  flag_walk_.y_on = STRAIGHT;
  flag_walk_.turn_on = STRAIGHT;

  // high speed walk
  xp_mv_walk_.num = 8;
  xp_mv_walk_.h_cog = 300.0f;
  // height of center of gravity xp_mv_ready.z3 + 0mm from CAD
  xp_mv_walk_.time = 0.23f;  // walk period (cycle)
  xp_mv_walk_.x_fwd_swg = 55.0f;
  xp_mv_walk_.x_fwd_spt = -55.0f;
  xp_mv_walk_.x_bwd_swg = -55.0f;
  xp_mv_walk_.x_bwd_spt = 55.0f;
  xp_mv_walk_.y_swg = 30.0f;
  xp_mv_walk_.y_spt = -40.0f;
  xp_mv_walk_.theta = 18.0f;
  xp_mv_walk_.z = 25.0f;
  xp_mv_walk_.y_balance = 110.0f;
  xp_mv_walk_.hip_roll = 2.0f;  // amount of hip roll when walking
  xp_mv_walk_.x_fwd_pitch = 2.0f;
  xp_mv_walk_.x_bwd_pitch = -2.0f;  // angle to lean forward when advancing
  xp_mv_walk_.x_fwd_acc_pitch = 2.0f;
  xp_mv_walk_.x_bwd_acc_pitch = -2.0f;  // angle to lean forward when accelerating
  xp_mv_walk_.arm_sh_pitch = 15.0f;
  xp_mv_walk_.start_zmp_k1 = 1.2f;
  xp_mv_walk_.start_time_k1 = 0.8f;
  //	xp_mv_walk_.start_time_k2		=	0.5f	;
  xp_mv_walk_.foot_cntl_p = 0.0;
  xp_mv_walk_.foot_cntl_r = 1.0;
  xp_mv_walk_.sidestep_time_k = 0.05f;
  xp_mv_walk_.sidestep_roll = 1.0f;
  xp_mv_walk_.y_wide = 1.0f;
  xp_mv_walk_.time_dutyfactor = 1.0f;
  xp_mv_walk_.accurate_x_percent_dlim = 1.0f;
  xp_mv_walk_.accurate_y_percent_dlim = 1.0f;
  xp_mv_walk_.accurate_th_percent_dlim = 0.6667f;
  xp_mv_walk_.accurate_step_z = 55.f;
  xp_mv_walk_.accurate_step_time = 0.32f;

  //	4 is even
  //	walk begins right foot first and fast walk begins left foot first,
  //	because of streight walk_.
  xv_mv_walk_.num = xp_mv_walk_.num;
  xv_mv_walk_.time = xp_mv_walk_.time;
  xv_mv_walk_.time_old = 0.f;
  xv_mv_walk_.x_swg = 0.f;
  xv_mv_walk_.x_spt = 0.f;
  xv_mv_walk_.y_swg = 0.f;
  xv_mv_walk_.y_spt = 0.f;
  xv_mv_walk_.theta = 0.f;
  xv_mv_walk_.z = xp_mv_walk_.z;
  xv_mv_walk_.pitch = 0.f;
  xv_mv_walk_.arm_sh_pitch = 0.f;
  xv_mv_walk_.zmp = 0.f;
  xv_mv_walk_.x_percent = 0.f;
  xv_mv_walk_.y_percent = 0.f;
  xv_mv_walk_.theta_percent = 0.f;
  xv_mv_walk_.sidestep_time_k_r = 0.f;
  xv_mv_walk_.sidestep_time_k_l = 0.f;
  xv_mv_walk_.sidestep_roll = 0.f;
  xv_mv_walk_.sidestep_roll_z = 0.f;
  xv_mv_walk_.x_percent_dlim = 0.f;
  xv_mv_walk_.y_percent_dlim = 0.f;
  xv_mv_walk_.theta_percent_dlim = 0.f;
  xv_mv_walk_.pitch_percent_dlim = 0.f;
  xv_mv_walk_.time_dutyfactor = xp_mv_walk_.time_dutyfactor;
  xv_mv_walk_.accurate_step_x = 0.f;
  xv_mv_walk_.accurate_step_y = 0.f;
  xv_mv_walk_.accurate_step_z = xp_mv_walk_.accurate_step_z;
  xv_mv_walk_.accurate_step_th = 0.f;
  xv_mv_walk_.accurate_step_time = xp_mv_walk_.accurate_step_time;

  xp_dlim_wait_x_.dlim = 1.0;
  xp_dlim_wait_y_.dlim = 1.0;
  xp_dlim_wait_theta_.dlim = 1.0f;                                                               /*	[1/sec]		*/
  xp_dlim_wait_pitch_.dlim = 1.0f;                                                               /*	[1/sec]		*/
  xp_dlim_wait_x_.wait_time = xp_dlim_wait_y_.wait_time = xp_dlim_wait_theta_.wait_time = 0.05f; /*	[sec]		*/
  xp_dlim_wait_pitch_.wait_time = 0.0f;
  /*	[sec]		*/  // do not create a wait time for pitch switching

  hr46::dlim_wait_fun_init(&xp_dlim_wait_x_, &xv_dlim_wait_x_);
  hr46::dlim_wait_fun_init(&xp_dlim_wait_y_, &xv_dlim_wait_y_);
  hr46::dlim_wait_fun_init(&xp_dlim_wait_theta_, &xv_dlim_wait_theta_);
  hr46::dlim_wait_fun_init(&xp_dlim_wait_pitch_, &xv_dlim_wait_pitch_);

  mv_tbl_zmp_sel_ = MV_TBL_ZMP2;  // use MV_TBL_ZMP2
}

int SqWalk::sq_walk()
{
  float abs_y_percent;  // y_percent_dlim absolute value of horizontal step percentage

  // set default walk parameters
  if (mode_sq_walk_ == SQ_WALK_INIT)  // initialize mode
  {
    xv_mv_walk_.z = xp_mv_walk_.z;  // height of foot z

    // dlimit initialize
    hr46::dlim_wait_fun_init(&xp_dlim_wait_x_, &xv_dlim_wait_x_);  // set percentage to 0
    hr46::dlim_wait_fun_init(&xp_dlim_wait_y_, &xv_dlim_wait_y_);
    hr46::dlim_wait_fun_init(&xp_dlim_wait_theta_, &xv_dlim_wait_theta_);
    hr46::dlim_wait_fun_init(&xp_dlim_wait_pitch_, &xv_dlim_wait_pitch_);
    xv_dlim_wait_pitch_.out = sq_ready_->xp_mv_ready_.pitch;

    /*	gyro sensor integrator clear	*/
    //		xv_gyro.gyro_roll	=
    //		xv_gyro.gyro_pitch	=
    //		xv_gyro.gyro_yaw	=	0.f;
  }

  // set parameters from command receive
  if (joy_->is_walk_change_)
  {
    joy_->copy_joy_parameter();  // command receive
    joy_->is_walk_change_ = 0;
  }

  // calculation dlimit of x, y, theta percent
  xv_dlim_wait_x_.in = xv_mv_walk_.x_percent;
  xv_dlim_wait_y_.in = xv_mv_walk_.y_percent;
  xv_dlim_wait_theta_.in = xv_mv_walk_.theta_percent;

  if (accurate_one_step_mode_ == 1)
  {
    if (xv_mv_walk_.accurate_step_x > -EPS_DATA && xv_mv_walk_.accurate_step_x < EPS_DATA)
      xv_mv_walk_.x_percent_dlim = 0.f;
    else
      xv_mv_walk_.x_percent_dlim = xv_mv_walk_.accurate_step_x > 0 ? 1.f : -1.f;

    if (xv_mv_walk_.accurate_step_y > -EPS_DATA && xv_mv_walk_.accurate_step_y < EPS_DATA)
      xv_mv_walk_.y_percent_dlim = 0.f;
    else
      xv_mv_walk_.y_percent_dlim = xv_mv_walk_.accurate_step_y > 0 ? 1.f : -1.f;

    if (xv_mv_walk_.accurate_step_th > -EPS_DATA && xv_mv_walk_.accurate_step_th < EPS_DATA)
      xv_mv_walk_.theta_percent_dlim = 0.f;
    else
      xv_mv_walk_.theta_percent_dlim = xv_mv_walk_.accurate_step_th > 0 ? 1.f : -1.f;
  }
  else
  {
    xv_mv_walk_.x_percent_dlim =
        hr46::dlim_wait_fun(&xp_dlim_wait_x_, &xv_dlim_wait_x_);  // Step length ratio (0-1) Mechanism for gradually
                                                                  // increasing stride length
    xv_mv_walk_.y_percent_dlim = hr46::dlim_wait_fun(&xp_dlim_wait_y_, &xv_dlim_wait_y_);
    xv_mv_walk_.theta_percent_dlim = xv_mv_walk_.theta_percent;
  }
  /*
  if( !flag_gyro.yaw_cntl ){
  xv_mv_walk.theta_percent_dlim	=	dlim_wait_fun( &xp_dlim_wait_theta, &xv_dlim_wait_theta );
}
  */

  // calculation walk forward and backward parameters
  if (xv_mv_walk_.x_percent_dlim > EPS_DATA)
  {  // walk forward
    if (accurate_one_step_mode_ == 1)
    {
      xv_mv_walk_.x_swg = (xv_mv_walk_.accurate_step_x / 2.f) * xp_mv_walk_.accurate_x_percent_dlim;
      xv_mv_walk_.x_spt = (-xv_mv_walk_.accurate_step_x / 2.f) * xp_mv_walk_.accurate_x_percent_dlim;
    }
    else
    {
      xv_mv_walk_.x_swg = xp_mv_walk_.x_fwd_swg * xv_mv_walk_.x_percent_dlim;  // swing stride length
      xv_mv_walk_.x_spt = xp_mv_walk_.x_fwd_spt * xv_mv_walk_.x_percent_dlim;  // support leg stride length
    }

    xv_mv_walk_.pitch = xp_mv_walk_.x_fwd_pitch * xv_mv_walk_.x_percent_dlim;  // angle of bending forward
    xv_mv_walk_.arm_sh_pitch = xp_mv_walk_.arm_sh_pitch;                       // shoulder angle
    xv_mv_walk_.arm_el_pitch = xp_mv_walk_.arm_el_pitch;                       // elbow angle
  }
  else if (xv_mv_walk_.x_percent_dlim < -EPS_DATA)
  {  // walk backward
    float w1, w2, w3;

    w1 = xp_mv_walk_.x_bwd_swg - xp_mv_walk_.x_bwd_spt;  // difference between front and rear strides of swing leg
    w2 = xp_mv_walk_.x_fwd_swg -
         xp_mv_walk_.x_fwd_spt;  // difference between the front and rear strides of the supporting leg
    w3 = std::fabs(w1 / w2);
    if (accurate_one_step_mode_ == 1)
    {
      xv_mv_walk_.x_swg = (xv_mv_walk_.accurate_step_x / 2.f) * xp_mv_walk_.accurate_x_percent_dlim;
      xv_mv_walk_.x_spt = (-xv_mv_walk_.accurate_step_x / 2.f) * xp_mv_walk_.accurate_x_percent_dlim;
    }
    else
    {
      xv_mv_walk_.x_swg = -xp_mv_walk_.x_bwd_swg * xv_mv_walk_.x_percent_dlim * w3;
      xv_mv_walk_.x_spt = -xp_mv_walk_.x_bwd_spt * xv_mv_walk_.x_percent_dlim * w3;
    }

    xv_mv_walk_.pitch = -xp_mv_walk_.x_bwd_pitch * xv_mv_walk_.x_percent_dlim;  // angle of bending backward
    xv_mv_walk_.arm_sh_pitch = -xp_mv_walk_.arm_sh_pitch;                       // shoulder angle
    xv_mv_walk_.arm_el_pitch = -xp_mv_walk_.arm_el_pitch;                       // elbow angle
  }
  else
  {
    xv_mv_walk_.x_swg = 0.f;
    xv_mv_walk_.x_spt = 0.f;

    xv_mv_walk_.pitch = 0.f;
    xv_mv_walk_.arm_sh_pitch = 0.f;
    xv_mv_walk_.arm_el_pitch = 0.f;
  }

  // calculate pitch
  if (xv_dlim_wait_x_.dout > 0)
  {
    xv_mv_walk_.pitch +=
        xp_mv_walk_.x_fwd_acc_pitch * xv_dlim_wait_x_.dout;  // correction of forward bending angle during acceleration
  }
  else if (xv_dlim_wait_x_.dout < 0)
  {
    xv_mv_walk_.pitch -=
        xp_mv_walk_.x_bwd_acc_pitch * xv_dlim_wait_x_.dout;  // correction of trailing angle during acceleration
  }

  xv_dlim_wait_pitch_.in = sq_ready_->xp_mv_ready_.pitch + xv_mv_walk_.pitch;
  calc_mv_->xv_data_pitch_.pos = hr46::dlim_wait_fun(&xp_dlim_wait_pitch_, &xv_dlim_wait_pitch_);
  calc_mv_->xv_data_pitch_.time = 0.1;

  // calculation walk sidestep parameters
  if (accurate_one_step_mode_ == 1)
  {
    xv_mv_walk_.y_swg = (xv_mv_walk_.accurate_step_y / 2.f) * xp_mv_walk_.accurate_y_percent_dlim;
    xv_mv_walk_.y_spt = (-xv_mv_walk_.accurate_step_y / 2.f) * xp_mv_walk_.accurate_y_percent_dlim;
  }
  else
  {
    xv_mv_walk_.y_swg = xp_mv_walk_.y_swg * xv_mv_walk_.y_percent_dlim;  // swing leg movement
    xv_mv_walk_.y_spt = xp_mv_walk_.y_spt * xv_mv_walk_.y_percent_dlim;  // support leg movement
  }
  abs_y_percent = std::fabs(xv_mv_walk_.y_percent_dlim);  // ratio to move sideways

  // calculation walk turn parameters
  if (accurate_one_step_mode_ == 1)
  {
    xv_mv_walk_.theta = xv_mv_walk_.accurate_step_th * xp_mv_walk_.accurate_th_percent_dlim;
  }
  else
  {
    xv_mv_walk_.theta = xp_mv_walk_.theta * xv_mv_walk_.theta_percent_dlim;
  }

  if (xv_mv_walk_.y_percent_dlim > EPS_DATA)
  {  // walk right
    flag_walk_.y = RIGHT;
    flag_walk_.turn = RIGHT;
    if (accurate_one_step_mode_ == 1)
    {
      xv_mv_walk_.sidestep_roll = 0.f;
      xv_mv_walk_.sidestep_time_k_r = 1.f;
      xv_mv_walk_.sidestep_time_k_l = 1.f;
    }
    else
    {
      xv_mv_walk_.sidestep_roll = xp_mv_walk_.sidestep_roll * abs_y_percent;  // angle to roll when walking sideways
      xv_mv_walk_.sidestep_time_k_r =
          1.0f + xp_mv_walk_.sidestep_time_k * abs_y_percent;  // ratio of right leg movement
      xv_mv_walk_.sidestep_time_k_l = 1.0f;                    // ratio of left leg movement
    }
  }
  else if (xv_mv_walk_.y_percent_dlim < -EPS_DATA)
  {  // walk left
    flag_walk_.y = LEFT;
    flag_walk_.turn = LEFT;
    if (accurate_one_step_mode_ == 1)
    {
      xv_mv_walk_.sidestep_roll = 0.f;
      xv_mv_walk_.sidestep_time_k_r = 1.f;
      xv_mv_walk_.sidestep_time_k_l = 1.f;
    }
    else
    {
      xv_mv_walk_.sidestep_roll = -xp_mv_walk_.sidestep_roll * abs_y_percent;
      xv_mv_walk_.sidestep_time_k_r = 1.0f;
      xv_mv_walk_.sidestep_time_k_l = 1.0f + xp_mv_walk_.sidestep_time_k * abs_y_percent;
    }
  }
  else
  {  // walk straight
    flag_walk_.y = STRAIGHT;
    if (xv_mv_walk_.theta_percent_dlim > EPS_DATA)
    {  // turn right
      flag_walk_.turn = RIGHT;
    }
    else if (xv_mv_walk_.theta_percent_dlim < -EPS_DATA)
    {  // turn left
      flag_walk_.turn = LEFT;
    }
    else
    {
      flag_walk_.turn = STRAIGHT;  // no turn
    }
    xv_mv_walk_.sidestep_roll = 0.f;
    xv_mv_walk_.sidestep_time_k_r = 1.0f;
    xv_mv_walk_.sidestep_time_k_l = 1.0f;
  }

  // gyro yaw feedback control
  gyro_->gyro_yaw_cntr_fun();  // yaw control

  // walk sequence
  sq_walk_fun();

  return flag_md_walk_end_;
}

void SqWalk::side_step_modify(float t1, float t2, float t1a, float t1b, float t1c, float t2a, float* t1_r, float* t2_r,
                              float* t1a_r, float* t1b_r, float* t1c_r, float* t2a_r, float* t1_l, float* t2_l,
                              float* t1a_l, float* t1b_l, float* t1c_l, float* t2a_l, float* _xv_mv_walk_y_swg,
                              float* _xv_mv_walk_y_spt, float* _xv_posture_roll2)
{
  // calculation side step time and change side step direction
  // right leg
  *t1_r = t1 * xv_mv_walk_.sidestep_time_k_r;    // period / 2
  *t2_r = t2 * xv_mv_walk_.sidestep_time_k_r;    // period / 4
  *t1a_r = t1a * xv_mv_walk_.sidestep_time_k_r;  // swing leg
  *t1b_r = t1b * xv_mv_walk_.sidestep_time_k_r;  // support leg at period / 2
  *t1c_r = t1c * xv_mv_walk_.sidestep_time_k_r;  // support leg at period
  *t2a_r = t2a * xv_mv_walk_.sidestep_time_k_r;  // swing leg at period / 2

  // left leg
  *t1_l = t1 * xv_mv_walk_.sidestep_time_k_l;    // period / 2
  *t2_l = t2 * xv_mv_walk_.sidestep_time_k_l;    // period / 4
  *t1a_l = t1a * xv_mv_walk_.sidestep_time_k_l;  // swing leg
  *t1b_l = t1b * xv_mv_walk_.sidestep_time_k_l;  // support leg at period / 2
  *t1c_l = t1c * xv_mv_walk_.sidestep_time_k_l;  // support leg at period
  *t2a_l = t2a * xv_mv_walk_.sidestep_time_k_l;  // swing leg at period / 2

  // leg chage
  *_xv_mv_walk_y_swg = xv_mv_walk_.y_swg;          // amount of swing leg movement when walking sideways
  *_xv_mv_walk_y_spt = xv_mv_walk_.y_spt;          // support leg movement
  *_xv_posture_roll2 = xv_mv_walk_.sidestep_roll;  // roll angle
}

void SqWalk::sq_walk_fun()
{
  float work;
  if (accurate_one_step_mode_ == 1)
  {
    t1 = xv_mv_walk_.accurate_step_time;
    t2 = xv_mv_walk_.accurate_step_time / 2.f;

    t1a = xv_mv_walk_.accurate_step_time * xv_mv_walk_.time_dutyfactor;
    t1b = xv_mv_walk_.accurate_step_time - t1a;
    t1c = xv_mv_walk_.accurate_step_time * 2.f - t1a;
    t2a = t1a / 2.f;

    gyro_->flag_gyro_.vib = ON;
  }
  else
  {
    t1 = xv_mv_walk_.time;  // walk period (one step)
    t2 = t1 / 2.f;          // half of the walk period

    t1a = xv_mv_walk_.time * xv_mv_walk_.time_dutyfactor;  // time during swing leg
    t1b = xv_mv_walk_.time - t1a;
    t1c = xv_mv_walk_.time * 2.f - t1a;  // time during support leg
    t2a = t1a / 2.f;                     // time during swing leg up

    gyro_->flag_gyro_.vib = ON;  // gain high
  }

  switch (mode_sq_walk_)
  {
    case SQ_WALK_INIT:             // init
      gyro_->flag_gyro_.vib = ON;  // gain normal

      calc_mv_->xv_mv_.count = 0;  // walk counter clear
      flag_md_walk_end_ = OFF;     // walk end flag OFF

      serv_->set_sw_ref_d(FOOT_XYZ);  // foot xyz position control

      // X trajectory table = lamp
      calc_mv_->xv_data_x_r_.mv_tbl_select = MV_TBL_LAMP;  // right foot table is lamp function
      calc_mv_->xv_data_x_l_.mv_tbl_select = MV_TBL_LAMP;  // left foot table is lamp function
      calc_mv_->chg_mvtbl(&calc_mv_->xv_mvdata_[0]);       // reset trajectory table xv_mvdata[0]
      calc_mv_->chg_mvtbl(&calc_mv_->xv_mvdata_[3]);       // reset trajectory table xv_mvdata[3]

      // THETA trajectory table = lamp
      calc_mv_->xv_data_d_[LEG_YAW_R].mv_tbl_select = MV_TBL_LAMP;  // right yaw is lamp function
      calc_mv_->xv_data_d_[LEG_YAW_L].mv_tbl_select = MV_TBL_LAMP;  // left yaw is lamp function
      calc_mv_->chg_mvtbl(&calc_mv_->xv_mvdata_d_[LEG_YAW_R]);      // reset trajectory table xv_mvdata[LEG_YAW_R]
      calc_mv_->chg_mvtbl(&calc_mv_->xv_mvdata_d_[LEG_YAW_L]);      // reset trajectory table xv_mvdata[LEG_YAW_L]

      // Y(xmp) trajectory table = lamp
      calc_mv_->xv_data_y_r_.mv_tbl_select = mv_tbl_zmp_sel_;  // set to MV_TBL_ZMP2
      calc_mv_->xv_data_y_l_.mv_tbl_select = mv_tbl_zmp_sel_;  // set to MV_TBL_ZMP2
      calc_mv_->chg_mvtbl(&calc_mv_->xv_mvdata_[1]);           // reset trajectory table xv_mvdata[1]
      calc_mv_->chg_mvtbl(&calc_mv_->xv_mvdata_[4]);           // reset trajectory table xv_mvdata[4]

      // Y(side step) trajectory table = lamp
      calc_mv_->xv_data_y_r2_.mv_tbl_select = mv_tbl_zmp_sel_;  // set to MV_TBL_ZMP2
      calc_mv_->xv_data_y_l2_.mv_tbl_select = mv_tbl_zmp_sel_;  // set to MV_TBL_ZMP2
      calc_mv_->chg_mvtbl(&calc_mv_->xv_mvdata_[9]);            // reset trajectory table xv_mvdata[9]
      calc_mv_->chg_mvtbl(&calc_mv_->xv_mvdata_[10]);           // reset trajectory table xv_mvdata[10]

      flag_walk_.y_on = flag_walk_.y;        // store actual walk y direction
      flag_walk_.turn_on = flag_walk_.turn;  // store actual turn direction

      // modify side walk parameters
      side_step_modify(t1, t2, t1a, t1b, t1c, t2a, &t1_r, &t2_r, &t1a_r, &t1b_r, &t1c_r, &t2a_r, &t1_l, &t2_l, &t1a_l,
                       &t1b_l, &t1c_l, &t2a_l, &_xv_mv_walk_y_swg, &_xv_mv_walk_y_spt, &_xv_posture_roll2);

      mode_sq_time_ = 0.f;  // clear state time

      // status
      // When starting to walk, if moving sideways, lift the foot on that side first.
      // If turning next, also lift the foot in the direction you're moving.
      // If moving straight, lift the foot that was last put down.

      if (flag_walk_.y == RIGHT)  // sidestep right
      {
        mode_sq_walk_ = SQ_WALK_MV_R;  // move cog to left
      }
      else if (flag_walk_.y == LEFT)  // sidestep left
      {
        mode_sq_walk_ = SQ_WALK_MV_L;  // move cog to right
      }
      else
      {
        if (flag_walk_.turn == RIGHT)  // turn right
        {
          mode_sq_walk_ = SQ_WALK_MV_R;  // move cog to left
        }
        else if (flag_walk_.turn == LEFT)  // turn left
        {
          mode_sq_walk_ = SQ_WALK_MV_L;  // move cog to right
        }
        else
        {
          // start opposite leg
          if (flag_walk_.upleg_last == RIGHT)  // last step is right leg
          {
            mode_sq_walk_ = SQ_WALK_MV_L;  // move cog to right
          }
          else  // last step is left leg
          {
            mode_sq_walk_ = SQ_WALK_MV_R;  // move cog to left
          }
        }
      }
      break;

    case SQ_WALK_MV_L:  // move cog to left
      // fist step should be adjusted because not based on zmp
      calc_mv_->xv_data_y_r_.time = calc_mv_->xv_data_y_l_.time = t2 * xp_mv_walk_.start_time_k1;
      work = xv_mv_walk_.zmp * xp_mv_walk_.start_zmp_k1 + xp_mv_walk_.y_wide;
      calc_mv_->xv_data_y_r_.pos = std::clamp(work, -xp_mv_walk_.y_balance, xp_mv_walk_.y_balance);
      // calc_mv_->xv_data_y_r_.pos = work;
      work = xv_mv_walk_.zmp * xp_mv_walk_.start_zmp_k1 - xp_mv_walk_.y_wide;
      calc_mv_->xv_data_y_l_.pos = std::clamp(work, -xp_mv_walk_.y_balance, xp_mv_walk_.y_balance);
      // calc_mv_->xv_data_y_l_.pos = work;
      mode_sq_time_ = 0.f;

      if (++calc_mv_->xv_mv_.count >= xv_mv_walk_.num)
        mode_sq_walk_ = SQ_WALK_UP_R2;  // go to last step
      else
        mode_sq_walk_ = SQ_WALK_UP_R0;  // continue

      break;

    case SQ_WALK_MV_R:  // move cog to right
      calc_mv_->xv_data_y_r_.time = calc_mv_->xv_data_y_l_.time = t2 * xp_mv_walk_.start_time_k1;
      work = -xv_mv_walk_.zmp * xp_mv_walk_.start_zmp_k1 + xp_mv_walk_.y_wide;
      calc_mv_->xv_data_y_r_.pos = std::clamp(work, -xp_mv_walk_.y_balance, xp_mv_walk_.y_balance);
      work = -xv_mv_walk_.zmp * xp_mv_walk_.start_zmp_k1 - xp_mv_walk_.y_wide;
      calc_mv_->xv_data_y_l_.pos = std::clamp(work, -xp_mv_walk_.y_balance, xp_mv_walk_.y_balance);

      mode_sq_time_ = 0.f;

      if (++calc_mv_->xv_mv_.count >= xv_mv_walk_.num)
        mode_sq_walk_ = SQ_WALK_UP_L2;  // go to last step
      else
        mode_sq_walk_ = SQ_WALK_UP_L0;  // continue

      break;

    case SQ_WALK_UP_R0:  // up right leg for first step
      if (mode_sq_time_ < (t2 * xp_mv_walk_.start_time_k1 - EPS_TIME))
      {
        break;  // wait until previous state finish
      }

      mode_sq_walk_ = SQ_WALK_UP_R;

    case SQ_WALK_UP_R:               // up right leg
      flag_walk_.upleg = RIGHT;      // right leg to lift up
      if (flag_walk_.y_on == RIGHT)  // if moving to right
      {
        calc_mv_->xv_data_x_l_.time = t1c_r;  // time to move the left foot (support leg) forward and backward
        calc_mv_->xv_data_x_l_.pos =
            xv_mv_walk_.x_spt;  // distance to move the left foot (support leg) forward and backward
        calc_mv_->xv_data_x_l_.mv_tbl_select = MV_TBL_LAMP;  // left foot table is lamp function

        calc_mv_->xv_data_y_l2_.time = t1c_r;             // time to move the left foot (support leg) left and right
        calc_mv_->xv_data_y_l2_.pos = _xv_mv_walk_y_spt;  // distance to move the left foot (support leg) left and right
        calc_mv_->xv_data_roll2_.time = t2_r;             // time to move the roll axis
        calc_mv_->xv_data_roll2_.pos = _xv_posture_roll2;  // roll axis position
      }
      else if (flag_walk_.y_on == LEFT)
      {
        calc_mv_->xv_data_x_l_.time = t1c_r;  // time to move the left foot (support leg) forward and backward
        calc_mv_->xv_data_x_l_.pos =
            xv_mv_walk_.x_spt;  // distance to move the left foot (support leg) forward and backward
        calc_mv_->xv_data_x_l_.mv_tbl_select = MV_TBL_LAMP;  // left foot table is lamp function

        calc_mv_->xv_data_y_l2_.time = t1c_r;  // time to move the left foot (support leg) left and right
        calc_mv_->xv_data_y_l2_.pos = 0.f;     // distance to move the left foot (support leg) left and right
        calc_mv_->xv_data_roll2_.time = t2_r;  // time to move the roll axis
        calc_mv_->xv_data_roll2_.pos = _xv_posture_roll2;  // roll axis position
      }
      else
      {
        calc_mv_->xv_data_x_l_.time = t1c_r;  // time to move the left foot (support leg) forward and backward
        calc_mv_->xv_data_x_l_.pos =
            xv_mv_walk_.x_spt;  // distance to move the left foot (support leg) forward and backward
        calc_mv_->xv_data_x_l_.mv_tbl_select = MV_TBL_LAMP;  // left foot table is lamp function

        calc_mv_->xv_data_y_l2_.time = t1c_r;  // time to move the left foot (support leg) left and right
        calc_mv_->xv_data_y_l2_.pos = 0.f;     // distance to move the left foot (support leg) left and right
        calc_mv_->xv_data_roll2_.time = t2_r;  // time to move the roll axis
        calc_mv_->xv_data_roll2_.pos = 0.f;    // roll axis position
      }

      mode_sq_walk_ = SQ_WALK_UP_R_1;

      break;

    case SQ_WALK_UP_R_1:  // up right leg
      if (mode_sq_time_ < (t1b_r - EPS_TIME))
      {
        break;  // wait until previous state finish
      }

      // calc_mv_->xv_data_x_r_.pos = xv_mv_walk_.x_swg;  // distance to move the right foot (swing leg) forward and
      // backward calc_mv_->xv_data_x_r_.time = t1a_r;             // time to move the right foot (swing leg) forward
      // and backward
      calc_mv_->xv_data_x_r_.pos = 0.0;     // distance to move the right foot (swing leg) forward and backward
      calc_mv_->xv_data_x_r_.time = t2a_r;  // time to move the right foot (swing leg) forward and backward
      calc_mv_->xv_data_x_r_.mv_tbl_select = MV_TBL_X_UP;

      if (flag_walk_.y_on == RIGHT)
      {
        calc_mv_->xv_data_y_r2_.pos = _xv_mv_walk_y_swg;  // distance to move the right foot (swing leg) left and right
      }
      else
      {
        calc_mv_->xv_data_y_r2_.pos = 0.0f;
      }
      calc_mv_->xv_data_y_r2_.time = t1a_r;  // time to move the right foot (swing leg) left and right

      work = (accurate_one_step_mode_ == 1 ? -xv_mv_walk_.accurate_step_z : -xv_mv_walk_.z) +
             sq_ready_->xp_mv_ready_.z3;  // value obtained by subtracting the foot lift amount from the ready height
      calc_mv_->xv_data_z_r_.pos =
          std::max(work, Z3_LIMIT_L);  // distance to move the right foot (swing leg) up and down
      calc_mv_->xv_data_z_r_.mv_tbl_select =
          MV_TBL_Z_UP;                      // table for the trajectory of lifting the right foot (swing leg)
      calc_mv_->xv_data_z_r_.time = t2a_r;  // time to move the right foot (swing leg) up and down

      // arm swing
      calc_mv_->xv_data_d_[ARM_PITCH_R].time = t1_r;
      calc_mv_->xv_data_d_[ARM_PITCH_L].time = t1_r;
      calc_mv_->xv_data_d_[ELBOW_PITCH_R].time = t1_r;
      calc_mv_->xv_data_d_[ELBOW_PITCH_L].time = t1_r;
      calc_mv_->xv_data_d_[ARM_PITCH_R].pos = sq_ready_->xp_mv_ready_.arm_sh_pitch - xv_mv_walk_.arm_sh_pitch;
      calc_mv_->xv_data_d_[ARM_PITCH_L].pos = sq_ready_->xp_mv_ready_.arm_sh_pitch + xv_mv_walk_.arm_sh_pitch;
      calc_mv_->xv_data_d_[ELBOW_PITCH_R].pos = sq_ready_->xp_mv_ready_.arm_el_pitch - xv_mv_walk_.arm_el_pitch;
      calc_mv_->xv_data_d_[ELBOW_PITCH_L].pos = sq_ready_->xp_mv_ready_.arm_el_pitch + xv_mv_walk_.arm_el_pitch;

      // yaw control
      calc_mv_->xv_data_d_[LEG_YAW_R].time = calc_mv_->xv_data_d_[LEG_YAW_L].time = t1a_r;  // time to move yaw

      if (flag_walk_.turn_on == RIGHT)  // turn right
      {
        calc_mv_->xv_data_d_[LEG_YAW_R].pos = xv_mv_walk_.theta;  // position of right leg when turning
        calc_mv_->xv_data_d_[LEG_YAW_L].pos = -xv_mv_walk_.theta;
      }
      else
      {
        calc_mv_->xv_data_d_[LEG_YAW_R].pos = 0.f;
        calc_mv_->xv_data_d_[LEG_YAW_L].pos = 0.f;
      }

      mode_sq_walk_ = SQ_WALK_DW_R;
      mode_sq_time_ = 0.f;
      break;

    case SQ_WALK_DW_R:  // down right leg
      if (mode_sq_time_ < (t2a_r - RTC_TIME_SEC - EPS_TIME))
      {
        break;  // wait until previous state finish
      }

      calc_mv_->xv_data_y_l_.time = t1_r;  // time to move the right foot (swing leg) left and right (half cycle)
      calc_mv_->xv_data_y_l_.pos =
          -xv_mv_walk_.zmp - xp_mv_walk_.y_wide;  // position to move the right foot (swing leg) left and right

      calc_mv_->xv_data_y_r_.time = t1_r;  // time to move the left foot (support leg) left and right (half cycle)
      calc_mv_->xv_data_y_r_.pos =
          -xv_mv_walk_.zmp + xp_mv_walk_.y_wide;  // position to move the left foot (support leg) left and right

      calc_mv_->xv_data_x_r_.pos =
          xv_mv_walk_.x_swg;                // distance to move the right foot (swing leg) forward and backward
      calc_mv_->xv_data_x_r_.time = t2a_r;  // time to move the right foot (swing leg) forward and backward
      calc_mv_->xv_data_x_r_.mv_tbl_select = MV_TBL_X_DW;

      calc_mv_->xv_data_z_r_.pos =
          std::max(sq_ready_->xp_mv_ready_.z3, Z3_LIMIT_L);  // position to move the right foot (swing leg) up and down
      calc_mv_->xv_data_z_r_.mv_tbl_select =
          MV_TBL_Z_DW;                      // trajectory table for moving the right foot (swing leg) up and down
      calc_mv_->xv_data_z_r_.time = t2a_r;  // time to move the right foot (swing leg) up and down

      mode_sq_walk_ = SQ_WALK_MV_R2;
      mode_sq_time_ = 0.f;
      break;

    case SQ_WALK_MV_R2:  // move cog to right
      if (mode_sq_time_ < (t2a_r - RTC_TIME_SEC - EPS_TIME))
      {
        break;  // wait until previous state finish
      }

      flag_walk_.upleg = OFF;

      if (flag_walk_.y_on != RIGHT)
      {  // adjust side walk parameters
        side_step_modify(t1, t2, t1a, t1b, t1c, t2a, &t1_r, &t2_r, &t1a_r, &t1b_r, &t1c_r, &t2a_r, &t1_l, &t2_l, &t1a_l,
                         &t1b_l, &t1c_l, &t2a_l, &_xv_mv_walk_y_swg, &_xv_mv_walk_y_spt, &_xv_posture_roll2);

        flag_walk_.y_on = flag_walk_.y;  // store actual walk y direction
      }

      if (flag_walk_.turn_on != RIGHT)
      {
        flag_walk_.turn_on = flag_walk_.turn;  // store actual turn direction
      }

      if (++calc_mv_->xv_mv_.count >= xv_mv_walk_.num)
      {
        mode_sq_walk_ = SQ_WALK_UP_L2;  // go to last step
      }
      else
      {
        // mode_sq_walk_ = SQ_WALK_UP_L;  // continue
        flag_walk_.upleg = LEFT;       // lift leg up
        if (flag_walk_.y_on == RIGHT)  // if moving right
        {
          calc_mv_->xv_data_x_r_.time = t1c_l;  // time to move the right foot (support leg) forward and backward
          calc_mv_->xv_data_x_r_.pos =
              xv_mv_walk_.x_spt;  // distance to move the right foot (support leg) forward and backward
          calc_mv_->xv_data_x_r_.mv_tbl_select = MV_TBL_LAMP;  // right foot table is lamp function

          calc_mv_->xv_data_y_r2_.time = t1c_l;  // time to move the right foot (support leg) left and right
          calc_mv_->xv_data_y_r2_.pos = 0.f;     // distance to move the right foot (support leg) left and right
          calc_mv_->xv_data_roll2_.time = t2_l;  // time to move the roll axis
          calc_mv_->xv_data_roll2_.pos = _xv_posture_roll2;  // roll axis position
        }
        else if (flag_walk_.y_on == LEFT)
        {
          calc_mv_->xv_data_x_r_.time = t1c_l;  // time to move the right foot (support leg) forward and backward
          calc_mv_->xv_data_x_r_.pos =
              xv_mv_walk_.x_spt;  // distance to move the right foot (support leg) forward and backward
          calc_mv_->xv_data_x_r_.mv_tbl_select = MV_TBL_LAMP;  // right foot table is lamp function

          calc_mv_->xv_data_y_r2_.time = t1c_l;  // time to move the right foot (support leg) left and right
          calc_mv_->xv_data_y_r2_.pos =
              _xv_mv_walk_y_spt;                 // distance to move the right foot (support leg) left and right
          calc_mv_->xv_data_roll2_.time = t2_l;  // time to move the roll axis
          calc_mv_->xv_data_roll2_.pos = _xv_posture_roll2;  // roll axis position
        }
        else
        {
          calc_mv_->xv_data_x_r_.time = t1c_l;  // time to move the right foot (support leg) forward and backward
          calc_mv_->xv_data_x_r_.pos =
              xv_mv_walk_.x_spt;  // distance to move the right foot (support leg) forward and backward
          calc_mv_->xv_data_x_r_.mv_tbl_select = MV_TBL_LAMP;  // right foot table is lamp function

          calc_mv_->xv_data_y_r2_.time = t1c_l;  // time to move the right foot (support leg) left and right
          calc_mv_->xv_data_y_r2_.pos = 0.f;     // distance to move the right foot (support leg) left and right
          calc_mv_->xv_data_roll2_.time = t2_l;  // time to move the roll axis
          calc_mv_->xv_data_roll2_.pos = 0.f;    // roll axis position
        }

        mode_sq_walk_ = SQ_WALK_UP_L_1;
      }

      mode_sq_time_ = 0.f;
      break;

    case SQ_WALK_UP_L0:  // up left leg for first step
      if (mode_sq_time_ < (t2 * xp_mv_walk_.start_time_k1 - EPS_TIME))
      {
        break;  // wait until previous state finish
      }

      mode_sq_walk_ = SQ_WALK_UP_L;
      // break;

    case SQ_WALK_UP_L:               // up left leg
      flag_walk_.upleg = LEFT;       // left leg to lift up
      if (flag_walk_.y_on == RIGHT)  // if moving to right
      {
        calc_mv_->xv_data_x_r_.time = t1c_l;  // time to move the right foot (support leg) forward and backward
        calc_mv_->xv_data_x_r_.pos =
            xv_mv_walk_.x_spt;  // distance to move the right foot (support leg) forward and backward
        calc_mv_->xv_data_x_r_.mv_tbl_select = MV_TBL_LAMP;  // right foot table is lamp function

        calc_mv_->xv_data_y_r2_.time = t1c_l;  // time to move the right foot (support leg) left and right
        calc_mv_->xv_data_y_r2_.pos = 0.f;     // distance to move the right foot (support leg) left and right
        calc_mv_->xv_data_roll2_.time = t2_l;  // time to move the roll axis
        calc_mv_->xv_data_roll2_.pos = _xv_posture_roll2;  // roll axis position
      }
      else if (flag_walk_.y_on == LEFT)
      {
        calc_mv_->xv_data_x_r_.time = t1c_l;  // time to move the right foot (support leg) forward and backward
        calc_mv_->xv_data_x_r_.pos =
            xv_mv_walk_.x_spt;  // distance to move the right foot (support leg) forward and backward
        calc_mv_->xv_data_x_r_.mv_tbl_select = MV_TBL_LAMP;  // right foot table is lamp function

        calc_mv_->xv_data_y_r2_.time = t1c_l;  // time to move the right foot (support leg) left and right
        calc_mv_->xv_data_y_r2_.pos =
            _xv_mv_walk_y_spt;                 // distance to move the right foot (support leg) left and right
        calc_mv_->xv_data_roll2_.time = t2_l;  // time to move the roll axis
        calc_mv_->xv_data_roll2_.pos = _xv_posture_roll2;  // roll axis position
      }
      else
      {
        calc_mv_->xv_data_x_r_.time = t1c_r;  // time to move the right foot (support leg) forward and backward
        calc_mv_->xv_data_x_r_.pos = xv_mv_walk_.x_spt;      // distance to move the right foot (support leg)
        calc_mv_->xv_data_x_r_.mv_tbl_select = MV_TBL_LAMP;  // right foot table is lamp function

        calc_mv_->xv_data_y_r2_.time = t1c_r;  // time to move the right foot (support leg) left and right
        calc_mv_->xv_data_y_r2_.pos = 0.f;     // distance to move the right foot (support leg) left and right
        calc_mv_->xv_data_roll2_.time = t2_r;  // time to move the roll axis
        calc_mv_->xv_data_roll2_.pos = 0.f;    // roll axis position
      }

      mode_sq_walk_ = SQ_WALK_UP_L_1;
      break;

    case SQ_WALK_UP_L_1:  // up left leg
      if (mode_sq_time_ < (t1b_l - EPS_TIME))
      {
        break;  // wait until previous state finish
      }

      // calc_mv_->xv_data_x_l_.pos = xv_mv_walk_.x_swg;  // distance to move the left foot (swing leg) forward and
      // backward calc_mv_->xv_data_x_l_.time = t1a_l;             // time to move the left foot (swing leg) forward and
      // backward
      calc_mv_->xv_data_x_l_.pos = 0.0;     // distance to move the left foot (swing leg) forward and backward
      calc_mv_->xv_data_x_l_.time = t2a_l;  // time to move the left foot (swing leg) forward and backward
      calc_mv_->xv_data_x_l_.mv_tbl_select = MV_TBL_X_UP;

      if (flag_walk_.y_on == LEFT)
      {
        calc_mv_->xv_data_y_l2_.pos = _xv_mv_walk_y_swg;  // distance to move the left foot (swing leg) left and right
      }
      else
      {
        calc_mv_->xv_data_y_l2_.pos = 0.0f;
      }
      calc_mv_->xv_data_y_l2_.time = t1a_l;  // time to move the left foot (swing leg) left and right

      work = (accurate_one_step_mode_ == 1 ? -xv_mv_walk_.accurate_step_z : -xv_mv_walk_.z) +
             sq_ready_->xp_mv_ready_.z3;  // value obtained by subtracting the foot lift amount from the ready height
      calc_mv_->xv_data_z_l_.pos =
          std::max(work, Z3_LIMIT_L);  // distance to move the left foot (swing leg) up and down
      calc_mv_->xv_data_z_l_.mv_tbl_select =
          MV_TBL_Z_UP;                      // table for the trajectory of lifting the left foot (swing leg)
      calc_mv_->xv_data_z_l_.time = t2a_l;  // time to move the left foot (swing leg) up and down

      // arm swing
      calc_mv_->xv_data_d_[ARM_PITCH_R].time = t1_l;
      calc_mv_->xv_data_d_[ARM_PITCH_L].time = t1_l;
      calc_mv_->xv_data_d_[ELBOW_PITCH_R].time = t1_l;
      calc_mv_->xv_data_d_[ELBOW_PITCH_L].time = t1_l;
      calc_mv_->xv_data_d_[ARM_PITCH_R].pos = sq_ready_->xp_mv_ready_.arm_sh_pitch + xv_mv_walk_.arm_sh_pitch;
      calc_mv_->xv_data_d_[ARM_PITCH_L].pos = sq_ready_->xp_mv_ready_.arm_sh_pitch - xv_mv_walk_.arm_sh_pitch;
      calc_mv_->xv_data_d_[ELBOW_PITCH_R].pos = sq_ready_->xp_mv_ready_.arm_el_pitch + xv_mv_walk_.arm_el_pitch;
      calc_mv_->xv_data_d_[ELBOW_PITCH_L].pos = sq_ready_->xp_mv_ready_.arm_el_pitch - xv_mv_walk_.arm_el_pitch;

      // yaw control
      calc_mv_->xv_data_d_[LEG_YAW_R].time = calc_mv_->xv_data_d_[LEG_YAW_L].time = t1a_l;  // time to move yaw

      if (flag_walk_.turn_on == LEFT)  // turn left
      {
        calc_mv_->xv_data_d_[LEG_YAW_R].pos = -xv_mv_walk_.theta;  // position of right leg when turning
        calc_mv_->xv_data_d_[LEG_YAW_L].pos = xv_mv_walk_.theta;
      }
      else
      {
        calc_mv_->xv_data_d_[LEG_YAW_R].pos = 0.f;
        calc_mv_->xv_data_d_[LEG_YAW_L].pos = 0.f;
      }

      mode_sq_walk_ = SQ_WALK_DW_L;
      mode_sq_time_ = 0.f;
      break;

    case SQ_WALK_DW_L:  // down left leg
      if (mode_sq_time_ < (t2a_l - RTC_TIME_SEC - EPS_TIME))
      {
        break;  // wait until previous state finish
      }

      calc_mv_->xv_data_y_l_.time = t1_l;  // time to move the left foot (swing leg) left and right (half cycle)
      calc_mv_->xv_data_y_l_.pos =
          xv_mv_walk_.zmp - xp_mv_walk_.y_wide;  // position to move the left foot (swing leg) left and right

      calc_mv_->xv_data_y_r_.time = t1_l;  // time to move the right foot (support leg) left and right (half cycle)
      calc_mv_->xv_data_y_r_.pos =
          xv_mv_walk_.zmp + xp_mv_walk_.y_wide;  // position to move the right foot (support leg) left and right

      calc_mv_->xv_data_x_l_.pos =
          xv_mv_walk_.x_swg;                // distance to move the left foot (swing leg) forward and backward
      calc_mv_->xv_data_x_l_.time = t2a_l;  // time to move the left foot (swing leg) forward and backward
      calc_mv_->xv_data_x_l_.mv_tbl_select = MV_TBL_X_DW;

      calc_mv_->xv_data_z_l_.pos =
          std::max(sq_ready_->xp_mv_ready_.z3, Z3_LIMIT_L);  // position to move the left foot (swing leg) up and down
      calc_mv_->xv_data_z_l_.mv_tbl_select =
          MV_TBL_Z_DW;                      // trajectory table for moving the left foot (swing leg) up and down
      calc_mv_->xv_data_z_l_.time = t2a_l;  // time to move the left foot (swing leg) up and down

      mode_sq_walk_ = SQ_WALK_MV_L2;
      mode_sq_time_ = 0.f;
      break;

    case SQ_WALK_MV_L2:  // move cog to left
      if (mode_sq_time_ < (t2a_l - EPS_TIME))
      {
        break;  // wait until previous state finish
      }

      flag_walk_.upleg = OFF;

      if (flag_walk_.y_on != LEFT)
      {  // adjust side walk parameters
        side_step_modify(t1, t2, t1a, t1b, t1c, t2a, &t1_r, &t2_r, &t1a_r, &t1b_r, &t1c_r, &t2a_r, &t1_l, &t2_l, &t1a_l,
                         &t1b_l, &t1c_l, &t2a_l, &_xv_mv_walk_y_swg, &_xv_mv_walk_y_spt, &_xv_posture_roll2);

        flag_walk_.y_on = flag_walk_.y;  // store actual walk y direction
      }

      if (flag_walk_.turn_on != LEFT)
      {
        flag_walk_.turn_on = flag_walk_.turn;  // store actual turn direction
      }

      if (++calc_mv_->xv_mv_.count >= xv_mv_walk_.num)
      {
        mode_sq_walk_ = SQ_WALK_UP_R2;  // go to last step
      }
      else
      {
        // mode_sq_walk_ = SQ_WALK_UP_R;  // continue
        flag_walk_.upleg = RIGHT;      // lift leg up
        if (flag_walk_.y_on == RIGHT)  // if moving right
        {
          calc_mv_->xv_data_x_l_.time = t1c_r;  // time to move the left foot (support leg) forward and backward
          calc_mv_->xv_data_x_l_.pos =
              xv_mv_walk_.x_spt;  // distance to move the left foot (support leg) forward and backward
          calc_mv_->xv_data_x_l_.mv_tbl_select = MV_TBL_LAMP;  // left foot table is lamp function

          calc_mv_->xv_data_y_l2_.time = t1c_r;  // time to move the left foot (support leg) left and right
          calc_mv_->xv_data_y_l2_.pos =
              _xv_mv_walk_y_spt;                 // distance to move the left foot (support leg) left and right
          calc_mv_->xv_data_roll2_.time = t2_r;  // time to move the roll axis
          calc_mv_->xv_data_roll2_.pos = _xv_posture_roll2;  // roll axis position
        }
        else if (flag_walk_.y_on == LEFT)
        {
          calc_mv_->xv_data_x_l_.time = t1c_r;  // time to move the left foot (support leg) forward and backward
          calc_mv_->xv_data_x_l_.pos =
              xv_mv_walk_.x_spt;  // distance to move the left foot (support leg) forward and backward
          calc_mv_->xv_data_x_l_.mv_tbl_select = MV_TBL_LAMP;  // left foot table is lamp function

          calc_mv_->xv_data_y_l2_.time = t1c_r;  // time to move the left foot (support leg) left and right
          calc_mv_->xv_data_y_l2_.pos = 0.f;     // distance to move the left foot (support leg) left and right
          calc_mv_->xv_data_roll2_.time = t2_r;  // time to move the roll axis
          calc_mv_->xv_data_roll2_.pos = _xv_posture_roll2;  // roll axis position
        }
        else
        {
          calc_mv_->xv_data_x_l_.time = t1c_r;  // time to move the left foot (support leg) forward and backward
          calc_mv_->xv_data_x_l_.pos =
              xv_mv_walk_.x_spt;  // distance to move the left foot (support leg) forward and backward
          calc_mv_->xv_data_x_l_.mv_tbl_select = MV_TBL_LAMP;  // left foot table is lamp function

          calc_mv_->xv_data_y_l2_.time = t1c_r;  // time to move the left foot (support leg) left and right
          calc_mv_->xv_data_y_l2_.pos = 0.f;     // distance to move the left foot (support leg) left and right
          calc_mv_->xv_data_roll2_.time = t2_r;  // time to move the roll axis
          calc_mv_->xv_data_roll2_.pos = 0.f;    // roll axis position
        }

        mode_sq_walk_ = SQ_WALK_UP_R_1;
      }

      mode_sq_time_ = 0.f;
      break;

    case SQ_WALK_UP_R2:  // up right leg for last step
      flag_walk_.upleg = RIGHT;
      calc_mv_->xv_data_x_l_.time = t1c_r;
      calc_mv_->xv_data_x_l_.pos = 0.f;
      calc_mv_->xv_data_x_l_.mv_tbl_select = MV_TBL_LAMP;

      calc_mv_->xv_data_y_l2_.time = t1c_r;
      calc_mv_->xv_data_y_l2_.pos = 0.f;

      mode_sq_walk_ = SQ_WALK_UP_R2_1;

      break;

    case SQ_WALK_UP_R2_1:  // up right leg for last step
      if (mode_sq_time_ < (t1b_r - EPS_TIME))
      {
        break;  // wait until previous state finish
      }

      calc_mv_->xv_data_z_r_.mv_tbl_select = MV_TBL_Z_UP;

      calc_mv_->xv_data_x_r_.time = t1a_r;
      calc_mv_->xv_data_z_r_.time = t2a_r;

      calc_mv_->xv_data_x_r_.pos = 0.f;
      calc_mv_->xv_data_x_r_.mv_tbl_select = MV_TBL_LAMP;

      calc_mv_->xv_data_y_r2_.time = t1a_r;
      calc_mv_->xv_data_y_r2_.pos = 0.f;

      work =
          (accurate_one_step_mode_ == 1 ? -xv_mv_walk_.accurate_step_z : -xv_mv_walk_.z) + sq_ready_->xp_mv_ready_.z3;
      calc_mv_->xv_data_z_r_.pos = std::max(work, Z3_LIMIT_L);

      calc_mv_->xv_data_z_l_.time = t2a_r;
      calc_mv_->xv_data_z_l_.pos = sq_ready_->xp_mv_ready_.z3;

      calc_mv_->xv_data_d_[ARM_PITCH_R].time = t1_r;
      calc_mv_->xv_data_d_[ARM_PITCH_L].time = t1_r;
      calc_mv_->xv_data_d_[ARM_PITCH_R].pos = sq_ready_->xp_mv_ready_.arm_sh_pitch - xv_mv_walk_.arm_sh_pitch;
      calc_mv_->xv_data_d_[ARM_PITCH_L].pos = sq_ready_->xp_mv_ready_.arm_sh_pitch + xv_mv_walk_.arm_sh_pitch;

      mode_sq_walk_ = SQ_WALK_UP_R2_2;
      mode_sq_time_ = 0.f;

      // break;

    case SQ_WALK_UP_R2_2:  // up right leg for last step
      calc_mv_->xv_data_d_[LEG_YAW_R].time = calc_mv_->xv_data_d_[LEG_YAW_L].time = t1a_r;

      calc_mv_->xv_data_d_[LEG_YAW_R].pos = 0.f;
      calc_mv_->xv_data_d_[LEG_YAW_L].pos = 0.f;

      mode_sq_walk_ = SQ_WALK_DW_R2;

      break;

    case SQ_WALK_DW_R2:  // down right leg for last step
      if (mode_sq_time_ < (t2a_r - EPS_TIME))
      {
        break;  // wait until previous state finish
      }

      calc_mv_->xv_data_z_r_.mv_tbl_select = MV_TBL_Z_DW;

      calc_mv_->xv_data_x_r_.time = t2a_r;

      calc_mv_->xv_data_z_r_.pos = std::max(sq_ready_->xp_mv_ready_.z3, Z3_LIMIT_L);

      calc_mv_->xv_data_z_l_.time = t2a_r;
      calc_mv_->xv_data_z_l_.pos = sq_ready_->xp_mv_ready_.z3;

      calc_mv_->xv_data_y_r_.time = calc_mv_->xv_data_y_l_.time = t1_r;
      calc_mv_->xv_data_y_r_.pos = -xv_mv_walk_.zmp + xp_mv_walk_.y_wide;
      calc_mv_->xv_data_y_l_.pos = -xv_mv_walk_.zmp - xp_mv_walk_.y_wide;
      calc_mv_->xv_data_roll2_.time = t2_r;
      calc_mv_->xv_data_roll2_.pos = 0.f;

      mode_sq_walk_ = SQ_WALK_READY;
      mode_sq_time_ = 0.f;

      break;

    case SQ_WALK_UP_L2:  // up left leg for last step
      flag_walk_.upleg = LEFT;
      calc_mv_->xv_data_x_r_.time = t1c_l;
      calc_mv_->xv_data_x_r_.pos = 0.f;
      calc_mv_->xv_data_x_r_.mv_tbl_select = MV_TBL_LAMP;

      calc_mv_->xv_data_y_r2_.time = t1c_l;
      calc_mv_->xv_data_y_r2_.pos = 0.f;

      mode_sq_walk_ = SQ_WALK_UP_L2_1;

      break;

    case SQ_WALK_UP_L2_1:  // up left leg for last step
      if (mode_sq_time_ < (t1b_l - EPS_TIME))
      {
        break;  // wait until previous state finish
      }

      calc_mv_->xv_data_z_l_.mv_tbl_select = MV_TBL_Z_UP;

      calc_mv_->xv_data_x_l_.time = t1a_l;
      calc_mv_->xv_data_z_l_.time = t2a_l;

      calc_mv_->xv_data_x_l_.pos = 0.f;
      calc_mv_->xv_data_x_l_.mv_tbl_select = MV_TBL_LAMP;

      calc_mv_->xv_data_y_l2_.time = t1a_l;
      calc_mv_->xv_data_y_l2_.pos = 0.f;

      work =
          (accurate_one_step_mode_ == 1 ? -xv_mv_walk_.accurate_step_z : -xv_mv_walk_.z) + sq_ready_->xp_mv_ready_.z3;
      calc_mv_->xv_data_z_l_.pos = std::max(work, Z3_LIMIT_L);

      calc_mv_->xv_data_z_r_.time = t2a_l;
      calc_mv_->xv_data_z_r_.pos = sq_ready_->xp_mv_ready_.z3;

      calc_mv_->xv_data_d_[ARM_PITCH_R].time = t1_l;
      calc_mv_->xv_data_d_[ARM_PITCH_L].time = t1_l;
      calc_mv_->xv_data_d_[ARM_PITCH_R].pos = sq_ready_->xp_mv_ready_.arm_sh_pitch + xv_mv_walk_.arm_sh_pitch;
      calc_mv_->xv_data_d_[ARM_PITCH_L].pos = sq_ready_->xp_mv_ready_.arm_sh_pitch - xv_mv_walk_.arm_sh_pitch;

      mode_sq_walk_ = SQ_WALK_UP_L2_2;
      mode_sq_time_ = 0.f;

      // break;

    case SQ_WALK_UP_L2_2:  // up left leg for last step
      calc_mv_->xv_data_d_[LEG_YAW_R].time = calc_mv_->xv_data_d_[LEG_YAW_L].time = t1a_l;

      calc_mv_->xv_data_d_[LEG_YAW_R].pos = 0.f;
      calc_mv_->xv_data_d_[LEG_YAW_L].pos = 0.f;

      mode_sq_walk_ = SQ_WALK_DW_L2;

      break;

    case SQ_WALK_DW_L2:  // down left leg for last step
      if (mode_sq_time_ < (t2a_l - EPS_TIME))
      {
        break;  // wait until previous state finish
      }

      calc_mv_->xv_data_z_l_.mv_tbl_select = MV_TBL_Z_DW;

      calc_mv_->xv_data_x_l_.time = t2a_l;

      calc_mv_->xv_data_z_l_.pos = std::max(sq_ready_->xp_mv_ready_.z3, Z3_LIMIT_L);

      calc_mv_->xv_data_z_r_.time = t2a_l;
      calc_mv_->xv_data_z_r_.pos = sq_ready_->xp_mv_ready_.z3;

      calc_mv_->xv_data_y_l_.time = calc_mv_->xv_data_y_r_.time = t1_l;
      calc_mv_->xv_data_y_l_.pos = xv_mv_walk_.zmp - xp_mv_walk_.y_wide;
      calc_mv_->xv_data_y_r_.pos = xv_mv_walk_.zmp + xp_mv_walk_.y_wide;
      calc_mv_->xv_data_roll2_.time = t2_l;
      calc_mv_->xv_data_roll2_.pos = 0.f;

      mode_sq_walk_ = SQ_WALK_READY;
      mode_sq_time_ = 0.f;

      break;

    case SQ_WALK_READY:            // ready position
      gyro_->flag_gyro_.vib = ON;  // normal gain

      if (mode_sq_time_ < (t2a - RTC_TIME_SEC * 2 - EPS_TIME))
      {
        break;  // wait until previous state finish
      }

      flag_walk_.upleg = OFF;
      flag_walk_.y_on = STRAIGHT;
      flag_walk_.turn_on = STRAIGHT;

      calc_mv_->xv_data_z_r_.mv_tbl_select = MV_TBL_SIN;
      calc_mv_->xv_data_z_l_.mv_tbl_select = MV_TBL_SIN;

      calc_mv_->xv_data_y_r_.time = t1;
      calc_mv_->xv_data_z_r_.time = t1;
      calc_mv_->xv_data_y_l_.time = t1;
      calc_mv_->xv_data_z_l_.time = t1;
      calc_mv_->xv_data_roll2_.time = t2;

      calc_mv_->xv_data_y_r_.pos = xp_mv_walk_.y_wide;
      calc_mv_->xv_data_z_r_.pos = sq_ready_->xp_mv_ready_.z3;
      calc_mv_->xv_data_y_l_.pos = -xp_mv_walk_.y_wide;
      calc_mv_->xv_data_z_l_.pos = sq_ready_->xp_mv_ready_.z3;
      calc_mv_->xv_data_pitch_.pos = sq_ready_->xp_mv_ready_.pitch;
      calc_mv_->xv_data_roll2_.pos = 0.f;

      calc_mv_->xv_data_d_[ARM_PITCH_R].time = t1;
      calc_mv_->xv_data_d_[ARM_PITCH_L].time = t1;
      calc_mv_->xv_data_d_[ELBOW_PITCH_R].time = t1;
      calc_mv_->xv_data_d_[ELBOW_PITCH_L].time = t1;
      calc_mv_->xv_data_d_[ARM_ROLL_R].time = t1;
      calc_mv_->xv_data_d_[ARM_ROLL_L].time = t1;

      calc_mv_->xv_data_d_[ARM_PITCH_L].pos = sq_ready_->xp_mv_ready_.arm_sh_pitch;
      calc_mv_->xv_data_d_[ARM_PITCH_R].pos = sq_ready_->xp_mv_ready_.arm_sh_pitch;
      calc_mv_->xv_data_d_[ARM_ROLL_L].pos = -sq_ready_->xp_mv_ready_.arm_sh_roll;
      calc_mv_->xv_data_d_[ARM_ROLL_R].pos = sq_ready_->xp_mv_ready_.arm_sh_roll;
      calc_mv_->xv_data_d_[ELBOW_PITCH_L].pos = sq_ready_->xp_mv_ready_.arm_el_pitch;
      calc_mv_->xv_data_d_[ELBOW_PITCH_R].pos = sq_ready_->xp_mv_ready_.arm_el_pitch;

      mode_sq_walk_ = SQ_WALK_END;
      mode_sq_time_ = 0.f;

      break;

    case SQ_WALK_END:              // end of walking
      gyro_->flag_gyro_.vib = ON;  // normal gain

      motion_->sq_flag_.walk = OFF;
      flag_md_walk_end_ = ON;
      mode_sq_walk_ = SQ_WALK_INIT;
      mode_sq_time_ = 0.f;

      calc_mv_->xv_mv_.count = 0;
      xv_mv_walk_.num = xp_mv_walk_.num;
      calc_mv_->xv_data_pitch_.pos = sq_ready_->xp_mv_ready_.pitch;

      accurate_one_step_mode_ = 0;
      break;

    default:
      break;
  }

  mode_sq_time_ += RTC_TIME_SEC;  // count up state time

  // store last leg for next start
  if (flag_walk_.upleg == RIGHT || flag_walk_.upleg == LEFT)
  {
    flag_walk_.upleg_last = flag_walk_.upleg;
  }
}
}  // namespace hr46
