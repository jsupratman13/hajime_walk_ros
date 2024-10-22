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
// calculation trajectory

#include <cmath>
#include "hr46/calc_mv.hpp"
#include "hr46/func.hpp"
#include "hr46/motion.hpp"
#include "hr46/mvtbl.hpp"

namespace hr46
{

void CalcMv::calc_mv_init(float z3)
{
  int i;
  for (i = 0; i < SERV_NUM; i++)
  {
    xv_mvdata_d_[i].t = 0.0f;
    xv_mvdata_d_[i].mv_tbl = MV_TBL_PI;
    xv_mvdata_d_[i].dt = MV_TBL_PI;
    xv_mvdata_d_[i].start = 0.0f;
    xv_mvdata_d_[i].amp = 0.0f;
    xv_mvdata_d_[i].out_old = 0.0f;
    xv_mvdata_d_[i].pos_old = 0.0f;
  }
  for (i = 0; i < MVDATA_NUM; i++)
  {
    xv_mvdata_[i].t = 0.0f;
    xv_mvdata_[i].mv_tbl = MV_TBL_PI;
    xv_mvdata_[i].dt = MV_TBL_PI;
    xv_mvdata_[i].start = 0.0f;
    xv_mvdata_[i].amp = 0.0f;
    xv_mvdata_[i].out_old = 0.0f;
    xv_mvdata_[i].pos_old = 0.0f;
  }

  for (i = 0; i < SERV_NUM; i++)
  {
    xv_data_d_[i].time = 0.1f;
    xv_data_d_[i].pos = 0.0f;
    xv_data_d_[i].mv_tbl_select = 0;
  }

  xv_data_x_r_.time = 0.1f;
  xv_data_x_r_.pos = 0.0f;
  xv_data_x_r_.mv_tbl_select = 0;
  xv_data_y_r_.time = 0.1f;
  xv_data_y_r_.pos = 0.0f;
  xv_data_y_r_.mv_tbl_select = 0;
  xv_data_y_r2_.time = 0.1f;
  xv_data_y_r2_.pos = 0.0f;
  xv_data_y_r2_.mv_tbl_select = 0;
  xv_data_z_r_.time = 0.1f;
  xv_data_z_r_.pos = z3;  // xp_mv_straight.z3;
  xv_data_z_r_.mv_tbl_select = 0;
  xv_data_x_l_.time = 0.1f;
  xv_data_x_l_.pos = 0.0f;
  xv_data_x_l_.mv_tbl_select = 0;
  xv_data_y_l_.time = 0.1f;
  xv_data_y_l_.pos = 0.0f;
  xv_data_y_l_.mv_tbl_select = 0;
  xv_data_y_l2_.time = 0.1f;
  xv_data_y_l2_.pos = 0.0f;
  xv_data_y_l2_.mv_tbl_select = 0;
  xv_data_z_l_.time = 0.1f;
  xv_data_z_l_.pos = z3;  // xp_mv_straight.z3;
  xv_data_z_l_.mv_tbl_select = 0;
  xv_data_pitch_.time = 0.1f;
  xv_data_pitch_.pos = 0.0f;
  xv_data_pitch_.mv_tbl_select = 0;
  xv_data_roll2_.time = 0.1f;
  xv_data_roll2_.pos = 0.0f;
  xv_data_roll2_.mv_tbl_select = 0;

  xv_mvdata_[2].start = xv_mvdata_[2].out_old = xv_mvdata_[2].pos_old = z3;  // xp_mv_straight.z3;
  xv_mvdata_[5].start = xv_mvdata_[5].out_old = xv_mvdata_[5].pos_old = z3;  // xp_mv_straight.z3;

  xv_odometry_.moveX = 0.0f;
  xv_odometry_.moveY = 0.0f;
  xv_odometry_.rotZ = 0.0f;
  xv_odometry_.x[0] = xv_odometry_.x[1] = 0.0f;
  xv_odometry_.x_old[0] = xv_odometry_.x_old[1] = 0.0f;
  xv_odometry_.y[0] = xv_odometry_.y[1] = 0.0f;
  xv_odometry_.y_old[0] = xv_odometry_.y_old[1] = 0.0f;
  xv_odometry_.theta[0] = xv_odometry_.theta[1] = 0.0f;
  xv_odometry_.theta_old[0] = xv_odometry_.theta_old[1] = 0.0f;

  odometry_correct_para_x_ = 1.0;  // odometry coefficient
  odometry_correct_para_y_ = 1.0;  // odometry coefficient

  xv_mv_.count = 0;  // step counter
}

void CalcMv::calc_mv()
{
  int i;
  float _xv_kine0y, _xv_kine1y, _xv_kine0y2, _xv_kine1y2;
  float _tmp_x, _tmp_y, _sinx, _cosx;

  // calculate trajectory tables for joint angles
  for (i = 0; i < SERV_NUM; i++)
  {
    serv_->xv_ref_.d_ref[i] = calc_mvdata(&xv_mvdata_d_[i], &xv_data_d_[i]);
  }

  if (motion_->mode_motion_ != MOTION_MOTION)
  {
    // calculate trajectory tables for inv kinetics XY point
    kine_->xv_kine_[0].x = calc_mvdata(&xv_mvdata_[0], &xv_data_x_r_);  // right foot x
    _xv_kine0y = calc_mvdata(&xv_mvdata_[1], &xv_data_y_r_);            // right foot y(zmp)
    kine_->xv_kine_[0].z = calc_mvdata(&xv_mvdata_[2], &xv_data_z_r_);  // right foot z
    kine_->xv_kine_[1].x = calc_mvdata(&xv_mvdata_[3], &xv_data_x_l_);  // left foot x
    _xv_kine1y = calc_mvdata(&xv_mvdata_[4], &xv_data_y_l_);            // left foot y(zmp)
    kine_->xv_kine_[1].z = calc_mvdata(&xv_mvdata_[5], &xv_data_z_l_);  // left foot z
    _xv_kine0y2 = calc_mvdata(&xv_mvdata_[9], &xv_data_y_r2_);          // right foot y(side step)
    _xv_kine1y2 = calc_mvdata(&xv_mvdata_[10], &xv_data_y_l2_);         // left foot y(side step)
    kine_->xv_kine_[0].y = _xv_kine0y + _xv_kine0y2;                    // right foot y total
    kine_->xv_kine_[1].y = _xv_kine1y + _xv_kine1y2;                    // left foot y total

    // calculate trajectory tables for pitch and roll posture
    kine_->xv_posture_.pitch = calc_mvdata(&xv_mvdata_[6], &xv_data_pitch_);  // TODO should be ankle pitch
    // kine_->xv_posture_.pitch = xv_data_pitch_.pos;  // TODO should be ankle pitch
    kine_->xv_posture_.roll2 = calc_mvdata(&xv_mvdata_[11], &xv_data_roll2_);  // roll
  }

  // calculate odometry
  xv_odometry_.x[0] = kine_->xv_kine_[0].x;
  xv_odometry_.x[1] = kine_->xv_kine_[1].x;
  xv_odometry_.y[0] = kine_->xv_kine_[0].y;
  xv_odometry_.y[1] = kine_->xv_kine_[1].y;
  xv_odometry_.theta[0] = serv_->xv_ref_.d[LEG_YAW_R];
  xv_odometry_.theta[1] = serv_->xv_ref_.d[LEG_YAW_L];

  if (sq_walk_->flag_walk_.upleg == LEFT)
  {
    xv_odometry_.rotZ -= (xv_odometry_.theta[0] - xv_odometry_.theta_old[0]);  // tracks of right leg
  }
  else
  {
    xv_odometry_.rotZ -= (xv_odometry_.theta[1] - xv_odometry_.theta_old[1]);  // tracks of left leg
  }

  if (sq_walk_->flag_walk_.upleg == LEFT)
  {
    // tracks of right leg
    _tmp_x = (xv_odometry_.x[0] - xv_odometry_.x_old[0]) * odometry_correct_para_x_;
    _tmp_y = ((xv_odometry_.y[0] - xv_odometry_.y[1]) - (xv_odometry_.y_old[0] - xv_odometry_.y_old[1])) *
             odometry_correct_para_y_;
  }
  else
  {
    // tracks of left leg
    _tmp_x = (xv_odometry_.x[1] - xv_odometry_.x_old[1]) * odometry_correct_para_x_;
    _tmp_y = ((xv_odometry_.y[1] - xv_odometry_.y[0]) - (xv_odometry_.y_old[1] - xv_odometry_.y_old[0])) *
             odometry_correct_para_y_;
  }

  _sinx = std::sin(hr46::rad2deg(xv_odometry_.rotZ));
  _cosx = std::cos(hr46::rad2deg(xv_odometry_.rotZ));

  xv_odometry_.moveX -= (_tmp_x * _cosx) - (_tmp_y * _sinx);
  xv_odometry_.moveY -= (_tmp_x * _sinx) + (_tmp_y * _cosx);

  xv_odometry_.x_old[0] = xv_odometry_.x[0];
  xv_odometry_.x_old[1] = xv_odometry_.x[1];
  xv_odometry_.y_old[0] = xv_odometry_.y[0];
  xv_odometry_.y_old[1] = xv_odometry_.y[1];
  xv_odometry_.theta_old[0] = xv_odometry_.theta[0];
  xv_odometry_.theta_old[1] = xv_odometry_.theta[1];
}

// calculate trajectory table
float CalcMv::calc_mvdata(st_xv_mvdata* x, st_xv_data* a)
{
  float w1;

  w1 = a->pos - x->pos_old;

  if (std::fabs(w1) > EPS)  // sv data changed
  {
    // set mv_data
    x->t = 0.0f;
    x->mv_tbl = MV_TBL_PI;
    x->dt = MV_TBL_PI / a->time / RTC_TICK;
    x->start = x->out_old;
    x->amp = a->pos - x->out_old;
  }

  x->t = hr46::dlimit(x->mv_tbl, x->dt, x->t);

  x->out_old = x->amp * mv_tbl_->mv_tbl_[a->mv_tbl_select][static_cast<int>(x->t + 0.5f)] + x->start;
  x->pos_old = a->pos;

  return x->out_old;
}

// reset trajectory data
void CalcMv::chg_mvtbl(st_xv_mvdata* x, st_xv_data* a)
{
  x->t = 0.0f;
  x->mv_tbl = MV_TBL_PI;
  x->dt = MV_TBL_PI;
  x->amp = 0.0f;
  x->start = x->pos_old = x->out_old;
}
}  // namespace hr46
