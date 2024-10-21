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

#include "hr46/kine.hpp"
#include "hr46/var.hpp"

namespace hr46
{
Kine::Kine()
{
  // inv. kinematics initial data = straight leg length
  xv_kine_[0].z = Z3_LIMIT_H;  // waist height
  xv_kine_[1].z = Z3_LIMIT_H;  // waist height
  for (int i = 0; i < 2; i++)
  {
    xv_kine_[i].x = 0.0f;       // x coordinate (mm)
    xv_kine_[i].y = 0.0f;       // y coordinate (mm)
    xv_kine_[i].z = 0.0f;       // z coordinate (mm)
    xv_kine_[i].yaw = 0.0f;     // hip yaw angle (deg)
    xv_kine_[i].hip_r = 0.0f;   // hip roll angle (deg)
    xv_kine_[i].leg = 0.0f;     // hip pitch angle (deg)
    xv_kine_[i].knee = 0.0f;    // knee pitch angle (deg)
    xv_kine_[i].foot_p = 0.0f;  // foot pitch angle (deg)
    xv_kine_[i].foot_r = 0.0f;  // foot roll angle (deg)
  }
  xv_posture_.pitch = 0.0f;  // initialize top half posture
  xv_posture_.yaw = 0.0f;
  xv_posture_.roll2 = 0.0f;
}

void Kine::kine()
{
  /*** right leg : xv_kine[0] ***/
  xv_kine_[0].yaw = serv_->xv_ref_.d_ref[11];   /* right hip yaw angle */
  kine_fun(&xv_kine_[0].x, &xv_kine_[0].hip_r); /* calculate right leg */

  /*** left leg  : xv_kine[1] ***/
  xv_kine_[1].yaw = serv_->xv_ref_.d_ref[5];    /* left hip yaw angle */
  kine_fun(&xv_kine_[1].x, &xv_kine_[1].hip_r); /* calculate left leg */
}

// calculation kinematics
// input
//   u[0] = X
//   u[1] = Y
//   u[2] = Z
//   u[3] = yaw
// output
//   y[0] = hip(roll)
//   y[1] = hip(pitch)
//   y[2] = knee2(pitch)
//   y[3] = knee1(pitch)
void Kine::kine_fun(float* u, float* y)
{
  float _sinx;
  float _cosx;
  float w1, w2;

  _sinx = std::sin(hr46::deg2rad(*(u + 3)));
  _cosx = std::cos(hr46::deg2rad(*(u + 3)));

  xv_k_.x[3] = (*u) * _cosx + (*(u + 1)) * _sinx;
  xv_k_.y[3] = -(*u) * _sinx + (*(u + 1)) * _cosx;
  xv_k_.z[3] = (*(u + 2)) - L3;
  xv_k_.d[0] = std::asin(xv_k_.y[3] / xv_k_.z[3]);
  w1 = xv_k_.z[3] * xv_k_.z[3] + xv_k_.y[3] * xv_k_.y[3];
  w2 = w1 - xv_k_.x[3] * xv_k_.x[3];
  if (w2 < 0.0f)
  {
    w2 = 0.0f;
  }
  xv_k_.z[3] = std::sqrt(w2);
  cal_inv_kine((st_xv_k*)&xv_k_.x);

  (*y) = hr46::rad2deg(xv_k_.d[0]);
  (*(y + 1)) = hr46::rad2deg(xv_k_.d[1]);
  (*(y + 2)) = hr46::rad2deg(xv_k_.d[2]);
  (*(y + 3)) = hr46::rad2deg(xv_k_.d[3]);
}

// calculation inverse kinematics
//    input x3, z3
//    output d1, d2, d3
//  @param[out] d[1]  hip angle (when the feet are no longer parallel)
//  @param[out] d[2]  ankle angle (angle from knee down)
//  @param[out] d[3]  hip angle
void Kine::cal_inv_kine(st_xv_k* a)
{
  float zz3;
  float w1;
  float w2;
  float w3;
  float w4;
  float w5;

  zz3 = a->z[3] - L01 - L12 - L23;  // parallel link leg

  w1 = std::atan2(a->x[3], zz3);  // pick angle

  w2 = a->x[3] * a->x[3] + zz3 * zz3;                                 // square distance from hip to ankle
  w3 = std::acos(std::clamp(std::sqrt(w2) / (2.0 * L1), -1.0, 1.0));  // knee angle / 2
  w4 = w1 + w3;
  w5 = std::atan2(a->x[3] - L1 * std::sin(w4), zz3 - L1 * std::cos(w4));  // parallel link leg

  a->d[1] = 0.0f;     // hip (pitch)
  a->d[2] = w4;       // knee2 (pitch)
  a->d[3] = w1 - w3;  // knee1 (pitch)
  // a->d[3] = std::clamp(w5, w5, w4); // knee1 (pitch)
}

/*------------------------------------------------------*/
/*	calculation kinematics								*/
/*	input d												*/
/*		xv_kine[0].hip_r								*/
/*		xv_kine[0].leg									*/
/*		xv_kine[0].knee									*/
/*		xv_kine[0].foot_p								*/
/*		xv_kine[0].foot_r								*/
/*	output x											*/
/*		xv_kine[0].x									*/
/*		xv_kine[0].y									*/
/*		xv_kine[0].z									*/
/*		xv_kine[0].yaw(input)							*/
/*------------------------------------------------------*/
void Kine::fwd_kine_fun(float* d, float* x)
{
  float _sinx;
  float _cosx;
  float x3, y3;

  xv_k2_.d[0] = hr46::deg2rad(*d);
  xv_k2_.d[1] = hr46::deg2rad(*(d + 1));
  xv_k2_.d[2] = hr46::deg2rad(*(d + 2));
  xv_k2_.d[3] = hr46::deg2rad(*(d + 3));
  xv_k2_.d[4] = hr46::deg2rad(*(d + 4));

  cal_fwd_kine((st_xv_k*)&xv_k2_.x);

  x3 = xv_k2_.x[3];
  y3 = xv_k2_.y[3];

  _sinx = std::sin(hr46::deg2rad(*(x + 3)));
  _cosx = std::cos(hr46::deg2rad(*(x + 3)));

  xv_k2_.x[3] = x3 * _cosx - y3 * _sinx;
  xv_k2_.y[3] = x3 * _sinx + y3 * _cosx;

  (*x) = xv_k2_.x[3];
  (*(x + 1)) = xv_k2_.y[3];
  (*(x + 2)) = xv_k2_.z[3];
}

/*--------------------------------------*/
/*	calclulation kinematics				*/
/*		input	d1, d2, d3				*/
/*		output	x0, z0, x1, z1, x2, z2 	*/
/*				x3, z3, x4, z4		 	*/
/*--------------------------------------*/
void Kine::cal_fwd_kine(st_xv_k* a)
{
  float w1;
  float w2;
  float w3;

  /*	parallel link leg	*/
  w3 = -a->d[3];      /*	foot(pitch)		*/
  a->d[3] -= a->d[2]; /*	knee1(pitch)	*/

  /*	parallel link leg	*/
  a->d[1] = a->d[2]; /*	hip(pitch)		*/
  a->d[2] = a->d[3]; /*	knee2(pitch)	*/
  a->d[3] = w3;      /*	knee1(pitch)	*/

  w1 = a->d[1] + a->d[2];
  w2 = w1 + a->d[3];

  a->x[0] = 0.0f;
  a->y[0] = 0.0f;
  a->z[0] = 0.0f;
  a->x[1] = a->x[0] + L1 * std::sin(a->d[1]);
  a->z[1] = a->z[0] + L01 + L1 * std::cos(a->d[1]);  //	 parallel link leg
  a->x[2] = a->x[1] + L2 * std::sin(w1);
  a->z[2] = a->z[1] + L12 + L2 * std::cos(w1);  //	 parallel link leg
  a->x[3] = a->x[4] = a->x[2] + L3 * std::sin(w2);
  a->z[3] = a->z[4] = a->z[2] + L23 + L3 * std::cos(w2);  //	 parallel link leg

  a->y[1] = a->z[1] * std::sin(a->d[0]);
  a->y[2] = a->z[2] * std::sin(a->d[0]);
  a->y[3] = (a->z[2] + L3) * std::sin(a->d[0]);
}

// tracking from degree to xyz
void Kine::trk_kine()
{
  xv_kine_[0].hip_r = serv_->xv_ref_.d[LEG_ROLL_R];
  xv_kine_[0].leg = 0.f;
  xv_kine_[0].knee = serv_->xv_ref_.d[KNEE_R2];
  xv_kine_[0].foot_p = serv_->xv_ref_.d[KNEE_R1];
  xv_kine_[0].foot_r = serv_->xv_ref_.d[FOOT_ROLL_R];
  xv_kine_[0].yaw = serv_->xv_ref_.d[LEG_YAW_R];

  fwd_kine_fun(&xv_kine_[0].hip_r, &xv_kine_[0].x);

  xv_kine_[1].hip_r = serv_->xv_ref_.d[LEG_ROLL_L];
  xv_kine_[1].leg = 0.f;
  xv_kine_[1].knee = serv_->xv_ref_.d[KNEE_L2];
  xv_kine_[1].foot_p = serv_->xv_ref_.d[KNEE_L1];
  xv_kine_[1].foot_r = serv_->xv_ref_.d[FOOT_ROLL_L];
  xv_kine_[1].yaw = serv_->xv_ref_.d[LEG_YAW_L];

  fwd_kine_fun(&xv_kine_[1].hip_r, &xv_kine_[1].x);

  calc_mv_->xv_data_x_r_.time = calc_mv_->xv_data_y_r_.time = calc_mv_->xv_data_y_r2_.time =
      calc_mv_->xv_data_z_r_.time = calc_mv_->xv_data_x_l_.time = calc_mv_->xv_data_y_l_.time =
          calc_mv_->xv_data_y_l2_.time = calc_mv_->xv_data_z_l_.time = EPS_TIME;

  calc_mv_->xv_data_x_r_.pos = xv_kine_[0].x;
  calc_mv_->xv_data_y_r_.pos = 0.f;
  calc_mv_->xv_data_y_r2_.pos = xv_kine_[0].y;
  calc_mv_->xv_data_z_r_.pos = xv_kine_[0].z;
  calc_mv_->xv_data_x_l_.pos = xv_kine_[1].x;
  calc_mv_->xv_data_y_l_.pos = 0.f;
  calc_mv_->xv_data_y_l2_.pos = xv_kine_[1].y;
  calc_mv_->xv_data_z_l_.pos = xv_kine_[1].z;
}
}  // namespace hr46
