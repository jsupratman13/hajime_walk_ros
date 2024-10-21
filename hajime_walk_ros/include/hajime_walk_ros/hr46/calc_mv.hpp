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

// calculate trajectory
#ifndef CALC_MV_HPP_
#define CALC_MV_HPP_

#include <array>
#include <memory>

#include "kine.hpp"
#include "motion.hpp"
#include "mvtbl.hpp"
#include "serv.hpp"
#include "sq_straight.hpp"
#include "sq_walk.hpp"
#include "var.hpp"

namespace hr46
{

#define MVDATA_NUM (12)  // number of trajectory tables
#define EPS (0.001)      // epsilon : nearly equals zero

struct st_xv_mv
{
  long count;  // number of walk steps [count]
};

// trajectory calculation data
struct st_xv_mvdata
{
  float t;        // [bit]
  float mv_tbl;   // [bit]
  float dt;       // [bit]
  float start;    // [deg]or[mm]
  float amp;      // [deg]or[mm]
  float out_old;  // [deg]or[mm]
  float pos_old;  // [deg]or[mm]
};

// trajectory input data from motion sequence
struct st_xv_data
{
  float time;          // [mm] or [deg]
  float pos;           // [sec]
  long mv_tbl_select;  // [bit]
};

// odometry data
struct st_xv_odometry
{
  float moveX;         // X [mm]
  float moveY;         // Y [mm]
  float rotZ;          // THETA [deg]
  float x[2];          // foot point x
  float x_old[2];      // foot point x old data
  float y[2];          // foot point y
  float y_old[2];      // foot point y old data
  float theta[2];      // leg yaw theta
  float theta_old[2];  // leg yaw theta old data
};

class CalcMv
{
public:
  st_xv_mv xv_mv_;
  std::array<st_xv_mvdata, SERV_NUM> xv_mvdata_d_;
  std::array<st_xv_mvdata, MVDATA_NUM> xv_mvdata_;
  std::array<st_xv_data, SERV_NUM> xv_data_d_;
  st_xv_data xv_data_x_r_;
  st_xv_data xv_data_y_r_;
  st_xv_data xv_data_y_r2_;
  st_xv_data xv_data_z_r_;
  st_xv_data xv_data_x_l_;
  st_xv_data xv_data_y_l_;
  st_xv_data xv_data_y_l2_;
  st_xv_data xv_data_z_l_;
  st_xv_data xv_data_pitch_;
  st_xv_data xv_data_roll2_;
  st_xv_odometry xv_odometry_;
  float odometry_correct_para_x_;
  float odometry_correct_para_y_;

  void calc_mv_init(st_xp_mv_straight xp_mv_straight);
  void calc_mv();
  float calc_mvdata(st_xv_mvdata*, st_xv_data*);
  void chg_mvtbl(st_xv_mvdata*, st_xv_data*);

  KineSharedPtr kine_;
  MotionSharedPtr motion_;
  MvTblSharedPtr mv_tbl_;
  ServSharedPtr serv_;
  SqWalkSharedPtr sq_walk_;
};

using CalcMvSharedPtr = std::shared_ptr<CalcMv>;
}  // namespace hr46

#endif  // CALC_MV_HPP_
