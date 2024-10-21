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

// kinetics calculation
#ifndef KINE_HPP_
#define KINE_HPP_

#include <array>
#include <memory>

#include "calc_mv.hpp"
#include "serv.hpp"

namespace hr46
{

struct st_xv_k
{
  float x[5];  // [mm]
  float y[5];  // [mm]
  float z[5];  // [mm]
  float d[5];  // [deg]
};

// inverse kinetics data
struct st_xv_kine
{
  float x;       // foot x [mm]
  float y;       // foot y [mm]
  float z;       // foot z [mm]
  float yaw;     // hip yaw [deg]
  float hip_r;   // hip roll [deg]
  float leg;     // hip pitch [deg]
  float knee;    // knee pitch [deg]
  float foot_p;  // foot pitch [deg]
  float foot_r;  // foot roll [deg]
};

struct st_xv_posture
{
  float pitch;  // [deg]
  float roll;   // [deg]
  float yaw;    // [deg]
  float roll2;  // [deg]
};

class Kine
{
private:
  st_xv_k xv_k_;
  st_xv_k xv_k2_;

public:
  std::array<st_xv_kine, 2> xv_kine_;
  // st_xv_kine xv_kine_[2];
  st_xv_posture xv_posture_;

  Kine();
  void kine();
  void kine_fun(float* u, float* y);
  void cal_inv_kine(st_xv_k* a);
  void fwd_kine_fun(float* d, float* x);
  void cal_fwd_kine(st_xv_k* a);
  void trk_kine();

  ServSharedPtr serv_;
  CalcMvSharedPtr calc_mv_;
};

using KineSharedPtr = std::shared_ptr<Kine>;
}  // namespace hr46

#endif  // KINE_HPP_
