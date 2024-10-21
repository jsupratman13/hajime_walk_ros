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
// servo control
#ifndef SERV_HPP_
#define SERV_HPP_

#include <array>
#include <memory>

#include "calc_mv.hpp"
#include "var.hpp"
#include "kine.hpp"
#include "func.hpp"

namespace hr46
{

struct st_xv_sv
{
  long d;           // [0.01deg]
  long deg_sw_out;  // [0.01deg]
  long deg_sv;      // [0.01deg]
  long deg_lim;     // [0.01deg]
  long pls;         // [bit]
  long pls_out;     // [bit]
};

struct st_xp_sv
{
  long deg_sign;        // [+/-]
  long deg_offset;      // [deg]
  long deg_lim_h;       // [deg]
  long deg_lim_l;       // [deg]
  long deg_lim_offset;  // [0.01deg]
  long deg2pls;         // [bit/(0.01deg)]
};

struct st_xv_ref
{
  float d[SERV_NUM];      // [deg]
  float d_ref[SERV_NUM];  // [deg]
};

struct st_sw
{
  short ref_d;
};

struct st_xv_pv
{
  short temp[SERV_NUM];     // [deg]
  float deg[SERV_NUM];      // [deg]
  short current[SERV_NUM];  // [mA]
};

class Serv
{
public:
  std::array<st_xv_sv, SERV_NUM> xv_sv_;
  std::array<st_xp_sv, SERV_NUM> xp_sv_;
  st_xv_ref xv_ref_;
  st_sw sw_;
  st_xv_pv xv_pv_;
  float xp_ref_d_lim_;

  Serv();
  void serv();
  void set_sw_ref_d(int n);
  short check_sw_ref_d();

  CalcMvSharedPtr calc_mv_;
  KineSharedPtr kine_;
};

using ServSharedPtr = std::shared_ptr<Serv>;
}  // namespace hr46

#endif  // SERV_HPP_
