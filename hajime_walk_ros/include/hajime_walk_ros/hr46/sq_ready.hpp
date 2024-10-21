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

// ready sequence
#ifndef SQ_READY_HPP_
#define SQ_READY_HPP_

#include <memory>

#include "kine.hpp"
#include "serv.hpp"
#include "sq_walk.hpp"
#include "gyro.hpp"
#include "motion.hpp"
#include "calc_mv.hpp"

namespace hr46
{
enum
{
  SQ_READY_INIT = 0,
  SQ_READY = 1,
  SQ_READY2 = 2,
  SQ_READY3 = 3,
  SQ_READY4 = 4,
  SQ_READY_END = 5
};

struct st_xp_mv_ready
{
  float time;          // move time [sec]
  float z3;            // hight of hip joint [mm]
  float arm_sh_pitch;  // shoulder pitch [deg]
  float arm_sh_roll;   // shoulder roll [deg]
  float arm_el_yaw;    // elbow yaw [deg]
  float arm_el_pitch;  // elbow pitch [deg]
  float pitch;         // pitch [deg]
};

class SqReady
{
public:
  st_xp_mv_ready xp_mv_ready_;
  // short flag_sq_ready;
  // short flag_md_ready_end;
  short mode_sq_ready_;
  short flag_ready_gyro_;
  short flag_md_ready_end_;
  float xv_mv_ready_time_;
  float mode_sq_time_;

  SqReady();
  void sq_ready_init(int slow_mode);
  int sq_ready(short flag_face_control);

  CalcMvSharedPtr calc_mv_;
  GyroSharedPtr gyro_;
  KineSharedPtr kine_;
  MotionSharedPtr motion_;
  ServSharedPtr serv_;
  SqWalkSharedPtr sq_walk_;
};

using SqReadySharedPtr = std::shared_ptr<SqReady>;
}  // namespace hr46

#endif  // SQ_READY_HPP_
