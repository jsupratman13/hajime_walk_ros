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

// straight sequence
#ifndef SQ_STRAIGHT_HPP_
#define SQ_STRAIGHT_HPP_

#include <memory>

#include "calc_mv.hpp"
#include "kine.hpp"
#include "serv.hpp"
#include "gyro.hpp"
#include "motion.hpp"

namespace hr46
{
// forward declaration
class CalcMv;
class Gyro;
class Kine;
class Motion;
class Serv;

enum
{
  // SQ_STRAIGHT_INIT	= 	0,
  SQ_STRAIGHT = 1,
  SQ_STRAIGHT2 = 2,
  SQ_STRAIGHT3 = 3,
  SQ_STRAIGHT4 = 4,
  SQ_STRAIGHT5 = 5,
  SQ_STRAIGHT_END = 6
};

struct st_xp_mv_straight
{
  float time;          // move time [sec]
  float z3;            // hight of hip joint [mm]
  float arm_sh_pitch;  // shoulder pitch [deg]
  float arm_sh_roll;   // shoulder roll [deg]
  float arm_el_yaw;    // elbow yaw [deg]
  float arm_el_pitch;  // elbow pitch [deg]
};

class SqStraight
{
public:
  st_xp_mv_straight xp_mv_straight_;
  short flag_md_straight_end_;
  short mode_sq_straight_;
  float xv_mv_straight_time_;
  float mode_sq_time_;

  SqStraight();
  void sq_straight_init();
  int sq_straight(short flag_face_control);

  // CalcMvSharedPtr calc_mv_;
  std::shared_ptr<CalcMv> calc_mv_;
  // GyroSharedPtr gyro_;
  std::shared_ptr<Gyro> gyro_;
  // KineSharedPtr kine_;
  std::shared_ptr<Kine> kine_;
  // MotionSharedPtr motion_;
  std::shared_ptr<Motion> motion_;
  // ServSharedPtr serv_;
  std::shared_ptr<Serv> serv_;
};

using SqStraightSharedPtr = std::shared_ptr<SqStraight>;
}  // namespace hr46

#endif  // SQ_STRAIGHT_HPP_
