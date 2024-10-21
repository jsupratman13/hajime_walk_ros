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

#ifndef CNTR_HPP_
#define CNTR_HPP_

#include "acc.hpp"
#include "calc_deg.hpp"
#include "calc_mv.hpp"
#include "gyro.hpp"
#include "joy.hpp"
#include "kine.hpp"
#include "motion.hpp"
#include "mvtbl.hpp"
#include "serv.hpp"
#include "sq_motion.hpp"
#include "sq_ready.hpp"
#include "sq_straight.hpp"
#include "sq_walk.hpp"

namespace hr46
{
class Cntr
{
private:
  AccSharedPtr acc_;
  CalcDegSharedPtr calc_deg_;
  CalcMvSharedPtr calc_mv_;
  GyroSharedPtr gyro_;
  JoySharedPtr joy_;
  KineSharedPtr kine_;
  MotionSharedPtr motion_;
  MvTblSharedPtr mv_tbl_;
  ServSharedPtr serv_;
  SqMotionSharedPtr sq_motion_;
  SqReadySharedPtr sq_ready_;
  SqStraightSharedPtr sq_straight_;
  SqWalkSharedPtr sq_walk_;

  short flag_face_control_;

public:
  Cntr();
  void cntr();
};
}  // namespace hr46

#endif  // CNTR_HPP_
