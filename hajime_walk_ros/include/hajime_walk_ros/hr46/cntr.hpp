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

#include <map>
#include <memory>

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
#include "sq_start.hpp"
#include "sq_straight.hpp"
#include "sq_walk.hpp"

namespace hr46
{

#define PARAM_TABLE_OFFSET 26
char ParamTable[53] = {
  'z', 'y', 'x', 'w', 'v', 'u', 't', 's', 'r', 'q', 'p', 'o', 'n',
  'm', 'l', 'k', 'j', 'i', 'h', 'g', 'f', 'e', 'd', 'c', 'b', 'a',  // -26 - -1
  '0',                                                              // 0
  '1', '2', '3', '4', '5', '6', '7', '8', '9', 'J', 'K', 'L', 'M',
  'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z'  // 1 - 26
};

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
  SqStartSharedPtr sq_start_;
  SqStraightSharedPtr sq_straight_;
  SqWalkSharedPtr sq_walk_;

  short flag_face_control_;

public:
  Cntr();
  void cntr();
  void setMotionPath(std::string& motion_directory);
  void setEEPROM(std::map<std::string, double>& param, st_xp_acc& xp_acc, st_xp_gyro& xp_gyro, st_flag_gyro& flag_gyro,
                 st_xp_mv_straight& xp_mv_straight, st_xp_mv_ready& xp_mv_ready, st_xp_mv_walk& xp_walk,
                 st_xp_dlim_wait& xp_dlim_wait_x, st_xp_dlim_wait& xp_dlim_wait_y, st_xp_dlim_wait& xp_dlim_wait_theta,
                 st_xp_dlim_wait& xp_dlim_wait_pitch);
  void setCommand(char cmd, int para1, int para2, int para3, int para4, int para5);
  std::vector<double> getJoints();
  void setImuData(st_xv_acc& xv_acc, st_xv_gyro& xv_gyro);
};

using CntrUniquePtr = std::unique_ptr<Cntr>;
}  // namespace hr46

#endif  // CNTR_HPP_
