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

// command interface
#ifndef JOY_HPP_
#define JOY_HPP_

#include <memory>
#include <vector>
#include <array>

#include "acc.hpp"
#include "calc_mv.hpp"
#include "sq_motion.hpp"
#include "serv.hpp"
#include "gyro.hpp"
#include "motion.hpp"
#include "calc_mv.hpp"
#include "sq_ready.hpp"
#include "sq_walk.hpp"
#include "var.hpp"

namespace hr46
{
// forward declaration
class Acc;
class CalcMv;
class Gyro;
class Motion;
class Serv;
class SqMotion;
class SqReady;
class SqWalk;

#define EOF_CODE ('\0')
#define EOF_CODE2 ('#')
#define EOF_CODE3 (0x0a)

#define RFMT_SIZE (256)
#define SFMT_SIZE (256)

struct st_xv_comm
{
  unsigned char cmd;
  unsigned char para1;
  unsigned char para2;
  unsigned char para3;
  unsigned char para4;
  unsigned char para5;
};

struct st_xv_comm_bin
{
  short cmd;
  short para1;
  short para2;
  short para3;
  short para4;
  short para5;
};

struct st_joy_status
{
  unsigned char cmd;
  unsigned char para1;
  unsigned char para2;
  unsigned char para3;
  unsigned char para4;
  unsigned char para5;
};

struct st_xv_joy
{
  short walk_num;              // step
  float walk_time;             // sec
  float walk_x_percent;        // -1<x<1 [1]
  float walk_y_percent;        // -1<x<1 [1]
  float walk_theta_percent;    // -1<x<1 [1]
  float walk_zmp;              // mm
  float walk_time_dutyfactor;  // 0<x<1 [-]
  float walk_step_len_offset;  // mm
};

void writeHexNumber(unsigned char* buffer, int nr, int len);
float max180(float ang);

class Joy
{
public:
  unsigned short count_joy_;  // number of joy command
  st_xv_comm xv_comm_;
  st_xv_comm_bin xv_comm_bin_;
  st_joy_status joy_status_;
  st_xv_joy xv_joy_;
  std::vector<char> xv_comm_response_;
  int is_walk_change_;  // flag to check if walk command is changed
  char rfmt_[RFMT_SIZE];
  std::array<char, SFMT_SIZE> sfmt_;
  float old_yaw_;

  // set command
  void set_xv_comm(st_xv_comm* xv, unsigned char cmd, unsigned char para1, unsigned char para2, unsigned char para3,
                   unsigned char para4, unsigned char para5);

  // void accurate_walk_command(char* accurate_one_step_mode, sq_flag_T* sq_flag, st_xv_mv* xv_mv,
  //                            st_xv_mv_walk* xv_mv_walk, short steps, float x, float y, float z, float th, float
  //                            time);

  // convert to send command
  void convert_bin(st_xv_comm_bin* xv_bin, st_xv_comm* xv);

  int scif1_tx_fun();

  void joy_init();
  void joy(short& flag_face_control);
  st_xv_mv_walk copy_joy_parameter();
  void joy_read();

  // AccSharedPtr acc_;
  std::shared_ptr<Acc> acc_;
  // CalcMvSharedPtr calc_mv_;
  std::shared_ptr<CalcMv> calc_mv_;
  // GyroSharedPtr gyro_;
  std::shared_ptr<Gyro> gyro_;
  // MotionSharedPtr motion_;
  std::shared_ptr<Motion> motion_;
  // ServSharedPtr serv_;
  std::shared_ptr<Serv> serv_;
  // SqMotionSharedPtr sq_motion_;
  std::shared_ptr<SqMotion> sq_motion_;
  // SqReadySharedPtr sq_ready_;
  std::shared_ptr<SqReady> sq_ready_;
  // SqWalkSharedPtr sq_walk_;
  std::shared_ptr<SqWalk> sq_walk_;
};

using JoySharedPtr = std::shared_ptr<Joy>;

}  // namespace hr46

#endif  // JOY_HPP_
