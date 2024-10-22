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

#include <algorithm>
#include <array>
#include <iostream>

#include "hr46/joy.hpp"
#include "hr46/mvtbl.hpp"
#include "hr46/var.hpp"

namespace hr46
{

#define MAX_ONE_STEP_X 200.0f  //[mm]
#define MAX_ONE_STEP_Y 200.0f  //[mm]
#define MAX_ONE_STEP_Z 80.0f   //[mm]
#define MAX_ONE_STEP_TH 50.0f  //[deg]

// 16 hex to string
void writeHexNumber(unsigned char* buffer, int nr, int len)
{
  unsigned char hexTable[17] = "0123456789abcdef";
  int digit;
  int i;
  buffer += len;
  for (i = 0; i < len; i++)
  {
    buffer--;
    digit = nr & 0x0f;
    *buffer = hexTable[digit];
    nr = nr >> 4;
  }
}

float max180(float ang)
{
  while (ang > 180.0f)
    ang -= 360.0f;
  while (ang < -180.0f)
    ang += 360.0f;
  return ang;
}

short ascii2bin(unsigned char c)
{
  short c1;
  c1 = c;

  if (c >= '0' && c <= '9')
    return static_cast<short>(c1 - '0');
  else if (c >= 'A' && c <= 'Z')
    return static_cast<short>(c1 - 'A' + 1);
  else if (c >= 'a' && c <= 'z')
    return static_cast<short>(-(c1 - 'a' + 1));

  return 0;
}

void Joy::set_xv_comm(st_xv_comm* xv, unsigned char cmd, unsigned char para1, unsigned char para2, unsigned char para3,
                      unsigned char para4, unsigned char para5)
{
  xv->cmd = cmd;
  xv->para1 = para1;
  xv->para2 = para2;
  xv->para3 = para3;
  xv->para4 = para4;
  xv->para5 = para5;
}

// x[mm], y[mm], z[mm], th[deg]
// void Joy::accurate_walk_command(char* accurate_one_step_mode, sq_flag_T* sq_flag, st_xv_mv* xv_mv,
//                                 st_sq_walk_->xv_mv_walk_* sq_walk_->xv_mv_walk_, short steps, float x, float y, float
//                                 z, float th, float time)
// {
//   sq_walk_->accurate_one_step_mode_ = 1;
//   if (!motion_->sq_flag_.walk)
//     calc_mv_->xv_mv_.count = 0;

//   xv_joy_.walk_num =
//       steps != 0 ? std::clamp(static_cast<int>(steps), 2, 100) + static_cast<short>(calc_mv_->xv_mv_.count) : 10000;

//   motion_->sq_flag_.walk = ON;
//   is_walk_change_ = 1;

//   sq_walk_->xv_mv_walk_->accurate_step_x = std::clamp(x, -MAX_ONE_STEP_X, MAX_ONE_STEP_X);
//   sq_walk_->xv_mv_walk_->accurate_step_y = -std::clamp(y, -MAX_ONE_STEP_Y, MAX_ONE_STEP_Y);
//   sq_walk_->xv_mv_walk_->accurate_step_z = std::clamp(z, 0.0f, MAX_ONE_STEP_Z);
//   sq_walk_->xv_mv_walk_->accurate_step_th = -std::clamp(th, -MAX_ONE_STEP_TH, MAX_ONE_STEP_TH);
//   sq_walk_->xv_mv_walk_->accurate_step_time = time;

//   if (std::fabs(xv_joy_.walk_time - sq_walk_->xv_mv_walk_->time_old) > EPS_DATA)
//   {
//     xv_joy_.walk_zmp = hr46::zmp_fun(xv_joy_.walk_time, sq_walk_->xp_mv_walk_.y_balance);
//     xv_joy_.walk_time_dutyfactor = std::clamp(0.30f / xv_joy_.walk_time, 0.05f, 1.0f);
//     sq_walk_->xv_mv_walk_->time_old = xv_joy_.walk_time;
//   }
// }

void Joy::convert_bin(st_xv_comm_bin* xv_bin, st_xv_comm* xv)
{
  xv_bin->para1 = ascii2bin(xv->para1);
  xv_bin->para2 = ascii2bin(xv->para2);
  xv_bin->para3 = ascii2bin(xv->para3);
  xv_bin->para4 = ascii2bin(xv->para4);
  xv_bin->para5 = ascii2bin(xv->para5);
}

int Joy::scif1_tx_fun()
{
  int len = 0;
  for (const auto& c : sfmt_)
  {
    if (c == EOF_CODE || c == EOF_CODE3)
    {
      break;
    }
    len++;
  }

  std::fill(sfmt_.begin(), sfmt_.end(), EOF_CODE);
  return len;
}

void Joy::joy_init()
{
  count_joy_ = 0;
  xv_comm_ = { '0', '0', '0', '0', '0', '0' };
  xv_comm_bin_ = { 0, 0, 0, 0, 0, 0 };
  xv_comm_response_ = { EOF_CODE, '0', '0', '0', '0', '0', '0', '0', '0' };
  joy_status_ = { '0', '0', '0', '0', '0', '0' };

  xv_joy_.walk_num = 0;
  xv_joy_.walk_time = 0.0f;
  xv_joy_.walk_x_percent = 0.0f;
  xv_joy_.walk_y_percent = 0.0f;
  xv_joy_.walk_theta_percent = 0.0f;
  xv_joy_.walk_zmp = sq_walk_->xv_mv_walk_.zmp;
  xv_joy_.walk_time_dutyfactor = sq_walk_->xv_mv_walk_.time_dutyfactor;
  xv_joy_.walk_step_len_offset = 0.0f;

  is_walk_change_ = 0;
  old_yaw_ = 0.0f;
}

void Joy::joy(short& flag_face_control)
{
  unsigned char* tmpCharPtr;
  int w;
  float w1, w2;
  short x_s[SERV_NUM];

  st_xv_comm xv_comm = xv_comm_;
  st_xv_comm_bin xv_comm_bin = xv_comm_bin_;
  xv_comm_ = { '0', '0', '0', '0', '0', '0' };
  xv_comm_bin_ = { 0, 0, 0, 0, 0, 0 };

  // command receive check
  if (xv_comm.cmd != '0')
  {
    xv_comm_response_[0] = xv_comm.cmd;
    xv_comm_response_[1] = xv_comm.para1;
    xv_comm_response_[2] = xv_comm.para2;
    xv_comm_response_[3] = xv_comm.para3;
    xv_comm_response_[4] = xv_comm.para4;
    xv_comm_response_[5] = xv_comm.para5;
  }

  switch (xv_comm.cmd)
  {
    case 'P':  // servo torque off
      switch (xv_comm_bin.para1)
      {
        case 0:
          std::cout << "Servo off" << std::endl;
          // TODO: flag_servo_off = ON;
          acc_->flag_ukemi_ = OFF;
          acc_->flag_motion_accept_ = ON;
          break;
        case 1:
          std::cout << "Servo on" << std::endl;
          motion_->sq_flag_.start = ON;
          break;
        case 2:
          acc_->flag_ukemi_ = ON;
          break;
        case 3:
          acc_->flag_ukemi_ = OFF;
          acc_->flag_motion_accept_ = ON;
          motion_->sq_flag_.straight = ON;
          motion_->sq_flag_.start = ON;
          break;
        case 4:
          acc_->flag_ukemi_ = OFF;
          acc_->flag_motion_accept_ = ON;
          break;
        default:
          break;
      }
      break;

      /*
       * return data format
       * 0:'R', 1:status, 2:'0', 3:fall, 4:'0', 5-8:odometry_x, 9-12:odometry_y, 13-16:odometry_the,
       * 17-20:servo0 , 21-24:servo1 , 25-28:servo2 , 29-32:servo3 , 33-36:servo4 , 37-40:servo5 ,
       * 41-44:servo6 , 45-48:servo7 , 49-52:servo8 , 53-56:servo9 , 57-60:servo10, 61-64:servo11,
       * 65-68:servo12, 69-72:gyro   , 73-76:pan    , 77-80:tilt
       */
    case 'R':  // status of robot
      joy_status_.cmd = 'R';

      // robot is moving
      if (motion_->flag_moving_ == STATE_WALKING)
        joy_status_.para1 = '1';  // walking
      else if (motion_->flag_moving_ == STATE_MOTION)
        joy_status_.para1 = '2';  // acting special action
      else if (motion_->flag_moving_ == STATE_MOVING)
        joy_status_.para1 = '3';  // moving
      else
        joy_status_.para1 = '0';  // stop

      // fall down flag
      joy_status_.para2 = '0';
      if (gyro_->flag_gyro_.fall == 1)
        joy_status_.para3 = '1';  // front
      else if (gyro_->flag_gyro_.fall == 2)
        joy_status_.para3 = '2';  // back
      else if (gyro_->flag_gyro_.fall == 3)
        joy_status_.para3 = '3';  // right
      else if (gyro_->flag_gyro_.fall == 4)
        joy_status_.para3 = '4';  // left
      else
        joy_status_.para3 = '0';  // not fallen

      joy_status_.para4 = '0';

      xv_comm_response_[0] = EOF_CODE;
      // legs and waist joint angle
      for (w = 0; w < 6; w++)
      {
        x_s[w] = static_cast<short>(serv_->xv_sv_[w].d / 100);
      }
      for (w = 6; w < 12; w++)
      {
        x_s[w] = static_cast<short>(serv_->xv_sv_[w + 3].d / 100);
      }
      x_s[13] = 0;
      // head joint angle
      x_s[HEAD_YAW] = static_cast<short>(serv_->xv_sv_[HEAD_YAW].d / 100);
      x_s[HEAD_PITCH] = static_cast<short>(serv_->xv_sv_[HEAD_PITCH].d / 100);
      // odometry
      x_s[14] = static_cast<short>(calc_mv_->xv_odometry_.moveX);
      x_s[15] = static_cast<short>(calc_mv_->xv_odometry_.moveY);
      x_s[16] = static_cast<short>(calc_mv_->xv_odometry_.rotZ);

      // write joy-status to output
      sfmt_[0] = joy_status_.cmd;
      sfmt_[1] = joy_status_.para1;
      sfmt_[2] = joy_status_.para2;
      sfmt_[3] = joy_status_.para3;
      sfmt_[4] = joy_status_.para4;
      // sfmt_[5] = joy_status_.para5;
      // tmpCharPtr = sfmt_ + 6;
      // TODO: tmpCharPtr = sfmt_ + 5;

      // write odometry to output
      writeHexNumber(tmpCharPtr, static_cast<int>(calc_mv_->xv_odometry_.moveX), 4);
      tmpCharPtr += 4;
      writeHexNumber(tmpCharPtr, static_cast<int>(calc_mv_->xv_odometry_.moveY), 4);
      tmpCharPtr += 4;
      writeHexNumber(tmpCharPtr, static_cast<int>(calc_mv_->xv_odometry_.rotZ * 100), 4);
      tmpCharPtr += 4;

      for (int i = 0; i < 13; i++)
      {
        writeHexNumber(tmpCharPtr, x_s[i], 4);
        tmpCharPtr += 4;
      }

      // write gyro yaw as 14th servo
      writeHexNumber(tmpCharPtr, static_cast<int>(max180(gyro_->xv_gyro_.gyro_yaw2 - old_yaw_) * 100), 4);
      tmpCharPtr += 4;

      writeHexNumber(tmpCharPtr, x_s[HEAD_YAW], 4);
      tmpCharPtr += 4;
      writeHexNumber(tmpCharPtr, x_s[HEAD_PITCH], 4);
      tmpCharPtr += 4;

      writeHexNumber(tmpCharPtr, static_cast<int>(gyro_->xv_gyro_.quaternion[0] * 32767), 4);  // w
      tmpCharPtr += 4;
      writeHexNumber(tmpCharPtr, static_cast<int>(gyro_->xv_gyro_.quaternion[1] * 32767), 4);  // x
      tmpCharPtr += 4;
      writeHexNumber(tmpCharPtr, static_cast<int>(gyro_->xv_gyro_.quaternion[2] * 32767), 4);  // y
      tmpCharPtr += 4;
      writeHexNumber(tmpCharPtr, static_cast<int>(gyro_->xv_gyro_.quaternion[3] * 32767), 4);  // z

      *tmpCharPtr = EOF_CODE3;

      // writing output data to serial port
      scif1_tx_fun();

      // reset values
      calc_mv_->xv_odometry_.moveX = 0.0f;
      calc_mv_->xv_odometry_.moveY = 0.0f;
      calc_mv_->xv_odometry_.rotZ = 0.0f;

      // reset gyro_yaw
      old_yaw_ = gyro_->xv_gyro_.gyro_yaw2;
      break;

    case 'S':
      xv_joy_.walk_num = 0;
      motion_->sq_flag_.ready = ON;
      break;

    case 'A':  // walk all direction
      sq_walk_->accurate_one_step_mode_ = 0;
      // number of steps
      if (!motion_->sq_flag_.walk)
      {
        calc_mv_->xv_mv_.count = 0;  // set to 0 if not walking
      }

      if (xv_comm_bin.para1 != 0)
      {
        xv_joy_.walk_num = static_cast<short>(std::clamp(static_cast<int>(xv_comm_bin.para1), 2, 100) +
                                              calc_mv_->xv_mv_.count);  // last steps [num/bit]
      }
      else
      {
        xv_joy_.walk_num = 10000;  // 0 means non stop [num/bit]
      }

      motion_->sq_flag_.walk = ON;
      is_walk_change_ = 1;  // flag to check if walk command is changed

      // hip yaw angle [deg/bit]
      w1 = std::clamp(static_cast<float>(xv_comm_bin.para2), -sq_walk_->xp_mv_walk_.theta, sq_walk_->xp_mv_walk_.theta);
      xv_joy_.walk_theta_percent = -w1 / sq_walk_->xp_mv_walk_.theta;  //	- : +Y=LEFT turn, -Y=RIGHT turn

      // stride X [5mm/bit]
      w1 = sq_walk_->xp_mv_walk_.x_fwd_swg - sq_walk_->xp_mv_walk_.x_fwd_spt;
      w2 = std::clamp(xv_joy_.walk_step_len_offset + (static_cast<float>(xv_comm_bin.para3) * 5.0f), -w1, w1);
      xv_joy_.walk_x_percent = w2 / w1;

      // time of one step
      if (xv_comm_bin.para4 > 0)
      {
        // offset + 0.10 sec [0.02sec/bit]
        xv_joy_.walk_time = static_cast<float>(xv_comm_bin.para4) / 50.0f + 0.10f;
      }
      else
      {
        // <= 0 means default time
        xv_joy_.walk_time = sq_walk_->xp_mv_walk_.time;
      }

      if (std::fabs(xv_joy_.walk_time - sq_walk_->xv_mv_walk_.time_old) > EPS_DATA)
      {
        // time changed
        xv_joy_.walk_zmp =
            hr46::zmp_fun(xv_joy_.walk_time, sq_walk_->xp_mv_walk_.y_balance, sq_walk_->xp_mv_walk_.h_cog);
        xv_joy_.walk_time_dutyfactor = std::clamp(0.30f / xv_joy_.walk_time, 0.05f, 1.0f);
        // leg idle time 0.30[s]
        sq_walk_->xv_mv_walk_.time_old = xv_joy_.walk_time;
      }

      // stride Y [2.5mm/bit]
      if (xv_comm_bin.para5 == 0)
      {
        xv_joy_.walk_y_percent = 0.0f;
      }
      else
      {
        w1 = sq_walk_->xp_mv_walk_.y_swg - sq_walk_->xp_mv_walk_.y_spt;
        w2 = std::clamp(static_cast<float>(xv_comm_bin.para5) * 2.5f, -w1, w1);
        xv_joy_.walk_y_percent = -w2 / w1;  // - : +Y=LEFT side, -Y=RIGHT side
      }

      break;

    case 'H':  // head yaw
      w = xv_comm_bin.para1 * 100 + xv_comm_bin.para2 * 10 + xv_comm_bin.para3;

      if (std::fabs(w) < 150.0f && xv_comm_bin.para4 / 10.0f)
      {
        calc_mv_->xv_data_d_[HEAD_YAW].time = static_cast<float>(xv_comm_bin.para4) / 10.0f;
        // check
        {  // match calculated value of the neck angle with the actual value
          float min_period = std::fabs(w - calc_mv_->xv_mvdata_d_[HEAD_YAW].out_old) / 60.0f * 0.22f;
          if (min_period < 0.1f)
          {
            min_period = 0.1f;
          }
          if (calc_mv_->xv_data_d_[HEAD_YAW].time < min_period)
          {
            calc_mv_->xv_data_d_[HEAD_YAW].time = min_period;
          }
        }
        calc_mv_->xv_data_d_[HEAD_YAW].pos = static_cast<float>(w);
      }

      if (xv_comm_bin.para1 == 0 && xv_comm_bin.para2 == 0 && xv_comm_bin.para3 == 0 && xv_comm_bin.para4 == 0)
      {
        flag_face_control = OFF;
      }
      else
      {
        flag_face_control = ON;
      }
      break;

    case 'h':  // head pitch
      w = xv_comm_bin.para1 * 100 + xv_comm_bin.para2 * 10 + xv_comm_bin.para3;

      if (std::fabs(w) < 150.0f && xv_comm_bin.para4 / 10.0f)
      {
        calc_mv_->xv_data_d_[HEAD_PITCH].time = static_cast<float>(xv_comm_bin.para4) / 10.0f;
        calc_mv_->xv_data_d_[HEAD_PITCH].pos = static_cast<float>(w);
      }

      if (xv_comm_bin.para1 == 0 && xv_comm_bin.para2 == 0 && xv_comm_bin.para3 == 0 && xv_comm_bin.para4 == 0)
      {
        flag_face_control = OFF;
      }
      else
      {
        flag_face_control = ON;
      }
      break;

    case 'N':  // neck control
      if (xv_comm_bin.para5 > 0)
      {
        calc_mv_->xv_data_d_[HEAD_YAW].time = static_cast<float>(xv_comm_bin.para5) / 10.0f;
        // check
        {  // match calculated value of the neck angle with the actual value
          float w = static_cast<float>(xv_comm_bin.para1 * 10 + xv_comm_bin.para2);
          float min_period = std::fabs(w - calc_mv_->xv_mvdata_d_[HEAD_YAW].out_old) / 60.0f * 0.22f;
          if (min_period < 0.1f)
          {
            min_period = 0.1f;
          }
          if (calc_mv_->xv_data_d_[HEAD_YAW].time < min_period)
          {
            calc_mv_->xv_data_d_[HEAD_YAW].time = min_period;
          }
        }
        calc_mv_->xv_data_d_[HEAD_PITCH].time = static_cast<float>(xv_comm_bin.para5) / 10.0f;
      }
      else
      {
        calc_mv_->xv_data_d_[HEAD_YAW].time = 0.1f;
        calc_mv_->xv_data_d_[HEAD_PITCH].time = 0.1f;
      }
      calc_mv_->xv_data_d_[HEAD_YAW].pos = static_cast<float>(xv_comm_bin.para1 * 10 + xv_comm_bin.para2);
      calc_mv_->xv_data_d_[HEAD_PITCH].pos = static_cast<float>(xv_comm_bin.para3 * 10 + xv_comm_bin.para4);
      break;

    case 'M':  // motion (special action)
      // TODO: if (((acc_->flag_motion_accept_ == ON) || (flag_gyro.fall == 0)) && (flag_servo_on))
      if (acc_->flag_motion_accept_ == ON || gyro_->flag_gyro_.fall == 0)
      {
        if (xv_comm_bin.para1 >= 0 && xv_comm_bin.para2 >= 0 && xv_comm_bin.para3 >= 0)
        {
          w = xv_comm_bin.para1 * 100 + xv_comm_bin.para2 * 10 + xv_comm_bin.para3;

          if (w >= 1 && w <= 100)
          {
            sq_motion_->flag_motion_select_ = w;
            motion_->sq_flag_.motion = ON;
          }
        }

        // number of repeat
        if (xv_comm_bin.para4 > 0)
        {
          sq_motion_->flag_motion_repeat_ = ON;
          sq_motion_->xv_motion_repeat_num_ =
              std::clamp(xv_comm_bin.para4, static_cast<short>(1), static_cast<short>(100));
        }
        else
        {
          sq_motion_->flag_motion_repeat_ = OFF;
        }
        break;
      }
      break;

    case 'C':  // cancel
      if (xv_comm_bin.para1 == 0)
      {
        if (motion_->mode_motion_ != MOTION_MOTION)  // only stop if current motion is not special action
        {
          calc_mv_->xv_mv_.count = 10001.0f;  // max. step number is 10000 (set by '0' as desired step number
          motion_->sq_flag_ = { 0, 0, 0, 0, 0 };
          xv_joy_.walk_x_percent = 0.0f;
          xv_joy_.walk_y_percent = 0.0f;
          xv_joy_.walk_theta_percent = 0.0f;
          // flag_sq_motion_cancel = ON;
        }
        else  // end of repeat motion
        {
          motion_->mode_motion_ = 1000;
        }
      }
      else if (xv_comm_bin.para1 == 1)
      {
        // end of repeat motion
        motion_->mode_motion_ = 1000;
      }
      break;

    case 'T':  // standing upright
      motion_->sq_flag_.straight = ON;
      break;

    case 'J':  // gyro on/off
      switch (xv_comm_bin.para1)
      {
        case 0:
          gyro_->flag_gyro_.vib_auto = ON;
          gyro_->flag_gyro_.vib_manu = OFF;
          break;
        case 1:
          gyro_->flag_gyro_.vib_auto = OFF;
          gyro_->flag_gyro_.vib_manu = ON;
          break;
        case 2:
          gyro_->flag_gyro_.vib_auto = OFF;
          gyro_->flag_gyro_.vib_manu = OFF;
          break;
        case 3:
          gyro_->flag_auto_gyro_offset_ = OFF;
          break;
        case 4:
          gyro_->flag_auto_gyro_offset_ = OFF;
          break;
        default:
          break;
      }
      break;

    case 'j':  // gyro reset
      if (xv_comm_bin.para1 == 1)
      {
        gyro_->flag_gyro_.zero = ON;
        gyro_->gyro_fun();
      }
      break;

    case 'w':  // change ready.z3
      switch (xv_comm_bin.para1)
      {
        case 0:
          sq_ready_->xp_mv_ready_.z3 = sq_walk_->xp_mv_walk_.h_cog;
          break;
        case 1:
          sq_ready_->xp_mv_ready_.z3 -= 10.0f;
          break;
        case 2:
          sq_ready_->xp_mv_ready_.z3 -= 20.0f;
          break;
        default:
          break;
      }
      motion_->sq_flag_.straight = ON;
      break;

    case 'O':
      calc_mv_->xv_mv_.count = 0;
      // TODO
      // accurate_walk_command(accurate_one_step_mode, sq_flag, xv_mv, sq_walk_->xv_mv_walk_, 2, xv_comm_bin.para1 * 10,
      //                       xv_comm_bin.para2 * 10, sq_walk_->xp_mv_walk_.accurate_step_z, xv_comm_bin.para3 * 2,
      //                       xv_joy_.walk_time);
      break;

    case 'U':
      // TODO
      // accurate_walk_command(accurate_one_step_mode, sq_flag, xv_mv, sq_walk_->xv_mv_walk_, 2, xv_comm_bin.para1 * 10,
      //                       xv_comm_bin.para2 * 10, sq_walk_->xp_mv_walk_.accurate_step_z, xv_comm_bin.para3 * 2,
      //                       xv_joy_.walk_time);
      break;

    case 'V':
      // if ((acc_->flag_motion_accept_ == ON) || (acc_->flag_acc_.fall == 0) && (flag_servo_on))
      if ((acc_->flag_motion_accept_ == ON || acc_->flag_acc_.fall == 0))
      {
        if (xv_comm_bin.para1 >= 0 && xv_comm_bin.para2 >= 0 && xv_comm_bin.para3 >= 0)
        {
          w = xv_comm_bin.para1 * 100 + xv_comm_bin.para2 * 10 + xv_comm_bin.para3;

          if (w >= 1 && w <= 100)
          {
            sq_motion_->flag_motion_select_ = w;
            motion_->sq_flag_.motion = ON;
            sq_motion_->flag_variable_motion_ = ON;
          }
        }
        sq_motion_->flag_motion_repeat_ = OFF;
        sq_motion_->xv_motion_repeat_num_ = 1;
        sq_motion_->variable_amount_ = xv_comm_bin.para4;
        break;
      }
      break;

    default:
      break;
  }

  if (xv_comm_response_[0] != EOF_CODE && xv_comm_response_[0] != '0')
  {
    sfmt_[0] = xv_comm_response_[0];
    sfmt_[1] = xv_comm_response_[1];
    sfmt_[2] = xv_comm_response_[2];
    sfmt_[3] = xv_comm_response_[3];
    sfmt_[4] = xv_comm_response_[4];
    sfmt_[5] = xv_comm_response_[5];
    sfmt_[6] = EOF_CODE3;

    xv_comm_response_[0] = EOF_CODE;
    scif1_tx_fun();
  }
}

void Joy::joy_read()
{
  sscanf(reinterpret_cast<char*>(rfmt_[5]), "%c%c%c%c%c%c", &xv_comm_.cmd, &xv_comm_.para1, &xv_comm_.para2,
         &xv_comm_.para3, &xv_comm_.para4, &xv_comm_.para5);

  // character to binary data
  xv_comm_bin_.para1 = ascii2bin(xv_comm_.para1);
  xv_comm_bin_.para2 = ascii2bin(xv_comm_.para2);
  xv_comm_bin_.para3 = ascii2bin(xv_comm_.para3);
  xv_comm_bin_.para4 = ascii2bin(xv_comm_.para4);
  xv_comm_bin_.para5 = ascii2bin(xv_comm_.para5);
  count_joy_++;
}

void Joy::copy_joy_parameter()
{
  sq_walk_->xv_mv_walk_.num = xv_joy_.walk_num;                          // number of steps
  sq_walk_->xv_mv_walk_.time = xv_joy_.walk_time;                        // walk period / 2
  sq_walk_->xv_mv_walk_.zmp = xv_joy_.walk_zmp;                          // Yzmp [mm]
  sq_walk_->xv_mv_walk_.time_dutyfactor = xv_joy_.walk_time_dutyfactor;  // duty factor
  sq_walk_->xv_mv_walk_.x_percent = xv_joy_.walk_x_percent;              // foot x (0-1)
  sq_walk_->xv_mv_walk_.y_percent = xv_joy_.walk_y_percent;              // foot y (0-1)
  sq_walk_->xv_mv_walk_.theta_percent = xv_joy_.walk_theta_percent;      // foot theta (0-1)
}

}  // namespace hr46
