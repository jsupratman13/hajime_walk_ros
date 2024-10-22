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

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <filesystem>

#include "hr46/sq_motion.hpp"
#include "hr46/var.hpp"

namespace hr46
{

int SqMotion::sq_motion(short flag_face_control)
{
  short i, j, n;
  short _flag_w;

  if (mode_sq_motion_ == SQ_MODE2_INIT)
  {
    current_flag_motion_select_ = flag_motion_select_;
  }
  n = current_flag_motion_select_;

  switch (mode_sq_motion_)
  {
    case SQ_MODE2_INIT:  // init
      count_motion_repeat_ = 0;
      flag_md_motion_end_ = OFF;
      flag_md_motion_hold_ = OFF;
      flag_sq_motion_cancel_ = OFF;
      flag_motion_gyro_ = OFF;
      gyro_->flag_gyro_.vib_auto = ON;  // Gyro feedback off

      {
        for (i = 0; i < SERV_NUM; i++)
          xv_ref_d_org_[i] = serv_->xv_ref_.d[i];
      }
      if (n >= MODE2_LAMP1 && n <= MODE2_LAMP2)
      {
        for (i = 0; i < SERV_NUM; i++)
        {
          calc_mv_->xv_data_d_[i].mv_tbl_select = MV_TBL_LAMP;
        }
      }

      // action
      sq_motion_mtn_ = 0;

      xv_motion_n_last_ = n;  // store last played motion number

      if (n >= 0 && n < MODE2_MOTION_NUM)  // size check
      {
        if (xp_mv_motionbuf_[n][sq_motion_mtn_].time > 0)
        {
          mode_sq_motion_ = SQ_MODE2_1A;
          mode_sq_time_ = 0.f;
          serv_->set_sw_ref_d(JOINT_ANGLE);  // joint angle control
        }
        else
        {
          mode_sq_motion_ = SQ_MODE2_END;
        }
      }
      else
      {
        mode_sq_motion_ = SQ_MODE2_END;
      }
      break;

    case SQ_MODE2_1A:  // 1, send motion data divided by cycle time
      for (i = 0; i <= 28; i++)
      {
        if (i != HEAD_YAW && i != HEAD_PITCH)
        {
          if (flag_variable_motion_ == ON)
          {
            calc_mv_->xv_data_d_[i].time = xp_mv_motionbuf_[n][sq_motion_mtn_].time / 100.f;
            calc_mv_->xv_data_d_[i].pos = xp_mv_motionbuf_[n][sq_motion_mtn_].d[i] +
                                          xp_mv_motionbuf_var_[n][sq_motion_mtn_].d[i] * variable_amount_;
          }
          else
          {
            calc_mv_->xv_data_d_[i].time = xp_mv_motionbuf_[n][sq_motion_mtn_].time / 100.f;
            calc_mv_->xv_data_d_[i].pos = xp_mv_motionbuf_[n][sq_motion_mtn_].d[i];
          }
        }
        else
        {
          if (!flag_face_control)
          {
            if (flag_variable_motion_ == ON)
            {
              calc_mv_->xv_data_d_[i].time = xp_mv_motionbuf_[n][sq_motion_mtn_].time / 100.f;
              calc_mv_->xv_data_d_[i].pos = xp_mv_motionbuf_[n][sq_motion_mtn_].d[i] +
                                            xp_mv_motionbuf_var_[n][sq_motion_mtn_].d[i] * variable_amount_;
            }
            else
            {
              calc_mv_->xv_data_d_[i].time = xp_mv_motionbuf_[n][sq_motion_mtn_].time / 100.f;
              calc_mv_->xv_data_d_[i].pos = xp_mv_motionbuf_[n][sq_motion_mtn_].d[i];
            }
          }
        }
      }

      if ((xp_mv_motionbuf_[n][sq_motion_mtn_].cntr[2] & 0xF0) == MODE2_GYRO1)
      {
        flag_motion_gyro_ = ON;
      }
      else if ((xp_mv_motionbuf_[n][sq_motion_mtn_].cntr[2] & 0xF0) == MODE2_GYRO2)
      {
        flag_motion_gyro_ = ON3;
      }
      else
      {
        flag_motion_gyro_ = OFF;
      }

      // status
      mode_sq_motion_ = SQ_MODE2_WAIT;
      mode_sq_time_ = 0.0f;

      break;

    case SQ_MODE2_WAIT:  // wait
                         // action
      _flag_w = 0;

      // status
      // time passed
      if (mode_sq_time_ >= (xp_mv_motionbuf_[n][sq_motion_mtn_].time / 100.0f - EPS_TIME))
      {
        if (flag_sq_motion_cancel_)
        {
          _flag_w = 1;
        }
        else if ((xp_mv_motionbuf_[n][sq_motion_mtn_].cntr[2] & 0x0F) >= MODE2_HOLD &&
                 sq_motion_mtn_ < MODE2_MOTION_SIZE)
        {
          _flag_w = 2;
          flag_md_motion_hold_ = ON;
        }
        else if ((!flag_motion_repeat_ && xp_mv_motionbuf_[n][sq_motion_mtn_].cntr[0] == 0) ||
                 (flag_motion_repeat_ && xp_mv_motionbuf_[n][sq_motion_mtn_].cntr[1] == 0))
        {
          _flag_w = 3;
        }
        else if ((!flag_motion_repeat_ && sq_motion_mtn_ == xp_mv_motionbuf_[n][sq_motion_mtn_].cntr[0] - 1) ||
                 (flag_motion_repeat_ && sq_motion_mtn_ == xp_mv_motionbuf_[n][sq_motion_mtn_].cntr[1] - 1))
        {
          if (sq_motion_mtn_ >= 0 && sq_motion_mtn_ <= MODE2_MOTION_SIZE - 1)
          {
            ++sq_motion_mtn_;
          }
          else
          {
            _flag_w = 4;
          }
        }
        else if (!flag_motion_repeat_ ||
                 (xp_mv_motionbuf_[n][sq_motion_mtn_].cntr[0] == xp_mv_motionbuf_[n][sq_motion_mtn_].cntr[1]))
        {
          if (xp_mv_motionbuf_[n][sq_motion_mtn_].cntr[0] >= 1 &&
              xp_mv_motionbuf_[n][sq_motion_mtn_].cntr[0] <= MODE2_MOTION_SIZE)
          {
            sq_motion_mtn_ = xp_mv_motionbuf_[n][sq_motion_mtn_].cntr[0] - 1;
          }
          else
          {
            _flag_w = 5;
          }
        }
        else if (flag_motion_repeat_ && count_motion_repeat_ < xv_motion_repeat_num_)
        {
          if (xp_mv_motionbuf_[n][sq_motion_mtn_].cntr[1] >= 1 &&
              xp_mv_motionbuf_[n][sq_motion_mtn_].cntr[1] <= MODE2_MOTION_SIZE)
          {
            ++count_motion_repeat_;
            sq_motion_mtn_ = xp_mv_motionbuf_[n][sq_motion_mtn_].cntr[1] - 1;
          }
          else
          {
            _flag_w = 6;
          }
        }
        else
        {
          if (sq_motion_mtn_ >= 1 && sq_motion_mtn_ <= MODE2_MOTION_SIZE)
          {
            ++sq_motion_mtn_;
          }
          else
          {
            _flag_w = 7;
          }
        }

        if (!_flag_w)
        {
          if (xp_mv_motionbuf_[n][sq_motion_mtn_].time > 0 && sq_motion_mtn_ < MODE2_MOTION_SIZE)
          {
            mode_sq_motion_ = SQ_MODE2_1A;
            mode_sq_time_ = 0.0f;
          }
          else
          {
            mode_sq_motion_ = SQ_MODE2_END;
          }
        }
        else
        {
          mode_sq_motion_ = SQ_MODE2_END;
        }
      }

      if (flag_sq_motion_cancel_)
      {
        mode_sq_motion_ = SQ_MODE2_END;
      }
      break;

    case SQ_MODE2_END:  // end
      gyro_->flag_gyro_.vib_auto = ON;
      gyro_->flag_gyro_.vib = ON;
      flag_variable_motion_ = OFF;

      // action
      if (n >= MODE2_LAMP1 && n <= MODE2_LAMP2)
      {
        for (i = 0; i < SERV_NUM; i++)
        {
          calc_mv_->xv_data_d_[i].mv_tbl_select = MV_TBL_SIN;
        }
      }

      // store last played motion data
      for (j = 0; j < MODE2_MOTION_SIZE; j++)
      {
        xp_mv_motion_last_[j].time = xp_mv_motionbuf_[xv_motion_n_last_][j].time;
        xp_mv_motion_last_[j].cntr[0] = xp_mv_motionbuf_[xv_motion_n_last_][j].cntr[0];
        xp_mv_motion_last_[j].cntr[1] = xp_mv_motionbuf_[xv_motion_n_last_][j].cntr[1];
        xp_mv_motion_last_[j].cntr[2] = xp_mv_motionbuf_[xv_motion_n_last_][j].cntr[2];

        for (i = 0; i < SERV_NUM; i++)
        {
          xp_mv_motion_last_[j].d[i] = xp_mv_motionbuf_[xv_motion_n_last_][j].d[i];
        }
      }

      // status
      if (flag_sq_motion_cancel_)
      {
        motion_->sq_flag_.motion = OFF;
        flag_md_motion_end_ = ON;
        mode_sq_motion_ = SQ_MODE2_INIT;
        mode_sq_time_ = 0.0f;
      }
      else if ((xp_mv_motionbuf_[n][sq_motion_mtn_].cntr[2] & 0x0F) == MODE2_CONTINUE)
      {
        flag_motion_select_ = n + 1;
        motion_->sq_flag_.motion = ON;
        mode_sq_motion_ = SQ_MODE2_INIT;
        mode_sq_time_ = 0.0f;
      }
      else if ((xp_mv_motionbuf_[n][sq_motion_mtn_].cntr[2] & 0x0F) == MODE2_CONTINUE_SW)
      {
        flag_motion_select_ = n + 1;
        motion_->sq_flag_.motion = OFF;
        flag_md_motion_end_ = ON;
        mode_sq_motion_ = SQ_MODE2_INIT;
        mode_sq_time_ = 0.0f;
      }
      else
      {
        motion_->sq_flag_.motion = OFF;
        flag_md_motion_end_ = ON;
        mode_sq_motion_ = SQ_MODE2_INIT;
        mode_sq_time_ = 0.0f;
      }
      break;

    default:
      break;
  }
  mode_sq_time_ += RTC_TIME_SEC;
  return flag_md_motion_end_;
}

void SqMotion::sq_motion_init()
{
  short i, j, k;
  motion_->sq_flag_.motion = OFF;
  flag_md_motion_end_ = OFF;
  mode_sq_motion_ = SQ_MODE2_INIT;

  flag_motion_select_ = 0;
  flag_md_motion_hold_ = OFF;
  flag_sq_motion_cancel_ = OFF;
  flag_motion_repeat_ = OFF;
  sq_motion_mtn_ = 0;
  count_motion_repeat_ = 0;
  xv_motion_repeat_num_ = 1000;
  flag_motion_preload_ = OFF;
  flag_motion_preload_end_ = OFF;
  xv_motion_n_last_ = 0;
  flag_motion_gyro_ = OFF;

  for (k = 0; k < MODE2_MOTION_NUM; k++)
  {
    for (j = 0; j < MODE2_MOTION_SIZE; j++)
    {
      xp_mv_motionbuf_[k][j].time = 0;
      xp_mv_motionbuf_[k][j].cntr[0] = 0;
      xp_mv_motionbuf_[k][j].cntr[1] = 0;
      xp_mv_motionbuf_[k][j].cntr[2] = 0;
      for (i = 0; i < SERV_NUM; i++)
      {
        xp_mv_motionbuf_[k][j].d[i] = 0;
      }
    }
  }
}

void SqMotion::set_motionbuf(st_xp_mv_motion motionbuf[MODE2_MOTION_SIZE], std::ifstream& file)
{
  char str[256];
  char linebuf[1024];

  for (int frame_no = 0; frame_no < MODE2_MOTION_SIZE; frame_no++)
  {
    file.getline(linebuf, 1024);
    std::string line_str(linebuf);
    std::istringstream sstrm(line_str);
    sstrm.getline(str, 10, ',');
    motionbuf[frame_no].time = std::atoi(str);  // keyframe time

    for (int joint_no = 0; joint_no < 29; joint_no++)
    {
      sstrm.getline(str, 10, ',');
      motionbuf[frame_no].d[joint_no] = std::atoi(str);
    }
    for (int next = 0; next < 3; next++)
    {
      sstrm.getline(str, 10, ',');
      motionbuf[frame_no].cntr[next] = std::atoi(str);  // next page and option
    }
  }
}

void SqMotion::load_pc_motion(std::string directory)
{
  std::filesystem::path dir(directory);

  for (int i = 0; i < MODE2_MOTION_NUM; i++)
  {
    for (int m = 0; m < MODE2_MOTION_SIZE; m++)
    {
      xp_mv_motionbuf_var_[i][m].time = 0;
      for (int joint_no = 0; joint_no < 29; joint_no++)
      {
        xp_mv_motionbuf_var_[i][m].d[joint_no] = 0;
      }
      for (int next = 0; next < 3; next++)
      {
        xp_mv_motionbuf_var_[i][m].cntr[next] = 0;
      }
    }
  }
  if (std::filesystem::exists(dir) && std::filesystem::is_directory(dir))
  {
    for (const auto& entry : std::filesystem::directory_iterator(dir))
    {
      if (std::filesystem::is_directory(entry.path()))
        continue;
      std::string fleaf = entry.path().filename().string();
      std::string ext = entry.path().extension().string();
      int motion_no = 0;
      sscanf(fleaf.substr(0, 3).c_str(), "%d", &motion_no);
      std::cout << fleaf << ": No." << motion_no << std::endl;
      std::ifstream ifs(entry.path().string().c_str());

      if (ext == ".txt")
        set_motionbuf(xp_mv_motionbuf_[motion_no], ifs);
      else if (ext == ".vm")
        set_motionbuf(xp_mv_motionbuf_var_[motion_no], ifs);
    }
  }
  else
  {
    std::cerr << "Directory not found: " << directory << std::endl;
  }
}
}  // namespace hr46
