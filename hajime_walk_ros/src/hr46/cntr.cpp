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
#include <string_view>
#include "hr46/cntr.hpp"

namespace hr46
{

Cntr::Cntr()
{
  // initialize all the shared pointers
  acc_ = std::make_shared<Acc>();
  calc_deg_ = std::make_shared<CalcDeg>();
  calc_mv_ = std::make_shared<CalcMv>();
  gyro_ = std::make_shared<Gyro>();
  joy_ = std::make_shared<Joy>();
  kine_ = std::make_shared<Kine>();
  motion_ = std::make_shared<Motion>();
  mv_tbl_ = std::make_shared<MvTbl>();
  serv_ = std::make_shared<Serv>();
  sq_motion_ = std::make_shared<SqMotion>();
  sq_ready_ = std::make_shared<SqReady>();
  sq_start_ = std::make_shared<SqStart>();
  sq_straight_ = std::make_shared<SqStraight>();
  sq_walk_ = std::make_shared<SqWalk>();

  // set all the shared pointers
  acc_->calc_mv_ = calc_mv_;
  acc_->motion_ = motion_;
  calc_deg_->gyro_ = gyro_;
  calc_deg_->kine_ = kine_;
  calc_deg_->sq_walk_ = sq_walk_;
  calc_mv_->kine_ = kine_;
  calc_mv_->motion_ = motion_;
  calc_mv_->mv_tbl_ = mv_tbl_;
  calc_mv_->serv_ = serv_;
  calc_mv_->sq_walk_ = sq_walk_;
  gyro_->acc_ = acc_;
  gyro_->joy_ = joy_;
  gyro_->kine_ = kine_;
  gyro_->serv_ = serv_;
  gyro_->sq_walk_ = sq_walk_;
  joy_->acc_ = acc_;
  joy_->calc_mv_ = calc_mv_;
  joy_->motion_ = motion_;
  joy_->serv_ = serv_;
  joy_->sq_motion_ = sq_motion_;
  joy_->sq_ready_ = sq_ready_;
  joy_->sq_walk_ = sq_walk_;
  kine_->serv_ = serv_;
  kine_->calc_mv_ = calc_mv_;
  motion_->sq_motion_ = sq_motion_;
  motion_->sq_ready_ = sq_ready_;
  motion_->sq_start_ = sq_start_;
  motion_->sq_straight_ = sq_straight_;
  motion_->sq_walk_ = sq_walk_;
  serv_->calc_mv_ = calc_mv_;
  serv_->kine_ = kine_;
  sq_motion_->calc_mv_ = calc_mv_;
  sq_motion_->gyro_ = gyro_;
  sq_motion_->motion_ = motion_;
  sq_motion_->serv_ = serv_;
  sq_ready_->calc_mv_ = calc_mv_;
  sq_ready_->gyro_ = gyro_;
  sq_ready_->kine_ = kine_;
  sq_ready_->motion_ = motion_;
  sq_ready_->serv_ = serv_;
  sq_ready_->sq_walk_ = sq_walk_;
  sq_start_->motion_ = motion_;
  sq_straight_->calc_mv_ = calc_mv_;
  sq_straight_->gyro_ = gyro_;
  sq_straight_->kine_ = kine_;
  sq_straight_->motion_ = motion_;
  sq_straight_->serv_ = serv_;
  sq_walk_->calc_mv_ = calc_mv_;
  sq_walk_->gyro_ = gyro_;
  sq_walk_->joy_ = joy_;
  sq_walk_->motion_ = motion_;
  sq_walk_->serv_ = serv_;
  sq_walk_->sq_ready_ = sq_ready_;

  flag_face_control_ = OFF;  // face control is off

  // initialize all the components
  acc_->acc_init();    // initialize acceleration sensor
  gyro_->gyro_init();  // initialize gyro sensor
  // servo_rs_init();			// initialize servo motors
  sq_motion_->sq_motion_init();  // initialize motion sequence from EEPROM
  joy_->joy_init();              // initialize command related variables
  auto z3 = sq_straight_->xp_mv_straight_.z3;
  calc_mv_->calc_mv_init(z3);  // initialize motion calculation
}

void Cntr::setMotionPath(std::string& motion_directory)
{
  sq_motion_->load_pc_motion(motion_directory);
}

void Cntr::setEEPROM(std::map<std::string, double>& param, st_xp_acc& xp_acc, st_xp_gyro& xp_gyro,
                     st_flag_gyro& flag_gyro, st_xp_mv_straight& xp_mv_straight, st_xp_mv_ready& xp_mv_ready,
                     st_xp_mv_walk& xp_walk, st_xp_dlim_wait& xp_dlim_wait_x, st_xp_dlim_wait& xp_dlim_wait_y,
                     st_xp_dlim_wait& xp_dlim_wait_theta, st_xp_dlim_wait& xp_dlim_wait_pitch)
{
  // set all the parameters
  calc_mv_->odometry_correct_para_x_ = param["odometry_correct_para_x"];
  calc_mv_->odometry_correct_para_y_ = param["odometry_correct_para_y"];

  acc_->xp_acc_ = xp_acc;
  gyro_->xp_gyro_ = xp_gyro;
  gyro_->flag_gyro_.fall_cntl = flag_gyro.fall_cntl;
  gyro_->flag_gyro_.zero = flag_gyro.zero;
  sq_straight_->xp_mv_straight_ = xp_mv_straight;
  sq_ready_->xp_mv_ready_ = xp_mv_ready;
  sq_walk_->xp_mv_walk_ = xp_walk;
  sq_walk_->xp_dlim_wait_x_ = xp_dlim_wait_x;
  sq_walk_->xp_dlim_wait_y_ = xp_dlim_wait_y;
  sq_walk_->xp_dlim_wait_theta_ = xp_dlim_wait_theta;
  sq_walk_->xp_dlim_wait_pitch_ = xp_dlim_wait_pitch;
}

void Cntr::setCommand(char cmd, int para1, int para2, int para3, int para4, int para5)
{
  char param1, param2, param3, param4, param5;

  // set command
  switch (cmd)
  {
    case 'A':
      param1 = ParamTable[para1 + PARAM_TABLE_OFFSET];
      param2 = ParamTable[para2 + PARAM_TABLE_OFFSET];
      param3 = ParamTable[para3 + PARAM_TABLE_OFFSET];
      param4 = ParamTable[para4 + PARAM_TABLE_OFFSET];
      param5 = ParamTable[para5 + PARAM_TABLE_OFFSET];
      break;
    case 'M': {
      std::string s = std::to_string(para1);
      s.insert(0, 3 - std::min(static_cast<size_t>(3), s.length()), '0');
      std::string_view pchar = s;
      param1 = pchar[0];
      param2 = pchar[1];
      param3 = pchar[2];
      param4 = ParamTable[para4 + PARAM_TABLE_OFFSET];
      param5 = 0;
      break;
    }
    default:
      cmd = 'C';
      param1 = param2 = param3 = param4 = param5 = 0;
      break;
  }

  joy_->set_xv_comm(&joy_->xv_comm_, cmd, param1, param2, param3, param4, param5);
  joy_->convert_bin(&joy_->xv_comm_bin_, &joy_->xv_comm_);
}

std::vector<double> Cntr::getJoints()
{
  // return calculated joint angles
  std::vector<double> joints;
  joints.push_back(-serv_->xv_ref_.d[ARM_PITCH_L] * (M_PI / 180));
  joints.push_back(serv_->xv_ref_.d[ARM_ROLL_L] * (M_PI / 180));
  joints.push_back(-serv_->xv_ref_.d[ELBOW_PITCH_L] * (M_PI / 180));
  joints.push_back(-serv_->xv_ref_.d[LEG_YAW_L] * (M_PI / 180));
  joints.push_back(-serv_->xv_ref_.d[LEG_ROLL_L] * (M_PI / 180));
  joints.push_back(-serv_->xv_ref_.d[KNEE_L2] * (M_PI / 180));
  joints.push_back(-serv_->xv_ref_.d[KNEE_L1] * (M_PI / 180));
  joints.push_back(-serv_->xv_ref_.d[LEG_PITCH_L] * (M_PI / 180));
  joints.push_back(-serv_->xv_ref_.d[FOOT_ROLL_L] * (M_PI / 180));

  joints.push_back(-serv_->xv_ref_.d[ARM_PITCH_R] * (M_PI / 180));
  joints.push_back(serv_->xv_ref_.d[ARM_ROLL_R] * (M_PI / 180));
  joints.push_back(-serv_->xv_ref_.d[ELBOW_PITCH_R] * (M_PI / 180));
  joints.push_back(-serv_->xv_ref_.d[LEG_YAW_R] * (M_PI / 180));
  joints.push_back(-serv_->xv_ref_.d[LEG_ROLL_R] * (M_PI / 180));
  joints.push_back(-serv_->xv_ref_.d[KNEE_R2] * (M_PI / 180));
  joints.push_back(-serv_->xv_ref_.d[KNEE_R1] * (M_PI / 180));
  joints.push_back(-serv_->xv_ref_.d[LEG_PITCH_R] * (M_PI / 180));
  joints.push_back(-serv_->xv_ref_.d[FOOT_ROLL_R] * (M_PI / 180));
  return joints;
}

void Cntr::setImuData(st_xv_acc& xv_acc, st_xv_gyro& xv_gyro)
{
  acc_->xv_acc_.acc_data1 = xv_acc.acc_data1;
  acc_->xv_acc_.acc_data2 = xv_acc.acc_data2;
  acc_->xv_acc_.acc_data3 = xv_acc.acc_data3;

  gyro_->xv_gyro_.gyro_data1 = xv_gyro.gyro_data1;
  gyro_->xv_gyro_.gyro_data2 = xv_gyro.gyro_data2;
  gyro_->xv_gyro_.gyro_data3 = xv_gyro.gyro_data3;
  gyro_->xv_gyro_.quaternion[0] = xv_gyro.quaternion[0];
  gyro_->xv_gyro_.quaternion[1] = xv_gyro.quaternion[1];
  gyro_->xv_gyro_.quaternion[2] = xv_gyro.quaternion[2];
  gyro_->xv_gyro_.quaternion[3] = xv_gyro.quaternion[3];
  gyro_->xv_gyro_.gyro_roll = xv_gyro.gyro_roll;
  gyro_->xv_gyro_.gyro_roll2 = xv_gyro.gyro_roll2;
  gyro_->xv_gyro_.gyro_pitch = xv_gyro.gyro_pitch;
  gyro_->xv_gyro_.gyro_pitch2 = xv_gyro.gyro_pitch2;
  gyro_->xv_gyro_.gyro_yaw = xv_gyro.gyro_yaw;
  gyro_->xv_gyro_.gyro_yaw2 = xv_gyro.gyro_yaw2;
}

void Cntr::cntr()
{
  joy_->joy(flag_face_control_);       /*	command receive						*/
  motion_->motion(flag_face_control_); /*	motion sequence						*/
  calc_mv_->calc_mv();                 /*	calculate trajectory tables			*/
  calc_deg_->calc_z();                 /*	calculate leg length by hip roll 	*/
  kine_->kine();                       /*	calculate inv. kinetics				*/
  acc_->acc_func();                    /*	acceleration sensor					*/
  gyro_->gyro_fun();                   /*	gyro sensor							*/
  calc_deg_->calc_deg();               /*	calculate joint angle by inv. kinetics data	*/
  serv_->serv();                       /*	servo motor control					*/

  // servo_rs_fun();			/*	output to servo motors				*/
}
}  // namespace hr46
