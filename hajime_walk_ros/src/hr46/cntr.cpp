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

#include "hr46/cntr.hpp"
#include "hr46/var.hpp"

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

  acc_->acc_init();    // initialize acceleration sensor
  gyro_->gyro_init();  // initialize gyro sensor
  // servo_rs_init();			// initialize servo motors
  sq_motion_->sq_motion_init();  // initialize motion sequence from EEPROM
  joy_->joy_init();              // initialize command related variables
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
