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

// high speed walk sequence
#ifndef SQ_WALK_HPP_
#define SQ_WALK_HPP_

#include <memory>

#include "func.hpp"
#include "joy.hpp"
#include "sq_ready.hpp"
#include "gyro.hpp"
#include "calc_mv.hpp"
#include "serv.hpp"

namespace hr46
{
// forward declaration
class CalcMv;
class Gyro;
class Joy;
class Motion;
class Serv;
class SqReady;

enum
{
  SQ_WALK_INIT,    //  0) Initialization                                    t: 0
  SQ_WALK_MV_L,    //  1) Shift the center of gravity to the left           t: 0 - period/4*k2 (reset to 0)
  SQ_WALK_UP_R0,   //  2) Waiting time before lifting the right foot (first step)    t: period/4*k2
  SQ_WALK_UP_R,    //  3) Motion to lift the right foot                     t: 0 - period/2*(1-duty)
  SQ_WALK_UP_R_1,  //  4) Motion to lift the right foot                     t: period/2*(1-duty)
  SQ_WALK_DW_R,    //  5) Motion to lower the right foot                    t: period/2*(1-duty/2) - period/2
  SQ_WALK_MV_R2,   //  6) Shift the center of gravity to the right          t: 0

  SQ_WALK_MV_R,    //  7) Shift the center of gravity to the right
  SQ_WALK_UP_L0,   //  8) Waiting time before lifting the left foot (first step)
  SQ_WALK_UP_L,    //  9) Motion to lift the left foot                      t: 0 - period/2*(1-duty)
  SQ_WALK_UP_L_1,  // 10) Motion to lift the left foot                      t: period/2*(1-duty)
  SQ_WALK_DW_L,    // 11) Motion to lower the left foot                     t: period/2*(1-duty/2) - period/2
  SQ_WALK_MV_L2,   // 12) Shift the center of gravity to the left → 4) Final step → 14)

  SQ_WALK_UP_R2,    // 13) Motion to lift the right foot (final step)
  SQ_WALK_UP_R2_1,  // 14) Motion to lift the right foot (final step)
  SQ_WALK_UP_R2_2,  // 15) Motion to lift the right foot (final step)
  SQ_WALK_DW_R2,    // 16) Motion to lower the right foot (final step)

  SQ_WALK_UP_L2,    // 17) Motion to lift the left foot (final step)
  SQ_WALK_UP_L2_1,  // 18) Motion to lift the left foot (final step)
  SQ_WALK_UP_L2_2,  // 19) Motion to lift the left foot (final step)
  SQ_WALK_DW_L2,    // 20) Motion to lower the left foot (final step)

  SQ_WALK_READY,  // 21) Return to ready state
  SQ_WALK_END,    // 22) End process
};

struct st_xp_mv_walk
{
  long num;                        // number of steps for test [-]
  float h_cog;                     // height of center of gravity [mm]
  float time;                      // step time [sec] (unsued)
  float next_walk;                 // timing for next step [1]
  float x_fwd_swg;                 // forward swing [mm]
  float x_fwd_spt;                 // forward support [mm]
  float x_bwd_swg;                 // backward swing [mm]
  float x_bwd_spt;                 // backward support [mm]
  float y_swg;                     // lateral swing [mm]
  float y_spt;                     // lateral support [mm]
  float theta;                     // rotation [deg]
  float z;                         // height [mm]
  float y_balance;                 // balance [mm]  (Distance between the legs / 2)
  float hip_roll;                  // hip roll [deg]
  float x_fwd_pitch;               // forward pitch [deg]
  float x_bwd_pitch;               // backward pitch [deg]
  float x_fwd_acc_pitch;           // forward acceleration pitch [deg]
  float x_bwd_acc_pitch;           // backward acceleration pitch [deg]
  float arm_sh_pitch;              // arm shoulder pitch [deg]
  float arm_el_pitch;              // elbow pitch angle [deg]
  float start_zmp_k1;              // start zmp k1 [-]
  float start_time_k1;             // start time k1 [sec]
  float start_time_k2;             // start time k2 [sec] (unused)
  float foot_cntl_p;               // foot control p [-]
  float foot_cntl_r;               // foot control r [-]
  float sidestep_time_k;           // sidestep time k [sec]
  float sidestep_roll;             // sidestep roll [-]
  float y_wide;                    // wide step [mm]
  float time_dutyfactor;           // time duty factor [-]
  float accurate_x_percent_dlim;   // accurate x percent dlim [-]
  float accurate_y_percent_dlim;   // accurate y percent dlim [-]
  float accurate_th_percent_dlim;  // accurate theta percent dlim [-]
  float accurate_step_z;           // accurate step z [mm]
  float accurate_step_time;        // accurate step time [sec]
};

// struct st_xv_mv_walk
// {
//   long num;                  // Number of steps	[-]
//   float time;                // Half of the walking cycle [sec]
//   float time_old;            // Time when the walking cycle was last changed [sec]
//   float x_swg;               // Step length in the forward/backward direction of the swing leg [mm]
//   float x_spt;               // Step length in the forward/backward direction of the support leg [mm]
//   float y_swg;               // Step length in the left/right direction of the swing leg [mm]
//   float y_spt;               // Step length in the left/right direction of the support leg [mm]
//   float theta;               // Yaw angle of the legs for each turn	[deg]
//   float z;                   // Height to lift the foot [mm]
//   float pitch;               // Angle to bend forward [deg]
//   float arm_sh_pitch;        // Shoulder pitch angle [deg]
//   float arm_el_pitch;        // Elbow pitch angle [deg]
//   float zmp;                 // Swing width of the foot according to ZMP criterion? [mm]
//   float x_percent;           // Final percentage of step length in the x direction (0-1) [-]
//   float y_percent;           // Final percentage of step length in the y direction (0-1) [-]
//   float theta_percent;       // Final percentage of yaw angle of the legs (0-1) [-]
//   float sidestep_time_k_r;   // Percentage of time for right foot movement (0-1) [bit]
//   float sidestep_time_k_l;   // Percentage of time for left foot movement (0-1) [bit]
//   float sidestep_roll;       // Roll axis rotation angle during side step [deg]
//   float sidestep_roll_z;     // Change in the height of the leg axis during side step [mm]
//   float x_percent_dlim;      // Current percentage of step length in the x direction (0-1) [-]
//   float y_percent_dlim;      // Current percentage of step length in the y direction (0-1) [-]
//   float theta_percent_dlim;  // Current percentage of yaw angle of the legs (0-1) [-]
//   float pitch_percent_dlim;  // Current percentage of pitch axis (0-1) [-]
//   float time_dutyfactor;     // Ratio of time when the leg is in the swing phase (0-1) [-]
//   float accurate_step_x;     // Distance moved forward in one step [mm]
//   float accurate_step_y;
//   float accurate_step_z;
//   float accurate_step_th;
//   float accurate_step_time;
// };

struct st_flag_walk
{
  short upleg;       // up right leg or left leg
  short upleg_last;  // the last up leg
  short y;           // walk sidestep
  short turn;        // turn right or left
  short y_on;        // walk sidestep now
  short turn_on;     // turn right or left now
};

class SqWalk
{
private:
  float t1, t2;
  float t1a, t1b, t1c, t2a;
  float t1_r, t2_r;
  float t1a_r, t1b_r, t1c_r, t2a_r;
  float t1_l, t2_l;
  float t1a_l, t1b_l, t1c_l, t2a_l;
  float mode_sq_time_;
  float _xv_mv_walk_y_swg;
  float _xv_mv_walk_y_spt;
  float _xv_posture_roll2;

public:
  st_xp_mv_walk xp_mv_walk_;            // Constants related to walking
  st_xv_mv_walk xv_mv_walk_;            // Variables related to walking
  st_flag_walk flag_walk_;              // Walking flag
  short mode_sq_walk_;                  // Walking mode number
  st_xp_dlim_wait xp_dlim_wait_x_;      // Constants for the percentage of step length in the x direction
  st_xv_dlim_wait xv_dlim_wait_x_;      // Variables for the percentage of step length in the x direction
  st_xp_dlim_wait xp_dlim_wait_y_;      // Constants for the percentage of step length in the y direction
  st_xv_dlim_wait xv_dlim_wait_y_;      // Variables for the percentage of step length in the y direction
  st_xp_dlim_wait xp_dlim_wait_theta_;  // Constants for the percentage of yaw angle of the legs
  st_xv_dlim_wait xv_dlim_wait_theta_;  // Variables for the percentage of yaw angle of the legs
  st_xp_dlim_wait xp_dlim_wait_pitch_;  // Constants for the pitch direction tilt
  st_xv_dlim_wait xv_dlim_wait_pitch_;  // Variables for the pitch direction tilt
  char accurate_one_step_mode_;         // Mode for accurate one-step walking
  short flag_md_walk_end_;              // Walking end flag

  SqWalk();
  int sq_walk();       // walking sequence, 1 finished, 0 not finished, called every 20 ms
  void reset_flag();   // initialize walking sequence
  void sq_walk_fun();  // walking sequence
  void side_step_modify(float t1, float t2, float t1a, float t1b, float t1c, float t2a, float* t1_r, float* t2_r,
                        float* t1a_r, float* t1b_r, float* t1c_r, float* t2a_r, float* t1_l, float* t2_l, float* t1a_l,
                        float* t1b_l, float* t1c_l, float* t2a_l, float* _xv_mv_walk_y_swg, float* _xv_mv_walk_y_spt,
                        float* _xv_posture_roll2);

  // CalcMvSharedPtr calc_mv_;
  std::shared_ptr<CalcMv> calc_mv_;
  // GyroSharedPtr gyro_;
  std::shared_ptr<Gyro> gyro_;
  // JoySharedPtr joy_;
  std::shared_ptr<Joy> joy_;
  // MotionSharedPtr motion_;
  std::shared_ptr<Motion> motion_;
  // ServSharedPtr serv_;
  std::shared_ptr<Serv> serv_;
  // SqReadySharedPtr sq_ready_;
  std::shared_ptr<SqReady> sq_ready_;
};

using SqWalkSharedPtr = std::shared_ptr<SqWalk>;
}  // namespace hr46
#endif  // SQ_WALK_HPP_
