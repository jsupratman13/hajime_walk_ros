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

// Global variables

#ifndef VAR_HPP_
#define VAR_HPP_

namespace hr46
{
#define RTC_TIME (10)         /* real time clock [msec] 			*/
#define RTC_TICK (100)        /* 1000[msec]/10(RTC_TIME)[msec] 	*/
#define RTC_TIME_SEC (0.010f) /* real time clock [sec] 			*/

#define EPS_TIME (0.001f) /* epsilon time [sec]	 			*/
#define EPS_DATA (0.001f) /* epsilon				 			*/

#define SERV_NUM (29) /* number of servos	(never change)		*/
#define ACCELITE_SERV_NUM 19

#define PI (3.14159f)
#define PI2 (6.28318f)
#define PIW2 (1.57080f)
/*	G	[mm/s^2]	*/
#define GRAVITY (9800.f)

/***	set mechanical data	***/
#define WIDTH (88.0f)
#define L01 (0.0f)
#define L1 (118.0f)
#define L12 (23.0f)
#define L2 (118.0f)
#define L23 (0.0f)
#define L3 (43.0f) /*	43+0(sole)mm	*/

#define Z3_LIMIT_H ((float)(L01 + L1 + L12 + L2 + L23 + L3))  // Max waist height
#define Z3_LIMIT_L (120.f)                                    // Min waist height

#define REDUCTION_RATION (2.0f)

/*** typdef  ***/
enum
{
  OFF = 0,
  ON = 1,
  ON2 = 2,
  ON3 = 3
};

enum
{
  STOP = 0,
  FWD = 1,
  BWD = 2
};

enum
{
  STRAIGHT = 0,
  RIGHT = 1,
  LEFT = 2
};

enum
{
  UP = 1,
  DOWN = 2
};

enum
{
  FOOT_XYZ = 1,    // Control foot in Cartesian coordinates
  JOINT_ANGLE = 2  // Control in joint coordinates
};

enum
{
  MV_TBL_SIN = 0,
  MV_TBL_LAMP = 1,
  MV_TBL_ZMP = 2,
  MV_TBL_Z_UP = 3,
  MV_TBL_Z_DW = 4,
  MV_TBL_ZMP2 = 5,
  MV_TBL_COS = 6,
  MV_TBL_X_UP = 7,
  MV_TBL_X_DW = 8,
};

enum
{
  FOOT_ROLL_R = 0,
  LEG_PITCH_R = 1,
  KNEE_R1 = 2,
  KNEE_R2 = 3,
  LEG_ROLL_R = 4,
  LEG_YAW_R = 5,
  ARM_PITCH_R = 6,
  ARM_ROLL_R = 7,
  ELBOW_PITCH_R = 8,
  FOOT_ROLL_L = 9,
  LEG_PITCH_L = 10,
  KNEE_L1 = 11,
  KNEE_L2 = 12,
  LEG_ROLL_L = 13,
  LEG_YAW_L = 14,
  ARM_PITCH_L = 15,
  ARM_ROLL_L = 16,
  ELBOW_PITCH_L = 17,
  HEAD_YAW = 18,
  HEAD_PITCH = 19,
  SPARE12 = 20,
  SPARE13 = 21,
  SPARE17 = 22,
  SPARE21 = 23,
  SPARE24 = 24,
  SPARE25 = 25,
  SPARE26 = 26,
  SPARE27 = 27,
  SPARE28 = 28
};

struct st_xv_mv_walk
{
  long num;                  // Number of steps	[-]
  float time;                // Half of the walking cycle [sec]
  float time_old;            // Time when the walking cycle was last changed [sec]
  float x_swg;               // Step length in the forward/backward direction of the swing leg [mm]
  float x_spt;               // Step length in the forward/backward direction of the support leg [mm]
  float y_swg;               // Step length in the left/right direction of the swing leg [mm]
  float y_spt;               // Step length in the left/right direction of the support leg [mm]
  float theta;               // Yaw angle of the legs for each turn	[deg]
  float z;                   // Height to lift the foot [mm]
  float pitch;               // Angle to bend forward [deg]
  float arm_sh_pitch;        // Shoulder pitch angle [deg]
  float arm_el_pitch;        // Elbow pitch angle [deg]
  float zmp;                 // Swing width of the foot according to ZMP criterion? [mm]
  float x_percent;           // Final percentage of step length in the x direction (0-1) [-]
  float y_percent;           // Final percentage of step length in the y direction (0-1) [-]
  float theta_percent;       // Final percentage of yaw angle of the legs (0-1) [-]
  float sidestep_time_k_r;   // Percentage of time for right foot movement (0-1) [bit]
  float sidestep_time_k_l;   // Percentage of time for left foot movement (0-1) [bit]
  float sidestep_roll;       // Roll axis rotation angle during side step [deg]
  float sidestep_roll_z;     // Change in the height of the leg axis during side step [mm]
  float x_percent_dlim;      // Current percentage of step length in the x direction (0-1) [-]
  float y_percent_dlim;      // Current percentage of step length in the y direction (0-1) [-]
  float theta_percent_dlim;  // Current percentage of yaw angle of the legs (0-1) [-]
  float pitch_percent_dlim;  // Current percentage of pitch axis (0-1) [-]
  float time_dutyfactor;     // Ratio of time when the leg is in the swing phase (0-1) [-]
  float accurate_step_x;     // Distance moved forward in one step [mm]
  float accurate_step_y;
  float accurate_step_z;
  float accurate_step_th;
  float accurate_step_time;
};
}  // namespace hr46

#endif  // VAR_HPP_
