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

// control functions
#ifndef FUNC_HPP_
#define FUNC_HPP_

#include <cmath>

#include "var.hpp"

namespace hr46
{

constexpr float rad2deg(float x)
{
  return x * 180.0f / M_PI;
}

constexpr float deg2rad(float x)
{
  return x * M_PI / 180.0f;
}

template <typename T>
constexpr T sw2(bool s, T x1, T x2)
{
  return s ? x1 : x2;
}

// limit x between (y-d) and (y+d)
template <typename T>
constexpr T dlimit(T x, T d, T y)
{
  return ((x > (y + d)) ? (y + d) : ((x < (y - d)) ? (y - d) : (x)));
}

// filter function with RTC_TIME constant
constexpr float filterf(float in, float out, float t)
{
  return ((in * RTC_TIME + out * t) / (t + RTC_TIME));
}

// integrator function
constexpr float integrator_f(float x, float y, float lim)
{
  return ((x + y) > lim ? lim : ((x + y) < -lim ? -lim : (x + y)));
}

// dead band function t : time constant [ms]
constexpr float deadband(float x, float d)
{
  return ((x > d) ? (x) : ((x < -(d)) ? (x) : (0)));
}

// HOLD logic function
// output = HOLD( set, reset, output )
constexpr bool HOLD(bool a, bool b, bool c)
{
  return (c || a) && (!b);
}

// TIMER logic function
void TIMER(bool in, short& out, float t, short& w);

struct st_xp_pid
{
  float kp;
  float ki;
  float i_lim;
  float d_lim;
  short sw;
};

struct st_xv_pid
{
  float sv;
  float pv;
  float err;
  float pv_s;
  float k_out;
  float mv;
  float pv_old;
};

struct st_xp_dlim_wait
{
  float dlim;       // max change (1/s)
  float wait_time;  // time to wait before changing direction (s)
};

struct st_xv_dlim_wait
{
  short sign_old;   // Which direction it was moving previously
  float in;         // Input (final target value)
  float out;        // Output (value that gradually changes)
  float dout;       // Change in output
  float time_work;  // Elapsed time since the direction was switched (in seconds)
};

float diff(float x, float t1, float t2, float* work);
int pulse1(short in, short* old);
void pid_fun(st_xp_pid* xp, st_xv_pid* xv);
void pid_fun_init(st_xp_pid* xp, st_xv_pid* xv);
float dlim_wait_fun(st_xp_dlim_wait* xp, st_xv_dlim_wait* xv);
void dlim_wait_fun_init(st_xp_dlim_wait* xp, st_xv_dlim_wait* xv);

}  // namespace hr46

#endif  // FUNC_HPP_
