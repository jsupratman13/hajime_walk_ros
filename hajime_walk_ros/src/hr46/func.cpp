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
#include "hr46/func.hpp"
#include "hr46/var.hpp"

namespace hr46
{

void TIMER(bool in, short& out, float t, short& w)
{
  if (in)
  {
    if (++w > t)
    {
      out = 1;
      w = t;
    }
  }
  else
  {
    out = w = 0;
  }
}

// differential
float diff(float x, float t1, float t2, float* work)
{
  float flt_out, out;

  flt_out = filterf(x, *work, t1);
  out = (flt_out - *work) * t2 / RTC_TIME;
  *work = flt_out;

  return out;
}

// pulse1
int pulse1(short in, short* old)
{
  short out;

  if (in && !*old)
  {
    out = 1;
  }
  else
  {
    out = 0;
  }

  *old = in;

  return out;
}

// pid_fun
void pid_fun(st_xp_pid* xp, st_xv_pid* xv)
{
  xv->err = xv->sv - xv->pv;
  xv->pv_s = xv->pv - xv->pv_old;
  xv->k_out = xv->err * xp->ki - xv->pv_s * xp->kp;

  if (xp->sw == 1)
  {
    xv->mv = integrator_f(xv->k_out / RTC_TICK, xv->mv, xp->i_lim);
  }
  else if (xp->sw == 0)
  {
    xv->mv = dlimit(0.f, xp->d_lim / RTC_TICK, xv->mv);
  }

  xv->pv_old = xv->pv;
}

// pid_fun_init
void pid_fun_init(st_xp_pid* xp, st_xv_pid* xv)
{
  xv->sv = 0.f;
  xv->pv = 0.f;
  xv->err = 0.f;
  xv->pv_s = 0.f;
  xv->k_out = 0.f;
  xv->mv = 0.f;
  xv->pv_old = 0.f;
}

// dlim_wait_fun
// implementation not complete?
float dlim_wait_fun(st_xp_dlim_wait* xp, st_xv_dlim_wait* xv)
{
  float y2 = dlimit(xv->in, xp->dlim / RTC_TICK, xv->out);
  xv->dout = (y2 - xv->out) * RTC_TICK;

  // ZERO
  if ((y2 >= -EPS_DATA) && (y2 <= EPS_DATA))
  {
    xv->sign_old = 0;
    xv->time_work += RTC_TIME_SEC;
  }
  // PLUS
  else if ((xv->sign_old > 0) && (y2 > EPS_DATA))
  {
    xv->sign_old = 1;
    xv->time_work = 0.f;
  }
  // MINUS
  else if ((xv->sign_old < 0) && (y2 < -EPS_DATA))
  {
    xv->sign_old = -1;
    xv->time_work = 0.f;
  }
  // PLUS -> wait_time -> MINUS
  else if ((xv->sign_old >= 0) && (y2 < -EPS_DATA) && (xv->time_work >= xp->wait_time))
  {
    xv->sign_old = -1;
  }
  // MINUS -> wait_time -> PLUS
  else if ((xv->sign_old <= 0) && (y2 > EPS_DATA) && (xv->time_work >= xp->wait_time))
  {
    xv->sign_old = 1;
  }
  else
  {
    y2 = 0.f;
    xv->time_work += RTC_TIME_SEC;
  }

  xv->out = y2;

  return xv->out;
}

// dlim_wait_fun_init
void dlim_wait_fun_init(st_xp_dlim_wait* xp, st_xv_dlim_wait* xv)
{
  xv->sign_old = 0;
  xv->in = xv->out = xv->dout = xv->time_work = 0.f;
}

}  // namespace hr46
