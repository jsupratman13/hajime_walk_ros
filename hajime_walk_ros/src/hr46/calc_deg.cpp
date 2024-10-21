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

#include "hr46/calc_deg.hpp"
#include "hr46/func.hpp"
#include "hr46/var.hpp"

namespace hr46
{
void CalcDeg::calc_deg()
{
  /*** calculate foot roll ***/
  /* foot is paralell to the ground */
  if (sq_walk_->flag_walk_.upleg == RIGHT) /* right leg is up */
  {
    kine_->xv_kine_[0].foot_r = -kine_->xv_kine_[0].hip_r; /* right foot roll */
    kine_->xv_kine_[1].foot_r = -kine_->xv_kine_[1].hip_r; /* left foot roll */
    kine_->xv_kine_[1].hip_r -= 2;
  }
  else if (sq_walk_->flag_walk_.upleg == LEFT) /* right leg is up */
  {
    kine_->xv_kine_[0].foot_r = -kine_->xv_kine_[0].hip_r; /* right foot roll */
    kine_->xv_kine_[1].foot_r = -kine_->xv_kine_[1].hip_r; /* left foot roll */
    kine_->xv_kine_[0].hip_r += 2;
  }
  else /* both legs touches to the ground */
  {
    kine_->xv_kine_[0].foot_r = -kine_->xv_kine_[0].hip_r; /* right foot roll */
    kine_->xv_kine_[1].foot_r = -kine_->xv_kine_[1].hip_r; /* left foot roll */
  }

  /*** calculate posture pitch ***/
  kine_->xv_kine_[0].leg += kine_->xv_posture_.pitch; /* right leg pitch */
  kine_->xv_kine_[1].leg += kine_->xv_posture_.pitch; /* left leg pitch */

  /*** calculate posture roll ***/
  kine_->xv_kine_[0].hip_r += kine_->xv_posture_.roll2; /* right hip roll */
  kine_->xv_kine_[1].hip_r += kine_->xv_posture_.roll2; /* lef hip roll */

  /*** calculate gyro feedback control ***/
  // TODO: gyro->gyro_cntr_fun();
  //  PostureControl();

  /*** foot touch control ***/
  if (sq_walk_->flag_walk_.upleg == RIGHT) /* right leg is up */
  {
    if (sq_walk_->flag_walk_.y_on == RIGHT)                           /* walk right */
      kine_->xv_kine_[0].foot_r += sq_walk_->xp_mv_walk_.foot_cntl_r; /* right foot right edge up */
    else if (sq_walk_->flag_walk_.y_on == LEFT)                       /* walk left */
      kine_->xv_kine_[0].foot_r -= sq_walk_->xp_mv_walk_.foot_cntl_r; /* right foot left edge up */
  }
  else if (sq_walk_->flag_walk_.upleg == LEFT) /* left leg is up */
  {
    if (sq_walk_->flag_walk_.y_on == RIGHT)                           /* walk right */
      kine_->xv_kine_[1].foot_r += sq_walk_->xp_mv_walk_.foot_cntl_r; /* left foot right edge up */
    else if (sq_walk_->flag_walk_.y_on == LEFT)                       /* walk left */
      kine_->xv_kine_[1].foot_r -= sq_walk_->xp_mv_walk_.foot_cntl_r; /* left foot left edge up */
  }
}

void CalcDeg::calc_z()
{
  sq_walk_->xv_mv_walk_.sidestep_roll_z =
      WIDTH / 2.0f * static_cast<float>(std::sin(hr46::deg2rad(kine_->xv_posture_.roll2)));
  if (kine_->xv_posture_.roll2 > EPS)
  {
    kine_->xv_kine_[0].z += sq_walk_->xv_mv_walk_.sidestep_roll_z;
  }
  else if (kine_->xv_posture_.roll2 < -EPS)
  {
    kine_->xv_kine_[1].z -= sq_walk_->xv_mv_walk_.sidestep_roll_z;
  }
}
}  // namespace hr46
