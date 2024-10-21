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
#include "hr46/motion.hpp"

namespace hr46
{

Motion::Motion()
{
  mode_motion_ = MOTION_START;   /// current motion mode
  sq_flag_ = { 0, 0, 0, 0, 0 };  // motion sequence flags
  flag_moving_ = STATE_STOP;     // moving state flag
}

// motion
// branching based on the state of the motion
void Motion::motion(short flag_face_control)
{
  int is_first = (mode_motion_ != mode_motion_from_);

  // choose motion based on flag. priority is as follows
  if (mode_motion_ == MOTION_NONE)
  {
    if (sq_flag_.start)
    {
      mode_motion_ = MOTION_START;
    }
    else if (sq_flag_.straight)
    {
      mode_motion_ = MOTION_STRAIGHT;
    }
    else if (sq_flag_.ready)
    {
      mode_motion_ = MOTION_READY;
    }
    else if (sq_flag_.walk)
    {
      if ((mode_motion_from_ != MOTION_WALK) && (mode_motion_from_ != MOTION_READY))
      {
        mode_motion_ = MOTION_READY;
      }
      else
      {
        mode_motion_ = MOTION_WALK;
      }
    }
    else if (sq_flag_.motion)
    {
      if ((mode_motion_from_ != MOTION_WALK) && (mode_motion_from_ != MOTION_READY))
      {
        mode_motion_ = MOTION_READY;
      }
      else
      {
        mode_motion_ = MOTION_MOTION;
      }
    }
  }

  switch (mode_motion_)
  {
    case MOTION_START:  // servo on
      mode_motion_from_ = mode_motion_;
      flag_moving_ = STATE_MOVING;
      // TODO: flag_servo_output_wait = 1;  // wait for servo output

      if (is_first)
      {  // initialize motion
        sq_start_->sq_start_init();
      }
      else if (sq_start_->sq_start())
      {
        // TODO flag_servo_on = ON;  // servo on
        mode_motion_ = MOTION_STRAIGHT;
      }
      break;

    case MOTION_STRAIGHT:  // straight position
      mode_motion_from_ = mode_motion_;
      flag_moving_ = STATE_MOVING;

      if (is_first)
      {
        sq_straight_->sq_straight_init();
      }
      else if (sq_straight_->sq_straight(flag_face_control))
      {
        flag_moving_ = STATE_STOP;
        mode_motion_ = MOTION_NONE;
      }
      break;

    case MOTION_READY:  // ready position
      if (is_first)
      {
        int slow_mode = (mode_motion_from_ == MOTION_STRAIGHT);
        sq_ready_->sq_ready_init(slow_mode);  // straight to ready mode transition should be slow
      }
      else if (sq_ready_->sq_ready(flag_face_control))
      {
        flag_moving_ = STATE_STOP;
        mode_motion_ = MOTION_NONE;
      }

      mode_motion_from_ = STATE_STOP;
      flag_moving_ = STATE_MOVING;
      break;

    case MOTION_WALK:  // walk
      mode_motion_from_ = mode_motion_;
      flag_moving_ = STATE_WALKING;
      if (is_first)
      {
      }
      if (sq_walk_->sq_walk() && (sq_flag_.walk == OFF))
      {
        mode_motion_ = MOTION_NONE;
      }
      break;

    case MOTION_MOTION:  // motion sequence
      mode_motion_from_ = mode_motion_;
      flag_moving_ = STATE_MOTION;
      if (sq_motion_->sq_motion())
      {
        flag_moving_ = STATE_STOP;
        mode_motion_ = MOTION_READY;
      }
      break;

    default:
      flag_moving_ = STATE_STOP;
      break;
  }
}

void Motion::reset_flag()
{
  sq_flag_ = { 0, 0, 0, 0, 0 };
}

}  // namespace hr46
