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

// Motion sequence
#ifndef MOTION_HPP_
#define MOTION_HPP_

#include <memory>

#include "var.hpp"
#include "sq_motion.hpp"
#include "sq_start.hpp"
#include "sq_straight.hpp"
#include "sq_ready.hpp"
#include "sq_walk.hpp"

namespace hr46
{
// forward declaration
class SqMotion;
class SqStart;
class SqReady;
class SqStraight;
class SqWalk;

enum
{
  MOTION_NONE,      // No motion
  MOTION_START,     // Servo on
  MOTION_STRAIGHT,  // Straighten body, aka move servo to initial position
  MOTION_READY,     // Bend knees/elbows and prepare to walk
  MOTION_WALK,      // Walking
  MOTION_MOTION,    // Motion sequence
};

enum
{
  STATE_STOP,
  STATE_WALKING,
  STATE_MOTION,
  STATE_MOVING  // stand, ready
};

struct sq_flag_T
{
  int start = 0;     // servo on sequence
  int straight = 0;  // straighten body sequence
  int ready = 0;     // ready sequence
  int walk = 0;      // walk sequence
  int motion = 0;    // motion sequence
};

class Motion
{
private:
  int relax_count_down_ = RTC_TICK;
  int mode_motion_from_ = MOTION_NONE;

public:
  short mode_motion_;
  short flag_moving_;  // 0:STOP,1:WALKING,2:MOTION,3:MOVING
  sq_flag_T sq_flag_;

  Motion();
  // void motion(SqStart* sq_start);
  void motion(short flag_face_control);
  void reset_flag();

  // SqMotionSharedPtr sq_motion_;
  std::shared_ptr<SqMotion> sq_motion_;
  // SqReadySharedPtr sq_ready_;
  std::shared_ptr<SqReady> sq_ready_;
  // SqStartSharedPtr sq_start_;
  std::shared_ptr<SqStart> sq_start_;
  // SqStraightSharedPtr sq_straight_;
  std::shared_ptr<SqStraight> sq_straight_;
  // SqWalkSharedPtr sq_walk_;
  std::shared_ptr<SqWalk> sq_walk_;
};

using MotionSharedPtr = std::shared_ptr<Motion>;

}  // namespace hr46

#endif  // MOTION_HPP_
