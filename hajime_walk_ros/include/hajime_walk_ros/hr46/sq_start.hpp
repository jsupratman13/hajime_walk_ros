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
// start sequence

#ifndef SQ_START_HPP_
#define SQ_START_HPP_

#include <memory>

#include "motion.hpp"

namespace hr46
{

// forward declaration
class Motion;

class SqStart
{
private:
  short flag_md_start_end_;
  short mode_sq_start_;
  short mode_sq_start_prev_;
  float mode_sq_time_;
  int servo_period_;

public:
  void sq_start_init();
  int sq_start();

  // MotionSharedPtr motion_;
  std::shared_ptr<Motion> motion_;
};

using SqStartSharedPtr = std::shared_ptr<SqStart>;
}  // namespace hr46

#endif  // SQ_START_HPP_
