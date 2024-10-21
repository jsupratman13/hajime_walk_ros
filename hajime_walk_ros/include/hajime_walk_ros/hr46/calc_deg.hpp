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

#ifndef CALC_DEG_HPP_
#define CALC_DEG_HPP_

#include <memory>
#include "gyro.hpp"
#include "kine.hpp"
#include "sq_walk.hpp"

namespace hr46
{
class CalcDeg
{
public:
  void calc_deg();
  void calc_z();

  KineSharedPtr kine_;
  GyroSharedPtr gyro_;
  SqWalkSharedPtr sq_walk_;
};

using CalcDegSharedPtr = std::shared_ptr<CalcDeg>;
}  // namespace hr46

#endif  // CALC_DEG_HPP_
