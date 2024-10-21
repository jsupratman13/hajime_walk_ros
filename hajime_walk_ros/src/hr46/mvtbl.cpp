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

// normalize trajectory tables

#include <cmath>
#include "hr46/mvtbl.hpp"
#include "hr46/var.hpp"

namespace hr46
{

float zmp_fun(float time, float r, float h_cog)
{
  float c1;
  float w;
  w = std::sqrt(GRAVITY / h_cog) * time / 2.f;
  c1 = -r / (std::exp(w) + std::exp(-w));
  return 2.f * c1 + r;
}

MvTbl::MvTbl()
{
  // make the up-and-down movement of the leg an elliptical arc
  for (int i = 0; i <= MV_TBL_PI; i++)
    mv_tbl_[MV_TBL_Z_UP][i] = std::sin(M_PI / 2.0 * i / MV_TBL_PI);
  for (int i = 0; i <= MV_TBL_PI; i++)
    mv_tbl_[MV_TBL_Z_DW][i] = 1.0 - std::cos(M_PI / 2.0 * i / MV_TBL_PI);
  for (int i = 0; i <= MV_TBL_PI; i++)
    mv_tbl_[MV_TBL_X_UP][i] = 1.0 - std::cos(M_PI / 2.0 * i / MV_TBL_PI);
  for (int i = 0; i <= MV_TBL_PI; i++)
    mv_tbl_[MV_TBL_X_DW][i] = std::sin(M_PI / 2.0 * i / MV_TBL_PI);
}

}  // namespace hr46
