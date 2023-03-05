// Copyright 2023 Maciej Stępień

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef UTIL_H
#define UTIL_H

#include <cmath>

inline double BoundToMinusPiPi(double angle)
{
  return remainder(angle, 2.0 * M_PI);
}
struct Observation
{
  float r;
  float angle;
};

#endif