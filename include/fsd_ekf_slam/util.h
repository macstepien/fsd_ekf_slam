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