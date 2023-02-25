#include <ekf_slam/util.h>

#include <cmath>

double BoundToMinusPiPi(double angle)
{
  return remainder(angle, 2.0 * M_PI);
}