#include "util.h"
#include <cmath>

double minus_pi_to_pi(double alfa)
{
    return remainder(alfa, 2.0 * M_PI);
}