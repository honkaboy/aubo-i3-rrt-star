#include "utilities.h"

#include <cmath>

double Utilities::DegreesToRadians(const double degrees) {
  return degrees / 180.0 * M_PI;
}
double Utilities::RadiansToDegrees(const double radians) {
  return radians / M_PI * 180.0;
}
