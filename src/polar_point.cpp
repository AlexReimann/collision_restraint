#include "collision_restraint/polar_point.hpp"

#include <angles/angles.h>

#include <cmath>
#include <format>
#include <stdexcept>

#include "collision_restraint/utility.hpp"

namespace collision_restraint
{

PolarPoint::PolarPoint(const float x, const float y) : point_{x, y}
{
  if (std::isfinite(x) && std::isfinite(y)) {
    return;
  }

  throw std::runtime_error(source_prefix() + std::format("Non-finite point: {}; {}", x, y));
}

PolarPoint PolarPoint::polar(const float r, const float theta)
{
  PolarPoint point{0.0F, 0.0F};
  point.point_ = std::polar(r, theta);
  return point;
}

float PolarPoint::angularToEucDistance(const float angular_distance, const float radius)
{
  return angular_distance * radius;
}

float PolarPoint::eucToAngularDistance(const float euc_distance, const float radius)
{
  return euc_distance / radius;
}

float PolarPoint::x() const { return point_.real(); }
float PolarPoint::y() const { return point_.imag(); }

float PolarPoint::r() const { return std::abs(point_); }
float PolarPoint::theta() const { return angles::normalize_angle_positive(std::arg(point_)); }

}  // namespace collision_restraint
