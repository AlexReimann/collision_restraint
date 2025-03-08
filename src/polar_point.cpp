#include "collision_restraint/polar_point.hpp"

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

float PolarPoint::x() const { return point_.real(); }

float PolarPoint::y() const { return point_.imag(); }

float PolarPoint::r() const { return std::abs(point_); }

float PolarPoint::theta() const { return std::arg(point_); }

}  // namespace collision_restraint
