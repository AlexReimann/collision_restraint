#include "collision_restraint/point.hpp"

#include <cmath>
#include <format>
#include <stdexcept>

#include "collision_restraint/utility.hpp"

namespace collision_restraint
{

Point::Point(const float x, const float y) : point_{x, y}
{
  if (!std::isfinite(x) || !std::isfinite(y)) {
    throw std::runtime_error(source_prefix() + std::format("Non-finite point: {}; {}", x, y));
  }
}

float Point::x() const { return point_.real(); }

float Point::y() const { return point_.imag(); }

float Point::r() const { return std::abs(point_); }

float Point::theta() const { return std::arg(point_); }

}  // namespace collision_restraint
