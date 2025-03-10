#include "collision_restraint/polar_line.hpp"

#include <angles/angles.h>

#include <cmath>
#include <format>
#include <stdexcept>

#include "collision_restraint/utility.hpp"

namespace collision_restraint
{

PolarLine::PolarLine(const float m, const float a, const float b) : m_{m}, a_{a}, b_{b}
{
  if (std::isfinite(m) && std::isfinite(a) && std::isfinite(b) && (a != 0.0F || b != 0.0F)) {
    return;
  }

  throw std::runtime_error(
    source_prefix() + std::format("Ill-constructed line: {}; {}, {}", m, a, b));
}

float PolarLine::m() const { return m_; }
float PolarLine::a() const { return a_; }
float PolarLine::b() const { return b_; }

float PolarLine::r(const float theta) const
{
  // ros coordinates -> x: forward, y: left

  if (m_ == 0.0F) {
    const float line_angle = std::atan2(a_, b_);  // ros coordinates -> x: forward, y: left
    constexpr float angle_eps = 0.0001F;
    if (std::fmod(std::abs(line_angle - theta), static_cast<float>(M_PI)) > angle_eps) {
      return 0.0F;
    }

    return std::numeric_limits<float>::infinity();
  }

  // line in polar:
  // ax + by = m
  // x = r*cos(theta)
  // y = r*sin(theta)
  // r = m / ( a*cos(theta) + b*sin(theta) )

  const float r = m_ / ((a_ * std::cos(theta)) + (b_ * std::sin(theta)));
  return r > 0.0F ? r : std::numeric_limits<float>::infinity();
}

float PolarLine::theta(const float r) const
{
  // ros coordinates -> x: forward, y: left

  if (r == 0.0F || m_ == 0.0F) {
    return std::atan2(a_, b_);
  }

  // r = m / u
  // u = k * cos(theta - alpha)
  // k = sqrt(a*a + b*b)
  // tan(alpha) = b / a

  const float k = std::sqrt((a_ * a_) + (b_ * b_));
  const float alpha = std::atan2(b_, a_);

  // u = m / r
  // k * cos(theta - alpha) = m / r
  // cos(theta - alpha) = m / (k * r)
  // theta - alpha = acos(m / (k * r))
  // theta = acos(m / (k * r)) + alpha

  return -std::acos(m_ / (k * r)) + alpha;
}

}  // namespace collision_restraint
