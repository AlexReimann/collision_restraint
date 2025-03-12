#include "collision_restraint/polar_axis_line.hpp"

#include <angles/angles.h>

#include <cmath>
#include <format>
#include <limits>
#include <stdexcept>

#include "collision_restraint/utility.hpp"

namespace collision_restraint
{

constexpr float M_F_PI = static_cast<float>(M_PI);
constexpr float M_F_PI_2 = static_cast<float>(M_PI_2);

PolarAxisLine::PolarAxisLine(const float m, const bool horizontal)
: m_{m}, horizontal_{horizontal}, line_angle_{horizontal ? M_F_PI_2 : 0.0F}
{
  if (std::isfinite(m)) {
    return;
  }

  throw std::runtime_error(source_prefix() + std::format("Non-finite line offset: {}", m));
}

float PolarAxisLine::m() const { return m_; }
bool PolarAxisLine::horizontal() const { return horizontal_; }

float PolarAxisLine::r(const float theta) const
{
  // ros coordinates -> x: forward, y: left

  if (std::abs(m_) <= 0.001F) {
    constexpr float angle_eps = 0.0001F;

    if (std::fmod(std::abs(line_angle_ - theta), M_F_PI) < angle_eps) {
      return std::numeric_limits<float>::infinity();
    }
    return 0.0F;
  }

  if (
    (horizontal_ &&
     ((m_ > 0.0F && std::abs(theta) >= M_F_PI_2) || (m_ < 0.0F && std::abs(theta) <= M_F_PI_2))) ||
    (!horizontal_ && ((m_ > 0.0F && theta <= 0.0F) || (m_ < 0.0F && theta >= 0.0F)))) {
    return std::numeric_limits<float>::infinity();
  }

  // line in polar:
  // ax + by = m
  // x = r*cos(theta)
  // y = r*sin(theta)
  // r = m / ( a*cos(theta) + b*sin(theta) )
  // -> simplified

  const float d = horizontal_ ? std::cos(theta) : std::sin(theta);
  const float r = m_ / d;

  // check for numeric errors
  return r > 0.0F ? r : std::numeric_limits<float>::infinity();
}

float PolarAxisLine::min_theta(const float r) const
{
  const auto [a, b] = thetas(r);
  return std::min(a, b);
}

float PolarAxisLine::max_theta(const float r) const
{
  const auto [a, b] = thetas(r);
  return std::max(a, b);
}

std::tuple<float, float> PolarAxisLine::thetas(const float r) const
{
  // ros coordinates -> x: forward, y: left

  if (r == 0.0F || m_ == 0.0F) {
    return {line_angle_, std::numeric_limits<float>::quiet_NaN()};
  }

  // r = m / ( a*cos(theta) + b*sin(theta) )
  // a*cos(theta) + b*sin(theta) = m / r
  // -> simplified

  if (horizontal_) {
    const float theta = std::acos(m_ / r);
    return {theta, -theta};
  }

  const float theta = std::asin(m_ / r);
  return {theta, angles::normalize_angle(theta + M_F_PI)};
}

}  // namespace collision_restraint
