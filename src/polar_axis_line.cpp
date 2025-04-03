#include "collision_restraint/polar_axis_line.hpp"

#include <angles/angles.h>

#include <cmath>
#include <complex>
#include <format>
#include <limits>
#include <stdexcept>

#include "collision_restraint/utility.hpp"

namespace collision_restraint
{

constexpr float M_F_PI = static_cast<float>(M_PI);
constexpr float M_F_PI_2 = static_cast<float>(M_PI_2);

PolarAxisLine::PolarAxisLine(const float m, const float a, const float b, const bool horizontal)
: m_{m}, horizontal_{horizontal}, line_angle_{horizontal ? M_F_PI_2 : 0.0F}
{
  if (std::isfinite(m_) && std::isfinite(a) && std::isfinite(b)) {
    const float a_abs = std::abs(a);
    const float b_abs = std::abs(b);

    const float min = a * b >= 0.0F ? std::min(a_abs, b_abs) : 0.0F;
    const float max = std::max(a_abs, b_abs);

    const auto radius = [horizontal, m](const float d) -> float {
      return horizontal ? std::abs(std::complex<float>(m, d)) : std::abs(std::complex<float>(d, m));
    };

    min_r_ = radius(min);
    max_r_ = radius(max);
    return;
  }

  throw std::runtime_error(
    source_prefix() + std::format("Non-finite line param: {}; {}, {}", m_, a, b));
}

float PolarAxisLine::m() const { return m_; }
float PolarAxisLine::min_r() const { return min_r_; }
float PolarAxisLine::max_r() const { return max_r_; }
bool PolarAxisLine::horizontal() const { return horizontal_; }

float PolarAxisLine::distance(const float theta, const float r, const bool use_min_theta) const
{
  if (r < min_r_ || r > max_r_) {
    return std::numeric_limits<float>::infinity();
  }

  if (use_min_theta) {
    return angles::normalize_angle_positive(theta - min_theta(r));
  }
  return angles::normalize_angle_positive(theta - max_theta(r));
}

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
  // mirrored around y-axis == +-M_PI_2
  const float offset = M_F_PI_2 - std::abs(theta);
  return {theta, (theta + std::copysign(2.0F * offset, theta))};
}

}  // namespace collision_restraint
