#pragma once

#include <tuple>

namespace collision_restraint
{

class PolarAxisLine
{
public:
  PolarAxisLine(const float m, const float a, const float b, const bool horizontal);

  [[nodiscard]] float m() const;
  [[nodiscard]] float min_r() const;
  [[nodiscard]] float max_r() const;
  [[nodiscard]] bool horizontal() const;

  [[nodiscard]] float distance(const float theta, const float r, const bool use_min_theta) const;
  [[nodiscard]] float r(const float theta) const;
  [[nodiscard]] float min_theta(const float r) const;
  [[nodiscard]] float max_theta(const float r) const;
  [[nodiscard]] std::tuple<float, float> thetas(const float r) const;

private:
  float m_;
  float min_r_;
  float max_r_;
  bool horizontal_;
  float line_angle_;
};

}  // namespace collision_restraint
