#pragma once

#include <tuple>

namespace collision_restraint
{

class PolarAxisLine
{
public:
  PolarAxisLine(const float m, const bool horizontal);

  [[nodiscard]] float m() const;
  [[nodiscard]] bool horizontal() const;

  [[nodiscard]] float r(const float theta) const;
  [[nodiscard]] std::tuple<float, float> thetas(const float r) const;

private:
  float m_;
  bool horizontal_;
  float line_angle_;
};

}  // namespace collision_restraint
