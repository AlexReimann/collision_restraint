#pragma once

#include <complex>

namespace collision_restraint
{

class PolarLine
{
public:
  PolarLine(const float m, const float a, const float b);

  [[nodiscard]] float m() const;
  [[nodiscard]] float a() const;
  [[nodiscard]] float b() const;

  [[nodiscard]] float r(const float theta) const;
  [[nodiscard]] float theta(const float r) const;

private:
  float m_;
  float a_;
  float b_;
};

}  // namespace collision_restraint
