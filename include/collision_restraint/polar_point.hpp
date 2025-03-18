#pragma once

#include <complex>

namespace collision_restraint
{

class PolarPoint
{
public:
  PolarPoint(const float x, const float y);
  static PolarPoint polar(const float r, const float theta);

  [[nodiscard]] float x() const;
  [[nodiscard]] float y() const;

  [[nodiscard]] float r() const;
  [[nodiscard]] float theta() const;

private:
  std::complex<float> point_;
};

}  // namespace collision_restraint
