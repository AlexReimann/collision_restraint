#pragma once

#include <complex>

namespace collision_restraint
{

class Point
{
public:
  Point(const float x, const float y);

  [[nodiscard]] float x() const;
  [[nodiscard]] float y() const;

  [[nodiscard]] float r() const;
  [[nodiscard]] float theta() const;

private:
  std::complex<float> point_;
};

}  // namespace collision_restraint
