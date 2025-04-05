#pragma once

#include <complex>

namespace collision_restraint
{

class PolarPoint
{
public:
  PolarPoint(const float x, const float y);
  [[nodiscard]] static PolarPoint polar(const float r, const float theta);
  [[nodiscard]] static float angularToEucDistance(const float arc_distance, const float radius);

  [[nodiscard]] float x() const;
  [[nodiscard]] float y() const;

  [[nodiscard]] float r() const;
  [[nodiscard]] float theta() const;

private:
  std::complex<float> point_;
};

}  // namespace collision_restraint
