#pragma once

#include "collision_restraint/velocities.hpp"

namespace collision_restraint
{

class MotionModel
{
public:
  explicit MotionModel(const float deceleration);

  [[nodiscard]] float stoppingDistance(const float linear_velocity) const;
  [[nodiscard]] Velocities scaleToStopDistance(
    const Velocities velocities, const float distance) const;

private:
  float abs_deceleration_;
};

}  // namespace collision_restraint
