#pragma once

#include "collision_restraint/velocities.hpp"

namespace collision_restraint
{

class MotionModel
{
public:
  explicit MotionModel(const float deceleration, const float execution_delay = 0.0F);

  [[nodiscard]] float minStoppingDistance(const float linear_velocity) const;
  [[nodiscard]] Velocities scaleToStopDistance(
    const Velocities velocities, const float distance) const;

private:
  float abs_deceleration_;

  float execution_delay_;
};

}  // namespace collision_restraint
