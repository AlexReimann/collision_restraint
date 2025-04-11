#pragma once

#include "collision_restraint/velocities.hpp"

namespace collision_restraint
{

class MotionModel
{
public:
  explicit MotionModel(const float linear_deceleration, const float angular_deceleration);

  [[nodiscard]] float stoppingDistance(const float linear_velocity) const;
  [[nodiscard]] float angularStoppingDistance(const float angular_velocity) const;
  [[nodiscard]] Velocities scaleToStopDistance(
    const Velocities velocities, const float distance, const float angular_distance) const;

private:
  float abs_linear_deceleration_;
  float abs_angular_deceleration_;
};

}  // namespace collision_restraint
