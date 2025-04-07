#include "collision_restraint/motion_model.hpp"

#include <cmath>

namespace collision_restraint
{

MotionModel::MotionModel(const float deceleration) : abs_deceleration_{std::abs(deceleration)} {}

float MotionModel::stoppingDistance(const float linear_velocity) const
{
  return (0.5F * linear_velocity * linear_velocity) / abs_deceleration_;
}

Velocities MotionModel::scaleToStopDistance(const Velocities velocities, const float distance) const
{
  if (distance <= 0.0F) {
    return {0.0F, 0.0F};
  }

  const float target_linear = std::sqrt(2.0F * distance * abs_deceleration_);

  // we want to keep the curvature constant to keep going along the same trajectory, but slower
  const float curvature = velocities.linear_ / velocities.angular_;

  // target angular velocity might actually not be reachable because of dynamics,
  // but ignoring this here
  const float target_angular = target_linear / curvature;

  return {target_linear, target_angular};
}

}  // namespace collision_restraint
