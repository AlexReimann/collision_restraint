#include "collision_restraint/motion_model.hpp"

#include <cmath>

namespace collision_restraint
{

MotionModel::MotionModel(const float linear_deceleration, const float angular_deceleration)
: abs_linear_deceleration_{std::abs(linear_deceleration)},
  abs_angular_deceleration_{std::abs(angular_deceleration)}
{
}

float MotionModel::stoppingDistance(const float linear_velocity) const
{
  return (0.5F * linear_velocity * linear_velocity) / abs_linear_deceleration_;
}

float MotionModel::angularStoppingDistance(const float angular_velocity) const
{
  return (0.5F * angular_velocity * angular_velocity) / abs_angular_deceleration_;
}

Velocities MotionModel::scaleToStopDistance(
  const Velocities velocities, const float distance, const float angular_distance) const
{
  if (distance <= 0.0F || angular_distance <= 0.0F) {
    return {0.0F, 0.0F};
  }

  const float target_linear = std::sqrt(2.0F * distance * abs_linear_deceleration_);
  const float target_angular = std::sqrt(2.0F * angular_distance * abs_angular_deceleration_);

  const float scaling =
    std::isnan(angular_distance)
      ? (target_linear / velocities.linear_)
      : std::min(target_linear / velocities.linear_, target_angular / velocities.angular_);

  return {scaling * velocities.linear_, scaling * velocities.angular_};
}

}  // namespace collision_restraint
