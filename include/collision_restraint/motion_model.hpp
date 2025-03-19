#pragma once

namespace collision_restraint
{

class MotionModel
{
public:
  struct Velocities
  {
    float linear_;
    float angular_;
  };

  explicit MotionModel(const float abs_max_deceleration, const float execution_delay = 0.0F);

  [[nodiscard]] float minStoppingDistance(const float linear_velocity) const;
  [[nodiscard]] Velocities scaleToStopDistance(
    const Velocities velocities, const float distance) const;

private:
  float abs_max_deceleration_;

  float execution_delay_;
};

}  // namespace collision_restraint
