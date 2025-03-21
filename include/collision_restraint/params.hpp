#pragma once

namespace collision_restraint
{
class Params
{
public:
  Params(
    const float deceleration, const float execution_delay, const float min_obstacle_height,
    const float max_obstacle_height, const bool ignore_inside_footprint)
  : deceleration_{deceleration},
    execution_delay_{execution_delay},
    min_obstacle_height_{min_obstacle_height},
    max_obstacle_height_{max_obstacle_height},
    ignore_inside_footprint_{ignore_inside_footprint}
  {
  }

  [[nodiscard]] float deceleration() const { return deceleration_; }
  [[nodiscard]] float executionDelay() const { return execution_delay_; }

  [[nodiscard]] float minObstacleHeight() const { return min_obstacle_height_; }
  [[nodiscard]] float maxObstacleHeight() const { return max_obstacle_height_; }

  [[nodiscard]] bool ignoreInsideFootrpint() const { return ignore_inside_footprint_; }

private:
  float deceleration_;
  float execution_delay_;

  float min_obstacle_height_;
  float max_obstacle_height_;

  bool ignore_inside_footprint_;
};

}  // namespace collision_restraint
