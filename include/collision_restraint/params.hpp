#pragma once

namespace collision_restraint
{
struct Params
{
  float deceleration_;
  float execution_delay_;

  float min_obstacle_height_;
  float max_obstacle_height_;

  bool ignore_inside_footprint_;
  float distance_buffer_;
};

}  // namespace collision_restraint
