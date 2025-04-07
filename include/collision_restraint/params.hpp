#pragma once

namespace collision_restraint
{

inline constexpr float g_straight_threshold = 0.001F;

struct Params
{
  float deceleration_;

  float min_obstacle_height_;
  float max_obstacle_height_;

  bool ignore_inside_footprint_;
  float distance_buffer_;
};

}  // namespace collision_restraint
