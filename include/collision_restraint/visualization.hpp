#pragma once

#include <string>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "collision_restraint/footprint.hpp"

namespace collision_restraint
{

class Visualization
{
public:
  Visualization(const std::string & frame, Footprint footprint);

  [[nodiscard]] visualization_msgs::msg::Marker trajectoryMarker(
    const float linear_velocity, const float angular_velocity, const float stopping_distance) const;

  [[nodiscard]] visualization_msgs::msg::Marker pointMarker(const float x, const float y) const;

private:
  [[nodiscard]] visualization_msgs::msg::Marker straightLineMarker(
    const float stopping_distance) const;
  [[nodiscard]] visualization_msgs::msg::Marker baseTrajectoryMarker() const;

  std::string frame_;
  Footprint footprint_;
};

}  // namespace collision_restraint
