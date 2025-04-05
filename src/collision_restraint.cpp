#include "collision_restraint/collision_restraint.hpp"

#include <limits>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace collision_restraint
{

CollisionRestraint::CollisionRestraint(
  Footprint footprint, const std::shared_ptr<const Params> & params)
: params_{params},
  distance_{footprint, params},
  motion_{params->deceleration_, params->execution_delay_}
{
}

std::tuple<bool, Velocities, PolarPoint, float, Radii> CollisionRestraint::restrain(
  const Velocities & velocities, const sensor_msgs::msg::PointCloud2 & point_cloud)
{
  distance_.setVelocities(velocities.linear_, velocities.angular_);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(point_cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(point_cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(point_cloud, "z");

  float min_angular_distance = std::numeric_limits<float>::infinity();
  float x = 0.0F;
  float y = 0.0F;

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (*iter_z < params_->min_obstacle_height_ || *iter_z > params_->max_obstacle_height_) {
      continue;
    }

    const float angular_distance = distance_.angularDistance(*iter_x, *iter_y);

    if (angular_distance < min_angular_distance) {
      min_angular_distance = angular_distance;
      x = *iter_x;
      y = *iter_y;
    }
  }

  const float stop_distance = motion_.stoppingDistance(velocities.linear_);
  if (std::isinf(min_angular_distance)) {
    return {false, velocities, PolarPoint(0.0F, 0.0F), stop_distance, distance_.radii()};
  }

  const PolarPoint closest_point{x, y};
  const float distance =
    (distance_.isStraight()
       ? min_angular_distance
       : PolarPoint::angularToEucDistance(min_angular_distance, closest_point.r()));

  const float min_distance = distance - params_->distance_buffer_;
  if (min_distance > stop_distance) {
    return {false, velocities, closest_point, stop_distance, distance_.radii()};
  }

  return {
    true, motion_.scaleToStopDistance(velocities, min_distance), closest_point, distance,
    distance_.radii()};
}

}  // namespace collision_restraint
