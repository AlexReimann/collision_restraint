#include "collision_restraint/collision_restraint.hpp"

#include <limits>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace collision_restraint
{

CollisionRestraint::CollisionRestraint(Footprint footprint, const std::shared_ptr<Params> & params)
: params_{params}, distance_{footprint}, motion_{params->deceleration(), params->executionDelay()}
{
}

std::tuple<bool, Velocities, PolarPoint> CollisionRestraint::restrain(
  const Velocities & velocities, const sensor_msgs::msg::PointCloud2 & point_cloud) const
{
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(point_cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(point_cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(point_cloud, "z");

  float min_arc_distance = std::numeric_limits<float>::infinity();
  float x = 0.0F;
  float y = 0.0F;

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (*iter_z < params_->minObstacleHeight() || *iter_z > params_->maxObstacleHeight()) {
      continue;
    }

    const float arc_distance = distance_.arcDistance(*iter_x, *iter_y);

    if (arc_distance < min_arc_distance) {
      min_arc_distance = arc_distance;
      x = *iter_x;
      y = *iter_y;
    }
  }

  const PolarPoint closest_point{x, y};
  const float min_distance = PolarPoint::arcToEucDistance(min_arc_distance, closest_point.r());
  const float stop_distance = motion_.stoppingDistance(velocities.linear_);

  if (min_distance > stop_distance) {
    return {false, velocities, closest_point};
  }

  return {true, motion_.scaleToStopDistance(velocities, min_distance), closest_point};
}

}  // namespace collision_restraint
