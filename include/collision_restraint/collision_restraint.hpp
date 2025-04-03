#pragma once

#include <memory>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tuple>

#include "collision_restraint/distance_model.hpp"
#include "collision_restraint/footprint.hpp"
#include "collision_restraint/motion_model.hpp"
#include "collision_restraint/params.hpp"
#include "collision_restraint/velocities.hpp"
#include "collision_restraint/visibility_control.h"

namespace collision_restraint
{

class CollisionRestraint
{
public:
  CollisionRestraint(Footprint footprint, const std::shared_ptr<const Params> & params);

  [[nodiscard]] std::tuple<bool, Velocities, PolarPoint, float> restrain(
    const Velocities & velocities, const sensor_msgs::msg::PointCloud2 & point_cloud) const;

private:
  std::shared_ptr<const Params> params_;

  DistanceModel distance_;
  MotionModel motion_;
};

}  // namespace collision_restraint
