#include "collision_restraint/model.hpp"

#include <cmath>
#include <format>
#include <stdexcept>

#include "collision_restraint/point.hpp"
#include "collision_restraint/utility.hpp"

namespace collision_restraint
{

Model::Model(const Footprint & footprint) : footprint_{footprint} {}

void Model::setVelocities(const float linear, const float angular)
{
  velocity_linear_ = linear;
  velocity_angular_ = angular;
}

float Model::get(const float x, const float y) const
{
  Point point{x, y};

  return 0.0F;
};

}  // namespace collision_restraint
