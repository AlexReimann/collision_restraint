#include "collision_restraint/model.hpp"

#include <cmath>

#include "collision_restraint/polar_point.hpp"
#include "collision_restraint/utility.hpp"

namespace collision_restraint
{

Model::Model(const Footprint & footprint) : footprint_{footprint} {}

void Model::setVelocities(const float linear, const float angular)
{
  velocity_linear_ = linear;
  velocity_angular_ = angular;

  constexpr float straight_threshold = 0.001F;
  straight_ = std::abs(velocity_angular_) <= straight_threshold;
  left_turn_ = velocity_angular_ >= 0.0F;

  if (straight_) {
    inner_radius_ = std::numeric_limits<float>::infinity();
    center_radius_ = std::numeric_limits<float>::infinity();
    outer_radius_ = std::numeric_limits<float>::infinity();
    return;
  }

  center_radius_ = std::abs(velocity_linear_ / velocity_angular_);
  inner_radius_ = center_radius_ - footprint_.halfWidth();

  const float max_offset =
    std::max(std::abs(footprint_.offsetFront()), std::abs(footprint_.offsetBack()));
  const float radius_perpendicular = center_radius_ + footprint_.halfWidth();
  outer_radius_ =
    std::sqrt((max_offset * max_offset) + (radius_perpendicular * radius_perpendicular));
}

bool Model::isStraight() const { return straight_; }
bool Model::isLeftTurn() const { return left_turn_; }

float Model::innerRadius() const { return inner_radius_; }
float Model::centerRadius() const { return center_radius_; }
float Model::outerRadius() const { return outer_radius_; }

float Model::distance(const float x, const float y) const
{
  PolarPoint point{x, y};

  if (insideFootprint(point.x(), point.y())) {
    return 0.0F;
  }

  if (velocity_linear_ == 0.0F) {
    return std::numeric_limits<float>::infinity();
  }

  if (straight_) {
    return straightDistance(point.x(), point.y());
  }

  return angularDistance(point);
}

bool Model::insideFootprint(const float x, const float y) const
{
  if (std::abs(y) > footprint_.halfWidth()) {
    return false;
  }

  if (x > footprint_.offsetFront() || x < footprint_.offsetBack()) {
    return false;
  }

  return true;
}

float Model::straightDistance(const float x, const float y) const
{
  if (std::abs(y) > footprint_.halfWidth()) {
    return std::numeric_limits<float>::infinity();
  }

  if (velocity_linear_ < 0.0F) {
    if (x < footprint_.offsetBack()) {
      return std::abs(x - footprint_.offsetBack());
    }
    // inside footprint already checked by insideFootprint()
    return std::numeric_limits<float>::infinity();
  }

  if (velocity_linear_ > 0.0F) {
    if (x > footprint_.offsetFront()) {
      return std::abs(x - footprint_.offsetFront());
    }
    // inside footprint already checked by insideFootprint()
    return std::numeric_limits<float>::infinity();
  }

  return 0.0F;
}

float Model::angularDistance(const PolarPoint & point) const
{
  // Use line in polar coordinates function to calculate angular point distance
  // Check if actually inside circles
  return point.theta();
}

}  // namespace collision_restraint
