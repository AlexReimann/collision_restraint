#include "collision_restraint/model.hpp"

#include <angles/angles.h>

#include <cmath>

#include "collision_restraint/polar_axis_line.hpp"
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
  inner_radius_ = std::max(0.0F, center_radius_ - footprint_.halfWidth());

  const float max_offset =
    std::max(std::abs(footprint_.offsetFront()), std::abs(footprint_.offsetBack()));
  const float radius_perpendicular = center_radius_ + footprint_.halfWidth();
  outer_radius_ =
    std::sqrt((max_offset * max_offset) + (radius_perpendicular * radius_perpendicular));

  const float corner_offset =
    velocity_linear_ > 0.0F ? footprint_.offsetFront() : footprint_.offsetBack();
  corner_radius_ = std::sqrt((inner_radius_ * inner_radius_) + (corner_offset * corner_offset));
}

bool Model::isStraight() const { return straight_; }
bool Model::isLeftTurn() const { return left_turn_; }

float Model::innerRadius() const { return inner_radius_; }
float Model::centerRadius() const { return center_radius_; }
float Model::outerRadius() const { return outer_radius_; }

float Model::arcDistance(const float x, const float y) const
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

float Model::angularDistance(const PolarPoint & point_base_link) const
{
  // handle on-spot rotation

  // transform into rotation center frame
  const float y_turn_adjusted = left_turn_ ? point_base_link.y() : -point_base_link.y();
  const PolarPoint point{point_base_link.x(), y_turn_adjusted - center_radius_};

  if (point.r() < inner_radius_ || point.r() > outer_radius_) {
    return std::numeric_limits<float>::infinity();
  }

  // rotation center is always along base_link y-axis
  // footprint is mirrored along x-axis
  // m = ax + by
  const PolarAxisLine front{footprint_.offsetFront(), true};
  const PolarAxisLine side{-(center_radius_ - footprint_.halfWidth()), false};

  // Going forwards, either the front or the side can hit

  if (point.r() > corner_radius_) {
    // front will hit
    return angles::normalize_angle_positive(point.theta() - front.min_theta(point.r()));
  }

  // side will hit
  return angles::normalize_angle_positive(point.theta() - side.max_theta(point.r()));

  // handle backwards case
}

}  // namespace collision_restraint
