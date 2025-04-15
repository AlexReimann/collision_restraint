#include "collision_restraint/distance_model.hpp"

#include <angles/angles.h>

#include <cmath>
#include <stdexcept>

#include "collision_restraint/polar_point.hpp"
#include "collision_restraint/utility.hpp"

namespace collision_restraint
{

DistanceModel::DistanceModel(Footprint footprint, const std::shared_ptr<const Params> & params)
: params_{params}, footprint_{std::move(footprint)}
{
  if (footprint_.offsetFront() < 0.0F) {
    throw std::range_error(
      source_prefix() + "Angular distance calculation only supports positive front offset");
  }
}

void DistanceModel::setVelocities(const float linear, const float angular)
{
  velocity_linear_ = linear;
  velocity_angular_ = angular;

  straight_ = std::abs(velocity_angular_) <= g_straight_threshold;
  left_turn_ = velocity_angular_ >= 0.0F;

  if (straight_) {
    radii_.inner_ = std::numeric_limits<float>::infinity();
    radii_.center_ = std::numeric_limits<float>::infinity();
    radii_.outer_ = std::numeric_limits<float>::infinity();
    return;
  }

  radii_.center_ = std::abs(velocity_linear_ / velocity_angular_);
  radii_.inner_ = std::max(0.0F, radii_.center_ - footprint_.halfWidth());

  const float max_offset =
    std::max(std::abs(footprint_.offsetFront()), std::abs(footprint_.offsetBack()));
  const float radius_perpendicular = radii_.center_ + footprint_.halfWidth();
  radii_.outer_ =
    std::sqrt((max_offset * max_offset) + (radius_perpendicular * radius_perpendicular));

  const float left_offset =  footprint_.halfWidth() - radii_.center_;
  const float right_offset = -footprint_.halfWidth() - radii_.center_;
  front_ = PolarAxisLine(footprint_.offsetFront(), left_offset, right_offset, true);
  back_ = PolarAxisLine(footprint_.offsetBack(), left_offset, right_offset, true);

  left_ = PolarAxisLine(
    -(radii_.center_ - footprint_.halfWidth()), footprint_.offsetFront(), footprint_.offsetBack(),
    false);
  right_ = PolarAxisLine(
    -(radii_.center_ + footprint_.halfWidth()), footprint_.offsetFront(), footprint_.offsetBack(),
    false);
}

bool DistanceModel::isStraight() const { return straight_; }
bool DistanceModel::isLeftTurn() const { return left_turn_; }
Radii DistanceModel::radii() const { return radii_; }

float DistanceModel::angularDistance(const float x, const float y) const
{
  PolarPoint point{x, y};

  if (!params_->ignore_inside_footprint_ && insideFootprint(point.x(), point.y())) {
    return 0.0F;
  }

  if (velocity_linear_ == 0.0F && velocity_angular_ == 0.0F) {
    return std::numeric_limits<float>::infinity();
  }

  if (velocity_linear_ != 0.0F && straight_) {
    return straightDistance(point.x(), point.y());
  }

  return angularDistance(point);
}

bool DistanceModel::insideFootprint(const float x, const float y) const
{
  if (std::abs(y) > footprint_.halfWidth()) {
    return false;
  }

  if (x > footprint_.offsetFront() || x < footprint_.offsetBack()) {
    return false;
  }

  return true;
}

float DistanceModel::straightDistance(const float x, const float y) const
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

float DistanceModel::angularDistance(const PolarPoint & point_base_link) const
{
  const PolarPoint point = turnTransform(point_base_link);

  if (point.r() < radii_.inner_ || point.r() > radii_.outer_) {
    return std::numeric_limits<float>::infinity();
  }

  const float front_distance = front_->distance(point.r(), point.theta(), true);

  // Don't need to check back for radii_.inner_ > 0.0F
  const float back_distance = radii_.inner_ == 0.0F
                                ? back_->distance(point.r(), point.theta(), false)
                                : std::numeric_limits<float>::infinity();

  // theta flips when crossing the x-axis
  const float left_distance = left_->distance(point.r(), point.theta(), left_->m() >= 0.0F);

  // The right side will only hit in case of the point being directly next to the robot.
  // (In any other case the point will be hit first by the front or left side)
  // In this case the right side is swinging out, meaning only the part farther back can hit.
  // Thus we check the distance with the lower (== min) theta
  const float right_distance = right_->distance(point.r(), point.theta(), true);

  // return right_distance;
  return std::min({front_distance, left_distance, right_distance, back_distance});
}

PolarPoint DistanceModel::turnTransform(const PolarPoint & base_link_point) const
{
  if (straight_) {
    return base_link_point;
  }

  // transform into rotation center frame
  const float y_turn_adjusted = left_turn_ ? base_link_point.y() : -base_link_point.y();
  return {base_link_point.x(), y_turn_adjusted - radii_.center_};
}

}  // namespace collision_restraint
