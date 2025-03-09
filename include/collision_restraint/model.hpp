#pragma once

#include "collision_restraint/footprint.hpp"
#include "collision_restraint/polar_point.hpp"

namespace collision_restraint
{

class Model
{
public:
  Model(const Footprint & footprint);

  void setVelocities(const float linear, const float angular);

  [[nodiscard]] bool isStraight() const;
  [[nodiscard]] bool isLeftTurn() const;

  [[nodiscard]] float innerRadius() const;
  [[nodiscard]] float centerRadius() const;
  [[nodiscard]] float outerRadius() const;

  [[nodiscard]] float distance(const float x, const float y) const;

private:
  [[nodiscard]] bool insideFootprint(const float x, const float y) const;
  [[nodiscard]] float straightDistance(const float x, const float y) const;
  [[nodiscard]] float angularDistance(const PolarPoint & point) const;

  Footprint footprint_;

  float velocity_linear_;
  float velocity_angular_;

  bool straight_;
  bool left_turn_;

  float inner_radius_;
  float center_radius_;
  float outer_radius_;
};

}  // namespace collision_restraint
