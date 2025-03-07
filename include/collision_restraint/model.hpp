#pragma once

#include "collision_restraint/footprint.hpp"

namespace collision_restraint
{

class Model
{
public:
  Model(const Footprint & footprint);

  void setVelocities(const float linear, const float angular);

  [[nodiscard]] float get(const float x, const float y) const;

private:
  Footprint footprint_;

  float velocity_linear_;
  float velocity_angular_;
};

}  // namespace collision_restraint
