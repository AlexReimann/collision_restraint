#pragma once

#include <limits>

namespace collision_restraint
{

struct Radii
{
  float inner_ = std::numeric_limits<float>::infinity();
  float center_ = std::numeric_limits<float>::infinity();
  float outer_ = std::numeric_limits<float>::infinity();
};

}  // namespace collision_restraint