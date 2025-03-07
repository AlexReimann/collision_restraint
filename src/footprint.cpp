#include "collision_restraint/footprint.hpp"

#include <cmath>
#include <stdexcept>

#include "collision_restraint/utility.hpp"

namespace collision_restraint
{
Footprint::Footprint(
  const float length_front, const float length_back, const float width, const float buffer)
: offset_front_{length_front + buffer},
  offset_back_{-(length_back + buffer)},
  half_width_{0.5f * width}
{
  if (
    !std::isfinite(offset_front_) || !std::isfinite(offset_back_) || !std::isfinite(half_width_)) {
    throw std::runtime_error(
      source_prefix() + "Non-finite footprint: " + std::to_string(offset_front_) + ", " +
      std::to_string(offset_back_) + "; " + std::to_string(half_width_));
  }
}

float Footprint::offsetFront() const { return offset_front_; }
float Footprint::offsetBack() const { return offset_back_; }
float Footprint::halfWidth() const { return half_width_; }

}  // namespace collision_restraint
