#pragma once

namespace collision_restraint
{

class Footprint
{
public:
  Footprint(
    const float length_front, const float length_back, const float width,
    const float buffer = 0.0f);

  float offsetFront() const;
  float offsetBack() const;
  float halfWidth() const;

private:
  float offset_front_;
  float offset_back_;
  float half_width_;
};

}  // namespace collision_restraint
