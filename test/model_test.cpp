
#include "collision_restraint/model.hpp"

#include <catch_ros2/catch_ros2.hpp>

#include "collision_restraint/footprint.hpp"

using namespace collision_restraint;  // NOLINT

TEST_CASE("constructor", "model") { CHECK_NOTHROW(Model(Footprint(0.0F, 0.0F, 0.0F))); }
