
#include "collision_restraint/motion_model.hpp"

#include <catch_ros2/catch_ros2.hpp>
#include <cmath>
#include <limits>

#include "collision_restraint/footprint.hpp"

using namespace collision_restraint;  // NOLINT

TEST_CASE("constructor", "[motion_model]") { CHECK_NOTHROW(MotionModel(0.0F, 1.0F)); }

TEST_CASE("stoppingDistance", "[motion_model]")
{
  CHECK(MotionModel(1.0F).stoppingDistance(1.0F) == Catch::Approx(0.5F));
  CHECK(MotionModel(1.0F, 1.0F).stoppingDistance(1.0F) == Catch::Approx(1.5F));

  CHECK(MotionModel(2.0F, 0.5F).stoppingDistance(1.0F) == Catch::Approx(0.75F));
}

TEST_CASE("scaleToStopDistance", "[motion_model]")
{
  SECTION("result_zero")
  {
    MotionModel model(2.0F);
    CHECK(model.scaleToStopDistance({3.0F, 0.0F}, 0.0F).linear_ == 0.0F);
    CHECK(model.scaleToStopDistance({3.0F, 0.0F}, 0.0F).angular_ == 0.0F);

    MotionModel model_delayed(1.0F, 0.5F);
    CHECK(model_delayed.scaleToStopDistance({3.0F, 0.0F}, 0.1F).linear_ == 0.0F);
    CHECK(model_delayed.scaleToStopDistance({3.0F, 0.0F}, 0.1F).angular_ == 0.0F);
  }

  SECTION("linear")
  {
    CHECK(MotionModel(1.0F).scaleToStopDistance({1.0F, 0.0F}, 0.5F).linear_ == 1.0F);
    CHECK(MotionModel(2.0F).scaleToStopDistance({1.0F, 0.0F}, 0.25F).linear_ == 1.0F);
    CHECK(MotionModel(2.0F).scaleToStopDistance({2.0F, 0.0F}, 0.25F).linear_ == 1.0F);

    CHECK(MotionModel(2.0F, 0.5F).scaleToStopDistance({2.0F, 0.0F}, 1.25F).linear_ == 1.0F);
  }

  SECTION("angular")
  {
    CHECK(MotionModel(1.0F).scaleToStopDistance({1.0F, 1.0F}, 0.5F).linear_ == 1.0F);
    CHECK(MotionModel(1.0F).scaleToStopDistance({2.0F, 1.0F}, 0.5F).angular_ == 0.5F);
  }
}
