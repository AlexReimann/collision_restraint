
#include "collision_restraint/model.hpp"

#include <catch_ros2/catch_ros2.hpp>
#include <cmath>
#include <limits>

#include "collision_restraint/footprint.hpp"

using namespace collision_restraint;  // NOLINT

TEST_CASE("constructor", "[model]") { CHECK_NOTHROW(Model(Footprint(0.0F, 0.0F, 0.0F))); }

TEST_CASE("setVelocities", "[model]")
{
  constexpr float front_offset = 1.3F;
  constexpr float back_offset = 0.4F;
  constexpr float half_width = front_offset - 1.0F;
  collision_restraint::Model model{Footprint(front_offset, back_offset, 2.0F * half_width)};

  SECTION("straight")
  {
    model.setVelocities(1.0F, 0.0F);
    CHECK(model.isStraight());
    CHECK(model.innerRadius() == std::numeric_limits<float>::infinity());
    CHECK(model.centerRadius() == std::numeric_limits<float>::infinity());
    CHECK(model.outerRadius() == std::numeric_limits<float>::infinity());

    model.setVelocities(0.0F, std::nextafterf(0.0F, 1.0F));
    CHECK(model.isStraight());
    CHECK(model.innerRadius() == std::numeric_limits<float>::infinity());
    CHECK(model.centerRadius() == std::numeric_limits<float>::infinity());
    CHECK(model.outerRadius() == std::numeric_limits<float>::infinity());

    model.setVelocities(0.0F, std::nextafterf(0.0F, -1.0F));
    CHECK(model.isStraight());
    CHECK(model.innerRadius() == std::numeric_limits<float>::infinity());
    CHECK(model.centerRadius() == std::numeric_limits<float>::infinity());
    CHECK(model.outerRadius() == std::numeric_limits<float>::infinity());
  }

  SECTION("left turn")
  {
    model.setVelocities(1.0F, 1.0F);
    CHECK(model.isLeftTurn());
    CHECK(model.innerRadius() == (1.0F - half_width));
    CHECK(model.centerRadius() == 1.0F);
    CHECK(model.outerRadius() == Catch::Approx(std::sqrt(2.0) * front_offset));
  }

  SECTION("right turn")
  {
    model.setVelocities(1.0F, -1.0F);
    CHECK_FALSE(model.isLeftTurn());
    CHECK(model.innerRadius() == (1.0F - half_width));
    CHECK(model.centerRadius() == 1.0F);
    CHECK(model.outerRadius() == Catch::Approx(std::sqrt(2.0) * front_offset));
  }

  SECTION("zero linear velocity")
  {
    model.setVelocities(0.0F, 1.0F);
    CHECK_FALSE(model.isStraight());
    CHECK(model.innerRadius() == 0.0F);
    CHECK(model.centerRadius() == 0.0F);
    CHECK(
      model.outerRadius() == std::sqrt((front_offset * front_offset) + (half_width * half_width)));
  }

  SECTION("back offset > front offset")
  {
    collision_restraint::Model long_back{Footprint(back_offset, front_offset, 2.0F * half_width)};
    long_back.setVelocities(1.0F, 1.0F);
    CHECK(long_back.innerRadius() == (1.0F - half_width));
    CHECK(long_back.centerRadius() == 1.0F);
    CHECK(long_back.outerRadius() == Catch::Approx(std::sqrt(2.0) * front_offset));

    long_back.setVelocities(0.0F, 1.0F);
    CHECK(
      long_back.outerRadius() ==
      std::sqrt((front_offset * front_offset) + (half_width * half_width)));
  }
}

TEST_CASE("distance_straight", "[model]")
{
  constexpr float front_offset = 1.3F;
  constexpr float back_offset = 0.4F;
  constexpr float half_width = front_offset - 1.0F;
  collision_restraint::Model model{Footprint(front_offset, back_offset, 2.0F * half_width)};

  model.setVelocities(1.0F, 0.0F);
  REQUIRE(model.isStraight());

  SECTION("inside footprint")
  {
    CHECK(model.arcDistance(0.0F, 0.0F) == 0.0F);
    CHECK(model.arcDistance(0.0F, half_width) == 0.0F);
    CHECK(model.arcDistance(0.0F, -half_width) == 0.0F);
    CHECK(model.arcDistance(front_offset, 0.0F) == 0.0F);
    CHECK(model.arcDistance(-back_offset, 0.0F) == 0.0F);

    CHECK(model.arcDistance(0.5F * front_offset, 0.5F * half_width) == 0.0F);
  }

  SECTION("forwards")
  {
    model.setVelocities(1.0F, 0.0F);
    CHECK(model.arcDistance(front_offset + 1.0F, 0.0F) == 1.0F);
    CHECK(model.arcDistance(2.0F * front_offset, 0.0F) == front_offset);

    CHECK(model.arcDistance(front_offset + 1.1F, 0.5F * half_width) == Catch::Approx(1.1F));
    CHECK(model.arcDistance(front_offset + 1.1F, -0.5F * half_width) == Catch::Approx(1.1F));
    CHECK(model.arcDistance(front_offset + 1.1F, half_width) == Catch::Approx(1.1F));
    CHECK(model.arcDistance(front_offset + 1.1F, half_width) == Catch::Approx(1.1F));

    CHECK(model.arcDistance(-2.0F * back_offset, 0.0F) == std::numeric_limits<float>::infinity());
  }

  SECTION("backwards")
  {
    model.setVelocities(-1.0F, 0.0F);
    CHECK(model.arcDistance(-back_offset - 1.0F, 0.0F) == 1.0F);
    CHECK(model.arcDistance(-2.0F * back_offset, 0.0F) == back_offset);

    CHECK(model.arcDistance(-back_offset - 1.1F, 0.5F * half_width) == Catch::Approx(1.1F));
    CHECK(model.arcDistance(-back_offset - 1.1F, -0.5F * half_width) == Catch::Approx(1.1F));
    CHECK(model.arcDistance(-back_offset - 1.1F, half_width) == Catch::Approx(1.1F));
    CHECK(model.arcDistance(-back_offset - 1.1F, half_width) == Catch::Approx(1.1F));

    CHECK(model.arcDistance(2.0F * front_offset, 0.0F) == std::numeric_limits<float>::infinity());
  }
}

TEST_CASE("distance_angular_forwards", "[model]")
{
  constexpr float front_offset = 0.5F;
  constexpr float back_offset = 0.3F;
  constexpr float half_width = 0.2F;
  collision_restraint::Model model{Footprint(front_offset, back_offset, 2.0F * half_width)};

  constexpr float turn_radius = 1.0F;
  constexpr float eps = 0.0000001;

  SECTION("left_turn")
  {
    model.setVelocities(1.0F, 1.0F);
    REQUIRE(!model.isStraight());

    SECTION("not_in_path")
    {
      CHECK(model.arcDistance(0.0F, 2.0F * half_width) == std::numeric_limits<float>::infinity());
      CHECK(model.arcDistance(0.0F, (half_width + eps)) == std::numeric_limits<float>::infinity());

      const float radius = half_width + turn_radius;
      const float outer_point =
        std::sqrt((radius * radius) + (front_offset * front_offset)) - turn_radius;
      CHECK(
        model.arcDistance(0.0F, -(outer_point + eps)) == std::numeric_limits<float>::infinity());

      CHECK(
        model.arcDistance(0.0F, -(half_width + front_offset)) ==
        std::numeric_limits<float>::infinity());
    }

    SECTION("left_turn_front")
    {
      // 90 degree offsets
      CHECK(model.arcDistance(turn_radius, turn_radius + front_offset) == Catch::Approx(M_PI_2));
      CHECK(model.arcDistance(-front_offset, 2.0F * turn_radius) == Catch::Approx(M_PI));
      CHECK(
        model.arcDistance(-turn_radius, -(front_offset - turn_radius)) ==
        Catch::Approx(M_PI + M_PI_2));

      // directly in front
      CHECK_THAT(model.arcDistance(front_offset + eps, 0.0F), Catch::Matchers::WithinAbs(eps, eps));

      // just behind
      CHECK(model.arcDistance(-(back_offset + eps), 0.0F) > 4.0F);
    }

    SECTION("left_turn_side")
    {
      // 90 degree offsets
      CHECK_THAT(
        model.arcDistance(turn_radius - half_width + eps, turn_radius),
        Catch::Matchers::WithinRel(static_cast<float>(M_PI_2), 0.01F));
      CHECK_THAT(
        model.arcDistance(0.0, (turn_radius - half_width) + turn_radius + eps),
        Catch::Matchers::WithinRel(static_cast<float>(M_PI), 0.01F));
      CHECK_THAT(
        model.arcDistance(-(turn_radius - half_width + eps), turn_radius),
        Catch::Matchers::WithinRel(static_cast<float>(M_PI + M_PI_2), 0.01F));

      // directly next to it
      CHECK_THAT(
        model.arcDistance(front_offset, half_width + eps),
        Catch::Matchers::WithinAbs(eps, eps * 10.0F));
    }
  }

  SECTION("right_turn")
  {
    model.setVelocities(1.0F, -1.0F);
    REQUIRE(!model.isStraight());

    SECTION("not_in_path")
    {
      CHECK(
        model.arcDistance(0.0F, -(2.0F * half_width)) == std::numeric_limits<float>::infinity());
      CHECK(model.arcDistance(0.0F, -(half_width + eps)) == std::numeric_limits<float>::infinity());

      const float radius = half_width + turn_radius;
      const float outer_point =
        std::sqrt((radius * radius) + (front_offset * front_offset)) - turn_radius;
      CHECK(model.arcDistance(0.0F, outer_point + eps) == std::numeric_limits<float>::infinity());

      CHECK(
        model.arcDistance(0.0F, half_width + front_offset) ==
        std::numeric_limits<float>::infinity());
    }

    SECTION("right_turn_front")
    {
      // 90 degree offsets
      CHECK(model.arcDistance(turn_radius, -(turn_radius + front_offset)) == Catch::Approx(M_PI_2));
      CHECK(model.arcDistance(-front_offset, -2.0F * turn_radius) == Catch::Approx(M_PI));
      CHECK(
        model.arcDistance(-turn_radius, front_offset - turn_radius) ==
        Catch::Approx(M_PI + M_PI_2));

      // directly in front
      CHECK_THAT(model.arcDistance(front_offset + eps, 0.0F), Catch::Matchers::WithinAbs(eps, eps));

      // just behind
      CHECK(model.arcDistance(-(back_offset + eps), 0.0F) > 4.0F);
    }

    SECTION("right_turn_side")
    {
      // 90 degree offsets
      CHECK_THAT(
        model.arcDistance(turn_radius - half_width + eps, -turn_radius),
        Catch::Matchers::WithinRel(static_cast<float>(M_PI_2), 0.01F));
      CHECK_THAT(
        model.arcDistance(0.0, -((turn_radius - half_width) + turn_radius + eps)),
        Catch::Matchers::WithinRel(static_cast<float>(M_PI), 0.01F));
      CHECK_THAT(
        model.arcDistance(-(turn_radius - half_width + eps), -turn_radius),
        Catch::Matchers::WithinRel(static_cast<float>(M_PI + M_PI_2), 0.01F));

      // directly next to it
      CHECK_THAT(
        model.arcDistance(front_offset, -(half_width + eps)),
        Catch::Matchers::WithinAbs(eps, eps * 10.0F));
    }
  }
}
