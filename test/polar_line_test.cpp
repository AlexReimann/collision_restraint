
#include "collision_restraint/polar_line.hpp"

#include <catch_ros2/catch_ros2.hpp>
#include <cmath>
#include <limits>

#include "collision_restraint/footprint.hpp"

using namespace collision_restraint;  // NOLINT

TEST_CASE("constructor", "[polar_line]")
{
  CHECK_NOTHROW(PolarLine(0.0F, 1.0F, 0.0F));
  CHECK_THROWS(PolarLine(std::numeric_limits<float>::infinity(), 1.0F, 0.0F));
  CHECK_THROWS(PolarLine(0.0F, std::numeric_limits<float>::infinity(), 1.0F));
  CHECK_THROWS(PolarLine(0.0F, 1.0F, std::numeric_limits<float>::infinity()));
}

TEST_CASE("radius", "[polar_line]")
{
  // ros coordinates -> x: forward, y: left

  SECTION("x_axis")
  {
    PolarLine x_axis{0.0F, 1.0F, 0.0F};

    CHECK(x_axis.r(0.0F) == 0.0F);
    CHECK(x_axis.r(M_PI) == 0.0F);
    CHECK(x_axis.r(1.0F) == 0.0F);
    CHECK(x_axis.r(-1.0F) == 0.0F);

    CHECK(x_axis.r(M_PI_2) == std::numeric_limits<float>::infinity());
    CHECK(x_axis.r(-M_PI_2) == std::numeric_limits<float>::infinity());
  }

  SECTION("y_axis")
  {
    PolarLine y_axis{0.0F, 0.0F, 1.0F};

    CHECK(y_axis.r(M_PI_2) == 0.0F);
    CHECK(y_axis.r(-M_PI_2) == 0.0F);
    CHECK(y_axis.r(1.0F) == 0.0F);
    CHECK(y_axis.r(-1.0F) == 0.0F);

    CHECK(y_axis.r(0.0) == std::numeric_limits<float>::infinity());
    CHECK(y_axis.r(M_PI) == std::numeric_limits<float>::infinity());
  }

  SECTION("diagonal")
  {
    PolarLine diag_pos{0.0F, 1.0F, 1.0F};

    CHECK(diag_pos.r(0.0F) == 0.0F);
    CHECK(diag_pos.r(M_PI) == 0.0F);
    CHECK(diag_pos.r(1.0F) == 0.0F);
    CHECK(diag_pos.r(-1.0F) == 0.0F);

    CHECK(diag_pos.r(M_PI_4) == std::numeric_limits<float>::infinity());
    CHECK(diag_pos.r(-M_PI + M_PI_4) == std::numeric_limits<float>::infinity());

    PolarLine diag_neg{0.0F, 1.0F, -1.0F};

    CHECK(diag_neg.r(0.0F) == 0.0F);
    CHECK(diag_neg.r(M_PI) == 0.0F);
    CHECK(diag_neg.r(1.0F) == 0.0F);
    CHECK(diag_neg.r(-1.0F) == 0.0F);

    CHECK(diag_neg.r(-M_PI_4) == std::numeric_limits<float>::infinity());
    CHECK(diag_neg.r(M_PI - M_PI_4) == std::numeric_limits<float>::infinity());
  }

  SECTION("horizontal")
  {
    PolarLine hor{1.0F, 1.0F, 0.0F};

    CHECK(hor.r(0.0F) == 1.0F);
    CHECK(hor.r(M_PI_4) == std::sqrt(2.0F));
    CHECK(hor.r(-M_PI_4) == std::sqrt(2.0F));

    CHECK(hor.r(M_PI_2) == std::numeric_limits<float>::infinity());
    CHECK(hor.r(-M_PI_2) == std::numeric_limits<float>::infinity());

    CHECK(hor.r(2.0) == std::numeric_limits<float>::infinity());
    CHECK(hor.r(-2.0) == std::numeric_limits<float>::infinity());
  }

  SECTION("vertical")
  {
    PolarLine ver{1.0F, 0.0F, 1.0F};

    CHECK(ver.r(M_PI_2) == 1.0F);
    CHECK(ver.r(M_PI_4) == std::sqrt(2.0F));
    CHECK(ver.r(M_PI_2 + M_PI_4) == std::sqrt(2.0F));

    CHECK(ver.r(0.0F) == std::numeric_limits<float>::infinity());
    CHECK(ver.r(M_PI) == std::numeric_limits<float>::infinity());

    CHECK(ver.r(-1.0) == std::numeric_limits<float>::infinity());
    CHECK(ver.r(-2.0) == std::numeric_limits<float>::infinity());
  }

  SECTION("arbitrary")
  {
    PolarLine diag{1.0F, 1.0F, 1.0F};

    CHECK(diag.r(0.0F) == Catch::Approx(1.0F));
    CHECK(diag.r(M_PI_2) == Catch::Approx(1.0F));
    CHECK(diag.r(M_PI_4) == Catch::Approx(std::sqrt(0.5F)));

    CHECK(diag.r(-M_PI_4) == std::numeric_limits<float>::infinity());
    CHECK(diag.r(M_PI_2 + M_PI_4) == std::numeric_limits<float>::infinity());

    CHECK(diag.r(3.0F) == std::numeric_limits<float>::infinity());
    CHECK(diag.r(-1.0F) == std::numeric_limits<float>::infinity());
  }
}
