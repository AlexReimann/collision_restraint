
#include "collision_restraint/polar_axis_line.hpp"

#include <catch_ros2/catch_ros2.hpp>
#include <cmath>
#include <limits>

#include "collision_restraint/footprint.hpp"

using namespace collision_restraint;  // NOLINT

TEST_CASE("constructor", "[polar_axis_line]")
{
  CHECK_NOTHROW(PolarAxisLine(0.0F, true));
  CHECK_NOTHROW(PolarAxisLine(-1.0F, false));
  CHECK_THROWS(PolarAxisLine(std::numeric_limits<float>::infinity(), true));
}

TEST_CASE("radius", "[polar_axis_line]")
{
  // ros coordinates -> x: forward, y: left

  SECTION("x_axis")
  {
    PolarAxisLine x_axis{0.0F, true};

    CHECK(x_axis.r(0.0F) == 0.0F);
    CHECK(x_axis.r(M_PI) == 0.0F);
    CHECK(x_axis.r(1.0F) == 0.0F);
    CHECK(x_axis.r(-1.0F) == 0.0F);

    CHECK(x_axis.r(M_PI_2) == std::numeric_limits<float>::infinity());
    CHECK(x_axis.r(-M_PI_2) == std::numeric_limits<float>::infinity());
  }

  SECTION("y_axis")
  {
    PolarAxisLine y_axis{0.0F, false};

    CHECK(y_axis.r(M_PI_2) == 0.0F);
    CHECK(y_axis.r(-M_PI_2) == 0.0F);
    CHECK(y_axis.r(1.0F) == 0.0F);
    CHECK(y_axis.r(-1.0F) == 0.0F);

    CHECK(y_axis.r(0.0) == std::numeric_limits<float>::infinity());
    CHECK(y_axis.r(M_PI) == std::numeric_limits<float>::infinity());
  }

  SECTION("horizontal")
  {
    PolarAxisLine hor_pos{1.0F, true};

    CHECK(hor_pos.r(0.0F) == 1.0F);
    CHECK(hor_pos.r(M_PI_4) == std::sqrt(2.0F));
    CHECK(hor_pos.r(-M_PI_4) == std::sqrt(2.0F));

    CHECK(hor_pos.r(M_PI_2) == std::numeric_limits<float>::infinity());
    CHECK(hor_pos.r(-M_PI_2) == std::numeric_limits<float>::infinity());

    CHECK(hor_pos.r(2.0) == std::numeric_limits<float>::infinity());
    CHECK(hor_pos.r(-2.0) == std::numeric_limits<float>::infinity());

    PolarAxisLine hor_neg{-1.0F, true};

    CHECK(hor_neg.r(M_PI) == 1.0F);
    CHECK(hor_neg.r(M_PI_2 + M_PI_4) == std::sqrt(2.0F));
    CHECK(hor_neg.r(-(M_PI_2 + M_PI_4)) == std::sqrt(2.0F));

    CHECK(hor_neg.r(M_PI_2) == std::numeric_limits<float>::infinity());
    CHECK(hor_neg.r(-M_PI_2) == std::numeric_limits<float>::infinity());

    CHECK(hor_neg.r(1.0) == std::numeric_limits<float>::infinity());
    CHECK(hor_neg.r(-1.0) == std::numeric_limits<float>::infinity());
  }

  SECTION("vertical")
  {
    PolarAxisLine ver_pos{1.0F, false};

    CHECK(ver_pos.r(M_PI_2) == 1.0F);
    CHECK(ver_pos.r(M_PI_4) == std::sqrt(2.0F));
    CHECK(ver_pos.r(M_PI_2 + M_PI_4) == std::sqrt(2.0F));

    CHECK(ver_pos.r(0.0F) == std::numeric_limits<float>::infinity());
    CHECK(ver_pos.r(M_PI) == std::numeric_limits<float>::infinity());
    CHECK(ver_pos.r(-M_PI) == std::numeric_limits<float>::infinity());

    CHECK(ver_pos.r(-1.0) == std::numeric_limits<float>::infinity());
    CHECK(ver_pos.r(-2.0) == std::numeric_limits<float>::infinity());

    PolarAxisLine ver_neg{-1.0F, false};

    CHECK(ver_neg.r(-M_PI_2) == 1.0F);
    CHECK(ver_neg.r(-M_PI_4) == std::sqrt(2.0F));
    CHECK(ver_neg.r(-(M_PI_2 + M_PI_4)) == std::sqrt(2.0F));

    CHECK(ver_neg.r(0.0F) == std::numeric_limits<float>::infinity());
    CHECK(ver_neg.r(M_PI) == std::numeric_limits<float>::infinity());
    CHECK(ver_neg.r(-M_PI) == std::numeric_limits<float>::infinity());

    CHECK(ver_neg.r(1.0) == std::numeric_limits<float>::infinity());
    CHECK(ver_neg.r(2.0) == std::numeric_limits<float>::infinity());
  }
}

TEST_CASE("thetas", "[polar_axis_line]")
{
  // ros coordinates -> x: forward, y: left

  SECTION("x_axis")
  {
    PolarAxisLine x_axis{0.0F, true};

    CHECK(std::get<0>(x_axis.thetas(0.0F)) == Catch::Approx(M_PI_2));
    CHECK(std::isnan(std::get<1>(x_axis.thetas(0.0F))));
    CHECK(std::get<0>(x_axis.thetas(M_PI)) == Catch::Approx(M_PI_2));
    CHECK(std::isnan(std::get<1>(x_axis.thetas(M_PI))));

    CHECK(std::get<0>(x_axis.thetas(M_PI_2)) == Catch::Approx(M_PI_2));
    CHECK(std::get<0>(x_axis.thetas(-M_PI_2)) == Catch::Approx(M_PI_2));

    CHECK(std::get<0>(x_axis.thetas(1.0F)) == Catch::Approx(M_PI_2));
    CHECK(std::get<0>(x_axis.thetas(-1.0F)) == Catch::Approx(M_PI_2));
    CHECK(std::get<0>(x_axis.thetas(2.0F)) == Catch::Approx(M_PI_2));
    CHECK(std::get<0>(x_axis.thetas(-2.0F)) == Catch::Approx(M_PI_2));
  }

  SECTION("y_axis")
  {
    PolarAxisLine y_axis{0.0F, false};

    CHECK(std::get<0>(y_axis.thetas(0.0F)) == 0.0F);
    CHECK(std::isnan(std::get<1>(y_axis.thetas(0.0F))));
    CHECK(std::get<0>(y_axis.thetas(M_PI)) == 0.0F);
    CHECK(std::isnan(std::get<1>(y_axis.thetas(M_PI))));

    CHECK(std::get<0>(y_axis.thetas(M_PI_2)) == 0.0F);
    CHECK(std::get<0>(y_axis.thetas(-M_PI_2)) == 0.0F);

    CHECK(std::get<0>(y_axis.thetas(1.0F)) == 0.0F);
    CHECK(std::get<0>(y_axis.thetas(-1.0F)) == 0.0F);
    CHECK(std::get<0>(y_axis.thetas(2.0F)) == 0.0F);
    CHECK(std::get<0>(y_axis.thetas(-2.0F)) == 0.0F);
  }

  SECTION("horizontal")
  {
    PolarAxisLine hor_pos{1.0F, true};

    CHECK(hor_pos.thetas(1.0F) == std::tuple<float, float>(0.0F, 0.0F));
    CHECK(std::get<0>(hor_pos.thetas(std::sqrt(2.0F))) == Catch::Approx(M_PI_4));
    CHECK(std::get<1>(hor_pos.thetas(std::sqrt(2.0F))) == Catch::Approx(-M_PI_4));


    PolarAxisLine hor_neg{-1.0F, true};

    CHECK(hor_neg.thetas(1.0F) == std::tuple<float, float>(M_PI, -M_PI));
    CHECK(std::get<0>(hor_neg.thetas(std::sqrt(2.0F))) == Catch::Approx(M_PI_2 + M_PI_4));
    CHECK(std::get<1>(hor_neg.thetas(std::sqrt(2.0F))) == Catch::Approx(-(M_PI_2 + M_PI_4)));
  }
}
