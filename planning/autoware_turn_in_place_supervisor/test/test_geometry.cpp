// Copyright 2026 azzamwildan462
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/turn_in_place_supervisor/geometry.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using autoware::turn_in_place_supervisor::disk_inside_area;
using autoware::turn_in_place_supervisor::Footprint;
using autoware::turn_in_place_supervisor::point_allowed;
using autoware::turn_in_place_supervisor::point_in_any;
using autoware::turn_in_place_supervisor::Polygon2d;
using autoware::turn_in_place_supervisor::sweep_inside_area;
using autoware::turn_in_place_supervisor::kDirectionCcw;
using autoware::turn_in_place_supervisor::kDirectionCw;
using autoware::turn_in_place_supervisor::normalize_angle;
using autoware::turn_in_place_supervisor::Point2d;
using autoware::turn_in_place_supervisor::rotation_in_direction;
using autoware::turn_in_place_supervisor::sweep_is_clear;
using autoware::turn_in_place_supervisor::sweep_radius;

namespace
{
constexpr double kDeg = M_PI / 180.0;
// 1.2 m x 0.7 m body spun about its centre.
const Footprint kBody{0.6, 0.6, 0.35, 0.35};
}  // namespace

TEST(Geometry, NormalizeAngle)
{
  EXPECT_NEAR(normalize_angle(3.0 * M_PI / 2.0), -M_PI / 2.0, 1e-9);
  EXPECT_NEAR(normalize_angle(-3.0 * M_PI / 2.0), M_PI / 2.0, 1e-9);
  EXPECT_NEAR(normalize_angle(M_PI), M_PI, 1e-9);
  EXPECT_NEAR(normalize_angle(0.1), 0.1, 1e-9);
}

TEST(Geometry, RotationInDirection)
{
  // From 0 to +90 deg: a quarter turn CCW, three quarters CW.
  EXPECT_NEAR(rotation_in_direction(0.0, 90.0 * kDeg, kDirectionCcw), 90.0 * kDeg, 1e-9);
  EXPECT_NEAR(rotation_in_direction(0.0, 90.0 * kDeg, kDirectionCw), 270.0 * kDeg, 1e-9);
  // Across the +-pi seam.
  EXPECT_NEAR(rotation_in_direction(170.0 * kDeg, -170.0 * kDeg, kDirectionCcw), 20.0 * kDeg, 1e-9);
}

TEST(Geometry, NoPointsIsClear)
{
  EXPECT_TRUE(sweep_is_clear({}, kBody, 0.2, M_PI, 5.0 * kDeg));
}

TEST(Geometry, PointOutsideTheCircleNeverBlocks)
{
  const double r = sweep_radius(kBody, 0.2);
  const std::vector<Point2d> far{{r + 0.05, 0.0}, {0.0, -(r + 0.05)}};
  EXPECT_TRUE(sweep_is_clear(far, kBody, 0.2, 2.0 * M_PI, 5.0 * kDeg));
}

TEST(Geometry, PointInsideTheBodyAlwaysBlocks)
{
  EXPECT_FALSE(sweep_is_clear({{0.3, 0.0}}, kBody, 0.2, 0.0, 5.0 * kDeg));
}

TEST(Geometry, DirectionMattersForAPartialTurn)
{
  // A point beside the front-left corner, outside the body but inside the corner circle.  With a
  // 0.05 m margin the body reaches it after 11-22 deg of a CCW turn, but only after 101-112 deg of
  // a CW turn: a 60 deg turn is blocked CCW and clear CW.
  const std::vector<Point2d> pts{{0.5, 0.5}};
  EXPECT_FALSE(sweep_is_clear(pts, kBody, 0.05, 60.0 * kDeg, 2.0 * kDeg));
  EXPECT_TRUE(sweep_is_clear(pts, kBody, 0.05, -60.0 * kDeg, 2.0 * kDeg));
  EXPECT_FALSE(sweep_is_clear(pts, kBody, 0.05, -120.0 * kDeg, 2.0 * kDeg));
}

TEST(Geometry, HalfTurnSweepsTheWholeCircleEitherWay)
{
  // Anything within the corner circle is hit by a 180 deg turn in either direction.
  const std::vector<Point2d> pts{{0.0, 0.62}};
  EXPECT_FALSE(sweep_is_clear(pts, kBody, 0.05, M_PI, 2.0 * kDeg));
  EXPECT_FALSE(sweep_is_clear(pts, kBody, 0.05, -M_PI, 2.0 * kDeg));
}

namespace
{
Polygon2d rect(double x0, double y0, double x1, double y1)
{
  return {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
}
}  // namespace

TEST(Geometry, NarrowLaneBlocksTheSpinWideRoadAllowsIt)
{
  // A 1.1 m lane (the body is 0.7 m wide, 1.2 m long): driving fits, spinning does not.
  const std::vector<Polygon2d> lane{rect(-10.0, -0.55, 10.0, 0.55)};
  EXPECT_TRUE(sweep_inside_area(lane, kBody, 0.1, 0.0, 5.0 * kDeg, 0.15));
  EXPECT_FALSE(sweep_inside_area(lane, kBody, 0.1, M_PI, 5.0 * kDeg, 0.15));
  EXPECT_FALSE(sweep_inside_area(lane, kBody, 0.1, -M_PI, 5.0 * kDeg, 0.15));
  // Add the opposite lane next to it (any road lanelet counts): now it fits.
  const std::vector<Polygon2d> road{rect(-10.0, -0.55, 10.0, 0.55), rect(-10.0, 0.55, 10.0, 1.65)};
  // Still not: the rotation centre is 0.55 m from the outer edge on the right side.
  EXPECT_FALSE(sweep_inside_area(road, kBody, 0.1, M_PI, 5.0 * kDeg, 0.15));
  const std::vector<Polygon2d> wide{rect(-10.0, -1.2, 10.0, 0.0), rect(-10.0, 0.0, 10.0, 1.2)};
  EXPECT_TRUE(sweep_inside_area(wide, kBody, 0.1, M_PI, 5.0 * kDeg, 0.15));
}

TEST(Geometry, NoRoadMeansNoTurn)
{
  EXPECT_FALSE(sweep_inside_area({}, kBody, 0.1, M_PI, 5.0 * kDeg, 0.15));
}

TEST(Geometry, DiskInsideTheUnionOfLanelets)
{
  const std::vector<Polygon2d> road{rect(0.0, -1.0, 5.0, 0.0), rect(0.0, 0.0, 5.0, 1.0)};
  EXPECT_TRUE(point_in_any({1.0, 0.5}, road));
  EXPECT_FALSE(point_in_any({6.0, 0.5}, road));
  EXPECT_TRUE(disk_inside_area(road, {2.5, 0.0}, 0.9, 10.0 * kDeg));
  EXPECT_FALSE(disk_inside_area(road, {2.5, 0.5}, 0.9, 10.0 * kDeg));
  EXPECT_FALSE(disk_inside_area(road, {0.5, 0.0}, 0.9, 10.0 * kDeg));
}

TEST(Geometry, NoTurnLaneletIsOffLimitsEvenWhereRoadOverlapsIt)
{
  // Wide road, but its upper half is also a no-turn lanelet.
  const std::vector<Polygon2d> road{rect(-10.0, -1.2, 10.0, 1.2)};
  const std::vector<Polygon2d> no_turn{rect(-10.0, 0.6, 10.0, 1.2)};
  EXPECT_TRUE(point_allowed({0.0, 0.0}, road, no_turn));
  EXPECT_FALSE(point_allowed({0.0, 0.9}, road, no_turn));
  EXPECT_TRUE(sweep_inside_area(road, kBody, 0.1, M_PI, 5.0 * kDeg, 0.15));
  EXPECT_FALSE(sweep_inside_area(road, kBody, 0.1, M_PI, 5.0 * kDeg, 0.15, no_turn));
  EXPECT_TRUE(disk_inside_area(road, {0.0, 0.0}, 0.9, 10.0 * kDeg));
  EXPECT_FALSE(disk_inside_area(road, {0.0, 0.0}, 0.9, 10.0 * kDeg, no_turn));
  // Clear of it further along: allowed again.
  const std::vector<Polygon2d> short_no_turn{rect(-10.0, -1.2, -2.0, 1.2)};
  EXPECT_FALSE(disk_inside_area(road, {-1.5, 0.0}, 0.9, 10.0 * kDeg, short_no_turn));
  EXPECT_TRUE(disk_inside_area(road, {0.0, 0.0}, 0.9, 10.0 * kDeg, short_no_turn));
}
