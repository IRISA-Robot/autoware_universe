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

#include "autoware/road_crossing_fsm/vehicle_check.hpp"

#include <gtest/gtest.h>

#include <cmath>

using autoware::road_crossing_fsm::vehicle_check::boxFootprint;
using autoware::road_crossing_fsm::vehicle_check::buildCorridor;
using autoware::road_crossing_fsm::vehicle_check::checkVehicle;
using autoware::road_crossing_fsm::vehicle_check::Polyline;

namespace
{
// Robot at the origin crossing along +y; the crossing (road) spans y = 2..12.
const Polyline kCrossing{{0.0, 2.0}, {0.0, 12.0}};
constexpr double kHalfWidth = 0.35 + 0.1;  // robot 0.70 m + 0.1 m margin

Polyline straightTrajectory(double y0, double y1, double step = 0.5)
{
  Polyline t;
  for (double y = y0; y <= y1 + 1e-9; y += step) t.push_back({0.0, y});
  return t;
}
}  // namespace

TEST(VehicleCheckCorridor, TrajectoryCutAtCrossingExit)
{
  const auto c = buildCorridor(straightTrajectory(-5.0, 30.0), {0.0, 0.0}, kCrossing);
  ASSERT_GE(c.size(), 2u);
  EXPECT_NEAR(c.front().y, 0.0, 1e-9);  // starts at ego, not at the trajectory start
  EXPECT_NEAR(c.back().y, 12.0, 1e-9);  // ends at the crossing exit
}

TEST(VehicleCheckCorridor, ShortTrajectoryExtendedWithCrossing)
{
  const auto c = buildCorridor(straightTrajectory(0.0, 5.0), {0.0, 0.0}, kCrossing);
  EXPECT_NEAR(c.back().y, 12.0, 1e-9);
}

TEST(VehicleCheckCorridor, NoTrajectoryUsesCrossing)
{
  const auto c = buildCorridor({}, {0.0, 0.0}, kCrossing);
  ASSERT_GE(c.size(), 2u);
  EXPECT_NEAR(c.front().y, 0.0, 1e-9);
  EXPECT_NEAR(c.back().y, 12.0, 1e-9);
}

TEST(VehicleCheck, CarOnThePathHolds)
{
  const auto c = buildCorridor(straightTrajectory(0.0, 12.0), {0.0, 0.0}, kCrossing);
  // A car parked across the crosswalk, length along x.
  const auto r = checkVehicle(c, kHalfWidth, boxFootprint(1.0, 6.0, 0.0, 4.5, 1.8), 0, 0, 0, 0.5);
  EXPECT_TRUE(r.hit);
  EXPECT_DOUBLE_EQ(r.distance, 0.0);
}

TEST(VehicleCheck, CarJustBesideThePathByMoreThanMarginIsClear)
{
  const auto c = buildCorridor(straightTrajectory(0.0, 12.0), {0.0, 0.0}, kCrossing);
  // Car body edge at x = 0.6 (> 0.45): beside the path, not on it.
  const auto fp = boxFootprint(0.6 + 0.9, 6.0, M_PI / 2, 4.5, 1.8);
  EXPECT_FALSE(checkVehicle(c, kHalfWidth, fp, 0, 0, 0, 0.5).hit);
  // Edge at x = 0.4 (< 0.45): inside the margin band.
  const auto fp2 = boxFootprint(0.4 + 0.9, 6.0, M_PI / 2, 4.5, 1.8);
  EXPECT_TRUE(checkVehicle(c, kHalfWidth, fp2, 0, 0, 0, 0.5).hit);
}

TEST(VehicleCheck, CarStoppedAtItsLineBeforeTheCrosswalkIsClear)
{
  const auto c = buildCorridor(straightTrajectory(0.0, 12.0), {0.0, 0.0}, kCrossing);
  // Car on the road heading -x, front bumper 1.5 m from the path, standing.
  const auto fp = boxFootprint(1.5 + 2.25, 6.0, M_PI, 4.5, 1.8);
  EXPECT_FALSE(checkVehicle(c, kHalfWidth, fp, 0, 0, 1.5, 0.5).hit);
}

TEST(VehicleCheck, ApproachingCarHoldsOnlyWithLookahead)
{
  const auto c = buildCorridor(straightTrajectory(0.0, 12.0), {0.0, 0.0}, kCrossing);
  // Car 3 m from the path driving towards it at 4 m/s (-x).
  const auto fp = boxFootprint(3.0 + 2.25, 6.0, M_PI, 4.5, 1.8);
  EXPECT_FALSE(checkVehicle(c, kHalfWidth, fp, -4.0, 0.0, 0.0, 0.5).hit);
  const auto r = checkVehicle(c, kHalfWidth, fp, -4.0, 0.0, 1.5, 0.5);
  EXPECT_TRUE(r.hit);
  EXPECT_TRUE(r.by_motion);
  // Driving away from the path: clear.
  EXPECT_FALSE(checkVehicle(c, kHalfWidth, fp, 4.0, 0.0, 1.5, 0.5).hit);
}

TEST(VehicleCheck, SlowCarIsNotSwept)
{
  const auto c = buildCorridor(straightTrajectory(0.0, 12.0), {0.0, 0.0}, kCrossing);
  const auto fp = boxFootprint(1.0 + 2.25, 6.0, M_PI, 4.5, 1.8);
  // 0.3 m/s (< min speed 0.5) towards the path: noise / creeping, no sweep.
  EXPECT_FALSE(checkVehicle(c, kHalfWidth, fp, -0.3, 0.0, 10.0, 0.5).hit);
}

TEST(VehicleCheck, ParallelTrafficBesideTheCrossingIsClear)
{
  const auto c = buildCorridor(straightTrajectory(0.0, 12.0), {0.0, 0.0}, kCrossing);
  // Car driving parallel to the crosswalk (+y) 3 m to the side, fast: stays beside it.
  const auto fp = boxFootprint(3.0, 0.0, M_PI / 2, 4.5, 1.8);
  EXPECT_FALSE(checkVehicle(c, kHalfWidth, fp, 0.0, 12.0, 1.5, 0.5).hit);
}

TEST(VehicleCheck, CarBeyondTheCrossingExitIsClear)
{
  // Trajectory continues after the crossing; a car there is not on the crossing part.
  const auto c = buildCorridor(straightTrajectory(0.0, 40.0), {0.0, 0.0}, kCrossing);
  const auto fp = boxFootprint(0.0, 20.0, 0.0, 4.5, 1.8);
  EXPECT_FALSE(checkVehicle(c, kHalfWidth, fp, 0, 0, 0, 0.5).hit);
}

TEST(VehicleCheck, ReversedRouteCorridorFollowsTravelOrder)
{
  // Crossing taken from the other side: travel order y = 12 -> 2, ego at y = 14.
  const Polyline crossing{{0.0, 12.0}, {0.0, 2.0}};
  Polyline traj;
  for (double y = 20.0; y >= -10.0; y -= 0.5) traj.push_back({0.0, y});
  const auto c = buildCorridor(traj, {0.0, 14.0}, crossing);
  EXPECT_NEAR(c.front().y, 14.0, 1e-9);
  EXPECT_NEAR(c.back().y, 2.0, 1e-9);
  const auto fp = boxFootprint(0.0, 5.0, 0.0, 4.5, 1.8);
  EXPECT_TRUE(checkVehicle(c, kHalfWidth, fp, 0, 0, 0, 0.5).hit);
}
