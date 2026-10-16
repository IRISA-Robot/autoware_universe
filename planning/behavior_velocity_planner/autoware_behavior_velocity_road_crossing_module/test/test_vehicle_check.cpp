// Copyright 2026 azzamwildan462
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "../src/experimental/vehicle_check.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace vc = autoware::behavior_velocity_planner::experimental::vehicle_check;

namespace
{
// The robot crosses a road straight along +y from y = 0 to y = 10 at x = 0 (s = y).
std::vector<vc::CorridorPoint> corridor()
{
  std::vector<vc::CorridorPoint> c;
  for (int i = 0; i <= 40; ++i) {
    const double y = 0.25 * i;
    c.push_back({y, {0.0, y}});
  }
  return c;
}

// A vehicle on the lane at y = lane_y, at x = x0, driving +x at `speed`; 8 s of prediction.
vc::Vehicle carOnLane(const double lane_y, const double x0, const double speed)
{
  vc::Vehicle v;
  v.speed = speed;
  v.half_width = 0.9;
  std::vector<vc::Point> path;
  for (int i = 0; i <= 80; ++i) {
    path.push_back({x0 + speed * 0.1 * i, lane_y});
  }
  v.predicted_paths.push_back(path);
  return v;
}

vc::Ego egoAt(const double s)
{
  vc::Ego ego;
  ego.s = s;
  ego.front_offset = 0.9;
  ego.rear_offset = 0.3;
  ego.half_width = 0.85;
  ego.crossing_speed = 1.39;
  return ego;
}
}  // namespace

TEST(VehicleCheck, ConflictZoneOnTheRobotsPath)
{
  const auto conflict = vc::findConflict(corridor(), carOnLane(3.0, -30.0, 8.0), 0.85);
  ASSERT_TRUE(conflict.has_value());
  EXPECT_NEAR(conflict->zone_start_s, 3.0 - 1.75, 0.25);
  EXPECT_NEAR(conflict->zone_end_s, 3.0 + 1.75, 0.25);
  EXPECT_NEAR(conflict->distance, 30.0 - 1.75, 0.8);
}

TEST(VehicleCheck, CarThatCanStillStopIsNoDanger)
{
  // Green for the robot, the car is slowing to its red light: at 8 m/s it needs ~10.7 m and has
  // ~28 m.  Its constant-speed prediction still crosses the robot's path.
  const auto zone = vc::findNearestDangerZone(
    corridor(), {carOnLane(3.0, -30.0, 8.0)}, egoAt(0.0), vc::Params{});
  EXPECT_FALSE(zone.has_value());
}

TEST(VehicleCheck, CarThatCannotStopHoldsTheRobotBeforeItsZone)
{
  // 10 m/s, ~10 m away: needs ~16.7 m to stop, and arrives long before the robot is through.
  const auto zone = vc::findNearestDangerZone(
    corridor(), {carOnLane(3.0, -12.0, 10.0)}, egoAt(0.0), vc::Params{});
  ASSERT_TRUE(zone.has_value());
  EXPECT_NEAR(*zone, 3.0 - 1.75, 0.25);
}

TEST(VehicleCheck, NeverHoldsInsideAZoneTheRobotIsAlreadyIn)
{
  // Same car, but the robot's front is already past the start of its zone: going on is the way
  // out, so this zone is not a place to stop.
  const auto zone = vc::findNearestDangerZone(
    corridor(), {carOnLane(3.0, -12.0, 10.0)}, egoAt(1.0), vc::Params{});
  EXPECT_FALSE(zone.has_value());
}

TEST(VehicleCheck, SecondLaneStillCheckedWhileCrossingTheFirst)
{
  // Robot is in lane 1's zone (y = 3); a car on lane 2 (y = 8) cannot stop: hold before lane 2.
  const auto zone = vc::findNearestDangerZone(
    corridor(), {carOnLane(3.0, -12.0, 10.0), carOnLane(8.0, -12.0, 10.0)}, egoAt(2.0),
    vc::Params{});
  ASSERT_TRUE(zone.has_value());
  EXPECT_NEAR(*zone, 8.0 - 1.75, 0.25);
}

TEST(VehicleCheck, StandingVehicles)
{
  // standing clear of the corridor: no danger
  EXPECT_FALSE(vc::findNearestDangerZone(
                 corridor(), {carOnLane(3.0, -10.0, 0.0)}, egoAt(0.0), vc::Params{})
                 .has_value());
  // standing in the corridor: danger
  EXPECT_TRUE(vc::findNearestDangerZone(
                corridor(), {carOnLane(3.0, 0.0, 0.0)}, egoAt(0.0), vc::Params{})
                .has_value());
}

TEST(VehicleCheck, CannotStopButArrivesAfterTheRobotIsThrough)
{
  vc::Vehicle vehicle;
  vehicle.speed = 30.0;  // needs 150 m to stop
  const vc::Conflict conflict{6.0, 7.0, 100.0};  // arrives in 3.3 s
  // robot rear at 6.2, through at 7.0: 0.6 s + 2 s margin < 3.3 s
  EXPECT_FALSE(vc::isDanger(conflict, vehicle, egoAt(6.5), vc::Params{}));
  // robot still at the start: 5.3 s + 2 s > 3.3 s
  EXPECT_TRUE(vc::isDanger(conflict, vehicle, egoAt(0.0), vc::Params{}));
}
