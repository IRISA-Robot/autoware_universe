// Copyright 2026 azzamwildan462
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Reverse must behave like forward: the same situation seen from the leading bumper gives the
// same stop, whichever bumper leads.

#include "../src/experimental/direction.hpp"
#include "../src/experimental/stop_zone.hpp"

#include <gtest/gtest.h>

#include <optional>
#include <vector>

namespace rc = autoware::behavior_velocity_planner::experimental::road_crossing;

namespace
{
// GAMA vehicle_info: wheel_base 0.6 + front_overhang 0.3, rear_overhang 0.3.
constexpr double kMaxOffset = 0.9;   // max_longitudinal_offset_m
constexpr double kMinOffset = -0.3;  // min_longitudinal_offset_m = -rear_overhang
}  // namespace

TEST(Gear, SignOfThePlannedVelocity)
{
  EXPECT_EQ(rc::gearFromVelocities({0.5, 0.5}), std::optional<bool>(true));
  EXPECT_EQ(rc::gearFromVelocities({-0.5, -0.5}), std::optional<bool>(false));
  // Held at a stop: zero up to ego, the motion further on decides.
  EXPECT_EQ(rc::gearFromVelocities({0.0, 0.0, -0.8}), std::optional<bool>(false));
  EXPECT_EQ(rc::gearFromVelocities({0.0, 0.0, 0.8}), std::optional<bool>(true));
  // All zero (stop at ego): unknown -> the caller keeps the previous gear.
  EXPECT_EQ(rc::gearFromVelocities({0.0, 0.0}), std::nullopt);
  EXPECT_EQ(rc::gearFromVelocities({}), std::nullopt);
  // Numerical noise below eps is not a direction.
  EXPECT_EQ(rc::gearFromVelocities({0.005, -0.005, -0.3}), std::optional<bool>(false));
}

TEST(LeadingEdge, FrontForwardRearReverse)
{
  EXPECT_DOUBLE_EQ(rc::leadingEdgeOffset(kMaxOffset, kMinOffset, true), 0.9);
  EXPECT_DOUBLE_EQ(rc::leadingEdgeOffset(kMaxOffset, kMinOffset, false), 0.3);
}

TEST(PastExit, MirroredTravelGivesTheSameAnswer)
{
  // Crossing drawn (0,0) -> (10,0).  Forward travel: entry (0,0), exit (10,0).  The reverse
  // direction takes it inverted: entry (10,0), exit (0,0).  Ego t metres into the crossing.
  for (double t = -3.0; t <= 13.0; t += 0.25) {
    const bool fwd = rc::pastExit(0.0, 0.0, 10.0, 0.0, t, 0.4);
    const bool rev = rc::pastExit(10.0, 0.0, 0.0, 0.0, 10.0 - t, 0.4);
    EXPECT_EQ(fwd, rev) << "t=" << t;
    EXPECT_EQ(fwd, t > 10.0) << "t=" << t;
  }
}

TEST(StopZoneParity, SameBumperPositionGivesTheSameStop)
{
  // Everything the zone decides is about the leading bumper: with the bumper at the same arc
  // length the stop must be the same bumper position (and mode) in both gears.
  const double entry = 20.0;
  const std::vector<std::optional<double>> pres{std::nullopt, 0.0, 15.0, 17.5};
  const std::vector<double> speeds{0.0, 0.3, 0.8, 1.4};
  for (const auto & pre : pres) {
    for (const double v : speeds) {
      for (const bool stopped : {false, true}) {
        if (stopped && v > 0.05) continue;
        for (double bumper = 5.0; bumper <= 21.0; bumper += 0.1) {
          rc::StopZoneParam fwd_p;
          fwd_p.leading_edge_offset = rc::leadingEdgeOffset(kMaxOffset, kMinOffset, true);
          rc::StopZoneParam rev_p = fwd_p;
          rev_p.leading_edge_offset = rc::leadingEdgeOffset(kMaxOffset, kMinOffset, false);
          const auto f = rc::computeStopZone(
            bumper - fwd_p.leading_edge_offset, entry, pre, v, stopped, fwd_p);
          const auto r = rc::computeStopZone(
            bumper - rev_p.leading_edge_offset, entry, pre, v, stopped, rev_p);
          ASSERT_EQ(f.mode, r.mode) << "bumper=" << bumper << " v=" << v << " pre="
                                    << (pre ? *pre : -1.0) << " stopped=" << stopped;
          ASSERT_NEAR(
            f.stop_s + fwd_p.leading_edge_offset, r.stop_s + rev_p.leading_edge_offset, 1e-9)
            << "bumper=" << bumper << " v=" << v << " pre=" << (pre ? *pre : -1.0)
            << " stopped=" << stopped;
        }
      }
    }
  }
}

TEST(TravelDirection, FromThePathPointOrderNotTheGear)
{
  // Crossing drawn (0,0) -> (10,0).  Path through it along the drawing and against it; the
  // answer must not depend on anything but the path's point order (both gears).
  EXPECT_FALSE(rc::travelAgainstDrawing(0, 0, 10, 0, 4.0, 0.1, 4.5, 0.1));
  EXPECT_TRUE(rc::travelAgainstDrawing(0, 0, 10, 0, 4.5, 0.1, 4.0, 0.1));
  // a curved approach, up to ~80 deg off the crossing, keeps the sign
  EXPECT_FALSE(rc::travelAgainstDrawing(0, 0, 10, 0, 5.0, -1.0, 5.1, -0.4));
  EXPECT_TRUE(rc::travelAgainstDrawing(0, 0, 10, 0, 5.1, -0.4, 4.9, -1.0));
  // mirrored crossing (drawn the other way) gives the opposite answer for the same path
  for (double x = 1.0; x < 9.0; x += 1.0) {
    EXPECT_NE(
      rc::travelAgainstDrawing(0, 0, 10, 0, x, 0.0, x + 0.3, 0.0),
      rc::travelAgainstDrawing(10, 0, 0, 0, x, 0.0, x + 0.3, 0.0));
  }
}
