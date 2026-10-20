// Copyright 2026 azzamwildan462
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "../src/experimental/stop_zone.hpp"

#include <gtest/gtest.h>

#include <optional>

namespace rc = autoware::behavior_velocity_planner::experimental::road_crossing;

namespace
{
// GAMA: base_link -> front bumper 0.9 m, zone 2.0 m, line 0.5 m.  Entry at s = 10:
//   barrier at 10 - 2.9 = 7.1, line at 10 - 1.4 = 8.6.
rc::StopZoneParam gama()
{
  return rc::StopZoneParam{};
}
constexpr double kEntry = 10.0;
constexpr double kLongPre = 0.0;  // pre lanelet from s = 0: long enough not to limit the zone
}  // namespace

TEST(StopZone, FarOutsideStopsAtBarrier)
{
  const auto r = rc::computeStopZone(2.0, kEntry, kLongPre, 0.8, false, gama());
  EXPECT_EQ(r.mode, rc::StopZoneMode::BARRIER);
  EXPECT_NEAR(r.stop_s, 7.1, 1e-9);
}

TEST(StopZone, StoppedJustBeforeBarrierIsHeldInPlace)
{
  // Smooth stop halted 1.0 m short of the barrier: it must not creep the last metre.
  const auto r = rc::computeStopZone(6.1, kEntry, kLongPre, 0.0, true, gama());
  EXPECT_EQ(r.mode, rc::StopZoneMode::HOLD_IN_ZONE);
  EXPECT_NEAR(r.stop_s, 6.1, 1e-9);
}

TEST(StopZone, StoppedBeyondToleranceDrivesToBarrier)
{
  // 2.0 m short of the barrier is a clean departure for the controller (> 1.5 m).
  const auto r = rc::computeStopZone(5.1, kEntry, kLongPre, 0.0, true, gama());
  EXPECT_EQ(r.mode, rc::StopZoneMode::BARRIER);
  EXPECT_NEAR(r.stop_s, 7.1, 1e-9);
}

TEST(StopZone, MovingOutsideToleranceIsNotHeld)
{
  // The tolerance is only for a robot standing still.
  const auto r = rc::computeStopZone(6.1, kEntry, kLongPre, 0.5, false, gama());
  EXPECT_EQ(r.mode, rc::StopZoneMode::BARRIER);
  EXPECT_NEAR(r.stop_s, 7.1, 1e-9);
}

TEST(StopZone, MovingIntoZoneBrakesWithinBrakingDistance)
{
  // 0.8 m/s at 1.0 m/s^2 -> 0.32 m.
  const auto r = rc::computeStopZone(7.3, kEntry, kLongPre, 0.8, false, gama());
  EXPECT_EQ(r.mode, rc::StopZoneMode::HOLD_IN_ZONE);
  EXPECT_NEAR(r.stop_s, 7.62, 1e-9);
}

TEST(StopZone, BrakingNeverPassesTheLine)
{
  const auto r = rc::computeStopZone(8.4, kEntry, kLongPre, 0.8, false, gama());
  EXPECT_EQ(r.mode, rc::StopZoneMode::HOLD_IN_ZONE);
  EXPECT_NEAR(r.stop_s, 8.6, 1e-9);
}

TEST(StopZone, PastTheLineStopsWhereItStands)
{
  const auto r = rc::computeStopZone(9.0, kEntry, kLongPre, 0.3, false, gama());
  EXPECT_EQ(r.mode, rc::StopZoneMode::HOLD_IN_ZONE);
  EXPECT_NEAR(r.stop_s, 9.0, 1e-9);
}

TEST(StopZone, ZoneShorterThanStopMarginIsClamped)
{
  auto p = gama();
  p.zone_length = 0.2;  // < stop_margin 0.5 -> barrier = line
  const auto r = rc::computeStopZone(2.0, kEntry, kLongPre, 0.8, false, p);
  EXPECT_EQ(r.mode, rc::StopZoneMode::BARRIER);
  EXPECT_NEAR(r.stop_s, 8.6, 1e-9);
}

TEST(StopZone, MapKantorLuarStartPositionIsHeld)
{
  // Robot engaged at s = 0.37 in pre lanelet 304, crossing entry 2.0 m ahead (s = 2.37):
  // the old 0.5 m stop line sat 0.6 m ahead and the robot crept there in hops.
  const auto r = rc::computeStopZone(0.37, 2.37, std::optional<double>{0.0}, 0.0, true, gama());
  EXPECT_EQ(r.mode, rc::StopZoneMode::HOLD_IN_ZONE);
  EXPECT_NEAR(r.stop_s, 0.37, 1e-9);
}

// --- pre lanelet shorter than the zone (local tags: pre 304 is 2.37 m, zone needs 2.9 m) ---
// entry at 2.37, pre from 0.0: barrier = max(2.37 - 2.9, 0.0 - 0.9 + 0.5) = -0.4 (front bumper
// 0.5 m into the pre), line = 0.97.  "On the pre" = front bumper on it (ego_s >= -0.9).

TEST(StopZone, ShortPreBarrierPutsTheFrontInsideThePre)
{
  const auto r = rc::computeStopZone(-5.0, 2.37, std::optional<double>{0.0}, 0.8, false, gama());
  EXPECT_EQ(r.mode, rc::StopZoneMode::BARRIER);
  EXPECT_NEAR(r.stop_s, -0.4, 1e-9);
}

TEST(StopZone, StoppedWithTheFrontOffThePreIsMovedInOneGo)
{
  // Front bumper 0.3 m short of the pre (base_link at -1.2): one move >= 1.5 m, not past the line.
  const auto r = rc::computeStopZone(-1.2, 2.37, std::optional<double>{0.0}, 0.0, true, gama());
  EXPECT_EQ(r.mode, rc::StopZoneMode::BARRIER);
  EXPECT_NEAR(r.stop_s, 0.3, 1e-9);
}

TEST(StopZone, StoppedWithOnlyTheFrontOnThePreIsHeld)
{
  // base_link 0.3 m before the pre, front bumper 0.6 m into it: the robot is at the pre.
  const auto r = rc::computeStopZone(-0.3, 2.37, std::optional<double>{0.0}, 0.0, true, gama());
  EXPECT_EQ(r.mode, rc::StopZoneMode::HOLD_IN_ZONE);
  EXPECT_NEAR(r.stop_s, -0.3, 1e-9);
}

TEST(StopZone, StoppedOnThePreBeforeTheBarrierIsHeld)
{
  // Long pre: barrier at 7.1, stopped at 6.1 (on the pre, within the tolerance) -> held.
  const auto r = rc::computeStopZone(6.1, 10.0, std::optional<double>{0.0}, 0.0, true, gama());
  EXPECT_EQ(r.mode, rc::StopZoneMode::HOLD_IN_ZONE);
  EXPECT_NEAR(r.stop_s, 6.1, 1e-9);
}

TEST(StopZone, PreShorterThanLineMarginStopsWithTheFrontOnIt)
{
  // Pre starts 1.2 m before the entry: barrier = max(7.1, 8.8 - 0.4) = 8.4 (front 0.5 m in).
  const auto r = rc::computeStopZone(-5.0, 10.0, std::optional<double>{8.8}, 0.8, false, gama());
  EXPECT_EQ(r.mode, rc::StopZoneMode::BARRIER);
  EXPECT_NEAR(r.stop_s, 8.4, 1e-9);
}

TEST(StopZone, LongPreKeepsTheTwoMetreZone)
{
  // Pre of 20 m: the barrier stays 2.0 m (front bumper) before the entry, not at the pre start.
  const auto r = rc::computeStopZone(-5.0, 20.0, std::optional<double>{0.0}, 0.8, false, gama());
  EXPECT_EQ(r.mode, rc::StopZoneMode::BARRIER);
  EXPECT_NEAR(r.stop_s, 17.1, 1e-9);
}

TEST(StopZone, NoPreFoundBehavesAsUnlimitedZone)
{
  const auto r = rc::computeStopZone(6.1, kEntry, std::nullopt, 0.0, true, gama());
  EXPECT_EQ(r.mode, rc::StopZoneMode::HOLD_IN_ZONE);
  EXPECT_NEAR(r.stop_s, 6.1, 1e-9);
}

TEST(StopZone, MapKantorLuarRealStartIsHeldWhereItStands)
{
  // UTM-projected map (what Autoware loads): the usual start has base_link 0.31 m BEFORE pre 304
  // (s 6.30 vs 6.62), front bumper 0.59 m inside it, entry 2.64 m ahead.  It is at the pre:
  // held where it stands, no creep onto it.
  const auto r = rc::computeStopZone(6.30, 8.95, std::optional<double>{6.62}, 0.0, true, gama());
  EXPECT_EQ(r.mode, rc::StopZoneMode::HOLD_IN_ZONE);
  EXPECT_NEAR(r.stop_s, 6.30, 1e-9);
}

TEST(StopZone, MovingWithTheFrontOffThePreKeepsADepartureDistance)
{
  // Rolling slowly 0.2 m short (front) of the pre: the stop point stays >= 1.5 m ahead (capped
  // at the line) instead of jumping back to a barrier the controller can only creep to.
  const auto r = rc::computeStopZone(-1.1, 2.37, std::optional<double>{0.0}, 0.1, false, gama());
  EXPECT_EQ(r.mode, rc::StopZoneMode::BARRIER);
  EXPECT_NEAR(r.stop_s, 0.4, 1e-9);
}
