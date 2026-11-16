// Copyright 2026 azzamwildan462
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "../src/area_check.hpp"

#include <boost/geometry/algorithms/correct.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace cc = autoware::behavior_velocity_planner::experimental::common_crossing;
using autoware_perception_msgs::msg::ObjectClassification;

namespace
{
cc::Polygon2d rect(const double x0, const double y0, const double x1, const double y1)
{
  cc::Polygon2d p;
  p.outer() = {{x0, y0}, {x0, y1}, {x1, y1}, {x1, y0}};
  boost::geometry::correct(p);
  return p;
}

// Car-sized box (4.5 x 1.8 m) centred at (cx, cy), rotated by yaw.
cc::Polygon2d car(const double cx, const double cy, const double yaw = 0.0)
{
  const double hl = 2.25;
  const double hw = 0.9;
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  cc::Polygon2d p;
  for (const auto & [lx, ly] : std::vector<std::pair<double, double>>{
         {hl, hw}, {hl, -hw}, {-hl, -hw}, {-hl, hw}}) {
    p.outer().emplace_back(cx + c * lx - s * ly, cy + s * lx + c * ly);
  }
  boost::geometry::correct(p);
  return p;
}

ObjectClassification cls(const uint8_t label, const float prob)
{
  ObjectClassification c;
  c.label = label;
  c.probability = prob;
  return c;
}
}  // namespace

TEST(LabelFilter, DefaultBlocksVehiclesOnly)
{
  const cc::BlockFilter f;
  EXPECT_TRUE(cc::labelBlocks(ObjectClassification::CAR, f));
  EXPECT_TRUE(cc::labelBlocks(ObjectClassification::TRUCK, f));
  EXPECT_TRUE(cc::labelBlocks(ObjectClassification::BUS, f));
  EXPECT_TRUE(cc::labelBlocks(ObjectClassification::TRAILER, f));
  EXPECT_TRUE(cc::labelBlocks(ObjectClassification::MOTORCYCLE, f));
  EXPECT_FALSE(cc::labelBlocks(ObjectClassification::BICYCLE, f));
  EXPECT_FALSE(cc::labelBlocks(ObjectClassification::PEDESTRIAN, f));
  EXPECT_FALSE(cc::labelBlocks(ObjectClassification::UNKNOWN, f));
}

TEST(LabelFilter, TopLabelIsHighestProbability)
{
  EXPECT_EQ(
    cc::topLabel({cls(ObjectClassification::PEDESTRIAN, 0.3F), cls(ObjectClassification::CAR, 0.7F)}),
    ObjectClassification::CAR);
  EXPECT_EQ(
    cc::topLabel({cls(ObjectClassification::CAR, 0.2F), cls(ObjectClassification::UNKNOWN, 0.8F)}),
    ObjectClassification::UNKNOWN);
  EXPECT_EQ(cc::topLabel({}), ObjectClassification::UNKNOWN);
}

TEST(Footprint, InsideOverlappingAndOutside)
{
  // Driveway area 6 m wide (x 0..6), 20 m long (y -10..10).
  const std::vector<cc::Polygon2d> areas{rect(0.0, -10.0, 6.0, 10.0)};
  EXPECT_TRUE(cc::footprintInAreas(car(3.0, 5.0, M_PI_2), areas));   // fully inside
  EXPECT_TRUE(cc::footprintInAreas(car(-1.5, 0.0), areas));           // nose 0.75 m in
  EXPECT_FALSE(cc::footprintInAreas(car(-3.0, 0.0), areas));          // 0.75 m short
  EXPECT_FALSE(cc::footprintInAreas(car(3.0, 13.0, M_PI_2), areas));  // beyond the far end
}

TEST(Footprint, AnyOfSeveralAreas)
{
  const std::vector<cc::Polygon2d> areas{rect(0.0, 0.0, 6.0, 10.0), rect(0.0, -20.0, 6.0, -10.0)};
  EXPECT_TRUE(cc::footprintInAreas(car(3.0, -15.0, M_PI_2), areas));
  EXPECT_FALSE(cc::footprintInAreas(car(3.0, -5.0, 0.0), {rect(0.0, 0.0, 6.0, 10.0)}));
}

TEST(Footprint, AreaWithoutAnyAreasNeverBlocks)
{
  EXPECT_FALSE(cc::footprintInAreas(car(0.0, 0.0), {}));
}

TEST(ClearTimer, CountsOnlyUninterruptedClear)
{
  cc::ClearTimer t;
  EXPECT_DOUBLE_EQ(t.update(false, 10.0), 0.0);
  EXPECT_DOUBLE_EQ(t.update(false, 11.0), 1.0);
  EXPECT_DOUBLE_EQ(t.update(true, 11.1), 0.0);   // one blocked cycle restarts it
  EXPECT_DOUBLE_EQ(t.update(false, 11.2), 0.0);
  EXPECT_NEAR(t.update(false, 12.7), 1.5, 1e-9);
  t.reset();
  EXPECT_DOUBLE_EQ(t.update(false, 20.0), 0.0);
}

TEST(HoldAgain, OnlyReleasedRobotBeforeTheEntry)
{
  // Released, not entered, entry ahead, blocked -> hold again.
  EXPECT_TRUE(cc::holdAgainBeforeEntry(true, false, true, true));
  // Entered: goes on whatever happens.
  EXPECT_FALSE(cc::holdAgainBeforeEntry(true, true, true, true));
  // Front already at/over the entry: goes on.
  EXPECT_FALSE(cc::holdAgainBeforeEntry(true, false, false, true));
  // Clear: nothing to do.
  EXPECT_FALSE(cc::holdAgainBeforeEntry(true, false, true, false));
  // Not released (still stopping / checking): handled by the state machine itself.
  EXPECT_FALSE(cc::holdAgainBeforeEntry(false, false, true, true));
}
