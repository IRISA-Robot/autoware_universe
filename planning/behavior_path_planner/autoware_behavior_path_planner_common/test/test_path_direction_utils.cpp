// Copyright 2026 Autoware Planning Team
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

// [TURN-IN-PLACE 2026-11-05] fold truncation, heading continuity and the inverted-lanelet
// turn_direction swap.

#include "autoware/behavior_path_planner_common/utils/path_direction_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"

#include <lanelet2_core/primitives/Lanelet.h>

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using autoware_internal_planning_msgs::msg::PathWithLaneId;

namespace
{
PathWithLaneId make_path(const std::vector<std::pair<double, double>> & xy, const float v = 1.0f)
{
  PathWithLaneId path;
  for (const auto & [x, y] : xy) {
    autoware_internal_planning_msgs::msg::PathPointWithLaneId p;
    p.point.pose.position.x = x;
    p.point.pose.position.y = y;
    p.point.pose.orientation.w = 1.0;
    p.point.longitudinal_velocity_mps = v;
    path.points.push_back(p);
  }
  return path;
}

lanelet::Lanelet make_straight_lanelet(
  lanelet::Id id, double x0, double y0, double x1, double y1)
{
  // left bound offset +1 in the lanelet's left normal, right bound -1.
  const double len = std::hypot(x1 - x0, y1 - y0);
  const double nx = -(y1 - y0) / len;
  const double ny = (x1 - x0) / len;
  lanelet::LineString3d left(
    id * 10 + 1, {lanelet::Point3d(id * 10 + 2, x0 + nx, y0 + ny, 0.0),
                  lanelet::Point3d(id * 10 + 3, x1 + nx, y1 + ny, 0.0)});
  lanelet::LineString3d right(
    id * 10 + 4, {lanelet::Point3d(id * 10 + 5, x0 - nx, y0 - ny, 0.0),
                  lanelet::Point3d(id * 10 + 6, x1 - nx, y1 - ny, 0.0)});
  return lanelet::Lanelet(id, left, right);
}
}  // namespace

using autoware::behavior_path_planner::utils::findAllDirectionReversalIndices;
using autoware::behavior_path_planner::utils::findFirstDirectionReversalIndex;
using autoware::behavior_path_planner::utils::get_turn_direction_in_travel_direction;
using autoware::behavior_path_planner::utils::isHeadingContinuousAcross;
using autoware::behavior_path_planner::utils::truncatePathAtFold;

constexpr double kThreshold = 150.0 * M_PI / 180.0;

namespace
{
geometry_msgs::msg::Pose ego(const double x, const double y, const double yaw)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.orientation.z = std::sin(yaw / 2.0);
  p.orientation.w = std::cos(yaw / 2.0);
  return p;
}

// out along +x to x=3, back along -x to x=1, then a second fold back to +x (x=2).
PathWithLaneId out_and_back()
{
  return make_path({{0, 0}, {1, 0}, {2, 0}, {3, 0}, {2, 0}, {1, 0}, {2, 0}});
}
}  // namespace

TEST(PathDirectionUtils, noFoldLeavesPathUntouched)
{
  auto path = make_path({{0, 0}, {1, 0}, {2, 0.5}, {3, 1.5}});
  EXPECT_FALSE(findFirstDirectionReversalIndex(path, kThreshold));
  EXPECT_FALSE(truncatePathAtFold(path, kThreshold, ego(0, 0, 0)));
  ASSERT_EQ(path.points.size(), 4u);
  EXPECT_FLOAT_EQ(path.points.back().point.longitudinal_velocity_mps, 1.0f);
}

TEST(PathDirectionUtils, findsAllFolds)
{
  const auto path = out_and_back();
  const auto folds = findAllDirectionReversalIndices(path, kThreshold);
  ASSERT_EQ(folds.size(), 2u);
  EXPECT_EQ(folds.at(0), 3u);
  EXPECT_EQ(folds.at(1), 5u);
  EXPECT_EQ(*findFirstDirectionReversalIndex(path, kThreshold), 3u);
}

TEST(PathDirectionUtils, withoutEgoTruncatesAtFirstFoldAndStops)
{
  auto path = out_and_back();
  const auto orientation_before = path.points.at(3).point.pose.orientation;
  const auto cut = truncatePathAtFold(path, kThreshold, std::nullopt);
  ASSERT_TRUE(cut);
  EXPECT_EQ(cut->kept_from, 0u);
  EXPECT_EQ(cut->kept_to, 3u);
  EXPECT_TRUE(cut->stops_at_end);
  ASSERT_EQ(path.points.size(), 4u);
  EXPECT_DOUBLE_EQ(path.points.back().point.pose.position.x, 3.0);
  EXPECT_FLOAT_EQ(path.points.back().point.longitudinal_velocity_mps, 0.0f);
  EXPECT_FLOAT_EQ(path.points.at(2).point.longitudinal_velocity_mps, 1.0f);
  EXPECT_EQ(path.points.back().point.pose.orientation, orientation_before);
}

TEST(PathDirectionUtils, egoOnFirstLegStopsAtFirstFold)
{
  // Ego at x=1.2 facing +x: nearest overall could be the return leg's x=1 point, but it faces
  // the other way, so ego is matched to the outbound leg.
  auto path = out_and_back();
  const auto cut = truncatePathAtFold(path, kThreshold, ego(1.2, 0.0, 0.0));
  ASSERT_TRUE(cut);
  EXPECT_EQ(cut->kept_from, 0u);
  EXPECT_EQ(cut->kept_to, 3u);
  EXPECT_TRUE(cut->stops_at_end);
  EXPECT_FLOAT_EQ(path.points.back().point.longitudinal_velocity_mps, 0.0f);
}

TEST(PathDirectionUtils, egoStoppedAtFoldStillFacingOutKeepsStopping)
{
  // Stopped just short of the apex, not yet rotated: still the outbound leg, still stopped.
  auto path = out_and_back();
  const auto cut = truncatePathAtFold(path, kThreshold, ego(2.9, 0.0, 0.0));
  ASSERT_TRUE(cut);
  EXPECT_EQ(cut->kept_from, 0u);
  EXPECT_EQ(cut->kept_to, 3u);
  EXPECT_TRUE(cut->stops_at_end);
}

TEST(PathDirectionUtils, egoTurnedAtFoldGetsReturnLeg)
{
  // After turning in place at the apex: ego faces -x; the outbound leg behind it is dropped and
  // the path stops at the next fold (x=1).
  auto path = out_and_back();
  const auto cut = truncatePathAtFold(path, kThreshold, ego(2.9, 0.0, M_PI));
  ASSERT_TRUE(cut);
  EXPECT_EQ(cut->kept_from, 3u);
  EXPECT_EQ(cut->kept_to, 5u);
  EXPECT_TRUE(cut->stops_at_end);
  ASSERT_EQ(path.points.size(), 3u);
  EXPECT_DOUBLE_EQ(path.points.front().point.pose.position.x, 3.0);
  EXPECT_DOUBLE_EQ(path.points.back().point.pose.position.x, 1.0);
  EXPECT_FLOAT_EQ(path.points.back().point.longitudinal_velocity_mps, 0.0f);
}

TEST(PathDirectionUtils, egoOnLastLegOnlyDropsBehind)
{
  auto path = make_path({{0, 0}, {1, 0}, {2, 0}, {3, 0}, {2, 0}, {1, 0}, {0, 0}});
  const auto cut = truncatePathAtFold(path, kThreshold, ego(1.5, 0.0, M_PI));
  ASSERT_TRUE(cut);
  EXPECT_EQ(cut->kept_from, 3u);
  EXPECT_EQ(cut->kept_to, 6u);
  EXPECT_FALSE(cut->stops_at_end);
  ASSERT_EQ(path.points.size(), 4u);
  EXPECT_FLOAT_EQ(path.points.back().point.longitudinal_velocity_mps, 1.0f);
}

TEST(PathDirectionUtils, degenerateSegmentsAreSkipped)
{
  // duplicate apex point: the fold is detected at the first segment that leaves it.
  auto path = make_path({{0, 0}, {1, 0}, {2, 0}, {2, 0}, {1, 0}});
  const auto cut = truncatePathAtFold(path, kThreshold, std::nullopt);
  ASSERT_TRUE(cut);
  EXPECT_EQ(cut->kept_to, 3u);
  EXPECT_FLOAT_EQ(path.points.back().point.longitudinal_velocity_mps, 0.0f);
}

TEST(PathDirectionUtils, sharpButNotFoldedTurnIsKept)
{
  // 120 deg turn: below the 150 deg fold threshold.
  const double a = 120.0 * M_PI / 180.0;
  auto path = make_path({{0, 0}, {1, 0}, {1 + std::cos(a), std::sin(a)}});
  EXPECT_FALSE(truncatePathAtFold(path, kThreshold, std::nullopt));
  EXPECT_EQ(path.points.size(), 3u);
}

TEST(PathDirectionUtils, isHeadingContinuousAcross)
{
  const auto a = make_straight_lanelet(1, 0, 0, 10, 0);
  const auto b = make_straight_lanelet(2, 10, 0, 20, 0);
  const auto c = make_straight_lanelet(3, 10, 0, 10, 10);  // 90 deg left
  EXPECT_TRUE(isHeadingContinuousAcross(a, b));
  EXPECT_TRUE(isHeadingContinuousAcross(a, c));
  // forward leg A meets the inverted return along A: ~180 deg seam
  EXPECT_FALSE(isHeadingContinuousAcross(a, lanelet::ConstLanelet(a).invert()));
  EXPECT_TRUE(isHeadingContinuousAcross(
    lanelet::ConstLanelet(b).invert(), lanelet::ConstLanelet(a).invert()));
}

TEST(PathDirectionUtils, turnDirectionSwapsOnInvertedLanelet)
{
  auto llt = make_straight_lanelet(4, 0, 0, 10, 0);
  EXPECT_EQ(get_turn_direction_in_travel_direction(llt, true, "none"), "none");
  llt.attributes()["turn_direction"] = "left";
  EXPECT_EQ(get_turn_direction_in_travel_direction(llt, false, "none"), "left");
  EXPECT_EQ(get_turn_direction_in_travel_direction(llt, true, "none"), "right");
  llt.attributes()["turn_direction"] = "right";
  EXPECT_EQ(get_turn_direction_in_travel_direction(llt, true, "none"), "left");
  llt.attributes()["turn_direction"] = "straight";
  EXPECT_EQ(get_turn_direction_in_travel_direction(llt, true, "none"), "straight");
}

// [TURN-IN-PLACE 2026-11-05] A reversing ego (path velocity negative, yaw = travel direction, body
// facing the other way) belongs to the leg it travels along, not the one its nose points along.
TEST(PathDirectionUtils, reversingEgoKeepsTheLegItTravels)
{
  auto path = out_and_back();
  for (auto & p : path.points) {
    p.point.longitudinal_velocity_mps = -1.0f;
  }
  // At x=1.2 on the outbound (+x) leg, reversing along +x: the body faces -x.
  const auto cut = truncatePathAtFold(path, kThreshold, ego(1.2, 0.0, M_PI));
  ASSERT_TRUE(cut);
  EXPECT_EQ(cut->kept_from, 0u);
  EXPECT_EQ(cut->kept_to, 3u);
  EXPECT_TRUE(cut->stops_at_end);
}
