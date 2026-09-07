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

// Pure-logic unit tests for the retrace path-reversal + sign-convention logic, the one-way
// reactivation-latch decision function, and the exact-goal path truncation. Deliberately does not
// spin up a ROS node / RouteHandler / lanelet2 map -- see
// autoware/behavior_path_reverse_lane_follow_module/utils.hpp for extractTraveledTailPath() and
// buildRouteReversedFollowPath(), which do need those and are exercised via sim/launch instead
// (see report). shouldSuppressReversedFollowReactivation() and truncatePathAtGoal() were both
// deliberately factored out as standalone pure functions specifically so this latch/goal-overshoot
// fix could be unit-tested without that heavier fixture.

#include "autoware/behavior_path_reverse_lane_follow_module/utils.hpp"

#include <lanelet2_core/primitives/Lanelet.h>

#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <vector>

namespace
{
using autoware::behavior_path_planner::reverse_lane_follow_utils::reversePathForRetrace;
using autoware::behavior_path_planner::reverse_lane_follow_utils::
  shouldSuppressReversedFollowReactivation;
using autoware::behavior_path_planner::reverse_lane_follow_utils::truncatePathAtGoal;
using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Pose;

PathWithLaneId makeForwardPath(const std::vector<double> & xs, const float forward_speed_mps)
{
  PathWithLaneId path;
  for (const auto x : xs) {
    PathPointWithLaneId p;
    p.point.pose.position.x = x;
    p.point.pose.position.y = 0.0;
    p.point.pose.position.z = 0.0;
    p.point.pose.orientation.w = 1.0;
    p.point.longitudinal_velocity_mps = forward_speed_mps;
    path.points.push_back(p);
  }
  return path;
}

// Minimal lanelet::ConstLanelet with a controllable id, for exercising
// shouldSuppressReversedFollowReactivation() without any lanelet2 map/routing-graph setup --
// that function only ever inspects .id(), so the geometry here is a throwaway straight segment.
lanelet::ConstLanelet makeLaneletWithId(const lanelet::Id id)
{
  lanelet::LineString3d left(
    id * 100 + 1,
    {lanelet::Point3d(id * 100 + 3, 0.0, 0.0, 0.0), lanelet::Point3d(id * 100 + 4, 1.0, 0.0, 0.0)});
  lanelet::LineString3d right(
    id * 100 + 2,
    {lanelet::Point3d(id * 100 + 5, 0.0, 1.0, 0.0), lanelet::Point3d(id * 100 + 6, 1.0, 1.0, 0.0)});
  return lanelet::Lanelet(id, left, right);
}

Pose makePose(const double x, const double y)
{
  Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0.0;
  pose.orientation.w = 1.0;
  return pose;
}

}  // namespace

TEST(ReverseLaneFollowUtils, ReversesPointOrder)
{
  const auto forward_tail = makeForwardPath({0.0, 1.0, 2.0, 3.0}, 1.5F);
  const auto retrace = reversePathForRetrace(forward_tail, 1.0);

  ASSERT_EQ(retrace.points.size(), forward_tail.points.size());
  for (size_t i = 0; i < retrace.points.size(); ++i) {
    EXPECT_DOUBLE_EQ(
      retrace.points[i].point.pose.position.x,
      forward_tail.points[forward_tail.points.size() - 1 - i].point.pose.position.x);
  }
}

TEST(ReverseLaneFollowUtils, LeavesPoseUntouchedOnlyFlipsVelocitySign)
{
  // Matches the exact convention proven in autoware_freespace_planner's utils.cpp
  // (~line 135): `(is_back ? -1 : 1) * point.longitudinal_velocity_mps`, where the pose itself
  // (`point.pose = awp.pose.pose`) is copied unmodified.
  const auto forward_tail = makeForwardPath({0.0, 1.0, 2.0}, 2.0F);
  const auto retrace = reversePathForRetrace(forward_tail, 0.8);

  // All but the last point (which is forced to a stop, see below) get the requested magnitude,
  // negative-signed.
  for (size_t i = 0; i + 1 < retrace.points.size(); ++i) {
    EXPECT_FLOAT_EQ(retrace.points[i].point.longitudinal_velocity_mps, -0.8F);
    EXPECT_LT(retrace.points[i].point.longitudinal_velocity_mps, 0.0F);
  }

  // orientation must be untouched (no yaw-flip invented)
  for (size_t i = 0; i < retrace.points.size(); ++i) {
    EXPECT_DOUBLE_EQ(retrace.points[i].point.pose.orientation.w, 1.0);
  }
}

TEST(ReverseLaneFollowUtils, EndsWithZeroVelocityStop)
{
  const auto forward_tail = makeForwardPath({0.0, 1.0, 2.0}, 1.0F);
  const auto retrace = reversePathForRetrace(forward_tail, 1.0);

  ASSERT_FALSE(retrace.points.empty());
  EXPECT_FLOAT_EQ(retrace.points.back().point.longitudinal_velocity_mps, 0.0F);
}

TEST(ReverseLaneFollowUtils, UsesAbsoluteValueOfRequestedVelocity)
{
  // Even if a caller mistakenly passes a negative "velocity magnitude", the sign convention must
  // still be applied on top of the magnitude, not compounded.
  const auto forward_tail = makeForwardPath({0.0, 1.0}, 1.0F);
  const auto retrace = reversePathForRetrace(forward_tail, -0.5);

  EXPECT_FLOAT_EQ(retrace.points.front().point.longitudinal_velocity_mps, -0.5F);
}

// --- shouldSuppressReversedFollowReactivation() -----------------------------------------------
// [BIDIR-BUG-FIX #3] regression coverage: (a) the latch must still suppress reactivation for a
// lanelet far from the goal (this is the original 6fafd8ede fix's intent -- must not regress),
// (b) the latch must NOT suppress reactivation once ego is near the goal, even for a
// previously-latched lanelet.

TEST(ShouldSuppressReversedFollowReactivation, NoLatchArmedNeverSuppresses)
{
  const std::vector<lanelet::ConstLanelet> follow_lanelets = {makeLaneletWithId(42)};
  EXPECT_FALSE(shouldSuppressReversedFollowReactivation(
    std::nullopt, follow_lanelets, /*distance_to_goal_m=*/100.0, /*goal_reach_tolerance_m=*/0.5));
}

TEST(ShouldSuppressReversedFollowReactivation, LatchedLaneletNotInFollowWindowNeverSuppresses)
{
  const std::vector<lanelet::ConstLanelet> follow_lanelets = {makeLaneletWithId(42)};
  EXPECT_FALSE(shouldSuppressReversedFollowReactivation(
    std::optional<lanelet::Id>(99), follow_lanelets, /*distance_to_goal_m=*/100.0,
    /*goal_reach_tolerance_m=*/0.5));
}

TEST(ShouldSuppressReversedFollowReactivation, SuppressesFarFromGoal_RegressionFor6fafd8ede)
{
  // This is the exact scenario the one-way latch was originally introduced for: ego re-enters a
  // lanelet it just (spuriously) exited, nowhere near the actual goal -- must still be suppressed.
  const std::vector<lanelet::ConstLanelet> follow_lanelets = {makeLaneletWithId(7),
                                                                makeLaneletWithId(42)};
  EXPECT_TRUE(shouldSuppressReversedFollowReactivation(
    std::optional<lanelet::Id>(42), follow_lanelets, /*distance_to_goal_m=*/100.0,
    /*goal_reach_tolerance_m=*/0.5));
}

TEST(ShouldSuppressReversedFollowReactivation, DoesNotSuppressWhenNearGoal)
{
  // Same latched/follow-window setup as the regression test above, but ego is now within
  // goal_reach_tolerance_m of the goal -- the escape hatch must allow reactivation so the
  // goal-truncation stop point in buildRouteReversedFollowPath() is not permanently lost.
  const std::vector<lanelet::ConstLanelet> follow_lanelets = {makeLaneletWithId(7),
                                                                makeLaneletWithId(42)};
  EXPECT_FALSE(shouldSuppressReversedFollowReactivation(
    std::optional<lanelet::Id>(42), follow_lanelets, /*distance_to_goal_m=*/0.2,
    /*goal_reach_tolerance_m=*/0.5));
}

TEST(ShouldSuppressReversedFollowReactivation, ExactlyAtToleranceBoundaryStillSuppresses)
{
  // distance_to_goal_m == goal_reach_tolerance_m is NOT "near enough" (strict less-than), matching
  // canTransitSuccessState()'s own strict "<" convention for goal_reach_tolerance_m.
  const std::vector<lanelet::ConstLanelet> follow_lanelets = {makeLaneletWithId(42)};
  EXPECT_TRUE(shouldSuppressReversedFollowReactivation(
    std::optional<lanelet::Id>(42), follow_lanelets, /*distance_to_goal_m=*/0.5,
    /*goal_reach_tolerance_m=*/0.5));
}

// --- truncatePathAtGoal() ----------------------------------------------------------------------
// [BIDIR-BUG-FIX #4] regression coverage: the truncation must land (approximately) exactly at the
// true goal pose, not at the nearest existing sample, for a path whose nearest sample is
// deliberately offset from the goal.

TEST(TruncatePathAtGoal, LandsAtExactGoalNotNearestSample)
{
  // Samples at x = 0, 1, 2, 3, 4 (1 m spacing); goal sits at x = 2.4, deliberately NOT coincident
  // with any existing sample -- the nearest existing sample (x = 2) is 0.4 m short of the goal.
  auto path = makeForwardPath({0.0, 1.0, 2.0, 3.0, 4.0}, 1.0F);
  const auto goal_pose = makePose(2.4, 0.0);

  truncatePathAtGoal(path, goal_pose);

  ASSERT_FALSE(path.points.empty());
  const auto & end_point = path.points.back().point;
  EXPECT_NEAR(end_point.pose.position.x, 2.4, 1e-2);
  EXPECT_FLOAT_EQ(end_point.longitudinal_velocity_mps, 0.0F);
  // Nothing beyond the goal should survive the truncation.
  for (const auto & p : path.points) {
    EXPECT_LE(p.point.pose.position.x, 2.4 + 1e-2);
  }
}

TEST(TruncatePathAtGoal, EmptyPathIsNoOp)
{
  PathWithLaneId path;
  truncatePathAtGoal(path, makePose(1.0, 0.0));
  EXPECT_TRUE(path.points.empty());
}

TEST(TruncatePathAtGoal, GoalAtExistingSampleStillEndsWithZeroVelocity)
{
  auto path = makeForwardPath({0.0, 1.0, 2.0}, 1.0F);
  truncatePathAtGoal(path, makePose(1.0, 0.0));

  ASSERT_FALSE(path.points.empty());
  EXPECT_NEAR(path.points.back().point.pose.position.x, 1.0, 1e-2);
  EXPECT_FLOAT_EQ(path.points.back().point.longitudinal_velocity_mps, 0.0F);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
