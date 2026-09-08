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
using autoware::behavior_path_planner::reverse_lane_follow_utils::applyGoalDecelerationRamp;
using autoware::behavior_path_planner::reverse_lane_follow_utils::capReverseFollowCruiseVelocity;
using autoware::behavior_path_planner::reverse_lane_follow_utils::clampVelocityNearEgoWhenCloseToGoal;
using autoware::behavior_path_planner::reverse_lane_follow_utils::densifyPathNearGoal;
using autoware::behavior_path_planner::reverse_lane_follow_utils::isEgoArrivedAndStoppedAtGoal;
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

// --- capReverseFollowCruiseVelocity() -----------------------------------------------------------
// [BIDIR-BUG-FIX #5] regression coverage (2026-09-06 goal-overshoot investigation, part 1): the
// cap must pull DOWN a raw velocity above the cruise magnitude, but leave an already-slower point
// untouched -- mirroring reversePathForRetrace()'s existing use of a cruise-speed param, but as a
// cap rather than a flat override.

TEST(CapReverseFollowCruiseVelocity, ClampsAboveCruiseDownToCruise)
{
  // Raw magnitude 1.389 m/s (the live-confirmed lanelet speed_limit value from the investigation)
  // must be pulled down to the 1.0 m/s cruise cap, sign preserved.
  auto path = makeForwardPath({0.0, 1.0, 2.0}, -1.389F);
  capReverseFollowCruiseVelocity(path, /*cruise_velocity_mps=*/1.0);

  for (const auto & p : path.points) {
    EXPECT_FLOAT_EQ(p.point.longitudinal_velocity_mps, -1.0F);
  }
}

TEST(CapReverseFollowCruiseVelocity, LeavesBelowCruiseUnaffected)
{
  // A raw velocity already slower than the cruise cap (e.g. through a tight curve, or a lanelet
  // with an even lower speed_limit) must not be sped UP to the cap.
  auto path = makeForwardPath({0.0, 1.0, 2.0}, -0.5F);
  capReverseFollowCruiseVelocity(path, /*cruise_velocity_mps=*/1.0);

  for (const auto & p : path.points) {
    EXPECT_FLOAT_EQ(p.point.longitudinal_velocity_mps, -0.5F);
  }
}

TEST(CapReverseFollowCruiseVelocity, EmptyPathIsNoOp)
{
  PathWithLaneId path;
  capReverseFollowCruiseVelocity(path, 1.0);
  EXPECT_TRUE(path.points.empty());
}

TEST(CapReverseFollowCruiseVelocity, UsesAbsoluteValueOfCruiseParamAndPreservesSign)
{
  // Even if a caller mistakenly passes a negative "cruise magnitude", the cap must still be a
  // positive magnitude applied with the point's own existing velocity sign, not compounded.
  auto path = makeForwardPath({0.0, 1.0}, -2.0F);
  capReverseFollowCruiseVelocity(path, -1.0);
  EXPECT_FLOAT_EQ(path.points.front().point.longitudinal_velocity_mps, -1.0F);
}

// --- applyGoalDecelerationRamp() ----------------------------------------------------------------
// [BIDIR-BUG-FIX #6] regression coverage (2026-09-06 goal-overshoot investigation, part 2): the
// ramp must produce a gradual, monotonically-decreasing-toward-goal velocity profile -- not an
// abrupt single-point drop -- while leaving points far enough from the goal (where the physics
// limit already exceeds cruise speed) untouched, and never exceeding the pre-ramp magnitude.

TEST(ApplyGoalDecelerationRamp, MonotonicallyDecreasesTowardGoalAndLeavesFarPointsAtCruise)
{
  // 11 points, 0.2 m spacing, spanning 2.0 m; all start at cruise speed -1.0 m/s except the last
  // (goal) point, which is already zero -- simulating truncatePathAtGoal()'s output.
  std::vector<double> xs;
  for (int i = 0; i <= 10; ++i) {
    xs.push_back(i * 0.2);
  }
  auto path = makeForwardPath(xs, -1.0F);
  path.points.back().point.longitudinal_velocity_mps = 0.0F;

  applyGoalDecelerationRamp(path, /*decel_mps2=*/0.3);

  // Goal point itself must remain exactly zero.
  EXPECT_FLOAT_EQ(path.points.back().point.longitudinal_velocity_mps, 0.0F);

  // Magnitude must be monotonically non-increasing as points approach the goal (walking forward
  // through the path, i.e. toward higher index / closer to the goal) -- a gradual ramp, not a
  // single abrupt drop at the very last point.
  for (size_t i = 0; i + 1 < path.points.size(); ++i) {
    const float mag_here = std::abs(path.points[i].point.longitudinal_velocity_mps);
    const float mag_next = std::abs(path.points[i + 1].point.longitudinal_velocity_mps);
    EXPECT_GE(mag_here, mag_next - 1e-4F)
      << "velocity magnitude must not increase when moving closer to the goal (index " << i
      << " -> " << i + 1 << ")";
    EXPECT_LE(mag_here, 1.0F + 1e-4F) << "ramp must never exceed the pre-ramp cruise magnitude";
  }

  // At this decel/spacing, points far enough from the goal (indices 0, 1 -- physics limit exceeds
  // cruise speed there) are outside the ramp's effective region and must be left untouched.
  EXPECT_FLOAT_EQ(path.points[0].point.longitudinal_velocity_mps, -1.0F);
  EXPECT_FLOAT_EQ(path.points[1].point.longitudinal_velocity_mps, -1.0F);

  // Points near the goal must be measurably slower than cruise -- confirming the ramp is actually
  // doing something, not silently a no-op.
  EXPECT_LT(std::abs(path.points[9].point.longitudinal_velocity_mps), 1.0F);
  EXPECT_GT(std::abs(path.points[9].point.longitudinal_velocity_mps), 0.0F);
}

TEST(ApplyGoalDecelerationRamp, NoOpWhenDecelIsZero)
{
  auto path = makeForwardPath({0.0, 1.0, 2.0}, -1.0F);
  path.points.back().point.longitudinal_velocity_mps = 0.0F;
  const auto before = path;

  applyGoalDecelerationRamp(path, /*decel_mps2=*/0.0);

  ASSERT_EQ(path.points.size(), before.points.size());
  for (size_t i = 0; i < path.points.size(); ++i) {
    EXPECT_FLOAT_EQ(
      path.points[i].point.longitudinal_velocity_mps,
      before.points[i].point.longitudinal_velocity_mps);
  }
}

TEST(ApplyGoalDecelerationRamp, NoOpOnSinglePointPath)
{
  auto path = makeForwardPath({0.0}, -1.0F);
  applyGoalDecelerationRamp(path, /*decel_mps2=*/0.3);
  ASSERT_EQ(path.points.size(), 1U);
  EXPECT_FLOAT_EQ(path.points[0].point.longitudinal_velocity_mps, -1.0F);
}

TEST(ApplyGoalDecelerationRamp, EmptyPathIsNoOp)
{
  PathWithLaneId path;
  applyGoalDecelerationRamp(path, 0.3);
  EXPECT_TRUE(path.points.empty());
}

TEST(ApplyGoalDecelerationRamp, FixesNonMonotonicRawSpeedViaForwardCleanupPass)
{
  // [BIDIR-BUG-FIX #7] Reproduces the exact live-observed failure (2026-09-07 continued-overshoot
  // investigation): the native centerline speed is NOT uniform along the path -- points several
  // meters from the goal sit on a naturally-slower upstream curve (-0.833 m/s), while the single
  // native waypoint closer to the goal (~2.05m away) sits on a naturally-faster straight stretch
  // (-1.1455 m/s, the live-confirmed value). The backward physics pass alone clamps that near-goal
  // point only down to ~1.109 m/s (still physically valid in isolation -- it CAN brake to 0 over
  // 2.05m at 0.3 m/s^2 from that speed), which is still faster than the -0.833 m/s point behind it
  // -- a real, non-monotonic "slows down, then speeds back up, then instant stop" bump. The
  // forward clean-up pass this fix adds must clip that bump down to match the slower point behind
  // it, producing a genuinely monotonic (non-increasing toward the goal) profile.
  PathWithLaneId path;
  const std::vector<std::pair<double, float>> raw = {
    {0.0, -0.833F}, {1.0, -0.833F}, {2.0, -0.833F}, {2.6494, -1.1455F}, {4.6994, 0.0F}};
  for (const auto & [x, v] : raw) {
    PathPointWithLaneId p;
    p.point.pose.position.x = x;
    p.point.pose.orientation.w = 1.0;
    p.point.longitudinal_velocity_mps = v;
    path.points.push_back(p);
  }

  applyGoalDecelerationRamp(path, /*decel_mps2=*/0.3);

  EXPECT_FLOAT_EQ(path.points.back().point.longitudinal_velocity_mps, 0.0F);
  for (size_t i = 0; i + 1 < path.points.size(); ++i) {
    const float here = std::abs(path.points[i].point.longitudinal_velocity_mps);
    const float next = std::abs(path.points[i + 1].point.longitudinal_velocity_mps);
    EXPECT_GE(here, next - 1e-4F)
      << "velocity magnitude must not increase approaching the goal (index " << i << " -> "
      << i + 1 << ") even when the raw/cruise-capped input speed is itself non-monotonic";
  }
  // The near-goal point must have been pulled down to (at most) the slower point behind it, not
  // left at its raw 1.1455 m/s or the backward-pass-only ~1.109 m/s.
  EXPECT_LE(std::abs(path.points[3].point.longitudinal_velocity_mps), 0.833F + 1e-4F);
}

// --- densifyPathNearGoal() -----------------------------------------------------------------------
// [BIDIR-BUG-FIX #7] regression coverage (2026-09-07 continued-overshoot investigation): the ramp
// can only set velocities at EXISTING waypoints, so a coarse native spacing near the goal (live-
// confirmed ~2m) leaves it almost no room to act before the hard zero-velocity point. These tests
// cover densifyPathNearGoal() in isolation, then a combined regression test reproducing the exact
// live-observed non-monotonic velocity bump and confirming densify+ramp together fix it.

TEST(DensifyPathNearGoal, InsertsIntermediatePointsWhenSpacingExceedsMax)
{
  // Segments: 0->1 (1m), 1->2 (1m), 2->4 (2m, exceeds max_spacing_m=0.5 -> needs 4 extra points to
  // reach 0.5m spacing). max_span_m=2.0 deliberately scopes densification to just the final 2->4
  // segment (exactly covers it, nothing more) so the 0->1/1->2 segments are left untouched -- see
  // the separate OnlyDensifiesWithinMaxSpanOfGoal / AlwaysDensifiesFinalSegmentEvenIfAloneExceeds
  // MaxSpan tests for span-boundary behavior in isolation.
  auto path = makeForwardPath({0.0, 1.0, 2.0, 4.0}, -1.0F);
  densifyPathNearGoal(path, /*max_span_m=*/2.0, /*max_spacing_m=*/0.5);

  // Untouched prefix (2 points) + the densified final segment: original 2 endpoints + 4 new
  // interpolated points = 6 points there, for a total of 8.
  ASSERT_EQ(path.points.size(), 8U);
  EXPECT_NEAR(path.points[0].point.pose.position.x, 0.0, 1e-6);
  EXPECT_NEAR(path.points[1].point.pose.position.x, 1.0, 1e-6);
  // Endpoints of the densified segment are preserved exactly.
  EXPECT_NEAR(path.points[2].point.pose.position.x, 2.0, 1e-6);
  EXPECT_NEAR(path.points[7].point.pose.position.x, 4.0, 1e-6);
  // The 4 inserted points are evenly spaced at 0.4 m (2.0m / 5 sub-segments).
  EXPECT_NEAR(path.points[3].point.pose.position.x, 2.4, 1e-6);
  EXPECT_NEAR(path.points[4].point.pose.position.x, 2.8, 1e-6);
  EXPECT_NEAR(path.points[5].point.pose.position.x, 3.2, 1e-6);
  EXPECT_NEAR(path.points[6].point.pose.position.x, 3.6, 1e-6);
  // No point within the densified region is spaced farther than max_spacing_m from its neighbor.
  for (size_t i = 2; i + 1 < path.points.size(); ++i) {
    EXPECT_LE(
      std::abs(path.points[i + 1].point.pose.position.x - path.points[i].point.pose.position.x),
      0.5 + 1e-6);
  }
}

TEST(DensifyPathNearGoal, LeavesAlreadyDenseSegmentsUnchanged)
{
  auto path = makeForwardPath({0.0, 0.3, 0.6, 0.9}, -1.0F);
  const auto before = path;

  densifyPathNearGoal(path, /*max_span_m=*/10.0, /*max_spacing_m=*/0.5);

  ASSERT_EQ(path.points.size(), before.points.size());
  for (size_t i = 0; i < path.points.size(); ++i) {
    EXPECT_NEAR(
      path.points[i].point.pose.position.x, before.points[i].point.pose.position.x, 1e-9);
  }
}

TEST(DensifyPathNearGoal, OnlyDensifiesWithinMaxSpanOfGoal)
{
  // A big 5m gap far from the goal (0->5) must be left alone once max_span_m only reaches back
  // 2m from the goal (points.back() at x=7, so the span covers segments 6->7 and 5->6 only,
  // exactly 2.0m of cumulative arc length) -- the far 0->5 segment falls entirely outside that
  // span and must not be touched (it may or may not itself need subdividing, but that is not this
  // test's concern -- see InsertsIntermediatePointsWhenSpacingExceedsMax for subdivision-shape
  // coverage).
  auto path = makeForwardPath({0.0, 5.0, 6.0, 7.0}, -1.0F);
  densifyPathNearGoal(path, /*max_span_m=*/2.0, /*max_spacing_m=*/0.5);

  // The far, un-reached 5m segment (0->5) must be completely untouched -- points[0] and points[1]
  // are exactly the original x=0.0/x=5.0 points, unmodified.
  ASSERT_GE(path.points.size(), 2U);
  EXPECT_NEAR(path.points[0].point.pose.position.x, 0.0, 1e-6);
  EXPECT_NEAR(path.points[1].point.pose.position.x, 5.0, 1e-6);
}

TEST(DensifyPathNearGoal, AlwaysDensifiesFinalSegmentEvenIfAloneExceedsMaxSpan)
{
  // A single 5m final segment, with max_span_m deliberately smaller (2.0) than that segment's own
  // length -- this is exactly the pathological "one huge terminal segment" case that must still
  // be densified rather than silently skipped (a lone long final segment right at the goal is the
  // live-confirmed failure mode this function exists to close).
  auto path = makeForwardPath({0.0, 5.0}, -1.0F);
  densifyPathNearGoal(path, /*max_span_m=*/2.0, /*max_spacing_m=*/0.5);

  EXPECT_GT(path.points.size(), 2U);
  for (size_t i = 0; i + 1 < path.points.size(); ++i) {
    EXPECT_LE(
      std::abs(path.points[i + 1].point.pose.position.x - path.points[i].point.pose.position.x),
      0.5 + 1e-6);
  }
}

TEST(DensifyPathNearGoal, EmptyPathIsNoOp)
{
  PathWithLaneId path;
  densifyPathNearGoal(path, 10.0, 0.5);
  EXPECT_TRUE(path.points.empty());
}

TEST(DensifyPathNearGoal, SinglePointPathIsNoOp)
{
  auto path = makeForwardPath({0.0}, -1.0F);
  densifyPathNearGoal(path, 10.0, 0.5);
  ASSERT_EQ(path.points.size(), 1U);
}

TEST(DensifyPathNearGoal, NoOpWhenMaxSpacingIsZero)
{
  auto path = makeForwardPath({0.0, 5.0}, -1.0F);
  densifyPathNearGoal(path, /*max_span_m=*/10.0, /*max_spacing_m=*/0.0);
  ASSERT_EQ(path.points.size(), 2U);
}

TEST(DensifyPathNearGoal, CombinedWithRampProducesGradualProfileNotJustFlatThenCliff)
{
  // Reproduces the exact live-observed raw-speed pattern (2026-09-07 continued-overshoot
  // investigation): a naturally-slower upstream stretch (-0.833 m/s) followed by a single native
  // waypoint ~2.05m from the goal on a naturally-faster stretch (-1.1455 m/s, the live-confirmed
  // value). ApplyGoalDecelerationRamp()'s own forward clean-up pass (see that suite's
  // FixesNonMonotonicRawSpeedViaForwardCleanupPass test) already guarantees a SAFE, monotonic
  // result on its own even without densifying first -- but with only one native waypoint in the
  // final ~2m stretch to work with, that safe result is a "flat cruise, then a single hard drop to
  // zero" cliff, not a gradual slowdown. densifyPathNearGoal(), run first, gives the ramp enough
  // intermediate waypoints in that same final stretch to shape a genuinely gradual, multi-step
  // decrease instead -- this is the actual value densify adds on top of (not instead of) the
  // forward clean-up pass fix.
  auto make_repro = [] {
    PathWithLaneId path;
    const std::vector<std::pair<double, float>> raw = {
      {0.0, -0.833F}, {1.0, -0.833F}, {2.0, -0.833F}, {2.6494, -1.1455F}, {4.6994, 0.0F}};
    for (const auto & [x, v] : raw) {
      PathPointWithLaneId p;
      p.point.pose.position.x = x;
      p.point.pose.orientation.w = 1.0;
      p.point.longitudinal_velocity_mps = v;
      path.points.push_back(p);
    }
    return path;
  };

  // Ramp alone (no densify): monotonic (per the other test), but only ever a single intermediate
  // step size within the final stretch -- count distinct velocity values strictly between 0 and
  // cruise (0.833) among the points closest to the goal.
  auto path_ramp_only = make_repro();
  applyGoalDecelerationRamp(path_ramp_only, /*decel_mps2=*/0.3);
  int distinct_intermediate_steps_ramp_only = 0;
  for (const auto & p : path_ramp_only.points) {
    const float mag = std::abs(p.point.longitudinal_velocity_mps);
    if (mag > 1e-3F && mag < 0.833F - 1e-3F) {
      ++distinct_intermediate_steps_ramp_only;
    }
  }
  EXPECT_EQ(distinct_intermediate_steps_ramp_only, 0)
    << "sanity check: without densify, the only near-goal waypoint gets clipped straight down to "
       "the cruise/predecessor speed (0.833) by the forward clean-up pass -- no room for a "
       "gradual intermediate step";

  // Densify + ramp together: must produce at least one genuinely graduated intermediate step.
  auto path_densified = make_repro();
  densifyPathNearGoal(path_densified, /*max_span_m=*/3.215, /*max_spacing_m=*/0.5);
  applyGoalDecelerationRamp(path_densified, /*decel_mps2=*/0.3);

  EXPECT_FLOAT_EQ(path_densified.points.back().point.longitudinal_velocity_mps, 0.0F);
  int distinct_intermediate_steps_densified = 0;
  for (const auto & p : path_densified.points) {
    const float mag = std::abs(p.point.longitudinal_velocity_mps);
    if (mag > 1e-3F && mag < 0.833F - 1e-3F) {
      ++distinct_intermediate_steps_densified;
    }
  }
  EXPECT_GT(distinct_intermediate_steps_densified, 0)
    << "densify+ramp must produce at least one gradual intermediate velocity step near the goal, "
       "not just a flat-cruise-then-cliff profile";

  // Still fully monotonic (magnitude non-increasing toward the goal), same requirement as the
  // ramp-alone case.
  for (size_t i = 0; i + 1 < path_densified.points.size(); ++i) {
    const float here = std::abs(path_densified.points[i].point.longitudinal_velocity_mps);
    const float next = std::abs(path_densified.points[i + 1].point.longitudinal_velocity_mps);
    EXPECT_GE(here, next - 1e-4F) << "velocity magnitude must not increase approaching the goal "
                                     "(index "
                                  << i << " -> " << i + 1 << ") once densified first";
  }
}

// --- clampVelocityNearEgoWhenCloseToGoal() -------------------------------------------------------
// [BIDIR-BUG-FIX #8] regression coverage (2026-09-07 continued-overshoot investigation): the
// reactive, closed-loop safety layer requested by the user as defense-in-depth alongside the
// densify/ramp fix above. Must engage only when both the goal is within the current follow window
// AND ego is genuinely close to it, must only ever reduce (never increase) velocity, and must only
// touch points near ego's current position.

TEST(ClampVelocityNearEgoWhenCloseToGoal, NoOpWhenGoalNotInFollowWindow)
{
  auto path = makeForwardPath({0.0, 1.0, 2.0}, -1.389F);
  const auto before = path;
  clampVelocityNearEgoWhenCloseToGoal(
    path, makePose(0.5, 0.0), /*goal_within_follow_window=*/false, /*distance_to_goal_m=*/0.1,
    /*trigger_distance_m=*/2.0, /*clamp_velocity_mps=*/0.5);

  for (size_t i = 0; i < path.points.size(); ++i) {
    EXPECT_FLOAT_EQ(
      path.points[i].point.longitudinal_velocity_mps,
      before.points[i].point.longitudinal_velocity_mps);
  }
}

TEST(ClampVelocityNearEgoWhenCloseToGoal, NoOpWhenFarFromGoalEvenIfGoalInWindow)
{
  auto path = makeForwardPath({0.0, 1.0, 2.0}, -1.389F);
  const auto before = path;
  clampVelocityNearEgoWhenCloseToGoal(
    path, makePose(0.5, 0.0), /*goal_within_follow_window=*/true, /*distance_to_goal_m=*/5.0,
    /*trigger_distance_m=*/2.0, /*clamp_velocity_mps=*/0.5);

  for (size_t i = 0; i < path.points.size(); ++i) {
    EXPECT_FLOAT_EQ(
      path.points[i].point.longitudinal_velocity_mps,
      before.points[i].point.longitudinal_velocity_mps);
  }
}

TEST(ClampVelocityNearEgoWhenCloseToGoal, ExactlyAtTriggerDistanceBoundaryDoesNotEngage)
{
  // Strict "<" convention, matching shouldSuppressReversedFollowReactivation()'s own
  // goal_reach_tolerance_m boundary handling.
  auto path = makeForwardPath({0.0}, -1.389F);
  clampVelocityNearEgoWhenCloseToGoal(
    path, makePose(0.0, 0.0), /*goal_within_follow_window=*/true, /*distance_to_goal_m=*/2.0,
    /*trigger_distance_m=*/2.0, /*clamp_velocity_mps=*/0.5);
  EXPECT_FLOAT_EQ(path.points[0].point.longitudinal_velocity_mps, -1.389F);
}

TEST(ClampVelocityNearEgoWhenCloseToGoal, ClampsOnlyPointsNearEgoWhenEngaged)
{
  // ego sits at x=5; points within 2m of ego (x in [3,7]) must be clamped, points farther away
  // (x=0) must be left alone -- "near ego's current position", not the whole follow window.
  auto path = makeForwardPath({0.0, 4.0, 5.0, 6.0}, -1.389F);
  clampVelocityNearEgoWhenCloseToGoal(
    path, makePose(5.0, 0.0), /*goal_within_follow_window=*/true, /*distance_to_goal_m=*/1.0,
    /*trigger_distance_m=*/2.0, /*clamp_velocity_mps=*/0.5);

  EXPECT_FLOAT_EQ(path.points[0].point.longitudinal_velocity_mps, -1.389F)
    << "point far from ego (10m away) must not be touched";
  EXPECT_FLOAT_EQ(path.points[1].point.longitudinal_velocity_mps, -0.5F);
  EXPECT_FLOAT_EQ(path.points[2].point.longitudinal_velocity_mps, -0.5F);
  EXPECT_FLOAT_EQ(path.points[3].point.longitudinal_velocity_mps, -0.5F);
}

TEST(ClampVelocityNearEgoWhenCloseToGoal, NeverIncreasesAnAlreadySlowPoint)
{
  auto path = makeForwardPath({0.0}, -0.2F);
  clampVelocityNearEgoWhenCloseToGoal(
    path, makePose(0.0, 0.0), /*goal_within_follow_window=*/true, /*distance_to_goal_m=*/0.5,
    /*trigger_distance_m=*/2.0, /*clamp_velocity_mps=*/0.5);
  EXPECT_FLOAT_EQ(path.points[0].point.longitudinal_velocity_mps, -0.2F);
}

TEST(ClampVelocityNearEgoWhenCloseToGoal, PreservesSignConvention)
{
  auto path = makeForwardPath({0.0}, 1.389F);  // positive (forward-sign) velocity
  clampVelocityNearEgoWhenCloseToGoal(
    path, makePose(0.0, 0.0), /*goal_within_follow_window=*/true, /*distance_to_goal_m=*/0.5,
    /*trigger_distance_m=*/2.0, /*clamp_velocity_mps=*/0.5);
  EXPECT_FLOAT_EQ(path.points[0].point.longitudinal_velocity_mps, 0.5F);
}

TEST(ClampVelocityNearEgoWhenCloseToGoal, EmptyPathIsNoOp)
{
  PathWithLaneId path;
  clampVelocityNearEgoWhenCloseToGoal(
    path, makePose(0.0, 0.0), true, 0.5, 2.0, 0.5);
  EXPECT_TRUE(path.points.empty());
}

// --- isEgoArrivedAndStoppedAtGoal() --------------------------------------------------------------
// [BIDIR-BUG-FIX #10] regression coverage (2026-09-07 ARRIVED-state investigation): this is the
// pure decision function that fixes ReverseLaneFollowModule::canTransitSuccessState()'s
// route_reversed_path branch, which previously returned `false` unconditionally and therefore
// never let this module (or, downstream, mission_planner's arrival check) report success once
// genuinely parked at the goal in reverse-follow mode.

TEST(IsEgoArrivedAndStoppedAtGoal, TrueWhenWithinToleranceAndStopped)
{
  EXPECT_TRUE(isEgoArrivedAndStoppedAtGoal(
    /*distance_to_goal_m=*/0.3, /*ego_speed_mps=*/0.01, /*goal_reach_tolerance_m=*/1.0,
    /*arrived_stop_velocity_mps=*/0.05));
}

TEST(IsEgoArrivedAndStoppedAtGoal, FalseWhenStillMovingEvenIfWithinTolerance)
{
  // Position looks "arrived", but ego is still rolling faster than the stop threshold -- must not
  // report arrived yet.
  EXPECT_FALSE(isEgoArrivedAndStoppedAtGoal(
    /*distance_to_goal_m=*/0.3, /*ego_speed_mps=*/0.5, /*goal_reach_tolerance_m=*/1.0,
    /*arrived_stop_velocity_mps=*/0.05));
}

TEST(IsEgoArrivedAndStoppedAtGoal, FalseWhenStoppedButStillFarFromGoal)
{
  // Ego could be stopped for an unrelated reason (traffic light, obstacle) well before the goal --
  // must not report arrived purely because speed is low.
  EXPECT_FALSE(isEgoArrivedAndStoppedAtGoal(
    /*distance_to_goal_m=*/5.0, /*ego_speed_mps=*/0.0, /*goal_reach_tolerance_m=*/1.0,
    /*arrived_stop_velocity_mps=*/0.05));
}

TEST(IsEgoArrivedAndStoppedAtGoal, ExactlyAtToleranceBoundaryIsNotArrived)
{
  // Strict `<` (not `<=`), mirroring shouldSuppressReversedFollowReactivation()'s own
  // ExactlyAtToleranceBoundaryStillSuppresses convention for the same goal_reach_tolerance_m param.
  EXPECT_FALSE(isEgoArrivedAndStoppedAtGoal(
    /*distance_to_goal_m=*/1.0, /*ego_speed_mps=*/0.0, /*goal_reach_tolerance_m=*/1.0,
    /*arrived_stop_velocity_mps=*/0.05));
}

TEST(IsEgoArrivedAndStoppedAtGoal, ExactlyAtStopVelocityBoundaryIsNotArrived)
{
  EXPECT_FALSE(isEgoArrivedAndStoppedAtGoal(
    /*distance_to_goal_m=*/0.3, /*ego_speed_mps=*/0.05, /*goal_reach_tolerance_m=*/1.0,
    /*arrived_stop_velocity_mps=*/0.05));
}

TEST(IsEgoArrivedAndStoppedAtGoal, NegativeEgoSpeedIsTreatedAsMagnitude)
{
  // Reverse-mode ego speed may be reported/derived as a negative x-twist -- must not let a
  // negative sign accidentally satisfy "< arrived_stop_velocity_mps" for a speed that is actually
  // large in magnitude.
  EXPECT_FALSE(isEgoArrivedAndStoppedAtGoal(
    /*distance_to_goal_m=*/0.3, /*ego_speed_mps=*/-0.5, /*goal_reach_tolerance_m=*/1.0,
    /*arrived_stop_velocity_mps=*/0.05));
  EXPECT_TRUE(isEgoArrivedAndStoppedAtGoal(
    /*distance_to_goal_m=*/0.3, /*ego_speed_mps=*/-0.01, /*goal_reach_tolerance_m=*/1.0,
    /*arrived_stop_velocity_mps=*/0.05));
}

TEST(IsEgoArrivedAndStoppedAtGoal, LiveConfirmedResidualCreepDoesNotBlockArrival)
{
  // Live-confirmed peak residual creep from the bug this fixes was ~0.00105 m/s (see
  // docs/research/reverse-lane-follow-goal-overshoot-fix.md, 2026-09-07 planning-engineer
  // section) -- the default arrived_stop_velocity_mps (0.05) must comfortably clear it, unlike
  // autoware_motion_utils::VehicleStopCheckerBase's own much stricter 1mm/s bound.
  EXPECT_TRUE(isEgoArrivedAndStoppedAtGoal(
    /*distance_to_goal_m=*/0.6, /*ego_speed_mps=*/0.00105, /*goal_reach_tolerance_m=*/1.0,
    /*arrived_stop_velocity_mps=*/0.05));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
