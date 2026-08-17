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

// Pure-logic unit tests for the retrace path-reversal + sign-convention logic. Deliberately does
// not spin up a ROS node / RouteHandler / lanelet2 map -- see
// autoware/behavior_path_reverse_lane_follow_module/utils.hpp for extractTraveledTailPath(),
// which does need those and is exercised via sim/launch instead (see report).

#include "autoware/behavior_path_reverse_lane_follow_module/utils.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace
{
using autoware::behavior_path_planner::reverse_lane_follow_utils::reversePathForRetrace;
using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using autoware_internal_planning_msgs::msg::PathWithLaneId;

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

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
