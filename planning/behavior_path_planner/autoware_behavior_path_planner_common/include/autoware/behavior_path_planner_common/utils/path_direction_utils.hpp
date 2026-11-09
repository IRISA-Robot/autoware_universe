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

#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PATH_DIRECTION_UTILS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PATH_DIRECTION_UTILS_HPP_

// [TURN-IN-PLACE 2026-11-05] Heading-discontinuity helpers shared by the planner manager's
// [PATH-FOLD] handling and the reverse_lane_follow module (moved here from that module's
// utils.cpp, unchanged), so the "is this a ~180 degree seam" question has one answer.

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/primitives/Lanelet.h>

#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace autoware::behavior_path_planner::utils
{
using autoware_internal_planning_msgs::msg::PathWithLaneId;

// A generous bound: allows a walk along the route to continue through even a fairly sharp
// real-world curve/switchback while still catching a genuine ~180 degree reversal.
constexpr double kMaxContinuousHeadingChangeRad = 100.0 * M_PI / 180.0;

/// Tangent direction (radians) of a lanelet's centerline at its very start, walked in whatever
/// point order this ConstLanelet's own .inverted() bit implies (lanelet2's centerline() already
/// returns the point-order-reversed view for an inverted lanelet).
double laneletEntryTangentRad(const lanelet::ConstLanelet & llt);

/// Tangent direction (radians) of a lanelet's centerline at its very end; see
/// laneletEntryTangentRad().
double laneletExitTangentRad(const lanelet::ConstLanelet & llt);

/// Two consecutive lanelets form one continuous direction of travel when the heading change
/// across their shared boundary is below kMaxContinuousHeadingChangeRad -- independent of
/// whether either is flagged inverted in the route.
bool isHeadingContinuousAcross(
  const lanelet::ConstLanelet & from_llt, const lanelet::ConstLanelet & to_llt);

/// Index of the FIRST point where the path folds back on itself: the point at which the
/// outgoing segment starts, when its azimuth differs from the previous (non-degenerate) segment
/// by more than reversal_threshold_rad. Segments shorter than 1 mm carry no direction and are
/// skipped (same convention as the planner manager's [PATH-FOLD] detector).
std::optional<size_t> findFirstDirectionReversalIndex(
  const PathWithLaneId & path, const double reversal_threshold_rad);

/// Every fold point of the path, in order (same definition as findFirstDirectionReversalIndex()).
std::vector<size_t> findAllDirectionReversalIndices(
  const PathWithLaneId & path, const double reversal_threshold_rad);

/// Result of truncatePathAtFold(): the original indices [kept_from, kept_to] that remain.
struct FoldTruncation
{
  size_t kept_from{0};
  size_t kept_to{0};
  bool stops_at_end{false};  ///< a fold lies ahead: the last kept point now has velocity 0
};

/// [TURN-IN-PLACE 2026-11-05] fold_handling="truncate". Keeps only the leg of a folded path that
/// ego is on and stops at that leg's end:
///  - ego's leg is found as the nearest point whose travel direction (outgoing segment; incoming
///    for the last point) is within 90 deg of ego's yaw, so on an out-and-back path ego is
///    matched to the leg it is FACING, not merely the closest one. Without ego_pose (or when no
///    point is aligned) the plain nearest point is used; with no pose at all, the first leg.
///  - points before the last fold at/behind ego are dropped (the fold point itself is kept as the
///    new first point) -- after the robot has turned in place at a fold, the leg it came along is
///    behind it and must not pin the path.
///  - when a fold lies ahead of ego, the path is cut at that fold point (kept) and that point's
///    longitudinal velocity is set to 0, so the vehicle stops at the end of its leg.
/// Orientations are left untouched. Returns nullopt (path untouched) when there is no fold.
std::optional<FoldTruncation> truncatePathAtFold(
  PathWithLaneId & path, const double reversal_threshold_rad,
  const std::optional<geometry_msgs::msg::Pose> & ego_pose);
}  // namespace autoware::behavior_path_planner::utils

#endif  // AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PATH_DIRECTION_UTILS_HPP_
