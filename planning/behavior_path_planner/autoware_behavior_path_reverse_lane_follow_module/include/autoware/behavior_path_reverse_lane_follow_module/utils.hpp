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

#ifndef AUTOWARE__BEHAVIOR_PATH_REVERSE_LANE_FOLLOW_MODULE__UTILS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_REVERSE_LANE_FOLLOW_MODULE__UTILS_HPP_

#include <autoware/route_handler/route_handler.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <memory>
#include <optional>

namespace autoware::behavior_path_planner::reverse_lane_follow_utils
{
using autoware::route_handler::RouteHandler;
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Pose;

// Forward-ordered traveled-path tail plus the lanelet sequence it was built from (the latter is
// needed downstream to (re)generate a drivable area for the retrace path).
struct TraveledTail
{
  PathWithLaneId path;
  lanelet::ConstLanelets lanelets;
};

/**
 * @brief Build the forward-ordered "tail of the already-traversed path" ending at ego's current
 * pose, using only RouteHandler's already-validated lanelet sequence.
 *
 * Deliberately sidesteps `.invert()` / lanelet2 routing-graph queries: the lanelet sequence
 * behind ego was already legally driven forward (RouteHandler tracked it), so re-deriving it
 * raises no new "is this direction allowed" question. See
 * bidirectional_plan/03-phase1-scaffold.md, "Why this needs no legality/routing changes".
 *
 * @param route_handler route handler tracking the currently-active route
 * @param ego_pose current ego pose (the tail path ends here)
 * @param retrace_distance_m how far back (meters) to walk the lanelet sequence
 * @return forward-ordered TraveledTail from (approximately) retrace_distance_m behind ego up to
 *         ego's current pose, or std::nullopt if no such lanelet sequence could be found.
 */
std::optional<TraveledTail> extractTraveledTailPath(
  const std::shared_ptr<RouteHandler> & route_handler, const Pose & ego_pose,
  const double retrace_distance_m);

/**
 * @brief Turn a forward-ordered traveled-path tail into a retrace (reverse) path: reverses point
 * order and negates longitudinal velocity.
 *
 * Reuses the exact sign convention already proven in
 * autoware_freespace_planner/src/autoware_freespace_planner/utils.cpp (~line 135):
 * `longitudinal_velocity_mps = (is_back ? -1 : 1) * point.longitudinal_velocity_mps`. Poses
 * (position + orientation) are left untouched, matching how that same file assigns
 * `point.pose = awp.pose.pose` unmodified for `is_back` waypoints -- only order and velocity sign
 * change. Downstream (shift_decider / PID / MPC) already interpret negative velocity as reverse
 * gear and flip curvature sign accordingly; no orientation flip is needed or wanted here.
 *
 * @param retrace_velocity_mps magnitude (m/s) of the commanded reverse speed
 */
PathWithLaneId reversePathForRetrace(
  const PathWithLaneId & forward_tail_path, const double retrace_velocity_mps);

// Result of buildRouteReversedFollowPath(): a negative-velocity path already in correct
// travel-point-order (see that function's doc comment for why no point-order reversal is
// needed here, unlike reversePathForRetrace), plus the (inverted) lanelet sequence it was built
// from -- needed downstream to (re)generate a drivable area.
struct RouteReversedFollow
{
  PathWithLaneId path;
  lanelet::ConstLanelets lanelets;
};

/**
 * @brief Build a reverse-direction follow path covering the *geometrically continuous* run of
 * lanelets around ego that contains at least one lanelet flagged reversed
 * (RouteHandler::isLaneletInvertedInRoute()), independent of any explicit retrace request. This
 * is the Phase-2 activation trigger added alongside Phase-1's retrace-request trigger
 * (extractTraveledTailPath()/reversePathForRetrace()) -- see
 * bidirectional_plan/04-routing-foundation.md §4d.
 *
 * [BIDIR-BUG-FIX #2] This does NOT gate purely on ego's *current* lanelet being flagged inverted
 * any more. That conflated two different questions: (1) which point-order/orientation convention
 * a single lanelet's centerline should be walked in (what isLaneletInvertedInRoute() is actually
 * for), and (2) whether the vehicle's gear/velocity-sign must be reverse *right now*. A route can
 * cross from an inverted lanelet into a forward-labeled one that continues the exact same
 * physical direction of travel -- e.g. taman map's 18(inv) -> 291(fwd), a ~40 degree curve, not a
 * reversal (confirmed live via lanelet geometry + a frozen repro: ego's actual yaw stayed
 * constant across the boundary while the old label-based trim forced a stop there and handed off
 * to a normal-forward-mode reference demanding an instantaneous 180 degree heading flip --
 * impossible for this vehicle, which never rotates in place -- producing a deterministic
 * yaw_err=pi deadlock). The route's fwd/inv label is about which end of a lanelet's own
 * centerline to start from for geometry purposes; it says nothing about whether the vehicle's
 * physical direction of travel actually reverses at that boundary.
 *
 * Instead, this walks outward from ego's position through the raw lanelet window (see
 * isHeadingContinuousAcross() in the .cpp) and includes every lanelet whose join with its
 * neighbor is a geometrically continuous direction of travel (any ordinary curve, however tight)
 * -- stopping only at a genuine near-180-degree flip, which is the one case this fixed-heading
 * vehicle truly cannot drive through without stopping and changing gear. The resulting window is
 * used as-is (reverse gear, negative velocity) as long as it contains at least one lanelet
 * actually flagged inverted-in-route somewhere; otherwise ego is just on an ordinary forward
 * stretch and this trigger does not apply.
 *
 * Unlike extractTraveledTailPath() (which reverses an already-forward-driven tail path point
 * order), this follows the route's own (auto-flipping) inverted centerline directly via
 * RouteHandler::getCenterLinePath(): lanelet::ConstLanelet::centerline() already returns points
 * in correct travel-direction order for an inverted lanelet (see the Phase-0 spike,
 * InvertedCenterlineIsReversed), so no manual point-order reversal is applied here -- only the
 * velocity sign is flipped negative, reusing the same convention as reversePathForRetrace()/
 * autoware_freespace_planning_algorithms.
 *
 * @return std::nullopt if ego is not within the route, or no geometrically continuous run
 *         touching ego's current lanelet contains an actually-inverted lanelet (i.e. this trigger
 *         does not apply -- the caller should fall back to / keep using the retrace-request
 *         trigger).
 */
std::optional<RouteReversedFollow> buildRouteReversedFollowPath(
  const std::shared_ptr<RouteHandler> & route_handler, const Pose & ego_pose,
  const double backward_distance_m, const double forward_distance_m);

}  // namespace autoware::behavior_path_planner::reverse_lane_follow_utils

#endif  // AUTOWARE__BEHAVIOR_PATH_REVERSE_LANE_FOLLOW_MODULE__UTILS_HPP_
