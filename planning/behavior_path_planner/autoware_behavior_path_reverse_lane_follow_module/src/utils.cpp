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

#include "autoware/behavior_path_reverse_lane_follow_module/utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

namespace autoware::behavior_path_planner::reverse_lane_follow_utils
{

namespace
{
// [BIDIR-BUG-FIX #2] Tangent direction (radians) of a lanelet's centerline at its very start/end,
// walked in whatever point order this ConstLanelet's own .inverted() bit implies.
// lanelet2_core's Lanelet::centerline() already returns an inverted (point-order-reversed) view
// when .inverted() is true -- same mechanism route_handler.cpp documents for
// leftBound()/rightBound() (Lanelet.h:164-175) -- so these tangents already point in the
// lanelet's *route-consistent* direction of travel, regardless of its fwd/inv route label.
double laneletEntryTangentRad(const lanelet::ConstLanelet & llt)
{
  const auto & centerline = llt.centerline();
  if (centerline.size() < 2) {
    return 0.0;
  }
  const auto & p0 = centerline[0];
  const auto & p1 = centerline[1];
  return std::atan2(p1.y() - p0.y(), p1.x() - p0.x());
}

double laneletExitTangentRad(const lanelet::ConstLanelet & llt)
{
  const auto & centerline = llt.centerline();
  const size_t n = centerline.size();
  if (n < 2) {
    return 0.0;
  }
  const auto & p0 = centerline[n - 2];
  const auto & p1 = centerline[n - 1];
  return std::atan2(p1.y() - p0.y(), p1.x() - p0.x());
}

// A generous bound: allows the window-extension walk below to continue through even a fairly
// sharp real-world curve/switchback while still catching a genuine ~180 degree reversal -- the
// one case this vehicle (fixed heading, forward/backward translation only, never rotates in
// place) truly cannot drive through without stopping and changing gear.
constexpr double kMaxContinuousHeadingChangeRad = 100.0 * M_PI / 180.0;

// [BIDIR-BUG-FIX #2] Replaces the old isLaneletInvertedInRoute()-based "same label" trim: two
// adjacent-in-route lanelets are treated as one continuous physical maneuver based purely on
// whether their shared boundary is a smooth direction of travel, independent of whether either
// side is flagged inverted in the route. See buildRouteReversedFollowPath()'s doc comment in
// utils.hpp for the full rationale and live evidence.
bool isHeadingContinuousAcross(
  const lanelet::ConstLanelet & from_llt, const lanelet::ConstLanelet & to_llt)
{
  const double exit_tangent = laneletExitTangentRad(from_llt);
  const double entry_tangent = laneletEntryTangentRad(to_llt);
  const double diff = std::abs(autoware_utils::normalize_radian(entry_tangent - exit_tangent));
  return diff < kMaxContinuousHeadingChangeRad;
}

// [SEAM-FOLD 2026-09-17] Drop points that do not advance along the path.
//
// getCenterLinePath() concatenates the centreline of each lanelet in the sequence end to end, and
// it assumes those centrelines meet at the seam.  On this map they do not: live-captured on route
// lanelets 193(inv) 206(inv) 21(inv), lanelet 206's centreline ends at (-25.083, 26.612) while
// lanelet 21's begins at (-24.392, 27.100) -- 0.85 m *behind* it along the direction of travel.
// Concatenated as-is the path advances, jumps 0.85 m backwards at the seam, then advances again
// on the same heading, which reads downstream as a 175.8 degree direction reversal.
//
// This is invisible to every check further on: resamplePath() accumulates arc length from
// UNSIGNED distances, so a fold still looks like a monotonically increasing s, and
// insertOrientation() faithfully reports the reversal as a ~180 degree yaw step between
// neighbouring points.  Live-confirmed consequence: the controller's nearest-point search locks
// onto the wrong side of the fold and the reference jumps (1.12 m of reference movement against
// 0.04 m of robot movement), which is the sharp steering swing seen on the vehicle.
//
// Note the heading check upstream (isHeadingContinuousAcross) is NOT at fault and does not catch
// this: the headings either side of the seam are the same, it is only the positions that overlap.
//
// A point is dropped when the step to it turns more than max_reversal_rad away from the direction
// already being travelled.  At the ~1 m point spacing these centrelines use, no legitimate curve
// on a vehicle that cannot rotate in place comes close to that, so only true backtracking is
// removed.
void dropNonAdvancingPoints(PathWithLaneId & path, const double max_reversal_rad)
{
  constexpr double min_step_length = 1.0e-3;
  if (path.points.size() < 3) {
    return;
  }

  std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> kept;
  kept.reserve(path.points.size());
  kept.push_back(path.points.front());

  std::optional<double> travel_dir;
  for (size_t i = 1; i < path.points.size(); ++i) {
    const auto & from = kept.back().point.pose.position;
    const auto & to = path.points[i].point.pose.position;
    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    const double ds = std::hypot(dx, dy);
    if (ds < min_step_length) {
      continue;  // duplicate of the point already kept
    }
    const double dir = std::atan2(dy, dx);
    if (travel_dir) {
      const double raw = dir - *travel_dir;
      if (std::abs(std::atan2(std::sin(raw), std::cos(raw))) > max_reversal_rad) {
        continue;  // this point lies behind the one already kept -- a seam overlap
      }
    }
    travel_dir = dir;
    kept.push_back(path.points[i]);
  }

  if (kept.size() >= 2) {
    path.points = std::move(kept);
  }
}

}  // namespace

std::optional<TraveledTail> extractTraveledTailPath(
  const std::shared_ptr<RouteHandler> & route_handler, const Pose & ego_pose,
  const double retrace_distance_m)
{
  if (!route_handler) {
    return std::nullopt;
  }

  lanelet::ConstLanelet current_lanelet;
  if (!route_handler->getClosestLaneletWithinRoute(ego_pose, &current_lanelet)) {
    return std::nullopt;
  }

  // Lanelet sequence covering (at least) retrace_distance_m behind ego and nothing ahead of ego.
  // This is purely a lookup into the already-computed, already-legal route -- no routing-graph
  // invert / new route message involved.
  const auto lanelet_sequence =
    route_handler->getLaneletSequence(current_lanelet, ego_pose, retrace_distance_m, 0.0);
  if (lanelet_sequence.empty()) {
    return std::nullopt;
  }

  auto tail_path =
    route_handler->getCenterLinePath(lanelet_sequence, 0.0, std::numeric_limits<double>::max());
  if (tail_path.points.empty()) {
    return std::nullopt;
  }

  // getLaneletSequence(..., forward_distance = 0.0) still returns whole lanelets, so the
  // resulting centerline can extend slightly past ego. Trim so the tail truly ends at ego's
  // current pose -- that is the point the retrace path starts from once reversed.
  const size_t ego_idx =
    autoware::motion_utils::findNearestIndex(tail_path.points, ego_pose.position);
  tail_path.points.resize(ego_idx + 1);

  return TraveledTail{tail_path, lanelet_sequence};
}

PathWithLaneId reversePathForRetrace(
  const PathWithLaneId & forward_tail_path, const double retrace_velocity_mps)
{
  auto reversed = forward_tail_path;
  std::reverse(reversed.points.begin(), reversed.points.end());

  const auto speed_magnitude_mps =
    static_cast<float>(std::abs(retrace_velocity_mps));

  for (auto & path_point : reversed.points) {
    constexpr bool is_back = true;  // this module only ever emits retrace (reverse) motion
    // Exact sign convention proven in autoware_freespace_planner's utils.cpp (~line 135):
    // `(is_back ? -1 : 1) * point.longitudinal_velocity_mps`.
    path_point.point.longitudinal_velocity_mps = (is_back ? -1.0F : 1.0F) * speed_magnitude_mps;
  }

  // Bring the retrace path to a stop at its end (the point ego started retracing from), same
  // idiom autoware_freespace_planner uses for its trajectory tail.
  if (!reversed.points.empty()) {
    reversed.points.back().point.longitudinal_velocity_mps = 0.0F;
  }

  return reversed;
}

std::optional<RouteReversedFollow> buildRouteReversedFollowPath(
  const std::shared_ptr<RouteHandler> & route_handler, const Pose & ego_pose,
  const double backward_distance_m, const double forward_distance_m,
  const double cruise_velocity_mps, const double goal_decel_mps2,
  const double goal_decel_ramp_resample_interval_m)
{
  // [BIDIR-DEBUG] temporary diagnostic logging for the "trajectory doesn't follow reversed
  // lanelet / control goes the wrong way" investigation. Remove once root cause is confirmed
  // fixed via live test.
  static auto logger = rclcpp::get_logger("reverse_lane_follow_utils");
  static rclcpp::Clock steady_clock{RCL_ROS_TIME};

  if (!route_handler) {
    RCLCPP_WARN_THROTTLE(
      logger, steady_clock, 2000,
      "[BIDIR-DEBUG] buildRouteReversedFollowPath: null route_handler");
    return std::nullopt;
  }

  lanelet::ConstLanelet current_lanelet;
  if (!route_handler->getClosestLaneletWithinRoute(ego_pose, &current_lanelet)) {
    RCLCPP_WARN_THROTTLE(
      logger, steady_clock, 2000,
      "[BIDIR-DEBUG] buildRouteReversedFollowPath: getClosestLaneletWithinRoute failed for ego "
      "pose (%.2f, %.2f)",
      ego_pose.position.x, ego_pose.position.y);
    return std::nullopt;
  }

  // [BIDIR-BUG-FIX #2] No longer gate on current_lanelet's own isLaneletInvertedInRoute() here --
  // see buildRouteReversedFollowPath()'s doc comment in utils.hpp. Logged for visibility only;
  // the real activation decision now happens further down, after the heading-continuity walk.
  RCLCPP_WARN_THROTTLE(
    logger, steady_clock, 2000,
    "[BIDIR-DEBUG] buildRouteReversedFollowPath: ego=(%.2f, %.2f) closest_lanelet_id=%ld "
    "closest_lanelet.inverted()=%d isLaneletInvertedInRoute=%d",
    ego_pose.position.x, ego_pose.position.y, current_lanelet.id(), current_lanelet.inverted(),
    route_handler->isLaneletInvertedInRoute(current_lanelet));

  // following()/previous() on routing_graph_ptr_ already resolve correctly for an inverted
  // ConstLanelet (verified by the Phase-0 spike), so "forward"/"backward" here already mean
  // ahead-of/behind-ego in the actual direction of travel, not map/centerline-index order.
  const auto lanelet_sequence_raw = route_handler->getLaneletSequence(
    current_lanelet, ego_pose, backward_distance_m, forward_distance_m);
  if (lanelet_sequence_raw.empty()) {
    RCLCPP_WARN_THROTTLE(
      logger, steady_clock, 2000,
      "[BIDIR-DEBUG] buildRouteReversedFollowPath: getLaneletSequence returned EMPTY for "
      "current_lanelet_id=%ld (inverted route segment detected but no sequence built!)",
      current_lanelet.id());
    return std::nullopt;
  }

  // [BIDIR-BUG-FIX #2] getLaneletSequence()'s backward/forward window is built purely from
  // requested distances and has no notion of the route's own inverted/forward direction flags --
  // it will happily walk straight across a mid-route direction-change boundary (e.g. taman map's
  // 18(inv) -> 291(fwd) -> 75(fwd), unlocked by RouteHandler::buildExtraBoundaryLinks()).
  //
  // An earlier version of this fix trimmed the window down to the maximal contiguous run of
  // *actually inverted* lanelets containing current_lanelet -- i.e. it treated every fwd/inv
  // route-label change as a mandatory gear-change boundary. That is wrong in general: a route can
  // cross from an inverted lanelet into a forward-labeled one that continues the exact same
  // physical direction of travel (18(inv) -> 291(fwd) is a ~40 degree curve, not a reversal --
  // confirmed via lanelet geometry and a live repro). Forcing a stop-and-handoff there demanded a
  // reference heading 180 degrees away from ego's actual (correct, continuous) heading -- this
  // vehicle never rotates in place, so that handoff deadlocked permanently
  // (yaw_err=pi, ego frozen, planning_validator rejecting every trajectory as a result).
  //
  // Fix: trim the window using *geometric heading continuity* instead of the route label --
  // extend through any number of lanelets, inverted or not, as long as each join is an ordinary
  // curve (see isHeadingContinuousAcross()), stopping only at a genuine near-180-degree flip
  // (the one case this vehicle truly cannot drive through without stopping and changing gear).
  const auto current_it = std::find_if(
    lanelet_sequence_raw.begin(), lanelet_sequence_raw.end(), [&](const auto & llt) {
      return llt.id() == current_lanelet.id() && llt.inverted() == current_lanelet.inverted();
    });
  const size_t current_idx = (current_it != lanelet_sequence_raw.end())
                                ? static_cast<size_t>(std::distance(lanelet_sequence_raw.begin(), current_it))
                                : lanelet_sequence_raw.size();

  lanelet::ConstLanelets lanelet_sequence;
  bool forward_end_is_genuine_discontinuity = false;
  if (current_idx >= lanelet_sequence_raw.size()) {
    // Defensive fallback: current_lanelet should always be present (getLaneletSequence's own
    // contract always inserts it), but if it is somehow missing, do not risk emitting a path
    // built from an unrelated/bled window -- fall back to just current_lanelet itself.
    lanelet_sequence = {current_lanelet};
  } else {
    // [PREFERRED-ONLY 2026-09-17] Stop the walk at any lanelet the mission planner did not
    // actually choose.
    //
    // getLaneletSequence() is already route-filtered, but it filters against
    // RouteHandler::route_lanelets_, which setLaneletsFromRouteMsg() fills with EVERY primitive of
    // every route segment -- the alternatives included. Only preferred_lanelets_ holds the ones
    // the mission planner picked, and nothing on the path-building side consults it. So on a map
    // with deliberately overlapping lanelets the routing graph can chain a chosen lanelet straight
    // into an unchosen one that covers the same ground, and the "is it in the route" test waves it
    // through. Live-captured here: 193(inv) 206(inv) 21(inv), where lanelet 21's centreline begins
    // 0.85 m BEHIND where 206's ends, which is the seam that folded the path.
    //
    // This module follows one reference path in reverse; it never changes lane, so it has no use
    // for the alternative primitives that the wider filter deliberately keeps. Narrowing here (and
    // only here) leaves lane change and avoidance -- which genuinely need those neighbours -- alone.
    const auto preferred_lanelets = route_handler->getPreferredLanelets();
    const auto is_preferred = [&preferred_lanelets](const lanelet::ConstLanelet & llt) {
      return std::any_of(
        preferred_lanelets.begin(), preferred_lanelets.end(), [&llt](const auto & pref) {
          return pref.id() == llt.id() && pref.inverted() == llt.inverted();
        });
    };
    // If ego itself is sitting on an unchosen primitive, narrowing would strand this module on a
    // single lanelet. Fall back to the old, unnarrowed behaviour in that case -- a longer window
    // built from route lanelets is still far better than no window at all.
    const bool apply_preferred_filter = is_preferred(current_lanelet);

    size_t begin_idx = current_idx;
    while (begin_idx > 0 &&
           isHeadingContinuousAcross(
             lanelet_sequence_raw[begin_idx - 1], lanelet_sequence_raw[begin_idx]) &&
           (!apply_preferred_filter || is_preferred(lanelet_sequence_raw[begin_idx - 1]))) {
      --begin_idx;
    }
    size_t end_idx = current_idx;  // inclusive
    bool stopped_on_heading = false;
    while (end_idx + 1 < lanelet_sequence_raw.size()) {
      if (!isHeadingContinuousAcross(
            lanelet_sequence_raw[end_idx], lanelet_sequence_raw[end_idx + 1])) {
        stopped_on_heading = true;
        break;
      }
      if (apply_preferred_filter && !is_preferred(lanelet_sequence_raw[end_idx + 1])) {
        break;  // an unchosen primitive -- stop, but this is NOT a gear-change boundary
      }
      ++end_idx;
    }
    // Only a heading break is a genuine discontinuity the vehicle must stop and change gear for.
    // Running into an unchosen primitive is just the edge of the window this module should use, so
    // it must NOT force the zero-velocity handoff below -- doing so would brake for nothing.
    forward_end_is_genuine_discontinuity = stopped_on_heading;
    if (apply_preferred_filter && end_idx + 1 < lanelet_sequence_raw.size() && !stopped_on_heading) {
      RCLCPP_WARN_THROTTLE(
        logger, steady_clock, 3000,
        "[PREFERRED-ONLY] window stopped at lanelet %ld: the next one (%ld) is in the route but is "
        "not the segment's preferred primitive. Building the reverse path from chosen lanelets only.",
        lanelet_sequence_raw[end_idx].id(), lanelet_sequence_raw[end_idx + 1].id());
    }
    lanelet_sequence = lanelet::ConstLanelets(
      lanelet_sequence_raw.begin() + static_cast<std::ptrdiff_t>(begin_idx),
      lanelet_sequence_raw.begin() + static_cast<std::ptrdiff_t>(end_idx) + 1);
  }

  // This trigger only applies if the geometrically-continuous run just built actually contains a
  // genuinely route-inverted lanelet somewhere -- otherwise ego is simply on an ordinary forward
  // stretch with no reverse maneuver nearby, and this trigger should not fire.
  const bool touches_inverted_lanelet = std::any_of(
    lanelet_sequence.begin(), lanelet_sequence.end(),
    [&route_handler](const auto & llt) { return route_handler->isLaneletInvertedInRoute(llt); });

  {
    std::string seq_str;
    for (const auto & llt : lanelet_sequence_raw) {
      seq_str += std::to_string(llt.id()) + (llt.inverted() ? "(inv) " : "(fwd) ");
    }
    std::string trimmed_str;
    for (const auto & llt : lanelet_sequence) {
      trimmed_str += std::to_string(llt.id()) + (llt.inverted() ? "(inv) " : "(fwd) ");
    }
    RCLCPP_WARN_THROTTLE(
      logger, steady_clock, 2000,
      "[BIDIR-DEBUG] buildRouteReversedFollowPath: raw_lanelet_sequence = [ %s] "
      "heading-continuous run = [ %s] touches_inverted_lanelet=%d",
      seq_str.c_str(), trimmed_str.c_str(), touches_inverted_lanelet);
  }

  if (!touches_inverted_lanelet) {
    // No actual reverse-in-route segment anywhere in the continuous run touching ego -- this
    // activation trigger does not apply; the caller should fall back to the (still-supported
    // standalone) retrace-request trigger.
    return std::nullopt;
  }

  auto path =
    route_handler->getCenterLinePath(lanelet_sequence, 0.0, std::numeric_limits<double>::max());
  if (path.points.empty()) {
    RCLCPP_WARN_THROTTLE(
      logger, steady_clock, 2000,
      "[BIDIR-DEBUG] buildRouteReversedFollowPath: getCenterLinePath returned EMPTY path");
    return std::nullopt;
  }

  // [SEAM-FOLD 2026-09-17] Remove the backward step the lanelet seams leave behind -- see
  // dropNonAdvancingPoints() for the measured geometry and why nothing downstream would catch it.
  {
    const size_t before = path.points.size();
    dropNonAdvancingPoints(path, M_PI / 2.0);
    if (path.points.size() != before) {
      RCLCPP_WARN_THROTTLE(
        logger, steady_clock, 2000,
        "[SEAM-FOLD] dropped %zu point(s) that stepped backwards where the lanelet centrelines "
        "overlap; path %zu -> %zu points",
        before - path.points.size(), before, path.points.size());
    }
  }

  for (auto & path_point : path.points) {
    path_point.point.longitudinal_velocity_mps =
      -std::abs(path_point.point.longitudinal_velocity_mps);
  }

  // [BIDIR-BUG-FIX #5] (2026-09-06 goal-overshoot investigation) Cap the raw centerline velocity
  // (just negated above) at cruise_velocity_mps (caller passes parameters_->retrace_velocity_mps)
  // instead of letting it ride at whatever the underlying lanelet's speed_limit happens to be --
  // see capReverseFollowCruiseVelocity()'s doc comment in utils.hpp for the live-confirmed
  // evidence (a 1.389 m/s lanelet speed_limit vs. this module's own intended 1.0 m/s reverse
  // speed).
  const float pre_cap_max_speed_mps = path.points.empty() ? 0.0F : std::max_element(
    path.points.begin(), path.points.end(),
    [](const auto & a, const auto & b) {
      return std::abs(a.point.longitudinal_velocity_mps) <
             std::abs(b.point.longitudinal_velocity_mps);
    })->point.longitudinal_velocity_mps;
  capReverseFollowCruiseVelocity(path, cruise_velocity_mps);
  RCLCPP_WARN_THROTTLE(
    logger, steady_clock, 2000,
    "[BIDIR-DEBUG] buildRouteReversedFollowPath: cruise-speed cap applied -- raw max speed on "
    "this window was %.3f m/s, capped to cruise_velocity_mps=%.3f m/s",
    std::abs(pre_cap_max_speed_mps), std::abs(cruise_velocity_mps));

  // [BIDIR-BUG-FIX] Unlike reversePathForRetrace() (which always stops at the end of its tail
  // path) or the normal forward-driving path (which goes through
  // DefaultFixedGoalPlanner::modifyPathForSmoothGoalConnection() and always gets a hard
  // zero-velocity point at the goal), this route-reversed-follow path is built purely from a
  // sliding [ego - backward_distance_m, ego + forward_distance_m] window with no notion of the
  // goal at all -- every point, including the last one, keeps riding at -abs(v). While this
  // module is active (i.e. for the entire final approach whenever the last route leg into the
  // goal is a reversed lanelet), its output *replaces* getPreviousModuleOutput() wholesale (see
  // plan()), so the goal-connection zero-velocity point normally injected upstream never makes
  // it into the trajectory -- root cause of "robot doesn't stop even after passing the goal" on
  // reversed routes. Fix: if the goal lies within this window's lanelet sequence, truncate the
  // path at the goal and force zero velocity there, mirroring reversePathForRetrace()'s idiom.
  const bool sequence_contains_goal = std::any_of(
    lanelet_sequence.begin(), lanelet_sequence.end(),
    [&route_handler](const auto & llt) { return route_handler->isInGoalRouteSection(llt); });
  if (sequence_contains_goal) {
    const auto goal_pose = route_handler->getGoalPose();
    // [BIDIR-BUG-FIX #4] Inserts an exact point at goal_pose (interpolated via
    // insertTargetPoint()) instead of snapping to the nearest existing resampled point -- see
    // truncatePathAtGoal()'s doc comment in utils.hpp for why the old findNearestIndex()-only
    // truncation could overshoot the true goal by up to one waypoint-spacing.
    truncatePathAtGoal(path, goal_pose);
    RCLCPP_WARN_THROTTLE(
      logger, steady_clock, 2000,
      "[BIDIR-DEBUG] buildRouteReversedFollowPath: goal is within this window's lanelet "
      "sequence -- truncated path to exact goal pose (%.2f, %.2f), %zu points remain, forced "
      "zero velocity at the new end point",
      goal_pose.position.x, goal_pose.position.y, path.points.size());

    // [BIDIR-BUG-FIX #7] (2026-09-07 continued-overshoot investigation) Densify the path near the
    // goal BEFORE ramping -- see densifyPathNearGoal()'s doc comment in utils.hpp. Without this,
    // applyGoalDecelerationRamp() only has whatever native centerline waypoints happen to exist
    // near the goal to act on, which can be too sparse (live-confirmed: ~2m gaps) to produce a
    // real gradual slowdown regardless of the ramp formula being applied correctly. Span sized
    // exactly to the ramp's own full braking distance (v^2 / (2*|decel|)) -- nothing farther back
    // than the ramp could ever reach needs densifying.
    const double decel_abs = std::abs(goal_decel_mps2);
    const double ramp_span_m = (decel_abs < 1e-6)
                                  ? 0.0
                                  : (cruise_velocity_mps * cruise_velocity_mps) / (2.0 * decel_abs);
    const size_t pre_densify_point_count = path.points.size();
    densifyPathNearGoal(path, ramp_span_m, goal_decel_ramp_resample_interval_m);
    RCLCPP_WARN_THROTTLE(
      logger, steady_clock, 2000,
      "[BIDIR-DEBUG] buildRouteReversedFollowPath: goal-approach densify applied -- ramp_span_m="
      "%.3f, max_spacing_m=%.3f, point count %zu -> %zu",
      ramp_span_m, goal_decel_ramp_resample_interval_m, pre_densify_point_count,
      path.points.size());

    // [BIDIR-BUG-FIX #6] (2026-09-06 goal-overshoot investigation) The truncation above places an
    // exact, geometrically-correct zero-velocity point at the goal -- but until now every
    // preceding point still rode at the (now cruise-capped, see BIDIR-BUG-FIX #5 above) cruise
    // speed right up to that last point, i.e. an abrupt single-cycle drop to zero with no
    // deceleration shaping at all. Apply a conservative backward-pass decel ramp so the commanded
    // trajectory itself gradually slows down approaching the goal -- see
    // applyGoalDecelerationRamp()'s doc comment in utils.hpp for the physics and reasoning.
    applyGoalDecelerationRamp(path, goal_decel_mps2);
    {
      // [BIDIR-DEBUG] Sample a few points near the goal so a future investigation can see the
      // ramp working (or not) immediately from logs, per this session's logging convention.
      std::string sample_str;
      const size_t n = path.points.size();
      const size_t sample_count = std::min<size_t>(5, n);
      for (size_t k = 0; k < sample_count; ++k) {
        const size_t idx = n - sample_count + k;
        sample_str += "[" + std::to_string(idx) + "]=" +
                      std::to_string(path.points[idx].point.longitudinal_velocity_mps) + " ";
      }
      RCLCPP_WARN_THROTTLE(
        logger, steady_clock, 2000,
        "[BIDIR-DEBUG] buildRouteReversedFollowPath: goal-decel ramp applied -- "
        "goal_decel_mps2=%.3f, last %zu point velocities (m/s) approaching the goal: %s",
        std::abs(goal_decel_mps2), sample_count, sample_str.c_str());
    }
  } else if (forward_end_is_genuine_discontinuity) {
    // [BIDIR-BUG-FIX #2] The forward walk stopped short of the raw window's own end because
    // isHeadingContinuousAcross() found a genuine ~180 degree flip right after
    // lanelet_sequence.back() -- a real gear-change point this vehicle cannot drive through
    // without stopping (unlike the old label-based trim, this is NOT triggered merely by the
    // route's fwd/inv label changing). Force a stop there, same idiom as the goal-truncation
    // branch above, so this module hands off to forward driving with a clean stop instead of
    // still reversing across an actual reversal.
    if (!path.points.empty()) {
      path.points.back().point.longitudinal_velocity_mps = 0.0F;
    }
    RCLCPP_WARN_THROTTLE(
      logger, steady_clock, 2000,
      "[BIDIR-DEBUG] buildRouteReversedFollowPath: window trimmed at a genuine heading "
      "discontinuity (lanelet %ld -> next raw lanelet requires an ~180 degree flip) -- forced "
      "zero velocity at trimmed path end so this module hands off to forward driving with a "
      "clean stop instead of attempting an impossible in-place reorientation",
      lanelet_sequence.back().id());
  }
  // else: the window simply ran out at backward_distance_m/forward_distance_m (or the raw
  // sequence's own end near the route's start/end) with no genuine discontinuity -- leave the
  // path riding at -abs(v) all the way to that edge, same as any other sliding-window path; the
  // next cycle's recentered window naturally continues it.

  RCLCPP_WARN_THROTTLE(
    logger, steady_clock, 2000,
    "[BIDIR-DEBUG] buildRouteReversedFollowPath: path built, %zu points, "
    "first=(%.2f,%.2f,v=%.2f) last=(%.2f,%.2f,v=%.2f)",
    path.points.size(), path.points.front().point.pose.position.x,
    path.points.front().point.pose.position.y,
    path.points.front().point.longitudinal_velocity_mps,
    path.points.back().point.pose.position.x, path.points.back().point.pose.position.y,
    path.points.back().point.longitudinal_velocity_mps);

  return RouteReversedFollow{path, lanelet_sequence};
}

bool shouldSuppressReversedFollowReactivation(
  const std::optional<lanelet::Id> & last_exited_inverted_lanelet_id,
  const lanelet::ConstLanelets & follow_lanelets, const double distance_to_goal_m,
  const double goal_reach_tolerance_m)
{
  if (!last_exited_inverted_lanelet_id.has_value()) {
    return false;
  }

  const bool follow_contains_latched_lanelet = std::any_of(
    follow_lanelets.begin(), follow_lanelets.end(), [&](const auto & llt) {
      return llt.id() == last_exited_inverted_lanelet_id.value();
    });
  if (!follow_contains_latched_lanelet) {
    return false;
  }

  // [BIDIR-BUG-FIX #3] Goal-proximity escape hatch -- see this function's doc comment in
  // utils.hpp. Never let the latch withhold the goal-stop itself.
  const bool ego_near_goal = distance_to_goal_m < goal_reach_tolerance_m;
  return !ego_near_goal;
}

void truncatePathAtGoal(PathWithLaneId & path, const Pose & goal_pose)
{
  if (path.points.empty()) {
    return;
  }

  const auto goal_seg_idx =
    autoware::motion_utils::findNearestSegmentIndex(path.points, goal_pose.position);
  const auto insert_idx =
    autoware::motion_utils::insertTargetPoint(goal_seg_idx, goal_pose.position, path.points, 1e-3);

  size_t goal_idx = 0;
  if (insert_idx) {
    goal_idx = insert_idx.value();
  } else {
    // insertTargetPoint() declined to insert (e.g. goal_pose is ~coincident with an existing
    // point, or its sharp-angle/overlap guard tripped) -- fall back to the previous (less
    // precise) nearest-existing-sample behavior rather than leaving the path untruncated.
    goal_idx = autoware::motion_utils::findNearestIndex(path.points, goal_pose.position);
  }

  if (goal_idx + 1 < path.points.size()) {
    path.points.resize(goal_idx + 1);
  }
  path.points.back().point.longitudinal_velocity_mps = 0.0F;
}

void capReverseFollowCruiseVelocity(PathWithLaneId & path, const double cruise_velocity_mps)
{
  const float cap_mps = static_cast<float>(std::abs(cruise_velocity_mps));
  for (auto & path_point : path.points) {
    const float raw_speed_mps = std::abs(path_point.point.longitudinal_velocity_mps);
    const float capped_speed_mps = std::min(raw_speed_mps, cap_mps);
    // Preserve the existing sign convention (this module always emits reverse/negative velocity
    // for this path -- see buildRouteReversedFollowPath()'s -std::abs() assignment right before
    // this is called) rather than assuming a sign here.
    const float sign = (path_point.point.longitudinal_velocity_mps < 0.0F) ? -1.0F : 1.0F;
    path_point.point.longitudinal_velocity_mps = sign * capped_speed_mps;
  }
}

void applyGoalDecelerationRamp(PathWithLaneId & path, const double decel_mps2)
{
  const double decel = std::abs(decel_mps2);
  if (path.points.size() < 2 || decel < 1e-6) {
    // Fewer than 2 points: nothing to ramp over. ~Zero decel: sqrt(2*0*d) == 0 for every point,
    // which would zero out the entire path rather than leaving it as a no-op -- guard against a
    // misconfigured param doing that.
    return;
  }

  double distance_to_goal_m = 0.0;
  // Tracks the smallest index the loop below actually modifies -- i.e. the farthest-from-goal
  // point the backward physics pass reaches into. Starts at points.size()-1 (the goal point
  // itself) so the forward clean-up pass below is a no-op if the backward pass never modifies
  // anything (physics limit already looser than cruise speed everywhere).
  size_t ramp_region_start_idx = path.points.size() - 1;
  // path.points.back() is expected to already be the exact, zero-velocity goal point (see
  // truncatePathAtGoal()) -- start the backward walk from the point just before it.
  for (size_t i = path.points.size() - 1; i-- > 0;) {
    distance_to_goal_m +=
      autoware_utils::calc_distance2d(
        path.points[i].point.pose.position, path.points[i + 1].point.pose.position);

    const double v_limit_mps = std::sqrt(2.0 * decel * distance_to_goal_m);
    const float current_speed_mps = std::abs(path.points[i].point.longitudinal_velocity_mps);
    if (v_limit_mps >= current_speed_mps) {
      // This point is already far enough from the goal that the physics limit exceeds its
      // (already cruise-capped) speed -- no clamp needed here. Every earlier point (walking
      // further backward) is at least as far from the goal, so the limit there is at least as
      // loose too -- safe to stop the backward walk here rather than scanning the whole path.
      // (The forward clean-up pass below is what guarantees full monotonicity regardless of this
      // early exit -- see its own comment.)
      break;
    }

    const float sign = (path.points[i].point.longitudinal_velocity_mps < 0.0F) ? -1.0F : 1.0F;
    path.points[i].point.longitudinal_velocity_mps = sign * static_cast<float>(v_limit_mps);
    ramp_region_start_idx = i;
  }

  // [BIDIR-BUG-FIX #7] (2026-09-07 continued-overshoot investigation) Forward monotonicity
  // clean-up pass, added on top of the backward physics pass above. The backward pass alone
  // clamps each point to sqrt(2*decel*distance_to_goal_m) -- a valid "can still physically brake
  // to 0 in the remaining distance" bound in isolation -- but that bound is computed from
  // ABSOLUTE distance-to-goal, not from the (possibly already-slower) point immediately behind
  // it. When the raw/cruise-capped input speed is itself non-monotonic along the path (live-
  // confirmed: a point ~2m from the goal sitting on a naturally-faster stretch, ~1.1455 m/s,
  // while points several meters farther back sat on a naturally-slower upstream curve, ~0.833
  // m/s), the backward pass alone can leave a point CLOSER to the goal faster than one farther
  // away -- a real, published, non-monotonic "slows down, speeds back up, then instant stop"
  // velocity bump, exactly the symptom this investigation's live evidence showed.
  //
  // Deliberately scoped to [ramp_region_start_idx, end] only -- NOT the whole path -- by
  // comparing each point from ramp_region_start_idx onward to its immediate predecessor (which
  // may itself be just outside the ramp-affected region, i.e. the untouched cruise/raw-speed
  // point the ramp region borders). Points before ramp_region_start_idx (never touched by the
  // backward pass, because the physics limit there was already looser than their raw/cruise
  // speed) are left completely alone: an unrelated slow curve far earlier in the path (nothing to
  // do with this goal) must not permanently cap every faster point after it for the rest of the
  // path -- only the ramp's own region, and its one shared boundary with whatever precedes it, is
  // ever clamped here. Only ever reduces a velocity further (never increases it, preserving every
  // other function's "only clamps down" principle), and is a no-op whenever the backward pass's
  // output was already monotonic (the common case of uniform/monotonic raw speed).
  for (size_t i = ramp_region_start_idx; i < path.points.size(); ++i) {
    if (i == 0) {
      continue;
    }
    const float prev_mag = std::abs(path.points[i - 1].point.longitudinal_velocity_mps);
    const float this_mag = std::abs(path.points[i].point.longitudinal_velocity_mps);
    if (this_mag > prev_mag) {
      const float sign = (path.points[i].point.longitudinal_velocity_mps < 0.0F) ? -1.0F : 1.0F;
      path.points[i].point.longitudinal_velocity_mps = sign * prev_mag;
    }
  }
}

void densifyPathNearGoal(PathWithLaneId & path, const double max_span_m, const double max_spacing_m)
{
  if (path.points.size() < 2 || max_spacing_m < 1e-3 || max_span_m < 1e-3) {
    return;
  }

  // Walk backward from the terminal (goal) point, same traversal direction/idiom as
  // applyGoalDecelerationRamp(), to find how far back (index-wise) densification needs to reach.
  // Always includes at least the final segment (guaranteed by decrementing at least once before
  // checking the span), even if that single segment alone already exceeds max_span_m -- that is
  // exactly the pathological "one huge terminal segment" case that must still be densified.
  double cumulative_m = 0.0;
  size_t first_idx = path.points.size() - 1;
  while (first_idx > 0) {
    const double seg_len = autoware_utils::calc_distance2d(
      path.points[first_idx - 1].point.pose.position, path.points[first_idx].point.pose.position);
    --first_idx;
    cumulative_m += seg_len;
    if (cumulative_m >= max_span_m) {
      break;
    }
  }

  // Rebuild points[first_idx .. end] with every original segment longer than max_spacing_m
  // subdivided into evenly spaced interpolated points. Original points are preserved exactly
  // (same pose, velocity, lane ids); only new interpolated points are added in between.
  std::vector<autoware_internal_planning_msgs::msg::PathPointWithLaneId> rebuilt;
  rebuilt.push_back(path.points[first_idx]);
  for (size_t i = first_idx + 1; i < path.points.size(); ++i) {
    const auto & from_point = path.points[i - 1];
    const auto & to_point = path.points[i];
    const double seg_len = autoware_utils::calc_distance2d(
      from_point.point.pose.position, to_point.point.pose.position);
    const int n_extra = static_cast<int>(std::floor(seg_len / max_spacing_m));
    for (int k = 1; k <= n_extra; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(n_extra + 1);
      auto pt = from_point;
      pt.point.pose.position.x =
        from_point.point.pose.position.x +
        t * (to_point.point.pose.position.x - from_point.point.pose.position.x);
      pt.point.pose.position.y =
        from_point.point.pose.position.y +
        t * (to_point.point.pose.position.y - from_point.point.pose.position.y);
      pt.point.pose.position.z =
        from_point.point.pose.position.z +
        t * (to_point.point.pose.position.z - from_point.point.pose.position.z);
      // Orientation/lane-id metadata: reuse the point closer to the goal (to_point) -- an
      // adequate approximation over a span this short; velocity: copy from_point's (the *faster*
      // of the two endpoints, since this module's paths always slow down toward the goal) so this
      // placeholder never reads as "already ramped" before applyGoalDecelerationRamp() runs --
      // that function re-shapes every one of these new points' velocity immediately after this
      // call regardless.
      pt.point.pose.orientation = to_point.point.pose.orientation;
      pt.point.longitudinal_velocity_mps = from_point.point.longitudinal_velocity_mps;
      rebuilt.push_back(pt);
    }
    rebuilt.push_back(to_point);
  }

  path.points.erase(path.points.begin() + static_cast<std::ptrdiff_t>(first_idx), path.points.end());
  path.points.insert(path.points.end(), rebuilt.begin(), rebuilt.end());
}

void clampVelocityNearEgoWhenCloseToGoal(
  PathWithLaneId & path, const Pose & ego_pose, const bool goal_within_follow_window,
  const double distance_to_goal_m, const double trigger_distance_m,
  const double clamp_velocity_mps)
{
  if (!goal_within_follow_window || distance_to_goal_m >= trigger_distance_m) {
    // [BIDIR-BUG-FIX #8] Narrow, goal-scoped gating -- mirrors
    // shouldSuppressReversedFollowReactivation()'s own discipline so this reactive layer can never
    // fire far from any actual goal (e.g. mid-route on an ordinary reversed stretch with no goal
    // anywhere nearby).
    return;
  }

  const float clamp_mps = static_cast<float>(std::abs(clamp_velocity_mps));
  for (auto & path_point : path.points) {
    const double dist_to_ego =
      autoware_utils::calc_distance2d(ego_pose.position, path_point.point.pose.position);
    if (dist_to_ego > trigger_distance_m) {
      // Only clamp points near ego's current position -- a (possibly much longer) follow window
      // farther away is left to whatever truncatePathAtGoal()/densifyPathNearGoal()/
      // applyGoalDecelerationRamp() already computed for it.
      continue;
    }
    const float current_speed_mps = std::abs(path_point.point.longitudinal_velocity_mps);
    if (current_speed_mps <= clamp_mps) {
      // Never increase a point's velocity -- same principle as every other velocity-shaping
      // helper in this file.
      continue;
    }
    const float sign = (path_point.point.longitudinal_velocity_mps < 0.0F) ? -1.0F : 1.0F;
    path_point.point.longitudinal_velocity_mps = sign * clamp_mps;
  }
}

bool isEgoArrivedAndStoppedAtGoal(
  const double distance_to_goal_m, const double ego_speed_mps,
  const double goal_reach_tolerance_m, const double arrived_stop_velocity_mps)
{
  const bool ego_within_goal_tolerance = distance_to_goal_m < goal_reach_tolerance_m;
  const bool ego_genuinely_stopped = std::abs(ego_speed_mps) < arrived_stop_velocity_mps;
  return ego_within_goal_tolerance && ego_genuinely_stopped;
}

}  // namespace autoware::behavior_path_planner::reverse_lane_follow_utils
