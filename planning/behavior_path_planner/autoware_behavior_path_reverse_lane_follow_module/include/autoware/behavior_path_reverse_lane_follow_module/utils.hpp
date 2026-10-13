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
 * [BIDIR-BUG-FIX #5/#6] (2026-09-06 goal-overshoot investigation) Two velocity-profile fixes on
 * top of the geometric goal-truncation from BIDIR-BUG-FIX #4: the final published trajectory was
 * confirmed geometrically correct (zero-velocity point within 1cm of the true goal), yet the
 * vehicle still coasted ~0.7-1.2m past it -- because every point up to (but not including) the
 * truncated goal point was riding at whatever the raw centerline's velocity happened to be (often
 * a lanelet speed_limit well above the intended low-speed maneuver speed), with no deceleration
 * shaping at all before the hard stop. Fixed here via two new pure, unit-testable helpers applied
 * in sequence:
 *   1. capReverseFollowCruiseVelocity() -- caps every point's velocity magnitude at
 *      `cruise_velocity_mps` (the caller passes parameters_->retrace_velocity_mps, consistent with
 *      that param's own name/intent) instead of blindly negating the raw centerline velocity.
 *   2. applyGoalDecelerationRamp() -- once truncatePathAtGoal() has placed the exact
 *      zero-velocity point at the goal, walks backward from it applying a conservative physics-
 *      based decel ramp (v(s) = sqrt(2 * decel_mps2 * distance_to_goal_m)) so the *commanded*
 *      trajectory itself slows down gradually approaching the goal, rather than leaving 100% of
 *      the deceleration shaping to the downstream general-purpose velocity_smoother.
 *
 * [BIDIR-BUG-FIX #7] (2026-09-07 continued-overshoot investigation) applyGoalDecelerationRamp()'s
 * backward walk implicitly assumes the pre-ramp (cruise-capped) velocity is non-decreasing as
 * distance-from-goal grows, so it can safely stop ("break") the instant the physics limit exceeds
 * the current point's speed. Live-confirmed false: the raw lanelet centerline's own speed varies
 * along the path (e.g. a slower curve a few meters back, a faster straight stretch right next to
 * the goal), so a point very close to the goal can start out *faster* than points farther back --
 * live-captured case: two points ~2m from the goal still carrying their raw ~1.1455 m/s
 * cruise-capped speed (the physics limit at that distance, ~1.11 m/s, was just barely under it, so
 * the ramp only nudged it slightly), while points several meters farther back sat at a
 * naturally-slower 0.833 m/s from an upstream curve -- i.e. speed went cruise -> higher -> instant
 * zero right at the goal, not a smooth monotonic decrease. Root cause: with the native centerline
 * point spacing near the goal being coarse (~2m in the live case, versus the ~3.2m total distance
 * the ramp needs to fully bind from a 1.389 m/s cruise at 0.3 m/s^2), the ramp has only one or two
 * existing waypoints to act on before the hard zero -- nowhere near enough resolution for a real
 * gradual slowdown, regardless of the formula being applied correctly at each of those few points.
 * Fixed via densifyPathNearGoal(), called right after truncatePathAtGoal() and before
 * applyGoalDecelerationRamp() so the ramp always has fine-grained waypoints to work with,
 * independent of how sparse the underlying lanelet centerline happens to be near this particular
 * goal.
 *
 * @return std::nullopt if ego is not within the route, or no geometrically continuous run
 *         touching ego's current lanelet contains an actually-inverted lanelet (i.e. this trigger
 *         does not apply -- the caller should fall back to / keep using the retrace-request
 *         trigger).
 */
std::optional<RouteReversedFollow> buildRouteReversedFollowPath(
  const std::shared_ptr<RouteHandler> & route_handler, const Pose & ego_pose,
  const double backward_distance_m, const double forward_distance_m,
  const double cruise_velocity_mps, const double goal_decel_mps2,
  const double goal_decel_ramp_resample_interval_m);

/**
 * @brief Pure decision function for the one-way reverse-follow reactivation latch (see
 * ReverseLaneFollowModule::last_exited_inverted_lanelet_id_'s doc comment in scene.hpp for the
 * latch's full rationale: it exists to stop getClosestLaneletWithinRoute()'s lack of hysteresis
 * from flip-flopping activation at a mid-route direction-change boundary, not to ever withhold
 * the goal-stop itself).
 *
 * [BIDIR-BUG-FIX #3] Goal-proximity escape hatch: goals are commonly placed right at/near a
 * lanelet boundary -- exactly where the latch above is most likely to have already fired once,
 * spuriously, before ego has actually reached the goal. If the latch is allowed to suppress
 * reactivation there, plan() falls back to getPreviousModuleOutput() for the rest of the
 * approach, which carries none of buildRouteReversedFollowPath()'s goal-truncation/zero-velocity
 * injection -- the vehicle drives straight through the goal with no stop at all. So: once ego is
 * within `goal_reach_tolerance_m` of the actual goal, the latch must never suppress reactivation,
 * regardless of which lanelet it was armed for. Exposed as a standalone pure function (no
 * RouteHandler/ROS-node dependency) so it is unit-testable, matching this module's existing
 * pure-logic test convention (see test/test_reverse_lane_follow_utils.cpp).
 *
 * @param last_exited_inverted_lanelet_id current latch state (nullopt = latch not armed at all)
 * @param follow_lanelets lanelet sequence buildRouteReversedFollowPath() just returned for this
 *        cycle
 * @param distance_to_goal_m ego's current straight-line distance to route_handler->getGoalPose()
 * @param goal_reach_tolerance_m same tolerance canTransitSuccessState() already uses to judge
 *        "close enough to the goal" for retrace paths -- reused here rather than inventing a new
 *        threshold, per this module's existing goal-proximity convention.
 * @return true if reactivation should be suppressed (latch fires this cycle), false otherwise
 *         (either the latch isn't armed for any lanelet in follow_lanelets, or ego is close
 *         enough to the goal that the latch is bypassed).
 */
bool shouldSuppressReversedFollowReactivation(
  const std::optional<lanelet::Id> & last_exited_inverted_lanelet_id,
  const lanelet::ConstLanelets & follow_lanelets, const double distance_to_goal_m,
  const double goal_reach_tolerance_m);

/**
 * @brief Truncate `path` at the exact geometric location of `goal_pose`: inserts an interpolated
 * point exactly at goal_pose (via autoware::motion_utils::insertTargetPoint()) rather than
 * snapping to the nearest existing resampled point, then forces zero velocity at that new point
 * and drops everything after it.
 *
 * [BIDIR-BUG-FIX #4] buildRouteReversedFollowPath()'s goal-truncation previously used
 * findNearestIndex() alone, which can only land on an *existing* path sample -- for a sliding
 * window resampled at, say, 1m spacing, that can overshoot the true goal by up to ~1 waypoint
 * spacing before the zero-velocity point takes effect. Using insertTargetPoint() to add an exact
 * point at goal_pose closes that gap. Falls back to the old nearest-existing-sample behavior if
 * insertTargetPoint() declines to insert (e.g. goal_pose is ~coincident with an existing point, or
 * its sharp-angle/overlap guard trips) rather than leaving the path untruncated.
 *
 * No-op if `path` is empty.
 */
void truncatePathAtGoal(PathWithLaneId & path, const Pose & goal_pose);

/**
 * @brief [BIDIR-BUG-FIX #5] Cap the magnitude of every point's longitudinal velocity in `path` at
 * `cruise_velocity_mps`, preserving sign and never *increasing* any point's speed.
 *
 * Root cause this fixes: buildRouteReversedFollowPath() previously set every point's velocity to
 * `-abs(raw_centerline_velocity)` -- i.e. whatever speed_limit the underlying lanelet happened to
 * carry, negated. For this module's low-speed reverse maneuvers that raw value can be
 * substantially higher than intended (confirmed live: a lanelet speed_limit of 5 km/h = 1.389 m/s,
 * ~39% faster than this module's own `retrace_velocity_mps=1.0` -- despite that param's name and
 * despite downstream braking-distance tuning (pid.param.yaml's stopping_state_stop_dist) being
 * computed assuming 1.0 m/s). This mirrors reversePathForRetrace()'s existing use of
 * retrace_velocity_mps as the intended cruise speed for this module's reverse maneuvers, but as a
 * cap rather than a flat override, so a path point that is already slower than
 * `cruise_velocity_mps` (e.g. through a tight curve, or a lanelet with an even lower speed_limit)
 * is left unaffected -- only a raw velocity *above* the cap is pulled down to it.
 *
 * No-op if `path` is empty.
 *
 * @param cruise_velocity_mps magnitude (m/s) cap; sign/abs handled internally, matching this
 *        module's existing `-std::abs(...)` reverse-gear convention.
 */
void capReverseFollowCruiseVelocity(PathWithLaneId & path, const double cruise_velocity_mps);

/**
 * @brief [BIDIR-BUG-FIX #6] Apply a physically-reasoned deceleration ramp to `path`, walking
 * backward from its terminal point (expected to already be the exact, zero-velocity goal point --
 * see truncatePathAtGoal()) and limiting each preceding point's velocity magnitude to
 * `sqrt(2 * abs(decel_mps2) * distance_to_goal_m)`, where `distance_to_goal_m` is the cumulative
 * point-to-point arc length back to the terminal point. This is the standard kinematic
 * braking-distance relation (v^2 = 2*a*d, solved for v) applied as a backward pass, the same
 * general family of technique `autoware_velocity_smoother` uses for its own backward jerk-limited
 * pass (see smoother_base.cpp/trajectory_utils.cpp) and `autoware_behavior_velocity_planner`'s
 * stop-point deceleration shaping (behavior_velocity_planner_common/utilization/util.cpp) --
 * deliberately simplified here to a closed-form sqrt (no jerk term) since this module's ramp only
 * needs to guarantee real stopping margin, not reproduce a full jerk-optimal profile; the
 * downstream velocity_smoother still jerk-shapes the final trajectory on top of this.
 *
 * Root cause this fixes: previously nothing shaped the approach to the goal at all -- every point
 * up to (but excluding) the truncated goal point kept riding at the (capped) cruise velocity, so
 * 100% of deceleration was left to the downstream general-purpose velocity_smoother, which (given
 * the too-high assumed cruise speed capReverseFollowCruiseVelocity() also fixes) did not always
 * have enough runway. This ramp instead makes the *commanded* trajectory itself slow down
 * gradually well before the goal.
 *
 * Only ever reduces a point's existing velocity magnitude (never increases it above whatever
 * capReverseFollowCruiseVelocity() already set). The backward physics pass above is monotonic
 * toward the goal ONLY WHEN the input's raw/cruise-capped speed is itself uniform or monotonic
 * along the path -- farther points have larger `distance_to_goal_m` and therefore a looser (or
 * no-op) limit purely as a function of absolute distance, which says nothing about a
 * point-to-point comparison against whatever speed the *immediately preceding* point ended up at.
 *
 * [BIDIR-BUG-FIX #7] (2026-09-07 continued-overshoot investigation) Live-confirmed that
 * assumption can be false: the raw lanelet centerline speed is not always uniform along the path
 * (e.g. a slower curve a few meters back, a faster straight stretch right next to the goal), so
 * the backward pass alone can leave a point CLOSER to the goal with a HIGHER commanded speed than
 * one farther away -- a real, published, non-monotonic "slows down, speeds back up, then instant
 * stop" bump, live-captured as two points ~2m from the goal riding at ~1.1455 m/s while points
 * several meters farther back sat at a naturally-slower 0.833 m/s. A second, forward clean-up pass
 * is applied after the backward pass: walking from the start toward the goal, each point's
 * velocity magnitude is clamped to at most the immediately preceding point's (already-shaped)
 * magnitude, scoped to only the region the backward pass actually touched (plus its one shared
 * boundary with whatever precedes it) -- an unrelated slow point far earlier in the path, with
 * nothing to do with this goal, is never touched or used to needlessly cap anything after it.
 * This guarantees the function's OUTPUT is always genuinely non-increasing toward the goal,
 * regardless of whether the raw input speed was. See densifyPathNearGoal() for the complementary
 * fix that gives this forward pass finer waypoints to work with, producing a gradual multi-step
 * decrease rather than a single "clip straight down to the previous point" cliff when the raw
 * input's own non-monotonic bump is close to the goal.
 *
 * No-op if `path` has fewer than 2 points or `decel_mps2` is ~0.
 *
 * @param decel_mps2 magnitude (m/s^2) of the conservative deceleration used for the ramp; sign is
 *        handled internally (abs is taken).
 */
void applyGoalDecelerationRamp(PathWithLaneId & path, const double decel_mps2);

/**
 * @brief [BIDIR-BUG-FIX #7] Ensure `path` has waypoints spaced at most `max_spacing_m` apart
 * within `max_span_m` of arc length of its terminal point, inserting linearly-interpolated points
 * as needed so applyGoalDecelerationRamp()'s backward pass always has enough resolution to
 * enforce a real, gradual, (near-)monotonic slowdown -- see that function's and
 * buildRouteReversedFollowPath()'s doc comments for the live-confirmed failure this closes (a
 * coarse native centerline spacing near the goal left the ramp only one or two waypoints to act
 * on, producing an almost-cruise-speed command until a last-instant cliff-drop to zero).
 *
 * Must be called AFTER truncatePathAtGoal() (so `path.points.back()` is already the exact,
 * zero-velocity goal point the span is measured back from) and BEFORE applyGoalDecelerationRamp()
 * (so the ramp has the densified points to shape).
 *
 * Inserted points copy lane-id metadata and orientation from whichever original endpoint is
 * closer to the goal, and copy longitudinal_velocity_mps from the original endpoint farther from
 * the goal (i.e. the *faster* of the two, matching this module's "only ever reduce, never
 * increase" convention) -- both are deliberately-rough approximations adequate for a subdivision
 * span this short (at most one original segment's length): only position + velocity matter for
 * this function's purpose, and velocity is immediately re-shaped by applyGoalDecelerationRamp()
 * right after this runs regardless of what placeholder value is copied in here.
 *
 * Always densifies at least the final segment (points.back() and the point before it), even if
 * that single segment alone already exceeds max_span_m -- that is exactly the pathological "one
 * huge terminal segment" case this function must not silently skip.
 *
 * No-op if `path` has fewer than 2 points, or if `max_spacing_m`/`max_span_m` is ~0.
 *
 * @param max_span_m how far back (arc length from the terminal point) densification should reach
 *        -- callers should pass the ramp's own full braking distance
 *        (cruise_velocity_mps^2 / (2 * abs(goal_decel_mps2))) so nothing farther back than the
 *        ramp could ever reach is needlessly densified.
 * @param max_spacing_m maximum allowed distance between consecutive points after densification.
 */
void densifyPathNearGoal(
  PathWithLaneId & path, const double max_span_m, const double max_spacing_m);

/**
 * @brief [BIDIR-BUG-FIX #8] Reactive, closed-loop safety layer, independent of (and applied after)
 * whatever truncatePathAtGoal()/applyGoalDecelerationRamp()/densifyPathNearGoal() already computed
 * this cycle. Those are all open-loop in the sense that they shape a PATH once per cycle from path
 * geometry (arc-length distance along the path's own, possibly-imperfectly-resampled,
 * waypoints) -- this function instead reacts directly to a live, real-time straight-line
 * distance-to-goal measurement (the same `distance_to_goal_m` ReverseLaneFollowModule::
 * updateRouteReversedFollow() already computes every cycle via
 * autoware_utils::calc_distance2d(ego_pose, route_handler->getGoalPose()) for the
 * shouldSuppressReversedFollowReactivation() latch-bypass check -- reused here, not recomputed),
 * so it cannot be defeated by any blind spot specific to the path-shaping helpers above (coarse
 * resampling, a path-generation-cycle lag, or any future bug in that open-loop chain this
 * function does not need to know about).
 *
 * Only engages when BOTH:
 *   1. `goal_within_follow_window` is true -- the same `sequence_contains_goal`-style condition
 *      buildRouteReversedFollowPath() already uses (any lanelet in the current follow window
 *      satisfies route_handler->isInGoalRouteSection()) -- passed in by the caller (scene.cpp),
 *      which already has a RouteHandler to evaluate it with; this function stays RouteHandler-free
 *      and pure/unit-testable, matching this module's existing convention.
 *   2. `distance_to_goal_m < trigger_distance_m` -- ego is genuinely close to the goal, not just
 *      somewhere within a possibly-long follow window that happens to contain the goal's lanelet.
 * This mirrors shouldSuppressReversedFollowReactivation()'s own "only in this narrow, goal-scoped
 * window" gating discipline so this layer cannot fire far from any actual goal.
 *
 * When engaged, clamps the velocity magnitude of every point in `path` that is within
 * `trigger_distance_m` of `ego_pose` (i.e. "near ego's current position", not the whole
 * -- possibly much longer -- follow window) down to at most `clamp_velocity_mps`, preserving sign.
 * Only ever reduces a point's velocity, never increases it (a point already at or below
 * `clamp_velocity_mps` is left untouched) -- same "never increases, only clamps down" principle as
 * capReverseFollowCruiseVelocity()/applyGoalDecelerationRamp().
 *
 * No-op if `path` is empty, `goal_within_follow_window` is false, or
 * `distance_to_goal_m >= trigger_distance_m`.
 */
void clampVelocityNearEgoWhenCloseToGoal(
  PathWithLaneId & path, const Pose & ego_pose, const bool goal_within_follow_window,
  const double distance_to_goal_m, const double trigger_distance_m,
  const double clamp_velocity_mps);

/**
 * @brief [BIDIR-BUG-FIX #10] (2026-09-07 ARRIVED-state investigation) Pure decision function for
 * "has ego genuinely arrived AND come to rest at the route's goal, for the route-driven
 * (route_reversed_path) goal-approach case".
 *
 * Root cause this fixes: ReverseLaneFollowModule::canTransitSuccessState()'s route_reversed_path
 * branch previously returned `false` UNCONDITIONALLY while status_.route_reversed_path was
 * non-empty -- correct for a mid-route reversed stretch that is not the goal's own segment (there
 * is genuinely nothing to "succeed" at until ego exits that segment), but wrong once the goal
 * itself lies inside the final reversed lanelet: ego reaches the goal but there is nothing to
 * "exit" to, so the module never reports success, buildRouteReversedFollowPath() keeps
 * regenerating the goal-approach path every cycle indefinitely, and each regeneration re-derives
 * a fresh (tiny but nonzero) velocity command from ego's live, marginally-noisy pose --
 * live-confirmed to keep autoware_motion_utils::VehicleStopCheckerBase::isVehicleStopped()'s
 * hardcoded 1mm/s-threshold, 1.0s-window check from ever seeing a clean stopped window, which in
 * turn blocks mission_planner's ArrivalChecker::is_arrived() (and therefore
 * /planning/mission_planning/state, /api/routing/state, /autoware/state) from ever reporting
 * ARRIVED.
 *
 * This function reuses `goal_reach_tolerance_m` -- the SAME threshold
 * shouldSuppressReversedFollowReactivation() already uses for "close enough to the goal", and the
 * retrace_path branch of canTransitSuccessState() already uses for its own "close enough to the
 * end of the retrace path" check -- rather than inventing a new distance convention, per this
 * module's established goal-proximity pattern. It additionally requires ego's own live speed to be
 * below `arrived_stop_velocity_mps` -- deliberately NOT the hardcoded 1mm/s
 * VehicleStopCheckerBase itself uses (that check is this module's DOWNSTREAM consumer, gating
 * mission_planner's own arrival report over a full continuous 1-second window; this function's job
 * is to decide when THIS module should stop being the thing that keeps re-perturbing ego's velocity
 * away from that 1mm/s bar in the first place, which needs a looser, "close enough to genuinely at
 * rest for our own purposes" threshold, comfortably above the live-confirmed peak residual creep
 * this bug produces (~0.00105 m/s) -- see arrived_stop_velocity_mps's doc comment in
 * data_structs.hpp for the exact value chosen and why).
 *
 * Callers (both canTransitSuccessState() and updateRouteReversedFollow(), see scene.cpp) are
 * responsible for first confirming the goal actually lies within the relevant lanelet window (via
 * route_handler->isInGoalRouteSection(), the same helper already used elsewhere in this module) --
 * this function only ever answers the position+speed question, not the "is the goal even nearby"
 * question, so it stays RouteHandler-free and unit-testable, matching this module's existing
 * pure-logic test convention.
 *
 * @param distance_to_goal_m ego's current straight-line distance to route_handler->getGoalPose().
 * @param ego_speed_mps ego's current speed magnitude (m/s); sign/abs handled internally.
 * @param goal_reach_tolerance_m same tolerance used elsewhere in this module for "close enough to
 *        the goal".
 * @param arrived_stop_velocity_mps speed magnitude below which ego is considered "at rest" for the
 *        purpose of this decision.
 * @return true once ego is within goal_reach_tolerance_m of the goal AND at/under
 *         arrived_stop_velocity_mps; false otherwise.
 */
bool isEgoArrivedAndStoppedAtGoal(
  const double distance_to_goal_m, const double ego_speed_mps,
  const double goal_reach_tolerance_m, const double arrived_stop_velocity_mps);

}  // namespace autoware::behavior_path_planner::reverse_lane_follow_utils

#endif  // AUTOWARE__BEHAVIOR_PATH_REVERSE_LANE_FOLLOW_MODULE__UTILS_HPP_
