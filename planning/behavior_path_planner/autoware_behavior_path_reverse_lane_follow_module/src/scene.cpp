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

#include "autoware/behavior_path_reverse_lane_follow_module/scene.hpp"

#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"
#include "autoware/behavior_path_reverse_lane_follow_module/utils.hpp"

#include <autoware_utils/geometry/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>

namespace autoware::behavior_path_planner
{
using reverse_lane_follow_utils::buildRouteReversedFollowPath;
using reverse_lane_follow_utils::capReverseFollowCruiseVelocity;
using reverse_lane_follow_utils::clampVelocityNearEgoWhenCloseToGoal;
using reverse_lane_follow_utils::extractTraveledTailPath;
using reverse_lane_follow_utils::isEgoArrivedAndStoppedAtGoal;
using reverse_lane_follow_utils::reversePathForRetrace;
using reverse_lane_follow_utils::shouldSuppressReversedFollowReactivation;

ReverseLaneFollowModule::ReverseLaneFollowModule(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<ReverseLaneFollowParameters> & parameters,
  const std::shared_ptr<autoware_utils::InterProcessPollingSubscriber<std_msgs::msg::Float64>> &
    retrace_request_subscriber,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
    objects_of_interest_marker_interface_ptr_map,
  const std::shared_ptr<PlanningFactorInterface> planning_factor_interface,
  const std::shared_ptr<std::optional<lanelet::Id>> & last_exited_inverted_lanelet_id)
: SceneModuleInterface{
    name, node, rtc_interface_ptr_map, objects_of_interest_marker_interface_ptr_map,
    planning_factor_interface},
  parameters_{parameters},
  retrace_request_subscriber_{retrace_request_subscriber},
  last_exited_inverted_lanelet_id_{last_exited_inverted_lanelet_id}
{
}

void ReverseLaneFollowModule::updateRetraceRequest()
{
  if (!retrace_request_subscriber_) {
    return;
  }

  const auto msg = retrace_request_subscriber_->take_data();
  if (!msg) {
    return;
  }

  // The polling subscriber keeps handing back the same cached message until a new one is
  // published, so only act when the message identity actually changed.
  const bool is_new_message = (msg.get() != last_seen_request_msg_.get());
  last_seen_request_msg_ = msg;

  if (!is_new_message) {
    return;
  }

  if (msg->data <= 0.0) {
    // explicit cancel: value <= 0 clears any pending/active request
    requested_distance_m_.reset();
    return;
  }

  requested_distance_m_ = std::max(msg->data, parameters_->min_retrace_distance_m);
}

void ReverseLaneFollowModule::updateData()
{
  updateRetraceRequest();

  if (!requested_distance_m_) {
    status_.retrace_path = PathWithLaneId{};
    status_.retrace_lanelets.clear();
  } else if (!planner_data_ || !planner_data_->route_handler) {
    // fall through to updateRouteReversedFollow() below regardless
  } else {
    const auto tail =
      extractTraveledTailPath(planner_data_->route_handler, getEgoPose(), *requested_distance_m_);
    if (!tail) {
      // Cannot build a valid retrace path (e.g. not enough traveled history yet) -- treat as no
      // request rather than emitting a bogus path.
      requested_distance_m_.reset();
      status_.retrace_path = PathWithLaneId{};
      status_.retrace_lanelets.clear();
    } else {
      status_.retrace_path = reversePathForRetrace(tail->path, parameters_->retrace_velocity_mps);
      status_.retrace_lanelets = tail->lanelets;
    }
  }

  // Alternative/additional activation trigger (Task E / §4d): active whenever ego's current
  // route segment is itself flagged reversed, not just during an explicit retrace request. Kept
  // independent of the retrace-request branch above so Phase 1's standalone retrace use case
  // keeps working unchanged.
  updateRouteReversedFollow();
}

void ReverseLaneFollowModule::updateRouteReversedFollow()
{
  const bool was_active = status_.is_route_reversed_active;
  // Snapshot before clearing below -- needed to identify *which* lanelet we are exiting if this
  // cycle turns out to be the true->false transition (see the one-way latch note in scene.hpp).
  const auto previously_active_lanelets = status_.route_reversed_lanelets;

  status_.is_route_reversed_active = false;
  status_.route_reversed_path = PathWithLaneId{};
  status_.route_reversed_lanelets.clear();

  if (!planner_data_ || !planner_data_->route_handler) {
    // [BIDIR-DEBUG] edge-triggered (unthrottled) flip log -- see note below. A flip caused by a
    // missing route_handler is distinguished here from one caused by buildRouteReversedFollowPath
    // itself returning nullopt.
    if (was_active) {
      RCLCPP_WARN(
        getLogger(),
        "[BIDIR-DEBUG] is_route_reversed_active FLIP true->false: no planner_data_/route_handler");
    }
    return;
  }

  // [BIDIR-BUG-FIX #5/#6] Pass retrace_velocity_mps as the cruise-speed cap (consistent with that
  // param's own name/intent -- see capReverseFollowCruiseVelocity()'s doc comment in utils.hpp)
  // and goal_stop_decel_mps2 as the conservative goal-approach ramp magnitude (see
  // applyGoalDecelerationRamp()'s doc comment in utils.hpp).
  const auto follow = buildRouteReversedFollowPath(
    planner_data_->route_handler, getEgoPose(), parameters_->route_reversed_backward_distance_m,
    parameters_->route_reversed_forward_distance_m, parameters_->retrace_velocity_mps,
    parameters_->goal_stop_decel_mps2, parameters_->goal_decel_ramp_resample_interval_m);

  // [BIDIR-BUG-FIX] One-way latch: if `follow` succeeded for the same lanelet we most recently
  // recorded a true->false exit from, this is boundary-line noise re-arming the module, not a
  // genuine new reversed segment -- suppress it (treat exactly like the nullopt case below). See
  // last_exited_inverted_lanelet_id_'s doc comment in scene.hpp for why this is safe/expected.
  //
  // [BIDIR-BUG-FIX #3] ...UNLESS ego is close enough to the actual goal that suppressing here
  // would drop the goal-stop entirely -- see shouldSuppressReversedFollowReactivation()'s doc
  // comment in utils.hpp for the full rationale (goals commonly sit right at a lanelet boundary,
  // exactly where this latch is most likely to have already fired spuriously).
  const double distance_to_goal_m =
    autoware_utils::calc_distance2d(getEgoPose(), planner_data_->route_handler->getGoalPose());
  const bool follow_contains_latched_lanelet =
    follow && last_exited_inverted_lanelet_id_->has_value() &&
    std::any_of(
      follow->lanelets.begin(), follow->lanelets.end(), [&](const auto & llt) {
        return llt.id() == last_exited_inverted_lanelet_id_->value();
      });
  const bool reactivation_suppressed =
    follow && shouldSuppressReversedFollowReactivation(
                *last_exited_inverted_lanelet_id_, follow->lanelets, distance_to_goal_m,
                parameters_->goal_reach_tolerance_m);

  if (follow_contains_latched_lanelet && !reactivation_suppressed) {
    RCLCPP_WARN(
      getLogger(),
      "[BIDIR-DEBUG] one-way latch BYPASSED (goal proximity): ego=(%.2f, %.2f) is %.2fm from "
      "goal (< goal_reach_tolerance_m=%.2f) -- allowing reversed-follow reactivation despite "
      "prior exit latch on lanelet_id=%ld, so the goal-truncation stop point is not lost",
      getEgoPose().position.x, getEgoPose().position.y, distance_to_goal_m,
      parameters_->goal_reach_tolerance_m, last_exited_inverted_lanelet_id_->value());
  }

  if (!follow || reactivation_suppressed) {
    // [BIDIR-DEBUG] edge-triggered, UNTHROTTLED (unlike buildRouteReversedFollowPath's own
    // 2000ms-throttled logging) so a rapid on/off flicker -- e.g. ego pose oscillating right at
    // the boundary between a forward and a reversed route segment, or between two lanelets where
    // getClosestLaneletWithinRoute's nearest-polygon query has no hysteresis -- is not masked by
    // the throttle window. If this fires repeatedly within a second or two of "true->false" then
    // "false->true" while ego is obviously still mid-maneuver, that confirms activation flicker
    // (not a one-shot end-of-segment transition) as the cause of the trajectory intermittently
    // disappearing: plan() falls back to getPreviousModuleOutput() every time this is inactive.
    if (was_active) {
      if (!previously_active_lanelets.empty()) {
        const auto & exited_lanelet = previously_active_lanelets.back();
        // [BIDIR-BUG-FIX #3] Never arm the latch for the lanelet that actually contains the
        // route's goal. This is the proactive counterpart to the goal-proximity escape hatch
        // above: if the exit we are about to latch IS the goal's own lanelet, a spurious
        // true->false transition here is exactly the boundary-noise case the latch cannot be
        // allowed to punish, since re-entering this exact lanelet is also how the final approach
        // reaches (and stops at) the goal itself.
        if (planner_data_->route_handler->isInGoalRouteSection(exited_lanelet)) {
          RCLCPP_WARN(
            getLogger(),
            "[BIDIR-DEBUG] one-way latch NOT armed for lanelet_id=%ld: isInGoalRouteSection() is "
            "true -- exempting the goal's own lanelet from the latch so a boundary-noise exit "
            "there can never permanently suppress the final approach's goal-stop",
            exited_lanelet.id());
        } else {
          *last_exited_inverted_lanelet_id_ = exited_lanelet.id();
        }
      }
      RCLCPP_WARN(
        getLogger(),
        "[BIDIR-DEBUG] is_route_reversed_active FLIP true->false: ego=(%.2f, %.2f) %s",
        getEgoPose().position.x, getEgoPose().position.y,
        reactivation_suppressed
          ? "buildRouteReversedFollowPath succeeded but reactivation suppressed (one-way latch)"
          : "buildRouteReversedFollowPath returned nullopt this cycle");
    }
    return;
  }

  status_.is_route_reversed_active = true;
  status_.route_reversed_path = follow->path;
  status_.route_reversed_lanelets = follow->lanelets;

  // [BIDIR-BUG-FIX #8] (2026-09-07 continued-overshoot investigation) Reactive, closed-loop safety
  // layer -- independent of, and applied AFTER, whatever truncatePathAtGoal()/
  // densifyPathNearGoal()/applyGoalDecelerationRamp() already computed inside
  // buildRouteReversedFollowPath() this cycle. Those are all open-loop (shaped once per cycle from
  // this cycle's path geometry); this instead reacts directly to the LIVE distance_to_goal_m
  // already computed above (reused, not recomputed) so it cannot be defeated by any blind spot
  // specific to that open-loop chain. Requested by the user as explicit defense-in-depth given
  // enable_overshoot_emergency (control-side emergency-brake-on-overshoot) is intentionally left
  // false -- with that backstop disabled, the path-shaping chain above is otherwise the ONLY
  // defense against a reverse-goal overshoot.
  //
  // goal_within_follow_window mirrors buildRouteReversedFollowPath()'s own sequence_contains_goal
  // check (any lanelet in this cycle's follow window satisfies isInGoalRouteSection()) so this
  // layer only ever engages in the same narrow, goal-scoped circumstances the rest of this
  // goal-approach logic already does -- it must never fire on an ordinary reversed stretch with no
  // goal anywhere nearby.
  const bool goal_within_follow_window = std::any_of(
    follow->lanelets.begin(), follow->lanelets.end(), [&](const auto & llt) {
      return planner_data_->route_handler->isInGoalRouteSection(llt);
    });

  // [BIDIR-BUG-FIX #9] (2026-09-07 live-verification pass) reactive_goal_clamp_trigger_distance_m
  // (2.0m, static config) is SMALLER than the open-loop ramp's own physics-required braking span
  // from cruise speed (buildRouteReversedFollowPath's ramp_span_m = cruise_velocity_mps^2 /
  // (2*|goal_decel_mps2|), currently 1.389^2/(2*0.3) = 3.216m -- see utils.cpp). Live-confirmed
  // (docs/research/reverse-lane-follow-goal-overshoot-fix.md, 2026-09-07 live-verification
  // section): this reactive layer engaged every single cycle across 5 separate goal approaches,
  // yet the vehicle still traveled 0.3-1.1m past its closest approach to the goal before actually
  // stopping in 4/5 of them -- because a defense-in-depth backstop that triggers LATER (2.0m) than
  // the primary ramp's own required stopping distance from cruise speed (3.216m) can never help:
  // by the time it engages, real ego velocity may still be well above
  // reactive_goal_clamp_velocity_mps if tracking has lagged the open-loop ramp at all, and there is
  // no longer enough remaining distance to actually arrest that speed. Fix: floor the effective
  // trigger distance at this same physics-derived span (recomputed here, not reused across the
  // buildRouteReversedFollowPath() call boundary, to keep this a pure function of already-threaded
  // params) so the reactive layer's own radius can never be narrower than the ramp it backstops --
  // reactive_goal_clamp_trigger_distance_m remains a floor for slower cruise speeds where the
  // physics span would be smaller than the configured minimum.
  const double reactive_clamp_min_span_m =
    (std::abs(parameters_->goal_stop_decel_mps2) < 1e-6)
      ? 0.0
      : (parameters_->retrace_velocity_mps * parameters_->retrace_velocity_mps) /
          (2.0 * std::abs(parameters_->goal_stop_decel_mps2));
  const double effective_reactive_clamp_trigger_distance_m =
    std::max(parameters_->reactive_goal_clamp_trigger_distance_m, reactive_clamp_min_span_m);
  if (effective_reactive_clamp_trigger_distance_m > parameters_->reactive_goal_clamp_trigger_distance_m) {
    // Separate static local clock from the "ENGAGED" log below -- distinct throttle windows, same
    // convention as the rest of this function.
    static rclcpp::Clock widened_clock{RCL_ROS_TIME};
    RCLCPP_WARN_THROTTLE(
      getLogger(), widened_clock, 2000,
      "[BIDIR-DEBUG] reactive goal-proximity clamp trigger distance WIDENED: configured "
      "reactive_goal_clamp_trigger_distance_m=%.3f < physics-required braking span from cruise "
      "(retrace_velocity_mps=%.3f, goal_stop_decel_mps2=%.3f) = %.3f -- using the wider value so "
      "this backstop's radius is never narrower than the primary ramp it is meant to guard",
      parameters_->reactive_goal_clamp_trigger_distance_m, parameters_->retrace_velocity_mps,
      parameters_->goal_stop_decel_mps2, reactive_clamp_min_span_m);
  }

  const size_t path_points_before_reactive_clamp = status_.route_reversed_path.points.size();
  clampVelocityNearEgoWhenCloseToGoal(
    status_.route_reversed_path, getEgoPose(), goal_within_follow_window, distance_to_goal_m,
    effective_reactive_clamp_trigger_distance_m, parameters_->reactive_goal_clamp_velocity_mps);
  if (goal_within_follow_window &&
      distance_to_goal_m < effective_reactive_clamp_trigger_distance_m) {
    // Static local clock + WARN_THROTTLE, matching this module's existing convention for logging
    // that fires every cycle while a condition holds (see utils.cpp's steady_clock) -- this one
    // engages, at closest approach, for as many cycles as ego stays within
    // effective_reactive_clamp_trigger_distance_m of the goal, so it needs the same throttle.
    static rclcpp::Clock steady_clock{RCL_ROS_TIME};
    RCLCPP_WARN_THROTTLE(
      getLogger(), steady_clock, 1000,
      "[BIDIR-DEBUG] reactive goal-proximity clamp ENGAGED (distinct from the path-generation-time "
      "goal-decel ramp above): ego=(%.2f, %.2f) is %.2fm from goal (< "
      "effective_reactive_clamp_trigger_distance_m=%.2f, configured "
      "reactive_goal_clamp_trigger_distance_m=%.2f) -- forcing any path point within that same "
      "radius of ego down to at most reactive_goal_clamp_velocity_mps=%.2f m/s, %zu points in "
      "window",
      getEgoPose().position.x, getEgoPose().position.y, distance_to_goal_m,
      effective_reactive_clamp_trigger_distance_m,
      parameters_->reactive_goal_clamp_trigger_distance_m,
      parameters_->reactive_goal_clamp_velocity_mps, path_points_before_reactive_clamp);
  }

  // [BIDIR-BUG-FIX #10] (2026-09-07 ARRIVED-state investigation) Once ego is genuinely at rest at
  // the goal, stop re-deriving a fresh goal-approach velocity profile from ego's live (possibly
  // mm-noisy) pose every single cycle -- that re-derivation is exactly what was injecting a fresh,
  // tiny nonzero velocity command every cycle (via the goal-decel ramp / reactive clamp above,
  // both of which are correctly-working but necessarily react to *this cycle's* ego pose), which
  // in turn kept autoware_motion_utils::VehicleStopCheckerBase::isVehicleStopped()'s hardcoded
  // 1mm/s-threshold, 1.0s-window check from ever seeing a clean stopped window -- blocking
  // mission_planner's ArrivalChecker::is_arrived() (and therefore /planning/mission_planning/state,
  // /api/routing/state, /autoware/state) from ever reporting ARRIVED. See
  // isEgoArrivedAndStoppedAtGoal()'s doc comment in utils.hpp for why this uses a dedicated,
  // deliberately-looser arrived_stop_velocity_mps rather than that same hardcoded 1mm/s bound.
  //
  // This does NOT deactivate the module (is_route_reversed_active stays true, status_
  // .route_reversed_path stays non-empty) -- only deactivating here would fall back to
  // getPreviousModuleOutput() for the rest of the approach, which carries none of
  // buildRouteReversedFollowPath()'s goal-truncation/zero-velocity injection (the exact bug #1
  // class this module's very first fix closed). Instead, force every point in the
  // already-computed path to exactly zero velocity -- reusing capReverseFollowCruiseVelocity()
  // with a 0.0 m/s cap (a cap of 0 clamps every point's magnitude down to 0 regardless of sign,
  // the same "only ever reduce, never increase" helper already used earlier in this same
  // function/buildRouteReversedFollowPath(), just called with the tightest possible cap) instead of
  // adding a new function. The result is a stable, byte-for-byte-zero-velocity path every
  // subsequent cycle, independent of ego's exact (possibly still mm-shifting) live pose, which
  // canTransitSuccessState() below (same condition, reused) then reports as success.
  const double ego_speed_mps = getEgoSpeedMps();
  const bool ego_arrived_and_stopped =
    goal_within_follow_window && isEgoArrivedAndStoppedAtGoal(
                                    distance_to_goal_m, ego_speed_mps,
                                    parameters_->goal_reach_tolerance_m,
                                    parameters_->arrived_stop_velocity_mps);
  if (ego_arrived_and_stopped) {
    capReverseFollowCruiseVelocity(status_.route_reversed_path, 0.0);
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 1000,
      "[BIDIR-DEBUG] ego ARRIVED and stopped at goal: dist_to_goal=%.4fm (< "
      "goal_reach_tolerance_m=%.2f), ego_speed=%.4fm/s (< arrived_stop_velocity_mps=%.2f) -- "
      "holding a fully zero-velocity goal-approach path instead of re-deriving fresh "
      "ramp/reactive-clamp velocities from ego's live pose every cycle; "
      "canTransitSuccessState() will now report success for this route_reversed_path",
      distance_to_goal_m, parameters_->goal_reach_tolerance_m, ego_speed_mps,
      parameters_->arrived_stop_velocity_mps);
  }

  if (!was_active) {
    RCLCPP_WARN(
      getLogger(),
      "[BIDIR-DEBUG] is_route_reversed_active FLIP false->true: ego=(%.2f, %.2f) %zu path points",
      getEgoPose().position.x, getEgoPose().position.y, follow->path.points.size());
  }
}

double ReverseLaneFollowModule::getEgoSpeedMps() const
{
  if (!planner_data_ || !planner_data_->self_odometry) {
    return 0.0;
  }
  return std::abs(planner_data_->self_odometry->twist.twist.linear.x);
}

bool ReverseLaneFollowModule::isExecutionRequested() const
{
  const bool retrace_active =
    requested_distance_m_.has_value() && !status_.retrace_path.points.empty();
  const bool route_reversed_active =
    status_.is_route_reversed_active && !status_.route_reversed_path.points.empty();
  return retrace_active || route_reversed_active;
}

bool ReverseLaneFollowModule::isExecutionReady() const
{
  return !status_.retrace_path.points.empty() || !status_.route_reversed_path.points.empty();
}

BehaviorModuleOutput ReverseLaneFollowModule::plan()
{
  BehaviorModuleOutput output{};

  // Explicit retrace request takes priority if both happen to be active simultaneously (should
  // not normally occur -- retrace is an operator-triggered scaffold use case, route-reversed
  // following is route-driven -- but retrace being an explicit ask wins ties).
  const PathWithLaneId * active_path = nullptr;
  const lanelet::ConstLanelets * active_lanelets = nullptr;
  if (!status_.retrace_path.points.empty()) {
    active_path = &status_.retrace_path;
    active_lanelets = &status_.retrace_lanelets;
  } else if (!status_.route_reversed_path.points.empty()) {
    active_path = &status_.route_reversed_path;
    active_lanelets = &status_.route_reversed_lanelets;
  }

  if (!active_path) {
    // [BIDIR-DEBUG] Nothing to follow (yet/anymore) -- do not clobber the previous module's
    // output. If this fires while ego is actually on a reversed route segment, the previous
    // module's (positive-velocity, slot0-seed) output passes straight through to
    // static_obstacle_avoidance and downstream control unmodified -- a likely root cause of
    // "control moves the wrong way" if buildRouteReversedFollowPath() failed to activate.
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 2000,
      "[BIDIR-DEBUG] ReverseLaneFollowModule::plan(): no active_path -- passing through previous "
      "module output unmodified (prev output has %zu points)",
      getPreviousModuleOutput().path.points.size());
    return getPreviousModuleOutput();
  }

  RCLCPP_WARN_THROTTLE(
    getLogger(), *clock_, 2000,
    "[BIDIR-DEBUG] ReverseLaneFollowModule::plan(): using %s path, %zu points, first_v=%.2f "
    "last_v=%.2f",
    (!status_.retrace_path.points.empty() ? "RETRACE" : "ROUTE_REVERSED"), active_path->points.size(),
    active_path->points.front().point.longitudinal_velocity_mps,
    active_path->points.back().point.longitudinal_velocity_mps);

  output.path = *active_path;
  output.reference_path = *active_path;

  if (!active_lanelets->empty()) {
    output.drivable_area_info.drivable_lanes = utils::generateDrivableLanes(*active_lanelets);
  }

  return output;
}

CandidateOutput ReverseLaneFollowModule::planCandidate() const
{
  if (!status_.retrace_path.points.empty()) {
    return CandidateOutput(status_.retrace_path);
  }
  return CandidateOutput(status_.route_reversed_path);
}

void ReverseLaneFollowModule::processOnEntry()
{
  status_.is_retracing = requested_distance_m_.has_value();
  status_.requested_distance_m =
    requested_distance_m_.value_or(parameters_->default_retrace_distance_m);

  // [BIDIR-BUG-FIX] Deliberately NOT reset here (nor in processOnExit()): initially this was
  // cleared in processOnEntry(), reasoning that it only fires once per genuinely new episode --
  // wrong. SceneModuleManagerInterface::updateIdleModuleInstance() calls onEntry() (-> this) on
  // the *same* still-idle module instance every single cycle it remains idle, not just once on a
  // fresh instance/episode (confirmed live: clearing here made the latch a no-op, ~zero
  // suppressions logged, identical flip-flop to having no latch at all). There is no reliable
  // "genuinely new episode" signal available at this call site to gate a reset on, so this is
  // intentionally a permanent, node-lifetime latch per lanelet id instead: once exited, a given
  // lanelet id is never treated as a fresh reversed-follow activation again for the rest of this
  // run. The only cost is the (rare, and not applicable to any route this fix was written for) case
  // of a single route needing to reverse over the exact same lanelet id twice.
}

void ReverseLaneFollowModule::processOnExit()
{
  status_.is_retracing = false;
  status_.retrace_path = PathWithLaneId{};
  status_.retrace_lanelets.clear();
  status_.is_route_reversed_active = false;
  status_.route_reversed_path = PathWithLaneId{};
  status_.route_reversed_lanelets.clear();
  // Edge-triggered: completing (or aborting) a retrace always clears the request. A new
  // std_msgs::msg::Float64 publish is required to trigger the next one.
  requested_distance_m_.reset();
}

bool ReverseLaneFollowModule::canTransitSuccessState()
{
  if (!status_.retrace_path.points.empty()) {
    const auto & end_pose = status_.retrace_path.points.back().point.pose;
    const auto remaining_distance = autoware_utils::calc_distance2d(getEgoPose(), end_pose);
    return remaining_distance < parameters_->goal_reach_tolerance_m;
  }

  if (!status_.route_reversed_path.points.empty()) {
    // Route-reversed following stays active as long as ego remains on a reversed route segment;
    // updateRouteReversedFollow() clears route_reversed_path once that stops being true, which
    // this then reports as "success" (nothing left to do) rather than "failure".
    //
    // [BIDIR-BUG-FIX #10] (2026-09-07 ARRIVED-state investigation) ...UNLESS the goal itself lies
    // inside the current follow window. The unconditional `return false` above is still correct
    // (and load-bearing -- verified, not assumed) for a mid-route reversed stretch that is NOT the
    // goal's own segment: there is genuinely nothing to "succeed" at yet, and the module must stay
    // active until updateRouteReversedFollow() clears route_reversed_path on its own (ego exits the
    // reversed segment). But when the goal lies inside this window, ego reaches the goal and then
    // has nothing left to "exit" to -- the old unconditional `false` here kept this module RUNNING
    // forever once parked at the goal, so updateRouteReversedFollow() kept regenerating a fresh
    // goal-approach path (and therefore a fresh, live-pose-derived nonzero velocity command) every
    // single cycle indefinitely, which is what was starving mission_planner's arrival check of a
    // clean stopped window (see isEgoArrivedAndStoppedAtGoal()'s doc comment in utils.hpp for the
    // full mechanism). Narrowed via the same isInGoalRouteSection() helper already used elsewhere
    // in this file (see updateRouteReversedFollow() above) -- a goal-scoped exemption on top of the
    // existing check, mirroring this session's earlier one-way-latch narrowing fix, not a blanket
    // replacement of the "still active while genuinely mid-maneuver" semantics.
    if (!planner_data_ || !planner_data_->route_handler) {
      // Defensive: should not happen (status_.route_reversed_path being non-empty this cycle
      // implies updateRouteReversedFollow() already used a valid route_handler this same cycle),
      // but do not risk a null-deref -- fall back to the original "still running" behavior.
      return false;
    }

    const bool goal_within_route_reversed_window = std::any_of(
      status_.route_reversed_lanelets.begin(), status_.route_reversed_lanelets.end(),
      [&](const auto & llt) { return planner_data_->route_handler->isInGoalRouteSection(llt); });
    if (!goal_within_route_reversed_window) {
      return false;
    }

    const double distance_to_goal_m = autoware_utils::calc_distance2d(
      getEgoPose(), planner_data_->route_handler->getGoalPose());
    const double ego_speed_mps = getEgoSpeedMps();
    const bool arrived = isEgoArrivedAndStoppedAtGoal(
      distance_to_goal_m, ego_speed_mps, parameters_->goal_reach_tolerance_m,
      parameters_->arrived_stop_velocity_mps);

    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 1000,
      "[BIDIR-DEBUG] canTransitSuccessState(): route_reversed_path branch, goal is within this "
      "window (isInGoalRouteSection) -- dist_to_goal=%.4fm (goal_reach_tolerance_m=%.2f), "
      "ego_speed=%.4fm/s (arrived_stop_velocity_mps=%.2f) -> %s",
      distance_to_goal_m, parameters_->goal_reach_tolerance_m, ego_speed_mps,
      parameters_->arrived_stop_velocity_mps, arrived ? "SUCCESS" : "still RUNNING");

    return arrived;
  }

  return true;
}

}  // namespace autoware::behavior_path_planner
