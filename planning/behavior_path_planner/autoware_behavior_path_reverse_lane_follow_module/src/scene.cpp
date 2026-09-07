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
#include <memory>
#include <string>
#include <unordered_map>

namespace autoware::behavior_path_planner
{
using reverse_lane_follow_utils::buildRouteReversedFollowPath;
using reverse_lane_follow_utils::extractTraveledTailPath;
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

  const auto follow = buildRouteReversedFollowPath(
    planner_data_->route_handler, getEgoPose(), parameters_->route_reversed_backward_distance_m,
    parameters_->route_reversed_forward_distance_m);

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

  if (!was_active) {
    RCLCPP_WARN(
      getLogger(),
      "[BIDIR-DEBUG] is_route_reversed_active FLIP false->true: ego=(%.2f, %.2f) %zu path points",
      getEgoPose().position.x, getEgoPose().position.y, follow->path.points.size());
  }
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
    return false;
  }

  return true;
}

}  // namespace autoware::behavior_path_planner
