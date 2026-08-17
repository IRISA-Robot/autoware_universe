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

ReverseLaneFollowModule::ReverseLaneFollowModule(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<ReverseLaneFollowParameters> & parameters,
  const std::shared_ptr<autoware_utils::InterProcessPollingSubscriber<std_msgs::msg::Float64>> &
    retrace_request_subscriber,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
    objects_of_interest_marker_interface_ptr_map,
  const std::shared_ptr<PlanningFactorInterface> planning_factor_interface)
: SceneModuleInterface{
    name, node, rtc_interface_ptr_map, objects_of_interest_marker_interface_ptr_map,
    planning_factor_interface},
  parameters_{parameters},
  retrace_request_subscriber_{retrace_request_subscriber}
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
  status_.is_route_reversed_active = false;
  status_.route_reversed_path = PathWithLaneId{};
  status_.route_reversed_lanelets.clear();

  if (!planner_data_ || !planner_data_->route_handler) {
    return;
  }

  const auto follow = buildRouteReversedFollowPath(
    planner_data_->route_handler, getEgoPose(), parameters_->route_reversed_backward_distance_m,
    parameters_->route_reversed_forward_distance_m);
  if (!follow) {
    return;
  }

  status_.is_route_reversed_active = true;
  status_.route_reversed_path = follow->path;
  status_.route_reversed_lanelets = follow->lanelets;
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
