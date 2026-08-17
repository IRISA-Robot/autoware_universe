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
    return;
  }

  if (!planner_data_ || !planner_data_->route_handler) {
    return;
  }

  const auto tail =
    extractTraveledTailPath(planner_data_->route_handler, getEgoPose(), *requested_distance_m_);
  if (!tail) {
    // Cannot build a valid retrace path (e.g. not enough traveled history yet) -- treat as no
    // request rather than emitting a bogus path.
    requested_distance_m_.reset();
    status_.retrace_path = PathWithLaneId{};
    status_.retrace_lanelets.clear();
    return;
  }

  status_.retrace_path = reversePathForRetrace(tail->path, parameters_->retrace_velocity_mps);
  status_.retrace_lanelets = tail->lanelets;
}

bool ReverseLaneFollowModule::isExecutionRequested() const
{
  return requested_distance_m_.has_value() && !status_.retrace_path.points.empty();
}

bool ReverseLaneFollowModule::isExecutionReady() const
{
  return !status_.retrace_path.points.empty();
}

BehaviorModuleOutput ReverseLaneFollowModule::plan()
{
  BehaviorModuleOutput output{};

  if (status_.retrace_path.points.empty()) {
    // Nothing to retrace (yet/anymore) -- do not clobber the previous module's output.
    return getPreviousModuleOutput();
  }

  output.path = status_.retrace_path;
  output.reference_path = status_.retrace_path;

  if (!status_.retrace_lanelets.empty()) {
    output.drivable_area_info.drivable_lanes = utils::generateDrivableLanes(status_.retrace_lanelets);
  }

  return output;
}

CandidateOutput ReverseLaneFollowModule::planCandidate() const
{
  return CandidateOutput(status_.retrace_path);
}

void ReverseLaneFollowModule::processOnEntry()
{
  status_.is_retracing = true;
  status_.requested_distance_m =
    requested_distance_m_.value_or(parameters_->default_retrace_distance_m);
}

void ReverseLaneFollowModule::processOnExit()
{
  status_.is_retracing = false;
  status_.retrace_path = PathWithLaneId{};
  status_.retrace_lanelets.clear();
  // Edge-triggered: completing (or aborting) a retrace always clears the request. A new
  // std_msgs::msg::Float64 publish is required to trigger the next one.
  requested_distance_m_.reset();
}

bool ReverseLaneFollowModule::canTransitSuccessState()
{
  if (status_.retrace_path.points.empty()) {
    return true;
  }

  const auto & end_pose = status_.retrace_path.points.back().point.pose;
  const auto remaining_distance = autoware_utils::calc_distance2d(getEgoPose(), end_pose);

  return remaining_distance < parameters_->goal_reach_tolerance_m;
}

}  // namespace autoware::behavior_path_planner
