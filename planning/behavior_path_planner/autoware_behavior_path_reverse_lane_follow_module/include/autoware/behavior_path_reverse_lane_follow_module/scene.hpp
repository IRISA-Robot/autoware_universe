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

#ifndef AUTOWARE__BEHAVIOR_PATH_REVERSE_LANE_FOLLOW_MODULE__SCENE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_REVERSE_LANE_FOLLOW_MODULE__SCENE_HPP_

#include "autoware/behavior_path_planner_common/interface/scene_module_interface.hpp"
#include "autoware/behavior_path_reverse_lane_follow_module/data_structs.hpp"

#include <autoware_utils/ros/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <std_msgs/msg/float64.hpp>

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware_internal_planning_msgs::msg::PathWithLaneId;

class ReverseLaneFollowModule : public SceneModuleInterface
{
public:
  ReverseLaneFollowModule(
    const std::string & name, rclcpp::Node & node,
    const std::shared_ptr<ReverseLaneFollowParameters> & parameters,
    const std::shared_ptr<autoware_utils::InterProcessPollingSubscriber<std_msgs::msg::Float64>> &
      retrace_request_subscriber,
    const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
    std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
      objects_of_interest_marker_interface_ptr_map,
    const std::shared_ptr<PlanningFactorInterface> planning_factor_interface);

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  void updateData() override;
  BehaviorModuleOutput plan() override;
  CandidateOutput planCandidate() const override;
  void processOnEntry() override;
  void processOnExit() override;

  void updateModuleParams(const std::any & parameters) override
  {
    parameters_ = std::any_cast<std::shared_ptr<ReverseLaneFollowParameters>>(parameters);
  }

  void acceptVisitor(
    [[maybe_unused]] const std::shared_ptr<SceneModuleVisitor> & visitor) const override
  {
  }

  // This module's own "am I currently retracing" status flag -- the reverse-lane-follow analogue
  // of autoware_behavior_path_start_planner_module's `isDrivingForward()` /
  // `status_.driving_forward` pattern. Other modules (or a future Phase-2 gating layer) can query
  // this to decide whether to stay out of the way.
  bool isRetracing() const { return status_.is_retracing; }

private:
  bool canTransitSuccessState() override;
  bool canTransitFailureState() override { return false; }

  // Polls the manager-owned retrace-request subscriber and refreshes requested_distance_m_.
  // Never creates/destroys the subscription itself (see manager.hpp).
  void updateRetraceRequest();

  std::shared_ptr<ReverseLaneFollowParameters> parameters_;
  std::shared_ptr<autoware_utils::InterProcessPollingSubscriber<std_msgs::msg::Float64>>
    retrace_request_subscriber_;

  ReverseLaneFollowStatus status_;

  // Latest polled request: nullopt when no active request, else the requested distance in
  // meters (already validated against parameters_->min_retrace_distance_m and defaulted from
  // parameters_->default_retrace_distance_m when the request value is <= 0).
  std::optional<double> requested_distance_m_;

  // Identity of the last std_msgs::msg::Float64 seen from the polling subscriber. The subscriber
  // keeps returning the same cached message between publishes (Latest policy), so this is used to
  // tell "operator published a new request" apart from "nothing changed since last poll" --
  // request activation is edge-triggered on a new message, not level-triggered on its value.
  std_msgs::msg::Float64::ConstSharedPtr last_seen_request_msg_;
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_REVERSE_LANE_FOLLOW_MODULE__SCENE_HPP_
