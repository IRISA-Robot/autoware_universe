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

#include "autoware/behavior_path_reverse_lane_follow_module/manager.hpp"

#include "autoware_utils/ros/update_param.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{

void ReverseLaneFollowModuleManager::init(rclcpp::Node * node)
{
  // init manager interface
  initInterface(node, {});

  // NOTE: declared with in-code defaults (no config/*.yaml shipped yet -- see CMakeLists.txt
  // comment). Once tuning-engineer adds config/reverse_lane_follow.param.yaml these become
  // overridable keys under the `reverse_lane_follow.` namespace.
  ReverseLaneFollowParameters p{};
  const std::string ns = "reverse_lane_follow.";
  p.default_retrace_distance_m =
    node->declare_parameter<double>(ns + "default_retrace_distance_m", p.default_retrace_distance_m);
  p.min_retrace_distance_m =
    node->declare_parameter<double>(ns + "min_retrace_distance_m", p.min_retrace_distance_m);
  p.retrace_velocity_mps =
    node->declare_parameter<double>(ns + "retrace_velocity_mps", p.retrace_velocity_mps);
  p.goal_reach_tolerance_m =
    node->declare_parameter<double>(ns + "goal_reach_tolerance_m", p.goal_reach_tolerance_m);
  // [BIDIR-BUG-FIX #5/#6] goal-overshoot fix (2026-09-06): conservative deceleration magnitude
  // for reverse_lane_follow_utils::applyGoalDecelerationRamp() -- see data_structs.hpp for the
  // full rationale and physics.
  p.goal_stop_decel_mps2 =
    node->declare_parameter<double>(ns + "goal_stop_decel_mps2", p.goal_stop_decel_mps2);
  // [BIDIR-BUG-FIX #7/#8] goal-overshoot fix continued (2026-09-07): see data_structs.hpp for the
  // full rationale for both new params.
  p.goal_decel_ramp_resample_interval_m = node->declare_parameter<double>(
    ns + "goal_decel_ramp_resample_interval_m", p.goal_decel_ramp_resample_interval_m);
  p.reactive_goal_clamp_trigger_distance_m = node->declare_parameter<double>(
    ns + "reactive_goal_clamp_trigger_distance_m", p.reactive_goal_clamp_trigger_distance_m);
  p.reactive_goal_clamp_velocity_mps = node->declare_parameter<double>(
    ns + "reactive_goal_clamp_velocity_mps", p.reactive_goal_clamp_velocity_mps);
  p.route_reversed_forward_distance_m = node->declare_parameter<double>(
    ns + "route_reversed_forward_distance_m", p.route_reversed_forward_distance_m);
  p.route_reversed_backward_distance_m = node->declare_parameter<double>(
    ns + "route_reversed_backward_distance_m", p.route_reversed_backward_distance_m);
  // [BIDIR-BUG-FIX #10] ARRIVED-state fix (2026-09-07): see data_structs.hpp for the full
  // rationale.
  p.arrived_stop_velocity_mps =
    node->declare_parameter<double>(ns + "arrived_stop_velocity_mps", p.arrived_stop_velocity_mps);

  parameters_ = std::make_shared<ReverseLaneFollowParameters>(p);

  // See manager.hpp for why this subscriber must be owned here (persistent manager) rather than
  // by the dynamically created/destroyed ReverseLaneFollowModule instances.
  retrace_request_subscriber_ =
    autoware_utils::InterProcessPollingSubscriber<std_msgs::msg::Float64>::create_subscription(
      node, "~/input/reverse_lane_follow/retrace_request");
}

void ReverseLaneFollowModuleManager::updateModuleParams(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;

  auto p = parameters_;

  const std::string ns = "reverse_lane_follow.";
  update_param<double>(parameters, ns + "default_retrace_distance_m", p->default_retrace_distance_m);
  update_param<double>(parameters, ns + "min_retrace_distance_m", p->min_retrace_distance_m);
  update_param<double>(parameters, ns + "retrace_velocity_mps", p->retrace_velocity_mps);
  update_param<double>(parameters, ns + "goal_reach_tolerance_m", p->goal_reach_tolerance_m);
  update_param<double>(parameters, ns + "goal_stop_decel_mps2", p->goal_stop_decel_mps2);
  update_param<double>(
    parameters, ns + "goal_decel_ramp_resample_interval_m", p->goal_decel_ramp_resample_interval_m);
  update_param<double>(
    parameters, ns + "reactive_goal_clamp_trigger_distance_m",
    p->reactive_goal_clamp_trigger_distance_m);
  update_param<double>(
    parameters, ns + "reactive_goal_clamp_velocity_mps", p->reactive_goal_clamp_velocity_mps);
  update_param<double>(
    parameters, ns + "route_reversed_forward_distance_m", p->route_reversed_forward_distance_m);
  update_param<double>(
    parameters, ns + "route_reversed_backward_distance_m", p->route_reversed_backward_distance_m);
  update_param<double>(
    parameters, ns + "arrived_stop_velocity_mps", p->arrived_stop_velocity_mps);

  std::for_each(observers_.begin(), observers_.end(), [&p](const auto & observer) {
    if (!observer.expired()) observer.lock()->updateModuleParams(p);
  });
}

}  // namespace autoware::behavior_path_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_path_planner::ReverseLaneFollowModuleManager,
  autoware::behavior_path_planner::SceneModuleManagerInterface)
