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

#ifndef AUTOWARE__BEHAVIOR_PATH_REVERSE_LANE_FOLLOW_MODULE__MANAGER_HPP_
#define AUTOWARE__BEHAVIOR_PATH_REVERSE_LANE_FOLLOW_MODULE__MANAGER_HPP_

#include "autoware/behavior_path_planner_common/interface/scene_module_manager_interface.hpp"
#include "autoware/behavior_path_reverse_lane_follow_module/data_structs.hpp"
#include "autoware/behavior_path_reverse_lane_follow_module/scene.hpp"

#include <autoware_utils/ros/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float64.hpp>

#include <lanelet2_core/primitives/Lanelet.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{

class ReverseLaneFollowModuleManager : public SceneModuleManagerInterface
{
public:
  ReverseLaneFollowModuleManager() : SceneModuleManagerInterface{"reverse_lane_follow"} {}

  void init(rclcpp::Node * node) override;

  std::unique_ptr<SceneModuleInterface> createNewSceneModuleInstance() override
  {
    return std::make_unique<ReverseLaneFollowModule>(
      name_, *node_, parameters_, retrace_request_subscriber_, rtc_interface_ptr_map_,
      objects_of_interest_marker_interface_ptr_map_, planning_factor_interface_,
      last_exited_inverted_lanelet_id_);
  }

  void updateModuleParams(const std::vector<rclcpp::Parameter> & parameters) override;

private:
  std::shared_ptr<ReverseLaneFollowParameters> parameters_;

  // Trigger input for the "retrace last N meters" request (topic: ~/input/reverse_lane_follow/
  // retrace_request, std_msgs/Float64; value <= 0 means "no active request", value > 0 is the
  // requested retrace distance in meters).
  //
  // Created once here, in the manager, which pluginlib constructs a single time and which lives
  // for the node's whole lifetime. Scene-module instances (ReverseLaneFollowModule), by contrast,
  // are dynamically created/destroyed by the planner manager as the module goes idle <->
  // approved <-> idle again. Owning a live rclcpp::Subscription inside one of those instances
  // would repeatedly add/remove it from the executor's wait set on every such transition, which
  // is exactly the known behavior_planning_container SIGABRT mechanism (dynamic subscription
  // churn corrupting the executor). Keeping the subscriber itself owned by this always-alive
  // manager and only *polling* it (via ::take_data()) from the module avoids that; every module
  // instance the manager creates gets a shared_ptr to the same, never-recreated subscriber.
  std::shared_ptr<autoware_utils::InterProcessPollingSubscriber<std_msgs::msg::Float64>>
    retrace_request_subscriber_;

  // [BIDIR-BUG-FIX] Same "must not live inside the dynamically-created/destroyed module
  // instance" constraint as retrace_request_subscriber_ above, for a different reason here: this
  // is the one-way latch (see scene.hpp) that stops is_route_reversed_active from flip-flopping
  // right at a mid-route direction-change boundary. Every reactivation attempt after a
  // true->false transition is evaluated on a *brand-new* ReverseLaneFollowModule instance (the
  // planner manager creates a fresh one via createNewSceneModuleInstance() each time), so a plain
  // instance member would reset to empty on every single re-attempt and never actually suppress
  // anything -- confirmed live: the flip-flop was completely unaffected until this was moved here.
  // Owned by this always-alive manager and handed to every instance as a shared_ptr so it survives
  // across create/destroy cycles.
  std::shared_ptr<std::optional<lanelet::Id>> last_exited_inverted_lanelet_id_ =
    std::make_shared<std::optional<lanelet::Id>>(std::nullopt);
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_REVERSE_LANE_FOLLOW_MODULE__MANAGER_HPP_
