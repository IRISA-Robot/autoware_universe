// Copyright 2026 azzamwildan462
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef MANAGER_HPP_
#define MANAGER_HPP_

#include "scene.hpp"

#include <autoware/behavior_velocity_planner_common/plugin_interface.hpp>
#include <autoware/behavior_velocity_planner_common/plugin_wrapper.hpp>
#include <autoware/behavior_velocity_planner_common/scene_module_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>

#include <functional>
#include <memory>

namespace autoware::behavior_velocity_planner
{

class RoadCrossingModuleManager
: public autoware::behavior_velocity_planner::SceneModuleManagerInterface<>
{
public:
  explicit RoadCrossingModuleManager(rclcpp::Node & node);

  const char * getModuleName() override { return "road_crossing"; }

  RequiredSubscriptionInfo getRequiredSubscriptions() const override
  {
    // STUB: predicted_objects required for next phase (prediction-path check).
    // For skeleton this can stay default; flip when wiring obstacle scan.
    return RequiredSubscriptionInfo{};
  }

private:
  RoadCrossingModule::PlannerParam planner_param_{};

  void launchNewModules(const autoware_internal_planning_msgs::msg::PathWithLaneId & path) override;

  std::function<bool(const std::shared_ptr<SceneModuleInterface> &)> getModuleExpiredFunction(
    const autoware_internal_planning_msgs::msg::PathWithLaneId & path) override;
};

class RoadCrossingModulePlugin
: public autoware::behavior_velocity_planner::PluginWrapper<RoadCrossingModuleManager>
{
};

}  // namespace autoware::behavior_velocity_planner

#endif  // MANAGER_HPP_
