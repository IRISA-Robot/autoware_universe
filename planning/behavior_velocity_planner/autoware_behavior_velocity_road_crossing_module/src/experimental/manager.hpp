// Copyright 2026 azzamwildan462
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef EXPERIMENTAL__MANAGER_HPP_
#define EXPERIMENTAL__MANAGER_HPP_

#include "scene.hpp"

#include <autoware/behavior_velocity_planner_common/experimental/plugin_wrapper.hpp>

#include <autoware_road_crossing_msgs/msg/road_crossing_gate.hpp>

#include <functional>
#include <map>
#include <memory>
#include <mutex>

namespace autoware::behavior_velocity_planner::experimental
{

class RoadCrossingModuleManager : public SceneModuleManagerInterface<>
{
public:
  explicit RoadCrossingModuleManager(rclcpp::Node & node);

  const char * getModuleName() override { return "road_crossing"; }

  RequiredSubscriptionInfo getRequiredSubscriptions() const override
  {
    RequiredSubscriptionInfo info;
    info.predicted_objects = true;
    return info;
  }

private:
  RoadCrossingModule::PlannerParam planner_param_;

  // Manager-owned gate subscription.
  // Created once in the constructor (when require_fsm_gate=true) and lives for the
  // lifetime of the node — never destroyed while modules are created/destroyed.
  // This avoids the wait_set SIGABRT triggered by per-module subscriptions.
  rclcpp::Subscription<autoware_road_crossing_msgs::msg::RoadCrossingGate>::SharedPtr sub_gate_;

  // Map: lanelet_id → latest gate command.  Written by sub_gate_ callback;
  // read in modifyPathVelocity() before iterating modules.
  std::map<lanelet::Id, uint8_t> gate_commands_;
  mutable std::mutex gate_mutex_;

  void launchNewModules(
    const Trajectory & path, const rclcpp::Time & stamp,
    const PlannerData & planner_data) override;

  std::function<bool(const std::shared_ptr<SceneModuleInterface> &)> getModuleExpiredFunction(
    const Trajectory & path, const PlannerData & planner_data) override;

  /**
   * Override the base modifyPathVelocity to push gate commands into each active
   * module before the base class calls module->modifyPathVelocity().
   */
  void modifyPathVelocity(
    Trajectory & path, const std_msgs::msg::Header & header,
    const std::vector<geometry_msgs::msg::Point> & left_bound,
    const std::vector<geometry_msgs::msg::Point> & right_bound,
    const PlannerData & planner_data) override;
};

class RoadCrossingModulePlugin : public PluginWrapper<RoadCrossingModuleManager>
{
};

}  // namespace autoware::behavior_velocity_planner::experimental

#endif  // EXPERIMENTAL__MANAGER_HPP_
