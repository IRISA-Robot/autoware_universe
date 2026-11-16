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

#include <autoware/behavior_velocity_planner_common/experimental/plugin_wrapper.hpp>

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{

class CommonCrossingModuleManager : public SceneModuleManagerInterface<>
{
public:
  explicit CommonCrossingModuleManager(rclcpp::Node & node);

  const char * getModuleName() override { return "common_crossing"; }

  RequiredSubscriptionInfo getRequiredSubscriptions() const override
  {
    RequiredSubscriptionInfo info;
    info.predicted_objects = true;
    return info;
  }

private:
  CommonCrossingModule::PlannerParam planner_param_;
  std::string crossing_tag_{"common_crossing"};
  std::string area_tag_{"common_crossing_area_id"};

  void launchNewModules(
    const Trajectory & path, const rclcpp::Time & stamp,
    const PlannerData & planner_data) override;

  std::function<bool(const std::shared_ptr<SceneModuleInterface> &)> getModuleExpiredFunction(
    const Trajectory & path, const PlannerData & planner_data) override;

  // Check areas of a common_crossing lanelet: the polygons named by its area tag, or the
  // lanelet's own polygon when there are none.  *source says which (for the log).
  std::vector<common_crossing::Polygon2d> resolveAreas(
    const lanelet::ConstLanelet & crossing, const PlannerData & planner_data,
    std::string * source) const;
};

class CommonCrossingModulePlugin : public PluginWrapper<CommonCrossingModuleManager>
{
};

}  // namespace autoware::behavior_velocity_planner::experimental

#endif  // MANAGER_HPP_
