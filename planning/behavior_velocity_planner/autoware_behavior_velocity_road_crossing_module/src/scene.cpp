// Copyright 2026 azzamwildan462
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "scene.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace autoware::behavior_velocity_planner
{

RoadCrossingModule::RoadCrossingModule(
  const int64_t module_id, const PlannerParam & planner_param, const rclcpp::Logger & logger,
  const rclcpp::Clock::SharedPtr clock,
  const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface)
: SceneModuleInterface(module_id, logger, clock, time_keeper, planning_factor_interface),
  planner_param_(planner_param)
{
  RCLCPP_INFO(
    logger_, "RoadCrossing module created for lanelet %ld", module_id);
}

bool RoadCrossingModule::modifyPathVelocity([[maybe_unused]] PathWithLaneId * path)
{
  // SKELETON: state machine + intersection check + stop-velocity insertion
  // are implemented in the next phase. For now we log once per scene and
  // return false (path unmodified).
  RCLCPP_INFO_ONCE(
    logger_, "RoadCrossing module is active on lanelet %ld (stub — no logic yet)",
    getModuleId());
  return false;
}

visualization_msgs::msg::MarkerArray RoadCrossingModule::createDebugMarkerArray()
{
  return visualization_msgs::msg::MarkerArray{};
}

autoware::motion_utils::VirtualWalls RoadCrossingModule::createVirtualWalls()
{
  return autoware::motion_utils::VirtualWalls{};
}

}  // namespace autoware::behavior_velocity_planner
