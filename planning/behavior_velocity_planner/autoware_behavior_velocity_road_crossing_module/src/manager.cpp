// Copyright 2026 azzamwildan462
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "manager.hpp"

#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_utils/ros/parameter.hpp>

#include <memory>
#include <set>
#include <string>
#include <unordered_set>
#include <vector>

namespace autoware::behavior_velocity_planner
{
using autoware_utils::get_or_declare_parameter;

RoadCrossingModuleManager::RoadCrossingModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  const std::string ns(RoadCrossingModuleManager::getModuleName());

  // State machine
  planner_param_.stop_distance_threshold =
    get_or_declare_parameter<double>(node, ns + ".stop_distance_threshold");
  planner_param_.stop_arrival_threshold =
    get_or_declare_parameter<double>(node, ns + ".stop_arrival_threshold");
  planner_param_.stop_state_speed =
    get_or_declare_parameter<double>(node, ns + ".stop_state_speed");
  planner_param_.sustained_clear_sec =
    get_or_declare_parameter<double>(node, ns + ".sustained_clear_sec");
  planner_param_.timeout_sec = get_or_declare_parameter<double>(node, ns + ".timeout_sec");

  // Prediction check
  planner_param_.predict_horizon_sec =
    get_or_declare_parameter<double>(node, ns + ".predict_horizon_sec");
  planner_param_.min_confidence = get_or_declare_parameter<double>(node, ns + ".min_confidence");
  planner_param_.enable_dynamic_horizon =
    get_or_declare_parameter<bool>(node, ns + ".enable_dynamic_horizon");
  planner_param_.crossing_speed_for_horizon =
    get_or_declare_parameter<double>(node, ns + ".crossing_speed_for_horizon");

  // Object class filter
  planner_param_.block_on_car = get_or_declare_parameter<bool>(node, ns + ".block_on_car");
  planner_param_.block_on_truck = get_or_declare_parameter<bool>(node, ns + ".block_on_truck");
  planner_param_.block_on_bus = get_or_declare_parameter<bool>(node, ns + ".block_on_bus");
  planner_param_.block_on_trailer = get_or_declare_parameter<bool>(node, ns + ".block_on_trailer");
  planner_param_.block_on_motorcycle =
    get_or_declare_parameter<bool>(node, ns + ".block_on_motorcycle");
  planner_param_.block_on_bicycle = get_or_declare_parameter<bool>(node, ns + ".block_on_bicycle");
  planner_param_.block_on_pedestrian =
    get_or_declare_parameter<bool>(node, ns + ".block_on_pedestrian");
  planner_param_.block_on_unknown = get_or_declare_parameter<bool>(node, ns + ".block_on_unknown");
}

namespace
{
// Collect lanelet IDs on the current path that carry the `road_crossing:true` tag.
std::set<int64_t> collectCrossingLaneletIdsOnPath(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
  const lanelet::LaneletMapConstPtr & lanelet_map)
{
  std::set<int64_t> result;
  if (!lanelet_map) {
    return result;
  }
  std::unordered_set<int64_t> visited;
  for (const auto & point : path.points) {
    for (const auto id : point.lane_ids) {
      if (!visited.insert(id).second) continue;
      try {
        const auto lanelet = lanelet_map->laneletLayer.get(id);
        // attributeOr(const char*, const char*) returns const char*; wrap explicitly
        // in std::string to get value-equality.
        const std::string attr_value(lanelet.attributeOr("road_crossing", "false"));
        if (attr_value == "true") {
          result.insert(id);
        }
      } catch (const lanelet::NoSuchPrimitiveError &) {
        // lanelet not in this map; skip
      }
    }
  }
  return result;
}
}  // namespace

void RoadCrossingModuleManager::launchNewModules(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  if (!planner_data_ || !planner_data_->route_handler_) {
    return;
  }
  const auto lanelet_map = planner_data_->route_handler_->getLaneletMapPtr();
  const auto crossing_ids = collectCrossingLaneletIdsOnPath(path, lanelet_map);

  for (const auto lane_id : crossing_ids) {
    if (isModuleRegistered(lane_id)) continue;
    registerModule(
      std::make_shared<RoadCrossingModule>(
        lane_id, planner_param_, logger_.get_child(getModuleName()), clock_, time_keeper_,
        planning_factor_interface_));
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
RoadCrossingModuleManager::getModuleExpiredFunction(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  std::set<int64_t> crossing_ids;
  if (planner_data_ && planner_data_->route_handler_) {
    crossing_ids =
      collectCrossingLaneletIdsOnPath(path, planner_data_->route_handler_->getLaneletMapPtr());
  }
  return [crossing_ids](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return crossing_ids.count(scene_module->getModuleId()) == 0;
  };
}

}  // namespace autoware::behavior_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_velocity_planner::RoadCrossingModulePlugin,
  autoware::behavior_velocity_planner::PluginInterface)
