// Copyright 2026 azzamwildan462
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "manager.hpp"

#include <autoware_utils/ros/parameter.hpp>

#include <autoware_road_crossing_msgs/msg/road_crossing_gate.hpp>

#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <unordered_set>

namespace autoware::behavior_velocity_planner::experimental
{
using autoware_utils::get_or_declare_parameter;

RoadCrossingModuleManager::RoadCrossingModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  const std::string ns(RoadCrossingModuleManager::getModuleName());
  auto & p = planner_param_;

  p.stop_distance_threshold =
    get_or_declare_parameter<double>(node, ns + ".stop_distance_threshold");
  p.stop_arrival_threshold =
    get_or_declare_parameter<double>(node, ns + ".stop_arrival_threshold");
  p.stop_state_speed = get_or_declare_parameter<double>(node, ns + ".stop_state_speed");
  p.sustained_clear_sec =
    get_or_declare_parameter<double>(node, ns + ".sustained_clear_sec");
  p.timeout_sec = get_or_declare_parameter<double>(node, ns + ".timeout_sec");

  p.predict_horizon_sec =
    get_or_declare_parameter<double>(node, ns + ".predict_horizon_sec");
  p.min_confidence = get_or_declare_parameter<double>(node, ns + ".min_confidence");
  p.enable_dynamic_horizon =
    get_or_declare_parameter<bool>(node, ns + ".enable_dynamic_horizon");
  p.crossing_speed_for_horizon =
    get_or_declare_parameter<double>(node, ns + ".crossing_speed_for_horizon");

  p.block_on_car = get_or_declare_parameter<bool>(node, ns + ".block_on_car");
  p.block_on_truck = get_or_declare_parameter<bool>(node, ns + ".block_on_truck");
  p.block_on_bus = get_or_declare_parameter<bool>(node, ns + ".block_on_bus");
  p.block_on_trailer = get_or_declare_parameter<bool>(node, ns + ".block_on_trailer");
  p.block_on_motorcycle = get_or_declare_parameter<bool>(node, ns + ".block_on_motorcycle");
  p.block_on_bicycle = get_or_declare_parameter<bool>(node, ns + ".block_on_bicycle");
  p.block_on_pedestrian = get_or_declare_parameter<bool>(node, ns + ".block_on_pedestrian");
  p.block_on_unknown = get_or_declare_parameter<bool>(node, ns + ".block_on_unknown");
  p.stop_margin = get_or_declare_parameter<double>(node, ns + ".stop_margin");
  p.pre_stop_dist_m = get_or_declare_parameter<double>(node, ns + ".pre_stop_dist_m");
  p.stop_zone_enable = get_or_declare_parameter<bool>(node, ns + ".stop_zone_enable");
  p.stop_zone_length = get_or_declare_parameter<double>(node, ns + ".stop_zone_length");
  p.stop_zone_hold_tolerance =
    get_or_declare_parameter<double>(node, ns + ".stop_zone_hold_tolerance");
  p.stop_zone_decel = get_or_declare_parameter<double>(node, ns + ".stop_zone_decel");
  p.stop_zone_pre_margin = get_or_declare_parameter<double>(node, ns + ".stop_zone_pre_margin");
  p.stop_zone_min_departure_dist =
    get_or_declare_parameter<double>(node, ns + ".stop_zone_min_departure_dist");

  // FSM gate integration (default false = backward compatible).
  p.require_fsm_gate = get_or_declare_parameter<bool>(node, ns + ".require_fsm_gate");
  p.gate_topic = get_or_declare_parameter<std::string>(node, ns + ".gate_topic");

  // GREEN-cross bounded timeout: max seconds to wait while GO_GREEN + predicted-path blocked.
  p.green_block_timeout_sec =
    get_or_declare_parameter<double>(node, ns + ".green_block_timeout_sec");

  p.gate_stale_sec = get_or_declare_parameter<double>(node, ns + ".gate_stale_sec");

  // Create ONE persistent gate subscription in the manager (not per-module).
  // Scene modules are created/destroyed dynamically; creating subscriptions inside
  // them corrupts the executor wait_set (SIGABRT).  The manager lives for the full
  // node lifetime so its subscription is safe.
  if (p.require_fsm_gate) {
    sub_gate_ = node.create_subscription<autoware_road_crossing_msgs::msg::RoadCrossingGate>(
      // Depth 10: the FSM publishes the PRE and the ROAD gate back to back every tick;
      // depth 1 lets the second overwrite the first before it is taken.
      p.gate_topic, rclcpp::QoS(10),
      [this](const autoware_road_crossing_msgs::msg::RoadCrossingGate::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(gate_mutex_);
        gate_commands_[static_cast<lanelet::Id>(msg->lanelet_id)] = {msg->command, clock_->now()};
      });
    RCLCPP_INFO(
      logger_,
      "[RoadCrossingModuleManager] FSM gate enabled — subscribed to '%s'",
      p.gate_topic.c_str());
  }
}

void RoadCrossingModuleManager::modifyPathVelocity(
  Trajectory & path, const std_msgs::msg::Header & header,
  const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound,
  const PlannerData & planner_data)
{
  // Push the latest gate commands from the manager's map into each active module
  // BEFORE calling the base class loop (which invokes module->modifyPathVelocity()).
  // This must happen each cycle so modules always see the freshest gate state.
  if (planner_param_.require_fsm_gate) {
    // Snapshot the gate map under lock, then push without holding the lock
    // (avoids holding gate_mutex_ while module logic runs).
    std::map<lanelet::Id, GateCommand> snapshot;
    {
      std::lock_guard<std::mutex> lk(gate_mutex_);
      snapshot = gate_commands_;
    }

    for (const auto & scene_module : scene_modules_) {
      auto * rc_module = dynamic_cast<RoadCrossingModule *>(scene_module.get());
      if (!rc_module) continue;
      const lanelet::Id id = static_cast<lanelet::Id>(rc_module->getModuleId());
      // No gate for this lanelet, or none recently: HOLD.  The FSM republishes the gate of
      // the crossing it is handling every tick; anything older is left over from an earlier
      // episode and must not release the robot.
      const auto it = snapshot.find(id);
      const bool fresh =
        it != snapshot.end() &&
        (clock_->now() - it->second.received).seconds() <= planner_param_.gate_stale_sec;
      rc_module->setGateCommand(
        fresh ? it->second.command : autoware_road_crossing_msgs::msg::RoadCrossingGate::HOLD);
    }
  }

  // Delegate to the base class which iterates scene_modules_ and calls each
  // module's modifyPathVelocity(), then publishes debug markers, virtual walls, etc.
  SceneModuleManagerInterface::modifyPathVelocity(
    path, header, left_bound, right_bound, planner_data);
}

namespace
{
// Walk the lane_ids on the trajectory; return IDs of lanelets carrying road_crossing:true tag.
// pre_road_crossing lanelets get no instance: their stop existed for crosswalk detection,
// which is gone, so the robot drives straight on to the ROAD stop line (the pre/road border).
// Returns a map from lanelet ID → CrossingKind.
std::map<lanelet::Id, RoadCrossingModule::CrossingKind> collectCrossingLaneletIdsOnPath(
  const Trajectory & path, const PlannerData & planner_data)
{
  std::map<lanelet::Id, RoadCrossingModule::CrossingKind> result;
  if (!planner_data.route_handler_) return result;
  const auto map = planner_data.route_handler_->getLaneletMapPtr();
  if (!map) return result;

  std::unordered_set<lanelet::Id> visited;
  const auto path_msg = path.restore();
  for (const auto & point : path_msg) {
    for (const auto id : point.lane_ids) {
      if (!visited.insert(id).second) continue;
      try {
        const auto llt = map->laneletLayer.get(id);
        const std::string road_attr(llt.attributeOr("road_crossing", "false"));
        if (road_attr == "true") {
          result[id] = RoadCrossingModule::CrossingKind::ROAD;
        }
      } catch (const lanelet::NoSuchPrimitiveError &) {
        // skip
      }
    }
  }
  return result;
}
}  // namespace

void RoadCrossingModuleManager::launchNewModules(
  const Trajectory & path, [[maybe_unused]] const rclcpp::Time & stamp,
  const PlannerData & planner_data)
{
  for (const auto & [lane_id, kind] : collectCrossingLaneletIdsOnPath(path, planner_data)) {
    if (isModuleRegistered(lane_id)) continue;
    registerModule(
      std::make_shared<RoadCrossingModule>(
        lane_id, kind, planner_param_, logger_.get_child(getModuleName()), clock_, time_keeper_,
        planning_factor_interface_, node_),
      planner_data);
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
RoadCrossingModuleManager::getModuleExpiredFunction(
  const Trajectory & path, const PlannerData & planner_data)
{
  const auto crossing_map = collectCrossingLaneletIdsOnPath(path, planner_data);
  return [crossing_map](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return crossing_map.count(scene_module->getModuleId()) == 0;
  };
}

}  // namespace autoware::behavior_velocity_planner::experimental

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_velocity_planner::experimental::RoadCrossingModulePlugin,
  autoware::behavior_velocity_planner::experimental::PluginInterface)
