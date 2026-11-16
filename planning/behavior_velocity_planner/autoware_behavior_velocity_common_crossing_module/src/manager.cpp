// Copyright 2026 azzamwildan462
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "manager.hpp"

#include <autoware_utils/ros/parameter.hpp>

#include <boost/geometry/algorithms/correct.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <cstdlib>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{
using autoware_utils::get_or_declare_parameter;

CommonCrossingModuleManager::CommonCrossingModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterface(node, getModuleName())
{
  const std::string ns(CommonCrossingModuleManager::getModuleName());
  auto & p = planner_param_;

  p.stop_distance_threshold =
    get_or_declare_parameter<double>(node, ns + ".stop_distance_threshold");
  p.stop_arrival_threshold = get_or_declare_parameter<double>(node, ns + ".stop_arrival_threshold");
  p.stop_state_speed = get_or_declare_parameter<double>(node, ns + ".stop_state_speed");
  p.sustained_clear_sec = get_or_declare_parameter<double>(node, ns + ".sustained_clear_sec");
  p.wait_warn_sec = get_or_declare_parameter<double>(node, ns + ".wait_warn_sec");
  p.always_stop_at_pre = get_or_declare_parameter<bool>(node, ns + ".always_stop_at_pre");

  p.block.car = get_or_declare_parameter<bool>(node, ns + ".block_on.car");
  p.block.truck = get_or_declare_parameter<bool>(node, ns + ".block_on.truck");
  p.block.bus = get_or_declare_parameter<bool>(node, ns + ".block_on.bus");
  p.block.trailer = get_or_declare_parameter<bool>(node, ns + ".block_on.trailer");
  p.block.motorcycle = get_or_declare_parameter<bool>(node, ns + ".block_on.motorcycle");
  p.block.bicycle = get_or_declare_parameter<bool>(node, ns + ".block_on.bicycle");
  p.block.pedestrian = get_or_declare_parameter<bool>(node, ns + ".block_on.pedestrian");
  p.block.unknown = get_or_declare_parameter<bool>(node, ns + ".block_on.unknown");

  p.stop_margin = get_or_declare_parameter<double>(node, ns + ".stop_margin");
  p.stop_zone_enable = get_or_declare_parameter<bool>(node, ns + ".stop_zone.enable");
  p.stop_zone_length = get_or_declare_parameter<double>(node, ns + ".stop_zone.length");
  p.stop_zone_hold_tolerance =
    get_or_declare_parameter<double>(node, ns + ".stop_zone.hold_tolerance");
  p.stop_zone_decel = get_or_declare_parameter<double>(node, ns + ".stop_zone.decel");
  p.stop_zone_pre_margin = get_or_declare_parameter<double>(node, ns + ".stop_zone.pre_margin");
  p.stop_zone_min_departure_dist =
    get_or_declare_parameter<double>(node, ns + ".stop_zone.min_departure_dist");

  crossing_tag_ = get_or_declare_parameter<std::string>(node, ns + ".tags.crossing");
  p.pre_tag = get_or_declare_parameter<std::string>(node, ns + ".tags.pre");
  area_tag_ = get_or_declare_parameter<std::string>(node, ns + ".tags.area_id");

  RCLCPP_INFO(
    logger_,
    "[CommonCrossing] tags %s / %s / %s, sustained_clear %.1f s, always_stop_at_pre %d, "
    "block car %d truck %d bus %d trailer %d motorcycle %d bicycle %d pedestrian %d unknown %d",
    crossing_tag_.c_str(), p.pre_tag.c_str(), area_tag_.c_str(), p.sustained_clear_sec,
    p.always_stop_at_pre, p.block.car, p.block.truck, p.block.bus, p.block.trailer,
    p.block.motorcycle, p.block.bicycle, p.block.pedestrian, p.block.unknown);
}

namespace
{
std::set<lanelet::Id> collectCrossingLaneletIdsOnPath(
  const Trajectory & path, const PlannerData & planner_data, const std::string & tag)
{
  std::set<lanelet::Id> result;
  if (!planner_data.route_handler_) return result;
  const auto map = planner_data.route_handler_->getLaneletMapPtr();
  if (!map) return result;

  std::unordered_set<lanelet::Id> visited;
  for (const auto & point : path.restore()) {
    for (const auto id : point.lane_ids) {
      if (!visited.insert(id).second) continue;
      try {
        const auto llt = map->laneletLayer.get(id);
        if (std::string(llt.attributeOr(tag.c_str(), "false")) == "true") result.insert(id);
      } catch (const lanelet::NoSuchPrimitiveError &) {
      }
    }
  }
  return result;
}

template <class Points>
common_crossing::Polygon2d toPolygon2d(const Points & points)
{
  common_crossing::Polygon2d poly;
  for (const auto & pt : points) {
    poly.outer().emplace_back(pt.x(), pt.y());
  }
  boost::geometry::correct(poly);
  return poly;
}
}  // namespace

std::vector<common_crossing::Polygon2d> CommonCrossingModuleManager::resolveAreas(
  const lanelet::ConstLanelet & crossing, const PlannerData & planner_data,
  std::string * source) const
{
  std::vector<common_crossing::Polygon2d> areas;
  const auto map = planner_data.route_handler_->getLaneletMapPtr();
  const std::string ids(crossing.attributeOr(area_tag_.c_str(), ""));

  std::stringstream ss(ids);
  std::string token;
  std::vector<std::string> used;
  while (std::getline(ss, token, ',')) {
    token.erase(0, token.find_first_not_of(" \t"));
    token.erase(token.find_last_not_of(" \t") + 1);
    if (token.empty()) continue;
    char * end = nullptr;
    const auto id = static_cast<lanelet::Id>(std::strtoll(token.c_str(), &end, 10));
    if (end == token.c_str() || *end != '\0') {
      RCLCPP_WARN(
        logger_, "CommonCrossing %ld: %s has a bad id '%s'", crossing.id(), area_tag_.c_str(),
        token.c_str());
      continue;
    }
    common_crossing::Polygon2d poly;
    if (map->polygonLayer.exists(id)) {
      poly = toPolygon2d(map->polygonLayer.get(id));
      used.push_back(token + " (polygon)");
    } else if (map->lineStringLayer.exists(id)) {
      poly = toPolygon2d(map->lineStringLayer.get(id));
      used.push_back(token + " (way)");
    } else {
      RCLCPP_WARN(
        logger_, "CommonCrossing %ld: check area %ld is not in the map", crossing.id(), id);
      continue;
    }
    if (poly.outer().size() < 4) {  // corrected ring: 3 points + closing point
      RCLCPP_WARN(
        logger_, "CommonCrossing %ld: check area %ld has fewer than 3 points", crossing.id(), id);
      continue;
    }
    areas.push_back(poly);
  }

  if (areas.empty()) {
    areas.push_back(toPolygon2d(crossing.polygon2d()));
    *source = "the crossing lanelet itself (no usable " + area_tag_ + ")";
    RCLCPP_WARN(
      logger_,
      "CommonCrossing %ld: no usable check area in tag %s='%s' -- checking the crossing "
      "lanelet itself (a car still inside the parking lot is not seen)",
      crossing.id(), area_tag_.c_str(), ids.c_str());
  } else {
    *source = area_tag_ + "=";
    for (size_t i = 0; i < used.size(); ++i) *source += (i ? ", " : "") + used[i];
  }
  return areas;
}

void CommonCrossingModuleManager::launchNewModules(
  const Trajectory & path, [[maybe_unused]] const rclcpp::Time & stamp,
  const PlannerData & planner_data)
{
  for (const auto id : collectCrossingLaneletIdsOnPath(path, planner_data, crossing_tag_)) {
    if (isModuleRegistered(id)) continue;
    const auto crossing = planner_data.route_handler_->getLaneletMapPtr()->laneletLayer.get(id);
    std::string source;
    auto areas = resolveAreas(crossing, planner_data, &source);
    registerModule(
      std::make_shared<CommonCrossingModule>(
        id, planner_param_, std::move(areas), source, logger_.get_child(getModuleName()),
        clock_, time_keeper_, planning_factor_interface_, node_),
      planner_data);
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterface> &)>
CommonCrossingModuleManager::getModuleExpiredFunction(
  const Trajectory & path, const PlannerData & planner_data)
{
  const auto ids = collectCrossingLaneletIdsOnPath(path, planner_data, crossing_tag_);
  return [ids](const std::shared_ptr<SceneModuleInterface> & scene_module) {
    return ids.count(scene_module->getModuleId()) == 0;
  };
}

}  // namespace autoware::behavior_velocity_planner::experimental

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_velocity_planner::experimental::CommonCrossingModulePlugin,
  autoware::behavior_velocity_planner::experimental::PluginInterface)
