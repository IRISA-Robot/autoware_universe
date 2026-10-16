// Copyright 2026 azzamwildan462
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "vehicle_check.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace autoware::behavior_velocity_planner::experimental::vehicle_check
{

std::optional<Conflict> findConflict(
  const std::vector<CorridorPoint> & corridor, const Vehicle & vehicle, const double ego_half_width)
{
  if (corridor.empty()) {
    return std::nullopt;
  }
  const double reach = ego_half_width + vehicle.half_width;
  const double reach_squared = reach * reach;

  // the span [min s, max s] of the corridor points within reach of p: the stretch of the robot's
  // path the vehicle's body covers there
  const auto span_in_reach = [&](const Point & p) -> std::optional<std::pair<double, double>> {
    std::optional<std::pair<double, double>> span;
    for (const auto & c : corridor) {
      const double dx = c.p.x - p.x;
      const double dy = c.p.y - p.y;
      if (dx * dx + dy * dy > reach_squared) {
        continue;
      }
      if (!span) {
        span = std::make_pair(c.s, c.s);
      } else {
        span->first = std::min(span->first, c.s);
        span->second = std::max(span->second, c.s);
      }
    }
    return span;
  };

  std::optional<Conflict> earliest;
  for (const auto & path : vehicle.predicted_paths) {
    double distance = 0.0;
    std::optional<Conflict> conflict;
    for (size_t i = 0; i < path.size(); ++i) {
      if (i > 0) {
        distance += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
      }
      const auto span = span_in_reach(path[i]);
      if (!span) {
        if (conflict) {
          break;  // through the corridor
        }
        continue;
      }
      if (!conflict) {
        conflict = Conflict{span->first, span->second, distance};
      } else {
        conflict->zone_start_s = std::min(conflict->zone_start_s, span->first);
        conflict->zone_end_s = std::max(conflict->zone_end_s, span->second);
      }
    }
    if (conflict && (!earliest || conflict->distance < earliest->distance)) {
      earliest = conflict;
    }
  }
  return earliest;
}

bool isDanger(const Conflict & conflict, const Vehicle & vehicle, const Ego & ego, const Params & p)
{
  if (conflict.distance < p.min_distance) {
    return true;  // at the robot's corridor already
  }
  if (vehicle.speed < p.min_speed) {
    return false;  // standing clear of it
  }
  const double stopping_distance =
    vehicle.speed * vehicle.speed / (2.0 * std::max(p.stop_deceleration, 1e-3));
  if (stopping_distance < conflict.distance - p.distance_margin) {
    return false;  // can still stop: assume it will (red light, or yielding to the robot)
  }
  // It cannot stop.  A danger only if it gets there before the robot is through.
  const double vehicle_arrival = conflict.distance / vehicle.speed;
  const double robot_clear =
    std::max(conflict.zone_end_s - (ego.s - ego.rear_offset), 0.0) /
    std::max(ego.crossing_speed, 1e-3);
  return vehicle_arrival < robot_clear + p.time_margin;
}

std::optional<double> findNearestDangerZone(
  const std::vector<CorridorPoint> & corridor, const std::vector<Vehicle> & vehicles,
  const Ego & ego, const Params & p)
{
  const double ego_front_s = ego.s + ego.front_offset;
  std::optional<double> nearest;
  for (const auto & vehicle : vehicles) {
    const auto conflict = findConflict(corridor, vehicle, ego.half_width);
    if (!conflict) {
      continue;
    }
    if (conflict->zone_start_s <= ego_front_s) {
      continue;  // already in (or past) it: going on is the way out
    }
    if (!isDanger(*conflict, vehicle, ego, p)) {
      continue;
    }
    if (!nearest || conflict->zone_start_s < *nearest) {
      nearest = conflict->zone_start_s;
    }
  }
  return nearest;
}

}  // namespace autoware::behavior_velocity_planner::experimental::vehicle_check
