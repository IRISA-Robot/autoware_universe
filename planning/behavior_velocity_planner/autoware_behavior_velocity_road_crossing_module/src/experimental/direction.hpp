// Copyright 2026 azzamwildan462
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef EXPERIMENTAL__DIRECTION_HPP_
#define EXPERIMENTAL__DIRECTION_HPP_

// Driving direction of the road_crossing module as plain functions (tested in
// test/test_direction.cpp, forward against reverse).
//
//   * gear (which bumper leads) = sign of the planned velocity from ego on, as the MPC decides
//     it.  planner_data.is_driving_forward compares the path yaw with the next point, and on a
//     reversed route the yaw is the travel direction, so it reads forward while reversing.
//   * entry / exit of the crossing = the centerline end the path reaches first / last, from the
//     path's own point order (travel order in both gears).  Not from the route handler: the
//     behavior_velocity planner's RouteHandler is built from the map only and never gets the
//     route, so getLaneletsFromId() there always returns the drawing direction.

#include <optional>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental::road_crossing
{

// Sign of the first velocity beyond eps; nullopt when all are ~0 (keep the previous gear).
inline std::optional<bool> gearFromVelocities(
  const std::vector<double> & velocities, const double eps = 1e-2)
{
  for (const double v : velocities) {
    if (v > eps) return true;
    if (v < -eps) return false;
  }
  return std::nullopt;
}

// base_link -> leading bumper (> 0).  min_longitudinal_offset is -rear_overhang.
inline double leadingEdgeOffset(
  const double max_longitudinal_offset, const double min_longitudinal_offset,
  const bool gear_forward)
{
  return gear_forward ? max_longitudinal_offset : -min_longitudinal_offset;
}

// The path runs through the crossing against its drawing direction: the path's own travel
// vector near the crossing (from one path point to the next -- the path is ordered in travel
// order in both gears) points from the centerline's back() towards its front().
inline bool travelAgainstDrawing(
  const double front_x, const double front_y, const double back_x, const double back_y,
  const double from_x, const double from_y, const double to_x, const double to_y)
{
  return (to_x - from_x) * (back_x - front_x) + (to_y - from_y) * (back_y - front_y) < 0.0;
}

// (px, py) beyond the exit, along entry -> exit (entry / exit in travel order).
inline bool pastExit(
  const double entry_x, const double entry_y, const double exit_x, const double exit_y,
  const double px, const double py)
{
  return (px - exit_x) * (exit_x - entry_x) + (py - exit_y) * (exit_y - entry_y) > 0.0;
}

}  // namespace autoware::behavior_velocity_planner::experimental::road_crossing

#endif  // EXPERIMENTAL__DIRECTION_HPP_
