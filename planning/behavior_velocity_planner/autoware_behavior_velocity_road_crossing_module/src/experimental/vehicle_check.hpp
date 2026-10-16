// Copyright 2026 azzamwildan462
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#ifndef EXPERIMENTAL__VEHICLE_CHECK_HPP_
#define EXPERIMENTAL__VEHICLE_CHECK_HPP_

#include <cstddef>
#include <optional>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental::vehicle_check
{

// "Can this vehicle still stop?" check for a road crossing.
//
// A vehicle's predicted path assumes it keeps its speed.  It cannot know that the vehicle will
// stop at its red light, or yield to the robot when turning across the crossing.  So a vehicle
// whose path meets the robot's is only a danger when it no longer CAN stop comfortably before
// the meeting point and would get there before the robot is through -- or when it is already
// at the robot's corridor.

struct Point
{
  double x{0.0};
  double y{0.0};
};

// a point of the robot's path over the crossing; s is arc length along the robot's path
struct CorridorPoint
{
  double s{0.0};
  Point p;
};

struct Vehicle
{
  double speed{0.0};       // [m/s]
  double half_width{0.0};  // [m]
  // predicted paths, each starting at the vehicle's current position
  std::vector<std::vector<Point>> predicted_paths;
};

struct Params
{
  double stop_deceleration{3.0};  // [m/s^2] braking a vehicle is assumed to be able to apply
  double distance_margin{3.0};    // [m] it must be able to stop this much before the conflict
  double time_margin{2.0};        // [s] it must arrive this much after the robot has cleared
  double min_distance{3.0};       // [m] nearer than this to the robot's corridor: always danger
  double min_speed{0.5};          // [m/s] slower than this: standing (danger only if near)
};

struct Ego
{
  double s{0.0};              // robot base_link on its path
  double front_offset{0.0};   // [m] base_link to the leading edge
  double rear_offset{0.0};    // [m] base_link to the trailing edge
  double half_width{0.0};     // [m] half width of the robot plus lateral margin
  double crossing_speed{0.0}; // [m/s] speed used to time the robot through the crossing
};

struct Conflict
{
  double zone_start_s{0.0};  // first point of the robot's path the vehicle's path meets
  double zone_end_s{0.0};    // last one
  double distance{0.0};      // [m] along the vehicle's path to the zone
};

/**
 * @brief where (if anywhere) the vehicle's predicted path meets the robot's corridor.
 * The earliest meeting over all predicted paths is returned.
 */
std::optional<Conflict> findConflict(
  const std::vector<CorridorPoint> & corridor, const Vehicle & vehicle, double ego_half_width);

/**
 * @brief whether this conflict is a danger to the robot (see the comment at the top).
 */
bool isDanger(const Conflict & conflict, const Vehicle & vehicle, const Ego & ego, const Params & p);

/**
 * @brief the start of the nearest conflict zone ahead of the robot that is a danger, as an arc
 * length on the robot's path.  Zones the robot's leading edge has already reached are skipped:
 * stopping there would leave it standing in the vehicle's way.
 */
std::optional<double> findNearestDangerZone(
  const std::vector<CorridorPoint> & corridor, const std::vector<Vehicle> & vehicles,
  const Ego & ego, const Params & p);

}  // namespace autoware::behavior_velocity_planner::experimental::vehicle_check

#endif  // EXPERIMENTAL__VEHICLE_CHECK_HPP_
