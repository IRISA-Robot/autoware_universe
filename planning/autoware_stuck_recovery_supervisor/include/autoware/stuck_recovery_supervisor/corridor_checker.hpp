// Copyright 2026 azzamwildan462
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__STUCK_RECOVERY_SUPERVISOR__CORRIDOR_CHECKER_HPP_
#define AUTOWARE__STUCK_RECOVERY_SUPERVISOR__CORRIDOR_CHECKER_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <optional>
#include <vector>

namespace autoware::stuck_recovery_supervisor
{

// ---------------------------------------------------------------------------
// One geometric primitive, two questions.
//
//   PROBE : "is there lateral room to manoeuvre around the blockage?"
//   EXIT  : "is the original path clear again?"
//
// Keeping them in one sweep means the two decisions can never disagree about what
// counts as free space.
//
// Sides are kept strictly separate and are never merged.  At each station the
// perpendicular profile is cut into maximal free spans; a span is LEFT or RIGHT
// according to which side of the blockage it lies on.  The per-side widths are then
// aggregated with a MINIMUM along the corridor, so a reported width means "the robot
// fits through on this one side for the whole stretch".  Merging the sides, or taking
// the best side per station, would silently accept a path that hops around the
// obstacle on the left and then back around the next one on the right -- which is not
// a path the robot can drive, and gets more wrong the more obstacles are detected.
// ---------------------------------------------------------------------------

enum class Side : uint8_t { NONE = 0, LEFT = 1, RIGHT = 2 };

struct CorridorParams
{
  double step_m{0.25};                // arc-length spacing between sample stations
  // Only sweep this far along the path.  The reference path now runs much further so
  // that goal candidates can be searched out towards the real mission goal, but the
  // corridor question is local -- and stations beyond the 20 m costmap window would
  // read as blocked and drag the minimum to zero.
  double max_arc_m{8.0};
  double max_scan_half_width_m{4.0};  // how far sideways a ray is allowed to travel
  // How far sideways the robot may work its way per metre travelled.  The corridor is
  // measured relative to the path, and a real free band drifts as the path curves, so
  // demanding one fixed offset the whole way is not a test any real detour passes.
  double max_lateral_shift_per_m{1.0};
  double vehicle_width_m{0.78};
  double lateral_margin_m{0.15};
  int8_t occupancy_threshold{50};  // cell >= this is occupied; unknown (-1) counts too

  // Left/right must be the VEHICLE's left and right, not the path's.
  //
  // For a reversed route the path is built from inverted lanelet centerlines, whose
  // point order -- and therefore the pose yaw -- follows the direction of *travel*
  // (see laneletEntryTangentRad in the reverse_lane_follow module).  The robot is
  // driving backwards along it, so it physically faces the other way and the normal
  // taken from the pose yaw comes out mirrored.  The caller compares the path yaw
  // with the actual ego heading and sets this when they disagree.
  bool mirror_lateral{false};
};

struct CorridorResult
{
  bool valid{false};  // false when the grid or path could not be sampled at all

  // Narrowest usable gap on each side, taken independently along the whole corridor.
  double free_left_m{0.0};
  double free_right_m{0.0};
  // The side the robot would actually use, and its width. Never a mix of the two.
  double free_width_m{0.0};
  Side best_side{Side::NONE};
  // Signed lateral offset (+left) from the path to the centre of that gap, at the
  // narrowest station -- how far off the centerline the manoeuvre has to go.
  double best_offset_m{0.0};

  double required_width_m{0.0};

  bool clear{false};  // EXIT: robot footprint fits centred on the path, everywhere
  size_t stations{0};
  size_t first_blocked_station{0};  // meaningful only when !clear
  // Arc distance from the sweep start to that first blocked station -- i.e. how far
  // ahead the blockage actually is.  The escape goal is placed beyond this, so it is
  // guaranteed to sit on the far side of the obstacle.
  double first_blocked_arc_m{0.0};
  bool has_blockage{false};
  geometry_msgs::msg::Pose narrowest_pose{};
};

/// Sample `path` from `start_index` onward and measure the free lateral corridor
/// around it in `grid`.  Poses are expected in the same frame as the grid.
CorridorResult sweep_corridor(
  const nav_msgs::msg::OccupancyGrid & grid, const std::vector<geometry_msgs::msg::Pose> & path,
  size_t start_index, const CorridorParams & params);

/// Index of the path pose closest to `pose`.  Returns nullopt for an empty path.
std::optional<size_t> find_nearest_index(
  const std::vector<geometry_msgs::msg::Pose> & path, const geometry_msgs::msg::Pose & pose);

}  // namespace autoware::stuck_recovery_supervisor

#endif  // AUTOWARE__STUCK_RECOVERY_SUPERVISOR__CORRIDOR_CHECKER_HPP_
