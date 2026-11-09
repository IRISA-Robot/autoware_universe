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

#ifndef AUTOWARE__TURN_IN_PLACE_SUPERVISOR__GEOMETRY_HPP_
#define AUTOWARE__TURN_IN_PLACE_SUPERVISOR__GEOMETRY_HPP_

#include <cstddef>
#include <vector>

namespace autoware::turn_in_place_supervisor
{

constexpr int kDirectionCcw = 1;   // yaw increasing
constexpr int kDirectionCw = -1;   // yaw decreasing

struct Point2d
{
  double x{0.0};
  double y{0.0};
};

// Body rectangle relative to the rotation centre, x forward, y left.  All extents >= 0.
struct Footprint
{
  double front{0.0};
  double rear{0.0};
  double left{0.0};
  double right{0.0};
};

/// Normalize to (-pi, pi].
double normalize_angle(double angle);

/// Rotation (>= 0, < 2*pi) needed to go from `yaw` to `target` turning only in `direction`
/// (kDirectionCcw / kDirectionCw).
double rotation_in_direction(double yaw, double target, int direction);

/// Whether the body sweeps through no point while it rotates by `rotation` [rad] (signed, CCW
/// positive) about the rotation centre.  `points` are in the body frame at the start of the turn
/// (x forward, y left, origin = rotation centre).  The sweep is sampled every `step` [rad]
/// (both ends included); `margin` inflates the footprint on every side.
bool sweep_is_clear(
  const std::vector<Point2d> & points, const Footprint & footprint, double margin, double rotation,
  double step);

/// Radius of the circle the body sweeps when it spins a full turn (corner distance + margin).
double sweep_radius(const Footprint & footprint, double margin);

// A simple polygon (no holes), vertices in order; the closing edge is implicit.
using Polygon2d = std::vector<Point2d>;

/// Even-odd point-in-polygon.
bool point_in_polygon(const Point2d & p, const Polygon2d & polygon);

/// Inside at least one of the polygons (a union of road lanelets).
bool point_in_any(const Point2d & p, const std::vector<Polygon2d> & polygons);

/// Points covering the footprint inflated by `margin`: its outline every `spacing` and an
/// interior grid of the same spacing (so a hole in the area under the body is caught too).
std::vector<Point2d> footprint_samples(const Footprint & footprint, double margin, double spacing);

/// Whether the body stays inside the area (union of `area` polygons, given in the body frame at
/// the start of the turn, origin = rotation centre) while it rotates by `rotation` [rad] (signed,
/// CCW positive), sampled every `step` [rad].
bool sweep_inside_area(
  const std::vector<Polygon2d> & area, const Footprint & footprint, double margin, double rotation,
  double step, double spacing);

/// Whether a disk of `radius` around `centre` lies inside the area: its rim sampled every
/// `angular_step` and its centre.  A body spinning a full turn about `centre` stays inside it.
bool disk_inside_area(
  const std::vector<Polygon2d> & area, const Point2d & centre, double radius, double angular_step);

}  // namespace autoware::turn_in_place_supervisor

#endif  // AUTOWARE__TURN_IN_PLACE_SUPERVISOR__GEOMETRY_HPP_
