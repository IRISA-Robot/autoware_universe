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

#ifndef AUTOWARE__ROAD_CROSSING_FSM__VEHICLE_CHECK_HPP_
#define AUTOWARE__ROAD_CROSSING_FSM__VEHICLE_CHECK_HPP_

// Vehicle check before the robot starts to cross (2026-11-20), as plain geometry (no ROS):
// a vehicle (CAR / TRUCK / BUS ...) whose footprint lies on the planned path over the crossing
// -- the path widened by half the robot width + a lateral margin -- holds the GO.  Optionally
// the footprint is also swept along the vehicle's own velocity for lookahead_sec (a straight
// line, not the predicted paths), so a car about to drive onto the path holds it too.

#include "autoware/road_crossing_fsm/direction_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace autoware::road_crossing_fsm::vehicle_check
{

using direction::Point2;
using direction::Polyline;

// The path the check covers: the planned trajectory from the point nearest ego up to the point
// nearest the crossing exit (crossing centerline in travel order, back() = exit).  When the
// trajectory stops short of the exit (or is missing) the rest of the crossing centerline is
// appended, so the whole crossing is always covered.
inline Polyline buildCorridor(
  const Polyline & trajectory, const Point2 & ego, const Polyline & crossing_centerline)
{
  Polyline out;
  if (crossing_centerline.size() < 2) return trajectory;
  const Point2 & exit = crossing_centerline.back();
  if (trajectory.size() >= 2) {
    size_t i0 = 0;
    double best = std::numeric_limits<double>::max();
    for (size_t i = 0; i < trajectory.size(); ++i) {
      const double d = direction::distance(trajectory[i], ego);
      if (d < best) {
        best = d;
        i0 = i;
      }
    }
    size_t j = i0;
    best = std::numeric_limits<double>::max();
    for (size_t i = i0; i < trajectory.size(); ++i) {
      const double d = direction::distance(trajectory[i], exit);
      if (d < best) {
        best = d;
        j = i;
      }
    }
    out.assign(trajectory.begin() + static_cast<std::ptrdiff_t>(i0),
               trajectory.begin() + static_cast<std::ptrdiff_t>(j) + 1);
  } else {
    out.push_back(ego);
  }
  // Short of the exit: continue on the crossing centerline after the last corridor point.
  constexpr double kReachedExitM = 1.0;
  if (direction::distance(out.back(), exit) > kReachedExitM) {
    const auto proj = direction::project(crossing_centerline, out.back());
    for (size_t k = proj.segment + 1; k < crossing_centerline.size(); ++k) {
      out.push_back(crossing_centerline[k]);
    }
  }
  if (out.size() == 1) out.push_back(out.front());
  return out;
}

inline double cross(const Point2 & o, const Point2 & a, const Point2 & b)
{
  return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

inline double pointSegmentDistance(const Point2 & p, const Point2 & a, const Point2 & b)
{
  const double vx = b.x - a.x;
  const double vy = b.y - a.y;
  const double len2 = vx * vx + vy * vy;
  double t = len2 > 0.0 ? ((p.x - a.x) * vx + (p.y - a.y) * vy) / len2 : 0.0;
  t = std::clamp(t, 0.0, 1.0);
  return std::hypot(p.x - (a.x + t * vx), p.y - (a.y + t * vy));
}

inline bool segmentsIntersect(const Point2 & a, const Point2 & b, const Point2 & c, const Point2 & d)
{
  const double d1 = cross(c, d, a);
  const double d2 = cross(c, d, b);
  const double d3 = cross(a, b, c);
  const double d4 = cross(a, b, d);
  return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0)) && d1 != 0 && d2 != 0 && d3 != 0 &&
         d4 != 0;
}

inline double segmentSegmentDistance(
  const Point2 & a, const Point2 & b, const Point2 & c, const Point2 & d)
{
  if (segmentsIntersect(a, b, c, d)) return 0.0;
  return std::min(
    {pointSegmentDistance(a, c, d), pointSegmentDistance(b, c, d), pointSegmentDistance(c, a, b),
     pointSegmentDistance(d, a, b)});
}

inline bool pointInPolygon(const Point2 & p, const Polyline & poly)
{
  bool inside = false;
  for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
    if (((poly[i].y > p.y) != (poly[j].y > p.y)) &&
        (p.x < (poly[j].x - poly[i].x) * (p.y - poly[i].y) / (poly[j].y - poly[i].y) + poly[i].x)) {
      inside = !inside;
    }
  }
  return inside;
}

// Smallest distance between a closed polygon (footprint) and the corridor centre polyline
// (0 when they cross or the polyline runs inside the polygon).
inline double footprintToPathDistance(const Polyline & footprint, const Polyline & path)
{
  if (footprint.empty() || path.empty()) return std::numeric_limits<double>::max();
  double best = std::numeric_limits<double>::max();
  if (footprint.size() >= 3) {
    for (const auto & p : path) {
      if (pointInPolygon(p, footprint)) return 0.0;
    }
  }
  for (size_t i = 0; i < footprint.size(); ++i) {
    const auto & a = footprint[i];
    const auto & b = footprint[(i + 1) % footprint.size()];
    for (size_t k = 0; k + 1 < path.size(); ++k) {
      best = std::min(best, segmentSegmentDistance(a, b, path[k], path[k + 1]));
      if (best <= 0.0) return 0.0;
    }
    if (path.size() == 1) best = std::min(best, pointSegmentDistance(path[0], a, b));
  }
  return best;
}

// Rectangle footprint (bounding box length x width) around (cx, cy) with heading yaw.
inline Polyline boxFootprint(
  const double cx, const double cy, const double yaw, const double length, const double width)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double hl = 0.5 * length;
  const double hw = 0.5 * width;
  Polyline out;
  for (const auto & [lx, ly] : {std::pair{hl, hw}, {-hl, hw}, {-hl, -hw}, {hl, -hw}}) {
    out.push_back({cx + c * lx - s * ly, cy + s * lx + c * ly});
  }
  return out;
}

struct Result
{
  bool hit{false};
  double distance{std::numeric_limits<double>::max()};  // footprint -> corridor centre line
  bool by_motion{false};  // hit only with the swept (lookahead) footprint, not where it is now
};

// Does a vehicle footprint (map frame, now) touch the corridor?  half_width = half the robot
// width + the lateral margin.  With lookahead_sec > 0 and a speed >= min_speed_mps, the
// footprint is also moved along (vx, vy) in steps of at most sweep_step_m up to lookahead_sec.
inline Result checkVehicle(
  const Polyline & corridor, const double half_width, const Polyline & footprint, const double vx,
  const double vy, const double lookahead_sec, const double min_speed_mps,
  const double sweep_step_m = 0.5)
{
  Result r;
  r.distance = footprintToPathDistance(footprint, corridor);
  if (r.distance <= half_width) {
    r.hit = true;
    return r;
  }
  const double speed = std::hypot(vx, vy);
  if (lookahead_sec <= 0.0 || speed < min_speed_mps || speed <= 0.0) return r;
  const double travel = speed * lookahead_sec;
  const int steps = std::max(1, static_cast<int>(std::ceil(travel / std::max(sweep_step_m, 0.05))));
  for (int k = 1; k <= steps; ++k) {
    const double f = lookahead_sec * static_cast<double>(k) / steps;
    Polyline moved = footprint;
    for (auto & p : moved) {
      p.x += vx * f;
      p.y += vy * f;
    }
    const double d = footprintToPathDistance(moved, corridor);
    if (d <= half_width) {
      r.hit = true;
      r.by_motion = true;
      return r;
    }
  }
  return r;
}

}  // namespace autoware::road_crossing_fsm::vehicle_check

#endif  // AUTOWARE__ROAD_CROSSING_FSM__VEHICLE_CHECK_HPP_
