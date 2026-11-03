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

#ifndef AUTOWARE__ROAD_CROSSING_FSM__DIRECTION_UTILS_HPP_
#define AUTOWARE__ROAD_CROSSING_FSM__DIRECTION_UTILS_HPP_

// Direction handling of the road-crossing FSM as plain functions (no ROS, no map), so forward
// and reverse driving can be tested against each other.
//
// Two different things, never mixed:
//   * travel order: which end of a lanelet ego reaches first.  Comes from the route (a lanelet
//     the route traverses against its drawing direction is inverted there), so every polyline
//     below is in TRAVEL order: front() = entry, back() = exit -- whatever the gear.
//   * gear: which bumper leads.  Comes from the sign of the planned velocity (the MPC does the
//     same, mpc.cpp), never from the path yaw, which is the travel direction on a reversed route.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::road_crossing_fsm::direction
{

struct Point2
{
  double x{0.0};
  double y{0.0};
};
using Polyline = std::vector<Point2>;

inline double wrapAngle(const double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

inline double distance(const Point2 & a, const Point2 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

inline double polylineLength(const Polyline & line)
{
  double len = 0.0;
  for (size_t i = 0; i + 1 < line.size(); ++i) len += distance(line[i], line[i + 1]);
  return len;
}

struct Projection
{
  double arc{0.0};       // arc length of the projection from front(), clamped to [0, length]
  size_t segment{0};     // segment (i, i + 1) the projection lies on
  double offset{0.0};    // distance from p to the projection
};

// Projection onto the nearest segment, clamped at both ends (lanelet::geometry::
// toArcCoordinates() does the same).  Requires line.size() >= 2.
inline Projection project(const Polyline & line, const Point2 & p)
{
  Projection best;
  best.offset = std::numeric_limits<double>::max();
  double acc = 0.0;
  for (size_t i = 0; i + 1 < line.size(); ++i) {
    const auto & a = line[i];
    const auto & b = line[i + 1];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len2 = dx * dx + dy * dy;
    const double seg_len = std::sqrt(len2);
    double u = len2 > 1e-12 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2 : 0.0;
    u = std::min(1.0, std::max(0.0, u));
    const Point2 q{a.x + u * dx, a.y + u * dy};
    const double d = distance(p, q);
    if (d < best.offset - 1e-12) {
      best.offset = d;
      best.arc = acc + u * seg_len;
      best.segment = i;
    }
    acc += seg_len;
  }
  return best;
}

// Distance from p to the entry (front()) of a travel-ordered centerline, >= 0, -1 if invalid.
// Along the centerline once p projects past the entry; straight-line while p is still before
// it (the projection clamps to the entry there and would read 0 = "arrived").
inline double distanceToEntry(const Polyline & line, const Point2 & p)
{
  if (line.size() < 2) return -1.0;
  const double s = project(line, p).arc;
  if (s < 1e-3) return distance(p, line.front());
  return s;
}

// p is before the entry of a travel-ordered centerline by more than margin.
inline bool upstreamOfEntry(const Polyline & line, const Point2 & p, const double margin)
{
  if (line.size() < 2) return false;
  if (project(line, p).arc >= 1e-3) return false;
  return distance(p, line.front()) > margin;
}

// p is beyond the exit (back()) of a travel-ordered centerline, along entry -> exit.
inline bool pastExit(const Polyline & line, const Point2 & p)
{
  if (line.size() < 2) return false;
  const auto & entry = line.front();
  const auto & exit = line.back();
  return (p.x - exit.x) * (exit.x - entry.x) + (p.y - exit.y) * (exit.y - entry.y) > 0.0;
}

// Yaw of the travel-ordered centerline where p projects onto it.
inline std::optional<double> tangentYawAt(const Polyline & line, const Point2 & p)
{
  if (line.size() < 2) return std::nullopt;
  const auto seg = project(line, p).segment;
  const auto & a = line[seg];
  const auto & b = line[seg + 1];
  if (distance(a, b) < 1e-9) return std::nullopt;
  return std::atan2(b.y - a.y, b.x - a.x);
}

// Ego moves along the route's travel order: its direction of motion (body yaw, + pi in reverse
// gear) within 90 deg of the travel-ordered centerline.  False when backing up along a forward
// route (retrace).
inline bool travelsInRouteOrder(
  const double body_yaw, const bool gear_forward, const double route_tangent_yaw)
{
  const double motion_yaw = body_yaw + (gear_forward ? 0.0 : M_PI);
  return std::abs(wrapAngle(motion_yaw - route_tangent_yaw)) < M_PI / 2.0;
}

// base_link -> leading bumper along the direction of motion (> 0).
inline double leadingEdgeOffset(const double front_m, const double rear_m, const bool gear_forward)
{
  return gear_forward ? front_m : rear_m;
}

// Leading bumper point: ahead of base_link along the body yaw in forward gear, behind it in
// reverse gear.
inline Point2 leadingEdgePoint(
  const Point2 & base_link, const double body_yaw, const double front_m, const double rear_m,
  const bool gear_forward)
{
  const double sign = gear_forward ? 1.0 : -1.0;
  const double d = leadingEdgeOffset(front_m, rear_m, gear_forward);
  return {base_link.x + sign * d * std::cos(body_yaw), base_link.y + sign * d * std::sin(body_yaw)};
}

// Gear from planned longitudinal velocities ordered from ego onwards: sign of the first one
// beyond eps; nullopt when all are ~0 (held at a stop) -- keep the previous gear then.
inline std::optional<bool> gearFromVelocities(
  const std::vector<double> & velocities, const double eps = 1e-2)
{
  for (const double v : velocities) {
    if (v > eps) return true;
    if (v < -eps) return false;
  }
  return std::nullopt;
}

// Ego can still stop before the crossing: leading-bumper gap to the entry >= braking distance
// at |speed| plus margin.
inline double stoppingDistance(const double speed, const double decel, const double margin)
{
  const double v = std::abs(speed);
  return v * v / (2.0 * std::max(decel, 0.1)) + margin;
}
inline bool canStopBefore(
  const double gap, const double speed, const double decel, const double margin)
{
  return gap >= stoppingDistance(speed, decel, margin);
}

// ---------------------------------------------------------------- paired pedestrian light

// The light to watch follows the side ego crosses from, not the gear: forward_pedestrian_tf
// when the route takes the crossing along its drawing direction, reverse_pedestrian_tf when
// against it.
inline const char * lightTagKey(const bool crossing_inverted_in_route)
{
  return crossing_inverted_in_route ? "reverse_pedestrian_tf" : "forward_pedestrian_tf";
}

struct TaggedLight
{
  int64_t id{0};
  std::optional<Point2> position;  // nullopt: not a tf_pedestrian way in the map
};

enum class LightChoice {
  FROM_TAG,        // the tag for this direction
  FROM_OTHER_TAG,  // this direction's tag empty -> the other one (same signal at both ends)
  SWAPPED_TAGS,    // this tag's light clearly on ego's side, the other's clearly across
  NONE             // no light tag at all: unsignalized
};

struct LightSelection
{
  std::vector<int64_t> ids;
  LightChoice choice{LightChoice::NONE};
};

// Fraction [0, 1] along the travel-ordered crossing where the light projects; nullopt if
// unknown.  Near 0 = ego's side, near 1 = the far side, where the light ego must read stands.
inline std::optional<double> lightFraction(const Polyline & crossing, const TaggedLight & light)
{
  if (!light.position || crossing.size() < 2) return std::nullopt;
  const double len = polylineLength(crossing);
  if (len < 1e-6) return std::nullopt;
  return project(crossing, *light.position).arc / len;
}

inline bool allClearly(
  const Polyline & crossing, const std::vector<TaggedLight> & lights, const bool far,
  const double near_max, const double far_min)
{
  if (lights.empty()) return false;
  for (const auto & l : lights) {
    const auto f = lightFraction(crossing, l);
    if (!f) return false;
    if (far ? *f < far_min : *f > near_max) return false;
  }
  return true;
}

inline std::vector<int64_t> idsOf(const std::vector<TaggedLight> & lights)
{
  std::vector<int64_t> ids;
  for (const auto & l : lights) ids.push_back(l.id);
  return ids;
}

// Pick the light(s) for this crossing direction.  `tagged` = this direction's tag, `other` =
// the other direction's.  A swap needs both clear-cut (this tag's lights all on ego's side,
// the other's all across); anything ambiguous follows the tags.  Never returns an empty
// selection while any light is tagged, so a missing tag cannot turn a signalized crossing into
// an unsignalized one.
inline LightSelection selectLights(
  const Polyline & crossing, const std::vector<TaggedLight> & tagged,
  const std::vector<TaggedLight> & other, const double near_max = 0.25,
  const double far_min = 0.75)
{
  if (tagged.empty() && other.empty()) return {{}, LightChoice::NONE};
  if (tagged.empty()) return {idsOf(other), LightChoice::FROM_OTHER_TAG};
  if (
    !other.empty() && allClearly(crossing, tagged, false, near_max, far_min) &&
    allClearly(crossing, other, true, near_max, far_min)) {
    return {idsOf(other), LightChoice::SWAPPED_TAGS};
  }
  return {idsOf(tagged), LightChoice::FROM_TAG};
}

}  // namespace autoware::road_crossing_fsm::direction

#endif  // AUTOWARE__ROAD_CROSSING_FSM__DIRECTION_UTILS_HPP_
