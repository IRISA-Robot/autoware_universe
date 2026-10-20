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

#ifndef EXPERIMENTAL__STOP_ZONE_HPP_
#define EXPERIMENTAL__STOP_ZONE_HPP_

#include <algorithm>
#include <optional>

namespace autoware::behavior_velocity_planner::experimental::road_crossing
{

// Where to stop for a crossing that must not be entered yet (red light / gate HOLD).
//
// A stop LINE asks the robot to end up at one exact spot, which the longitudinal controller
// cannot do from rest when the line is 0.5..1.5 m ahead: it leaves STOPPED only beyond
// drive_state_stop_dist, re-enters STOPPING below stopping_state_stop_dist, and the robot
// creeps there in hops.  A stop ZONE asks only "has the robot reached it?", like an obstacle.
//
// The zone has to lie on the pre lanelet (the route lanelet right before the crossing): the
// road_crossing FSM arms, and keeps its episode, only while the robot is on it -- base_link or
// front bumper on the pre, the same test here and in the FSM.
//
//   line    = entry - (leading edge + stop_margin)        furthest the base_link may go
//   barrier = entry - (leading edge + zone_length), moved forward to at least the front bumper
//             pre_margin into the pre, never past the line
//
//   - outside the zone                          -> stop at the barrier (BARRIER); front not on
//                                                  the pre yet: at least min_departure_dist
//                                                  ahead (not past the line)
//
// is_stopped means "slower than the module's stop_state_speed": a robot the controller left
// crawling at a few cm/s is standing still for the zone.
//   - on the pre and past the barrier, or standing still on the pre with the barrier at most
//     hold_tolerance ahead                      -> stop where the robot is (HOLD_IN_ZONE);
//     a moving robot gets its braking distance, never past the line.
//
// All positions are arc lengths [m] on the path; entry_s is where it enters the crossing,
// pre_entry_s where it enters the pre lanelet (no value: no pre found, the zone is not limited).

enum class StopZoneMode { BARRIER, HOLD_IN_ZONE };

struct StopZoneParam
{
  double leading_edge_offset{0.9};  // base_link -> leading bumper [m]
  double zone_length{2.0};          // leading bumper to crossing entry at the barrier [m]
  double stop_margin{0.5};          // leading bumper to crossing entry at the line [m]
  double hold_tolerance{1.5};       // stopped this close before the barrier counts as in [m]
  double pre_margin{0.5};           // barrier: front bumper at least this far into the pre [m]
  // A robot standing still OFF the pre is sent at least this far, so the longitudinal controller
  // can set off at all (it does not leave STOPPED for a stop point closer than its
  // drive_state_stop_dist, and creeps in hops below drive_state_stop_dist + offset).
  double min_departure_dist{1.5};   // [m]
  double decel{1.0};                // braking deceleration of a moving robot in the zone [m/s^2]
};

struct StopZoneResult
{
  double stop_s;
  StopZoneMode mode;
};

inline StopZoneResult computeStopZone(
  const double ego_s, const double entry_s, const std::optional<double> pre_entry_s,
  const double ego_speed, const bool is_stopped, const StopZoneParam & p)
{
  const double zone_length = std::max(p.zone_length, p.stop_margin);
  const double line_s = entry_s - (p.leading_edge_offset + p.stop_margin);
  double barrier_s = entry_s - (p.leading_edge_offset + zone_length);
  if (pre_entry_s) {
    barrier_s = std::max(
      barrier_s, *pre_entry_s - p.leading_edge_offset + std::max(0.0, p.pre_margin));
  }
  barrier_s = std::min(barrier_s, line_s);

  // On the pre once the front bumper is (base_link may still be on the lanelet before it).
  const bool on_pre = !pre_entry_s || ego_s + p.leading_edge_offset >= *pre_entry_s;
  const bool in_zone =
    on_pre &&
    (ego_s >= barrier_s ||
     (is_stopped && barrier_s - ego_s <= std::max(0.0, p.hold_tolerance)));
  if (!in_zone) {
    if (!on_pre) {
      // Not at the pre yet: keep the stop point at least min_departure_dist ahead (never past
      // the line), so a robot short of the pre drives onto it in one go instead of being handed
      // a stop point it can only creep to.  Once its front is on the pre the zone takes over.
      return {std::min(line_s, std::max(barrier_s, ego_s + p.min_departure_dist)),
              StopZoneMode::BARRIER};
    }
    return {barrier_s, StopZoneMode::BARRIER};
  }

  const double v = std::max(0.0, ego_speed);
  const double brake = is_stopped || p.decel <= 0.0 ? 0.0 : v * v / (2.0 * p.decel);
  // Stop within the braking distance, but not past the line -- unless already past it, in
  // which case where the robot stands is the only place left to stop.
  const double stop_s = std::min(ego_s + brake, std::max(line_s, ego_s));
  return {stop_s, StopZoneMode::HOLD_IN_ZONE};
}

}  // namespace autoware::behavior_velocity_planner::experimental::road_crossing

#endif  // EXPERIMENTAL__STOP_ZONE_HPP_
