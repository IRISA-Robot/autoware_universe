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

#ifndef AUTOWARE__STUCK_RECOVERY_SUPERVISOR__MISSION_DIRECTION_HPP_
#define AUTOWARE__STUCK_RECOVERY_SUPERVISOR__MISSION_DIRECTION_HPP_

#include <optional>
#include <string>

namespace autoware::stuck_recovery_supervisor
{

// [TURN-IN-PLACE 2026-11-05] Which way the MISSION drives the robot.
//
// The supervisor has always inferred this from the geometry: if the path heading near ego
// opposes ego's own heading, the mission must be running in reverse motion mode (the path
// poses face the direction of travel, the robot faces the other way).  That inference is
// only valid on a robot that drives inverted lanelets backwards.
//
// Robot v2 never does.  It drives every lanelet nose-first and, when its body ends up facing
// against the route, the turn-in-place supervisor spins it round on the spot.  Before and
// during that spin the geometry reads exactly like a reverse mission, and every decision
// built on it -- the direction plans must travel in, the heading escape goals are given, the
// heading TRANSIT lines up to, the counter-mission ledger -- would be built for a mission
// that is not happening.  FORWARD_ONLY pins the answer to "nose-first".
//
// This is ONLY the motion decision.  Geometric mirroring -- "the path's left is the robot's
// right because the robot happens to face the other way" -- stays on the raw geometry in
// both modes, because that is a statement about where things are, not about where the
// mission goes.
enum class MissionDirection {
  AUTO,          ///< infer from the geometry (the original behaviour)
  FORWARD_ONLY,  ///< the mission is always driven nose-first
};

/// "auto" / "forward_only"; anything else is std::nullopt so the caller can warn and fall
/// back instead of silently guessing.
std::optional<MissionDirection> parse_mission_direction(const std::string & text);

const char * mission_direction_name(MissionDirection direction);

/// Is the mission driven in reverse motion mode?  `path_yaw_opposes_ego` is the raw
/// geometric answer (cos(path_yaw - ego_yaw) < 0); FORWARD_ONLY overrides it.
bool mission_is_reverse(MissionDirection direction, bool path_yaw_opposes_ego);

/// +1 nose-first, -1 backwards along the route.
double mission_travel_sign(MissionDirection direction, bool path_yaw_opposes_ego);

}  // namespace autoware::stuck_recovery_supervisor

#endif  // AUTOWARE__STUCK_RECOVERY_SUPERVISOR__MISSION_DIRECTION_HPP_
