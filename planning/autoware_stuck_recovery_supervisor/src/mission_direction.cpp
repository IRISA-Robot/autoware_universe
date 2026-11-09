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

#include "autoware/stuck_recovery_supervisor/mission_direction.hpp"

#include <string>

namespace autoware::stuck_recovery_supervisor
{

// [TURN-IN-PLACE 2026-11-05]
std::optional<MissionDirection> parse_mission_direction(const std::string & text)
{
  if (text == "auto") {
    return MissionDirection::AUTO;
  }
  if (text == "forward_only") {
    return MissionDirection::FORWARD_ONLY;
  }
  return std::nullopt;
}

const char * mission_direction_name(MissionDirection direction)
{
  switch (direction) {
    case MissionDirection::AUTO:
      return "auto";
    case MissionDirection::FORWARD_ONLY:
      return "forward_only";
  }
  return "auto";
}

bool mission_is_reverse(MissionDirection direction, bool path_yaw_opposes_ego)
{
  if (direction == MissionDirection::FORWARD_ONLY) {
    return false;  // v2: inverted lanelets are driven nose-first, after a turn in place
  }
  return path_yaw_opposes_ego;
}

double mission_travel_sign(MissionDirection direction, bool path_yaw_opposes_ego)
{
  return mission_is_reverse(direction, path_yaw_opposes_ego) ? -1.0 : 1.0;
}

}  // namespace autoware::stuck_recovery_supervisor
