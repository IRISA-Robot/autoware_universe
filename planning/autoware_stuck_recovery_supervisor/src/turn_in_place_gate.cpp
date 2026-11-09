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

#include "autoware/stuck_recovery_supervisor/turn_in_place_gate.hpp"

#include <autoware_turn_in_place_msgs/msg/turn_in_place_supervisor_state.hpp>

#include <string>

namespace autoware::stuck_recovery_supervisor
{

// [TURN-IN-PLACE 2026-11-05]
using TurnInPlaceState = autoware_turn_in_place_msgs::msg::TurnInPlaceSupervisorState;

bool turn_in_place_state_holds(uint8_t state)
{
  switch (state) {
    case TurnInPlaceState::REQUEST_STOP:
    case TurnInPlaceState::CHECK_CLEARANCE:
    case TurnInPlaceState::TURNING:
    case TurnInPlaceState::COOLDOWN:
    // Backing up to a turn spot: a manoeuvre of the turn supervisor's own; a forward_only
    // mission direction would read it as ground given against the mission.
    case TurnInPlaceState::REVERSING_TO_TURN_SPOT:
      return true;
    default:
      return false;  // MONITOR, ERROR, and anything this build does not know about
  }
}

std::string turn_in_place_state_name(uint8_t state)
{
  switch (state) {
    case TurnInPlaceState::MONITOR:
      return "MONITOR";
    case TurnInPlaceState::REQUEST_STOP:
      return "REQUEST_STOP";
    case TurnInPlaceState::CHECK_CLEARANCE:
      return "CHECK_CLEARANCE";
    case TurnInPlaceState::TURNING:
      return "TURNING";
    case TurnInPlaceState::COOLDOWN:
      return "COOLDOWN";
    case TurnInPlaceState::ERROR:
      return "ERROR";
    case TurnInPlaceState::REVERSING_TO_TURN_SPOT:
      return "REVERSING_TO_TURN_SPOT";
    default:
      return "UNKNOWN(" + std::to_string(static_cast<int>(state)) + ")";
  }
}

TurnInPlaceGateStatus evaluate_turn_in_place_gate(const TurnInPlaceGateInput & input)
{
  if (!input.respect) {
    return TurnInPlaceGateStatus::DISABLED;
  }
  // A robot without the turn-in-place node never publishes; the gate must then be inert,
  // not "stale", so the two are kept apart.
  if (!input.have_message) {
    return TurnInPlaceGateStatus::NEVER_HEARD;
  }
  // A holding state from a node that has since died must not hold this one forever.
  if (input.age_sec > input.timeout_sec) {
    return TurnInPlaceGateStatus::STALE;
  }
  return turn_in_place_state_holds(input.state) ? TurnInPlaceGateStatus::HOLDING
                                                 : TurnInPlaceGateStatus::NOT_HOLDING;
}

const char * turn_in_place_gate_status_name(TurnInPlaceGateStatus status)
{
  switch (status) {
    case TurnInPlaceGateStatus::DISABLED:
      return "disabled";
    case TurnInPlaceGateStatus::NEVER_HEARD:
      return "never_heard";
    case TurnInPlaceGateStatus::STALE:
      return "stale";
    case TurnInPlaceGateStatus::NOT_HOLDING:
      return "not_holding";
    case TurnInPlaceGateStatus::HOLDING:
      return "holding";
  }
  return "unknown";
}

}  // namespace autoware::stuck_recovery_supervisor
