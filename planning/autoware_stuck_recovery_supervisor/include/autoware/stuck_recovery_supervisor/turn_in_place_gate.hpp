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

#ifndef AUTOWARE__STUCK_RECOVERY_SUPERVISOR__TURN_IN_PLACE_GATE_HPP_
#define AUTOWARE__STUCK_RECOVERY_SUPERVISOR__TURN_IN_PLACE_GATE_HPP_

#include <cstdint>
#include <string>

namespace autoware::stuck_recovery_supervisor
{

// [TURN-IN-PLACE 2026-11-05] The turn-in-place gate.
//
// The turn-in-place supervisor holds the robot with a zero velocity limit while its body
// faces against the route, waits for a free side, spins it, and cools down.  To this node
// every second of that looks like "ego stopped without a legitimate reason", which is the
// one thing it exists to act on.  Two controllers working the same robot is worse than
// either alone (the road-crossing FSM taught the same lesson), so while the turn supervisor
// is in one of its holding states the stop is legitimate here.
//
// MONITOR (nothing to do) and ERROR (a phase timed out, the turn failed) do not gate: after
// a failed turn the robot may genuinely be stuck, and normal handling must be able to see it.

/// Why the gate is open or closed this tick.  Logged on change only.
enum class TurnInPlaceGateStatus {
  DISABLED,     ///< respect_turn_in_place is false
  NEVER_HEARD,  ///< no state message ever received (no turn-in-place node on this robot)
  STALE,        ///< last message older than the timeout; the publisher is gone
  NOT_HOLDING,  ///< MONITOR or ERROR (or an unknown value): normal stuck handling
  HOLDING,      ///< REQUEST_STOP, CHECK_CLEARANCE, TURNING, COOLDOWN or REVERSING_TO_TURN_SPOT: legit
};

struct TurnInPlaceGateInput
{
  bool respect{true};
  bool have_message{false};
  uint8_t state{0};
  /// Age of the last message, by the time it was RECEIVED on this node's clock.
  double age_sec{0.0};
  double timeout_sec{2.0};
};

/// True for the turn-in-place states that hold the robot on purpose.
bool turn_in_place_state_holds(uint8_t state);

/// Upper-case name of a TurnInPlaceSupervisorState.state value, for logs and legit_reason.
std::string turn_in_place_state_name(uint8_t state);

TurnInPlaceGateStatus evaluate_turn_in_place_gate(const TurnInPlaceGateInput & input);

inline bool gate_holds(TurnInPlaceGateStatus status)
{
  return status == TurnInPlaceGateStatus::HOLDING;
}

const char * turn_in_place_gate_status_name(TurnInPlaceGateStatus status);

}  // namespace autoware::stuck_recovery_supervisor

#endif  // AUTOWARE__STUCK_RECOVERY_SUPERVISOR__TURN_IN_PLACE_GATE_HPP_
