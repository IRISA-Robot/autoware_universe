// Copyright 2026 IRISA
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

#ifndef AUTOWARE__ROAD_CROSSING_FSM__CROSSING_LIGHT_RULE_HPP_
#define AUTOWARE__ROAD_CROSSING_FSM__CROSSING_LIGHT_RULE_HPP_

namespace autoware::road_crossing_fsm::crossing_light
{

// Light status as the interlock reports it (same meaning as LightStatus in the node).
enum class Status { NONE, MERAH, HIJAU };

// After GO_GREEN, while ego is still on the pre: does the light still let ego go on?
//
// 2026-11-17: the GO rule (green stable green_min_on_before_go_sec) is NOT re-applied here.
// A green that drops out for a few ticks (detector NONE: camera switch, box lost when ego
// starts moving) reset the stable counter, so allows_crossing stayed false for the whole
// stable time after the green came back and ego was sent back to WAIT_GREEN (log 2026-11-16
// 14:48:33 / 14:52:46).  Here:
//   MERAH                              -> revoke at once
//   HIJAU inside the green phase       -> keep going
//   NONE shorter than none_tolerance   -> keep going (inside the green phase)
//   NONE for none_tolerance or longer  -> revoke
//   green phase over (go window)       -> revoke, as before
// allows_crossing (the GO rule itself) always keeps going.
inline bool lightStillAllowsCrossing(
  const bool allows_crossing, const Status status, const bool green_phase_open,
  const double none_for_sec, const double none_tolerance_sec)
{
  if (allows_crossing) return true;
  if (status == Status::MERAH || !green_phase_open) return false;
  if (status == Status::HIJAU) return true;
  return none_for_sec < none_tolerance_sec;  // NONE
}

// A green may open a GO phase only if it shows up right after the red: at most max_gap_ticks
// FSM ticks after the last tick with status MERAH (status MERAH already outlives the red by
// threshold_merah_off).  2026-11-17: red seen, the view covered through the solid green and
// opened again in the blinking green -> that green is not a phase start: wait for the next
// red.  max_gap_ticks < 0 = rule off.
inline bool greenFollowsRed(const int ticks_since_red, const int max_gap_ticks)
{
  return max_gap_ticks < 0 || ticks_since_red <= max_gap_ticks;
}

}  // namespace autoware::road_crossing_fsm::crossing_light

#endif  // AUTOWARE__ROAD_CROSSING_FSM__CROSSING_LIGHT_RULE_HPP_
