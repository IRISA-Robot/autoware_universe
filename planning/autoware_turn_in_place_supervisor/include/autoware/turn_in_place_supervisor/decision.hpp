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

#ifndef AUTOWARE__TURN_IN_PLACE_SUPERVISOR__DECISION_HPP_
#define AUTOWARE__TURN_IN_PLACE_SUPERVISOR__DECISION_HPP_

#include <cstdint>
#include <optional>
#include <string>

namespace autoware::turn_in_place_supervisor
{

// Values match autoware_turn_in_place_msgs/TurnInPlaceSupervisorState.
enum class SupervisorState : uint8_t {
  MONITOR = 0,
  REQUEST_STOP = 1,
  CHECK_CLEARANCE = 2,
  TURNING = 3,
  COOLDOWN = 4,
  ERROR = 5,
  REVERSING_TO_TURN_SPOT = 6,
};

// Values match autoware_turn_in_place_msgs/TurnInPlaceStatus.
enum class ExecutorState : uint8_t {
  IDLE = 0,
  SWITCHING_MODE = 1,
  ROTATING = 2,
  RESTORING_MODE = 3,
  DONE = 4,
  FAILED = 5,
};

const char * to_string(SupervisorState state);

struct DecisionParams
{
  double trigger_angle{80.0 * 3.14159265358979323846 / 180.0};  // [rad] |yaw error| to act on
  double done_angle{10.0 * 3.14159265358979323846 / 180.0};     // [rad] |yaw error| that is fine
  double trigger_confirm_sec{0.3};  // |yaw error| must stay above trigger_angle this long
  double stop_speed{0.05};         // [m/s] robot counts as standing still below this
  double cooldown_sec{3.0};        // hold after a turn before handing back
  double stop_timeout_sec{15.0};   // REQUEST_STOP -> ERROR
  double clearance_timeout_sec{60.0};  // CHECK_CLEARANCE -> ERROR
  double turn_timeout_sec{30.0};   // TURNING -> ERROR
  // When the road is too narrow to turn here, back up along the route (towards the goal) to the
  // first spot wide enough, or all the way to the goal when there is none.
  bool enable_reverse_to_turn_spot{true};
};

// One decision cycle's view of the world.
struct DecisionInput
{
  double now{0.0};             // [s]
  bool autonomous{false};      // operation mode AUTONOMOUS and autoware control enabled
  bool route_changed{false};   // a new route arrived since the previous cycle
  bool route_valid{false};     // route heading at ego is known (active route window fresh)
  double yaw_error{0.0};       // [rad] route heading - body heading, normalized
  double route_yaw{0.0};       // [rad] route heading at ego (map frame)
  double speed{0.0};           // [m/s] |longitudinal velocity|
  bool ccw_clear{false};       // turning counter-clockwise to route_yaw sweeps no obstacle
  bool cw_clear{false};        // turning clockwise to route_yaw sweeps no obstacle
  // Both directions leave the road area (lanelets): no waiting will make room here.
  bool road_blocks_both{false};
  bool turn_spot_found{false};  // a spot wide enough to turn lies ahead along the route
  bool at_turn_spot{false};     // reversing: ego reached that spot and stands still
  bool executor_fresh{false};  // an executor status arrived recently
  ExecutorState executor_state{ExecutorState::IDLE};
};

struct DecisionOutput
{
  SupervisorState state{SupervisorState::MONITOR};
  bool hold_vehicle{false};    // publish a zero velocity limit
  bool command_active{false};  // ask the executor to turn
  double target_yaw{0.0};
  int direction{0};            // kDirectionCcw / kDirectionCw, 0 when not turning
  bool reverse_active{false};  // ask reverse_lane_follow to back up along the route
  bool reverse_to_spot{false};  // ... and stop at the turn spot (else go on to the goal)
  std::string reason;
};

/// The turn-in-place decision, one call per decision cycle (1 Hz by default).  Pure logic: no
/// ROS, no clock of its own, so it is driven and tested with explicit inputs.
class TurnInPlaceDecision
{
public:
  explicit TurnInPlaceDecision(const DecisionParams & params) : params_(params) {}

  DecisionOutput step(const DecisionInput & in);

  SupervisorState state() const { return state_; }
  void set_params(const DecisionParams & params) { params_ = params; }

private:
  void enter(SupervisorState next, double now);
  double elapsed(double now) const { return now - state_since_; }
  DecisionOutput output(bool hold, const std::string & reason) const;

  DecisionParams params_;
  SupervisorState state_{SupervisorState::MONITOR};
  double state_since_{0.0};
  std::optional<double> over_trigger_since_;
  bool reverse_to_spot_{false};
  double target_yaw_{0.0};
  int direction_{0};
};

}  // namespace autoware::turn_in_place_supervisor

#endif  // AUTOWARE__TURN_IN_PLACE_SUPERVISOR__DECISION_HPP_
