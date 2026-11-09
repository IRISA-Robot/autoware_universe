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

#ifndef AUTOWARE__TURN_IN_PLACE_SUPERVISOR__EXECUTOR_LOGIC_HPP_
#define AUTOWARE__TURN_IN_PLACE_SUPERVISOR__EXECUTOR_LOGIC_HPP_

#include "autoware/turn_in_place_supervisor/decision.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace autoware::turn_in_place_supervisor
{

struct ExecutorParams
{
  uint8_t spin_mode{4};       // middleware STEER_MODE_HOLONOMIC
  uint8_t driving_mode{2};    // mode to restore when the pre-turn mode is unknown
  double kp{1.5};             // [1/s] yaw rate per rad of remaining error
  double max_yaw_rate{0.6};   // [rad/s]
  double min_yaw_rate{0.1};   // [rad/s] floor so the last degrees still close
  double done_tolerance{3.0 * 3.14159265358979323846 / 180.0};  // [rad]
  double switch_timeout_sec{5.0};
  double rotate_timeout_sec{30.0};
  double request_retry_sec{1.0};  // re-send a mode request the chassis has not taken up
};

struct ExecutorInput
{
  double now{0.0};
  bool command_active{false};
  double target_yaw{0.0};
  int direction{0};          // kDirectionCcw / kDirectionCw
  double yaw{0.0};           // current body heading
  bool feedback_fresh{false};
  uint8_t feedback_mode{0};
  uint8_t feedback_target_mode{0};
  bool feedback_switching{false};
};

struct ExecutorOutput
{
  ExecutorState state{ExecutorState::IDLE};
  std::optional<uint8_t> mode_request;  // publish /gama/cmd/steering_mode when set
  bool publish_twist{false};            // the executor owns the spin command this tick
  double yaw_rate{0.0};                 // [rad/s] CCW positive
  double yaw_error{0.0};                // target - yaw, normalized
  std::string reason;
};

/// Spin-in-place executor: switch the chassis to its spin mode, rotate to the target heading,
/// switch back.  Pure logic; the node feeds it odometry, the command and the chassis'
/// steering-mode feedback, and publishes what it returns.
class TurnInPlaceExecutor
{
public:
  explicit TurnInPlaceExecutor(const ExecutorParams & params) : params_(params) {}

  ExecutorOutput step(const ExecutorInput & in);

  ExecutorState state() const { return state_; }
  void set_params(const ExecutorParams & params) { params_ = params; }

private:
  void enter(ExecutorState next, double now);
  std::optional<uint8_t> request_mode(uint8_t mode, const ExecutorInput & in);
  bool mode_in_force(uint8_t mode, const ExecutorInput & in) const;
  double compute_yaw_rate(double error, int direction) const;

  ExecutorParams params_;
  ExecutorState state_{ExecutorState::IDLE};
  double state_since_{0.0};
  double last_request_time_{-1.0e9};
  std::optional<uint8_t> last_requested_mode_;
  uint8_t restore_mode_{2};
  bool target_reached_{false};
};

}  // namespace autoware::turn_in_place_supervisor

#endif  // AUTOWARE__TURN_IN_PLACE_SUPERVISOR__EXECUTOR_LOGIC_HPP_
