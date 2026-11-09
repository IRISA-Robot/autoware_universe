// Copyright 2026 The Autoware Foundation.
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

#ifndef AUTOWARE__SIMPLE_PLANNING_SIMULATOR__UTILS__STEERING_MODE_MOCK_HPP_
#define AUTOWARE__SIMPLE_PLANNING_SIMULATOR__UTILS__STEERING_MODE_MOCK_HPP_

#include <cstdint>
#include <optional>

namespace autoware::simulator::simple_planning_simulator
{

// [STEERING-MODE-MOCK 2026-11-05] The gama v2 chassis (middleware_controller_manager) switches
// steering mode on /gama/cmd/steering_mode: 0=FWS 1=RWS 2=4WCS 3=CRAB 4=HOLONOMIC.  While the
// wheels re-orient the chassis does not move; in HOLONOMIC the wheels are locked and a Twist
// angular.z spins it in place.  The simulator has no notion of steering mode, so this small
// state machine mocks that behaviour.  It is pure C++ (time is passed in as seconds) so it can be
// unit tested without ROS timing; SimplePlanningSimulator owns one and feeds it from on_timer().

struct SteeringModeMockConfig
{
  uint8_t driving_mode = 2;          //!< initial mode; any mode != HOLONOMIC drives normally
  double mode_switch_time_s = 1.5;   //!< duration of a transition (vehicle held still)
  double max_yaw_rate = 1.0;         //!< [rad/s] clamp on the commanded HOLONOMIC yaw rate
  double max_yaw_acc = 2.0;          //!< [rad/s^2] slew limit on the HOLONOMIC yaw rate
  bool fail_switch = false;          //!< fault injection: a transition never completes
  double twist_timeout_s = 0.5;      //!< a yaw-rate command older than this counts as zero
};

struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

class SteeringModeMock
{
public:
  static constexpr uint8_t HOLONOMIC = 4;

  /// What the simulator has to do with the vehicle model after a step().
  enum class Action {
    RUN_MODEL,           //!< normal driving: run the Ackermann vehicle model as usual
    HOLD,                //!< transition or HOLONOMIC: freeze the model at held_pose()
    RESUME_FROM_HOLD,    //!< transition into a driving mode just finished: reset the model to
                         //!< held_pose() with zero velocity, then run it normally
  };

  explicit SteeringModeMock(const SteeringModeMockConfig & config);

  /// Request a mode.  A request equal to the current target is a no-op; any other request
  /// (also one received mid-transition) (re)starts a transition to it at `now_s`.
  /// `current_pose` is captured as the held pose when the vehicle was not already held.
  void request_mode(uint8_t mode, double now_s, const Pose2D & current_pose);

  /// Latest HOLONOMIC yaw-rate command [rad/s] received at `now_s`.
  void set_yaw_rate_command(double yaw_rate, double now_s);

  /// Advance the state machine to `now_s`.  `dt` is the integration step for the HOLONOMIC yaw
  /// (pass 0 to pause the motion while still letting a transition time out).
  Action step(double now_s, double dt);

  /// Teleport while held (e.g. a new initial pose arrives during HOLONOMIC).
  void set_held_pose(const Pose2D & pose) { held_pose_ = pose; }

  void set_fail_switch(bool fail_switch) { config_.fail_switch = fail_switch; }

  uint8_t mode() const { return mode_; }
  uint8_t target_mode() const { return target_mode_; }
  bool switching() const { return switching_; }
  /// True while the vehicle model is frozen (transition or HOLONOMIC).
  bool holding() const { return switching_ || mode_ == HOLONOMIC; }
  const Pose2D & held_pose() const { return held_pose_; }
  /// Current yaw rate [rad/s]; non-zero only in HOLONOMIC.
  double yaw_rate() const { return yaw_rate_; }

private:
  SteeringModeMockConfig config_;
  uint8_t mode_;
  uint8_t target_mode_;
  bool switching_ = false;
  double switch_start_s_ = 0.0;
  Pose2D held_pose_{};
  double yaw_rate_ = 0.0;
  double yaw_rate_cmd_ = 0.0;
  std::optional<double> yaw_rate_cmd_stamp_s_{};
};

}  // namespace autoware::simulator::simple_planning_simulator

#endif  // AUTOWARE__SIMPLE_PLANNING_SIMULATOR__UTILS__STEERING_MODE_MOCK_HPP_
