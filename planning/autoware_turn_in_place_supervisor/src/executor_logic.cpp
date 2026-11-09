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

#include "autoware/turn_in_place_supervisor/executor_logic.hpp"

#include "autoware/turn_in_place_supervisor/geometry.hpp"

#include <algorithm>
#include <cmath>

namespace autoware::turn_in_place_supervisor
{

void TurnInPlaceExecutor::enter(const ExecutorState next, const double now)
{
  if (next != state_) {
    state_ = next;
    state_since_ = now;
    last_requested_mode_.reset();  // a new phase asks for its mode afresh
  }
}

bool TurnInPlaceExecutor::mode_in_force(const uint8_t mode, const ExecutorInput & in) const
{
  return in.feedback_fresh && in.feedback_mode == mode && !in.feedback_switching;
}

std::optional<uint8_t> TurnInPlaceExecutor::request_mode(
  const uint8_t mode, const ExecutorInput & in)
{
  // Ask once per phase, and again only if the chassis has not taken the request up after
  // request_retry_sec: re-sending while it is switching could restart its transition.
  const bool taken_up = in.feedback_fresh && in.feedback_target_mode == mode;
  const bool due = !last_requested_mode_ || *last_requested_mode_ != mode ||
                   (!taken_up && in.now - last_request_time_ >= params_.request_retry_sec);
  if (!due) {
    return std::nullopt;
  }
  last_requested_mode_ = mode;
  last_request_time_ = in.now;
  return mode;
}

double TurnInPlaceExecutor::compute_yaw_rate(const double error, const int direction) const
{
  // Always turn the commanded way -- the supervisor chose it because that side is clear -- even
  // when the other way is shorter.  The one exception is a small overshoot just past the target:
  // that is corrected back the short way instead of by going all the way round.
  constexpr double kOvershootWindow = 15.0 * M_PI / 180.0;
  int sign = direction >= 0 ? 1 : -1;
  double remaining = rotation_in_direction(0.0, error, sign);
  if (remaining > 2.0 * M_PI - kOvershootWindow) {
    sign = -sign;
    remaining = 2.0 * M_PI - remaining;
  }
  const double magnitude =
    std::clamp(params_.kp * remaining, params_.min_yaw_rate, params_.max_yaw_rate);
  return sign * magnitude;
}

ExecutorOutput TurnInPlaceExecutor::step(const ExecutorInput & in)
{
  ExecutorOutput out;
  const double error = normalize_angle(in.target_yaw - in.yaw);
  out.yaw_error = error;
  const double in_state = in.now - state_since_;

  switch (state_) {
    case ExecutorState::IDLE: {
      if (in.command_active) {
        // Remember the mode to come back to; the parameter covers a chassis with no feedback yet.
        restore_mode_ = (in.feedback_fresh && in.feedback_mode != params_.spin_mode)
                          ? in.feedback_mode
                          : params_.driving_mode;
        target_reached_ = false;
        enter(ExecutorState::SWITCHING_MODE, in.now);
        out.mode_request = request_mode(params_.spin_mode, in);
        out.publish_twist = true;
        out.reason = "switching to the spin mode";
        break;
      }
      out.reason = "idle";
      break;
    }

    case ExecutorState::SWITCHING_MODE: {
      out.publish_twist = true;
      if (!in.command_active) {
        enter(ExecutorState::RESTORING_MODE, in.now);
        out.mode_request = request_mode(restore_mode_, in);
        out.reason = "cancelled while switching; restoring the driving mode";
        break;
      }
      if (mode_in_force(params_.spin_mode, in)) {
        enter(ExecutorState::ROTATING, in.now);
        out.yaw_rate = compute_yaw_rate(error, in.direction);
        out.reason = "spin mode active; rotating";
        break;
      }
      if (in_state > params_.switch_timeout_sec) {
        enter(ExecutorState::FAILED, in.now);
        out.mode_request = request_mode(restore_mode_, in);
        out.reason = "timed out switching to the spin mode";
        break;
      }
      out.mode_request = request_mode(params_.spin_mode, in);
      out.reason = "waiting for the spin mode";
      break;
    }

    case ExecutorState::ROTATING: {
      out.publish_twist = true;
      if (!in.command_active) {
        enter(ExecutorState::RESTORING_MODE, in.now);
        out.mode_request = request_mode(restore_mode_, in);
        out.reason = "cancelled while rotating; restoring the driving mode";
        break;
      }
      if (std::abs(error) <= params_.done_tolerance) {
        target_reached_ = true;
        enter(ExecutorState::RESTORING_MODE, in.now);
        out.mode_request = request_mode(restore_mode_, in);
        out.reason = "target heading reached; restoring the driving mode";
        break;
      }
      if (in.feedback_fresh && in.feedback_mode != params_.spin_mode) {
        enter(ExecutorState::FAILED, in.now);
        out.mode_request = request_mode(restore_mode_, in);
        out.reason = "chassis left the spin mode while rotating";
        break;
      }
      if (in_state > params_.rotate_timeout_sec) {
        enter(ExecutorState::FAILED, in.now);
        out.mode_request = request_mode(restore_mode_, in);
        out.reason = "rotation timed out";
        break;
      }
      out.yaw_rate = compute_yaw_rate(error, in.direction);
      out.reason = "rotating";
      break;
    }

    case ExecutorState::RESTORING_MODE: {
      out.publish_twist = true;
      if (mode_in_force(restore_mode_, in)) {
        if (target_reached_ && in.command_active) {
          enter(ExecutorState::DONE, in.now);
          out.reason = "turn done";
        } else if (in.command_active) {
          // Cancelled and re-requested meanwhile: start over.
          target_reached_ = false;
          enter(ExecutorState::SWITCHING_MODE, in.now);
          out.mode_request = request_mode(params_.spin_mode, in);
          out.reason = "new turn requested; switching to the spin mode";
        } else {
          enter(ExecutorState::IDLE, in.now);
          out.publish_twist = false;
          out.reason = "driving mode restored";
        }
        break;
      }
      if (in_state > params_.switch_timeout_sec) {
        enter(ExecutorState::FAILED, in.now);
        out.mode_request = request_mode(restore_mode_, in);
        out.reason = "timed out restoring the driving mode";
        break;
      }
      out.mode_request = request_mode(restore_mode_, in);
      out.reason = "waiting for the driving mode";
      break;
    }

    case ExecutorState::DONE: {
      if (!in.command_active) {
        enter(ExecutorState::IDLE, in.now);
        out.reason = "idle";
        break;
      }
      out.reason = "turn done; waiting for the supervisor to release";
      break;
    }

    case ExecutorState::FAILED: {
      // Keep the chassis still and keep asking for the driving mode back.
      out.publish_twist = true;
      out.mode_request = request_mode(restore_mode_, in);
      if (!in.command_active && mode_in_force(restore_mode_, in)) {
        enter(ExecutorState::IDLE, in.now);
        out.publish_twist = false;
        out.reason = "released after failure; driving mode restored";
        break;
      }
      out.reason = "failed";
      break;
    }
  }

  out.state = state_;
  if (state_ != ExecutorState::ROTATING) {
    out.yaw_rate = 0.0;
  }
  return out;
}

}  // namespace autoware::turn_in_place_supervisor
