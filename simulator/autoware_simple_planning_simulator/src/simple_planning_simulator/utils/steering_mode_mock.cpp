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

#include "autoware/simple_planning_simulator/utils/steering_mode_mock.hpp"

#include <algorithm>
#include <cmath>

namespace autoware::simulator::simple_planning_simulator
{

// [STEERING-MODE-MOCK 2026-11-05] see the header for why this exists.

SteeringModeMock::SteeringModeMock(const SteeringModeMockConfig & config)
: config_(config), mode_(config.driving_mode), target_mode_(config.driving_mode)
{
}

void SteeringModeMock::request_mode(
  const uint8_t mode, const double now_s, const Pose2D & current_pose)
{
  // The mode already in force (or already being switched to) is a no-op.
  if (mode == target_mode_) {
    return;
  }
  // Capture the pose only when the model is still live; when already held (HOLONOMIC or a
  // transition) the held pose -- including the integrated spin yaw -- is the truth.
  if (!holding()) {
    held_pose_ = current_pose;
  }
  target_mode_ = mode;
  switching_ = true;
  switch_start_s_ = now_s;
  yaw_rate_ = 0.0;
}

void SteeringModeMock::set_yaw_rate_command(const double yaw_rate, const double now_s)
{
  yaw_rate_cmd_ = yaw_rate;
  yaw_rate_cmd_stamp_s_ = now_s;
}

SteeringModeMock::Action SteeringModeMock::step(const double now_s, const double dt)
{
  if (switching_) {
    // Wheels re-orienting: the chassis does not move.
    yaw_rate_ = 0.0;
    if (config_.fail_switch || now_s - switch_start_s_ < config_.mode_switch_time_s) {
      return Action::HOLD;
    }
    switching_ = false;
    mode_ = target_mode_;
    return mode_ == HOLONOMIC ? Action::HOLD : Action::RESUME_FROM_HOLD;
  }

  if (mode_ != HOLONOMIC) {
    return Action::RUN_MODEL;
  }

  // HOLONOMIC: spin in place with a rate- and acceleration-limited yaw rate.
  const bool fresh = yaw_rate_cmd_stamp_s_.has_value() &&
                     now_s - *yaw_rate_cmd_stamp_s_ <= config_.twist_timeout_s;
  const double max_rate = std::abs(config_.max_yaw_rate);
  const double cmd = fresh ? std::clamp(yaw_rate_cmd_, -max_rate, max_rate) : 0.0;
  if (dt <= 0.0) {
    yaw_rate_ = 0.0;  // motion paused (not engaged): report standing still
    return Action::HOLD;
  }
  const double max_delta = std::abs(config_.max_yaw_acc) * dt;
  yaw_rate_ += std::clamp(cmd - yaw_rate_, -max_delta, max_delta);
  held_pose_.yaw = std::remainder(held_pose_.yaw + yaw_rate_ * dt, 2.0 * M_PI);
  return Action::HOLD;
}

}  // namespace autoware::simulator::simple_planning_simulator
