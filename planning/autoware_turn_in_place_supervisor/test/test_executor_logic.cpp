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

#include <gtest/gtest.h>

#include <cmath>

using autoware::turn_in_place_supervisor::ExecutorInput;
using autoware::turn_in_place_supervisor::ExecutorParams;
using autoware::turn_in_place_supervisor::ExecutorState;
using autoware::turn_in_place_supervisor::kDirectionCcw;
using autoware::turn_in_place_supervisor::kDirectionCw;
using autoware::turn_in_place_supervisor::normalize_angle;
using autoware::turn_in_place_supervisor::TurnInPlaceExecutor;

namespace
{
constexpr double kDeg = M_PI / 180.0;

// A tiny chassis model: mode switches take switch_time, the spin integrates the commanded rate.
struct FakeChassis
{
  uint8_t mode{2};
  uint8_t target{2};
  double switch_done_at{0.0};
  double yaw{0.0};
  double switch_time{1.0};

  void request(const uint8_t m, const double now)
  {
    if (m == target) {
      return;
    }
    target = m;
    switch_done_at = now + switch_time;
  }
  bool switching(const double now) const { return target != mode && now < switch_done_at; }
  void update(const double now, const double rate, const double dt)
  {
    if (target != mode && now >= switch_done_at) {
      mode = target;
    }
    if (mode == 4 && !switching(now)) {
      yaw = normalize_angle(yaw + rate * dt);
    }
  }
};

struct Sim
{
  TurnInPlaceExecutor executor{ExecutorParams{}};
  FakeChassis chassis;
  double now{0.0};
  ExecutorState last{ExecutorState::IDLE};

  // Steps at 20 Hz for `seconds`, returns the final state.
  ExecutorState go(const double seconds, const bool active, const double target, const int dir)
  {
    const double dt = 0.05;
    for (double t = 0.0; t < seconds; t += dt) {
      ExecutorInput in;
      in.now = now;
      in.command_active = active;
      in.target_yaw = target;
      in.direction = dir;
      in.yaw = chassis.yaw;
      in.feedback_fresh = true;
      in.feedback_mode = chassis.mode;
      in.feedback_target_mode = chassis.target;
      in.feedback_switching = chassis.switching(now);
      const auto out = executor.step(in);
      if (out.mode_request) {
        chassis.request(*out.mode_request, now);
      }
      chassis.update(now, out.yaw_rate, dt);
      now += dt;
      last = out.state;
    }
    return last;
  }
};
}  // namespace

TEST(Executor, IdleWithoutACommand)
{
  Sim r;
  EXPECT_EQ(r.go(1.0, false, 0.0, 0), ExecutorState::IDLE);
  EXPECT_EQ(r.chassis.target, 2);
}

TEST(Executor, FullTurnSwitchesSpinsAndRestores)
{
  Sim r;
  const double target = 170.0 * kDeg;
  EXPECT_EQ(r.go(0.5, true, target, kDirectionCcw), ExecutorState::SWITCHING_MODE);
  EXPECT_EQ(r.chassis.target, 4);
  EXPECT_DOUBLE_EQ(r.chassis.yaw, 0.0);  // no motion while the wheels re-orient
  EXPECT_EQ(r.go(20.0, true, target, kDirectionCcw), ExecutorState::DONE);
  EXPECT_NEAR(normalize_angle(r.chassis.yaw - target), 0.0, 3.5 * kDeg);
  EXPECT_EQ(r.chassis.mode, 2);  // driving mode restored
  EXPECT_EQ(r.go(0.2, false, target, kDirectionCcw), ExecutorState::IDLE);
}

TEST(Executor, LargeTurnFollowsTheCommandedDirection)
{
  // Target 90 deg to the left, but the supervisor asked for CW (left side blocked): the robot
  // must go round the long way, 270 deg clockwise, never through the left.
  Sim r;
  const double target = 90.0 * kDeg;
  r.go(1.2, true, target, kDirectionCw);  // through the switch
  double min_yaw = 0.0;
  for (int i = 0; i < 400 && r.last != ExecutorState::DONE; ++i) {
    r.go(0.05, true, target, kDirectionCw);
    if (r.chassis.yaw > 0.0 && r.chassis.yaw < 80.0 * kDeg) {
      min_yaw = -1.0;  // went through the forbidden side
    }
  }
  EXPECT_EQ(r.last, ExecutorState::DONE);
  EXPECT_EQ(min_yaw, 0.0);
}

TEST(Executor, SmallErrorLongWayStillFollowsTheCommand)
{
  // 60 deg to the left with the left blocked: 300 deg clockwise, never through the left.
  Sim r;
  const double target = 60.0 * kDeg;
  bool went_left = false;
  for (int i = 0; i < 600 && r.last != ExecutorState::DONE; ++i) {
    r.go(0.05, true, target, kDirectionCw);
    if (r.chassis.yaw > 1.0 * kDeg && r.chassis.yaw < 55.0 * kDeg) {
      went_left = true;
    }
  }
  EXPECT_EQ(r.last, ExecutorState::DONE);
  EXPECT_FALSE(went_left);
}

TEST(Executor, CancelWhileRotatingRestoresTheMode)
{
  Sim r;
  r.go(3.0, true, 170.0 * kDeg, kDirectionCcw);
  ASSERT_EQ(r.last, ExecutorState::ROTATING);
  EXPECT_EQ(r.go(0.1, false, 170.0 * kDeg, kDirectionCcw), ExecutorState::RESTORING_MODE);
  EXPECT_EQ(r.go(2.0, false, 170.0 * kDeg, kDirectionCcw), ExecutorState::IDLE);
  EXPECT_EQ(r.chassis.mode, 2);
}

TEST(Executor, ChassisNeverSwitchingFails)
{
  Sim r;
  r.chassis.switch_time = 1.0e9;  // the chassis never finishes the switch
  EXPECT_EQ(r.go(6.0, true, 170.0 * kDeg, kDirectionCcw), ExecutorState::FAILED);
}

TEST(Executor, RestoresThePreTurnMode)
{
  Sim r;
  r.chassis.mode = 0;
  r.chassis.target = 0;
  EXPECT_EQ(r.go(20.0, true, -100.0 * kDeg, kDirectionCw), ExecutorState::DONE);
  EXPECT_EQ(r.chassis.mode, 0);  // came back to FWS, not to the driving_mode parameter
}

TEST(Executor, ModeRequestIsNotSpammedDuringTheSwitch)
{
  // Re-sending during a switch could restart it; the request goes out once per phase.
  TurnInPlaceExecutor executor{ExecutorParams{}};
  int requests = 0;
  for (int i = 0; i < 10; ++i) {
    ExecutorInput in;
    in.now = 0.05 * i;
    in.command_active = true;
    in.target_yaw = M_PI;
    in.direction = kDirectionCcw;
    in.feedback_fresh = true;
    in.feedback_mode = 2;
    in.feedback_target_mode = 4;  // taken up, still switching
    in.feedback_switching = true;
    if (executor.step(in).mode_request) {
      ++requests;
    }
  }
  EXPECT_EQ(requests, 1);
}
