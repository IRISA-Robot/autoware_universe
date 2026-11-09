// Copyright 2026 The Autoware Foundation
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

// [STEERING-MODE-MOCK 2026-11-05] unit tests of the steering-mode mock state machine.

#include "autoware/simple_planning_simulator/utils/steering_mode_mock.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace autoware::simulator::simple_planning_simulator
{
namespace
{
using Action = SteeringModeMock::Action;
constexpr double DT = 0.025;
constexpr uint8_t DRIVING = 2;
constexpr uint8_t HOLONOMIC = SteeringModeMock::HOLONOMIC;

SteeringModeMockConfig make_config()
{
  SteeringModeMockConfig c;
  c.driving_mode = DRIVING;
  c.mode_switch_time_s = 1.5;
  c.max_yaw_rate = 1.0;
  c.max_yaw_acc = 2.0;
  c.fail_switch = false;
  return c;
}

/// Step from t to t_end at DT, returning the last action; t is advanced.
Action run_until(SteeringModeMock & mock, double & t, const double t_end)
{
  Action a = Action::RUN_MODEL;
  while (t < t_end - 1e-9) {
    t += DT;
    a = mock.step(t, DT);
  }
  return a;
}

/// Bring the mock into an active HOLONOMIC mode at pose p; t is advanced past the switch.
void enter_holonomic(SteeringModeMock & mock, double & t, const Pose2D & p)
{
  mock.request_mode(HOLONOMIC, t, p);
  run_until(mock, t, t + 1.5 + DT);
  ASSERT_EQ(mock.mode(), HOLONOMIC);
  ASSERT_FALSE(mock.switching());
}
}  // namespace

TEST(SteeringModeMock, StartsInDrivingModeAndRunsModel)
{
  SteeringModeMock mock(make_config());
  EXPECT_EQ(mock.mode(), DRIVING);
  EXPECT_EQ(mock.target_mode(), DRIVING);
  EXPECT_FALSE(mock.switching());
  EXPECT_FALSE(mock.holding());
  EXPECT_EQ(mock.step(0.0, DT), Action::RUN_MODEL);
}

TEST(SteeringModeMock, SwitchTimeHoldsVehicle)
{
  SteeringModeMock mock(make_config());
  const Pose2D p{3.0, -2.0, 0.5};
  double t = 10.0;
  mock.request_mode(HOLONOMIC, t, p);
  EXPECT_TRUE(mock.switching());
  EXPECT_EQ(mock.mode(), DRIVING);  // still the mode in force
  EXPECT_EQ(mock.target_mode(), HOLONOMIC);

  // A twist during the transition must not move the chassis.
  mock.set_yaw_rate_command(1.0, t);
  while (t + DT < 10.0 + 1.5 - 1e-9) {
    t += DT;
    mock.set_yaw_rate_command(1.0, t);
    EXPECT_EQ(mock.step(t, DT), Action::HOLD);
    EXPECT_TRUE(mock.switching());
    EXPECT_TRUE(mock.holding());
    EXPECT_DOUBLE_EQ(mock.yaw_rate(), 0.0);
    EXPECT_DOUBLE_EQ(mock.held_pose().x, 3.0);
    EXPECT_DOUBLE_EQ(mock.held_pose().y, -2.0);
    EXPECT_DOUBLE_EQ(mock.held_pose().yaw, 0.5);
  }
  t = 10.0 + 1.5;
  EXPECT_EQ(mock.step(t, DT), Action::HOLD);  // now HOLONOMIC, still frozen
  EXPECT_FALSE(mock.switching());
  EXPECT_EQ(mock.mode(), HOLONOMIC);
}

TEST(SteeringModeMock, HolonomicYawIntegratesWithLimitsPositionFixed)
{
  SteeringModeMock mock(make_config());
  const Pose2D p{1.0, 2.0, 0.0};
  double t = 0.0;
  enter_holonomic(mock, t, p);

  // Command beyond max_yaw_rate: rate ramps at max_yaw_acc and saturates at max_yaw_rate.
  double prev_rate = mock.yaw_rate();
  double expected_yaw = mock.held_pose().yaw;
  for (int i = 0; i < 80; ++i) {  // 2 s
    t += DT;
    mock.set_yaw_rate_command(5.0, t);
    EXPECT_EQ(mock.step(t, DT), Action::HOLD);
    EXPECT_LE(mock.yaw_rate() - prev_rate, 2.0 * DT + 1e-9);
    EXPECT_LE(mock.yaw_rate(), 1.0 + 1e-9);
    expected_yaw += mock.yaw_rate() * DT;
    prev_rate = mock.yaw_rate();
    EXPECT_DOUBLE_EQ(mock.held_pose().x, 1.0);
    EXPECT_DOUBLE_EQ(mock.held_pose().y, 2.0);
  }
  EXPECT_NEAR(mock.yaw_rate(), 1.0, 1e-9);
  // 0.5 s ramp (0.25 rad) + 1.5 s at 1 rad/s ~= 1.75 rad
  EXPECT_NEAR(std::remainder(expected_yaw, 2.0 * M_PI), mock.held_pose().yaw, 1e-9);
  EXPECT_NEAR(mock.held_pose().yaw, 1.75, 0.05);

  // Negative command, also limited.
  for (int i = 0; i < 80; ++i) {
    t += DT;
    mock.set_yaw_rate_command(-5.0, t);
    mock.step(t, DT);
    EXPECT_GE(mock.yaw_rate(), -1.0 - 1e-9);
  }
  EXPECT_NEAR(mock.yaw_rate(), -1.0, 1e-9);
}

TEST(SteeringModeMock, StaleTwistCountsAsZero)
{
  SteeringModeMock mock(make_config());
  double t = 0.0;
  enter_holonomic(mock, t, Pose2D{});
  mock.set_yaw_rate_command(1.0, t);
  run_until(mock, t, t + 0.4);
  EXPECT_GT(mock.yaw_rate(), 0.5);
  // No new command for > 0.5 s: the rate decays to zero at max_yaw_acc.
  run_until(mock, t, t + 1.5);
  EXPECT_DOUBLE_EQ(mock.yaw_rate(), 0.0);
  const double yaw = mock.held_pose().yaw;
  run_until(mock, t, t + 1.0);
  EXPECT_DOUBLE_EQ(mock.held_pose().yaw, yaw);
}

TEST(SteeringModeMock, ReturningRestoresDrivingAtHeldPose)
{
  SteeringModeMock mock(make_config());
  double t = 0.0;
  enter_holonomic(mock, t, Pose2D{5.0, 6.0, 0.0});
  for (int i = 0; i < 40; ++i) {
    t += DT;
    mock.set_yaw_rate_command(1.0, t);
    mock.step(t, DT);
  }
  ASSERT_GT(mock.yaw_rate(), 0.0);
  const double spun_yaw = mock.held_pose().yaw;

  // Request driving with a bogus "current pose": the held pose must win.
  mock.request_mode(DRIVING, t, Pose2D{99.0, 99.0, 3.0});
  EXPECT_TRUE(mock.switching());
  EXPECT_DOUBLE_EQ(mock.yaw_rate(), 0.0);  // held still during the transition
  const double t_req = t;
  Action a = Action::HOLD;
  while (a == Action::HOLD) {
    t += DT;
    mock.set_yaw_rate_command(1.0, t);
    a = mock.step(t, DT);
    if (a == Action::HOLD) {
      EXPECT_DOUBLE_EQ(mock.held_pose().yaw, spun_yaw);
      EXPECT_DOUBLE_EQ(mock.yaw_rate(), 0.0);
    }
  }
  EXPECT_EQ(a, Action::RESUME_FROM_HOLD);
  EXPECT_NEAR(t - t_req, 1.5, DT + 1e-9);
  EXPECT_EQ(mock.mode(), DRIVING);
  EXPECT_FALSE(mock.switching());
  EXPECT_FALSE(mock.holding());
  EXPECT_DOUBLE_EQ(mock.yaw_rate(), 0.0);
  EXPECT_DOUBLE_EQ(mock.held_pose().x, 5.0);
  EXPECT_DOUBLE_EQ(mock.held_pose().y, 6.0);
  EXPECT_DOUBLE_EQ(mock.held_pose().yaw, spun_yaw);
  // RESUME is reported once, then the model runs normally.
  t += DT;
  EXPECT_EQ(mock.step(t, DT), Action::RUN_MODEL);
}

TEST(SteeringModeMock, FailSwitchNeverCompletes)
{
  auto config = make_config();
  config.fail_switch = true;
  SteeringModeMock mock(config);
  double t = 0.0;
  mock.request_mode(HOLONOMIC, t, Pose2D{});
  EXPECT_EQ(run_until(mock, t, 30.0), Action::HOLD);
  EXPECT_TRUE(mock.switching());
  EXPECT_EQ(mock.mode(), DRIVING);
  EXPECT_EQ(mock.target_mode(), HOLONOMIC);

  // Clearing the fault at runtime lets the (overdue) transition complete.
  mock.set_fail_switch(false);
  t += DT;
  mock.step(t, DT);
  EXPECT_FALSE(mock.switching());
  EXPECT_EQ(mock.mode(), HOLONOMIC);
}

TEST(SteeringModeMock, SameModeRequestIsNoOp)
{
  SteeringModeMock mock(make_config());
  double t = 0.0;
  mock.request_mode(DRIVING, t, Pose2D{});
  EXPECT_FALSE(mock.switching());
  EXPECT_EQ(mock.step(t + DT, DT), Action::RUN_MODEL);

  // Re-requesting the target mid-transition does not restart the timer.
  t = 1.0;
  mock.request_mode(HOLONOMIC, t, Pose2D{});
  run_until(mock, t, 2.0);
  mock.request_mode(HOLONOMIC, t, Pose2D{});
  run_until(mock, t, 2.5 + DT);
  EXPECT_EQ(mock.mode(), HOLONOMIC);
  EXPECT_FALSE(mock.switching());

  // And in HOLONOMIC.
  mock.request_mode(HOLONOMIC, t, Pose2D{});
  EXPECT_FALSE(mock.switching());
}

TEST(SteeringModeMock, NewRequestDuringTransitionRestartsIt)
{
  SteeringModeMock mock(make_config());
  double t = 0.0;
  mock.request_mode(HOLONOMIC, t, Pose2D{1.0, 1.0, 0.0});
  run_until(mock, t, 1.0);
  mock.request_mode(3, t, Pose2D{50.0, 50.0, 1.0});  // CRAB; held pose must not change
  EXPECT_EQ(mock.target_mode(), 3);
  EXPECT_DOUBLE_EQ(mock.held_pose().x, 1.0);
  run_until(mock, t, 2.0);  // 1.0 s into the restarted transition
  EXPECT_TRUE(mock.switching());
  Action a = Action::HOLD;
  while (a == Action::HOLD && t < 10.0) {
    t += DT;
    a = mock.step(t, DT);
  }
  EXPECT_EQ(a, Action::RESUME_FROM_HOLD);
  EXPECT_NEAR(t, 2.5, DT + 1e-9);
  EXPECT_EQ(mock.mode(), 3);
}

TEST(SteeringModeMock, PausedMotionDoesNotSpin)
{
  SteeringModeMock mock(make_config());
  double t = 0.0;
  enter_holonomic(mock, t, Pose2D{});
  for (int i = 0; i < 20; ++i) {
    t += DT;
    mock.set_yaw_rate_command(1.0, t);
    mock.step(t, 0.0);
  }
  EXPECT_DOUBLE_EQ(mock.yaw_rate(), 0.0);
  EXPECT_DOUBLE_EQ(mock.held_pose().yaw, 0.0);
}

}  // namespace autoware::simulator::simple_planning_simulator
