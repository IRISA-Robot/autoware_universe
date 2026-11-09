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

#include "autoware/turn_in_place_supervisor/decision.hpp"

#include "autoware/turn_in_place_supervisor/geometry.hpp"

#include <gtest/gtest.h>

#include <cmath>

using autoware::turn_in_place_supervisor::DecisionInput;
using autoware::turn_in_place_supervisor::DecisionParams;
using autoware::turn_in_place_supervisor::ExecutorState;
using autoware::turn_in_place_supervisor::kDirectionCcw;
using autoware::turn_in_place_supervisor::kDirectionCw;
using autoware::turn_in_place_supervisor::SupervisorState;
using autoware::turn_in_place_supervisor::TurnInPlaceDecision;

namespace
{
constexpr double kDeg = M_PI / 180.0;

DecisionInput standing(double now, double yaw_error_deg)
{
  DecisionInput in;
  in.now = now;
  in.autonomous = true;
  in.route_valid = true;
  in.yaw_error = yaw_error_deg * kDeg;
  in.route_yaw = 0.0;
  in.speed = 0.0;
  in.ccw_clear = true;
  in.cw_clear = true;
  return in;
}

// Drives the decision from MONITOR into TURNING with a standing robot and a free area.
void into_turning(TurnInPlaceDecision & d, double yaw_error_deg)
{
  d.step(standing(0.0, yaw_error_deg));  // 1st cycle over the trigger
  d.step(standing(1.0, yaw_error_deg));  // 2nd -> REQUEST_STOP
  d.step(standing(2.0, yaw_error_deg));  // standing -> CHECK_CLEARANCE
  d.step(standing(3.0, yaw_error_deg));  // free -> TURNING
}
}  // namespace

TEST(Decision, ConfirmedAfterTriggerConfirmSecButHeldAtOnce)
{
  TurnInPlaceDecision d{DecisionParams{}};  // trigger_confirm_sec 0.3
  auto out = d.step(standing(0.0, 85.0));
  EXPECT_EQ(out.state, SupervisorState::MONITOR);
  EXPECT_TRUE(out.hold_vehicle);  // held from the very first cycle
  out = d.step(standing(0.1, 20.0));  // back under: confirmation restarts, hold released
  EXPECT_FALSE(out.hold_vehicle);
  out = d.step(standing(0.2, 85.0));
  out = d.step(standing(0.4, 85.0));
  EXPECT_EQ(out.state, SupervisorState::MONITOR);  // 0.2 s over
  out = d.step(standing(0.5, 85.0));
  EXPECT_EQ(out.state, SupervisorState::REQUEST_STOP);  // 0.3 s over
  EXPECT_TRUE(out.hold_vehicle);
}

TEST(Decision, HeldBeforeAutonomousWhenHeadingIsOff)
{
  // Not engaged yet: no turn, but the robot is already held so that engaging cannot move it
  // the wrong way.
  TurnInPlaceDecision d{DecisionParams{}};
  auto in = standing(0.0, 170.0);
  in.autonomous = false;
  auto out = d.step(in);
  EXPECT_EQ(out.state, SupervisorState::MONITOR);
  EXPECT_TRUE(out.hold_vehicle);
  EXPECT_FALSE(out.command_active);
  in.yaw_error = 5.0 * M_PI / 180.0;
  EXPECT_FALSE(d.step(in).hold_vehicle);
}

TEST(Decision, UnderTheTriggerNeverActs)
{
  TurnInPlaceDecision d{DecisionParams{}};
  for (int i = 0; i < 10; ++i) {
    EXPECT_EQ(d.step(standing(i, 75.0)).state, SupervisorState::MONITOR);
  }
}

TEST(Decision, MovingRobotIsStoppedFirst)
{
  TurnInPlaceDecision d{DecisionParams{}};
  auto moving = standing(0.0, 170.0);
  moving.speed = 1.0;
  d.step(moving);
  moving.now = 1.0;
  auto out = d.step(moving);
  ASSERT_EQ(out.state, SupervisorState::REQUEST_STOP);
  EXPECT_TRUE(out.hold_vehicle);
  EXPECT_FALSE(out.command_active);
  moving.now = 2.0;
  EXPECT_EQ(d.step(moving).state, SupervisorState::REQUEST_STOP);  // still rolling
  out = d.step(standing(3.0, 170.0));
  EXPECT_EQ(out.state, SupervisorState::CHECK_CLEARANCE);
}

TEST(Decision, NeverStoppingTimesOut)
{
  DecisionParams p;
  p.stop_timeout_sec = 5.0;
  TurnInPlaceDecision d{p};
  auto moving = standing(0.0, 170.0);
  moving.speed = 1.0;
  for (int i = 0; i <= 8; ++i) {
    moving.now = i;
    d.step(moving);
  }
  EXPECT_EQ(d.state(), SupervisorState::ERROR);
}

TEST(Decision, TurnsTheShortWayWhenFree)
{
  TurnInPlaceDecision d{DecisionParams{}};
  into_turning(d, 120.0);  // route is 120 deg to the left: CCW is short
  auto out = d.step(standing(4.0, 120.0));
  EXPECT_EQ(out.state, SupervisorState::TURNING);
  EXPECT_TRUE(out.command_active);
  EXPECT_EQ(out.direction, kDirectionCcw);

  TurnInPlaceDecision d2{DecisionParams{}};
  into_turning(d2, -120.0);
  EXPECT_EQ(d2.step(standing(4.0, -120.0)).direction, kDirectionCw);
}

TEST(Decision, ShortWayBlockedTurnsTheOtherWay)
{
  TurnInPlaceDecision d{DecisionParams{}};
  d.step(standing(0.0, 120.0));
  d.step(standing(1.0, 120.0));
  d.step(standing(2.0, 120.0));
  auto in = standing(3.0, 120.0);
  in.ccw_clear = false;
  const auto out = d.step(in);
  EXPECT_EQ(out.state, SupervisorState::TURNING);
  EXPECT_EQ(out.direction, kDirectionCw);
}

TEST(Decision, BothSidesBlockedWaitsThenTimesOut)
{
  DecisionParams p;
  p.clearance_timeout_sec = 10.0;
  TurnInPlaceDecision d{p};
  d.step(standing(0.0, 180.0));
  d.step(standing(1.0, 180.0));
  d.step(standing(2.0, 180.0));
  auto blocked = standing(3.0, 180.0);
  blocked.ccw_clear = false;
  blocked.cw_clear = false;
  auto out = d.step(blocked);
  EXPECT_EQ(out.state, SupervisorState::CHECK_CLEARANCE);
  EXPECT_TRUE(out.hold_vehicle);
  EXPECT_FALSE(out.command_active);
  blocked.now = 8.0;
  EXPECT_EQ(d.step(blocked).state, SupervisorState::CHECK_CLEARANCE);
  // Freed before the timeout: the turn goes ahead (obstacles, not the road: no back-up).
  blocked.now = 9.0;
  blocked.cw_clear = true;
  EXPECT_EQ(d.step(blocked).state, SupervisorState::TURNING);

  TurnInPlaceDecision d2{p};
  d2.step(standing(0.0, 180.0));
  d2.step(standing(1.0, 180.0));
  d2.step(standing(2.0, 180.0));
  blocked.cw_clear = false;
  for (int i = 3; i <= 14; ++i) {
    blocked.now = i;
    d2.step(blocked);
  }
  EXPECT_EQ(d2.state(), SupervisorState::ERROR);
}

TEST(Decision, TurnDoneCoolsDownThenHandsBack)
{
  DecisionParams p;
  p.cooldown_sec = 3.0;
  TurnInPlaceDecision d{p};
  into_turning(d, 170.0);
  auto in = standing(4.0, 5.0);
  in.executor_fresh = true;
  in.executor_state = ExecutorState::DONE;
  auto out = d.step(in);
  EXPECT_EQ(out.state, SupervisorState::COOLDOWN);
  EXPECT_TRUE(out.hold_vehicle);
  EXPECT_FALSE(out.command_active);
  in.now = 6.0;
  EXPECT_EQ(d.step(in).state, SupervisorState::COOLDOWN);
  in.now = 7.0;
  out = d.step(in);
  EXPECT_EQ(out.state, SupervisorState::MONITOR);
  EXPECT_FALSE(out.hold_vehicle);
}

TEST(Decision, ExecutorDoneButHeadingStillOffReChecks)
{
  TurnInPlaceDecision d{DecisionParams{}};
  into_turning(d, 170.0);
  auto in = standing(4.0, 30.0);
  in.executor_fresh = true;
  in.executor_state = ExecutorState::DONE;
  EXPECT_EQ(d.step(in).state, SupervisorState::CHECK_CLEARANCE);
}

TEST(Decision, BlockedDuringTheTurnStopsIt)
{
  TurnInPlaceDecision d{DecisionParams{}};
  into_turning(d, 170.0);  // CCW
  auto in = standing(4.0, 100.0);
  in.executor_fresh = true;
  in.executor_state = ExecutorState::ROTATING;
  in.ccw_clear = false;
  const auto out = d.step(in);
  EXPECT_EQ(out.state, SupervisorState::CHECK_CLEARANCE);
  EXPECT_FALSE(out.command_active);
  EXPECT_TRUE(out.hold_vehicle);
}

TEST(Decision, ExecutorFailureIsAnError)
{
  TurnInPlaceDecision d{DecisionParams{}};
  into_turning(d, 170.0);
  auto in = standing(4.0, 170.0);
  in.executor_fresh = true;
  in.executor_state = ExecutorState::FAILED;
  const auto out = d.step(in);
  EXPECT_EQ(out.state, SupervisorState::ERROR);
  EXPECT_TRUE(out.hold_vehicle);
  EXPECT_FALSE(out.command_active);
}

TEST(Decision, TurnTimesOut)
{
  DecisionParams p;
  p.turn_timeout_sec = 5.0;
  TurnInPlaceDecision d{p};
  into_turning(d, 170.0);
  auto in = standing(0.0, 170.0);
  in.executor_fresh = true;
  in.executor_state = ExecutorState::SWITCHING_MODE;
  for (int i = 4; i <= 10; ++i) {
    in.now = i;
    d.step(in);
  }
  EXPECT_EQ(d.state(), SupervisorState::ERROR);
}

TEST(Decision, ErrorClearsWhenTheHeadingIsFine)
{
  DecisionParams p;
  p.stop_timeout_sec = 1.0;
  TurnInPlaceDecision d{p};
  auto moving = standing(0.0, 170.0);
  moving.speed = 1.0;
  for (int i = 0; i <= 4; ++i) {
    moving.now = i;
    d.step(moving);
  }
  ASSERT_EQ(d.state(), SupervisorState::ERROR);
  EXPECT_EQ(d.step(standing(5.0, 150.0)).state, SupervisorState::ERROR);
  EXPECT_EQ(d.step(standing(6.0, 2.0)).state, SupervisorState::MONITOR);
}

TEST(Decision, LeavingAutonomousOrANewRouteResets)
{
  TurnInPlaceDecision d{DecisionParams{}};
  into_turning(d, 170.0);
  auto in = standing(4.0, 170.0);
  in.autonomous = false;
  auto out = d.step(in);
  EXPECT_EQ(out.state, SupervisorState::MONITOR);
  EXPECT_TRUE(out.hold_vehicle);  // heading still off: keep holding until it is engaged again
  EXPECT_FALSE(out.command_active);

  TurnInPlaceDecision d2{DecisionParams{}};
  into_turning(d2, 170.0);
  auto in2 = standing(4.0, 170.0);
  in2.route_changed = true;
  EXPECT_EQ(d2.step(in2).state, SupervisorState::MONITOR);
}

TEST(Decision, UnknownRouteHeadingNeverTriggers)
{
  TurnInPlaceDecision d{DecisionParams{}};
  auto in = standing(0.0, 170.0);
  in.route_valid = false;
  for (int i = 0; i < 5; ++i) {
    in.now = i;
    EXPECT_EQ(d.step(in).state, SupervisorState::MONITOR);
  }
}

TEST(Decision, NarrowRoadBacksUpToTheTurnSpotThenTurns)
{
  TurnInPlaceDecision d{DecisionParams{}};
  d.step(standing(0.0, 180.0));
  d.step(standing(1.0, 180.0));
  d.step(standing(2.0, 180.0));  // CHECK_CLEARANCE
  auto narrow = standing(3.0, 180.0);
  narrow.ccw_clear = false;
  narrow.cw_clear = false;
  narrow.road_blocks_both = true;
  narrow.turn_spot_found = true;
  auto out = d.step(narrow);
  EXPECT_EQ(out.state, SupervisorState::REVERSING_TO_TURN_SPOT);
  EXPECT_TRUE(out.reverse_active);
  EXPECT_TRUE(out.reverse_to_spot);
  EXPECT_FALSE(out.hold_vehicle);  // the robot must move
  EXPECT_FALSE(out.command_active);

  narrow.now = 4.0;
  narrow.speed = 1.0;
  EXPECT_EQ(d.step(narrow).state, SupervisorState::REVERSING_TO_TURN_SPOT);

  // On the spot and standing: check again, and this time the spin fits.
  auto there = standing(9.0, 180.0);
  there.turn_spot_found = true;
  there.at_turn_spot = true;
  out = d.step(there);
  EXPECT_EQ(out.state, SupervisorState::CHECK_CLEARANCE);
  EXPECT_TRUE(out.hold_vehicle);
  EXPECT_FALSE(out.reverse_active);
  EXPECT_EQ(d.step(standing(9.1, 180.0)).state, SupervisorState::TURNING);
}

TEST(Decision, NoTurnSpotBacksUpTowardsTheGoalUntilOneAppears)
{
  TurnInPlaceDecision d{DecisionParams{}};
  d.step(standing(0.0, 180.0));
  d.step(standing(1.0, 180.0));
  d.step(standing(2.0, 180.0));
  auto narrow = standing(3.0, 180.0);
  narrow.ccw_clear = false;
  narrow.cw_clear = false;
  narrow.road_blocks_both = true;
  narrow.turn_spot_found = false;
  auto out = d.step(narrow);
  EXPECT_EQ(out.state, SupervisorState::REVERSING_TO_TURN_SPOT);
  EXPECT_TRUE(out.reverse_active);
  EXPECT_FALSE(out.reverse_to_spot);  // all the way to the goal
  narrow.now = 20.0;
  narrow.speed = 1.0;
  out = d.step(narrow);
  EXPECT_FALSE(out.reverse_to_spot);
  narrow.now = 21.0;
  narrow.turn_spot_found = true;  // the road widened ahead
  out = d.step(narrow);
  EXPECT_EQ(out.state, SupervisorState::REVERSING_TO_TURN_SPOT);
  EXPECT_TRUE(out.reverse_to_spot);
}

TEST(Decision, ObstaclesAloneNeverStartABackUp)
{
  // Blocked by obstacles but the road is wide enough: wait (still), never back up.
  TurnInPlaceDecision d{DecisionParams{}};
  d.step(standing(0.0, 180.0));
  d.step(standing(1.0, 180.0));
  d.step(standing(2.0, 180.0));
  auto blocked = standing(3.0, 180.0);
  blocked.ccw_clear = false;
  blocked.cw_clear = false;
  blocked.road_blocks_both = false;
  const auto out = d.step(blocked);
  EXPECT_EQ(out.state, SupervisorState::CHECK_CLEARANCE);
  EXPECT_FALSE(out.reverse_active);
  EXPECT_TRUE(out.hold_vehicle);
}

TEST(Decision, BackUpDisabledWaitsInstead)
{
  DecisionParams p;
  p.enable_reverse_to_turn_spot = false;
  TurnInPlaceDecision d{p};
  d.step(standing(0.0, 180.0));
  d.step(standing(1.0, 180.0));
  d.step(standing(2.0, 180.0));
  auto narrow = standing(3.0, 180.0);
  narrow.ccw_clear = false;
  narrow.cw_clear = false;
  narrow.road_blocks_both = true;
  EXPECT_EQ(d.step(narrow).state, SupervisorState::CHECK_CLEARANCE);
}
