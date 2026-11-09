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

// [TURN-IN-PLACE 2026-11-05]

#include "autoware/stuck_recovery_supervisor/turn_in_place_gate.hpp"

#include <autoware_turn_in_place_msgs/msg/turn_in_place_supervisor_state.hpp>

#include <gtest/gtest.h>

namespace autoware::stuck_recovery_supervisor
{

using TurnInPlaceState = autoware_turn_in_place_msgs::msg::TurnInPlaceSupervisorState;

namespace
{
TurnInPlaceGateInput fresh(uint8_t state)
{
  TurnInPlaceGateInput in;
  in.respect = true;
  in.have_message = true;
  in.state = state;
  in.age_sec = 0.5;
  in.timeout_sec = 2.0;
  return in;
}
}  // namespace

TEST(TurnInPlaceGate, HoldingStates)
{
  EXPECT_FALSE(turn_in_place_state_holds(TurnInPlaceState::MONITOR));
  EXPECT_TRUE(turn_in_place_state_holds(TurnInPlaceState::REQUEST_STOP));
  EXPECT_TRUE(turn_in_place_state_holds(TurnInPlaceState::CHECK_CLEARANCE));
  EXPECT_TRUE(turn_in_place_state_holds(TurnInPlaceState::TURNING));
  EXPECT_TRUE(turn_in_place_state_holds(TurnInPlaceState::COOLDOWN));
  // ERROR means the turn failed: the robot may really be stuck, normal handling applies.
  EXPECT_FALSE(turn_in_place_state_holds(TurnInPlaceState::ERROR));
  EXPECT_FALSE(turn_in_place_state_holds(42));
}

TEST(TurnInPlaceGate, FreshHoldingStateGates)
{
  for (const uint8_t s :
       {TurnInPlaceState::REQUEST_STOP, TurnInPlaceState::CHECK_CLEARANCE,
        TurnInPlaceState::TURNING, TurnInPlaceState::COOLDOWN}) {
    EXPECT_EQ(evaluate_turn_in_place_gate(fresh(s)), TurnInPlaceGateStatus::HOLDING)
      << turn_in_place_state_name(s);
    EXPECT_TRUE(gate_holds(evaluate_turn_in_place_gate(fresh(s))));
  }
}

TEST(TurnInPlaceGate, MonitorAndErrorDoNotGate)
{
  EXPECT_EQ(
    evaluate_turn_in_place_gate(fresh(TurnInPlaceState::MONITOR)),
    TurnInPlaceGateStatus::NOT_HOLDING);
  EXPECT_EQ(
    evaluate_turn_in_place_gate(fresh(TurnInPlaceState::ERROR)),
    TurnInPlaceGateStatus::NOT_HOLDING);
}

TEST(TurnInPlaceGate, NeverHeardIsInert)
{
  auto in = fresh(TurnInPlaceState::TURNING);
  in.have_message = false;
  EXPECT_EQ(evaluate_turn_in_place_gate(in), TurnInPlaceGateStatus::NEVER_HEARD);
  EXPECT_FALSE(gate_holds(evaluate_turn_in_place_gate(in)));
}

TEST(TurnInPlaceGate, DisabledNeverGates)
{
  auto in = fresh(TurnInPlaceState::TURNING);
  in.respect = false;
  EXPECT_EQ(evaluate_turn_in_place_gate(in), TurnInPlaceGateStatus::DISABLED);
}

TEST(TurnInPlaceGate, StaleHoldingStateReleases)
{
  auto in = fresh(TurnInPlaceState::TURNING);
  in.age_sec = 2.0;  // at the limit: still fresh
  EXPECT_EQ(evaluate_turn_in_place_gate(in), TurnInPlaceGateStatus::HOLDING);
  in.age_sec = 2.01;
  EXPECT_EQ(evaluate_turn_in_place_gate(in), TurnInPlaceGateStatus::STALE);
  EXPECT_FALSE(gate_holds(evaluate_turn_in_place_gate(in)));
}

TEST(TurnInPlaceGate, StateNames)
{
  EXPECT_EQ(turn_in_place_state_name(TurnInPlaceState::MONITOR), "MONITOR");
  EXPECT_EQ(turn_in_place_state_name(TurnInPlaceState::REQUEST_STOP), "REQUEST_STOP");
  EXPECT_EQ(turn_in_place_state_name(TurnInPlaceState::CHECK_CLEARANCE), "CHECK_CLEARANCE");
  EXPECT_EQ(turn_in_place_state_name(TurnInPlaceState::TURNING), "TURNING");
  EXPECT_EQ(turn_in_place_state_name(TurnInPlaceState::COOLDOWN), "COOLDOWN");
  EXPECT_EQ(turn_in_place_state_name(TurnInPlaceState::ERROR), "ERROR");
  EXPECT_EQ(turn_in_place_state_name(9), "UNKNOWN(9)");
}

}  // namespace autoware::stuck_recovery_supervisor
