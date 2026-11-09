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

#include "autoware/stuck_recovery_supervisor/mission_direction.hpp"

#include <gtest/gtest.h>

namespace autoware::stuck_recovery_supervisor
{

TEST(MissionDirection, ParsesKnownValues)
{
  EXPECT_EQ(parse_mission_direction("auto"), MissionDirection::AUTO);
  EXPECT_EQ(parse_mission_direction("forward_only"), MissionDirection::FORWARD_ONLY);
}

TEST(MissionDirection, RejectsUnknownValues)
{
  EXPECT_FALSE(parse_mission_direction("").has_value());
  EXPECT_FALSE(parse_mission_direction("forward").has_value());
  EXPECT_FALSE(parse_mission_direction("FORWARD_ONLY").has_value());
  EXPECT_FALSE(parse_mission_direction("reverse").has_value());
}

TEST(MissionDirection, NamesRoundTrip)
{
  for (const auto d : {MissionDirection::AUTO, MissionDirection::FORWARD_ONLY}) {
    EXPECT_EQ(parse_mission_direction(mission_direction_name(d)), d);
  }
}

TEST(MissionDirection, AutoFollowsTheGeometry)
{
  // The original behaviour: a path opposing ego's heading is a reverse mission.
  EXPECT_FALSE(mission_is_reverse(MissionDirection::AUTO, false));
  EXPECT_TRUE(mission_is_reverse(MissionDirection::AUTO, true));
  EXPECT_DOUBLE_EQ(mission_travel_sign(MissionDirection::AUTO, false), 1.0);
  EXPECT_DOUBLE_EQ(mission_travel_sign(MissionDirection::AUTO, true), -1.0);
}

TEST(MissionDirection, ForwardOnlyIsAlwaysNoseFirst)
{
  // Robot v2: before the turn in place the body faces against the route.  That must not
  // read as a reverse mission.
  EXPECT_FALSE(mission_is_reverse(MissionDirection::FORWARD_ONLY, false));
  EXPECT_FALSE(mission_is_reverse(MissionDirection::FORWARD_ONLY, true));
  EXPECT_DOUBLE_EQ(mission_travel_sign(MissionDirection::FORWARD_ONLY, false), 1.0);
  EXPECT_DOUBLE_EQ(mission_travel_sign(MissionDirection::FORWARD_ONLY, true), 1.0);
}

}  // namespace autoware::stuck_recovery_supervisor
