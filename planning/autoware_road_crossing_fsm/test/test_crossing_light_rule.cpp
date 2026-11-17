// Copyright 2026 IRISA
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

#include "autoware/road_crossing_fsm/crossing_light_rule.hpp"

#include <gtest/gtest.h>

using autoware::road_crossing_fsm::crossing_light::lightStillAllowsCrossing;
using autoware::road_crossing_fsm::crossing_light::Status;

TEST(CrossingLightRule, GoRuleAlwaysKeepsGoing)
{
  EXPECT_TRUE(lightStillAllowsCrossing(true, Status::HIJAU, true, 0.0, 1.0));
}

TEST(CrossingLightRule, RedRevokesAtOnce)
{
  EXPECT_FALSE(lightStillAllowsCrossing(false, Status::MERAH, true, 0.0, 1.0));
}

TEST(CrossingLightRule, ShortNoneInsideGreenPhaseIsTolerated)
{
  // 2026-11-16 14:48:33: 0.3 s NONE right after GO sent ego back to WAIT_GREEN
  EXPECT_TRUE(lightStillAllowsCrossing(false, Status::NONE, true, 0.3, 1.0));
  EXPECT_TRUE(lightStillAllowsCrossing(false, Status::NONE, true, 0.99, 1.0));
}

TEST(CrossingLightRule, LongNoneRevokes)
{
  EXPECT_FALSE(lightStillAllowsCrossing(false, Status::NONE, true, 1.0, 1.0));
  EXPECT_FALSE(lightStillAllowsCrossing(false, Status::NONE, true, 5.0, 1.0));
}

TEST(CrossingLightRule, GreenBackAfterDropoutNeedsNoNewStableTime)
{
  // stable counter was reset by the NONE, allows_crossing false, but the light is green again
  EXPECT_TRUE(lightStillAllowsCrossing(false, Status::HIJAU, true, 0.0, 1.0));
}

TEST(CrossingLightRule, GreenPhaseOverRevokes)
{
  EXPECT_FALSE(lightStillAllowsCrossing(false, Status::HIJAU, false, 0.0, 1.0));
  EXPECT_FALSE(lightStillAllowsCrossing(false, Status::NONE, false, 0.1, 1.0));
}

TEST(CrossingLightRule, GreenRightAfterRedOpensThePhase)
{
  using autoware::road_crossing_fsm::crossing_light::greenFollowsRed;
  EXPECT_TRUE(greenFollowsRed(0, 10));    // MERAH -> HIJAU
  EXPECT_TRUE(greenFollowsRed(10, 10));   // 1 s NONE in between (camera switch, majority)
}

TEST(CrossingLightRule, GreenAfterALongNoneWaitsForRed)
{
  using autoware::road_crossing_fsm::crossing_light::greenFollowsRed;
  // red, then covered for the whole solid green, opened in the blinking green
  EXPECT_FALSE(greenFollowsRed(11, 10));
  EXPECT_FALSE(greenFollowsRed(300, 10));
  EXPECT_TRUE(greenFollowsRed(300, -1));  // rule off
}

TEST(CrossingLightRule, ZeroToleranceIsTheOldBehaviourForNone)
{
  EXPECT_FALSE(lightStillAllowsCrossing(false, Status::NONE, true, 0.0, 0.0));
}
