// Copyright 2026 Autoware Planning Team
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

#ifndef AUTOWARE__BEHAVIOR_PATH_REVERSE_LANE_FOLLOW_MODULE__SCENE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_REVERSE_LANE_FOLLOW_MODULE__SCENE_HPP_

#include "autoware/behavior_path_planner_common/interface/scene_module_interface.hpp"
#include "autoware/behavior_path_reverse_lane_follow_module/data_structs.hpp"

#include <autoware_utils/ros/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <std_msgs/msg/float64.hpp>

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware_internal_planning_msgs::msg::PathWithLaneId;

class ReverseLaneFollowModule : public SceneModuleInterface
{
public:
  ReverseLaneFollowModule(
    const std::string & name, rclcpp::Node & node,
    const std::shared_ptr<ReverseLaneFollowParameters> & parameters,
    const std::shared_ptr<autoware_utils::InterProcessPollingSubscriber<std_msgs::msg::Float64>> &
      retrace_request_subscriber,
    const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
    std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
      objects_of_interest_marker_interface_ptr_map,
    const std::shared_ptr<PlanningFactorInterface> planning_factor_interface,
    const std::shared_ptr<std::optional<lanelet::Id>> & last_exited_inverted_lanelet_id);

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  void updateData() override;
  BehaviorModuleOutput plan() override;
  CandidateOutput planCandidate() const override;
  void processOnEntry() override;
  void processOnExit() override;

  void updateModuleParams(const std::any & parameters) override
  {
    parameters_ = std::any_cast<std::shared_ptr<ReverseLaneFollowParameters>>(parameters);
  }

  void acceptVisitor(
    [[maybe_unused]] const std::shared_ptr<SceneModuleVisitor> & visitor) const override
  {
  }

  // This module's own "am I currently retracing" status flag -- the reverse-lane-follow analogue
  // of autoware_behavior_path_start_planner_module's `isDrivingForward()` /
  // `status_.driving_forward` pattern. Other modules (or a future Phase-2 gating layer) can query
  // this to decide whether to stay out of the way.
  bool isRetracing() const { return status_.is_retracing; }

  // True when this module is active because ego is on a route segment RouteHandler's direction
  // side-table flags as reversed (is_reversed), as opposed to an explicit retrace request. See
  // updateData()/bidirectional_plan/04-routing-foundation.md §4d.
  bool isFollowingReversedRouteSegment() const { return status_.is_route_reversed_active; }

private:
  bool canTransitSuccessState() override;
  bool canTransitFailureState() override { return false; }

  // Polls the manager-owned retrace-request subscriber and refreshes requested_distance_m_.
  // Never creates/destroys the subscription itself (see manager.hpp).
  void updateRetraceRequest();

  // Checks RouteHandler::isLaneletInvertedInRoute() for ego's current lanelet and, if flagged,
  // builds a follow path directly from the route's own inverted centerline. This is the
  // alternative/additional activation trigger alongside the retrace-request trigger -- see
  // buildRouteReversedFollowPath().
  void updateRouteReversedFollow();

  // [BIDIR-BUG-FIX] One-way latch used to stop is_route_reversed_active from flip-flopping right
  // at a mid-route direction-change boundary (e.g. taman map's 18(inv) -> 291(fwd)):
  // getClosestLaneletWithinRoute() has no hysteresis, so as ego's pose sits within centimeters of
  // the shared boundary line, "closest lanelet" (and therefore is_inverted) can toggle several
  // times across consecutive planning cycles before settling. Each toggle re-emits a full
  // buildRouteReversedFollowPath() window built from ego's pose *at that instant*, so a vehicle
  // caught mid-oscillation can end up handed a forward-driving reference built from a slightly
  // different ego pose/velocity state each cycle -- observed live as an intermittent, several
  // -meter lateral excursion right at hand-off (ego ends up outside isEgoOutOfRoute()'s tolerance
  // and the whole route stalls) roughly half the time, even though the module's own direction
  // logic is otherwise correct. Once ego is seen to have genuinely left a given inverted lanelet
  // (a false transition), physically it should never need to re-enter reverse mode for that same
  // lanelet again -- re-arming only ever happens because of boundary-line noise, not a real
  // reversal of travel intent. Remember the id of the lanelet we just exited and refuse to
  // reactivate for it again, for the remaining lifetime of this node (see scene.cpp's
  // processOnEntry() for why there is no reliable earlier point to reset this).
  //
  // Owned by the (always-alive) ReverseLaneFollowModuleManager, not this instance: the planner
  // manager creates a *brand-new* ReverseLaneFollowModule via createNewSceneModuleInstance() for
  // every activation attempt, so a plain instance member here would reset on every single
  // re-attempt and never actually suppress anything (confirmed live).
  //
  // [BIDIR-BUG-FIX #3] This one-way latch resurfaced the exact bug commit 4f4bc9378 ("stop at
  // goal on reversed final approach") had already fixed: goals commonly sit right at a lanelet
  // boundary, exactly where getClosestLaneletWithinRoute()'s lack of hysteresis is most likely to
  // fire a spurious true->false transition -- if that transition latches the goal's own lanelet
  // (or any lanelet ego re-enters while still approaching the goal), plan() falls back to
  // getPreviousModuleOutput() permanently for that lanelet, which carries none of
  // buildRouteReversedFollowPath()'s goal-truncation/zero-velocity injection: ego drives straight
  // through the goal with no stop. Narrowed (not removed -- the original flip-flop bug this latch
  // exists for is still real) via two complementary, goal-scoped exemptions in
  // updateRouteReversedFollow():
  //   1. Never arm the latch for a lanelet where route_handler->isInGoalRouteSection() is true
  //      (see the exemption at the point the latch is set).
  //   2. Even if the latch is already armed for a lanelet in the newly-built follow window, allow
  //      reactivation anyway once ego is within parameters_->goal_reach_tolerance_m of
  //      route_handler->getGoalPose() (see reverse_lane_follow_utils::
  //      shouldSuppressReversedFollowReactivation() in utils.hpp).
  // Both are narrow, goal-scoped bypasses -- the latch still suppresses exactly as before for any
  // lanelet far from the goal.
  std::shared_ptr<std::optional<lanelet::Id>> last_exited_inverted_lanelet_id_;

  std::shared_ptr<ReverseLaneFollowParameters> parameters_;
  std::shared_ptr<autoware_utils::InterProcessPollingSubscriber<std_msgs::msg::Float64>>
    retrace_request_subscriber_;

  ReverseLaneFollowStatus status_;

  // Latest polled request: nullopt when no active request, else the requested distance in
  // meters (already validated against parameters_->min_retrace_distance_m and defaulted from
  // parameters_->default_retrace_distance_m when the request value is <= 0).
  std::optional<double> requested_distance_m_;

  // Identity of the last std_msgs::msg::Float64 seen from the polling subscriber. The subscriber
  // keeps returning the same cached message between publishes (Latest policy), so this is used to
  // tell "operator published a new request" apart from "nothing changed since last poll" --
  // request activation is edge-triggered on a new message, not level-triggered on its value.
  std_msgs::msg::Float64::ConstSharedPtr last_seen_request_msg_;
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_REVERSE_LANE_FOLLOW_MODULE__SCENE_HPP_
