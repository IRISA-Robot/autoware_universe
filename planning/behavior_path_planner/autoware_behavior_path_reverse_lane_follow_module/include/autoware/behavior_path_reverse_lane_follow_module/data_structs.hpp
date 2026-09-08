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

#ifndef AUTOWARE__BEHAVIOR_PATH_REVERSE_LANE_FOLLOW_MODULE__DATA_STRUCTS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_REVERSE_LANE_FOLLOW_MODULE__DATA_STRUCTS_HPP_

#include <lanelet2_core/primitives/Lanelet.h>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>

namespace autoware::behavior_path_planner
{
using autoware_internal_planning_msgs::msg::PathWithLaneId;

// Tunables for Phase 1 "retrace last N meters" scaffold. All declared with in-code defaults (see
// manager.cpp) -- once tuning-engineer wires config/reverse_lane_follow.param.yaml these are the
// keys under the `reverse_lane_follow.` namespace to expose.
struct ReverseLaneFollowParameters
{
  // Default distance to retrace when a request does not specify its own distance
  // (request value <= 0 falls back to this).
  double default_retrace_distance_m{10.0};

  // Minimum meaningful retrace request; requests below this are ignored.
  double min_retrace_distance_m{1.0};

  // Magnitude (m/s) of the commanded reverse speed. Sign is applied separately using the
  // `(is_back ? -1 : 1) * velocity` convention from autoware_freespace_planner's utils.cpp.
  double retrace_velocity_mps{1.0};

  // Ego-to-path-end distance below which the retrace is considered complete.
  double goal_reach_tolerance_m{0.5};

  // [BIDIR-BUG-FIX #5/#6] Magnitude (m/s^2) of the deceleration used by
  // reverse_lane_follow_utils::applyGoalDecelerationRamp() to shape a gradual pre-goal slowdown in
  // buildRouteReversedFollowPath()'s output, once the path has been truncated to the exact goal
  // pose (see truncatePathAtGoal()). Physics: v(s) = sqrt(2 * goal_stop_decel_mps2 *
  // distance_to_goal_m), applied backward from the terminal (already-zero-velocity) goal point so
  // the commanded velocity itself ramps down smoothly on approach, instead of relying entirely on
  // the downstream general-purpose velocity_smoother to invent braking out of a flat -abs(v)
  // profile that previously rode at cruise speed right up to the last waypoint.
  //
  // Deliberately conservative -- NOT reusing velocity_smoother's aggressive general-maneuvering
  // limits (normal.min_acc=-2.0 / limit.min_acc=-4.0, see
  // docs/research/aggressive-tuning-ruang-bawah-kantor.md): this is a much gentler, narrowly-scoped
  // "definitely stop at the goal" ramp, so it reliably provides real stopping margin regardless of
  // what the downstream smoother's general limits would otherwise allow. Picked to sit in the same
  // conservative ballpark as pid.param.yaml's smooth_stop_weak_acc=-0.3.
  double goal_stop_decel_mps2{0.3};

  // [BIDIR-BUG-FIX #7] (2026-09-07 continued-overshoot investigation) Maximum allowed waypoint
  // spacing (m) reverse_lane_follow_utils::densifyPathNearGoal() enforces within the ramp's own
  // braking-distance span of the goal, inserting interpolated points as needed, before
  // applyGoalDecelerationRamp() runs. Root cause this fixes: the ramp can only set velocities at
  // EXISTING waypoints -- if the native lanelet centerline spacing near the goal is coarse (e.g.
  // the ~2m gap live-confirmed in this investigation) relative to the ramp's required stopping
  // distance, the ramp has only one or two points to act on before the hard zero-velocity point,
  // producing an almost-cruise-speed command right up to a last-instant cliff-drop rather than a
  // real gradual slowdown -- regardless of the ramp formula itself being correct. 0.5 m gives the
  // ramp roughly 6-7 points to work with over the ~3.2 m braking distance implied by
  // retrace_velocity_mps=1.389 and goal_stop_decel_mps2=0.3 -- fine enough for a visibly smooth
  // profile without an excessive point count.
  double goal_decel_ramp_resample_interval_m{0.5};

  // [BIDIR-BUG-FIX #8] (2026-09-07 continued-overshoot investigation) Reactive, closed-loop safety
  // layer -- see reverse_lane_follow_utils::clampVelocityNearEgoWhenCloseToGoal()'s doc comment in
  // utils.hpp. Independent, defense-in-depth backstop against the open-loop path-shaping chain
  // above (truncatePathAtGoal()/densifyPathNearGoal()/applyGoalDecelerationRamp()): once the LIVE,
  // real-time straight-line distance to the goal (ReverseLaneFollowModule::
  // updateRouteReversedFollow()'s existing distance_to_goal_m, not recomputed) drops below this
  // threshold (and the goal is confirmed within the current follow window), path points near ego's
  // current position are force-clamped to at most reactive_goal_clamp_velocity_mps regardless of
  // what the open-loop chain computed this cycle. Deliberately requested by the user as a
  // backstop given enable_overshoot_emergency (control-side emergency-brake-on-overshoot) is
  // intentionally left `false` (considered dangerous) -- with that backstop disabled, this module's
  // own path-shaping is the only defense against a reverse-goal overshoot, so a second,
  // independent, closed-loop layer inside this module is warranted. Picked larger than
  // goal_reach_tolerance_m (1.0) so this layer is already active for some distance before ego would
  // ever be considered "at" the goal by that separate check.
  double reactive_goal_clamp_trigger_distance_m{2.0};

  // [BIDIR-BUG-FIX #8] Forced/clamped velocity magnitude (m/s) applied by
  // clampVelocityNearEgoWhenCloseToGoal() once triggered -- a conservative crawl speed, well below
  // retrace_velocity_mps (1.389), in the same conservative ballpark as this file's other
  // narrowly-scoped safety margins (goal_stop_decel_mps2=0.3, pid.param.yaml's
  // smooth_stop_weak_acc=-0.3).
  double reactive_goal_clamp_velocity_mps{0.5};

  // Bidirectional-driving support (Task E / §4d): forward/backward lookahead (meters) used when
  // this module activates because ego is on a route segment RouteHandler flags as reversed,
  // independent of any explicit retrace request. "forward" here means ahead of ego in the actual
  // direction of travel (i.e. along the inverted lanelet sequence), not map/centerline order.
  //
  // [BIDIR-QUALITY-FIX] These used to default to 30.0/5.0 -- a 35 m total sliding window centered
  // on ego, rebuilt from scratch every planning cycle by buildRouteReversedFollowPath(). Compare
  // to forward driving's utils::getReferencePath() (path_utils.cpp), which windows on
  // forward_path_length/backward_path_length -- 300.0/5.0(+10 extra margin) in
  // behavior_path_planner.param.yaml, i.e. ~315 m of lookahead. Downstream modules that run on
  // top of this module's output in slot1+ (static_obstacle_avoidance, lane_change, goal_planner
  // -- see scene_module_manager.param.yaml's slot0=reverse_lane_follow, slot2=avoidance/lane
  // change, slot3=goal_planner) can only see/plan shift-lines within whatever window this module
  // handed them. A 30 m forward window gives avoidance roughly 1/10th the runway it gets while
  // driving forward to detect obstacles early and build a smooth, gradual lateral shift --
  // directly reproducing "avoidance jelek"/"motion planning jelek" while reversing even though
  // the module chain itself composes correctly (confirmed: slot0's output IS the previous-slot
  // input for slot1..slot4, this is not a wholesale-replace-blocks-everything bug). Widened here
  // to give avoidance meaningfully more room while staying reverse-scoped (no change to forward
  // driving's own 300 m window, no change to avoidance/lane_change code). Still deliberately less
  // than forward's 300 m: reverse maneuvers are typically short (parking/backing out of a
  // dead-end), and a too-large window costs replanning stability (more of the window churns per
  // cycle since it recenters on ego every tick) for diminishing benefit. Revisit if reverse routes
  // routinely exceed ~100 m.
  double route_reversed_forward_distance_m{150.0};
  // [BIDIR-BUG-FIX #2] Bumped from 15.0. buildRouteReversedFollowPath() no longer gates
  // activation purely on ego's *current* lanelet being flagged inverted-in-route -- it instead
  // walks outward from ego's position using geometric heading continuity (see
  // isHeadingContinuousAcross() in utils.cpp) and only requires that *some* lanelet within that
  // continuous run is flagged inverted. As ego progresses forward through a multi-lanelet reverse
  // maneuver (e.g. taman map's 18(inv) -> 291(fwd) -> 75(fwd), a single continuous curve, not a
  // reversal), the lanelet that's actually flagged inverted can end up further and further behind
  // ego -- this distance is what bounds how far back that continuity walk is allowed to look to
  // (re)confirm "yes, we are still mid the same reverse episode that started back there." Too
  // small and the module incorrectly deactivates mid-maneuver once ego outruns the window (handing
  // off to normal forward-driving mode with a stale/wrong heading target -- the exact deadlock this
  // fix addresses); too large costs a bit of wasted lanelet-sequence/centerline work every cycle
  // for routes with no reverse segment nearby. 100 m comfortably covers this vehicle's short
  // parking/backing-out maneuvers; revisit if a route needs a longer one.
  double route_reversed_backward_distance_m{100.0};

  // [BIDIR-BUG-FIX #10] (2026-09-07 ARRIVED-state investigation) Speed magnitude (m/s) below which
  // ego is considered "genuinely at rest" by
  // reverse_lane_follow_utils::isEgoArrivedAndStoppedAtGoal() -- used by both
  // ReverseLaneFollowModule::canTransitSuccessState() (to correctly report success for the
  // route_reversed_path branch once ego is truly done, instead of the previous unconditional
  // `return false`) and updateRouteReversedFollow() (to decide when to stop re-deriving a fresh,
  // live-pose-sensitive goal-approach velocity profile and instead hold a fully zero-velocity
  // path).
  //
  // Deliberately NOT the same as autoware_motion_utils::VehicleStopCheckerBase's own hardcoded,
  // non-parameterized `squared_stop_velocity` bound (1mm/s) that mission_planner's
  // ArrivalChecker::is_arrived() gates on: that check is this module's DOWNSTREAM consumer, over a
  // continuous 1.0s window -- reusing that same ultra-tight bound here would recreate exactly the
  // chicken-and-egg deadlock this fix closes (this module's own continual re-planning was the thing
  // producing the residual velocity that kept tripping that 1mm/s bound in the first place, so
  // requiring ego to already be under 1mm/s before THIS module stops perturbing it could never
  // converge). Picked comfortably above the live-confirmed peak residual creep this bug produced
  // (~0.00105 m/s, see docs/research/reverse-lane-follow-goal-overshoot-fix.md's 2026-09-07
  // planning-engineer section) so this check reliably fires as soon as ego is genuinely
  // decelerating into its final rest, breaking the loop early and letting real vehicle dynamics
  // settle to a clean, unperturbed near-zero that isVehicleStopped()'s much stricter downstream
  // bound can then observe over its own full 1.0s window.
  double arrived_stop_velocity_mps{0.05};
};

// This module's own "am I currently retracing" status flag -- the reverse-lane-follow analogue of
// autoware_behavior_path_start_planner_module's `status_.driving_forward` /
// `isDrivingForward()` pattern.
struct ReverseLaneFollowStatus
{
  bool is_retracing{false};
  double requested_distance_m{0.0};
  PathWithLaneId retrace_path{};
  // Lanelet sequence the retrace_path was built from -- kept around so plan() can regenerate a
  // drivable area without re-querying RouteHandler every cycle.
  lanelet::ConstLanelets retrace_lanelets{};

  // Bidirectional-driving support (Task E / §4d): true when this module is active because ego is
  // on a route segment RouteHandler flags as reversed, rather than because of an explicit
  // retrace request. route_reversed_path/route_reversed_lanelets mirror retrace_path/
  // retrace_lanelets for that trigger.
  bool is_route_reversed_active{false};
  PathWithLaneId route_reversed_path{};
  lanelet::ConstLanelets route_reversed_lanelets{};
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_REVERSE_LANE_FOLLOW_MODULE__DATA_STRUCTS_HPP_
