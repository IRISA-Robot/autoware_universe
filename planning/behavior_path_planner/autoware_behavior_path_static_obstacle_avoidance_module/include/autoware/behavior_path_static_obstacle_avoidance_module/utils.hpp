// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__UTILS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__UTILS_HPP_

#include "autoware/behavior_path_planner_common/data_manager.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp"
#include "autoware/behavior_path_static_obstacle_avoidance_module/data_structs.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner::helper::static_obstacle_avoidance
{
class AvoidanceHelper;
}  // namespace autoware::behavior_path_planner::helper::static_obstacle_avoidance

namespace autoware::behavior_path_planner::utils::static_obstacle_avoidance
{

using autoware::behavior_path_planner::PlannerData;
using autoware::behavior_path_planner::utils::path_safety_checker::ExtendedPredictedObject;
using autoware::behavior_path_planner::utils::path_safety_checker::
  PoseWithVelocityAndPolygonStamped;
using autoware::behavior_path_planner::utils::path_safety_checker::PoseWithVelocityStamped;
using autoware::behavior_path_planner::utils::path_safety_checker::PredictedPathWithPolygon;

static constexpr const char * logger_namespace =
  "planning.scenario_planning.lane_driving.behavior_planning.behavior_path_planner.static_obstacle_"
  "avoidance.utils";
/**
 * @brief check object offset direction.
 * @param object data.
 * @return if the object is on right side of ego path, return true.
 */
bool isOnRight(const ObjectData & obj);

/**
 * @brief calculate shift length from centerline of current lane.
 * @param object offset direction.
 * @param distance between object polygon and centerline of current lane. (signed)
 * @param margin distance between ego and object.
 * @return necessary shift length. (signed)
 */
double calcShiftLength(
  const bool & is_object_on_right, const double & overhang_dist, const double & avoid_margin);

bool isWithinLanes(
  const std::optional<lanelet::ConstLanelet> & closest_lanelet,
  const std::shared_ptr<const PlannerData> & planner_data);

/**
 * @brief check if the ego has to shift driving position.
 * @param if object is on right side of ego path.
 * @param ego shift length.
 * @return necessity of shifting.
 */
bool isShiftNecessary(const bool & is_object_on_right, const double & shift_length);

/**
 * @brief check if the ego has to avoid object with the trajectory whose shift direction is same as
 * object offset.
 * @param object offset direction.
 * @param ego shift length.
 * @return if the direction of shift and object offset, return true.
 */
bool isSameDirectionShift(const bool & is_object_on_right, const double & shift_length);

size_t findPathIndexFromArclength(
  const std::vector<double> & path_arclength_arr, const double target_arc);

ShiftedPath toShiftedPath(const PathWithLaneId & path);

ShiftLineArray toShiftLineArray(const AvoidLineArray & avoid_points);

std::vector<UUID> concatParentIds(const std::vector<UUID> & ids1, const std::vector<UUID> & ids2);

std::vector<UUID> calcParentIds(const AvoidLineArray & lines1, const AvoidLine & lines2);

double lerpShiftLengthOnArc(double arc, const AvoidLine & al);

/**
 * @brief calculate distance between ego and object. object length along with the path is calculated
 * as well.
 * @param current path.
 * @param ego position.
 * @param object data.
 */
void fillLongitudinalAndLengthByClosestEnvelopeFootprint(
  const PathWithLaneId & path, const Point & ego_pos, ObjectData & obj);

/**
 * @brief calculate overhang distance for all of the envelope polygon outer points.
 * @param object data.
 * @param current path.
 * @param baselink_to_vehicle_front to vehicle front.
 * @param baselink_to_vehicle_rear to vehicle rear.
 * @return first: overhang distance, second: outer point. this vector is sorted by overhang
 * distance.
 */
std::vector<std::pair<double, Point>> calcEnvelopeOverhangDistance(
  const ObjectData & object_data, const PathWithLaneId & path, double baselink_to_vehicle_front,
  double baselink_to_vehicle_rear);

void setEndData(
  AvoidLine & al, const double length, const geometry_msgs::msg::Pose & end, const size_t end_idx,
  const double end_dist);

void setStartData(
  AvoidLine & al, const double start_shift_length, const geometry_msgs::msg::Pose & start,
  const size_t start_idx, const double start_dist);

/**
 * @brief create envelope polygon which is parallel to current path.
 * @param object polygon.
 * @param closest point pose of the current path.
 * @param buffer.
 * @return envelope polygon.
 */
Polygon2d createEnvelopePolygon(
  const Polygon2d & object_polygon, const Pose & closest_pose, const double envelope_buffer);

/**
 * @brief create envelope polygon which is parallel to current path.
 * @param object data.
 * @param closest point pose of the current path.
 * @param buffer.
 * @return envelope polygon.
 */
Polygon2d createEnvelopePolygon(
  const ObjectData & object_data, const Pose & closest_pose, const double envelope_buffer);

/**
 * @brief create data structs which are used in clipping drivable area process.
 * @param objects.
 * @param avoidance module parameters.
 * @param ego vehicle width.
 * @return struct which includes expanded polygon.
 */
std::vector<DrivableAreaInfo::Obstacle> generateObstaclePolygonsForDrivableArea(
  const ObjectDataArray & objects, const std::shared_ptr<AvoidanceParameters> & parameters,
  const double vehicle_width);

lanelet::ConstLanelets getAdjacentLane(
  const lanelet::ConstLanelet & current_lane,
  const std::shared_ptr<const PlannerData> & planner_data,
  const std::shared_ptr<AvoidanceParameters> & parameters, const bool is_right_shift);

lanelet::ConstLanelets getCurrentLanesFromPath(
  const PathWithLaneId & path, const std::shared_ptr<const PlannerData> & planner_data);

lanelet::ConstLanelets getExtendLanes(
  const lanelet::ConstLanelets & lanelets, const Pose & ego_pose,
  const std::shared_ptr<const PlannerData> & planner_data);

/**
 * @brief insert target stop/decel point.
 * @param ego current position.
 * @param distance between ego and stop/decel position.
 * @param target velocity.
 * @param target path.
 * @param insert point.
 */
void insertDecelPoint(
  const Point & p_src, const double offset, const double velocity, PathWithLaneId & path,
  PoseWithDetailOpt & p_out);

/**
 * @brief update envelope polygon based on object position reliability.
 * @param current detected object.
 * @param previous stopped objects.
 * @param base pose to create envelope polygon.
 * @param threshold parameters.
 */
void fillObjectEnvelopePolygon(
  ObjectData & object_data, const ObjectDataArray & registered_objects, const Pose & closest_pose,
  const std::shared_ptr<AvoidanceParameters> & parameters);

/**
 * @brief fill stopping duration.
 * @param current detected object.
 * @param previous stopped objects.
 * @param threshold parameters.
 */
void fillObjectMovingTime(
  ObjectData & object_data, ObjectDataArray & stopped_objects,
  const std::shared_ptr<AvoidanceParameters> & parameters);

/**
 * @brief update classification unstable objects.
 * @param current detected object.
 * @param unknown type object first seen time map.
 * @param unstable classification time.
 */
void updateClassificationUnstableObjects(
  ObjectData & object_data,
  std::unordered_map<std::string, rclcpp::Time> & unknown_type_object_first_seen_time_map,
  const double unstable_classification_time);

/**
 * @brief check whether ego has to avoid the objects.
 * @param current detected object.
 * @param previous stopped objects.
 * @param threshold parameters.
 */
void fillAvoidanceNecessity(
  ObjectData & object_data, const ObjectDataArray & registered_objects, const double vehicle_width,
  const std::shared_ptr<AvoidanceParameters> & parameters);

void fillObjectStoppableJudge(
  ObjectData & object_data, const ObjectDataArray & registered_objects,
  const double feasible_stop_distance, const std::shared_ptr<AvoidanceParameters> & parameters);

void fillObjectAvoidableByDesiredShiftLength(
  ObjectData & object_data, const ObjectDataArray & previous_target_objects);

/**
 * @brief Persist the "avoidance committed" sticky flag across cycles by matching object_id
 * against previous_target_objects. Once an object was found avoidable (is_avoidable == true) in
 * ANY previous cycle, and its lateral avoid_margin is still available (non-nullopt) this cycle,
 * it stays "committed" so that shift_line_generator's purely longitudinal-distance/timing gates
 * can be bypassed for it going forward -- this prevents an already-in-progress avoidance shift
 * from being silently reverted back to centerline just because remaining longitudinal distance
 * shrank as ego approached. If avoid_margin is lost (no lateral room), the commitment is dropped:
 * the lateral room-availability check is never bypassed.
 * @param object_data current-cycle object data (object_data.avoid_margin must already be filled).
 * @param previous_target_objects previous cycle's target object list.
 */
void fillObjectAvoidanceCommitted(
  ObjectData & object_data, const ObjectDataArray & previous_target_objects);

/**
 * @brief [REV-F2 debounce, 2026-09-03] carry forward object_data.hard_margin_infeasible_streak
 * from the previous cycle's matching object (by object_id), defaulting to 0 if not found (first
 * time seen, or the previous cycle's hard-margin check passed / was never checked). This seeds
 * the counter that ShiftLineGenerator::computeFeasibleShiftProfile() increments/resets on its own
 * hard lateral-margin feasibility check, so a single noisy cycle at the boundary doesn't
 * immediately null out new_shift_line for an already-committed object. See
 * docs/research/avoidance-test-campaign.md, "Session 4 -- REV-F2".
 * @param object_data current-cycle object data.
 * @param previous_target_objects previous cycle's target object list.
 */
void fillObjectHardMarginDebounce(
  ObjectData & object_data, const ObjectDataArray & previous_target_objects);

/**
 * @brief [ASYM-HYSTERESIS 2026-09-04] carry forward object_data.shift_grow_streak /
 * shift_shrink_streak from the previous cycle's matching object (by object_id), defaulting both to
 * 0 if not found. Seeds the counters that ShiftLineGenerator::generateAvoidOutline() increments/
 * resets when deciding whether to grow or shrink an already-approved object's shift length (see
 * AvoidanceParameters::shift_hysteresis_in_cycles / shift_hysteresis_out_cycles).
 * @param object_data current-cycle object data.
 * @param previous_target_objects previous cycle's target object list.
 */
void fillObjectShiftHysteresis(
  ObjectData & object_data, const ObjectDataArray & previous_target_objects);

void updateClipObject(ObjectDataArray & clip_objects, AvoidancePlanningData & data);

/**
 * @brief compensate lost objects until a certain time elapses.
 * @param avoidance planning data.
 * @param previous stopped object.
 * @param current time.
 * @param avoidance parameters which includes duration of compensation.
 */
void compensateLostTargetObjects(
  AvoidancePlanningData & data, const ObjectDataArray & stored_objects,
  const std::shared_ptr<const PlannerData> & planner_data);

/**
 * @brief check whether the remaining geometric room to attempt an avoidance shift has already
 * shrunk to (or below) the minimum distance required to act, for an UNKNOWN-classified object
 * whose classification is still within the "unstable" observation window.
 * @param object object data. `object.longitudinal` must already be filled in.
 * @param avoid_margin lateral avoidance margin for the object, if avoidable at all.
 * @param helper avoidance helper, used for the prepare/front-constant/min-avoidance distance
 * calculation.
 * @return true if there is no longer a safety benefit in waiting out the rest of the
 * classification-instability window (i.e. the classification-stability safeguard can be
 * short-circuited without giving up genuine avoidance opportunity).
 */
bool isAvoidanceOpportunityRunningOut(
  const ObjectData & object, const std::optional<double> & avoid_margin,
  const std::shared_ptr<helper::static_obstacle_avoidance::AvoidanceHelper> & helper);

void filterTargetObjects(
  ObjectDataArray & objects, AvoidancePlanningData & data, const double forward_detection_range,
  const std::shared_ptr<const PlannerData> & planner_data,
  const std::shared_ptr<AvoidanceParameters> & parameters,
  const std::shared_ptr<helper::static_obstacle_avoidance::AvoidanceHelper> & helper);

void updateRoadShoulderDistance(
  AvoidancePlanningData & data, const std::shared_ptr<const PlannerData> & planner_data,
  const std::shared_ptr<AvoidanceParameters> & parameters);

void fillAdditionalInfoFromPoint(const AvoidancePlanningData & data, AvoidLineArray & lines);

void fillAdditionalInfoFromLongitudinal(const AvoidancePlanningData & data, AvoidLine & line);

void fillAdditionalInfoFromLongitudinal(
  const AvoidancePlanningData & data, AvoidOutlines & outlines);

void fillAdditionalInfoFromLongitudinal(const AvoidancePlanningData & data, AvoidLineArray & lines);

AvoidLine fillAdditionalInfo(const AvoidancePlanningData & data, const AvoidLine & line);

AvoidLineArray combineRawShiftLinesWithUniqueCheck(
  const AvoidLineArray & base_lines, const AvoidLineArray & added_lines);

std::vector<ExtendedPredictedObject> getSafetyCheckTargetObjects(
  const AvoidancePlanningData & data, const std::shared_ptr<const PlannerData> & planner_data,
  const std::shared_ptr<AvoidanceParameters> & parameters, const bool has_left_shift,
  const bool has_right_shift, DebugData & debug, const bool ego_already_shifted = false);

std::pair<PredictedObjects, PredictedObjects> separateObjectsByPath(
  const PathWithLaneId & reference_path, const PathWithLaneId & spline_path,
  const std::shared_ptr<const PlannerData> & planner_data, const AvoidancePlanningData & data,
  const std::shared_ptr<AvoidanceParameters> & parameters,
  const double object_check_forward_distance, DebugData & debug);

DrivableLanes generateNotExpandedDrivableLanes(const lanelet::ConstLanelet & lanelet);

DrivableLanes generateExpandedDrivableLanes(
  const lanelet::ConstLanelet & lanelet, const std::shared_ptr<const PlannerData> & planner_data,
  const std::string & use_lane_type);

double calcDistanceToReturnDeadLine(
  const lanelet::ConstLanelets & lanelets, const PathWithLaneId & path,
  const std::shared_ptr<const PlannerData> & planner_data,
  const std::shared_ptr<AvoidanceParameters> & parameters,
  const std::optional<double> distance_to_red_traffic, const bool is_allowed_goal_modification);

double calcDistanceToAvoidStartLine(
  const lanelet::ConstLanelets & lanelets, const std::shared_ptr<AvoidanceParameters> & parameters,
  const std::optional<double> distance_to_red_traffic);

/**
 * @brief calculate error eclipse radius based on object pose covariance.
 * @param pose with covariance.
 * @return error eclipse long radius.
 */
double calcErrorEclipseLongRadius(const PoseWithCovariance & pose);

void updateStoredObjects(
  ObjectDataArray & stored_objects, const ObjectDataArray & current_objects,
  const rclcpp::Time & now, const std::shared_ptr<AvoidanceParameters> & parameters);

/**
 * @brief Decide whether a stale registered shift line (held because a fresh candidate was
 * rejected by isSafePath() while canYieldManeuver() also refused a yield) should be forcibly
 * re-armed to a freshly-computed, larger shift instead of being held forever.
 *
 * Background (2026-09-04 hold-forever deadlock fix): holding a rejected candidate only stops the
 * module from COMMITTING a new unsafe shift -- it does nothing about a shift line that was
 * already registered on a previous, then-safe cycle. If that stale value keeps providing less
 * clearance than what is now genuinely required for the same still-unsafe object, and
 * canYieldManeuver() keeps refusing (e.g. avoidance is already mid-maneuver), the hold can
 * deadlock forever at an insufficient value. This function allows the hold to be bypassed and the
 * larger value re-armed, but ONLY when it is safe to do so (ego near-stationary, so no sudden
 * lateral jerk) and ONLY to INCREASE clearance on the same side -- never to decrease it.
 *
 * @param registered_end_shift_length end shift length of the currently path_shifter_-registered
 * main shift line.
 * @param desired_end_shift_length end shift length of the freshly-computed (still rejected as
 * unsafe) candidate shift line. Pass 0.0 with has_new_candidate=false when there is no candidate.
 * @param has_new_candidate whether a fresh candidate shift line exists this cycle.
 * @param ego_speed current ego speed magnitude [m/s].
 * @param ego_speed_threshold "safely stationary" speed gate [m/s].
 * @param clearance_margin minimum extra clearance [m] required before treating the candidate as
 * "more" than the registered value, to avoid re-arming on noise-level differences.
 * @return true if the hold should be bypassed and the registered shift line re-armed to the
 * larger, freshly-computed value.
 */
bool shouldRearmHeldShiftLine(
  const double registered_end_shift_length, const double desired_end_shift_length,
  const bool has_new_candidate, const double ego_speed, const double ego_speed_threshold,
  const double clearance_margin);

/**
 * @brief [BUG-A fix 2026-09-04] Decide whether a close-range avoid shift-line candidate whose raw
 * end_longitudinal has collapsed to at or below start_longitudinal (object very close --
 * "object.longitudinal - constant_distance" going non-positive) can be rescued, instead of being
 * silently treated as geometrically invalid and dropped.
 *
 * Background: ShiftLineGenerator::generateAvoidOutline() used to build end_longitudinal as a raw
 * "object longitudinal - constant distance" arithmetic result with no floor. As ego closes in on
 * an object, this naturally goes toward zero and then negative, at which point
 * start_longitudinal < end_longitudinal no longer holds and the whole candidate is discarded for
 * any not-yet-approved object -- exactly the case that most urgently needs a NEW, larger shift.
 * computeFeasibleShiftProfile() already proves a jerk-feasible shift of this magnitude fits in
 * whatever longitudinal room remains; this function re-anchors end_longitudinal to at least
 * start_longitudinal + min_transition_distance so that proof is actually used to build a valid
 * (steeper) shift line, rather than being discarded by degenerate arithmetic.
 *
 * @param start_longitudinal the already-valid (> 0) start point of the shift line.
 * @param min_transition_distance minimum jerk-feasible longitudinal transition distance for this
 * shift's magnitude (see AvoidanceHelper::getMinAvoidanceDistance()).
 * @param object_longitudinal the object's longitudinal distance from ego.
 * @param approach_buffer small buffer [m] to keep the transition from finishing at/after the
 * object itself.
 * @return the rescued end_longitudinal if the minimum transition still fits before reaching the
 * object; std::nullopt if even the minimum transition cannot fit -- a genuine kinematic
 * infeasibility, not an arithmetic accident, and the caller should treat this as a real rejection.
 */
std::optional<double> rescueCloseRangeShiftLineEnd(
  const double start_longitudinal, const double min_transition_distance,
  const double object_longitudinal, const double approach_buffer);

/**
 * @brief [ASYM-HYSTERESIS 2026-09-04] Apply asymmetric hysteresis to the decision of whether to
 * grow or shrink an already-approved object's shift length: growing (more clearance/more urgent)
 * is let through within `hysteresis_in_cycles` consecutive cycles (default configuration is 1 ==
 * immediate, since this is the safety-critical direction), while shrinking (less clearance)
 * requires `hysteresis_out_cycles` consecutive cycles first, to avoid flicker-driven premature
 * collapse of an already-committed shift (the same failure family as REV-F2 / HOLD-REARM /
 * existsUnclearedAvoidanceObjectAhead).
 *
 * @param desired_end_shift_length freshly-computed end shift length for this cycle.
 * @param registered_shift_length shift length currently registered at the object's position.
 * @param clearance_margin [m] ignore noise-level differences between desired and registered.
 * @param hysteresis_in_cycles consecutive cycles required before a grow is let through.
 * @param hysteresis_out_cycles consecutive cycles required before a shrink is let through.
 * @param grow_streak in/out: consecutive-grow-cycle counter (persisted by the caller across
 * cycles). Reset to 0 whenever the outcome is not a grow.
 * @param shrink_streak in/out: consecutive-shrink-cycle counter (persisted by the caller across
 * cycles). Reset to 0 whenever the outcome is not a shrink.
 * @return the shift length to actually use this cycle: either desired_end_shift_length (grow/shrink
 * let through, or no change) or registered_shift_length (grow/shrink held back by hysteresis).
 */
double applyAsymmetricShiftHysteresis(
  const double desired_end_shift_length, const double registered_shift_length,
  const double clearance_margin, const int hysteresis_in_cycles, const int hysteresis_out_cycles,
  int & grow_streak, int & shrink_streak);

/**
 * @brief [MULTI-OBJ-HOLD fix 2026-09-04] Decide whether the return-to-centerline shift must be
 * suppressed because a sequential-avoidance target object is still ahead of ego.
 *
 * ROOT CAUSE this guards against: addReturnShiftLine() previously only checked for *unavoidable*
 * objects still ahead before adding a return-to-center shift. In a sequential multi-object
 * scenario (object 1 just cleared, object 2 close behind it along the route), object 2 is
 * perfectly avoidable -- but if its own avoidance candidate hasn't been generated yet THIS
 * cycle (detection-timing lag, or generation-order lag inside the same planning cycle), the
 * candidate shift-line list is empty for it and the return shift would start collapsing object
 * 1's shift back to centerline immediately, putting ego on a near-collision course with object 2
 * for the few cycles until object 2's own avoidance candidate catches up.
 *
 * This is a per-cycle recomputation from the live target_objects list (NOT a sticky/latched
 * flag): once ego has genuinely passed every currently-detected object in the stretch, the very
 * next cycle sees no object ahead and the normal return-to-lane proceeds. This cannot become a
 * new permanent-stuck state the way a latched hold could.
 *
 * @param target_objects the module's current full list of avoidance target objects (avoidable or
 * not), each with `longitudinal` = Frenet longitudinal distance from ego (positive = ahead).
 * @return true if at least one target object is still ahead of ego (return shift must be held).
 */
bool existsUnclearedAvoidanceObjectAhead(const ObjectDataArray & target_objects);

}  // namespace autoware::behavior_path_planner::utils::static_obstacle_avoidance

#endif  // AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__UTILS_HPP_
