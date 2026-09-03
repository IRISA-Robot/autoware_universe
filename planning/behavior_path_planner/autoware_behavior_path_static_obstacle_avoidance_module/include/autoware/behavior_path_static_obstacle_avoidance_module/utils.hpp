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

}  // namespace autoware::behavior_path_planner::utils::static_obstacle_avoidance

#endif  // AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__UTILS_HPP_
