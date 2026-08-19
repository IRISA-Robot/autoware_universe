// Copyright 2022 TIER IV, Inc.
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

#include "../src/utils.cpp"  // NOLINT
#include "autoware/behavior_path_static_obstacle_avoidance_module/data_structs.hpp"
#include "autoware/behavior_path_static_obstacle_avoidance_module/helper.hpp"
#include "autoware/behavior_path_static_obstacle_avoidance_module/shift_line_generator.hpp"
#include "autoware/behavior_path_static_obstacle_avoidance_module/type_alias.hpp"
#include "autoware/behavior_path_static_obstacle_avoidance_module/utils.hpp"
#include "autoware_test_utils/autoware_test_utils.hpp"
#include "autoware_utils/math/unit_conversion.hpp"

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <limits>
#include <memory>

namespace autoware::behavior_path_planner::utils::static_obstacle_avoidance
{

using autoware::behavior_path_planner::AvoidanceParameters;
using autoware::behavior_path_planner::ObjectData;
using autoware::route_handler::Direction;
using autoware_utils::create_point;
using autoware_utils::create_quaternion_from_rpy;
using autoware_utils::create_vector3;
using autoware_utils::deg2rad;
using autoware_utils::generate_uuid;

constexpr double epsilon = 1e-6;

ObjectData create_test_object(
  const Pose & pose, const Vector3 & velocity, const Shape & shape, const uint8_t type)
{
  ObjectData object_data;
  object_data.object.shape = shape;
  object_data.object.kinematics.initial_pose_with_covariance.pose = pose;
  object_data.object.kinematics.initial_twist_with_covariance.twist.linear = velocity;
  object_data.object.classification.emplace_back(
    autoware_perception_msgs::build<ObjectClassification>().label(type).probability(1.0));
  return object_data;
}

auto get_planner_data() -> std::shared_ptr<PlannerData>
{
  PlannerData planner_data;

  // set odometry.
  Odometry odometry;
  odometry.pose.pose = geometry_msgs::build<geometry_msgs::msg::Pose>()
                         .position(create_point(0.0, 0.0, 0.0))
                         .orientation(create_quaternion_from_rpy(0.0, 0.0, 0.0));
  odometry.twist.twist.linear = create_vector3(0.0, 0.0, 0.0);

  planner_data.self_odometry = std::make_shared<Odometry>(odometry);

  // set vehicle info.
  planner_data.parameters.vehicle_width = 2.0;

  return std::make_shared<PlannerData>(planner_data);
}

auto get_parameters() -> std::shared_ptr<AvoidanceParameters>
{
  AvoidanceParameters parameters;

  parameters.object_check_backward_distance = 1.0;
  parameters.object_check_goal_distance = 2.0;
  parameters.object_check_yaw_deviation = deg2rad(20);
  parameters.object_check_backward_distance = 1.0;
  parameters.object_check_goal_distance = 2.0;
  parameters.object_check_shiftable_ratio = 0.6;
  parameters.object_check_min_road_shoulder_width = 0.5;
  parameters.object_last_seen_threshold = 0.2;
  parameters.threshold_distance_object_is_on_center = 1.0;
  parameters.hard_drivable_bound_margin = 0.1;
  parameters.soft_drivable_bound_margin = 0.5;
  parameters.lateral_execution_threshold = 0.5;

  parameters.min_prepare_distance = 1.5;
  parameters.max_prepare_time = 3.0;
  parameters.nominal_avoidance_speed = 8.0;
  parameters.velocity_map = {1.0, 3.0, 10.0};
  parameters.avoid_lateral_min_jerk_map = {0.1, 1.0, 10.0};
  parameters.return_lateral_min_jerk_map = {0.1, 1.0, 10.0};
  parameters.lateral_max_jerk_map = {0.4, 1.5, 15.0};
  parameters.lateral_max_accel_map = {0.7, 0.8, 0.9};

  const auto get_truck_param = []() {
    ObjectParameter param{};
    param.is_avoidance_target = true;
    param.is_safety_check_target = true;
    param.moving_time_threshold = 0.5;
    param.moving_speed_threshold = 1.0;
    param.lateral_soft_margin = 0.7;
    param.lateral_hard_margin = 0.2;
    param.lateral_hard_margin_for_parked_vehicle = 0.7;
    param.envelope_buffer_margin = 0.0;
    param.th_error_eclipse_long_radius = 2.0;
    return param;
  };

  parameters.object_parameters.emplace(ObjectClassification::TRUCK, get_truck_param());

  const auto get_unknown_param = []() {
    ObjectParameter param{};
    param.is_avoidance_target = false;
    param.is_safety_check_target = false;
    param.moving_time_threshold = 0.5;
    param.moving_speed_threshold = 1.0;
    param.lateral_soft_margin = 0.7;
    param.lateral_hard_margin = 0.2;
    param.lateral_hard_margin_for_parked_vehicle = 0.7;
    param.envelope_buffer_margin = 0.0;
    param.th_error_eclipse_long_radius = 2.0;
    return param;
  };

  parameters.object_parameters.emplace(ObjectClassification::UNKNOWN, get_unknown_param());

  return std::make_shared<AvoidanceParameters>(parameters);
}

TEST(TestUtils, isMovingObject)
{
  {
    ObjectData object_data;
    object_data.move_time = 0.49;
    object_data.object.classification.emplace_back(
      autoware_perception_msgs::build<ObjectClassification>()
        .label(ObjectClassification::TRUCK)
        .probability(1.0));

    EXPECT_FALSE(filtering_utils::isMovingObject(object_data, get_parameters()));
  }

  {
    ObjectData object_data;
    object_data.move_time = 0.51;
    object_data.object.classification.emplace_back(
      autoware_perception_msgs::build<ObjectClassification>()
        .label(ObjectClassification::TRUCK)
        .probability(1.0));

    EXPECT_TRUE(filtering_utils::isMovingObject(object_data, get_parameters()));
  }
}

TEST(TestUtils, getObjectBehavior)
{
  const auto parameters = get_parameters();

  lanelet::LineString3d left_bound;
  lanelet::LineString3d right_bound;

  left_bound.push_back(lanelet::Point3d{lanelet::InvalId, -1, -1});
  left_bound.push_back(lanelet::Point3d{lanelet::InvalId, 0, -1});
  left_bound.push_back(lanelet::Point3d{lanelet::InvalId, 1, -1});
  right_bound.push_back(lanelet::Point3d{lanelet::InvalId, -1, 1});
  right_bound.push_back(lanelet::Point3d{lanelet::InvalId, 0, 1});
  right_bound.push_back(lanelet::Point3d{lanelet::InvalId, 1, 1});

  lanelet::Lanelet lanelet{lanelet::InvalId, left_bound, right_bound};

  lanelet::LineString3d centerline;
  centerline.push_back(lanelet::Point3d{lanelet::InvalId, -1, 0});
  centerline.push_back(lanelet::Point3d{lanelet::InvalId, 0, 0});
  centerline.push_back(lanelet::Point3d{lanelet::InvalId, 1, 0});

  lanelet.setCenterline(centerline);

  {
    ObjectData object_data;
    object_data.overhang_lanelet = lanelet;
    object_data.direction = Direction::LEFT;
    object_data.object.kinematics.initial_pose_with_covariance.pose =
      geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(create_point(0.0, 0.1, 0.0))
        .orientation(create_quaternion_from_rpy(0.0, 0.0, 0.0));

    EXPECT_EQ(
      filtering_utils::getObjectBehavior(object_data, parameters), ObjectData::Behavior::NONE);
  }

  {
    ObjectData object_data;
    object_data.overhang_lanelet = lanelet;
    object_data.direction = Direction::RIGHT;
    object_data.object.kinematics.initial_pose_with_covariance.pose =
      geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(create_point(0.0, -0.1, 0.0))
        .orientation(create_quaternion_from_rpy(0.0, 0.0, deg2rad(170)));

    EXPECT_EQ(
      filtering_utils::getObjectBehavior(object_data, parameters), ObjectData::Behavior::NONE);
  }

  {
    ObjectData object_data;
    object_data.overhang_lanelet = lanelet;
    object_data.direction = Direction::LEFT;
    object_data.object.kinematics.initial_pose_with_covariance.pose =
      geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(create_point(0.0, 0.1, 0.0))
        .orientation(create_quaternion_from_rpy(0.0, 0.0, deg2rad(30)));

    EXPECT_EQ(
      filtering_utils::getObjectBehavior(object_data, parameters), ObjectData::Behavior::DEVIATING);
  }

  {
    ObjectData object_data;
    object_data.overhang_lanelet = lanelet;
    object_data.direction = Direction::RIGHT;
    object_data.object.kinematics.initial_pose_with_covariance.pose =
      geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(create_point(0.0, -0.1, 0.0))
        .orientation(create_quaternion_from_rpy(0.0, 0.0, deg2rad(30)));

    EXPECT_EQ(
      filtering_utils::getObjectBehavior(object_data, parameters), ObjectData::Behavior::MERGING);
  }

  {
    ObjectData object_data;
    object_data.overhang_lanelet = lanelet;
    object_data.direction = Direction::LEFT;
    object_data.object.kinematics.initial_pose_with_covariance.pose =
      geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(create_point(0.0, 0.1, 0.0))
        .orientation(create_quaternion_from_rpy(0.0, 0.0, deg2rad(-150)));

    EXPECT_EQ(
      filtering_utils::getObjectBehavior(object_data, parameters), ObjectData::Behavior::DEVIATING);
  }

  {
    ObjectData object_data;
    object_data.overhang_lanelet = lanelet;
    object_data.direction = Direction::RIGHT;
    object_data.object.kinematics.initial_pose_with_covariance.pose =
      geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(create_point(0.0, -0.1, 0.0))
        .orientation(create_quaternion_from_rpy(0.0, 0.0, deg2rad(-150)));

    EXPECT_EQ(
      filtering_utils::getObjectBehavior(object_data, parameters), ObjectData::Behavior::MERGING);
  }

  {
    ObjectData object_data;
    object_data.overhang_lanelet = lanelet;
    object_data.direction = Direction::LEFT;
    object_data.object.kinematics.initial_pose_with_covariance.pose =
      geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(create_point(0.0, 0.1, 0.0))
        .orientation(create_quaternion_from_rpy(0.0, 0.0, deg2rad(-30)));

    EXPECT_EQ(
      filtering_utils::getObjectBehavior(object_data, parameters), ObjectData::Behavior::MERGING);
  }

  {
    ObjectData object_data;
    object_data.overhang_lanelet = lanelet;
    object_data.direction = Direction::RIGHT;
    object_data.object.kinematics.initial_pose_with_covariance.pose =
      geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(create_point(0.0, -0.1, 0.0))
        .orientation(create_quaternion_from_rpy(0.0, 0.0, deg2rad(-30)));

    EXPECT_EQ(
      filtering_utils::getObjectBehavior(object_data, parameters), ObjectData::Behavior::DEVIATING);
  }

  {
    ObjectData object_data;
    object_data.overhang_lanelet = lanelet;
    object_data.direction = Direction::LEFT;
    object_data.object.kinematics.initial_pose_with_covariance.pose =
      geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(create_point(0.0, 0.1, 0.0))
        .orientation(create_quaternion_from_rpy(0.0, 0.0, deg2rad(150)));

    EXPECT_EQ(
      filtering_utils::getObjectBehavior(object_data, parameters), ObjectData::Behavior::MERGING);
  }

  {
    ObjectData object_data;
    object_data.overhang_lanelet = lanelet;
    object_data.direction = Direction::RIGHT;
    object_data.object.kinematics.initial_pose_with_covariance.pose =
      geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(create_point(0.0, -0.1, 0.0))
        .orientation(create_quaternion_from_rpy(0.0, 0.0, deg2rad(150)));

    EXPECT_EQ(
      filtering_utils::getObjectBehavior(object_data, parameters), ObjectData::Behavior::DEVIATING);
  }
}

TEST(TestUtils, isNoNeedAvoidanceBehavior)
{
  const auto parameters = get_parameters();

  const auto path = autoware::test_utils::generateTrajectory<PathWithLaneId>(
    50L, 1.0, 20.0, 0.0, 0.0, std::numeric_limits<size_t>::max());

  const auto object_pose = geometry_msgs::build<geometry_msgs::msg::Pose>()
                             .position(create_point(0.0, 0.0, 0.0))
                             .orientation(create_quaternion_from_rpy(0.0, 0.0, deg2rad(45)));

  const auto nearest_path_pose =
    path.points.at(autoware::motion_utils::findNearestIndex(path.points, object_pose.position))
      .point.pose;

  const auto shape = autoware_perception_msgs::build<Shape>()
                       .type(Shape::BOUNDING_BOX)
                       .footprint(geometry_msgs::msg::Polygon{})
                       .dimensions(create_vector3(2.8284271247461901, 1.41421356237309505, 2.0));

  auto object_data = create_test_object(
    object_pose, create_vector3(0.0, 0.0, 0.0), shape, ObjectClassification::CAR);

  // object is NOT avoidable. but there is possibility that the ego has to avoid it.
  {
    object_data.avoid_margin = std::nullopt;

    EXPECT_FALSE(filtering_utils::isNoNeedAvoidanceBehavior(object_data, parameters));
  }

  // don't have to avoid.
  {
    object_data.object.kinematics.initial_pose_with_covariance.pose =
      geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(create_point(2.5, 3.6, 0.0))
        .orientation(create_quaternion_from_rpy(0.0, 0.0, deg2rad(45)));
    object_data.direction = Direction::LEFT;
    object_data.avoid_margin = 2.0;
    object_data.envelope_poly = createEnvelopePolygon(object_data, nearest_path_pose, 0.0);
    object_data.overhang_points = calcEnvelopeOverhangDistance(object_data, path, 0.0, 0.0);

    EXPECT_TRUE(filtering_utils::isNoNeedAvoidanceBehavior(object_data, parameters));
    EXPECT_EQ(object_data.info, ObjectInfo::ENOUGH_LATERAL_DISTANCE);
  }

  // smaller than execution threshold.
  {
    object_data.object.kinematics.initial_pose_with_covariance.pose =
      geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(create_point(2.5, 3.4, 0.0))
        .orientation(create_quaternion_from_rpy(0.0, 0.0, deg2rad(45)));
    object_data.direction = Direction::LEFT;
    object_data.avoid_margin = 2.0;
    object_data.envelope_poly = createEnvelopePolygon(object_data, nearest_path_pose, 0.0);
    object_data.overhang_points = calcEnvelopeOverhangDistance(object_data, path, 0.0, 0.0);

    EXPECT_TRUE(filtering_utils::isNoNeedAvoidanceBehavior(object_data, parameters));
    EXPECT_EQ(object_data.info, ObjectInfo::LESS_THAN_EXECUTION_THRESHOLD);
  }

  // larger than execution threshold.
  {
    object_data.object.kinematics.initial_pose_with_covariance.pose =
      geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(create_point(2.5, 2.9, 0.0))
        .orientation(create_quaternion_from_rpy(0.0, 0.0, deg2rad(45)));
    object_data.direction = Direction::LEFT;
    object_data.avoid_margin = 2.0;
    object_data.envelope_poly = createEnvelopePolygon(object_data, nearest_path_pose, 0.0);
    object_data.overhang_points = calcEnvelopeOverhangDistance(object_data, path, 0.0, 0.0);

    EXPECT_FALSE(filtering_utils::isNoNeedAvoidanceBehavior(object_data, parameters));
  }
}

TEST(TestUtils, getAvoidMargin)
{
  const auto parameters = get_parameters();

  const auto planner_data = get_planner_data();

  // wide road
  {
    ObjectData object_data;
    object_data.is_parked = false;
    object_data.distance_factor = 1.0;
    object_data.to_road_shoulder_distance = 5.0;
    object_data.object.classification.emplace_back(
      autoware_perception_msgs::build<ObjectClassification>()
        .label(ObjectClassification::TRUCK)
        .probability(1.0));

    const auto output = filtering_utils::getAvoidMargin(object_data, planner_data, parameters);
    ASSERT_TRUE(output.has_value());
    EXPECT_DOUBLE_EQ(output.value(), 1.9);
  }

  // narrow road (relax lateral soft margin)
  {
    ObjectData object_data;
    object_data.is_parked = false;
    object_data.distance_factor = 1.0;
    object_data.to_road_shoulder_distance = 3.0;
    object_data.object.classification.emplace_back(
      autoware_perception_msgs::build<ObjectClassification>()
        .label(ObjectClassification::TRUCK)
        .probability(1.0));

    const auto output = filtering_utils::getAvoidMargin(object_data, planner_data, parameters);
    ASSERT_TRUE(output.has_value());
    EXPECT_DOUBLE_EQ(output.value(), 1.5);
  }

  // narrow road (relax drivable bound margin)
  {
    ObjectData object_data;
    object_data.is_parked = false;
    object_data.distance_factor = 1.0;
    object_data.to_road_shoulder_distance = 2.5;
    object_data.object.classification.emplace_back(
      autoware_perception_msgs::build<ObjectClassification>()
        .label(ObjectClassification::TRUCK)
        .probability(1.0));

    const auto output = filtering_utils::getAvoidMargin(object_data, planner_data, parameters);
    ASSERT_TRUE(output.has_value());
    EXPECT_DOUBLE_EQ(output.value(), 1.2);
  }

  // road width is not enough.
  {
    ObjectData object_data;
    object_data.is_parked = true;
    object_data.distance_factor = 1.0;
    object_data.to_road_shoulder_distance = 2.5;
    object_data.object.classification.emplace_back(
      autoware_perception_msgs::build<ObjectClassification>()
        .label(ObjectClassification::TRUCK)
        .probability(1.0));

    const auto output = filtering_utils::getAvoidMargin(object_data, planner_data, parameters);
    EXPECT_FALSE(output.has_value());
  }
}

TEST(TestUtils, isAvoidanceOpportunityRunningOut)
{
  // Keep front-constant-distance computation independent of PlannerData vehicle geometry, which
  // `get_planner_data()` does not populate (only vehicle_width is set there).
  auto parameters = get_parameters();
  parameters->consider_front_overhang = false;

  const auto planner_data = get_planner_data();

  auto helper = std::make_shared<helper::static_obstacle_avoidance::AvoidanceHelper>(parameters);
  helper->setData(planner_data);

  const auto make_unknown_object = [](const double longitudinal) {
    ObjectData object_data;
    object_data.object.classification.emplace_back(
      autoware_perception_msgs::build<ObjectClassification>()
        .label(ObjectClassification::UNKNOWN)
        .probability(1.0));
    object_data.direction = Direction::RIGHT;
    // Unbiased scenario: true-lane-relative side agrees with the path-relative `direction` above.
    // isAvoidanceOpportunityRunningOut() now uses is_on_right_of_true_lane (see the
    // calcShiftLength frame-mismatch fix) instead of isOnRight(object)/direction.
    object_data.is_on_right_of_true_lane = true;
    object_data.overhang_points.emplace_back(0.5, Point{});
    object_data.longitudinal = longitudinal;
    return object_data;
  };

  const std::optional<double> avoid_margin = 0.5;

  // determine the distance required to still attempt a shift with the above setup, so the test
  // doesn't hardcode brittle numbers derived from the jerk-based distance formula.
  const auto probe_object = make_unknown_object(0.0);
  const auto is_object_on_right = probe_object.is_on_right_of_true_lane;
  const auto desire_shift_length =
    helper->getShiftLength(probe_object, is_object_on_right, avoid_margin.value());
  const auto required_distance = helper->getNominalPrepareDistance() +
                                  helper->getFrontConstantDistance(probe_object) +
                                  helper->getMinAvoidanceDistance(desire_shift_length);
  ASSERT_GT(required_distance, 0.0);

  // Case 1: fresh UNKNOWN object, but already close enough that waiting out the rest of the
  // instability window would consume the remaining avoidance opportunity -- must NOT be
  // excluded (classification-stability safeguard is short-circuited).
  {
    auto object_data = make_unknown_object(required_distance - 0.5);
    EXPECT_TRUE(
      utils::static_obstacle_avoidance::isAvoidanceOpportunityRunningOut(
        object_data, avoid_margin, helper));
  }

  // Case 2 (regression): fresh UNKNOWN object with ample remaining distance -- the "wait and
  // see" safety behavior during the instability window must be preserved.
  {
    auto object_data = make_unknown_object(required_distance + 5.0);
    EXPECT_FALSE(
      utils::static_obstacle_avoidance::isAvoidanceOpportunityRunningOut(
        object_data, avoid_margin, helper));
  }

  // Case 3: even at zero remaining distance, if the object isn't geometrically avoidable at all
  // (no avoid margin), don't short-circuit the safeguard -- there's nothing to gain.
  {
    auto object_data = make_unknown_object(0.0);
    EXPECT_FALSE(
      utils::static_obstacle_avoidance::isAvoidanceOpportunityRunningOut(
        object_data, std::nullopt, helper));
  }
}

TEST(TestUtils, isSafetyCheckTargetObjectType)
{
  const auto parameters = get_parameters();

  {
    ObjectData object_data;
    object_data.object.classification.emplace_back(
      autoware_perception_msgs::build<ObjectClassification>()
        .label(ObjectClassification::TRUCK)
        .probability(1.0));

    EXPECT_TRUE(filtering_utils::isSafetyCheckTargetObjectType(object_data.object, parameters));
    EXPECT_TRUE(filtering_utils::isVehicleTypeObject(object_data));
    EXPECT_FALSE(filtering_utils::isUnknownTypeObject(object_data));
  }

  {
    ObjectData object_data;
    object_data.object.classification.emplace_back(
      autoware_perception_msgs::build<ObjectClassification>()
        .label(ObjectClassification::UNKNOWN)
        .probability(1.0));

    EXPECT_FALSE(filtering_utils::isSafetyCheckTargetObjectType(object_data.object, parameters));
    EXPECT_TRUE(filtering_utils::isVehicleTypeObject(object_data));
    EXPECT_TRUE(filtering_utils::isUnknownTypeObject(object_data));
  }

  {
    ObjectData object_data;
    object_data.object.classification.emplace_back(
      autoware_perception_msgs::build<ObjectClassification>()
        .label(ObjectClassification::PEDESTRIAN)
        .probability(1.0));

    EXPECT_FALSE(filtering_utils::isSafetyCheckTargetObjectType(object_data.object, parameters));
    EXPECT_FALSE(filtering_utils::isVehicleTypeObject(object_data));
    EXPECT_FALSE(filtering_utils::isUnknownTypeObject(object_data));
  }
}

TEST(TestUtils, isSatisfiedWithCommonCondition)
{
  constexpr double forward_detection_range = 5.5;

  const auto path = autoware::test_utils::generateTrajectory<PathWithLaneId>(
    50L, 1.0, 20.0, 0.0, 0.0, std::numeric_limits<size_t>::max());

  const auto parameters = get_parameters();

  // no configuration for this object.
  {
    ObjectData object_data;
    object_data.object.classification.emplace_back(
      autoware_perception_msgs::build<ObjectClassification>()
        .label(ObjectClassification::PEDESTRIAN)
        .probability(1.0));

    EXPECT_FALSE(
      filtering_utils::isSatisfiedWithCommonCondition(
        object_data, path, forward_detection_range, 4.0, create_point(0.0, 0.0, 0.0), false,
        parameters));
    EXPECT_EQ(object_data.info, ObjectInfo::IS_NOT_TARGET_OBJECT);
  }

  // not target object.
  {
    ObjectData object_data;
    object_data.object.classification.emplace_back(
      autoware_perception_msgs::build<ObjectClassification>()
        .label(ObjectClassification::UNKNOWN)
        .probability(1.0));

    EXPECT_FALSE(
      filtering_utils::isSatisfiedWithCommonCondition(
        object_data, path, forward_detection_range, 4.0, create_point(0.0, 0.0, 0.0), false,
        parameters));
    EXPECT_EQ(object_data.info, ObjectInfo::IS_NOT_TARGET_OBJECT);
  }

  // moving object.
  {
    ObjectData object_data;
    object_data.object.classification.emplace_back(
      autoware_perception_msgs::build<ObjectClassification>()
        .label(ObjectClassification::TRUCK)
        .probability(1.0));
    object_data.move_time = 0.6;

    EXPECT_FALSE(
      filtering_utils::isSatisfiedWithCommonCondition(
        object_data, path, forward_detection_range, 4.0, create_point(0.0, 0.0, 0.0), false,
        parameters));
    EXPECT_EQ(object_data.info, ObjectInfo::MOVING_OBJECT);
  }

  // object behind the ego.
  {
    const auto object_pose = geometry_msgs::build<geometry_msgs::msg::Pose>()
                               .position(create_point(2.5, 1.0, 0.0))
                               .orientation(create_quaternion_from_rpy(0.0, 0.0, deg2rad(45)));

    ObjectData object_data;
    object_data.object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    object_data.object.shape.dimensions.x = 2.8284271247461901;
    object_data.object.shape.dimensions.y = 1.41421356237309505;
    object_data.object.kinematics.initial_pose_with_covariance.pose = object_pose;
    object_data.direction = Direction::LEFT;
    object_data.object.classification.emplace_back(
      autoware_perception_msgs::build<ObjectClassification>()
        .label(ObjectClassification::TRUCK)
        .probability(1.0));
    object_data.move_time = 0.4;

    constexpr double margin = 0.0;
    const auto pose =
      path.points.at(autoware::motion_utils::findNearestIndex(path.points, object_pose.position))
        .point.pose;

    object_data.envelope_poly = createEnvelopePolygon(object_data, pose, margin);

    EXPECT_FALSE(
      filtering_utils::isSatisfiedWithCommonCondition(
        object_data, path, forward_detection_range, 4.0, create_point(8.0, 0.5, 0.0), false,
        parameters));
    EXPECT_EQ(object_data.info, ObjectInfo::FURTHER_THAN_THRESHOLD);
  }

  // farther than detection range.
  {
    const auto object_pose = geometry_msgs::build<geometry_msgs::msg::Pose>()
                               .position(create_point(7.5, 1.0, 0.0))
                               .orientation(create_quaternion_from_rpy(0.0, 0.0, deg2rad(45)));

    ObjectData object_data;
    object_data.object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    object_data.object.shape.dimensions.x = 2.8284271247461901;
    object_data.object.shape.dimensions.y = 1.41421356237309505;
    object_data.object.kinematics.initial_pose_with_covariance.pose = object_pose;
    object_data.direction = Direction::LEFT;
    object_data.object.classification.emplace_back(
      autoware_perception_msgs::build<ObjectClassification>()
        .label(ObjectClassification::TRUCK)
        .probability(1.0));
    object_data.move_time = 0.4;

    constexpr double margin = 0.0;
    const auto pose =
      path.points.at(autoware::motion_utils::findNearestIndex(path.points, object_pose.position))
        .point.pose;

    object_data.envelope_poly = createEnvelopePolygon(object_data, pose, margin);

    EXPECT_FALSE(
      filtering_utils::isSatisfiedWithCommonCondition(
        object_data, path, forward_detection_range, 4.0, create_point(0.0, 0.0, 0.0), false,
        parameters));
    EXPECT_EQ(object_data.info, ObjectInfo::FURTHER_THAN_THRESHOLD);
  }

  // farther than goal position.
  {
    const auto object_pose = geometry_msgs::build<geometry_msgs::msg::Pose>()
                               .position(create_point(7.0, 1.0, 0.0))
                               .orientation(create_quaternion_from_rpy(0.0, 0.0, deg2rad(45)));

    ObjectData object_data;
    object_data.object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    object_data.object.shape.dimensions.x = 2.8284271247461901;
    object_data.object.shape.dimensions.y = 1.41421356237309505;
    object_data.object.kinematics.initial_pose_with_covariance.pose = object_pose;
    object_data.direction = Direction::LEFT;
    object_data.object.classification.emplace_back(
      autoware_perception_msgs::build<ObjectClassification>()
        .label(ObjectClassification::TRUCK)
        .probability(1.0));
    object_data.move_time = 0.4;

    constexpr double margin = 0.0;
    const auto pose =
      path.points.at(autoware::motion_utils::findNearestIndex(path.points, object_pose.position))
        .point.pose;

    object_data.envelope_poly = createEnvelopePolygon(object_data, pose, margin);

    EXPECT_FALSE(
      filtering_utils::isSatisfiedWithCommonCondition(
        object_data, path, forward_detection_range, 4.0, create_point(0.0, 0.0, 0.0), false,
        parameters));
    EXPECT_EQ(object_data.info, ObjectInfo::FURTHER_THAN_GOAL);
  }

  // within detection range.
  {
    const auto object_pose = geometry_msgs::build<geometry_msgs::msg::Pose>()
                               .position(create_point(4.5, 1.0, 0.0))
                               .orientation(create_quaternion_from_rpy(0.0, 0.0, deg2rad(45)));

    ObjectData object_data;
    object_data.object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    object_data.object.shape.dimensions.x = 2.8284271247461901;
    object_data.object.shape.dimensions.y = 1.41421356237309505;
    object_data.object.kinematics.initial_pose_with_covariance.pose = object_pose;
    object_data.direction = Direction::LEFT;
    object_data.object.classification.emplace_back(
      autoware_perception_msgs::build<ObjectClassification>()
        .label(ObjectClassification::TRUCK)
        .probability(1.0));
    object_data.move_time = 0.4;

    constexpr double margin = 0.0;
    const auto pose =
      path.points.at(autoware::motion_utils::findNearestIndex(path.points, object_pose.position))
        .point.pose;

    object_data.envelope_poly = createEnvelopePolygon(object_data, pose, margin);

    EXPECT_FALSE(
      filtering_utils::isSatisfiedWithCommonCondition(
        object_data, path, forward_detection_range, 6.4, create_point(0.0, 0.0, 0.0), false,
        parameters));
    EXPECT_EQ(object_data.info, ObjectInfo::TOO_NEAR_TO_GOAL);
  }

  // within detection range.
  {
    const auto object_pose = geometry_msgs::build<geometry_msgs::msg::Pose>()
                               .position(create_point(4.5, 1.0, 0.0))
                               .orientation(create_quaternion_from_rpy(0.0, 0.0, deg2rad(45)));

    ObjectData object_data;
    object_data.object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    object_data.object.shape.dimensions.x = 2.8284271247461901;
    object_data.object.shape.dimensions.y = 1.41421356237309505;
    object_data.object.kinematics.initial_pose_with_covariance.pose = object_pose;
    object_data.direction = Direction::LEFT;
    object_data.object.classification.emplace_back(
      autoware_perception_msgs::build<ObjectClassification>()
        .label(ObjectClassification::TRUCK)
        .probability(1.0));
    object_data.move_time = 0.4;

    constexpr double margin = 0.0;
    const auto pose =
      path.points.at(autoware::motion_utils::findNearestIndex(path.points, object_pose.position))
        .point.pose;

    object_data.envelope_poly = createEnvelopePolygon(object_data, pose, margin);

    EXPECT_TRUE(
      filtering_utils::isSatisfiedWithCommonCondition(
        object_data, path, forward_detection_range, 6.6, create_point(0.0, 0.0, 0.0), false,
        parameters));
  }

  // within detection range.
  {
    const auto object_pose = geometry_msgs::build<geometry_msgs::msg::Pose>()
                               .position(create_point(4.5, 1.0, 0.0))
                               .orientation(create_quaternion_from_rpy(0.0, 0.0, deg2rad(45)));

    ObjectData object_data;
    object_data.object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    object_data.object.shape.dimensions.x = 2.8284271247461901;
    object_data.object.shape.dimensions.y = 1.41421356237309505;
    object_data.object.kinematics.initial_pose_with_covariance.pose = object_pose;
    object_data.direction = Direction::LEFT;
    object_data.object.classification.emplace_back(
      autoware_perception_msgs::build<ObjectClassification>()
        .label(ObjectClassification::TRUCK)
        .probability(1.0));
    object_data.move_time = 0.4;

    constexpr double margin = 0.0;
    const auto pose =
      path.points.at(autoware::motion_utils::findNearestIndex(path.points, object_pose.position))
        .point.pose;

    object_data.envelope_poly = createEnvelopePolygon(object_data, pose, margin);

    EXPECT_TRUE(
      filtering_utils::isSatisfiedWithCommonCondition(
        object_data, path, forward_detection_range, 4.0, create_point(0.0, 0.0, 0.0), true,
        parameters));
  }
}

TEST(TestUtils, isSameDirectionShift)
{
  constexpr double negative_shift_length = -1.0;
  constexpr double positive_shift_length = 1.0;

  {
    ObjectData object;
    object.direction = Direction::RIGHT;

    EXPECT_TRUE(isSameDirectionShift(isOnRight(object), negative_shift_length));
    EXPECT_FALSE(isSameDirectionShift(isOnRight(object), positive_shift_length));
  }

  {
    ObjectData object;
    object.direction = Direction::LEFT;
    EXPECT_TRUE(isSameDirectionShift(isOnRight(object), positive_shift_length));
    EXPECT_FALSE(isSameDirectionShift(isOnRight(object), negative_shift_length));
  }

  {
    ObjectData object;
    object.direction = Direction::NONE;
    EXPECT_THROW(isSameDirectionShift(isOnRight(object), positive_shift_length), std::logic_error);
  }
}

TEST(TestUtils, isShiftNecessary)
{
  constexpr double negative_shift_length = -1.0;
  constexpr double positive_shift_length = 1.0;

  {
    ObjectData object;
    object.direction = Direction::RIGHT;

    EXPECT_TRUE(isShiftNecessary(isOnRight(object), positive_shift_length));
    EXPECT_FALSE(isShiftNecessary(isOnRight(object), negative_shift_length));
  }

  {
    ObjectData object;
    object.direction = Direction::LEFT;
    EXPECT_TRUE(isShiftNecessary(isOnRight(object), negative_shift_length));
    EXPECT_FALSE(isShiftNecessary(isOnRight(object), positive_shift_length));
  }

  {
    ObjectData object;
    object.direction = Direction::NONE;
    EXPECT_THROW(isShiftNecessary(isOnRight(object), positive_shift_length), std::logic_error);
  }
}

TEST(TestUtils, calcShiftLength)
{
  {
    constexpr bool is_on_right = true;
    constexpr double overhang = -1.0;
    constexpr double margin = 1.5;

    const auto output = calcShiftLength(is_on_right, overhang, margin);
    EXPECT_DOUBLE_EQ(output, 0.5);
  }

  {
    constexpr bool is_on_right = false;
    constexpr double overhang = -1.0;
    constexpr double margin = 1.5;

    const auto output = calcShiftLength(is_on_right, overhang, margin);
    EXPECT_DOUBLE_EQ(output, -2.5);
  }
}

TEST(TestUtils, insertDecelPoint)
{
  // invalid target decel point
  {
    auto path = autoware::test_utils::generateTrajectory<PathWithLaneId>(
      50L, 1.0, 20.0, 0.0, 0.0, std::numeric_limits<size_t>::max());

    const auto ego_position = create_point(2.5, 0.5, 0.0);
    constexpr double offset = 100.0;
    constexpr double velocity = 1.0;

    PoseWithDetailOpt p_out{std::nullopt};
    insertDecelPoint(ego_position, offset, velocity, path, p_out);

    EXPECT_FALSE(p_out.has_value());
    std::for_each(path.points.begin(), path.points.end(), [](const auto & p) {
      EXPECT_DOUBLE_EQ(p.point.longitudinal_velocity_mps, 20.0);
    });
  }

  // nominal case
  {
    auto path = autoware::test_utils::generateTrajectory<PathWithLaneId>(
      50L, 1.0, 20.0, 0.0, 0.0, std::numeric_limits<size_t>::max());

    const auto ego_position = create_point(3.5, 0.5, 0.0);
    constexpr double offset = 3.0;
    constexpr double velocity = 1.0;

    PoseWithDetailOpt p_out{std::nullopt};
    insertDecelPoint(ego_position, offset, velocity, path, p_out);

    ASSERT_TRUE(p_out.has_value());
    EXPECT_DOUBLE_EQ(p_out.value().pose.position.x, 6.5);
    EXPECT_DOUBLE_EQ(p_out.value().pose.position.y, 0.0);
    EXPECT_DOUBLE_EQ(p_out.value().pose.position.z, 0.0);
    for (size_t i = 7; i < path.points.size(); i++) {
      EXPECT_DOUBLE_EQ(path.points.at(i).point.longitudinal_velocity_mps, 1.0);
    }
  }
}

TEST(TestUtils, DISABLED_fillObjectMovingTime)
{
  using namespace std::literals::chrono_literals;

  const auto parameters = get_parameters();

  const auto object_pose = geometry_msgs::build<geometry_msgs::msg::Pose>()
                             .position(create_point(0.0, 0.0, 0.0))
                             .orientation(create_quaternion_from_rpy(0.0, 0.0, deg2rad(45)));

  const auto shape = autoware_perception_msgs::build<Shape>()
                       .type(Shape::BOUNDING_BOX)
                       .footprint(geometry_msgs::msg::Polygon{})
                       .dimensions(create_vector3(2.0, 1.0, 2.0));

  const auto uuid = generate_uuid();

  auto old_object = create_test_object(
    object_pose, create_vector3(0.0, 0.0, 0.0), shape, ObjectClassification::TRUCK);

  // set previous object data.
  {
    old_object.object.object_id = uuid;
    old_object.last_move = rclcpp::Clock{RCL_ROS_TIME}.now();
    old_object.last_stop = rclcpp::Clock{RCL_ROS_TIME}.now();
    old_object.move_time = 0.0;
    old_object.object.kinematics.initial_twist_with_covariance.twist.linear =
      create_vector3(0.0, 0.0, 0.0);
  }

  rclcpp::sleep_for(490ms);

  auto new_object = create_test_object(
    object_pose, create_vector3(0.0, 0.0, 0.0), shape, ObjectClassification::TRUCK);

  // find new stop object
  {
    new_object.object.object_id = generate_uuid();
    new_object.object.kinematics.initial_twist_with_covariance.twist.linear =
      create_vector3(0.99, 0.0, 0.0);

    ObjectDataArray buffer{};
    fillObjectMovingTime(new_object, buffer, parameters);
    EXPECT_NEAR(new_object.stop_time, 0.0, 1e-3);
    EXPECT_NEAR(new_object.move_time, 0.0, 1e-3);
    EXPECT_FALSE(buffer.empty());
  }

  // find new move object
  {
    new_object.object.object_id = generate_uuid();
    new_object.object.kinematics.initial_twist_with_covariance.twist.linear =
      create_vector3(1.01, 0.0, 0.0);

    ObjectDataArray buffer{};
    fillObjectMovingTime(new_object, buffer, parameters);
    EXPECT_NEAR(new_object.stop_time, 0.0, 1e-3);
    EXPECT_TRUE(buffer.empty());
  }

  // stop to move (moving time < threshold)
  {
    new_object.object.object_id = uuid;
    new_object.object.kinematics.initial_twist_with_covariance.twist.linear =
      create_vector3(1.01, 0.0, 0.0);

    ObjectDataArray buffer{old_object};
    fillObjectMovingTime(new_object, buffer, parameters);

    EXPECT_NEAR(new_object.stop_time, 0.0, 1e-3);
    EXPECT_NEAR(new_object.move_time, 0.49, 1e-3);
    EXPECT_FALSE(buffer.empty());
  }

  // stop to stop
  {
    new_object.object.object_id = uuid;
    new_object.object.kinematics.initial_twist_with_covariance.twist.linear =
      create_vector3(0.99, 0.0, 0.0);

    ObjectDataArray buffer{old_object};
    fillObjectMovingTime(new_object, buffer, parameters);

    EXPECT_NEAR(new_object.stop_time, 0.49, 1e-3);
    EXPECT_NEAR(new_object.move_time, 0.0, 1e-3);
    EXPECT_FALSE(buffer.empty());
  }

  rclcpp::sleep_for(20ms);

  // stop to move (threshold < moving time)
  {
    new_object.object.object_id = uuid;
    new_object.object.kinematics.initial_twist_with_covariance.twist.linear =
      create_vector3(1.01, 0.0, 0.0);

    ObjectDataArray buffer{old_object};
    fillObjectMovingTime(new_object, buffer, parameters);

    EXPECT_NEAR(new_object.stop_time, 0.0, 1e-3);
    EXPECT_NEAR(new_object.move_time, 0.51, 1e-3);
    EXPECT_TRUE(buffer.empty());
  }
}

TEST(TestUtils, calcEnvelopeOverhangDistance)
{
  const auto path = autoware::test_utils::generateTrajectory<PathWithLaneId>(
    50L, 1.0, 20.0, 0.0, 0.0, std::numeric_limits<size_t>::max());

  const auto object_pose = geometry_msgs::build<geometry_msgs::msg::Pose>()
                             .position(create_point(0.0, 0.0, 0.0))
                             .orientation(create_quaternion_from_rpy(0.0, 0.0, deg2rad(45)));

  const auto nearest_path_pose =
    path.points.at(autoware::motion_utils::findNearestIndex(path.points, object_pose.position))
      .point.pose;

  const auto shape = autoware_perception_msgs::build<Shape>()
                       .type(Shape::BOUNDING_BOX)
                       .footprint(geometry_msgs::msg::Polygon{})
                       .dimensions(create_vector3(2.8284271247461901, 1.41421356237309505, 2.0));

  auto object_data = create_test_object(
    object_pose, create_vector3(0.0, 0.0, 0.0), shape, ObjectClassification::TRUCK);

  {
    object_data.object.kinematics.initial_pose_with_covariance.pose =
      geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(create_point(2.5, 1.0, 0.0))
        .orientation(create_quaternion_from_rpy(0.0, 0.0, deg2rad(45)));
    object_data.direction = Direction::LEFT;
    object_data.envelope_poly = createEnvelopePolygon(object_data, nearest_path_pose, 0.0);

    const auto output = calcEnvelopeOverhangDistance(object_data, path, 0.0, 0.0);

    ASSERT_EQ(output.size(), 8);
    EXPECT_NEAR(output.at(0).first, -0.5, epsilon);
    EXPECT_NEAR(output.at(1).first, -0.5, epsilon);
    EXPECT_NEAR(output.at(2).first, -0.5, epsilon);
    EXPECT_NEAR(output.at(3).first, 1, epsilon);
    EXPECT_NEAR(output.at(4).first, 1, epsilon);
    EXPECT_NEAR(output.at(5).first, 2.5, epsilon);
    EXPECT_NEAR(output.at(6).first, 2.5, epsilon);
    EXPECT_NEAR(output.at(7).first, 2.5, epsilon);
  }

  {
    object_data.object.kinematics.initial_pose_with_covariance.pose =
      geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(create_point(2.5, -1.0, 0.0))
        .orientation(create_quaternion_from_rpy(0.0, 0.0, deg2rad(45)));
    object_data.direction = Direction::RIGHT;
    object_data.envelope_poly = createEnvelopePolygon(object_data, nearest_path_pose, 0.0);

    const auto output = calcEnvelopeOverhangDistance(object_data, path, 0.0, 0.0);

    ASSERT_EQ(output.size(), 8);
    EXPECT_NEAR(output.at(0).first, 0.5, epsilon);
    EXPECT_NEAR(output.at(1).first, 0.5, epsilon);
    EXPECT_NEAR(output.at(2).first, 0.5, epsilon);
    EXPECT_NEAR(output.at(3).first, -1, epsilon);
    EXPECT_NEAR(output.at(4).first, -1, epsilon);
    EXPECT_NEAR(output.at(5).first, -2.5, epsilon);
    EXPECT_NEAR(output.at(6).first, -2.5, epsilon);
    EXPECT_NEAR(output.at(7).first, -2.5, epsilon);
  }
}

TEST(TestUtils, createEnvelopePolygon)
{
  Polygon2d footprint;
  footprint.outer() = {
    Point2d{1.0, 0.0}, Point2d{3.0, 0.0}, Point2d{3.0, 1.0}, Point2d{1.0, 1.0}, Point2d{1.0, 0.0}};

  constexpr double margin = 0.353553390593273762;
  const auto pose = geometry_msgs::build<geometry_msgs::msg::Pose>()
                      .position(create_point(0.0, 0.0, 0.0))
                      .orientation(create_quaternion_from_rpy(0.0, 0.0, deg2rad(45)));

  const auto output = createEnvelopePolygon(footprint, pose, margin);

  ASSERT_EQ(output.outer().size(), 5);
  EXPECT_NEAR(output.outer().at(0).x(), 2.0, epsilon);
  EXPECT_NEAR(output.outer().at(0).y(), -1.5, epsilon);
  EXPECT_NEAR(output.outer().at(1).x(), 0.0, epsilon);
  EXPECT_NEAR(output.outer().at(1).y(), 0.5, epsilon);
  EXPECT_NEAR(output.outer().at(2).x(), 2.0, epsilon);
  EXPECT_NEAR(output.outer().at(2).y(), 2.5, epsilon);
  EXPECT_NEAR(output.outer().at(3).x(), 4.0, epsilon);
  EXPECT_NEAR(output.outer().at(3).y(), 0.5, epsilon);
  EXPECT_NEAR(output.outer().at(4).x(), 2.0, epsilon);
  EXPECT_NEAR(output.outer().at(4).y(), -1.5, epsilon);
}

TEST(TestUtils, generateObstaclePolygonsForDrivableArea)
{
  const auto create_params = [](
                               const double envelope_buffer, const double hard, const double soft) {
    ObjectParameter param{};
    param.envelope_buffer_margin = envelope_buffer;
    param.lateral_soft_margin = soft;
    param.lateral_hard_margin = hard;
    return param;
  };

  constexpr double vehicle_width = 1.0;
  constexpr double envelope_buffer = 0.353553390593273762;
  constexpr double lateral_soft_margin = 0.707106781186547524;

  const auto parameters = std::make_shared<AvoidanceParameters>();
  parameters->object_parameters.emplace(
    ObjectClassification::TRUCK, create_params(envelope_buffer, 0.5, lateral_soft_margin));

  const auto object_pose = geometry_msgs::build<geometry_msgs::msg::Pose>()
                             .position(create_point(0.0, 0.0, 0.0))
                             .orientation(create_quaternion_from_rpy(0.0, 0.0, deg2rad(45)));

  const auto shape = autoware_perception_msgs::build<Shape>()
                       .type(Shape::BOUNDING_BOX)
                       .footprint(geometry_msgs::msg::Polygon{})
                       .dimensions(create_vector3(2.0, 1.0, 2.0));

  auto object_data = create_test_object(
    object_pose, create_vector3(0.0, 0.0, 0.0), shape, ObjectClassification::TRUCK);

  // empty
  {
    ObjectDataArray objects{};
    const auto output = generateObstaclePolygonsForDrivableArea(objects, parameters, vehicle_width);
    EXPECT_TRUE(output.empty());
  }

  // invalid margin
  {
    const auto object_type = utils::getHighestProbLabel(object_data.object.classification);
    const auto object_parameter = parameters->object_parameters.at(object_type);
    object_data.avoid_margin = std::nullopt;

    Polygon2d footprint;
    footprint.outer() = {
      Point2d{1.0, 0.0}, Point2d{3.0, 0.0}, Point2d{3.0, 1.0}, Point2d{1.0, 1.0},
      Point2d{1.0, 0.0}};

    object_data.envelope_poly =
      createEnvelopePolygon(footprint, object_pose, object_parameter.envelope_buffer_margin);

    ObjectDataArray objects{object_data};
    const auto output = generateObstaclePolygonsForDrivableArea(objects, parameters, vehicle_width);
    EXPECT_TRUE(output.empty());
  }

  // invalid envelope polygon
  {
    const auto object_type = utils::getHighestProbLabel(object_data.object.classification);
    const auto object_parameter = parameters->object_parameters.at(object_type);
    object_data.avoid_margin = object_parameter.lateral_soft_margin + 0.5 * vehicle_width;
    object_data.envelope_poly = {};

    ObjectDataArray objects{object_data};
    const auto output = generateObstaclePolygonsForDrivableArea(objects, parameters, vehicle_width);
    EXPECT_TRUE(output.empty());
  }

  // nominal case
  {
    object_data.direction = Direction::RIGHT;
    const auto object_type = utils::getHighestProbLabel(object_data.object.classification);
    const auto object_parameter = parameters->object_parameters.at(object_type);
    object_data.avoid_margin = object_parameter.lateral_soft_margin + 0.5 * vehicle_width;

    Polygon2d footprint;
    footprint.outer() = {
      Point2d{1.0, 0.0}, Point2d{3.0, 0.0}, Point2d{3.0, 1.0}, Point2d{1.0, 1.0},
      Point2d{1.0, 0.0}};

    object_data.envelope_poly =
      createEnvelopePolygon(footprint, object_pose, object_parameter.envelope_buffer_margin);

    ObjectDataArray objects{object_data};
    const auto output = generateObstaclePolygonsForDrivableArea(objects, parameters, vehicle_width);
    EXPECT_FALSE(output.empty());

    ASSERT_EQ(output.front().poly.outer().size(), 5);
    EXPECT_NEAR(output.front().poly.outer().at(0).x(), 2.0, epsilon);
    EXPECT_NEAR(output.front().poly.outer().at(0).y(), -2.0, epsilon);
    EXPECT_NEAR(output.front().poly.outer().at(1).x(), -0.5, epsilon);
    EXPECT_NEAR(output.front().poly.outer().at(1).y(), 0.5, epsilon);
    EXPECT_NEAR(output.front().poly.outer().at(2).x(), 2.0, epsilon);
    EXPECT_NEAR(output.front().poly.outer().at(2).y(), 3.0, epsilon);
    EXPECT_NEAR(output.front().poly.outer().at(3).x(), 4.5, epsilon);
    EXPECT_NEAR(output.front().poly.outer().at(3).y(), 0.5, epsilon);
    EXPECT_NEAR(output.front().poly.outer().at(4).x(), 2.0, epsilon);
    EXPECT_NEAR(output.front().poly.outer().at(4).y(), -2.0, epsilon);
  }
}

TEST(TestUtils, fillLongitudinalAndLengthByClosestEnvelopeFootprint)
{
  const auto path = autoware::test_utils::generateTrajectory<PathWithLaneId>(
    50L, 1.0, 20.0, 0.0, 0.0, std::numeric_limits<size_t>::max());

  const auto object_pose = geometry_msgs::build<geometry_msgs::msg::Pose>()
                             .position(create_point(2.5, 1.0, 0.0))
                             .orientation(create_quaternion_from_rpy(0.0, 0.0, deg2rad(45)));

  const auto nearest_path_pose =
    path.points.at(autoware::motion_utils::findNearestIndex(path.points, object_pose.position))
      .point.pose;

  const auto shape = autoware_perception_msgs::build<Shape>()
                       .type(Shape::BOUNDING_BOX)
                       .footprint(geometry_msgs::msg::Polygon{})
                       .dimensions(create_vector3(2.8284271247461901, 1.41421356237309505, 2.0));

  auto object_data = create_test_object(
    object_pose, create_vector3(0.0, 0.0, 0.0), shape, ObjectClassification::TRUCK);
  object_data.direction = Direction::LEFT;
  object_data.envelope_poly = createEnvelopePolygon(object_data, nearest_path_pose, 0.0);

  fillLongitudinalAndLengthByClosestEnvelopeFootprint(
    path, create_point(0.0, 0.0, 0.0), object_data);
  EXPECT_NEAR(object_data.longitudinal, 1.0, epsilon);
  EXPECT_NEAR(object_data.length, 3.0, epsilon);
}

TEST(TestUtils, fillObjectEnvelopePolygon)
{
  const auto path = autoware::test_utils::generateTrajectory<PathWithLaneId>(
    50L, 1.0, 20.0, 0.0, 0.0, std::numeric_limits<size_t>::max());

  const auto parameters = get_parameters();

  const auto uuid = generate_uuid();

  const auto object_pose = geometry_msgs::build<geometry_msgs::msg::Pose>()
                             .position(create_point(2.5, 1.0, 0.0))
                             .orientation(create_quaternion_from_rpy(0.0, 0.0, deg2rad(45)));

  const auto nearest_path_pose =
    path.points.at(autoware::motion_utils::findNearestIndex(path.points, object_pose.position))
      .point.pose;

  const auto shape = autoware_perception_msgs::build<Shape>()
                       .type(Shape::BOUNDING_BOX)
                       .footprint(geometry_msgs::msg::Polygon{})
                       .dimensions(create_vector3(2.8284271247461901, 1.41421356237309505, 2.0));

  auto stored_object = create_test_object(
    object_pose, create_vector3(0.0, 0.0, 0.0), shape, ObjectClassification::TRUCK);
  stored_object.object.object_id = uuid;
  stored_object.envelope_poly = createEnvelopePolygon(stored_object, nearest_path_pose, 0.0);
  stored_object.object.kinematics.initial_pose_with_covariance =
    geometry_msgs::build<geometry_msgs::msg::PoseWithCovariance>()
      .pose(object_pose)
      .covariance({1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});
  stored_object.error_eclipse_max =
    calcErrorEclipseLongRadius(stored_object.object.kinematics.initial_pose_with_covariance);
  stored_object.direction = Direction::LEFT;

  auto object_data = create_test_object(
    object_pose, create_vector3(0.0, 0.0, 0.0), shape, ObjectClassification::TRUCK);

  // new object.
  {
    ObjectDataArray stored_objects{};

    object_data.object.object_id = generate_uuid();
    object_data.object.kinematics.initial_pose_with_covariance =
      geometry_msgs::build<geometry_msgs::msg::PoseWithCovariance>()
        .pose(object_pose)
        .covariance({1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});
    object_data.direction = Direction::LEFT;
    fillObjectEnvelopePolygon(object_data, stored_objects, nearest_path_pose, parameters);

    ASSERT_EQ(object_data.envelope_poly.outer().size(), 5);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(0).x(), 1.0, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(0).y(), -0.5, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(1).x(), 1.0, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(1).y(), 2.5, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(2).x(), 4.0, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(2).y(), 2.5, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(3).x(), 4.0, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(3).y(), -0.5, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(4).x(), 1.0, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(4).y(), -0.5, epsilon);
  }

  // update envelope polygon by new pose.
  {
    ObjectDataArray stored_objects{stored_object};

    const auto new_pose = geometry_msgs::build<geometry_msgs::msg::Pose>()
                            .position(create_point(3.0, 0.5, 0.0))
                            .orientation(create_quaternion_from_rpy(0.0, 0.0, deg2rad(45)));

    object_data.object.object_id = uuid;
    object_data.object.kinematics.initial_pose_with_covariance =
      geometry_msgs::build<geometry_msgs::msg::PoseWithCovariance>().pose(new_pose).covariance(
        {1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});
    object_data.direction = Direction::LEFT;
    fillObjectEnvelopePolygon(object_data, stored_objects, nearest_path_pose, parameters);

    ASSERT_EQ(object_data.envelope_poly.outer().size(), 5);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(0).x(), 1.0, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(0).y(), -1.0, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(1).x(), 1.0, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(1).y(), 2.5, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(2).x(), 4.5, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(2).y(), 2.5, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(3).x(), 4.5, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(3).y(), -1.0, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(4).x(), 1.0, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(4).y(), -1.0, epsilon);
  }

  // use previous envelope polygon because new pose's error eclipse long radius is larger than
  // threshold. error eclipse long radius: 2.1213203435596
  {
    ObjectDataArray stored_objects{stored_object};

    const auto new_pose = geometry_msgs::build<geometry_msgs::msg::Pose>()
                            .position(create_point(3.0, 0.5, 0.0))
                            .orientation(create_quaternion_from_rpy(0.0, 0.0, deg2rad(45)));

    object_data.object.object_id = uuid;
    object_data.object.kinematics.initial_pose_with_covariance =
      geometry_msgs::build<geometry_msgs::msg::PoseWithCovariance>().pose(new_pose).covariance(
        {2.5, 2.0, 0.0, 0.0, 0.0, 0.0, 2.0, 2.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});
    object_data.direction = Direction::LEFT;
    fillObjectEnvelopePolygon(object_data, stored_objects, nearest_path_pose, parameters);

    ASSERT_EQ(object_data.envelope_poly.outer().size(), 5);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(0).x(), 1.0, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(0).y(), -0.5, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(1).x(), 1.0, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(1).y(), 2.5, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(2).x(), 4.0, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(2).y(), 2.5, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(3).x(), 4.0, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(3).y(), -0.5, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(4).x(), 1.0, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(4).y(), -0.5, epsilon);
  }

  // use new envelope polygon because new pose's error eclipse long radius is smaller than
  // threshold.
  {
    auto huge_covariance_object = create_test_object(
      object_pose, create_vector3(0.0, 0.0, 0.0), shape, ObjectClassification::TRUCK);
    huge_covariance_object.object.object_id = uuid;
    huge_covariance_object.object.kinematics.initial_pose_with_covariance =
      geometry_msgs::build<geometry_msgs::msg::PoseWithCovariance>()
        .pose(object_pose)
        .covariance({5.0, 4.0, 0.0, 0.0, 0.0, 0.0, 4.0, 5.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});
    huge_covariance_object.error_eclipse_max = calcErrorEclipseLongRadius(
      huge_covariance_object.object.kinematics.initial_pose_with_covariance);
    huge_covariance_object.direction = Direction::LEFT;

    huge_covariance_object.envelope_poly =
      createEnvelopePolygon(huge_covariance_object, nearest_path_pose, 0.0);

    ObjectDataArray stored_objects{huge_covariance_object};

    const auto new_pose = geometry_msgs::build<geometry_msgs::msg::Pose>()
                            .position(create_point(3.0, 0.5, 0.0))
                            .orientation(create_quaternion_from_rpy(0.0, 0.0, deg2rad(45)));

    object_data.object.object_id = uuid;
    object_data.object.kinematics.initial_pose_with_covariance =
      geometry_msgs::build<geometry_msgs::msg::PoseWithCovariance>().pose(new_pose).covariance(
        {2.5, 2.0, 0.0, 0.0, 0.0, 0.0, 2.0, 2.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});
    object_data.direction = Direction::LEFT;
    fillObjectEnvelopePolygon(object_data, stored_objects, nearest_path_pose, parameters);

    ASSERT_EQ(object_data.envelope_poly.outer().size(), 5);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(0).x(), 1.5, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(0).y(), -1.0, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(1).x(), 1.5, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(1).y(), 2.0, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(2).x(), 4.5, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(2).y(), 2.0, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(3).x(), 4.5, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(3).y(), -1.0, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(4).x(), 1.5, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(4).y(), -1.0, epsilon);
  }

  // use previous envelope polygon because the new one is within old one.
  {
    ObjectDataArray stored_objects{stored_object};

    object_data.object.object_id = uuid;
    object_data.object.kinematics.initial_pose_with_covariance =
      geometry_msgs::build<geometry_msgs::msg::PoseWithCovariance>()
        .pose(object_pose)
        .covariance({1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});
    object_data.direction = Direction::LEFT;
    fillObjectEnvelopePolygon(object_data, stored_objects, nearest_path_pose, parameters);

    ASSERT_EQ(object_data.envelope_poly.outer().size(), 5);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(0).x(), 1.0, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(0).y(), -0.5, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(1).x(), 1.0, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(1).y(), 2.5, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(2).x(), 4.0, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(2).y(), 2.5, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(3).x(), 4.0, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(3).y(), -0.5, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(4).x(), 1.0, epsilon);
    EXPECT_NEAR(object_data.envelope_poly.outer().at(4).y(), -0.5, epsilon);
  }
}

TEST(TestUtils, compensateLostTargetObjects)
{
  using namespace std::literals::chrono_literals;

  const auto parameters = get_parameters();

  const auto planner_data = get_planner_data();

  const auto uuid = generate_uuid();

  const auto init_time = rclcpp::Clock{RCL_ROS_TIME}.now();

  ObjectData stored_object;
  stored_object.object.object_id = uuid;
  stored_object.last_seen = init_time;
  stored_object.object.kinematics.initial_pose_with_covariance.pose =
    geometry_msgs::build<geometry_msgs::msg::Pose>()
      .position(create_point(1.0, 1.0, 0.0))
      .orientation(create_quaternion_from_rpy(0.0, 0.0, 0.0));

  rclcpp::sleep_for(100ms);

  // add stored objects.
  {
    const auto now = rclcpp::Clock{RCL_ROS_TIME}.now();

    ObjectDataArray stored_objects{};

    ObjectData new_object;
    new_object.object.object_id = generate_uuid();
    new_object.last_seen = now;
    new_object.object.kinematics.initial_pose_with_covariance.pose =
      geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(create_point(2.0, 5.0, 0.0))
        .orientation(create_quaternion_from_rpy(0.0, 0.0, 0.0));

    AvoidancePlanningData avoidance_planning_data;
    avoidance_planning_data.target_objects = {new_object};
    avoidance_planning_data.other_objects = {};

    auto current_target_objects_snapshot = avoidance_planning_data.target_objects;

    utils::static_obstacle_avoidance::compensateLostTargetObjects(
      avoidance_planning_data, stored_objects, planner_data);
    utils::static_obstacle_avoidance::updateStoredObjects(
      stored_objects, current_target_objects_snapshot, now, parameters);
    ASSERT_FALSE(stored_objects.empty());
    EXPECT_EQ(stored_objects.front().object.object_id, new_object.object.object_id);
  }

  // compensate detection lost.
  {
    const auto now = rclcpp::Clock{RCL_ROS_TIME}.now();

    ObjectDataArray stored_objects{stored_object};

    AvoidancePlanningData avoidance_planning_data;
    avoidance_planning_data.target_objects = {};
    avoidance_planning_data.other_objects = {};

    auto current_target_objects_snapshot = avoidance_planning_data.target_objects;

    utils::static_obstacle_avoidance::compensateLostTargetObjects(
      avoidance_planning_data, stored_objects, planner_data);
    utils::static_obstacle_avoidance::updateStoredObjects(
      stored_objects, current_target_objects_snapshot, now, parameters);
    ASSERT_FALSE(avoidance_planning_data.target_objects.empty());
    EXPECT_EQ(
      avoidance_planning_data.target_objects.front().object.object_id,
      stored_object.object.object_id);
  }

  // update stored objects (same uuid).
  {
    const auto now = rclcpp::Clock{RCL_ROS_TIME}.now();

    ObjectDataArray stored_objects{stored_object};

    ObjectData detected_object;
    detected_object.object.object_id = uuid;
    detected_object.last_seen = now;
    detected_object.object.kinematics.initial_pose_with_covariance.pose =
      geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(create_point(1.0, 1.0, 0.0))
        .orientation(create_quaternion_from_rpy(0.0, 0.0, 0.0));

    AvoidancePlanningData avoidance_planning_data;
    avoidance_planning_data.target_objects = {detected_object};
    avoidance_planning_data.other_objects = {};

    auto current_target_objects_snapshot = avoidance_planning_data.target_objects;

    utils::static_obstacle_avoidance::compensateLostTargetObjects(
      avoidance_planning_data, stored_objects, planner_data);
    utils::static_obstacle_avoidance::updateStoredObjects(
      stored_objects, current_target_objects_snapshot, now, parameters);
    ASSERT_FALSE(stored_objects.empty());
    EXPECT_EQ(stored_objects.front().last_seen, detected_object.last_seen);
  }

  // update stored objects (detected near the stored object).
  {
    const auto now = rclcpp::Clock{RCL_ROS_TIME}.now();

    ObjectDataArray stored_objects{stored_object};

    ObjectData detected_object;
    detected_object.object.object_id = generate_uuid();
    detected_object.last_seen = now;
    detected_object.object.kinematics.initial_pose_with_covariance.pose =
      geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(create_point(1.1, 1.1, 0.0))
        .orientation(create_quaternion_from_rpy(0.0, 0.0, 0.0));

    AvoidancePlanningData avoidance_planning_data;
    avoidance_planning_data.target_objects = {detected_object};
    avoidance_planning_data.other_objects = {};

    auto current_target_objects_snapshot = avoidance_planning_data.target_objects;
    utils::static_obstacle_avoidance::compensateLostTargetObjects(
      avoidance_planning_data, stored_objects, planner_data);
    utils::static_obstacle_avoidance::updateStoredObjects(
      stored_objects, current_target_objects_snapshot, now, parameters);
    ASSERT_FALSE(stored_objects.empty());
    EXPECT_EQ(stored_objects.front().last_seen, detected_object.last_seen);
  }

  // don't update stored object because there is no matched object.
  {
    const auto now = rclcpp::Clock{RCL_ROS_TIME}.now();

    ObjectDataArray stored_objects{stored_object};

    ObjectData detected_object;
    detected_object.object.object_id = generate_uuid();
    detected_object.last_seen = now;
    detected_object.object.kinematics.initial_pose_with_covariance.pose =
      geometry_msgs::build<geometry_msgs::msg::Pose>()
        .position(create_point(3.0, 3.0, 0.0))
        .orientation(create_quaternion_from_rpy(0.0, 0.0, 0.0));

    AvoidancePlanningData avoidance_planning_data;
    avoidance_planning_data.target_objects = {detected_object};
    avoidance_planning_data.other_objects = {};

    auto current_target_objects_snapshot = avoidance_planning_data.target_objects;
    utils::static_obstacle_avoidance::compensateLostTargetObjects(
      avoidance_planning_data, stored_objects, planner_data);
    utils::static_obstacle_avoidance::updateStoredObjects(
      stored_objects, current_target_objects_snapshot, now, parameters);

    ASSERT_FALSE(stored_objects.empty());
    EXPECT_EQ(stored_objects.front().last_seen, init_time);
  }

  rclcpp::sleep_for(200ms);

  // don't compensate detection lost because time elapses more than threshold.
  {
    const auto now = rclcpp::Clock{RCL_ROS_TIME}.now();

    ObjectDataArray stored_objects{stored_object};

    AvoidancePlanningData avoidance_planning_data;
    avoidance_planning_data.target_objects = {};
    avoidance_planning_data.other_objects = {};

    auto current_target_objects_snapshot = avoidance_planning_data.target_objects;
    utils::static_obstacle_avoidance::compensateLostTargetObjects(
      avoidance_planning_data, stored_objects, planner_data);
    utils::static_obstacle_avoidance::updateStoredObjects(
      stored_objects, current_target_objects_snapshot, now, parameters);

    avoidance_planning_data.target_objects = {};
    current_target_objects_snapshot = avoidance_planning_data.target_objects;
    utils::static_obstacle_avoidance::compensateLostTargetObjects(
      avoidance_planning_data, stored_objects, planner_data);
    utils::static_obstacle_avoidance::updateStoredObjects(
      stored_objects, current_target_objects_snapshot, now, parameters);

    EXPECT_TRUE(avoidance_planning_data.target_objects.empty());
  }
}

TEST(TestUtils, calcErrorEclipseLongRadius)
{
  const auto pose = geometry_msgs::build<geometry_msgs::msg::Pose>()
                      .position(create_point(3.0, 3.0, 0.0))
                      .orientation(create_quaternion_from_rpy(0.0, 0.0, 0.0));
  const auto pose_with_covariance =
    geometry_msgs::build<geometry_msgs::msg::PoseWithCovariance>().pose(pose).covariance(
      {5.0, 4.0, 0.0, 0.0, 0.0, 0.0, 4.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
       0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});

  EXPECT_DOUBLE_EQ(calcErrorEclipseLongRadius(pose_with_covariance), 3.0);
}

namespace
{
// Builds a single, straight 100m-long lanelet of the given half-width (bounds run parallel to the
// x-axis, centerline at y = 0), registers it as the sole lanelet of a minimal route so that
// `RouteHandler::getClosestLaneletWithinRoute()` can resolve it, and returns both the handler and
// the lanelet (the latter is what callers put into `AvoidancePlanningData::current_lanelets`).
std::pair<std::shared_ptr<autoware::route_handler::RouteHandler>, lanelet::ConstLanelet>
make_straight_lane_route_handler(const double half_width)
{
  lanelet::LineString3d left_bound_ls(lanelet::utils::getId());
  left_bound_ls.push_back(lanelet::Point3d(lanelet::utils::getId(), 0.0, half_width, 0.0));
  left_bound_ls.push_back(lanelet::Point3d(lanelet::utils::getId(), 100.0, half_width, 0.0));

  lanelet::LineString3d right_bound_ls(lanelet::utils::getId());
  right_bound_ls.push_back(lanelet::Point3d(lanelet::utils::getId(), 0.0, -half_width, 0.0));
  right_bound_ls.push_back(lanelet::Point3d(lanelet::utils::getId(), 100.0, -half_width, 0.0));

  lanelet::Lanelet lanelet_obj(lanelet::utils::getId(), left_bound_ls, right_bound_ls);
  lanelet_obj.attributes()[lanelet::AttributeName::Subtype] =
    lanelet::AttributeValueString::Road;
  lanelet_obj.attributes()[lanelet::AttributeName::Location] =
    lanelet::AttributeValueString::Urban;
  lanelet_obj.attributes()[lanelet::AttributeName::OneWay] = "yes";

  const auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(lanelet_obj);

  autoware_map_msgs::msg::LaneletMapBin map_bin_msg;
  map_bin_msg.header.frame_id = "map";
  lanelet::utils::conversion::toBinMsg(map, &map_bin_msg);

  auto route_handler = std::make_shared<autoware::route_handler::RouteHandler>();
  route_handler->setMap(map_bin_msg);
  route_handler->setRouteLanelets(lanelet::ConstLanelets{lanelet::ConstLanelet(lanelet_obj)});

  return {route_handler, lanelet::ConstLanelet(lanelet_obj)};
}

// Builds a straight reference path (constant lateral offset `lateral_offset` from the true
// centerline, heading along +x) spanning x in [0, 20].
PathWithLaneId make_straight_reference_path(const double lateral_offset)
{
  PathWithLaneId path;
  for (double x = 0.0; x <= 20.0 + 1e-6; x += 2.0) {
    PathPointWithLaneId point;
    point.point.pose = autoware::test_utils::createPose(x, lateral_offset, 0.0, 0.0, 0.0, 0.0);
    path.points.push_back(point);
  }
  return path;
}
}  // namespace

TEST(TestUtils, getRoadShoulderDistanceTrueLaneRelativeClassification)
{
  constexpr double half_width = 1.75;  // 3.5m-wide lane, matching the documented repro.
  constexpr double path_bias = -0.5;   // reference path biased toward the right, e.g. by a
                                        // prefer_lateral_ratio lanelet tag.

  const auto planner_data = get_planner_data();

  // Bias case: the reference path is off-center, and the object sits between the biased path
  // and the TRUE centerline (x=-0.3, while path=-0.5, centerline=0). Relative to the biased
  // path the object reads as being on its LEFT (isOnRight() == false), but relative to the true
  // lane centerline it is actually on the RIGHT. The correct "far" boundary is therefore the
  // left bound (ample real space), not the right bound the old path-relative classification
  // would have selected.
  {
    auto [route_handler, lanelet_obj] = make_straight_lane_route_handler(half_width);
    planner_data->route_handler = route_handler;

    AvoidancePlanningData data;
    data.reference_path = make_straight_reference_path(path_bias);
    data.current_lanelets = {lanelet_obj};
    for (double x = 0.0; x <= 100.0 + 1e-6; x += 10.0) {
      data.left_bound.push_back(create_point(x, half_width, 0.0));
      data.right_bound.push_back(create_point(x, -half_width, 0.0));
    }

    constexpr double object_y = -0.3;
    const auto object_pose =
      autoware::test_utils::createPose(10.0, object_y, 0.0, 0.0, 0.0, 0.0);

    ObjectData object_data;
    object_data.object.kinematics.initial_pose_with_covariance.pose = object_pose;
    object_data.overhang_points.emplace_back(0.0, object_pose.position);

    // Sanity-check the two classifications actually disagree for this geometry.
    const double path_relative_deviation =
      autoware_utils::calc_lateral_deviation(object_pose, object_pose.position);
    // object_closest_pose used inside getRoadShoulderDistance is the reference_path pose at the
    // object's x, i.e. (x, path_bias). Recompute the same deviation used by
    // isOnRight()/object.direction (see scene.cpp) for clarity:
    const auto path_pose_at_object =
      autoware::test_utils::createPose(10.0, path_bias, 0.0, 0.0, 0.0, 0.0);
    const double deviation_from_biased_path =
      autoware_utils::calc_lateral_deviation(path_pose_at_object, object_pose.position);
    EXPECT_GT(deviation_from_biased_path, 0.0);  // path-relative: LEFT (isOnRight() == false)
    EXPECT_LE(object_y, 0.0);                    // true-lane-relative: RIGHT (getDistanceToCenterline <= 0)
    (void)path_relative_deviation;

    const auto distance = filtering_utils::getRoadShoulderDistance(object_data, data, planner_data);

    // Correct behavior: far bound is the LEFT bound (true-lane-relative), giving ample space.
    EXPECT_NEAR(distance, half_width - object_y, 1e-2);
    EXPECT_GT(distance, 1.5);  // ample real clearance, not a false near-zero/insufficient verdict.
  }

  // Regression case (no bias): reference path matches the true centerline, so path-relative and
  // true-lane-relative classification always agree. Verify both a left-side and a right-side
  // object still resolve to the correct (opposite-side) far boundary as before this fix.
  {
    auto [route_handler, lanelet_obj] = make_straight_lane_route_handler(half_width);
    planner_data->route_handler = route_handler;

    AvoidancePlanningData data;
    data.reference_path = make_straight_reference_path(0.0);
    data.current_lanelets = {lanelet_obj};
    for (double x = 0.0; x <= 100.0 + 1e-6; x += 10.0) {
      data.left_bound.push_back(create_point(x, half_width, 0.0));
      data.right_bound.push_back(create_point(x, -half_width, 0.0));
    }

    // object on the right side of the (unbiased) path/centerline -> far bound is left bound.
    {
      constexpr double object_y = -0.6;
      const auto object_pose =
        autoware::test_utils::createPose(10.0, object_y, 0.0, 0.0, 0.0, 0.0);

      ObjectData object_data;
      object_data.object.kinematics.initial_pose_with_covariance.pose = object_pose;
      object_data.overhang_points.emplace_back(0.0, object_pose.position);

      const auto distance =
        filtering_utils::getRoadShoulderDistance(object_data, data, planner_data);
      EXPECT_NEAR(distance, half_width - object_y, 1e-2);
    }

    // object on the left side of the (unbiased) path/centerline -> far bound is right bound.
    {
      constexpr double object_y = 0.6;
      const auto object_pose =
        autoware::test_utils::createPose(10.0, object_y, 0.0, 0.0, 0.0, 0.0);

      ObjectData object_data;
      object_data.object.kinematics.initial_pose_with_covariance.pose = object_pose;
      object_data.overhang_points.emplace_back(0.0, object_pose.position);

      const auto distance =
        filtering_utils::getRoadShoulderDistance(object_data, data, planner_data);
      EXPECT_NEAR(distance, half_width + object_y, 1e-2);
    }
  }
}

// Regression test for the "must be on the edge lane" gate that used to live in
// isSatisfiedWithNonVehicleCondition(): a pedestrian/bicycle object was excluded from avoidance
// outright whenever its overhang lanelet had *any* right/left neighbor lanelet (real lane or
// shoulder) or geometrically-opposite lanelet on the object's side -- even if the object itself
// was sitting well inside the middle of its own (only) lane with genuinely sufficient room to
// shift around. That restriction has been removed for this platform (single-lane-per-direction
// maps); this test builds exactly the trap configuration (a real, non-shoulder right-side
// neighbor lane) and asserts a mid-lane pedestrian is no longer excluded by it.
TEST(TestUtils, isSatisfiedWithNonVehicleConditionAllowsMidLanePedestrianWithAdjacentLane)
{
  constexpr double half_width = 1.75;  // 3.5m-wide lane, matching the documented repro.

  const auto planner_data = get_planner_data();
  const auto parameters = get_parameters();
  // Match the tight production threshold (see static_obstacle_avoidance.param.yaml) so this test
  // exercises the edge-lane gate specifically, not the (already correct) centerline gate.
  parameters->threshold_distance_object_is_on_center = 0.05;

  // Main lane A (ego's lane), and a genuine right-side neighbor lane B sharing A's right
  // boundary -- i.e. a real second lane, not a shoulder. This is the configuration that used to
  // make `getRightLanelet(..., get_shoulder_lane=true)` return a non-"road_shoulder" neighbor
  // and unconditionally exclude any object on that side.
  lanelet::LineString3d left_bound_ls(lanelet::utils::getId());
  left_bound_ls.push_back(lanelet::Point3d(lanelet::utils::getId(), 0.0, half_width, 0.0));
  left_bound_ls.push_back(lanelet::Point3d(lanelet::utils::getId(), 100.0, half_width, 0.0));

  lanelet::LineString3d shared_bound_ls(lanelet::utils::getId());
  shared_bound_ls.push_back(lanelet::Point3d(lanelet::utils::getId(), 0.0, -half_width, 0.0));
  shared_bound_ls.push_back(lanelet::Point3d(lanelet::utils::getId(), 100.0, -half_width, 0.0));

  lanelet::LineString3d right_far_bound_ls(lanelet::utils::getId());
  right_far_bound_ls.push_back(
    lanelet::Point3d(lanelet::utils::getId(), 0.0, -2.0 * half_width, 0.0));
  right_far_bound_ls.push_back(
    lanelet::Point3d(lanelet::utils::getId(), 100.0, -2.0 * half_width, 0.0));

  lanelet::Lanelet lanelet_a(lanelet::utils::getId(), left_bound_ls, shared_bound_ls);
  lanelet_a.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
  lanelet_a.attributes()[lanelet::AttributeName::Location] = lanelet::AttributeValueString::Urban;
  lanelet_a.attributes()[lanelet::AttributeName::OneWay] = "yes";

  lanelet::Lanelet lanelet_b(lanelet::utils::getId(), shared_bound_ls, right_far_bound_ls);
  lanelet_b.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
  lanelet_b.attributes()[lanelet::AttributeName::Location] = lanelet::AttributeValueString::Urban;
  lanelet_b.attributes()[lanelet::AttributeName::OneWay] = "yes";

  const auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(lanelet_a);
  map->add(lanelet_b);

  autoware_map_msgs::msg::LaneletMapBin map_bin_msg;
  map_bin_msg.header.frame_id = "map";
  lanelet::utils::conversion::toBinMsg(map, &map_bin_msg);

  auto route_handler = std::make_shared<autoware::route_handler::RouteHandler>();
  route_handler->setMap(map_bin_msg);

  // IMPORTANT: setMap() deserializes the map, so the RoutingGraph is built over brand-new
  // Lanelet/LineString primitive instances (same ids, different underlying data pointers) --
  // NOT the `lanelet_a` / `lanelet_b` locals constructed above. Routing-graph lookups
  // (right()/left()/adjacentRight()/adjacentLeft(), all used by getRightLanelet()) key on the
  // primitive identity backing the graph, so any lanelet passed into those queries (including
  // `object.overhang_lanelet` and `data.current_lanelets`) must be re-fetched from the route
  // handler's own map, exactly as production code does (e.g. via getLaneletsFromId()/
  // getClosestLaneletWithinRoute()) -- never a locally-constructed lanelet with matching id.
  const auto ego_lanelet = route_handler->getLaneletsFromId(lanelet_a.id());
  route_handler->setRouteLanelets(lanelet::ConstLanelets{ego_lanelet});
  planner_data->route_handler = route_handler;

  // Sanity check: B is indeed discoverable as A's right neighbor, and is NOT a shoulder -- this
  // is exactly the "must be on edge lane" trap the removed code fell into.
  const auto right_lane = route_handler->getRightLanelet(ego_lanelet, true, true);
  ASSERT_TRUE(right_lane.has_value());
  EXPECT_NE(right_lane.value().attribute(lanelet::AttributeName::Subtype).value(), "road_shoulder");

  AvoidancePlanningData data;
  data.current_lanelets = {ego_lanelet};

  // Pedestrian standing mid-lane: 0.3m right of centerline, well clear of both bounds (1.45m of
  // clearance to the right/shared bound, 2.05m to the left bound) -- not "near the edge" by any
  // reasonable definition.
  constexpr double object_y = -0.3;
  const auto object_pose = autoware::test_utils::createPose(10.0, object_y, 0.0, 0.0, 0.0, 0.0);

  ObjectData object_data;
  object_data.object.classification.emplace_back(
    autoware_perception_msgs::build<ObjectClassification>()
      .label(ObjectClassification::PEDESTRIAN)
      .probability(1.0));
  object_data.object.kinematics.initial_pose_with_covariance.pose = object_pose;
  object_data.overhang_lanelet = ego_lanelet;
  object_data.overhang_points.emplace_back(0.0, object_pose.position);
  object_data.direction = Direction::RIGHT;

  EXPECT_TRUE(filtering_utils::isSatisfiedWithNonVehicleCondition(
    object_data, data, planner_data, parameters));
  // is_on_ego_lane is still computed/populated (used elsewhere); confirm it reads correctly too.
  EXPECT_TRUE(object_data.is_on_ego_lane);
}

TEST(TestUtils, getNominalReturnPrepareDistanceIsDecoupledFromPrepareDistance)
{
  // The post-object "hold distance" (used by addReturnShiftLine()) must be independently
  // configurable from the before-object "prepare distance" (used by every other call site of
  // getNominalPrepareDistance()). Confirm the two accessors read distinct parameters and can
  // diverge when only one side is tuned.
  auto parameters = get_parameters();
  const auto planner_data = get_planner_data();

  auto helper = std::make_shared<helper::static_obstacle_avoidance::AvoidanceHelper>(parameters);
  helper->setData(planner_data);

  // Sanity check: with return params left at their (unset) defaults, the two accessors differ
  // from the pre-object prepare distance, proving they are not silently aliased to the same
  // parameter fields.
  parameters->min_return_prepare_distance = 0.0;
  parameters->max_return_prepare_time = 0.0;
  EXPECT_NE(helper->getNominalReturnPrepareDistance(), helper->getNominalPrepareDistance());

  // When mirrored to the same values as min_prepare_distance/max_prepare_time, behavior must be
  // identical (this is the default-identical-behavior guarantee).
  parameters->min_return_prepare_distance = parameters->min_prepare_distance;
  parameters->max_return_prepare_time = parameters->max_prepare_time;
  EXPECT_DOUBLE_EQ(
    helper->getNominalReturnPrepareDistance(), helper->getNominalPrepareDistance());

  // Raising only the return-specific params must change getNominalReturnPrepareDistance() while
  // leaving getNominalPrepareDistance() (the before-object distance) completely untouched.
  const auto prepare_distance_before = helper->getNominalPrepareDistance();
  parameters->min_return_prepare_distance = parameters->min_prepare_distance + 5.0;
  parameters->max_return_prepare_time = parameters->max_prepare_time + 5.0;
  EXPECT_DOUBLE_EQ(helper->getNominalPrepareDistance(), prepare_distance_before);
  EXPECT_GT(helper->getNominalReturnPrepareDistance(), prepare_distance_before);
}

namespace
{
// Helper shared by the "always avoid" / "mid-approach reversion" tests below: builds a minimal,
// self-consistent AvoidanceHelper + ShiftLineGenerator pair, and a TRUCK-classified ObjectData
// positioned directly ahead of ego, on the right, with no pre-existing approved shift.
struct AlwaysAvoidTestFixture
{
  std::shared_ptr<AvoidanceParameters> parameters;
  std::shared_ptr<PlannerData> planner_data;
  std::shared_ptr<helper::static_obstacle_avoidance::AvoidanceHelper> helper;
  ShiftLineGenerator generator;

  explicit AlwaysAvoidTestFixture(const double vehicle_width)
  : parameters(get_parameters()),
    planner_data(get_planner_data()),
    helper(std::make_shared<helper::static_obstacle_avoidance::AvoidanceHelper>(parameters)),
    generator(parameters)
  {
    // Keep distance computations independent of vehicle geometry not populated by
    // get_planner_data().
    parameters->consider_front_overhang = false;
    parameters->consider_rear_overhang = false;
    // Force the "reliable" deceleration policy so the *unbypassed* path takes the hard-refusal
    // branch instead of silently relaxing (best_effort would relax regardless of the new flag,
    // which would not isolate the behavior under test).
    parameters->policy_deceleration = "reliable";
    planner_data->parameters.vehicle_width = vehicle_width;

    helper->setData(planner_data);
    generator.setHelper(helper);
    generator.setData(planner_data);

    // Minimal single-point "previous cycle" path so AvoidanceHelper::getShift()/getEgoShift()
    // (validated against matching point counts) do not throw, and report a fixed zero shift (no
    // pre-existing approved maneuver) regardless of query position.
    PathWithLaneId prev_path;
    PathPointWithLaneId p;
    p.point.pose = autoware::test_utils::createPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    prev_path.points.push_back(p);
    ShiftedPath zero_shift;
    zero_shift.path = prev_path;
    zero_shift.shift_length = {0.0};
    helper->setPreviousReferencePath(prev_path);
    helper->setPreviousSplineShiftPath(zero_shift);
    helper->setPreviousLinearShiftPath(zero_shift);
  }

  ObjectData make_object(const double longitudinal) const
  {
    ObjectData object_data;
    object_data.object.classification.emplace_back(
      autoware_perception_msgs::build<ObjectClassification>()
        .label(ObjectClassification::TRUCK)
        .probability(1.0));
    object_data.direction = Direction::RIGHT;
    object_data.overhang_points.emplace_back(0.0, Point{});
    object_data.longitudinal = longitudinal;
    object_data.is_parked = false;
    object_data.is_stoppable = false;
    return object_data;
  }
};
}  // namespace

// Part 1 root-cause regression: an object whose remaining longitudinal distance has shrunk below
// what shift_line_generator's "reliable" deceleration policy would normally accept (a hard
// nullopt/INSUFFICIENT_LONGITUDINAL_DISTANCE refusal -- the exact mechanism that, combined with
// getCurrentModuleState()'s cancel-on-shrinking-distance transition, produced the "avoids far,
// reverts to centerline close" symptom) must NOT be silently reverted once it is already
// "committed" (was found avoidable in a previous cycle and still has a valid lateral avoid_margin)
// -- confirming the sticky bypass in ShiftLineGenerator::computeFeasibleShiftProfile().
TEST(TestUtils, ComputeFeasibleShiftProfileMidApproachReversionIsPrevented)
{
  AlwaysAvoidTestFixture fixture(/*vehicle_width=*/0.5);

  constexpr double desire_shift_length = 1.5;
  const auto avoiding_shift = desire_shift_length;  // current_ego_shift == 0

  // Pick a longitudinal distance whose remaining avoidance room is comfortably inside the
  // "reliable policy hard refusal" zone (< getMinAvoidanceDistance(avoiding_shift)), but not so
  // small that the achievable jerk-limited shift collapses below the execution threshold --
  // derived from the helper's own distance formula rather than a hardcoded magic number.
  const auto min_avoidance_distance = fixture.helper->getMinAvoidanceDistance(avoiding_shift);
  ASSERT_GT(min_avoidance_distance, 0.0);
  const auto remaining_distance = 0.9 * min_avoidance_distance;

  const auto prepare_distance = fixture.helper->getNominalPrepareDistance();
  auto object = fixture.make_object(prepare_distance + remaining_distance);

  // Not yet committed, and the global override is off: the "reliable" policy refuses outright.
  object.is_avoidance_committed = false;
  const auto not_committed_result =
    fixture.generator.computeFeasibleShiftProfile(object, desire_shift_length, 0.0);
  EXPECT_FALSE(not_committed_result.has_value());

  // Same object, same shrunk distance, but now marked "committed" (as it would be after a
  // previous cycle found it avoidable) -- must still produce a feasible shift instead of
  // reverting to "no avoidance".
  object.is_avoidance_committed = true;
  object.info = ObjectInfo::NONE;
  const auto committed_result =
    fixture.generator.computeFeasibleShiftProfile(object, desire_shift_length, 0.0);
  ASSERT_TRUE(committed_result.has_value());
  EXPECT_GT(std::abs(committed_result.value().first), 0.0);
}

// Same scenario, but driven by the global `always_avoid_if_geometrically_possible` override
// instead of the per-object sticky commitment -- confirms the flag bypasses the longitudinal gate
// when true, and that default (false) behavior is unchanged (gated).
TEST(TestUtils, AlwaysAvoidFlagBypassesLongitudinalGate)
{
  AlwaysAvoidTestFixture fixture(/*vehicle_width=*/0.5);

  constexpr double desire_shift_length = 1.5;
  const auto avoiding_shift = desire_shift_length;

  const auto min_avoidance_distance = fixture.helper->getMinAvoidanceDistance(avoiding_shift);
  const auto remaining_distance = 0.9 * min_avoidance_distance;
  const auto prepare_distance = fixture.helper->getNominalPrepareDistance();

  // default: flag off -> gated (nullopt).
  {
    fixture.parameters->always_avoid_if_geometrically_possible = false;
    auto object = fixture.make_object(prepare_distance + remaining_distance);
    const auto result = fixture.generator.computeFeasibleShiftProfile(object, desire_shift_length, 0.0);
    EXPECT_FALSE(result.has_value());
  }

  // flag on -> bypassed (feasible shift produced).
  {
    fixture.parameters->always_avoid_if_geometrically_possible = true;
    auto object = fixture.make_object(prepare_distance + remaining_distance);
    const auto result = fixture.generator.computeFeasibleShiftProfile(object, desire_shift_length, 0.0);
    ASSERT_TRUE(result.has_value());
    EXPECT_GT(std::abs(result.value().first), 0.0);
  }
}

// The lateral room-availability check is the one physical safety boundary that must never be
// bypassed by the new flag: (a) getAvoidMargin() returning nullopt for a genuinely too-narrow road
// is untouched by the flag (it doesn't even read it), and (b) computeFeasibleShiftProfile's final
// hard lateral-margin geometric feasibility check still refuses when the vehicle is simply too
// wide to fit, regardless of the flag.
TEST(TestUtils, AlwaysAvoidFlagNeverBypassesLateralRoomCheck)
{
  // (a) getAvoidMargin(): narrow-road "not enough room" case from the existing getAvoidMargin
  // test, replayed with the new flag turned on.
  {
    auto parameters = get_parameters();
    parameters->always_avoid_if_geometrically_possible = true;
    const auto planner_data = get_planner_data();

    ObjectData object_data;
    object_data.is_parked = true;
    object_data.distance_factor = 1.0;
    object_data.to_road_shoulder_distance = 2.5;
    object_data.object.classification.emplace_back(
      autoware_perception_msgs::build<ObjectClassification>()
        .label(ObjectClassification::TRUCK)
        .probability(1.0));

    const auto output = filtering_utils::getAvoidMargin(object_data, planner_data, parameters);
    EXPECT_FALSE(output.has_value());
  }

  // (b) computeFeasibleShiftProfile(): even with the global override on and the object marked
  // "committed", a vehicle too wide to physically fit beside the object (hard lateral margin
  // check fails) must still be refused.
  {
    // A large vehicle_width forces the final hard-margin geometric feasibility check to fail
    // regardless of how much shift is jerk-feasible.
    AlwaysAvoidTestFixture fixture(/*vehicle_width=*/50.0);
    fixture.parameters->always_avoid_if_geometrically_possible = true;

    constexpr double desire_shift_length = 1.5;
    const auto avoiding_shift = desire_shift_length;
    const auto min_avoidance_distance = fixture.helper->getMinAvoidanceDistance(avoiding_shift);
    const auto remaining_distance = 0.9 * min_avoidance_distance;
    const auto prepare_distance = fixture.helper->getNominalPrepareDistance();

    auto object = fixture.make_object(prepare_distance + remaining_distance);
    object.is_avoidance_committed = true;

    const auto result = fixture.generator.computeFeasibleShiftProfile(object, desire_shift_length, 0.0);
    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(object.info, ObjectInfo::NEED_DECELERATION);
  }
}

// fillObjectAvoidanceCommitted(): the sticky flag persists across cycles as long as
// avoid_margin is still valid, and is dropped the moment lateral room is genuinely lost.
TEST(TestUtils, FillObjectAvoidanceCommittedStickyBehavior)
{
  // No matching previous object -> not committed.
  {
    ObjectData current;
    current.object.object_id = generate_uuid();
    current.avoid_margin = 0.5;
    ObjectDataArray previous;
    fillObjectAvoidanceCommitted(current, previous);
    EXPECT_FALSE(current.is_avoidance_committed);
  }

  // Previous object was avoidable, current still has room -> becomes committed.
  {
    ObjectData current;
    current.object.object_id = generate_uuid();
    current.avoid_margin = 0.5;

    ObjectData previous_obj;
    previous_obj.object.object_id = current.object.object_id;
    previous_obj.is_avoidable = true;
    previous_obj.is_avoidance_committed = false;
    ObjectDataArray previous{previous_obj};

    fillObjectAvoidanceCommitted(current, previous);
    EXPECT_TRUE(current.is_avoidance_committed);
  }

  // Previous object was already committed (sticky) -> stays committed even if it wasn't
  // is_avoidable in the immediately preceding cycle (e.g. it was rescued via the is_approved()
  // path rather than the nominal one).
  {
    ObjectData current;
    current.object.object_id = generate_uuid();
    current.avoid_margin = 0.5;

    ObjectData previous_obj;
    previous_obj.object.object_id = current.object.object_id;
    previous_obj.is_avoidable = false;
    previous_obj.is_avoidance_committed = true;
    ObjectDataArray previous{previous_obj};

    fillObjectAvoidanceCommitted(current, previous);
    EXPECT_TRUE(current.is_avoidance_committed);
  }

  // Lateral room genuinely lost this cycle (avoid_margin == nullopt) -> commitment is dropped
  // regardless of history. This is the one check that must never be bypassed.
  {
    ObjectData current;
    current.object.object_id = generate_uuid();
    current.avoid_margin = std::nullopt;

    ObjectData previous_obj;
    previous_obj.object.object_id = current.object.object_id;
    previous_obj.is_avoidable = true;
    previous_obj.is_avoidance_committed = true;
    ObjectDataArray previous{previous_obj};

    fillObjectAvoidanceCommitted(current, previous);
    EXPECT_FALSE(current.is_avoidance_committed);
  }
}

// AvoidanceHelper::isReady()'s wait-and-see gate: the always-avoid flag bypasses the delayed-start
// behavior for MERGING/DEVIATING vehicles once a valid avoid_margin exists; default behavior
// (flag off) is unchanged.
TEST(TestUtils, AlwaysAvoidFlagBypassesWaitAndSeeGate)
{
  auto parameters = get_parameters();
  parameters->wait_and_see_target_behaviors = {"MERGING"};
  parameters->wait_and_see_th_closest_distance = 0.0;

  const auto planner_data = get_planner_data();
  auto helper = std::make_shared<helper::static_obstacle_avoidance::AvoidanceHelper>(parameters);
  helper->setData(planner_data);

  ObjectData object_data;
  object_data.object.classification.emplace_back(
    autoware_perception_msgs::build<ObjectClassification>()
      .label(ObjectClassification::TRUCK)
      .probability(1.0));
  object_data.direction = Direction::RIGHT;
  // Unbiased scenario: true-lane-relative side agrees with the path-relative one above. isReady()
  // now uses is_on_right_of_true_lane (see the calcShiftLength frame-mismatch fix) instead of
  // isOnRight(object)/direction, so it must be set explicitly here to match.
  object_data.is_on_right_of_true_lane = true;
  object_data.overhang_points.emplace_back(0.0, Point{});
  object_data.is_ambiguous = true;
  object_data.behavior = ObjectData::Behavior::MERGING;
  object_data.avoid_margin = 0.5;
  // Far away: nominal wait-and-see logic says "not ready yet" (waits until closer).
  object_data.longitudinal = helper->getNominalPrepareDistance(0.0) +
                              helper->getFrontConstantDistance(object_data) +
                              helper->getMinAvoidanceDistance(1.0) + 100.0;

  parameters->always_avoid_if_geometrically_possible = false;
  {
    const auto [ready, ambiguous] = helper->isReady(ObjectDataArray{object_data});
    EXPECT_FALSE(ready);
    EXPECT_FALSE(ambiguous);
  }

  parameters->always_avoid_if_geometrically_possible = true;
  {
    const auto [ready, ambiguous] = helper->isReady(ObjectDataArray{object_data});
    EXPECT_TRUE(ready);
    EXPECT_FALSE(ambiguous);
  }
}

// isNoNeedAvoidanceBehavior()'s calcShiftLength() frame-mismatch fix: object.avoid_margin is
// computed relative to the TRUE lane (getAvoidMargin()/getRoadShoulderDistance(), bias-independent
// -- see the getRoadShoulderDistance fix), but object.overhang_points/direction (isOnRight()) are
// computed relative to the CURRENT reference path, which can be laterally biased away from true
// center (e.g. by a prefer_lateral_ratio lanelet tag). Before this fix, calcShiftLength() combined
// avoid_margin with the path-relative side to decide whether to add or subtract the margin; when
// the path bias makes the path-relative side disagree with the true-lane side, this produces a
// wrong-signed (here: numerically-cancelled, near-zero) shift_length even for an object genuinely
// blocking the TRUE driving line. Fixed by introducing object.is_on_right_of_true_lane (populated
// once, bias-independent, alongside to_road_shoulder_distance/avoid_margin) and using it instead
// of isOnRight(object)/direction wherever it is combined with avoid_margin.
TEST(TestUtils, isNoNeedAvoidanceBehaviorTrueLaneRelativeShiftLength)
{
  const auto parameters = get_parameters();  // lateral_execution_threshold = 0.5

  // Live-bug repro: a pedestrian genuinely blocking the TRUE path produces a healthy, correctly
  // true-lane-relative avoid_margin (1.81 m, matching the recorded-bag value), but the current
  // (biased) reference path happens to sit almost exactly `avoid_margin` away from the object on
  // its near-edge/path-relative measurement -- so old code (mixing true-lane avoid_margin with
  // path-relative side) computed shift_length = overhang_dist - avoid_margin = 1.81 - 1.81 = 0.0,
  // misclassified LESS_THAN_EXECUTION_THRESHOLD ("no need to shift") despite the object truly
  // blocking the path.
  {
    ObjectData object_data;
    object_data.avoid_margin = 1.81;
    // Path-relative: the biased reference path is far enough right that the object reads as being
    // on its LEFT (isOnRight(object) == false via direction == LEFT).
    object_data.direction = Direction::LEFT;
    // True-lane-relative: the object is actually on the RIGHT of the true lane centerline --
    // this is what getAvoidMargin() assumed when picking the road bound to measure margin
    // against, and disagrees with the path-relative classification above.
    object_data.is_on_right_of_true_lane = true;
    // Path-relative near-edge overhang distance: numerically equal to avoid_margin, which is
    // exactly what causes the old (buggy) subtract-branch math to cancel to ~0.
    object_data.overhang_points.emplace_back(1.81, Point{});

    EXPECT_FALSE(filtering_utils::isNoNeedAvoidanceBehavior(object_data, parameters))
      << "genuinely-blocking object must not be dropped as ENOUGH_LATERAL_DISTANCE / "
         "LESS_THAN_EXECUTION_THRESHOLD just because the biased path's path-relative side "
         "disagrees with the object's true-lane side";
    EXPECT_NE(object_data.info, ObjectInfo::ENOUGH_LATERAL_DISTANCE);
    EXPECT_NE(object_data.info, ObjectInfo::LESS_THAN_EXECUTION_THRESHOLD);
  }

  // Regression (unbiased path): path-relative and true-lane-relative side agree, so behavior must
  // be identical to before this fix. Mirrors the same avoid_margin/overhang_dist magnitudes as the
  // bug repro above, but with both sides consistently RIGHT -- add-branch math correctly yields a
  // healthy positive shift_length, so avoidance is (still) required.
  {
    ObjectData object_data;
    object_data.avoid_margin = 1.81;
    object_data.direction = Direction::RIGHT;
    object_data.is_on_right_of_true_lane = true;
    object_data.overhang_points.emplace_back(0.0, Point{});

    EXPECT_FALSE(filtering_utils::isNoNeedAvoidanceBehavior(object_data, parameters));
  }

  // Regression (unbiased path, genuinely sufficient clearance): both sides agree the object is on
  // the right and already comfortably clear of the required margin -- must still resolve to
  // ENOUGH_LATERAL_DISTANCE exactly as before this fix.
  {
    ObjectData object_data;
    object_data.avoid_margin = 1.81;
    object_data.direction = Direction::RIGHT;
    object_data.is_on_right_of_true_lane = true;
    object_data.overhang_points.emplace_back(-3.0, Point{});

    EXPECT_TRUE(filtering_utils::isNoNeedAvoidanceBehavior(object_data, parameters));
    EXPECT_EQ(object_data.info, ObjectInfo::ENOUGH_LATERAL_DISTANCE);
  }
}
}  // namespace autoware::behavior_path_planner::utils::static_obstacle_avoidance
