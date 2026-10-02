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

  // [ALWAYS-AVOID-DIRECTIVE 2026-09-03] This case used to be "road width is not enough" -> reject
  // (EXPECT_FALSE), back when Step1's feasibility gate compared against the full
  // lateral_hard_margin_for_parked_vehicle (0.7 here). Per explicit user directive (see
  // docs/research/always-avoid-constraint-removal.md), that gate now only requires
  // kRelaxedHardMarginFloor (0.03 m) of clearance beyond half the vehicle width, so this same
  // to_road_shoulder_distance (2.5) is now geometrically feasible: hard_lateral_distance_limit =
  // 2.5 - 0.1(hard_drivable_bound_margin) - 1.0(0.5*vehicle_width) = 1.4, which is now >= 1.03
  // (0.03 + 1.0), so it no longer trips the Step1 gate. Falls through to Step2 (soft_distance_limit
  // 1.0 < the *unrelaxed* min_avoid_margin 1.7, since Step2's sizing math was intentionally left
  // untouched), which returns min_avoid_margin (1.7) as before.
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
    ASSERT_TRUE(output.has_value());
    EXPECT_DOUBLE_EQ(output.value(), 1.7);
  }

  // [ALWAYS-AVOID-DIRECTIVE 2026-09-03] The Step1 gate still rejects when there is truly no
  // physical room -- the relaxed floor (kRelaxedHardMarginFloor = 0.03 m) is a small non-zero
  // floor, not a full removal. With to_road_shoulder_distance = 0.5:
  // hard_lateral_distance_limit = 0.5 - 0.1 - 1.0 = -0.6, well below 1.03, so this still rejects.
  {
    ObjectData object_data;
    object_data.is_parked = true;
    object_data.distance_factor = 1.0;
    object_data.to_road_shoulder_distance = 0.5;
    object_data.object.classification.emplace_back(
      autoware_perception_msgs::build<ObjectClassification>()
        .label(ObjectClassification::TRUCK)
        .probability(1.0));

    const auto output = filtering_utils::getAvoidMargin(object_data, planner_data, parameters);
    EXPECT_FALSE(output.has_value());
  }
}

TEST(TestUtils, getAvoidMarginNarrowLaneLanelet)
{
  const auto parameters = get_parameters();
  const auto planner_data = get_planner_data();

  // Build a small lanelet map with a single lanelet tagged `narrow_lane=yes`, straddling the
  // object position used below.
  lanelet::LineString3d left_bound_ls(lanelet::utils::getId());
  left_bound_ls.push_back(lanelet::Point3d(lanelet::utils::getId(), 0.0, 1.75, 0.0));
  left_bound_ls.push_back(lanelet::Point3d(lanelet::utils::getId(), 100.0, 1.75, 0.0));

  lanelet::LineString3d right_bound_ls(lanelet::utils::getId());
  right_bound_ls.push_back(lanelet::Point3d(lanelet::utils::getId(), 0.0, -1.75, 0.0));
  right_bound_ls.push_back(lanelet::Point3d(lanelet::utils::getId(), 100.0, -1.75, 0.0));

  lanelet::Lanelet narrow_lanelet(lanelet::utils::getId(), left_bound_ls, right_bound_ls);
  narrow_lanelet.attributes()[lanelet::AttributeName::Subtype] =
    lanelet::AttributeValueString::Road;
  narrow_lanelet.attributes()["narrow_lane"] = "yes";

  const auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(narrow_lanelet);

  autoware_map_msgs::msg::LaneletMapBin map_bin_msg;
  map_bin_msg.header.frame_id = "map";
  lanelet::utils::conversion::toBinMsg(map, &map_bin_msg);

  auto route_handler = std::make_shared<autoware::route_handler::RouteHandler>();
  route_handler->setMap(map_bin_msg);
  planner_data->route_handler = route_handler;

  // object well inside the narrow_lane-tagged lanelet's polygon -> forced nullopt (no in-lane
  // shift), regardless of otherwise-favorable road width.
  {
    ObjectData object_data;
    object_data.is_parked = false;
    object_data.distance_factor = 1.0;
    object_data.to_road_shoulder_distance = 5.0;
    object_data.object.classification.emplace_back(
      autoware_perception_msgs::build<ObjectClassification>()
        .label(ObjectClassification::TRUCK)
        .probability(1.0));
    object_data.object.kinematics.initial_pose_with_covariance.pose =
      autoware::test_utils::createPose(50.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    const auto output = filtering_utils::getAvoidMargin(object_data, planner_data, parameters);
    EXPECT_FALSE(output.has_value());
  }

  // control case: object in a normal (untagged) location -> unaffected, behaves as before.
  {
    ObjectData object_data;
    object_data.is_parked = false;
    object_data.distance_factor = 1.0;
    object_data.to_road_shoulder_distance = 5.0;
    object_data.object.classification.emplace_back(
      autoware_perception_msgs::build<ObjectClassification>()
        .label(ObjectClassification::TRUCK)
        .probability(1.0));
    object_data.object.kinematics.initial_pose_with_covariance.pose =
      autoware::test_utils::createPose(500.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    const auto output = filtering_utils::getAvoidMargin(object_data, planner_data, parameters);
    ASSERT_TRUE(output.has_value());
    EXPECT_DOUBLE_EQ(output.value(), 1.9);
  }
}

TEST(TestUtils, getAvoidMarginDisableStaticAvoidanceLanelet)
{
  const auto parameters = get_parameters();
  const auto planner_data = get_planner_data();

  // Two lanelets side by side along x: [0, 100] tagged disable_static_avoidance=true and
  // [100, 200] tagged disable_static_avoidance=false (an explicit false behaves as untagged).
  const auto make_lanelet = [](const double x0, const double x1, const std::string & value) {
    lanelet::LineString3d left(lanelet::utils::getId());
    left.push_back(lanelet::Point3d(lanelet::utils::getId(), x0, 1.75, 0.0));
    left.push_back(lanelet::Point3d(lanelet::utils::getId(), x1, 1.75, 0.0));
    lanelet::LineString3d right(lanelet::utils::getId());
    right.push_back(lanelet::Point3d(lanelet::utils::getId(), x0, -1.75, 0.0));
    right.push_back(lanelet::Point3d(lanelet::utils::getId(), x1, -1.75, 0.0));
    lanelet::Lanelet ll(lanelet::utils::getId(), left, right);
    ll.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
    ll.attributes()["disable_static_avoidance"] = value;
    return ll;
  };

  const auto tagged = make_lanelet(0.0, 100.0, "true");
  const auto map = std::make_shared<lanelet::LaneletMap>();
  map->add(tagged);
  map->add(make_lanelet(100.0, 200.0, "false"));

  autoware_map_msgs::msg::LaneletMapBin map_bin_msg;
  map_bin_msg.header.frame_id = "map";
  lanelet::utils::conversion::toBinMsg(map, &map_bin_msg);

  auto route_handler = std::make_shared<autoware::route_handler::RouteHandler>();
  route_handler->setMap(map_bin_msg);
  planner_data->route_handler = route_handler;

  const auto object_at = [](const double x) {
    ObjectData object_data;
    object_data.is_parked = false;
    object_data.distance_factor = 1.0;
    object_data.to_road_shoulder_distance = 5.0;
    object_data.object.classification.emplace_back(
      autoware_perception_msgs::build<ObjectClassification>()
        .label(ObjectClassification::TRUCK)
        .probability(1.0));
    object_data.object.kinematics.initial_pose_with_covariance.pose =
      autoware::test_utils::createPose(x, 0.0, 0.0, 0.0, 0.0, 0.0);
    return object_data;
  };

  // inside the tagged lanelet -> not avoidable, regardless of otherwise-favorable road width.
  EXPECT_FALSE(
    filtering_utils::getAvoidMargin(object_at(50.0), planner_data, parameters).has_value());

  // centre outside every lanelet (road edge), but the lane it blocks is tagged -> not avoidable.
  {
    auto object_data = object_at(50.0);
    object_data.object.kinematics.initial_pose_with_covariance.pose.position.y = 2.0;
    object_data.overhang_lanelet = map->laneletLayer.get(tagged.id());
    EXPECT_FALSE(
      filtering_utils::getAvoidMargin(object_data, planner_data, parameters).has_value());
  }

  // inside the lanelet tagged false -> unaffected.
  const auto output = filtering_utils::getAvoidMargin(object_at(150.0), planner_data, parameters);
  ASSERT_TRUE(output.has_value());
  EXPECT_DOUBLE_EQ(output.value(), 1.9);
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

  // [ALWAYS-AVOID-DIRECTIVE 2026-09-03] This case used to be "farther than goal position" ->
  // reject (FURTHER_THAN_GOAL). That gate was removed per explicit user directive (see
  // docs/research/always-avoid-constraint-removal.md), so this object -- otherwise a normal
  // within-forward-detection-range object -- is now accepted.
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

    EXPECT_TRUE(
      filtering_utils::isSatisfiedWithCommonCondition(
        object_data, path, forward_detection_range, 4.0, create_point(0.0, 0.0, 0.0), false,
        parameters));
  }

  // [ALWAYS-AVOID-DIRECTIVE 2026-09-03] This case used to be "too near to goal" -> reject
  // (TOO_NEAR_TO_GOAL). That gate was removed per explicit user directive (see
  // docs/research/always-avoid-constraint-removal.md), so this object is now accepted.
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
        object_data, path, forward_detection_range, 6.4, create_point(0.0, 0.0, 0.0), false,
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

// Live-bug repro for generateAvoidOutline()'s SAME_DIRECTION_SHIFT (info=17) false positive.
//
// Root cause: isSameDirectionShift(is_right, shift) is, by construction, the exact logical
// complement of isShiftNecessary(is_right, shift) for every (is_right, shift) pair except the
// single degenerate case shift == 0.0 with is_right == false (both are true there). This means
// is_same_direction_shift can ONLY be true for an object whose (is_on_right_of_true_lane,
// overhang_dist, avoid_margin) triple would ALSO fail isNoNeedAvoidanceBehavior()'s
// ENOUGH_LATERAL_DISTANCE check -- using the exact same inputs. Every object reaching
// generateAvoidOutline() already passed that exact check once in filterTargetObjects().
//
// The only way an object reaches generateAvoidOutline() with is_same_direction_shift == true is
// for avoid_margin to have been re-derived afterward (by updateRoadShoulderDistance(), which runs
// on every already-accepted target_object every cycle) against a now-stale overhang_dist -- e.g.
// a second object narrows the usable road-shoulder width, so avoid_margin shrinks from an
// original value that made the shift "necessary" down to a smaller value for which the object's
// (unchanged) real overhang distance already provides enough clearance.
//
// This test reproduces that exact narrowing with concrete numbers and confirms
// isShiftNecessary()/isSameDirectionShift() land in the same "already clear, not a wrong-signed
// shift" combination that shift_line_generator.cpp's generateAvoidOutline() now special-cases
// (via is_shift_still_necessary) to ObjectInfo::ENOUGH_LATERAL_DISTANCE instead of the old,
// unconditional ObjectInfo::SAME_DIRECTION_SHIFT -- which callers treat as "absolutely not
// avoidable" and which can `break` outline generation for every target object behind this one.
TEST(TestUtils, sameDirectionShiftFalsePositiveAfterMarginNarrowing)
{
  ObjectData object;
  object.direction = Direction::LEFT;  // path-frame side, unused directly here
  const bool is_on_right_of_true_lane = false;  // object is on the LEFT of the true lane
  const double overhang_dist = 0.30;            // fixed: object measured 0.30 m left of the path

  // Cycle N: object first accepted into target_objects with a generous avoid_margin (e.g. no
  // other object yet narrows the road-shoulder). shift = overhang - margin = 0.30 - 0.50 = -0.20
  // -> necessary (shift toward the right, away from the left-side object), NOT same-direction.
  {
    const double avoid_margin_original = 0.50;
    const double shift = calcShiftLength(is_on_right_of_true_lane, overhang_dist, avoid_margin_original);
    EXPECT_TRUE(isShiftNecessary(is_on_right_of_true_lane, shift));
    EXPECT_FALSE(isSameDirectionShift(is_on_right_of_true_lane, shift));
  }

  // Cycle N+1: updateRoadShoulderDistance() re-derives avoid_margin (e.g. a second object now
  // clips the drivable road-shoulder on this side), narrowing it to 0.20 m -- still a valid
  // ("Some") margin, just smaller. overhang_dist (captured once, at filter time) is unchanged.
  // shift = 0.30 - 0.20 = +0.10 -> the object's real clearance (0.30 m) now already exceeds the
  // (smaller) required margin: genuinely no longer necessary to shift for this object.
  {
    const double avoid_margin_narrowed = 0.20;
    const double shift = calcShiftLength(is_on_right_of_true_lane, overhang_dist, avoid_margin_narrowed);

    // Old (buggy) classification: isSameDirectionShift() alone says "same direction" (bad) even
    // though the object is genuinely, comfortably clear.
    EXPECT_TRUE(isSameDirectionShift(is_on_right_of_true_lane, shift))
      << "demonstrates the exact false-positive raw signal generateAvoidOutline() used to trust "
         "unconditionally";

    // isShiftNecessary() -- the same check isNoNeedAvoidanceBehavior() uses -- correctly says this
    // is the harmless ENOUGH_LATERAL_DISTANCE condition, not a genuine same-direction hazard.
    EXPECT_FALSE(isShiftNecessary(is_on_right_of_true_lane, shift));

    // The fix's guard: is_same_direction_shift is only trusted when the shift is still necessary.
    const bool is_shift_still_necessary = isShiftNecessary(is_on_right_of_true_lane, shift);
    const bool is_same_direction_shift =
      is_shift_still_necessary && isSameDirectionShift(is_on_right_of_true_lane, shift);
    EXPECT_FALSE(is_shift_still_necessary);
    EXPECT_FALSE(is_same_direction_shift)
      << "after the fix, a stale/narrowed margin must degrade to a harmless "
         "ENOUGH_LATERAL_DISTANCE skip, not a false SAME_DIRECTION_SHIFT/absolutely-not-avoidable";
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

  // [ALWAYS-AVOID-DIRECTIVE 2026-09-03] always_avoid_if_geometrically_possible now defaults to
  // true (see docs/research/always-avoid-constraint-removal.md). This test specifically wants to
  // isolate "not committed AND global override off" behavior, so force it off explicitly rather
  // than relying on the struct default.
  fixture.parameters->always_avoid_if_geometrically_possible = false;

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
  // (a) getAvoidMargin(): [ALWAYS-AVOID-DIRECTIVE 2026-09-03] This sub-case used to replay the
  // "not enough room" case from the getAvoidMargin test at to_road_shoulder_distance = 2.5 and
  // expect a reject. Per explicit user directive (see
  // docs/research/always-avoid-constraint-removal.md), getAvoidMargin()'s Step1 gate now uses a
  // small non-zero floor (kRelaxedHardMarginFloor = 0.03 m) instead of the full
  // lateral_hard_margin_for_parked_vehicle, so to_road_shoulder_distance = 2.5 is now
  // geometrically feasible regardless of the always_avoid flag (this gate doesn't even read that
  // flag -- it never did). Replaced with a genuinely-tight case (to_road_shoulder_distance = 0.5)
  // to keep verifying the flag truly never bypasses the lateral room check, and that the room
  // check itself still refuses when there is truly no physical space (the floor is relaxed, not
  // removed).
  {
    auto parameters = get_parameters();
    parameters->always_avoid_if_geometrically_possible = true;
    const auto planner_data = get_planner_data();

    ObjectData object_data;
    object_data.is_parked = true;
    object_data.distance_factor = 1.0;
    object_data.to_road_shoulder_distance = 0.5;
    object_data.object.classification.emplace_back(
      autoware_perception_msgs::build<ObjectClassification>()
        .label(ObjectClassification::TRUCK)
        .probability(1.0));

    const auto output = filtering_utils::getAvoidMargin(object_data, planner_data, parameters);
    EXPECT_FALSE(output.has_value());
  }

  // (b) computeFeasibleShiftProfile(): even with the global override on and the object marked
  // "committed", a vehicle too wide to physically fit beside the object (hard lateral margin
  // check fails) must still be refused -- eventually. [REV-F2 fix 2026-09-03] committed objects
  // now get a small consecutive-cycle debounce on this specific check (see
  // ObjectData::hard_margin_infeasible_streak / kHardMarginDebounceCycles in
  // shift_line_generator.cpp) so a single noisy cycle doesn't immediately null out
  // new_shift_line. This does NOT weaken the check itself: once the streak reaches the debounce
  // threshold, the object is still refused, every time, from then on.
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

    // Must match shift_line_generator.cpp's kHardMarginDebounceCycles.
    constexpr int kHardMarginDebounceCycles = 3;

    auto object = fixture.make_object(prepare_distance + remaining_distance);
    object.is_avoidance_committed = true;

    // First kHardMarginDebounceCycles consecutive failing cycles are debounced: the check still
    // fails underneath, but the object is not yet treated as genuinely infeasible.
    for (int cycle = 0; cycle < kHardMarginDebounceCycles; ++cycle) {
      const auto result =
        fixture.generator.computeFeasibleShiftProfile(object, desire_shift_length, 0.0);
      EXPECT_TRUE(result.has_value()) << "cycle " << cycle;
      EXPECT_EQ(object.hard_margin_infeasible_streak, cycle + 1);
    }

    // The check has now failed kHardMarginDebounceCycles times in a row: refused for real, same
    // outcome as the un-debounced check would have produced immediately.
    const auto result =
      fixture.generator.computeFeasibleShiftProfile(object, desire_shift_length, 0.0);
    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(object.info, ObjectInfo::NEED_DECELERATION);
  }

  // (c) The debounce in (b) only applies to already is_avoidance_committed objects -- a
  // newly-encountered object (not yet committed) still gets the strict, non-debounced check and
  // is refused on the very first cycle, exactly as before this fix.
  {
    AlwaysAvoidTestFixture fixture(/*vehicle_width=*/50.0);
    fixture.parameters->always_avoid_if_geometrically_possible = true;

    constexpr double desire_shift_length = 1.5;
    const auto avoiding_shift = desire_shift_length;
    const auto min_avoidance_distance = fixture.helper->getMinAvoidanceDistance(avoiding_shift);
    const auto remaining_distance = 0.9 * min_avoidance_distance;
    const auto prepare_distance = fixture.helper->getNominalPrepareDistance();

    auto object = fixture.make_object(prepare_distance + remaining_distance);
    object.is_avoidance_committed = false;

    const auto result =
      fixture.generator.computeFeasibleShiftProfile(object, desire_shift_length, 0.0);
    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(object.info, ObjectInfo::NEED_DECELERATION);
  }
}

// [REV-F2 fix 2026-09-03] direct regression test for the debounce mechanism itself: a committed
// object whose hard-margin check flips infeasible for a single, isolated cycle (pose/overhang
// jitter) must still produce a feasible shift that cycle, and the streak must reset to 0 the
// moment the check passes again -- confirming this isn't a one-way ratchet toward permanent
// refusal.
TEST(TestUtils, ComputeFeasibleShiftProfileDebouncesSingleCycleHardMarginFlicker)
{
  AlwaysAvoidTestFixture fixture(/*vehicle_width=*/0.5);
  fixture.parameters->always_avoid_if_geometrically_possible = false;

  constexpr double desire_shift_length = 1.5;
  const auto avoiding_shift = desire_shift_length;
  const auto min_avoidance_distance = fixture.helper->getMinAvoidanceDistance(avoiding_shift);
  ASSERT_GT(min_avoidance_distance, 0.0);
  const auto remaining_distance = 0.9 * min_avoidance_distance;
  const auto prepare_distance = fixture.helper->getNominalPrepareDistance();

  auto object = fixture.make_object(prepare_distance + remaining_distance);
  object.is_avoidance_committed = true;
  // Overhang set at the object's expected shift target: with the nominal vehicle_width=0.5 below
  // the hard-margin check passes (plenty of clearance vs. the small half-width + relaxed floor);
  // temporarily inflating vehicle_width simulates the boundary-condition jitter (observed live as
  // ~5.5mm from the threshold) without needing to reverse-engineer the exact jerk-limited
  // feasible_shift_length value.
  object.overhang_points.front().first = desire_shift_length;

  // Cycle 1: hard-margin check passes normally (vehicle_width=0.5, plenty of clearance).
  const auto pass_result =
    fixture.generator.computeFeasibleShiftProfile(object, desire_shift_length, 0.0);
  ASSERT_TRUE(pass_result.has_value());
  EXPECT_EQ(object.hard_margin_infeasible_streak, 0);

  // Cycle 2: simulate a single noisy cycle where the vehicle's effective half-width vs. the
  // object's overhang trips the hard-margin check. Must still produce a feasible shift
  // (debounced), not nullopt.
  fixture.planner_data->parameters.vehicle_width = 50.0;
  const auto jitter_result =
    fixture.generator.computeFeasibleShiftProfile(object, desire_shift_length, 0.0);
  EXPECT_TRUE(jitter_result.has_value());
  EXPECT_EQ(object.hard_margin_infeasible_streak, 1);

  // Cycle 3: jitter clears, vehicle_width back to normal -- streak resets to 0.
  fixture.planner_data->parameters.vehicle_width = 0.5;
  const auto recover_result =
    fixture.generator.computeFeasibleShiftProfile(object, desire_shift_length, 0.0);
  ASSERT_TRUE(recover_result.has_value());
  EXPECT_EQ(object.hard_margin_infeasible_streak, 0);
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

// Two avoidance-target objects on opposite sides of the TRUE lane, with longitudinally
// overlapping footprints, force a single ego shift value to satisfy both constraints
// simultaneously. getAvoidMargin()/getRoadShoulderDistance() alone only ever check each object's
// own distance to the true lane boundary, so without applyJointLateralFeasibility() two such
// objects could each report a healthy individual avoid_margin even when the real gap between them
// is too tight for both to be avoided at once. These tests build the two objects' envelope_poly
// directly as simple rectangles separated along the lateral (y) axis so the physical gap between
// them is known exactly, then exercise applyJointLateralFeasibility() + getAvoidMargin().
namespace
{
// TRUCK: lateral_hard_margin = 0.2 (non-parked), vehicle_width (get_planner_data()) = 2.0,
// hard_drivable_bound_margin = 0.1, soft_drivable_bound_margin = 0.5 (see get_parameters()).
ObjectData make_joint_feasibility_object(
  const bool is_on_right, const double y_min, const double y_max, const double longitudinal,
  const double length, const double original_to_road_shoulder_distance)
{
  ObjectData object_data;
  object_data.object.classification.emplace_back(
    autoware_perception_msgs::build<ObjectClassification>()
      .label(ObjectClassification::TRUCK)
      .probability(1.0));
  object_data.is_parked = false;
  object_data.distance_factor = 1.0;
  object_data.is_on_right_of_true_lane = is_on_right;
  object_data.longitudinal = longitudinal;
  object_data.length = length;
  object_data.to_road_shoulder_distance = original_to_road_shoulder_distance;

  object_data.envelope_poly.outer() = {
    Point2d{0.0, y_min}, Point2d{4.0, y_min}, Point2d{4.0, y_max}, Point2d{0.0, y_max},
    Point2d{0.0, y_min}};

  return object_data;
}
}  // namespace

TEST(TestUtils, applyJointLateralFeasibilityFeasibleSqueezeTightensMargin)
{
  const auto parameters = get_parameters();
  const auto planner_data = get_planner_data();

  // Untouched (single-object) baseline: what getAvoidMargin() would report from each object's
  // original, far-bound-relative to_road_shoulder_distance alone.
  constexpr double original_distance = 10.0;
  const auto baseline =
    filtering_utils::getAvoidMargin(
      make_joint_feasibility_object(true, -3.6, -1.6, 2.0, 4.0, original_distance), planner_data,
      parameters)
      .value();

  // Rectangles separated by a 3.2 m lateral gap (closest edges at y = -1.6 and y = +1.6),
  // longitudinally overlapping ([0, 4] for both).
  ObjectDataArray objects{
    make_joint_feasibility_object(true, -3.6, -1.6, 2.0, 4.0, original_distance),
    make_joint_feasibility_object(false, 1.6, 3.6, 2.0, 4.0, original_distance)};

  filtering_utils::applyJointLateralFeasibility(objects, planner_data, parameters);

  // Both to_road_shoulder_distance values must have been tightened down from the untouched
  // far-bound distance to (gap - other object's hard margin) = 3.2 - 0.2 = 3.0.
  EXPECT_LT(objects.at(0).to_road_shoulder_distance, original_distance);
  EXPECT_LT(objects.at(1).to_road_shoulder_distance, original_distance);
  EXPECT_DOUBLE_EQ(objects.at(0).to_road_shoulder_distance, 3.0);
  EXPECT_DOUBLE_EQ(objects.at(1).to_road_shoulder_distance, 3.0);

  const auto margin_0 = filtering_utils::getAvoidMargin(objects.at(0), planner_data, parameters);
  const auto margin_1 = filtering_utils::getAvoidMargin(objects.at(1), planner_data, parameters);

  ASSERT_TRUE(margin_0.has_value());
  ASSERT_TRUE(margin_1.has_value());
  EXPECT_LT(margin_0.value(), baseline);
  EXPECT_LT(margin_1.value(), baseline);
}

TEST(TestUtils, applyJointLateralFeasibilityInfeasibleSqueezeReturnsNullopt)
{
  const auto parameters = get_parameters();
  const auto planner_data = get_planner_data();

  // Rectangles separated by only a 2.0 m lateral gap (closest edges at y = -1.0 and y = +1.0) --
  // below the feasibility threshold for two TRUCK objects with vehicle_width = 2.0.
  ObjectDataArray objects{
    make_joint_feasibility_object(true, -3.0, -1.0, 2.0, 4.0, 10.0),
    make_joint_feasibility_object(false, 1.0, 3.0, 2.0, 4.0, 10.0)};

  filtering_utils::applyJointLateralFeasibility(objects, planner_data, parameters);

  EXPECT_FALSE(filtering_utils::getAvoidMargin(objects.at(0), planner_data, parameters).has_value());
  EXPECT_FALSE(filtering_utils::getAvoidMargin(objects.at(1), planner_data, parameters).has_value());
}

TEST(TestUtils, applyJointLateralFeasibilityNoOpForSingleObject)
{
  const auto parameters = get_parameters();
  const auto planner_data = get_planner_data();

  constexpr double original_distance = 10.0;
  ObjectDataArray objects{
    make_joint_feasibility_object(true, -3.6, -1.6, 2.0, 4.0, original_distance)};

  const auto expected_margin =
    filtering_utils::getAvoidMargin(objects.at(0), planner_data, parameters);

  filtering_utils::applyJointLateralFeasibility(objects, planner_data, parameters);

  EXPECT_DOUBLE_EQ(objects.at(0).to_road_shoulder_distance, original_distance);
  const auto actual_margin =
    filtering_utils::getAvoidMargin(objects.at(0), planner_data, parameters);
  ASSERT_EQ(expected_margin.has_value(), actual_margin.has_value());
  if (expected_margin.has_value()) {
    EXPECT_DOUBLE_EQ(expected_margin.value(), actual_margin.value());
  }
}

TEST(TestUtils, applyJointLateralFeasibilityNoOpForSameSideObjects)
{
  const auto parameters = get_parameters();
  const auto planner_data = get_planner_data();

  constexpr double original_distance = 10.0;
  // Same side (both is_on_right_of_true_lane == true), even though longitudinally overlapping and
  // laterally close -- must be left untouched, since a same-side pair isn't a simultaneous
  // opposite-side squeeze.
  ObjectDataArray objects{
    make_joint_feasibility_object(true, -3.6, -1.6, 2.0, 4.0, original_distance),
    make_joint_feasibility_object(true, -1.4, 0.6, 2.0, 4.0, original_distance)};

  filtering_utils::applyJointLateralFeasibility(objects, planner_data, parameters);

  EXPECT_DOUBLE_EQ(objects.at(0).to_road_shoulder_distance, original_distance);
  EXPECT_DOUBLE_EQ(objects.at(1).to_road_shoulder_distance, original_distance);
}

TEST(TestUtils, applyJointLateralFeasibilityNoOpForNonOverlappingLongitudinalRange)
{
  const auto parameters = get_parameters();
  const auto planner_data = get_planner_data();

  constexpr double original_distance = 10.0;
  // Opposite sides, close laterally, but far apart longitudinally (ranges [-1, 1] and [19, 21] do
  // not overlap) -- a single ego shift value never has to satisfy both at once, so must be left
  // untouched.
  ObjectDataArray objects{
    make_joint_feasibility_object(true, -3.0, -1.0, 0.0, 2.0, original_distance),
    make_joint_feasibility_object(false, 1.0, 3.0, 20.0, 2.0, original_distance)};

  filtering_utils::applyJointLateralFeasibility(objects, planner_data, parameters);

  EXPECT_DOUBLE_EQ(objects.at(0).to_road_shoulder_distance, original_distance);
  EXPECT_DOUBLE_EQ(objects.at(1).to_road_shoulder_distance, original_distance);
}

// [ROBOT-NOT-A-CAR] Regression test: a pedestrian/bicycle object sitting exactly on (or within
// threshold_distance_object_is_on_center of) the lane centerline used to be unconditionally
// excluded from avoidance by isSatisfiedWithNonVehicleCondition() (ObjectInfo::
// TOO_NEAR_TO_CENTERLINE), regardless of how much real lateral room was available. That gate has
// been removed for this platform (see rationale comment at the call site); confirm a dead-center
// pedestrian is no longer rejected by it.
TEST(TestUtils, isSatisfiedWithNonVehicleConditionAllowsDeadCenterPedestrian)
{
  constexpr double half_width = 1.75;  // 3.5m-wide lane.

  const auto planner_data = get_planner_data();
  const auto parameters = get_parameters();
  parameters->threshold_distance_object_is_on_center = 0.05;  // matches production tuning.

  auto [route_handler, lanelet_obj] = make_straight_lane_route_handler(half_width);
  route_handler->setRouteLanelets(lanelet::ConstLanelets{lanelet_obj});
  planner_data->route_handler = route_handler;

  AvoidancePlanningData data;
  data.current_lanelets = {lanelet_obj};

  // Dead center: to_centerline will be exactly 0.0, i.e. as "too near the centerline" as an
  // object can possibly be.
  const auto object_pose = autoware::test_utils::createPose(10.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  ObjectData object_data;
  object_data.object.classification.emplace_back(
    autoware_perception_msgs::build<ObjectClassification>()
      .label(ObjectClassification::PEDESTRIAN)
      .probability(1.0));
  object_data.object.kinematics.initial_pose_with_covariance.pose = object_pose;
  object_data.overhang_lanelet = lanelet_obj;
  object_data.overhang_points.emplace_back(0.0, object_pose.position);

  EXPECT_TRUE(filtering_utils::isSatisfiedWithNonVehicleCondition(
    object_data, data, planner_data, parameters));
  EXPECT_NE(object_data.info, ObjectInfo::TOO_NEAR_TO_CENTERLINE);
  // to_centerline is still populated (used elsewhere, e.g. debug output) even though it no
  // longer gates eligibility.
  EXPECT_NEAR(object_data.to_centerline, 0.0, 1e-6);
}

// [ROBOT-NOT-A-CAR] Regression test: a vehicle-classified obstacle sitting square in the middle
// of the ego's own lane (not "is_parked" -- it doesn't hug either edge) used to fall through
// isObviousAvoidanceTarget() into the ambiguous-vehicle path, which -- even under the most
// permissive policy -- withholds avoidance for several seconds of stop time before treating it as
// avoidable. Confirm such an object is now treated as an obvious avoidance target immediately,
// exactly like a curb-parked vehicle, as long as it actually has room to shift (avoid_margin has
// a value).
TEST(TestUtils, isObviousAvoidanceTargetAcceptsCenteredBlockingVehicleImmediately)
{
  const auto planner_data = get_planner_data();
  const auto parameters = get_parameters();

  // isObviousAvoidanceTarget() calls isWithinFreespace(), which dereferences
  // route_handler->getLaneletMapPtr() -- needs a real (even if empty-of-parking-lots) map set,
  // unlike the default route_handler from get_planner_data().
  auto [route_handler, lanelet_obj] = make_straight_lane_route_handler(1.75);
  planner_data->route_handler = route_handler;

  AvoidancePlanningData data;

  ObjectData object_data;
  object_data.object.classification.emplace_back(
    autoware_perception_msgs::build<ObjectClassification>()
      .label(ObjectClassification::CAR)
      .probability(1.0));
  object_data.behavior = ObjectData::Behavior::NONE;
  object_data.is_within_intersection = false;
  object_data.is_on_ego_lane = true;
  object_data.is_parked = false;  // dead-center: fails the classic parked-vehicle classification.
  object_data.avoid_margin = 0.5;  // genuine lateral room is available.

  EXPECT_TRUE(
    filtering_utils::isObviousAvoidanceTarget(object_data, data, planner_data, parameters));

  // Sanity check on the old behavior: with no lateral room at all (avoid_margin unset), the
  // object must NOT be force-accepted -- the real feasibility check is never bypassed.
  ObjectData infeasible_object_data = object_data;
  infeasible_object_data.avoid_margin = std::nullopt;
  EXPECT_FALSE(filtering_utils::isObviousAvoidanceTarget(
    infeasible_object_data, data, planner_data, parameters));
}

// Reproduces the "hold-forever deadlock" scenario (2026-09-04 investigation): a shift line was
// registered on a previous, then-safe cycle at a value that is now insufficient (the object still
// requires more clearance), isSafePath() keeps rejecting the fresh candidate, and
// canYieldManeuver() keeps refusing a yield -- so scene.cpp's "hold" branch would otherwise clear
// data.safe_shift_line and hold the same insufficient registered value forever. This test
// exercises the extracted decision function directly (shouldRearmHeldShiftLine), since driving the
// full StaticObstacleAvoidanceModule scene-module state machine end-to-end is impractical at the
// unit level.
TEST(TestUtils, ShouldRearmHeldShiftLineResolvesDeadlockWhenEgoStationary)
{
  // Registered on a previous cycle: -0.715 (matches the live-observed plateau value). Freshly
  // required this cycle: -1.08 (matches the live-observed desire_shift_length). Same side
  // (both negative / rightward), ego genuinely stationary.
  constexpr double registered_end_shift_length = -0.715;
  constexpr double desired_end_shift_length = -1.08;
  constexpr double ego_speed_threshold = 0.1;
  constexpr double clearance_margin = 1e-2;

  // Ego stationary: the deadlock must be broken -- re-arm to the larger (more negative) value.
  EXPECT_TRUE(shouldRearmHeldShiftLine(
    registered_end_shift_length, desired_end_shift_length, /*has_new_candidate=*/true,
    /*ego_speed=*/0.0, ego_speed_threshold, clearance_margin))
    << "must re-arm: registered shift falls short of the freshly-required, still-unsafe "
       "candidate, and ego is safely stationary -- this is exactly the deadlock condition.";

  // Ego still moving: never re-arm, regardless of how large the shortfall is -- avoids a sudden
  // lateral jerk while driving.
  EXPECT_FALSE(shouldRearmHeldShiftLine(
    registered_end_shift_length, desired_end_shift_length, /*has_new_candidate=*/true,
    /*ego_speed=*/0.5, ego_speed_threshold, clearance_margin))
    << "must NOT re-arm while ego is moving, even if the registered shift is insufficient.";

  // No fresh candidate this cycle (e.g. generator produced nothing): nothing to re-arm to.
  EXPECT_FALSE(shouldRearmHeldShiftLine(
    registered_end_shift_length, desired_end_shift_length, /*has_new_candidate=*/false,
    /*ego_speed=*/0.0, ego_speed_threshold, clearance_margin));

  // Opposite-side shift (registered negative, desired positive): never re-arm -- "more clearance"
  // is meaningless across sides.
  EXPECT_FALSE(shouldRearmHeldShiftLine(
    registered_end_shift_length, /*desired_end_shift_length=*/1.08, /*has_new_candidate=*/true,
    /*ego_speed=*/0.0, ego_speed_threshold, clearance_margin));

  // Candidate does NOT need more clearance than what is already registered (e.g. registered is
  // already sufficient, or the fresh candidate is smaller): must NOT re-arm -- re-arming here
  // would DECREASE the registered shift, which is exactly what the original Sept-2 hold fix
  // exists to prevent.
  EXPECT_FALSE(shouldRearmHeldShiftLine(
    /*registered_end_shift_length=*/-1.08, /*desired_end_shift_length=*/-0.715,
    /*has_new_candidate=*/true, /*ego_speed=*/0.0, ego_speed_threshold, clearance_margin))
    << "must NOT re-arm to a SMALLER shift -- that would defeat the original hold-branch fix.";

  // Difference within noise-level clearance_margin: must NOT re-arm (avoids chattering).
  EXPECT_FALSE(shouldRearmHeldShiftLine(
    /*registered_end_shift_length=*/-0.715, /*desired_end_shift_length=*/-0.716,
    /*has_new_candidate=*/true, /*ego_speed=*/0.0, ego_speed_threshold, clearance_margin));
}

// [MULTI-OBJ-HOLD fix 2026-09-04, RECONSTRUCTED 2026-09-04] Lost to an accidental
// `git checkout -- test/test_utils.cpp` during a later same-day investigation session;
// reconstructed from the surviving function doc comment in utils.hpp and the surviving call
// site in shift_line_generator.cpp. Please review against your own memory of the original.
TEST(TestUtils, ExistsUnclearedAvoidanceObjectAheadHoldsUntilAllObjectsPassed)
{
  const auto make_object = [](const double longitudinal, const bool is_avoidable) {
    ObjectData object_data{};
    object_data.longitudinal = longitudinal;
    object_data.is_avoidable = is_avoidable;
    return object_data;
  };

  // No target objects at all: must NOT hold.
  {
    ObjectDataArray target_objects{};
    EXPECT_FALSE(existsUnclearedAvoidanceObjectAhead(target_objects));
  }

  // Single object, still ahead and avoidable: must hold.
  {
    ObjectDataArray target_objects{make_object(/*longitudinal=*/1.5, /*is_avoidable=*/true)};
    EXPECT_TRUE(existsUnclearedAvoidanceObjectAhead(target_objects));
  }

  // Single object, already behind ego (cleared): must NOT hold.
  {
    ObjectDataArray target_objects{make_object(/*longitudinal=*/-1.2, /*is_avoidable=*/true)};
    EXPECT_FALSE(existsUnclearedAvoidanceObjectAhead(target_objects));
  }

  // Object 1 already cleared (behind), object 2 still ahead: must hold on object 2 alone.
  {
    ObjectDataArray target_objects{
      make_object(/*longitudinal=*/-1.2, /*is_avoidable=*/true),
      make_object(/*longitudinal=*/2.0, /*is_avoidable=*/true)};
    EXPECT_TRUE(existsUnclearedAvoidanceObjectAhead(target_objects));
  }

  // Object 2 still ahead but marked unavoidable: still must hold (this function does not
  // distinguish avoidable/unavoidable -- any object ahead, avoidable or not, blocks the return
  // shift; the pre-existing exist_unavoidable_object check in addReturnShiftLine covers the
  // unavoidable case separately and returns even earlier).
  {
    ObjectDataArray target_objects{
      make_object(/*longitudinal=*/-1.2, /*is_avoidable=*/true),
      make_object(/*longitudinal=*/2.0, /*is_avoidable=*/false)};
    EXPECT_TRUE(existsUnclearedAvoidanceObjectAhead(target_objects));
  }

  // All objects behind ego: must NOT hold -- normal return-to-lane may proceed.
  {
    ObjectDataArray target_objects{
      make_object(/*longitudinal=*/-1.2, /*is_avoidable=*/true),
      make_object(/*longitudinal=*/-0.3, /*is_avoidable=*/false)};
    EXPECT_FALSE(existsUnclearedAvoidanceObjectAhead(target_objects));
  }
}

// [BUG-A fix 2026-09-04] Direct regression test for the close-range candidate-drop bug: with the
// object very close (live-observed longitudinal ~= 0.22 m) and a small floor-clamped
// start_longitudinal, the raw "object longitudinal - constant_distance" end point collapses to at
// or below start_longitudinal. rescueCloseRangeShiftLineEnd() must anchor end_longitudinal to at
// least start_longitudinal + min_transition_distance instead of the candidate being silently
// dropped, as long as that still fits before reaching the object.
TEST(TestUtils, RescueCloseRangeShiftLineEndRescuesGenuineCloseRangeCandidate)
{
  constexpr double start_longitudinal = 0.05;       // floor-clamped start (nearest_avoid_distance)
  constexpr double min_transition_distance = 0.15;  // jerk-feasible minimum for this shift
  constexpr double object_longitudinal = 0.22;      // live-observed close-range value
  constexpr double approach_buffer = 1e-2;

  const auto result = rescueCloseRangeShiftLineEnd(
    start_longitudinal, min_transition_distance, object_longitudinal, approach_buffer);

  ASSERT_TRUE(result.has_value())
    << "a close-range candidate with enough room for the minimum jerk-feasible transition must "
       "be rescued instead of silently dropped.";
  EXPECT_DOUBLE_EQ(result.value(), start_longitudinal + min_transition_distance);
  EXPECT_GT(result.value(), start_longitudinal)
    << "rescued end_longitudinal must satisfy is_valid_shift_line's start < end requirement.";
  EXPECT_LE(result.value(), object_longitudinal - approach_buffer);
}

// Boundary case: the rescued end lands exactly at the latest feasible point (object_longitudinal -
// approach_buffer) -- must still be treated as rescuable (inclusive bound), not infeasible.
TEST(TestUtils, RescueCloseRangeShiftLineEndBoundaryIsInclusive)
{
  constexpr double start_longitudinal = 0.05;
  constexpr double min_transition_distance = 0.16;
  constexpr double approach_buffer = 1e-2;
  // object_longitudinal chosen so rescued_end == object_longitudinal - approach_buffer exactly.
  constexpr double object_longitudinal =
    start_longitudinal + min_transition_distance + approach_buffer;

  const auto result = rescueCloseRangeShiftLineEnd(
    start_longitudinal, min_transition_distance, object_longitudinal, approach_buffer);

  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result.value(), object_longitudinal - approach_buffer);
}

// [BUG-A fix 2026-09-04] Genuine kinematic infeasibility: the object is so close that even the
// minimum jerk-feasible transition distance cannot fit before reaching it. This must correctly
// report infeasibility (nullopt) rather than producing an unsafe/aggressive shift line that runs
// past the object.
TEST(TestUtils, RescueCloseRangeShiftLineEndReportsGenuineInfeasibility)
{
  constexpr double start_longitudinal = 0.05;
  constexpr double min_transition_distance = 0.5;  // large shift needs a long transition
  constexpr double object_longitudinal = 0.22;      // not enough room for 0.5 m of transition
  constexpr double approach_buffer = 1e-2;

  const auto result = rescueCloseRangeShiftLineEnd(
    start_longitudinal, min_transition_distance, object_longitudinal, approach_buffer);

  EXPECT_FALSE(result.has_value())
    << "must NOT rescue when even the minimum jerk-feasible transition cannot fit before the "
       "object -- this is a real kinematic infeasibility, not an arithmetic accident.";
}

// [index-gap fix 2026-09-04] Direct regression test for the live-observed follow-on bug: the
// original rescue (no path_arclength_arr) produced start=0.050, end=0.365 (~0.315 m window),
// which passes is_valid_shift_line()'s start < end check but, against a reference path resampled
// at resample_interval_for_planning ~= 0.3 m, snaps to reference-path point indices only 1 apart
// (idx_gap <= 1) -- exactly the "shift start point and end point can't be adjoining" failure
// PathShifter::generate() then hits, causing scene.cpp to silently fall back to the previous path.
// With path_arclength_arr supplied (points every 0.3 m, matching the live resample interval), the
// rescue must extend end_longitudinal at least to the arclength of the point 2 indices ahead of
// start's nearest index, so PathShifter::generate() can actually build the shift line.
TEST(TestUtils, RescueCloseRangeShiftLineEndExtendsForIndexGapRequirement)
{
  constexpr double start_longitudinal = 0.05;
  constexpr double min_transition_distance = 0.315;  // reproduces the live ~0.315 m naive window
  constexpr double object_longitudinal = 2.0;        // plenty of room past the naive window
  constexpr double approach_buffer = 1e-2;
  // Reference path resampled at 0.3 m, matching the live resample_interval_for_planning default.
  const std::vector<double> path_arclength_arr{0.0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.8, 2.1, 2.4};

  const auto naive_result = rescueCloseRangeShiftLineEnd(
    start_longitudinal, min_transition_distance, object_longitudinal, approach_buffer);
  ASSERT_TRUE(naive_result.has_value());
  // Confirm the premise: the naive (no-path-awareness) rescue really does land inside a single
  // 0.3 m segment -- i.e. it would snap start/end to adjacent indices (idx_gap <= 1).
  EXPECT_NEAR(naive_result.value(), 0.365, 1e-9);

  const auto result = rescueCloseRangeShiftLineEnd(
    start_longitudinal, min_transition_distance, object_longitudinal, approach_buffer,
    path_arclength_arr);

  ASSERT_TRUE(result.has_value())
    << "there is plenty of room before the object -- this must be rescued, just with a wider "
       "window than the naive longitudinal-only computation.";
  // With ego_offset_along_path defaulted to 0.0 (path_arclength_arr is already ego-relative),
  // start_longitudinal=0.05 snaps to start_idx=1 (first arr[i] > 0.05 is arr[1]=0.3). To get
  // end_idx >= start_idx + 2 = 3, end_longitudinal must be at least arr[start_idx + 1] = arr[2] =
  // 0.6 (findPathIndexFromArclength(arr, 0.6) returns 3, since arr[2]=0.6 is not > 0.6).
  EXPECT_DOUBLE_EQ(result.value(), 0.6)
    << "rescued end_longitudinal must be extended far enough to satisfy PathShifter::generate()'s "
       "idx_gap > 1 requirement, not just the naive min_transition_distance window.";
  EXPECT_GT(result.value(), naive_result.value())
    << "the index-gap-aware rescue must extend beyond the naive (too-short) window.";
  EXPECT_LE(result.value(), object_longitudinal - approach_buffer);
}

// [index-gap fix 2026-09-04] Genuine "no room" infeasibility that is ONLY visible once the
// index-gap requirement is taken into account: the object is close enough that satisfying
// idx_gap > 1 would require running past the object itself. The naive (path-unaware) rescue would
// incorrectly report success here with a window that PathShifter::generate() would then reject
// anyway -- this must instead report std::nullopt directly.
TEST(TestUtils, RescueCloseRangeShiftLineEndReportsInfeasibilityWhenIndexGapCannotFit)
{
  constexpr double start_longitudinal = 0.05;
  constexpr double min_transition_distance = 0.05;  // tiny -- naive rescue would "succeed"
  constexpr double object_longitudinal = 0.2;       // but object is inside the same 0.3 m segment
  constexpr double approach_buffer = 1e-2;
  const std::vector<double> path_arclength_arr{0.0, 0.3, 0.6, 0.9, 1.2};

  const auto naive_result = rescueCloseRangeShiftLineEnd(
    start_longitudinal, min_transition_distance, object_longitudinal, approach_buffer);
  ASSERT_TRUE(naive_result.has_value())
    << "premise check: the naive path-unaware rescue does report success here.";

  const auto result = rescueCloseRangeShiftLineEnd(
    start_longitudinal, min_transition_distance, object_longitudinal, approach_buffer,
    path_arclength_arr);

  EXPECT_FALSE(result.has_value())
    << "satisfying idx_gap > 1 here requires end_longitudinal >= arr[start_idx + 1] = 0.6 "
       "(start_longitudinal=0.05 snaps to start_idx=1), which is past "
       "object_longitudinal - approach_buffer (0.19) -- this is genuine infeasibility and must "
       "NOT be silently rescued into another idx_gap <= 1 failure downstream.";
}

// [index-gap fix, shared helper] Direct test of computeIndexGapSafeEndLongitudinal(), factored out
// of rescueCloseRangeShiftLineEnd() so it can be reused by evaluateFillGapShiftLine(). Same
// resample geometry as RescueCloseRangeShiftLineEndExtendsForIndexGapRequirement above.
TEST(TestUtils, ComputeIndexGapSafeEndLongitudinalMatchesRescueHelper)
{
  const std::vector<double> path_arclength_arr{0.0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.8, 2.1, 2.4};

  const auto result = computeIndexGapSafeEndLongitudinal(0.05, path_arclength_arr);

  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result.value(), 0.6);
}

// [index-gap fix, shared helper] Empty path_arclength_arr means "no path to check against" --
// must pass start_longitudinal straight through unchanged rather than reporting infeasibility.
TEST(TestUtils, ComputeIndexGapSafeEndLongitudinalNoOpWhenArrayEmpty)
{
  const auto result = computeIndexGapSafeEndLongitudinal(0.05, {});

  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result.value(), 0.05);
}

// [index-gap fix, shared helper] No room at all ahead of start -- genuine infeasibility.
TEST(TestUtils, ComputeIndexGapSafeEndLongitudinalReportsInfeasibilityAtPathEnd)
{
  const std::vector<double> path_arclength_arr{0.0, 0.3};

  const auto result = computeIndexGapSafeEndLongitudinal(0.05, path_arclength_arr);

  EXPECT_FALSE(result.has_value());
}

namespace
{
AvoidLine makeFillGapTestLine(
  const size_t start_idx, const size_t end_idx, const double start_shift_length,
  const double end_shift_length)
{
  AvoidLine line{};
  line.start_idx = start_idx;
  line.end_idx = end_idx;
  line.start_shift_length = start_shift_length;
  line.end_shift_length = end_shift_length;
  return line;
}
}  // namespace

// [FILL-GAP-INDEX-FIX 2026-09-04] Regression test for the second, independent "adjoining" source
// found live: a fresh, far object (longitudinal=8.22 m, current_ego_shift=0.0, NOT a BUG-A-RESCUE
// close-range case at all) whose avoid-to-return gap-fill connector line collapsed to idx_gap<=1
// because its front+rear longitudinal margins were small. A window with idx_gap > 1 must be kept
// unchanged.
TEST(TestUtils, EvaluateFillGapShiftLineKeepsIndexGapSafeLine)
{
  const auto line = makeFillGapTestLine(/*start_idx=*/10, /*end_idx=*/13, -0.9, -0.9);

  const auto decision = evaluateFillGapShiftLine(line);

  EXPECT_TRUE(decision.keep);
}

// The exact live-observed failure shape: start_idx and end_idx only 1 apart (idx_gap == 1), and
// the connector is FLAT (start_shift_length == end_shift_length, always true for the
// avoid-line-end-to-return-line-start connector by construction -- see
// ShiftLineGenerator::generateAvoidOutline(), al_return.start_shift_length =
// al_avoid.end_shift_length). This must be dropped (not kept), and reported as the safe/lossless
// "flat" case -- PathShifter carries the prior shift value forward through the gap regardless, so
// dropping this connector changes nothing about the actual shift profile.
TEST(TestUtils, EvaluateFillGapShiftLineDropsFlatIndexGapUnsafeLineLosslessly)
{
  const auto line = makeFillGapTestLine(/*start_idx=*/40, /*end_idx=*/41, -0.895, -0.895);

  const auto decision = evaluateFillGapShiftLine(line);

  EXPECT_FALSE(decision.keep);
  EXPECT_TRUE(decision.is_flat)
    << "matching start/end shift_length must be classified as the safe/lossless flat case.";
}

// idx_gap == 0 (end_idx == start_idx, an even more degenerate collapse than idx_gap == 1) must
// also be dropped, not kept -- PathShifter::generate()'s own check is `idx_gap <= 1`, not `< 1`.
TEST(TestUtils, EvaluateFillGapShiftLineDropsZeroIndexGapFlatLine)
{
  const auto line = makeFillGapTestLine(/*start_idx=*/5, /*end_idx=*/5, 0.0, 0.0);

  const auto decision = evaluateFillGapShiftLine(line);

  EXPECT_FALSE(decision.keep);
  EXPECT_TRUE(decision.is_flat);
}

// Index-gap-unsafe AND non-flat (a genuine shift-length transition is needed across this window,
// e.g. the generic "fill gap among shift lines" pass between two different objects' shift lines
// that do not happen to share the same boundary shift length): must still be dropped (never hand
// PathShifter a doomed candidate that fails generate() for the entire shift_lines_ list), but
// reported as the non-flat case so the caller can log that a real transition was lost.
TEST(TestUtils, EvaluateFillGapShiftLineDropsNonFlatIndexGapUnsafeLineWithWarning)
{
  const auto line = makeFillGapTestLine(/*start_idx=*/40, /*end_idx=*/41, -0.9, -0.4);

  const auto decision = evaluateFillGapShiftLine(line);

  EXPECT_FALSE(decision.keep);
  EXPECT_FALSE(decision.is_flat)
    << "differing start/end shift_length must be classified as the non-flat, real-transition-lost "
       "case, distinct from the always-safe flat case.";
}

// A tolerance-level near-flat line (tiny floating-point/noise-level difference) must still be
// treated as flat -- the default flat_shift_length_tolerance exists exactly to absorb this.
TEST(TestUtils, EvaluateFillGapShiftLineTreatsNearFlatWithinToleranceAsFlat)
{
  const auto line = makeFillGapTestLine(/*start_idx=*/40, /*end_idx=*/41, -0.9, -0.9 + 1e-4);

  const auto decision = evaluateFillGapShiftLine(line);

  EXPECT_FALSE(decision.keep);
  EXPECT_TRUE(decision.is_flat);
}

// [MERGE-INDEX-FIX 2026-09-04] Regression tests for the THIRD, independent "adjoining" source found
// live: ShiftLineGenerator::extractShiftLinesFromLine() (the MAIN merge pipeline's Step1, feeding
// applyMergeProcess() -> generateCandidateShiftLine(), distinct from both BUG-A-RESCUE's close-range
// rescue and FILL-GAP-INDEX-FIX's gap-fill connector construction) builds AvoidLine segments purely
// from gradient-change detection with no index-gap validation at all -- confirmed live by a
// PathShifter "shift start point and end point can't be adjoining" failure with neither
// [BUG-A-RESCUE] nor [FILL-GAP-INDEX-FIX] logged nearby, right after several ASYM-HYSTERESIS cycles.
//
// The production fix (extractShiftLinesFromLine() in shift_line_generator.cpp) reuses this SAME
// evaluateFillGapShiftLine() helper at each candidate-segment finalization point -- these tests
// pin down that the helper's keep/drop decision is exactly what that call site relies on for a
// segment shaped like the ones extractShiftLinesFromLine() builds (plain start/end index + shift
// length, no dependency on how the segment's indices were derived).

// A normal merge-pipeline segment with plenty of index-gap margin (the vast majority of real
// gradient-change segments) must be completely unaffected by the new guard.
TEST(TestUtils, MergeIndexFixKeepsNormalMergeSegmentUnaffected)
{
  const auto line = makeFillGapTestLine(/*start_idx=*/44, /*end_idx=*/60, -0.5, -0.9);

  const auto decision = evaluateFillGapShiftLine(line);

  EXPECT_TRUE(decision.keep)
    << "a segment with plenty of index-gap margin must flow through completely unchanged.";
}

// The exact live-observed failure shape for this third source: two gradient-change points detected
// only 1 index apart (idx_gap == 1) right after an ASYM-HYSTERESIS-driven nudge. Since this is a
// mid-approach segment (there is more path ahead to extend into), the production call site treats
// this as "extend" (do not finalize the boundary yet) rather than an unconditional drop -- but the
// underlying keep/drop decision from evaluateFillGapShiftLine() that triggers that extension must
// correctly flag the segment as unsafe to finalize as-is.
TEST(TestUtils, MergeIndexFixFlagsNearAdjacentGradientChangeSegmentForExtension)
{
  const auto line = makeFillGapTestLine(/*start_idx=*/44, /*end_idx=*/45, -0.5, -0.62);

  const auto decision = evaluateFillGapShiftLine(line);

  EXPECT_FALSE(decision.keep)
    << "idx_gap == 1 must be flagged as unsafe to finalize, triggering the extend-forward path.";
  EXPECT_FALSE(decision.is_flat)
    << "a genuine gradient-change-driven segment (differing start/end shift_length) must be "
       "classified as the non-flat, real-transition-at-risk case.";
}

// The final-closing-boundary shape: no more path to extend into (path end), and the trailing
// segment happens to be flat (constant shift value all the way to the end of the reference path).
// The production call site's final-boundary branch must be able to tell this apart from the
// non-flat case so it can log it as the safe/lossless drop.
TEST(TestUtils, MergeIndexFixFlagsFlatFinalSegmentAsLosslessDrop)
{
  const auto line = makeFillGapTestLine(/*start_idx=*/98, /*end_idx=*/99, -0.9, -0.9);

  const auto decision = evaluateFillGapShiftLine(line);

  EXPECT_FALSE(decision.keep);
  EXPECT_TRUE(decision.is_flat)
    << "a constant-shift trailing segment dropped at path end is lossless -- PathShifter carries "
       "the prior shift value forward regardless.";
}

// [ASYM-HYSTERESIS 2026-09-04] Growing an already-approved shift (more clearance/more urgent)
// must be let through immediately when hysteresis_in_cycles == 1 (the default configuration) --
// this is the safety-critical direction and must react fast, matching the live bug scenario where
// desire_shift_length grew from -0.855 to -0.935 while the committed shift stayed frozen.
TEST(TestUtils, ApplyAsymmetricShiftHysteresisGrowsImmediatelyByDefault)
{
  int grow_streak = 0;
  int shrink_streak = 0;
  const auto applied = applyAsymmetricShiftHysteresis(
    /*desired_end_shift_length=*/-0.935, /*registered_shift_length=*/-0.855,
    /*clearance_margin=*/1e-2, /*hysteresis_in_cycles=*/1, /*hysteresis_out_cycles=*/5,
    grow_streak, shrink_streak);

  EXPECT_DOUBLE_EQ(applied, -0.935) << "growth must be applied on the very first cycle.";
  EXPECT_EQ(grow_streak, 1);
  EXPECT_EQ(shrink_streak, 0);
}

// A larger hysteresis_in_cycles must hold the registered value for that many consecutive growing
// cycles before letting the grow through -- confirms the "in" side is genuinely configurable, not
// hardcoded to immediate.
TEST(TestUtils, ApplyAsymmetricShiftHysteresisRespectsCustomGrowCycles)
{
  int grow_streak = 0;
  int shrink_streak = 0;
  constexpr double registered = -0.855;
  constexpr double desired = -0.935;

  // Cycles 1 and 2: still held at the registered value.
  for (int cycle = 1; cycle <= 2; ++cycle) {
    const auto applied = applyAsymmetricShiftHysteresis(
      desired, registered, /*clearance_margin=*/1e-2, /*hysteresis_in_cycles=*/3,
      /*hysteresis_out_cycles=*/5, grow_streak, shrink_streak);
    EXPECT_DOUBLE_EQ(applied, registered) << "cycle " << cycle;
    EXPECT_EQ(grow_streak, cycle);
  }

  // Cycle 3: streak reaches hysteresis_in_cycles -- grow is finally let through.
  const auto applied = applyAsymmetricShiftHysteresis(
    desired, registered, /*clearance_margin=*/1e-2, /*hysteresis_in_cycles=*/3,
    /*hysteresis_out_cycles=*/5, grow_streak, shrink_streak);
  EXPECT_DOUBLE_EQ(applied, desired);
  EXPECT_EQ(grow_streak, 3);
}

// [ASYM-HYSTERESIS 2026-09-04] Shrinking must be held for hysteresis_out_cycles consecutive
// cycles before being let through -- the "exit" direction that must be slow, to avoid the same
// flicker/premature-collapse failure family as REV-F2 / HOLD-REARM /
// existsUnclearedAvoidanceObjectAhead.
TEST(TestUtils, ApplyAsymmetricShiftHysteresisDelaysShrinkByOutCycles)
{
  int grow_streak = 0;
  int shrink_streak = 0;
  constexpr double registered = -1.08;
  constexpr double desired = -0.715;  // less clearance than registered -> a shrink
  constexpr int hysteresis_out_cycles = 5;

  for (int cycle = 1; cycle < hysteresis_out_cycles; ++cycle) {
    const auto applied = applyAsymmetricShiftHysteresis(
      desired, registered, /*clearance_margin=*/1e-2, /*hysteresis_in_cycles=*/1,
      hysteresis_out_cycles, grow_streak, shrink_streak);
    EXPECT_DOUBLE_EQ(applied, registered)
      << "shrink must be held at the registered value until the debounce is satisfied, cycle "
      << cycle;
    EXPECT_EQ(shrink_streak, cycle);
    EXPECT_EQ(grow_streak, 0);
  }

  // Final cycle: debounce satisfied, shrink is let through.
  const auto applied = applyAsymmetricShiftHysteresis(
    desired, registered, /*clearance_margin=*/1e-2, /*hysteresis_in_cycles=*/1,
    hysteresis_out_cycles, grow_streak, shrink_streak);
  EXPECT_DOUBLE_EQ(applied, desired);
  EXPECT_EQ(shrink_streak, hysteresis_out_cycles);
}

// A grow interrupting a shrink streak (or vice versa) must reset the opposite counter, and a
// within-margin ("no real change") outcome must reset both -- confirms the streaks are not a
// one-way ratchet and correctly track only the currently-active direction.
TEST(TestUtils, ApplyAsymmetricShiftHysteresisResetsOppositeStreakOnDirectionChange)
{
  int grow_streak = 0;
  int shrink_streak = 0;

  // Two shrink cycles build up shrink_streak.
  applyAsymmetricShiftHysteresis(
    -0.715, -1.08, 1e-2, /*hysteresis_in_cycles=*/1, /*hysteresis_out_cycles=*/10, grow_streak,
    shrink_streak);
  applyAsymmetricShiftHysteresis(
    -0.715, -1.08, 1e-2, /*hysteresis_in_cycles=*/1, /*hysteresis_out_cycles=*/10, grow_streak,
    shrink_streak);
  ASSERT_EQ(shrink_streak, 2);

  // Now a growing cycle: shrink_streak must reset to 0, grow_streak starts counting.
  const auto grow_applied = applyAsymmetricShiftHysteresis(
    -1.2, -1.08, 1e-2, /*hysteresis_in_cycles=*/1, /*hysteresis_out_cycles=*/10, grow_streak,
    shrink_streak);
  EXPECT_DOUBLE_EQ(grow_applied, -1.2);
  EXPECT_EQ(grow_streak, 1);
  EXPECT_EQ(shrink_streak, 0);

  // A within-margin ("no real change") outcome resets both streaks.
  const auto noop_applied = applyAsymmetricShiftHysteresis(
    -1.081, -1.08, 1e-2, /*hysteresis_in_cycles=*/1, /*hysteresis_out_cycles=*/10, grow_streak,
    shrink_streak);
  EXPECT_DOUBLE_EQ(noop_applied, -1.081);
  EXPECT_EQ(grow_streak, 0);
  EXPECT_EQ(shrink_streak, 0);
}

// ---------------------------------------------------------------------------------------------
// [OBSTACLE-CLUSTER 2026-09-04] tests for the preprocessing clustering stage.
// ---------------------------------------------------------------------------------------------
namespace
{
PredictedObject make_cluster_test_object(
  const double x, const double y, const uint8_t id_seed,
  const uint8_t label = ObjectClassification::CAR)
{
  PredictedObject object{};
  object.object_id.uuid.fill(0);
  object.object_id.uuid[0] = id_seed;
  object.existence_probability = 1.0F;

  ObjectClassification classification{};
  classification.label = label;
  classification.probability = 1.0F;
  object.classification.push_back(classification);

  object.kinematics.initial_pose_with_covariance.pose =
    autoware::test_utils::createPose(x, y, 0.0, 0.0, 0.0, 0.0);

  object.shape.type = Shape::BOUNDING_BOX;
  object.shape.dimensions = create_vector3(1.0, 1.0, 1.0);
  return object;
}

std::shared_ptr<AvoidanceParameters> make_cluster_test_parameters(
  const double max_neighbor_distance, const int merge_in_cycles = 1,
  const int split_out_cycles = 5, const bool enable = true)
{
  auto parameters = get_parameters();
  parameters->enable_obstacle_clustering = enable;
  parameters->obstacle_cluster_max_neighbor_distance = max_neighbor_distance;
  parameters->obstacle_cluster_merge_in_cycles = merge_in_cycles;
  parameters->obstacle_cluster_split_out_cycles = split_out_cycles;
  return parameters;
}
}  // namespace

TEST(TestUtils, UnionFindGroupsHandlesSingletonsAndPairs)
{
  // no edges at all -- every node is its own singleton group.
  {
    const auto groups = unionFindGroups(3, {});
    ASSERT_EQ(groups.size(), 3U);
    for (size_t i = 0; i < 3; ++i) {
      EXPECT_THAT(groups.at(i), ::testing::ElementsAre(i));
    }
  }

  // a single edge merges exactly those two nodes, leaving the third alone.
  {
    const auto groups = unionFindGroups(3, {{0, 1}});
    ASSERT_EQ(groups.size(), 2U);
    EXPECT_THAT(groups.at(0), ::testing::ElementsAre(0U, 1U));
    EXPECT_THAT(groups.at(1), ::testing::ElementsAre(2U));
  }
}

// Transitive union-find behavior: edges (0,1) and (1,2) must merge nodes 0, 1, AND 2 into one
// group, even though (0,2) is never itself an edge -- confirms this is genuine union-find/DBSCAN-
// style transitive clustering, not a literal (non-transitive) k-nearest-neighbor pairing.
TEST(TestUtils, UnionFindGroupsIsTransitive)
{
  const auto groups = unionFindGroups(3, {{0, 1}, {1, 2}});
  ASSERT_EQ(groups.size(), 1U);
  EXPECT_THAT(groups.front(), ::testing::ElementsAre(0U, 1U, 2U));
}

// [REV-F1-style regression] calcPathFrameDistance() must use arc-length-along-the-polyline +
// lateral offset, NOT raw map-frame Euclidean distance -- otherwise two points on opposite sides
// of a bend that happen to be geometrically close (e.g. a lane that loops back near itself) would
// be wrongly treated as "close" when they are actually far apart along the route. Build a path
// that loops nearly all the way around a circle, so index 1 (near the start) and index ~last (near
// the end, but geometrically almost back at the start) sit right next to each other in XY while
// being almost the entire path length apart along the polyline.
TEST(TestUtils, CalcPathFrameDistanceUsesArcLengthNotRawEuclideanOnCurvedPath)
{
  // Note: autoware::test_utils::generateTrajectory<PathWithLaneId>() places point i at radius
  // i*point_interval (an Archimedean spiral, radius growing with index), which is NOT suitable
  // here -- every point ends up farther from the origin than the last, so nothing ever loops back
  // close in XY. Build an actual constant-radius circular path by hand instead, so the path
  // genuinely loops back near itself while the arc-length-along-the-polyline stays large.
  constexpr size_t num_points = 100;
  constexpr double circumference = 100.0;
  constexpr double radius = circumference / (2.0 * M_PI);
  constexpr double delta_theta = 2.0 * M_PI / static_cast<double>(num_points);

  PathWithLaneId path;
  for (size_t i = 0; i < num_points; ++i) {
    const double theta = static_cast<double>(i) * delta_theta;
    PathPointWithLaneId p;
    p.point.pose = autoware::test_utils::createPose(
      radius * std::cos(theta), radius * std::sin(theta), 0.0, 0.0, 0.0, theta + M_PI / 2.0);
    path.points.push_back(p);
  }

  const auto & p_start = path.points.at(1).point.pose.position;
  const auto & p_near_loop_close = path.points.at(num_points - 1).point.pose.position;
  const auto & ego_pos = path.points.at(0).point.pose.position;

  const auto raw_euclidean = autoware_utils::calc_distance2d(p_start, p_near_loop_close);
  const auto path_frame_distance =
    calcPathFrameDistance(path, ego_pos, p_start, p_near_loop_close);

  // Geometrically these two points are right next to each other (the loop almost closes)...
  EXPECT_LT(raw_euclidean, 3.0);
  // ...but they are actually separated by nearly the entire route length -- a naive Euclidean
  // clustering distance would wrongly consider merging them; the path-frame metric must not.
  EXPECT_GT(path_frame_distance, 50.0);
}

TEST(TestUtils, CalcClusterConvexHullOfTwoBoxesIsTheirBoundingHull)
{
  // Two 1x1 boxes centered 3m apart along x: (0,0) and (3,0). The convex hull of their union
  // should span roughly x in [-0.5, 3.5], y in [-0.5, 0.5].
  const auto object_a = make_cluster_test_object(0.0, 0.0, 1);
  const auto object_b = make_cluster_test_object(3.0, 0.0, 2);

  const auto hull = calcClusterConvexHull({object_a, object_b});

  ASSERT_GE(hull.outer().size(), 4U);

  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();
  for (const auto & p : hull.outer()) {
    min_x = std::min(min_x, p.x());
    max_x = std::max(max_x, p.x());
    min_y = std::min(min_y, p.y());
    max_y = std::max(max_y, p.y());
  }

  EXPECT_NEAR(min_x, -0.5, 1e-6);
  EXPECT_NEAR(max_x, 3.5, 1e-6);
  EXPECT_NEAR(min_y, -0.5, 1e-6);
  EXPECT_NEAR(max_y, 0.5, 1e-6);
}

TEST(TestUtils, BuildMergedClusterObjectIsAlwaysUnknownAndAlwaysStatic)
{
  // One member is fast-moving (would fail isMovingObject() if processed individually).
  auto moving_member = make_cluster_test_object(0.0, 0.0, 1, ObjectClassification::CAR);
  moving_member.kinematics.initial_twist_with_covariance.twist.linear.x = 5.0;
  const auto static_member = make_cluster_test_object(1.0, 0.0, 2, ObjectClassification::CAR);

  const auto merged = buildMergedClusterObject({moving_member, static_member});

  ASSERT_EQ(merged.classification.size(), 1U);
  EXPECT_EQ(merged.classification.front().label, ObjectClassification::UNKNOWN);

  // [SAFETY-RELEVANT] the merged object's own twist must be zero, regardless of the moving
  // member's velocity -- this is what makes fillObjectMovingTime()/isMovingObject() treat the
  // merged blob as always-static/always-avoidable downstream, with zero changes to that
  // downstream machinery.
  EXPECT_DOUBLE_EQ(merged.kinematics.initial_twist_with_covariance.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(merged.kinematics.initial_twist_with_covariance.twist.linear.y, 0.0);

  ObjectData merged_object_data;
  merged_object_data.object = merged;
  merged_object_data.move_time = 0.0;  // fillObjectMovingTime() would set this given zero twist.
  EXPECT_FALSE(filtering_utils::isMovingObject(merged_object_data, get_parameters()))
    << "a cluster containing a moving member must not be excluded via the MOVING_OBJECT path.";
}

TEST(TestUtils, BuildMergedClusterObjectIdIsDeterministicAndOrderIndependent)
{
  const auto object_a = make_cluster_test_object(0.0, 0.0, 11);
  const auto object_b = make_cluster_test_object(1.0, 0.0, 22);

  const auto merged_ab = buildMergedClusterObject({object_a, object_b});
  const auto merged_ba = buildMergedClusterObject({object_b, object_a});

  EXPECT_EQ(
    autoware_utils::to_hex_string(merged_ab.object_id),
    autoware_utils::to_hex_string(merged_ba.object_id))
    << "merged object_id must depend only on the set of member ids, not their order, so the same "
       "underlying cluster gets the same id across cycles regardless of iteration order.";

  const auto object_c = make_cluster_test_object(2.0, 0.0, 33);
  const auto merged_abc = buildMergedClusterObject({object_a, object_b, object_c});
  EXPECT_NE(
    autoware_utils::to_hex_string(merged_ab.object_id),
    autoware_utils::to_hex_string(merged_abc.object_id))
    << "a different member set must (in practice) yield a different merged id.";
}

TEST(TestUtils, ClusterNearbyObjectsMergesTwoCloseObjects)
{
  const auto path = make_straight_reference_path(0.0);
  const auto parameters = make_cluster_test_parameters(/*max_neighbor_distance=*/2.0);
  ObjectClusterEdgeStateMap edge_states;

  const std::vector<PredictedObject> objects{
    make_cluster_test_object(5.0, 0.0, 1), make_cluster_test_object(5.5, 0.0, 2)};

  const auto result = clusterNearbyObjects(
    objects, path, geometry_msgs::msg::Point{}, parameters, edge_states);

  ASSERT_EQ(result.size(), 1U);
  EXPECT_EQ(result.front().classification.front().label, ObjectClassification::UNKNOWN);
}

TEST(TestUtils, ClusterNearbyObjectsKeepsTwoFarObjectsSeparate)
{
  const auto path = make_straight_reference_path(0.0);
  const auto parameters = make_cluster_test_parameters(/*max_neighbor_distance=*/2.0);
  ObjectClusterEdgeStateMap edge_states;

  const std::vector<PredictedObject> objects{
    make_cluster_test_object(2.0, 0.0, 1), make_cluster_test_object(18.0, 0.0, 2)};

  const auto result = clusterNearbyObjects(
    objects, path, geometry_msgs::msg::Point{}, parameters, edge_states);

  ASSERT_EQ(result.size(), 2U);
  // both objects pass through completely unchanged (not reclassified/re-shaped) since neither was
  // merged with anything.
  EXPECT_EQ(result.at(0).classification.front().label, ObjectClassification::CAR);
  EXPECT_EQ(result.at(1).classification.front().label, ObjectClassification::CAR);
}

// Transitive chain: A-B close, B-C close, A-C far apart. All three must end up in ONE merged
// cluster (genuine union-find transitivity), not two separate pairs / a pair + a singleton.
TEST(TestUtils, ClusterNearbyObjectsMergesTransitiveChainIntoOneCluster)
{
  const auto path = make_straight_reference_path(0.0);
  const auto parameters = make_cluster_test_parameters(/*max_neighbor_distance=*/1.5);
  ObjectClusterEdgeStateMap edge_states;

  // A at x=2.0, B at x=3.2 (A-B distance 1.2 < 1.5), C at x=4.4 (B-C distance 1.2 < 1.5).
  // A-C distance is 2.4 > 1.5, so A and C are only connected transitively through B.
  const std::vector<PredictedObject> objects{
    make_cluster_test_object(2.0, 0.0, 1), make_cluster_test_object(3.2, 0.0, 2),
    make_cluster_test_object(4.4, 0.0, 3)};

  const auto result = clusterNearbyObjects(
    objects, path, geometry_msgs::msg::Point{}, parameters, edge_states);

  ASSERT_EQ(result.size(), 1U);
  EXPECT_EQ(result.front().classification.front().label, ObjectClassification::UNKNOWN);
}

TEST(TestUtils, ClusterNearbyObjectsDisabledIsNoOp)
{
  const auto path = make_straight_reference_path(0.0);
  const auto parameters = make_cluster_test_parameters(
    /*max_neighbor_distance=*/2.0, /*merge_in_cycles=*/1, /*split_out_cycles=*/5,
    /*enable=*/false);
  ObjectClusterEdgeStateMap edge_states;

  const std::vector<PredictedObject> objects{
    make_cluster_test_object(5.0, 0.0, 1), make_cluster_test_object(5.5, 0.0, 2)};

  const auto result = clusterNearbyObjects(
    objects, path, geometry_msgs::msg::Point{}, parameters, edge_states);

  ASSERT_EQ(result.size(), 2U);
  EXPECT_EQ(result.at(0).classification.front().label, ObjectClassification::CAR);
  EXPECT_EQ(result.at(1).classification.front().label, ObjectClassification::CAR);
}

// [OBSTACLE-CLUSTER hysteresis] an object pair oscillating right at the threshold boundary across
// cycles must not cause cluster membership to flicker: with merge_in_cycles=1 (fast merge) and
// split_out_cycles=3 (slow split), the pair merges immediately on the first close cycle, then must
// STAY merged through brief single-cycle excursions back out past the threshold, only actually
// splitting after 3 CONSECUTIVE out-of-range cycles.
TEST(TestUtils, ClusterNearbyObjectsMembershipHysteresisPreventsFlicker)
{
  const auto path = make_straight_reference_path(0.0);
  const auto parameters =
    make_cluster_test_parameters(/*max_neighbor_distance=*/1.0, /*merge_in_cycles=*/1,
    /*split_out_cycles=*/3);
  ObjectClusterEdgeStateMap edge_states;

  const auto close_objects = [] {
    return std::vector<PredictedObject>{
      make_cluster_test_object(5.0, 0.0, 1), make_cluster_test_object(5.5, 0.0, 2)};
  };
  const auto far_objects = [] {
    return std::vector<PredictedObject>{
      make_cluster_test_object(5.0, 0.0, 1), make_cluster_test_object(6.5, 0.0, 2)};
  };

  // Cycle 1: close -> merges immediately (merge_in_cycles=1).
  {
    const auto result =
      clusterNearbyObjects(close_objects(), path, geometry_msgs::msg::Point{}, parameters, edge_states);
    ASSERT_EQ(result.size(), 1U) << "cycle 1: should merge immediately";
  }

  // Cycles 2-3: briefly out of range (2 consecutive cycles, less than split_out_cycles=3) -- must
  // still be reported as merged (flicker prevention).
  for (int cycle = 2; cycle <= 3; ++cycle) {
    const auto result =
      clusterNearbyObjects(far_objects(), path, geometry_msgs::msg::Point{}, parameters, edge_states);
    EXPECT_EQ(result.size(), 1U) << "cycle " << cycle << ": must stay merged during brief excursion";
  }

  // Cycle 4: back close again -- resets the split streak, still merged.
  {
    const auto result = clusterNearbyObjects(
      close_objects(), path, geometry_msgs::msg::Point{}, parameters, edge_states);
    EXPECT_EQ(result.size(), 1U) << "cycle 4: back in range, still merged";
  }

  // Cycles 5-7: now genuinely stay far for 3 consecutive cycles -- must actually split on the 3rd.
  {
    const auto result =
      clusterNearbyObjects(far_objects(), path, geometry_msgs::msg::Point{}, parameters, edge_states);
    EXPECT_EQ(result.size(), 1U) << "cycle 5: still within the debounce window";
  }
  {
    const auto result =
      clusterNearbyObjects(far_objects(), path, geometry_msgs::msg::Point{}, parameters, edge_states);
    EXPECT_EQ(result.size(), 1U) << "cycle 6: still within the debounce window";
  }
  {
    const auto result =
      clusterNearbyObjects(far_objects(), path, geometry_msgs::msg::Point{}, parameters, edge_states);
    EXPECT_EQ(result.size(), 2U)
      << "cycle 7: 3 consecutive out-of-range cycles -- must actually split now";
  }
}
}  // namespace autoware::behavior_path_planner::utils::static_obstacle_avoidance
