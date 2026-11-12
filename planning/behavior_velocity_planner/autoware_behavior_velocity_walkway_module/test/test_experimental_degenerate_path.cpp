// Copyright 2026 TIER IV, Inc.
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

// IRISA 2026-11-12: the experimental behavior_velocity_planner with the experimental walkway
// plugin (as on the robot) must survive a path whose points all sit at one position. On
// 2026-11-11 14:29 such a path (ego long out of route + localization jump) segfaulted the whole
// behavior_planning_container in WalkwayModuleManager::launchNewModules -> Trajectory::restore().

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/behavior_velocity_planner/experimental/node.hpp>
#include <autoware/planning_test_manager/autoware_planning_test_manager.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

namespace
{
using autoware::behavior_velocity_planner::experimental::BehaviorVelocityPlannerNode;
using autoware::planning_test_manager::PlanningInterfaceTestManager;

const std::string kIn = "behavior_velocity_planner_node/input/";

std::shared_ptr<BehaviorVelocityPlannerNode> make_experimental_node_with_walkway()
{
  const auto share = [](const std::string & pkg) {
    return ament_index_cpp::get_package_share_directory(pkg);
  };
  auto options = rclcpp::NodeOptions{};
  options.parameter_overrides(
    {rclcpp::Parameter(
       "launch_modules",
       std::vector<std::string>{
         "autoware::behavior_velocity_planner::experimental::WalkwayModulePlugin"}),
     rclcpp::Parameter("is_simulation", false)});
  autoware::test_utils::updateNodeOptions(
    options, {share("autoware_test_utils") + "/config/test_common.param.yaml",
              share("autoware_test_utils") + "/config/test_nearest_search.param.yaml",
              share("autoware_test_utils") + "/config/test_vehicle_info.param.yaml",
              share("autoware_velocity_smoother") + "/config/default_velocity_smoother.param.yaml",
              share("autoware_velocity_smoother") + "/config/Analytical.param.yaml",
              share("autoware_behavior_velocity_planner_common") +
                "/config/behavior_velocity_planner_common.param.yaml",
              share("autoware_behavior_velocity_planner") +
                "/config/behavior_velocity_planner.param.yaml",
              share("autoware_behavior_velocity_walkway_module") + "/config/walkway.param.yaml"});
  return std::make_shared<BehaviorVelocityPlannerNode>(options);
}

void publish_mandatory(
  const std::shared_ptr<PlanningInterfaceTestManager> & m,
  const std::shared_ptr<BehaviorVelocityPlannerNode> & n)
{
  m->publishInput(n, "/tf", autoware::test_utils::makeTFMsg(n, "base_link", "map"));
  m->publishInput(n, kIn + "accel", geometry_msgs::msg::AccelWithCovarianceStamped{});
  m->publishInput(n, kIn + "dynamic_objects", autoware_perception_msgs::msg::PredictedObjects{});
  m->publishInput(
    n, kIn + "no_ground_pointcloud",
    sensor_msgs::msg::PointCloud2{}.set__header(std_msgs::msg::Header{}.set__frame_id("base_link")));
  m->publishInput(n, kIn + "vehicle_odometry", autoware::test_utils::makeOdometry());
  m->publishInput(n, kIn + "vector_map", autoware::test_utils::makeMapBinMsg());
  m->publishInput(
    n, kIn + "traffic_signals", autoware_perception_msgs::msg::TrafficLightGroupArray{});
  m->publishInput(
    n, kIn + "external_velocity_limit_mps", autoware_internal_planning_msgs::msg::VelocityLimit{});
  m->publishInput(n, kIn + "occupancy_grid", autoware::test_utils::makeCostMapMsg());
}
}  // namespace

TEST(ExperimentalWalkwayDegeneratePath, NodeSurvivesPathWithAllPointsAtOnePosition)
{
  rclcpp::init(0, nullptr);
  auto manager = std::make_shared<PlanningInterfaceTestManager>();
  manager->subscribeOutput<autoware_planning_msgs::msg::Path>(
    "behavior_velocity_planner_node/output/path");
  auto node = make_experimental_node_with_walkway();
  publish_mandatory(manager, node);

  const auto normal = autoware::test_utils::loadPathWithLaneIdInYaml();
  ASSERT_GE(normal.points.size(), 2u);

  // 1. normal path: planned and published
  manager->publishInput(node, kIn + "path_with_lane_id", normal, 5);
  const int after_normal = manager->getReceivedTopicNum();
  EXPECT_GE(after_normal, 1);

  // 2. the crash condition: every point at the first point's position (2 and 5 points)
  for (const size_t n_points : {2u, 5u}) {
    auto degenerate = normal;
    degenerate.points.assign(n_points, normal.points.front());
    manager->publishInput(node, kIn + "path_with_lane_id", degenerate, 5);
  }
  // 2b. almost identical points (1e-9 m apart)
  {
    auto degenerate = normal;
    degenerate.points.assign(2, normal.points.front());
    degenerate.points[1].point.pose.position.x += 1e-9;
    manager->publishInput(node, kIn + "path_with_lane_id", degenerate, 5);
  }

  // 3. normal path again: the node is still alive and still planning
  manager->publishInput(node, kIn + "path_with_lane_id", normal, 5);
  EXPECT_GT(manager->getReceivedTopicNum(), after_normal);

  rclcpp::shutdown();
}
