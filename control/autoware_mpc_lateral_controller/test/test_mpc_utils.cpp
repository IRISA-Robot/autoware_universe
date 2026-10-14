// Copyright 2021 The Autoware Foundation
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

#include "autoware/mpc_lateral_controller/mpc_trajectory.hpp"
#include "autoware/mpc_lateral_controller/mpc_utils.hpp"
#include "gtest/gtest.h"

#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/trajectory_point.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <cmath>
#include <memory>
#include <utility>
#include <vector>

namespace
{
namespace MPCUtils = autoware::motion::control::mpc_lateral_controller::MPCUtils;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

TrajectoryPoint makePoint(const double x, const double y, const float vx)
{
  TrajectoryPoint p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.longitudinal_velocity_mps = vx;
  return p;
}

/* cppcheck-suppress syntaxError */
TEST(TestMPC, CalcStopDistance)
{
  constexpr float MOVE = 1.0f;
  constexpr float STOP = 0.0f;

  Trajectory trajectory_msg;
  trajectory_msg.points.push_back(makePoint(0.0, 0.0, MOVE));
  trajectory_msg.points.push_back(makePoint(1.0, 0.0, MOVE));
  trajectory_msg.points.push_back(makePoint(2.0, 0.0, STOP));  // STOP
  trajectory_msg.points.push_back(makePoint(3.0, 0.0, MOVE));
  trajectory_msg.points.push_back(makePoint(4.0, 0.0, MOVE));
  trajectory_msg.points.push_back(makePoint(5.0, 0.0, MOVE));
  trajectory_msg.points.push_back(makePoint(6.0, 0.0, STOP));  // STOP
  trajectory_msg.points.push_back(makePoint(7.0, 0.0, STOP));  // STOP

  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 0), 2.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 1), 1.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 2), 0.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 3), 3.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 4), 2.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 5), 1.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 6), 0.0);
  EXPECT_EQ(MPCUtils::calcStopDistance(trajectory_msg, 7), -1.0);
}
}  // namespace

namespace
{
// /planning/trajectory at lanelet 303 (hibikino_kanan, reverse, bag urutn2 t=176.38): the leg
// coming out of 188 with a wiggle at the joint, then the 121 deg corner.
const std::vector<std::pair<double, double>> lanelet_303_corner = {
  {406.563, 115.801}, {406.612, 115.714}, {406.661, 115.626}, {406.710, 115.539},
  {406.759, 115.452}, {406.808, 115.365}, {406.857, 115.277}, {406.906, 115.190},
  {406.955, 115.103}, {407.003, 115.016}, {407.052, 114.928}, {407.101, 114.841},
  {407.150, 114.754}, {407.199, 114.667}, {407.248, 114.580}, {407.297, 114.493},
  {407.359, 114.414}, {407.431, 114.345}, {407.508, 114.281}, {407.585, 114.217},
  {407.658, 114.148}, {407.719, 114.069}, {407.767, 113.982}, {407.811, 113.892},
  {407.852, 113.801}, {407.891, 113.708}, {407.928, 113.615}, {407.964, 113.522},
  {408.000, 113.429}, {408.035, 113.335}, {408.069, 113.241}, {408.101, 113.147},
  {408.129, 113.051}, {408.152, 112.953}, {408.167, 112.854}, {408.164, 112.754},
  {408.146, 112.656}, {408.115, 112.561}, {408.077, 112.469}, {408.075, 112.464},
  {408.030, 112.375}, {407.981, 112.288}, {407.929, 112.202}, {407.875, 112.118},
  {407.818, 112.035}, {407.761, 111.954}, {407.702, 111.873}, {407.644, 111.791},
  {407.585, 111.710}, {407.525, 111.630}, {407.463, 111.552}, {407.398, 111.476},
  {407.330, 111.403}, {407.256, 111.335}, {407.178, 111.273}, {407.096, 111.215},
  {407.011, 111.163}, {406.923, 111.116}, {406.832, 111.074}, {406.741, 111.033},
  {406.649, 110.993}, {406.558, 110.952}, {406.466, 110.911}, {406.375, 110.871},
  {406.284, 110.830}, {406.192, 110.789}, {406.101, 110.749}, {406.010, 110.708},
  {405.918, 110.667}, {405.827, 110.627}, {405.736, 110.586}, {405.644, 110.545},
  {405.553, 110.505}, {405.462, 110.464}, {405.370, 110.423}};
}  // namespace

TEST(TestMPC, FindNearestConformingIndexPastTightCorner)
{
  using autoware::motion::control::mpc_lateral_controller::MPCTrajectory;
  MPCTrajectory traj;
  double t = 0.0;
  for (const auto & [x, y] : lanelet_303_corner) {
    traj.push_back(x, y, 0.0, 0.0, -0.68, 0.0, 0.0, t);
    t += 0.15;
  }
  MPCUtils::calcTrajectoryYawFromXY(traj, false);  // reverse: yaw = body heading

  geometry_msgs::msg::Pose ego;  // past the apex, nose at 78 deg
  ego.position.x = 408.15;
  ego.position.y = 112.58;
  ego.orientation = autoware_utils::create_quaternion_from_yaw(78.0 * M_PI / 180.0);
  constexpr double max_dist = 3.0;
  const double max_yaw = M_PI / 3.0;

  const auto dist_to_ego = [&](const size_t i) {
    return std::hypot(traj.x.at(i) - ego.position.x, traj.y.at(i) - ego.position.y);
  };

  // the first conforming run is the leg into the corner, 2 m behind the robot
  const size_t first_run = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    MPCUtils::convertToAutowareTrajectory(traj).points, ego, max_dist, max_yaw);
  EXPECT_GT(dist_to_ego(first_run), 1.5);

  const size_t nearest = MPCUtils::findNearestConformingIndex(traj, ego, max_dist, max_yaw);
  EXPECT_LT(dist_to_ego(nearest), 0.1);

  geometry_msgs::msg::Pose nearest_pose;
  size_t nearest_index = 0;
  double nearest_time = 0.0;
  ASSERT_TRUE(MPCUtils::calcNearestPoseInterp(
    traj, ego, &nearest_pose, &nearest_index, &nearest_time, max_dist, max_yaw));
  EXPECT_LT(
    std::hypot(nearest_pose.position.x - ego.position.x, nearest_pose.position.y - ego.position.y),
    0.1);
}

TEST(TestMPC, FindNearestConformingIndexSkipsOppositeLeg)
{
  using autoware::motion::control::mpc_lateral_controller::MPCTrajectory;
  // out along y = 0 heading +x, back along y = 1 heading -x
  MPCTrajectory traj;
  for (int i = 0; i <= 20; ++i) traj.push_back(0.5 * i, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.1 * i);
  for (int i = 20; i >= 0; --i) traj.push_back(0.5 * i, 1.0, 0.0, M_PI, 1.0, 0.0, 0.0, 3.0 - 0.1 * i);

  geometry_msgs::msg::Pose ego;
  ego.position.x = 5.0;
  ego.position.y = 0.4;  // nearer the outbound leg, but facing the return direction
  ego.orientation = autoware_utils::create_quaternion_from_yaw(M_PI);

  const size_t nearest = MPCUtils::findNearestConformingIndex(traj, ego, 3.0, M_PI / 3.0);
  EXPECT_DOUBLE_EQ(traj.y.at(nearest), 1.0);
  EXPECT_DOUBLE_EQ(traj.x.at(nearest), 5.0);
}
