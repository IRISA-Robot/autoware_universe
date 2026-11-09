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

#include "autoware/behavior_path_planner_common/utils/path_direction_utils.hpp"

#include <autoware_utils/math/normalization.hpp>

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace autoware::behavior_path_planner::utils
{
double laneletEntryTangentRad(const lanelet::ConstLanelet & llt)
{
  const auto & centerline = llt.centerline();
  if (centerline.size() < 2) {
    return 0.0;
  }
  const auto & p0 = centerline[0];
  const auto & p1 = centerline[1];
  return std::atan2(p1.y() - p0.y(), p1.x() - p0.x());
}

double laneletExitTangentRad(const lanelet::ConstLanelet & llt)
{
  const auto & centerline = llt.centerline();
  const size_t n = centerline.size();
  if (n < 2) {
    return 0.0;
  }
  const auto & p0 = centerline[n - 2];
  const auto & p1 = centerline[n - 1];
  return std::atan2(p1.y() - p0.y(), p1.x() - p0.x());
}

bool isHeadingContinuousAcross(
  const lanelet::ConstLanelet & from_llt, const lanelet::ConstLanelet & to_llt)
{
  const double exit_tangent = laneletExitTangentRad(from_llt);
  const double entry_tangent = laneletEntryTangentRad(to_llt);
  const double diff = std::abs(autoware_utils::normalize_radian(entry_tangent - exit_tangent));
  return diff < kMaxContinuousHeadingChangeRad;
}

std::vector<size_t> findAllDirectionReversalIndices(
  const PathWithLaneId & path, const double reversal_threshold_rad)
{
  constexpr double min_segment_length = 1.0e-3;

  std::vector<size_t> folds;
  std::optional<double> prev_azimuth;
  for (size_t i = 0; i + 1 < path.points.size(); ++i) {
    const auto & from = path.points.at(i).point.pose.position;
    const auto & to = path.points.at(i + 1).point.pose.position;
    const double ds = std::hypot(to.x - from.x, to.y - from.y);
    if (ds < min_segment_length) {
      continue;
    }
    const double azimuth = std::atan2(to.y - from.y, to.x - from.x);
    if (prev_azimuth) {
      const double raw = azimuth - *prev_azimuth;
      const double delta = std::abs(std::atan2(std::sin(raw), std::cos(raw)));
      if (delta > reversal_threshold_rad) {
        folds.push_back(i);
      }
    }
    prev_azimuth = azimuth;
  }
  return folds;
}

std::optional<size_t> findFirstDirectionReversalIndex(
  const PathWithLaneId & path, const double reversal_threshold_rad)
{
  const auto folds = findAllDirectionReversalIndices(path, reversal_threshold_rad);
  if (folds.empty()) {
    return std::nullopt;
  }
  return folds.front();
}

namespace
{
/// Travel direction at point i: its outgoing non-degenerate segment, else its incoming one.
std::optional<double> travelAzimuthAt(const PathWithLaneId & path, const size_t i)
{
  constexpr double min_segment_length = 1.0e-3;
  const auto azimuth = [&](const size_t a, const size_t b) -> std::optional<double> {
    const auto & p = path.points.at(a).point.pose.position;
    const auto & q = path.points.at(b).point.pose.position;
    if (std::hypot(q.x - p.x, q.y - p.y) < min_segment_length) {
      return std::nullopt;
    }
    return std::atan2(q.y - p.y, q.x - p.x);
  };
  for (size_t j = i; j + 1 < path.points.size(); ++j) {
    if (const auto az = azimuth(j, j + 1)) return az;
  }
  for (size_t j = i; j > 0; --j) {
    if (const auto az = azimuth(j - 1, j)) return az;
  }
  return std::nullopt;
}

size_t findEgoLegIndex(const PathWithLaneId & path, const geometry_msgs::msg::Pose & ego_pose)
{
  const double ego_yaw = 2.0 * std::atan2(ego_pose.orientation.z, ego_pose.orientation.w);
  std::optional<size_t> best_aligned;
  double best_aligned_d2 = std::numeric_limits<double>::max();
  size_t best_any = 0;
  double best_any_d2 = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path.points.size(); ++i) {
    const auto & p = path.points.at(i).point.pose.position;
    const double d2 = (p.x - ego_pose.position.x) * (p.x - ego_pose.position.x) +
                      (p.y - ego_pose.position.y) * (p.y - ego_pose.position.y);
    if (d2 < best_any_d2) {
      best_any_d2 = d2;
      best_any = i;
    }
    const auto az = travelAzimuthAt(path, i);
    if (!az) continue;
    // [TURN-IN-PLACE 2026-11-05] A reversing point (negative velocity) is travelled with the
    // body facing away from the travel direction: compare it with ego's travel heading
    // (yaw + pi), or a reversing ego would be matched to the leg running the other way.
    const double travel_yaw =
      path.points.at(i).point.longitudinal_velocity_mps < 0.0F ? ego_yaw + M_PI : ego_yaw;
    const double diff = std::abs(autoware_utils::normalize_radian(*az - travel_yaw));
    if (diff < M_PI / 2.0 && d2 < best_aligned_d2) {
      best_aligned_d2 = d2;
      best_aligned = i;
    }
  }
  return best_aligned.value_or(best_any);
}
}  // namespace

std::optional<FoldTruncation> truncatePathAtFold(
  PathWithLaneId & path, const double reversal_threshold_rad,
  const std::optional<geometry_msgs::msg::Pose> & ego_pose)
{
  const auto folds = findAllDirectionReversalIndices(path, reversal_threshold_rad);
  if (folds.empty()) {
    return std::nullopt;
  }
  const size_t ego_idx = ego_pose ? findEgoLegIndex(path, *ego_pose) : 0;

  FoldTruncation result;
  result.kept_from = 0;
  result.kept_to = path.points.size() - 1;
  for (const auto fold : folds) {
    if (fold <= ego_idx) {
      result.kept_from = fold;
    } else {
      result.kept_to = fold;
      result.stops_at_end = true;
      break;
    }
  }

  path.points.erase(
    path.points.begin() + static_cast<std::ptrdiff_t>(result.kept_to) + 1, path.points.end());
  path.points.erase(
    path.points.begin(), path.points.begin() + static_cast<std::ptrdiff_t>(result.kept_from));
  if (result.stops_at_end) {
    path.points.back().point.longitudinal_velocity_mps = 0.0f;
  }
  return result;
}
}  // namespace autoware::behavior_path_planner::utils
