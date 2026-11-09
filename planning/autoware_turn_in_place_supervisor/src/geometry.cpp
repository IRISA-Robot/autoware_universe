// Copyright 2026 azzamwildan462
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

#include "autoware/turn_in_place_supervisor/geometry.hpp"

#include <algorithm>
#include <cmath>

namespace autoware::turn_in_place_supervisor
{

double normalize_angle(const double angle)
{
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a <= 0.0) {
    a += 2.0 * M_PI;
  }
  return a - M_PI;
}

double rotation_in_direction(const double yaw, const double target, const int direction)
{
  double r = direction >= 0 ? target - yaw : yaw - target;
  r = std::fmod(r, 2.0 * M_PI);
  if (r < 0.0) {
    r += 2.0 * M_PI;
  }
  return r;
}

double sweep_radius(const Footprint & footprint, const double margin)
{
  const double x = std::max(footprint.front, footprint.rear) + margin;
  const double y = std::max(footprint.left, footprint.right) + margin;
  return std::hypot(x, y);
}

bool sweep_is_clear(
  const std::vector<Point2d> & points, const Footprint & footprint, const double margin,
  const double rotation, const double step)
{
  if (points.empty()) {
    return true;
  }
  // A point outside the full-turn circle can never be touched: skip it cheaply.
  const double r = sweep_radius(footprint, margin);
  const double r2 = r * r;
  std::vector<Point2d> near;
  near.reserve(points.size());
  for (const auto & p : points) {
    if (p.x * p.x + p.y * p.y <= r2) {
      near.push_back(p);
    }
  }
  if (near.empty()) {
    return true;
  }

  const double safe_step = std::max(step, 1e-3);
  const size_t n = static_cast<size_t>(std::ceil(std::abs(rotation) / safe_step));
  for (size_t i = 0; i <= n; ++i) {
    const double theta =
      n == 0 ? 0.0 : rotation * static_cast<double>(i) / static_cast<double>(n);
    // A point fixed in the world, seen from the body rotated by theta: rotate it by -theta.
    const double c = std::cos(-theta);
    const double s = std::sin(-theta);
    for (const auto & p : near) {
      const double bx = c * p.x - s * p.y;
      const double by = s * p.x + c * p.y;
      if (
        bx <= footprint.front + margin && bx >= -(footprint.rear + margin) &&
        by <= footprint.left + margin && by >= -(footprint.right + margin)) {
        return false;
      }
    }
  }
  return true;
}

bool point_in_polygon(const Point2d & p, const Polygon2d & polygon)
{
  const size_t n = polygon.size();
  if (n < 3) {
    return false;
  }
  bool in = false;
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const auto & a = polygon.at(i);
    const auto & b = polygon.at(j);
    if ((a.y > p.y) != (b.y > p.y)) {
      const double x = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
      if (p.x < x) {
        in = !in;
      }
    }
  }
  return in;
}

bool point_in_any(const Point2d & p, const std::vector<Polygon2d> & polygons)
{
  return std::any_of(polygons.begin(), polygons.end(), [&](const auto & polygon) {
    return point_in_polygon(p, polygon);
  });
}

std::vector<Point2d> footprint_samples(
  const Footprint & footprint, const double margin, const double spacing)
{
  const double x0 = -(footprint.rear + margin);
  const double x1 = footprint.front + margin;
  const double y0 = -(footprint.right + margin);
  const double y1 = footprint.left + margin;
  const double ds = std::max(spacing, 0.01);
  const size_t nx = static_cast<size_t>(std::ceil((x1 - x0) / ds));
  const size_t ny = static_cast<size_t>(std::ceil((y1 - y0) / ds));
  std::vector<Point2d> samples;
  samples.reserve((nx + 1) * (ny + 1));
  for (size_t i = 0; i <= nx; ++i) {
    const double x = x0 + (x1 - x0) * static_cast<double>(i) / static_cast<double>(nx);
    for (size_t j = 0; j <= ny; ++j) {
      const double y = y0 + (y1 - y0) * static_cast<double>(j) / static_cast<double>(ny);
      samples.push_back(Point2d{x, y});
    }
  }
  return samples;
}

bool sweep_inside_area(
  const std::vector<Polygon2d> & area, const Footprint & footprint, const double margin,
  const double rotation, const double step, const double spacing)
{
  if (area.empty()) {
    return false;
  }
  const auto samples = footprint_samples(footprint, margin, spacing);
  const double safe_step = std::max(step, 1e-3);
  const size_t n = static_cast<size_t>(std::ceil(std::abs(rotation) / safe_step));
  for (size_t i = 0; i <= n; ++i) {
    const double theta =
      n == 0 ? 0.0 : rotation * static_cast<double>(i) / static_cast<double>(n);
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    for (const auto & p : samples) {
      // A body point, with the body rotated by theta, in the start frame.
      const Point2d q{c * p.x - s * p.y, s * p.x + c * p.y};
      if (!point_in_any(q, area)) {
        return false;
      }
    }
  }
  return true;
}

bool disk_inside_area(
  const std::vector<Polygon2d> & area, const Point2d & centre, const double radius,
  const double angular_step)
{
  if (!point_in_any(centre, area)) {
    return false;
  }
  const double step = std::max(angular_step, 1e-3);
  const size_t n = static_cast<size_t>(std::ceil(2.0 * M_PI / step));
  // Two rings: the rim, and half way in (catches a narrow notch between rim samples).
  for (const double r : {radius, 0.5 * radius}) {
    for (size_t i = 0; i < n; ++i) {
      const double a = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(n);
      if (!point_in_any(Point2d{centre.x + r * std::cos(a), centre.y + r * std::sin(a)}, area)) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace autoware::turn_in_place_supervisor
