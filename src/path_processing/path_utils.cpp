// Copyright 2026 hengzhiyou
// SPDX-License-Identifier: GPL-3.0-only

#include "path_processing/path_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

namespace dog_ego_planner::path_processing
{
namespace
{

constexpr double kEpsilon = 1e-9;

double angleDegrees(const Eigen::Vector2d & first, const Eigen::Vector2d & second)
{
  const double first_norm = first.norm();
  const double second_norm = second.norm();
  if (first_norm < kEpsilon || second_norm < kEpsilon) {
    return 0.0;
  }
  const double cosine = std::clamp(
    first.dot(second) / (first_norm * second_norm), -1.0, 1.0);
  return std::acos(cosine) * 180.0 / std::acos(-1.0);
}

}  // namespace

double distance2D(const Eigen::Vector2d & lhs, const Eigen::Vector2d & rhs)
{
  return (lhs - rhs).norm();
}

std::optional<std::size_t> closestIndex(
  const std::vector<Eigen::Vector2d> & points, const Eigen::Vector2d & query)
{
  if (points.empty()) {
    return std::nullopt;
  }
  std::size_t best_index = 0;
  double best_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < points.size(); ++index) {
    const double distance = (points[index] - query).squaredNorm();
    if (distance < best_distance) {
      best_distance = distance;
      best_index = index;
    }
  }
  return best_index;
}

double squaredDistanceToSegment(
  const Eigen::Vector2d & query, const Eigen::Vector2d & start,
  const Eigen::Vector2d & end)
{
  const Eigen::Vector2d segment = end - start;
  const double segment_squared = segment.squaredNorm();
  if (segment_squared <= kEpsilon) {
    return (query - start).squaredNorm();
  }
  const double alpha = std::clamp(
    (query - start).dot(segment) / segment_squared, 0.0, 1.0);
  return (query - (start + alpha * segment)).squaredNorm();
}

PathProjection projectForward(
  const std::vector<Eigen::Vector2d> & path, const Eigen::Vector2d & query,
  std::size_t start_segment, double max_forward_length_m,
  double minimum_arc_length_m)
{
  PathProjection best;
  best.squared_distance = std::numeric_limits<double>::infinity();
  if (path.size() < 2 || !query.allFinite()) {
    return best;
  }

  start_segment = std::min(start_segment, path.size() - 2);
  minimum_arc_length_m = std::max(0.0, minimum_arc_length_m);
  const double search_length =
    std::isfinite(max_forward_length_m) && max_forward_length_m > 0.0 ?
    max_forward_length_m : std::numeric_limits<double>::infinity();
  double segment_arc_start = 0.0;
  for (std::size_t index = 0; index < start_segment; ++index) {
    segment_arc_start += (path[index + 1] - path[index]).norm();
  }
  const double search_origin_arc = std::max(segment_arc_start, minimum_arc_length_m);

  for (std::size_t index = start_segment; index + 1 < path.size(); ++index) {
    const Eigen::Vector2d & start = path[index];
    const Eigen::Vector2d & end = path[index + 1];
    const Eigen::Vector2d segment = end - start;
    const double segment_squared = segment.squaredNorm();
    const double segment_length = std::sqrt(segment_squared);
    const double segment_arc_end = segment_arc_start + segment_length;
    if (segment_arc_end + kEpsilon < minimum_arc_length_m) {
      segment_arc_start = segment_arc_end;
      continue;
    }
    if (segment_arc_start - search_origin_arc > search_length + kEpsilon) {
      break;
    }

    double alpha = 0.0;
    Eigen::Vector2d projected = start;
    if (segment_squared > kEpsilon) {
      const double minimum_alpha = std::clamp(
        (minimum_arc_length_m - segment_arc_start) / segment_length, 0.0, 1.0);
      const double maximum_alpha = std::isfinite(search_length) ?
        std::clamp(
        (search_origin_arc + search_length - segment_arc_start) / segment_length,
        minimum_alpha, 1.0) : 1.0;
      alpha = std::clamp(
        (query - start).dot(segment) / segment_squared, minimum_alpha, maximum_alpha);
      projected = start + alpha * segment;
    }
    const double squared_distance = (query - projected).squaredNorm();
    if (squared_distance < best.squared_distance) {
      best.valid = true;
      best.segment_index = index;
      best.alpha = alpha;
      best.arc_length_m = segment_arc_start + alpha * segment_length;
      best.squared_distance = squared_distance;
      best.point = projected;
    }
    segment_arc_start = segment_arc_end;
  }
  return best;
}

PathProjection projectionAtArcLength(
  const std::vector<Eigen::Vector2d> & path, double arc_length_m)
{
  PathProjection projection;
  if (path.size() < 2 || !std::isfinite(arc_length_m)) {
    return projection;
  }

  const double requested = std::max(0.0, arc_length_m);
  double accumulated = 0.0;
  for (std::size_t index = 0; index + 1 < path.size(); ++index) {
    const Eigen::Vector2d segment = path[index + 1] - path[index];
    const double length = segment.norm();
    if (length > kEpsilon && accumulated + length >= requested - kEpsilon) {
      projection.valid = true;
      projection.segment_index = index;
      projection.alpha = std::clamp((requested - accumulated) / length, 0.0, 1.0);
      projection.arc_length_m = accumulated + projection.alpha * length;
      projection.point = path[index] + projection.alpha * segment;
      projection.squared_distance = 0.0;
      return projection;
    }
    accumulated += length;
  }

  projection.valid = true;
  projection.segment_index = path.size() - 2;
  projection.alpha = 1.0;
  projection.arc_length_m = accumulated;
  projection.point = path.back();
  projection.squared_distance = 0.0;
  return projection;
}

std::vector<Eigen::Vector2d> unfinishedFromProjection(
  const std::vector<Eigen::Vector2d> & path, const Eigen::Vector2d & current,
  const PathProjection & projection)
{
  std::vector<Eigen::Vector2d> result;
  if (!projection.valid || path.size() < 2 || projection.segment_index + 1 >= path.size()) {
    return result;
  }

  result.reserve(path.size() - projection.segment_index + 1);
  result.push_back(current);
  if ((projection.point - current).norm() > kEpsilon) {
    result.push_back(projection.point);
  }
  for (std::size_t index = projection.segment_index + 1; index < path.size(); ++index) {
    if (result.empty() || (path[index] - result.back()).norm() > kEpsilon) {
      result.push_back(path[index]);
    }
  }
  return result;
}

std::vector<Eigen::Vector2d> extractHorizon(
  const std::vector<Eigen::Vector2d> & path, double horizon_m,
  Eigen::Vector2d * temporary_goal)
{
  std::vector<Eigen::Vector2d> result;
  if (path.size() < 2 || !std::isfinite(horizon_m) || horizon_m <= 0.0) {
    return result;
  }

  result.reserve(path.size());
  result.push_back(path.front());
  double accumulated = 0.0;
  for (std::size_t index = 1; index < path.size(); ++index) {
    const Eigen::Vector2d delta = path[index] - path[index - 1];
    const double segment_length = delta.norm();
    if (segment_length > kEpsilon && accumulated + segment_length >= horizon_m) {
      const double alpha = (horizon_m - accumulated) / segment_length;
      const Eigen::Vector2d endpoint = path[index - 1] + alpha * delta;
      result.push_back(endpoint);
      if (temporary_goal != nullptr) {
        *temporary_goal = endpoint;
      }
      return result;
    }
    if (segment_length > kEpsilon) {
      result.push_back(path[index]);
      accumulated += segment_length;
    }
  }
  if (temporary_goal != nullptr) {
    *temporary_goal = path.back();
  }
  return result;
}

std::vector<double> descendingHorizonLengths(
  double maximum_horizon_m, double minimum_horizon_m, double step_m)
{
  std::vector<double> result;
  if (!std::isfinite(maximum_horizon_m) || !std::isfinite(minimum_horizon_m) ||
    !std::isfinite(step_m) || maximum_horizon_m <= 0.0 || minimum_horizon_m <= 0.0 ||
    minimum_horizon_m > maximum_horizon_m || step_m <= 0.0)
  {
    return result;
  }

  constexpr std::size_t kMaximumAttempts = 64U;
  const double tolerance = kEpsilon * std::max(
    {1.0, std::abs(maximum_horizon_m), std::abs(minimum_horizon_m)});
  result.push_back(maximum_horizon_m);
  double current = maximum_horizon_m;
  while (current > minimum_horizon_m + tolerance && result.size() + 1U < kMaximumAttempts) {
    const double next = std::max(minimum_horizon_m, current - step_m);
    if (!std::isfinite(next) || next >= current - tolerance) {
      break;
    }
    result.push_back(next);
    current = next;
  }
  if (result.back() > minimum_horizon_m + tolerance) {
    result.push_back(minimum_horizon_m);
  } else {
    result.back() = minimum_horizon_m;
  }
  return result;
}

double polylineLength(const std::vector<Eigen::Vector2d> & points)
{
  double length = 0.0;
  for (std::size_t index = 1; index < points.size(); ++index) {
    length += (points[index] - points[index - 1]).norm();
  }
  return length;
}

double remainingLength(
  const std::vector<Eigen::Vector2d> & points, const Eigen::Vector2d & current)
{
  const auto closest = closestIndex(points, current);
  if (!closest || points.size() < 2) {
    return 0.0;
  }
  double length = 0.0;
  for (std::size_t index = *closest + 1; index < points.size(); ++index) {
    length += (points[index] - points[index - 1]).norm();
  }
  return length;
}

bool corridorIsClear(
  const std::vector<Eigen::Vector2d> & path,
  const std::vector<Eigen::Vector2d> & obstacles, double clearance_m)
{
  if (path.size() < 2 || !std::isfinite(clearance_m) || clearance_m < 0.0) {
    return false;
  }
  if (!std::all_of(
      path.begin(), path.end(), [](const Eigen::Vector2d & point) {return point.allFinite();}))
  {
    return false;
  }
  const double clearance_squared = clearance_m * clearance_m;
  for (const Eigen::Vector2d & obstacle : obstacles) {
    if (!obstacle.allFinite()) {
      continue;
    }
    for (std::size_t index = 0; index + 1 < path.size(); ++index) {
      if (squaredDistanceToSegment(obstacle, path[index], path[index + 1]) <=
        clearance_squared)
      {
        return false;
      }
    }
  }
  return true;
}

std::vector<Eigen::Vector2d> densify(
  const std::vector<Eigen::Vector2d> & path, double step_m)
{
  if (path.size() < 2) {
    return path;
  }
  const double step = std::max(0.02, step_m);
  std::vector<Eigen::Vector2d> result;
  result.reserve(path.size() * 4);
  result.push_back(path.front());
  for (std::size_t index = 0; index + 1 < path.size(); ++index) {
    const Eigen::Vector2d delta = path[index + 1] - path[index];
    const double segment_length = delta.norm();
    if (segment_length <= kEpsilon) {
      continue;
    }
    const Eigen::Vector2d direction = delta / segment_length;
    for (double distance = step; distance < segment_length; distance += step) {
      result.push_back(path[index] + direction * distance);
    }
    result.push_back(path[index + 1]);
  }
  return result;
}

std::vector<Eigen::Vector2d> smooth(
  const std::vector<Eigen::Vector2d> & path, int iterations)
{
  if (path.size() < 3 || iterations <= 0) {
    return path;
  }
  std::vector<Eigen::Vector2d> current = path;
  std::vector<Eigen::Vector2d> next = path;
  const int bounded_iterations = std::clamp(iterations, 1, 10);
  for (int iteration = 0; iteration < bounded_iterations; ++iteration) {
    next = current;
    for (std::size_t index = 1; index + 1 < current.size(); ++index) {
      next[index] =
        0.25 * current[index - 1] + 0.5 * current[index] + 0.25 * current[index + 1];
    }
    current.swap(next);
  }
  return current;
}

std::vector<Eigen::Vector2d> straightReference(
  const Eigen::Vector2d & start, const Eigen::Vector2d & goal, double step_m)
{
  std::vector<Eigen::Vector2d> result;
  const double length = (goal - start).norm();
  if (length <= kEpsilon || !std::isfinite(length)) {
    return result;
  }
  const int segment_count =
    std::max(3, static_cast<int>(std::ceil(length / std::max(0.05, step_m))));
  result.reserve(static_cast<std::size_t>(segment_count) + 1);
  for (int index = 0; index <= segment_count; ++index) {
    const double alpha = static_cast<double>(index) / static_cast<double>(segment_count);
    result.push_back(start + alpha * (goal - start));
  }
  return result;
}

std::vector<std::size_t> sampleControlPointIndices(
  const std::vector<Eigen::Vector2d> & path, std::size_t minimum_count,
  std::size_t maximum_count)
{
  if (path.empty()) {
    return {};
  }
  minimum_count = std::max<std::size_t>(4, minimum_count);
  maximum_count = std::max(minimum_count, maximum_count);
  std::unordered_set<std::size_t> selected{0, path.size() - 1};
  for (std::size_t index = 0; index < path.size(); index += 3) {
    selected.insert(index);
  }
  for (std::size_t index = 1; index + 1 < path.size(); ++index) {
    if (angleDegrees(path[index] - path[index - 1], path[index + 1] - path[index]) > 15.0) {
      selected.insert(index);
    }
  }

  if (selected.size() < minimum_count) {
    const std::size_t desired = std::min(minimum_count, path.size());
    for (std::size_t index = 0; index < desired; ++index) {
      const double alpha = desired == 1 ? 0.0 :
        static_cast<double>(index) / static_cast<double>(desired - 1);
      selected.insert(
        static_cast<std::size_t>(
          std::llround(alpha * static_cast<double>(path.size() - 1))));
    }
  }

  std::vector<std::size_t> result(selected.begin(), selected.end());
  std::sort(result.begin(), result.end());
  if (result.size() > maximum_count) {
    std::vector<std::size_t> reduced;
    reduced.reserve(maximum_count);
    for (std::size_t index = 0; index < maximum_count; ++index) {
      const double alpha = maximum_count == 1 ? 0.0 :
        static_cast<double>(index) / static_cast<double>(maximum_count - 1);
      reduced.push_back(
        result[static_cast<std::size_t>(
          std::llround(alpha * static_cast<double>(result.size() - 1)))]);
    }
    std::sort(reduced.begin(), reduced.end());
    reduced.erase(std::unique(reduced.begin(), reduced.end()), reduced.end());
    result = std::move(reduced);
  }
  return result;
}

std::vector<Eigen::Vector2d> inheritedGuide(
  const Eigen::Vector2d & current,
  const std::vector<Eigen::Vector2d> & previous_local_path,
  const std::vector<Eigen::Vector2d> & unfinished_path,
  double target_length_m, double minimum_length_m)
{
  if (previous_local_path.size() < 2) {
    return unfinished_path;
  }
  const auto closest = closestIndex(previous_local_path, current);
  if (!closest) {
    return unfinished_path;
  }

  std::vector<Eigen::Vector2d> result{current};
  for (std::size_t index = *closest + 1; index < previous_local_path.size(); ++index) {
    result.push_back(previous_local_path[index]);
  }
  if (unfinished_path.size() >= 2) {
    const PathProjection anchor = projectForward(
      unfinished_path, result.back(), 0, std::max(target_length_m * 1.5, minimum_length_m));
    const double maximum_anchor_distance = std::max(0.5, minimum_length_m * 0.25);
    if (!anchor.valid || anchor.squared_distance >
      maximum_anchor_distance * maximum_anchor_distance)
    {
      return unfinished_path;
    }
    const std::size_t start = anchor.segment_index + 1;
    for (std::size_t index = start; index < unfinished_path.size(); ++index) {
      result.push_back(unfinished_path[index]);
      if (polylineLength(result) >= target_length_m) {
        break;
      }
    }
  }
  return result.size() >= 2 && polylineLength(result) >= minimum_length_m ?
         result : unfinished_path;
}

double tangentYaw(
  const std::vector<Eigen::Vector2d> & path, std::size_t index,
  double fallback_yaw)
{
  if (path.size() < 2) {
    return fallback_yaw;
  }
  index = std::min(index, path.size() - 1);
  Eigen::Vector2d tangent;
  if (index + 1 < path.size()) {
    tangent = path[index + 1] - path[index];
  } else {
    tangent = path[index] - path[index - 1];
  }
  return tangent.norm() > kEpsilon ? std::atan2(tangent.y(), tangent.x()) : fallback_yaw;
}

std::unordered_set<std::int64_t> quantizedObstacleSet(
  const std::vector<Eigen::Vector2d> & obstacles, double resolution_m)
{
  std::unordered_set<std::int64_t> result;
  const double resolution = std::max(std::abs(resolution_m), 1e-6);
  result.reserve(obstacles.size());
  for (const Eigen::Vector2d & obstacle : obstacles) {
    if (!obstacle.allFinite()) {
      continue;
    }
    const auto x = static_cast<std::int64_t>(std::llround(obstacle.x() / resolution));
    const auto y = static_cast<std::int64_t>(std::llround(obstacle.y() / resolution));
    const auto packed =
      (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32U) |
      static_cast<std::uint32_t>(y);
    result.insert(static_cast<std::int64_t>(packed));
  }
  return result;
}

}  // namespace dog_ego_planner::path_processing
