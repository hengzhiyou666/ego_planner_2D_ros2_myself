// Copyright 2026 hengzhiyou
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>

namespace dog_ego_planner::path_processing
{

struct PathProjection
{
  bool valid{false};
  std::size_t segment_index{0};
  double alpha{0.0};
  double arc_length_m{0.0};
  double squared_distance{0.0};
  Eigen::Vector2d point{Eigen::Vector2d::Zero()};
};

double distance2D(const Eigen::Vector2d & lhs, const Eigen::Vector2d & rhs);

std::optional<std::size_t> closestIndex(
  const std::vector<Eigen::Vector2d> & points, const Eigen::Vector2d & query);

PathProjection projectForward(
  const std::vector<Eigen::Vector2d> & path, const Eigen::Vector2d & query,
  std::size_t start_segment, double max_forward_length_m,
  double minimum_arc_length_m = 0.0);

PathProjection projectionAtArcLength(
  const std::vector<Eigen::Vector2d> & path, double arc_length_m);

std::vector<Eigen::Vector2d> unfinishedFromProjection(
  const std::vector<Eigen::Vector2d> & path, const Eigen::Vector2d & current,
  const PathProjection & projection);

std::vector<Eigen::Vector2d> extractHorizon(
  const std::vector<Eigen::Vector2d> & path, double horizon_m,
  Eigen::Vector2d * temporary_goal = nullptr);

std::vector<double> descendingHorizonLengths(
  double maximum_horizon_m, double minimum_horizon_m, double step_m);

double polylineLength(const std::vector<Eigen::Vector2d> & points);

double remainingLength(
  const std::vector<Eigen::Vector2d> & points, const Eigen::Vector2d & current);

double squaredDistanceToSegment(
  const Eigen::Vector2d & query, const Eigen::Vector2d & start,
  const Eigen::Vector2d & end);

bool corridorIsClear(
  const std::vector<Eigen::Vector2d> & path,
  const std::vector<Eigen::Vector2d> & obstacles, double clearance_m);

std::vector<Eigen::Vector2d> densify(
  const std::vector<Eigen::Vector2d> & path, double step_m);

std::vector<Eigen::Vector2d> smooth(
  const std::vector<Eigen::Vector2d> & path, int iterations);

std::vector<Eigen::Vector2d> straightReference(
  const Eigen::Vector2d & start, const Eigen::Vector2d & goal,
  double step_m = 0.2);

std::vector<std::size_t> sampleControlPointIndices(
  const std::vector<Eigen::Vector2d> & path, std::size_t minimum_count = 20,
  std::size_t maximum_count = 25);

std::vector<Eigen::Vector2d> inheritedGuide(
  const Eigen::Vector2d & current,
  const std::vector<Eigen::Vector2d> & previous_local_path,
  const std::vector<Eigen::Vector2d> & unfinished_path,
  double target_length_m, double minimum_length_m);

double tangentYaw(
  const std::vector<Eigen::Vector2d> & path, std::size_t index,
  double fallback_yaw = 0.0);

std::unordered_set<std::int64_t> quantizedObstacleSet(
  const std::vector<Eigen::Vector2d> & obstacles, double resolution_m);

}  // namespace dog_ego_planner::path_processing
