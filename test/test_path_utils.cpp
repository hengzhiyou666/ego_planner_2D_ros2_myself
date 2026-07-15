// Copyright 2026 hengzhiyou
// SPDX-License-Identifier: GPL-3.0-only

#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "path_processing/path_utils.hpp"

namespace
{

using dog_ego_planner::path_processing::PathProjection;

TEST(PathUtils, EmptyInputsAreSafe)
{
  const std::vector<Eigen::Vector2d> empty;
  EXPECT_FALSE(
    dog_ego_planner::path_processing::closestIndex(
      empty, Eigen::Vector2d::Zero()));
  EXPECT_FALSE(
    dog_ego_planner::path_processing::projectForward(
      empty, Eigen::Vector2d::Zero(), 0, 1.0).valid);
  EXPECT_TRUE(dog_ego_planner::path_processing::extractHorizon(empty, 1.0).empty());
}

TEST(PathUtils, ForwardProjectionDoesNotJumpAcrossFigureEight)
{
  const std::vector<Eigen::Vector2d> figure_eight{
    {-2.0, 0.0}, {-1.0, 1.0}, {0.0, 0.0}, {-1.0, -1.0}, {-2.0, 0.0},
    {0.0, 0.0}, {1.0, 1.0}, {2.0, 0.0}, {1.0, -1.0}, {0.0, 0.0}};
  const PathProjection projection = dog_ego_planner::path_processing::projectForward(
    figure_eight, Eigen::Vector2d(0.0, 0.0), 1, 2.0);
  ASSERT_TRUE(projection.valid);
  EXPECT_LT(projection.segment_index, 5U);
}

TEST(PathUtils, ProjectionNeverMovesBehindMinimumArcLength)
{
  const std::vector<Eigen::Vector2d> path{{0.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}};
  const PathProjection projection = dog_ego_planner::path_processing::projectForward(
    path, Eigen::Vector2d(2.0, 0.0), 0U, 10.0, 5.0);

  ASSERT_TRUE(projection.valid);
  EXPECT_NEAR(projection.arc_length_m, 5.0, 1e-9);
  EXPECT_TRUE(projection.point.isApprox(Eigen::Vector2d(5.0, 0.0)));
}

TEST(PathUtils, ReusableSuffixDoesNotContainPointsBehindRobot)
{
  const std::vector<Eigen::Vector2d> path{
    {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}};
  const Eigen::Vector2d current(1.4, 0.0);
  const PathProjection projection = dog_ego_planner::path_processing::projectForward(
    path, current, 0U, std::numeric_limits<double>::infinity());
  const auto suffix = dog_ego_planner::path_processing::unfinishedFromProjection(
    path, current, projection);

  ASSERT_GE(suffix.size(), 2U);
  EXPECT_TRUE(suffix.front().isApprox(current));
  for (const Eigen::Vector2d & point : suffix) {
    EXPECT_GE(point.x(), current.x() - 1e-9);
  }
}

TEST(PathUtils, ArcLengthProjectionClampsToPathEnd)
{
  const std::vector<Eigen::Vector2d> path{{0.0, 0.0}, {3.0, 0.0}, {3.0, 4.0}};
  const PathProjection projection =
    dog_ego_planner::path_processing::projectionAtArcLength(path, 100.0);

  ASSERT_TRUE(projection.valid);
  EXPECT_NEAR(projection.arc_length_m, 7.0, 1e-9);
  EXPECT_TRUE(projection.point.isApprox(path.back()));
}

TEST(PathUtils, SmoothingRequiresCollisionRevalidation)
{
  const std::vector<Eigen::Vector2d> corner{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};
  const std::vector<Eigen::Vector2d> obstacles{{0.90, 0.10}};
  ASSERT_TRUE(
    dog_ego_planner::path_processing::corridorIsClear(
      corner, obstacles, 0.09));
  const auto smoothed = dog_ego_planner::path_processing::smooth(
    dog_ego_planner::path_processing::densify(corner, 0.1), 4);
  EXPECT_FALSE(
    dog_ego_planner::path_processing::corridorIsClear(
      smoothed, obstacles, 0.09));
}

TEST(PathUtils, NonFinitePathNeverPassesCollisionValidation)
{
  const std::vector<Eigen::Vector2d> invalid_path{
    {0.0, 0.0}, {std::numeric_limits<double>::quiet_NaN(), 1.0}};
  EXPECT_FALSE(
    dog_ego_planner::path_processing::corridorIsClear(
      invalid_path, {}, 0.1));
}

TEST(PathUtils, HorizonAttemptsAlwaysIncludeConfiguredMinimum)
{
  const auto non_divisible = dog_ego_planner::path_processing::descendingHorizonLengths(
    7.0, 2.0, 4.0);
  ASSERT_EQ(non_divisible.size(), 3U);
  EXPECT_DOUBLE_EQ(non_divisible[0], 7.0);
  EXPECT_DOUBLE_EQ(non_divisible[1], 3.0);
  EXPECT_DOUBLE_EQ(non_divisible[2], 2.0);

  const auto oversized_step = dog_ego_planner::path_processing::descendingHorizonLengths(
    7.0, 2.0, 10.0);
  ASSERT_EQ(oversized_step.size(), 2U);
  EXPECT_DOUBLE_EQ(oversized_step.front(), 7.0);
  EXPECT_DOUBLE_EQ(oversized_step.back(), 2.0);

  const auto tiny_step = dog_ego_planner::path_processing::descendingHorizonLengths(
    7.0, 2.0, std::numeric_limits<double>::denorm_min());
  ASSERT_FALSE(tiny_step.empty());
  EXPECT_LE(tiny_step.size(), 64U);
  EXPECT_DOUBLE_EQ(tiny_step.front(), 7.0);
  EXPECT_DOUBLE_EQ(tiny_step.back(), 2.0);
}

TEST(PathUtils, PathMessagesCanUseFiniteTangents)
{
  const std::vector<Eigen::Vector2d> path{{0.0, 0.0}, {1.0, 1.0}, {2.0, 1.0}};
  EXPECT_NEAR(
    dog_ego_planner::path_processing::tangentYaw(path, 0), std::acos(-1.0) / 4.0,
    1e-9);
  EXPECT_NEAR(dog_ego_planner::path_processing::tangentYaw(path, 2), 0.0, 1e-9);
}

}  // namespace
