// Copyright 2026 hengzhiyou
// SPDX-License-Identifier: GPL-3.0-only

#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "planning/a_star.hpp"

namespace
{
std::shared_ptr<GridMap2D> makeMap(int extent_m)
{
  auto map = std::make_shared<GridMap2D>(1.0, Eigen::Vector2i(extent_m, extent_m));
  map->setInflateRadius(0.0);
  return map;
}

void setWorldObstacle(GridMap2D & map, double x, double y)
{
  map.setObstacle(map.worldToGrid(Eigen::Vector2d(x, y)));
}
}  // namespace

TEST(AStar, FindsStraightPathInFreeSpace)
{
  const auto map = makeMap(20);
  AStar search;
  search.initGridMap(map, Eigen::Vector2i(30, 30));

  ASSERT_TRUE(
    search.AstarSearch(
      1.0, Eigen::Vector2d(-4.0, 0.0),
      Eigen::Vector2d(4.0, 0.0)));
  const std::vector<Eigen::Vector2d> path = search.getPath();

  ASSERT_FALSE(path.empty());
  EXPECT_TRUE(path.front().isApprox(Eigen::Vector2d(-4.0, 0.0)));
  EXPECT_TRUE(path.back().isApprox(Eigen::Vector2d(4.0, 0.0)));
  for (const Eigen::Vector2d & point : path) {
    EXPECT_DOUBLE_EQ(point.y(), 0.0);
    EXPECT_FALSE(map->getInflateOccupancy(point));
  }
}

TEST(AStar, RoutesAroundObstacleWithoutEnteringOccupiedCells)
{
  const auto map = makeMap(20);
  for (int y = -2; y <= 2; ++y) {
    setWorldObstacle(*map, 0.0, static_cast<double>(y));
  }

  AStar search;
  search.initGridMap(map, Eigen::Vector2i(30, 30));
  ASSERT_TRUE(
    search.AstarSearch(
      1.0, Eigen::Vector2d(-4.0, 0.0),
      Eigen::Vector2d(4.0, 0.0)));

  bool left_straight_corridor = false;
  for (const Eigen::Vector2d & point : search.getPath()) {
    EXPECT_FALSE(map->getInflateOccupancy(point));
    left_straight_corridor = left_straight_corridor || std::abs(point.y()) >= 3.0;
  }
  EXPECT_TRUE(left_straight_corridor);
}

TEST(AStar, FailureClearsPathFromPreviousSearch)
{
  const auto map = makeMap(10);
  AStar search;
  search.initGridMap(map, Eigen::Vector2i(20, 20));
  ASSERT_TRUE(
    search.AstarSearch(
      1.0, Eigen::Vector2d(-2.0, 0.0),
      Eigen::Vector2d(2.0, 0.0)));
  ASSERT_FALSE(search.getPath().empty());

  for (int y_index = 0; y_index < map->mapSize().y(); ++y_index) {
    map->setObstacle(Eigen::Vector2i(map->worldToGrid(Eigen::Vector2d(0.0, 0.0)).x(), y_index));
  }

  EXPECT_FALSE(
    search.AstarSearch(
      1.0, Eigen::Vector2d(-2.0, 0.0),
      Eigen::Vector2d(2.0, 0.0)));
  EXPECT_TRUE(search.getPath().empty());
}

TEST(AStar, DoesNotCutAcrossBlockedDiagonalCorner)
{
  const auto map = makeMap(6);
  const std::vector<Eigen::Vector2d> blocked_neighbours{
    {-2.0, -2.0}, {-2.0, -1.0}, {-2.0, 0.0}, {-1.0, -2.0},
    {0.0, -2.0}, {0.0, -1.0}, {-1.0, 0.0}};
  for (const Eigen::Vector2d & point : blocked_neighbours) {
    setWorldObstacle(*map, point.x(), point.y());
  }

  AStar search;
  search.initGridMap(map, Eigen::Vector2i(10, 10));
  EXPECT_FALSE(
    search.AstarSearch(
      1.0, Eigen::Vector2d(-1.0, -1.0),
      Eigen::Vector2d(1.0, 1.0)));
  EXPECT_TRUE(search.getPath().empty());
}

TEST(AStar, TraversesOneCellWidePassageWithoutTouchingWalls)
{
  const auto map = makeMap(20);
  for (int x = -5; x <= 5; ++x) {
    setWorldObstacle(*map, static_cast<double>(x), -1.0);
    setWorldObstacle(*map, static_cast<double>(x), 1.0);
  }

  AStar search;
  search.initGridMap(map, Eigen::Vector2i(30, 30));
  ASSERT_TRUE(
    search.AstarSearch(
      1.0, Eigen::Vector2d(-4.0, 0.0), Eigen::Vector2d(4.0, 0.0)));
  for (const Eigen::Vector2d & point : search.getPath()) {
    EXPECT_DOUBLE_EQ(point.y(), 0.0);
    EXPECT_FALSE(map->getInflateOccupancy(point));
  }
}

TEST(AStar, BlockedCoincidentStartAndGoalFailsCleanly)
{
  const auto map = makeMap(10);
  setWorldObstacle(*map, 0.0, 0.0);
  AStar search;
  search.initGridMap(map, Eigen::Vector2i(20, 20));

  EXPECT_FALSE(
    search.AstarSearch(
      1.0, Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, 0.0)));
  EXPECT_TRUE(search.getPath().empty());
}

TEST(AStar, SearchStrideNeverExceedsMapResolution)
{
  auto map = std::make_shared<GridMap2D>(0.1, Eigen::Vector2i(4, 4));
  map->setInflateRadius(0.0);
  AStar search;
  search.initGridMap(map, Eigen::Vector2i(100, 100));

  ASSERT_TRUE(
    search.AstarSearch(
      0.5, Eigen::Vector2d(-0.5, 0.0), Eigen::Vector2d(0.5, 0.0)));
  const auto path = search.getPath();
  ASSERT_GE(path.size(), 2U);
  for (std::size_t index = 1; index < path.size(); ++index) {
    EXPECT_LE((path[index] - path[index - 1]).norm(), std::sqrt(2.0) * 0.1 + 1e-9);
  }
}
