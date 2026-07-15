// Copyright 2026 hengzhiyou
// SPDX-License-Identifier: GPL-3.0-only

#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "mapping/grid_map.hpp"

TEST(GridMap2D, IsUsableImmediatelyAndUsesOccupancyGridCoordinates)
{
  GridMap2D map(0.5, Eigen::Vector2i(4, 2));

  EXPECT_EQ(map.mapSize(), Eigen::Vector2i(8, 4));
  EXPECT_DOUBLE_EQ(map.resolution(), 0.5);
  EXPECT_TRUE(map.origin().isApprox(Eigen::Vector2d(-2.0, -1.0)));
  EXPECT_EQ(map.worldToGrid(Eigen::Vector2d(-2.0, -1.0)), Eigen::Vector2i(0, 0));
  EXPECT_EQ(map.worldToGrid(Eigen::Vector2d(-2.01, -1.01)), Eigen::Vector2i(-1, -1));
  EXPECT_EQ(map.worldToGrid(Eigen::Vector2d(2.0, 1.0)), Eigen::Vector2i(8, 4));
  EXPECT_TRUE(
    map.gridToWorld(Eigen::Vector2i(0, 0)).isApprox(
      Eigen::Vector2d(-1.75, -0.75)));
}

TEST(GridMap2D, RollingWindowKeepsObstaclesAtTheirWorldPosition)
{
  GridMap2D map(1.0, Eigen::Vector2i(6, 6));
  map.setInflateRadius(0.0);
  map.setObstacle(Eigen::Vector2i(3, 3));
  const Eigen::Vector2d obstacle_world = map.gridToWorld(Eigen::Vector2i(3, 3));

  map.setCurPose(1.0, 0.0);

  EXPECT_TRUE(map.origin().isApprox(Eigen::Vector2d(-2.0, -3.0)));
  EXPECT_TRUE(map.isObstacle(obstacle_world));
  EXPECT_EQ(map.worldToGrid(obstacle_world), Eigen::Vector2i(2, 3));
  EXPECT_FALSE(map.isOccupied(Eigen::Vector2i(3, 3), false));
}

TEST(GridMap2D, InflationUsesCircularMetricRadius)
{
  GridMap2D map(1.0, Eigen::Vector2i(7, 7));
  map.setObstacle(Eigen::Vector2i(3, 3));
  map.inflateObstacles(1.0);

  EXPECT_TRUE(map.isOccupied(Eigen::Vector2i(3, 3)));
  EXPECT_TRUE(map.isOccupied(Eigen::Vector2i(4, 3)));
  EXPECT_TRUE(map.isOccupied(Eigen::Vector2i(2, 3)));
  EXPECT_TRUE(map.isOccupied(Eigen::Vector2i(3, 4)));
  EXPECT_TRUE(map.isOccupied(Eigen::Vector2i(3, 2)));
  EXPECT_FALSE(map.isOccupied(Eigen::Vector2i(4, 4)));
}

TEST(GridMap2D, InflateObstaclesPersistsRadiusAcrossMapUpdates)
{
  GridMap2D map(0.1, Eigen::Vector2i(2, 2));
  map.inflateObstacles(0.1);
  const Eigen::Vector2i obstacle(10, 10);
  map.setObstacle(obstacle);

  EXPECT_TRUE(map.isOccupied(obstacle + Eigen::Vector2i(1, 0), true));
  EXPECT_FALSE(map.isOccupied(obstacle + Eigen::Vector2i(3, 0), true));
}

TEST(GridMap2D, OccupancyDataUsesStandardRosEncoding)
{
  GridMap2D map(1.0, Eigen::Vector2i(3, 3));
  map.setObstacle(Eigen::Vector2i(0, 0), false);
  map.setObstacle(Eigen::Vector2i(1, 1));

  std::vector<int8_t> data;
  map.getOccupancyGridData(data, false);

  ASSERT_EQ(data.size(), 9U);
  for (std::size_t index = 0; index < data.size(); ++index) {
    const std::int8_t expected = index == 4U ? 100 : (index == 0U ? 0 : -1);
    EXPECT_EQ(data[index], expected);
  }
}

TEST(GridMap2D, SensorRayMarksFreeCellsButKeepsEndpointOccupied)
{
  GridMap2D map(1.0, Eigen::Vector2i(7, 3));
  const Eigen::Vector2d start = map.gridToWorld(Eigen::Vector2i(1, 1));
  const Eigen::Vector2d end = map.gridToWorld(Eigen::Vector2i(5, 1));
  map.markFreeRay(start, end);
  map.setObstacle(Eigen::Vector2i(5, 1));

  std::vector<std::int8_t> data;
  map.getOccupancyGridData(data, false);

  for (int x = 1; x < 5; ++x) {
    EXPECT_EQ(data[static_cast<std::size_t>(1 * 7 + x)], 0);
  }
  EXPECT_EQ(data[static_cast<std::size_t>(1 * 7 + 5)], 100);
  EXPECT_EQ(data[0], -1);
}
