// Copyright 2026 hengzhiyou
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Core>

/**
 * @brief A rolling, two-dimensional occupancy grid.
 *
 * The constructor keeps the historical interface used by this package: map_size_m
 * is the requested map extent in metres, not a number of cells. mapSize() returns
 * the resulting number of cells as (width, height).
 *
 * origin() follows nav_msgs/OccupancyGrid semantics and denotes the lower-left
 * corner of cell (0, 0). gridToWorld() returns cell centres.
 */
class GridMap2D
{
public:
  GridMap2D(double resolution, const Eigen::Vector2i & map_size_m);

  /** Centre the rolling window near (x, y), preserving obstacles at world-fixed cells. */
  void setCurPose(double x, double y);

  void resetMap();
  void resetGrids();

  /** Mark a sensor ray as observed free space, excluding its endpoint. */
  void markFreeRay(const Eigen::Vector2d & start, const Eigen::Vector2d & end);
  void setObstacle(const Eigen::Vector2i & index, bool is_obstacle = true);
  void setPrintfOpenOrNot(bool enabled);

  /** Rebuild the inflated map using a circular radius expressed in metres. */
  void inflateObstacles(double radius);
  void inflate();
  void setInflateRadius(double radius);

  bool isObstacle(const Eigen::Vector2d & pos) const;
  bool getInflateOccupancy(const Eigen::Vector2d & world_pos) const;

  /** Query by grid index. Out-of-bounds cells are treated as occupied. */
  bool isOccupied(const Eigen::Vector2i & index, bool inflated = true) const;

  Eigen::Vector2i worldToGrid(const Eigen::Vector2d & coord) const;
  Eigen::Vector2d gridToWorld(const Eigen::Vector2i & index) const;
  bool isIndexValid(const Eigen::Vector2i & index) const;

  std::vector<Eigen::Vector2d> getObstaclePointCloud(
    bool return_inflated_map = true) const;

  /** ROS OccupancyGrid encoding: unknown=-1, free=0, occupied=100. */
  void getOccupancyGridData(
    std::vector<int8_t> & data, bool return_inflated_map = true) const;

  double resolution() const noexcept;
  double getResolution() const noexcept;
  const Eigen::Vector2i & mapSize() const noexcept;
  const Eigen::Vector2d & origin() const noexcept;

private:
  using Cell = std::uint8_t;
  static constexpr Cell kFree = 0U;
  static constexpr Cell kOccupied = 1U;

  std::size_t flatIndex(const Eigen::Vector2i & index) const noexcept;
  Eigen::Vector2d alignedOrigin(double center_x, double center_y) const;
  void rollToOrigin(const Eigen::Vector2d & new_origin);
  void rebuildInflated(double radius) const;
  void ensureInflated() const;

  std::vector<Cell> original_grid_;
  mutable std::vector<Cell> inflated_grid_;
  std::vector<Cell> observed_grid_;
  double resolution_{1.0};
  Eigen::Vector2i map_size_{Eigen::Vector2i::Zero()};
  Eigen::Vector2d origin_{Eigen::Vector2d::Zero()};
  double inflate_radius_{0.5};
  Eigen::Vector2d requested_size_m_{Eigen::Vector2d::Zero()};
  mutable bool inflation_valid_{true};
};

using GridMap2DPtr = std::shared_ptr<GridMap2D>;
