// Copyright 2026 hengzhiyou
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <cstdint>
#include <limits>
#include <memory>
#include <queue>
#include <vector>

#include <Eigen/Core>

#include "mapping/grid_map.hpp"

struct GridNode;
using GridNodePtr = GridNode *;

struct GridNode
{
  enum enum_state
  {
    OPENSET = 1,
    CLOSEDSET = 2,
    UNDEFINED = 3
  };

  int rounds{0};
  enum_state state{UNDEFINED};
  Eigen::Vector2i index{Eigen::Vector2i::Zero()};
  double gScore{std::numeric_limits<double>::infinity()};
  double fScore{std::numeric_limits<double>::infinity()};
  GridNodePtr cameFrom{nullptr};
};

class AStar
{
public:
  using Ptr = std::shared_ptr<AStar>;

  AStar() = default;
  ~AStar() = default;
  AStar(const AStar &) = delete;
  AStar & operator=(const AStar &) = delete;

  void initGridMap(
    std::shared_ptr<GridMap2D> occ_map, const Eigen::Vector2i pool_size);

  bool AstarSearch(
    double step_size, Eigen::Vector2d start_pt, Eigen::Vector2d end_pt);

  std::vector<Eigen::Vector2d> getPath();
  std::vector<Eigen::Vector3d> get3DPath();

private:
  struct OpenEntry
  {
    double f_score{std::numeric_limits<double>::infinity()};
    double g_score{std::numeric_limits<double>::infinity()};
    std::uint64_t sequence{0U};
    GridNodePtr node{nullptr};
  };

  struct OpenEntryComparator
  {
    bool operator()(const OpenEntry & lhs, const OpenEntry & rhs) const noexcept;
  };

  double getDiagHeu(const GridNode * node1, const GridNode * node2) const noexcept;
  bool ConvertToIndexAndAdjustStartEndPoints(
    Eigen::Vector2d start_pt, Eigen::Vector2d end_pt,
    Eigen::Vector2i & start_idx, Eigen::Vector2i & end_idx) const;

  double getHeu(const GridNode * node1, const GridNode * node2) const noexcept;
  Eigen::Vector2d Index2Coord(const Eigen::Vector2i & index) const noexcept;
  bool Coord2Index(const Eigen::Vector2d & pt, Eigen::Vector2i & idx) const noexcept;
  bool checkOccupancy(const Eigen::Vector2d & pos) const;
  bool isPoolIndexValid(const Eigen::Vector2i & index) const noexcept;
  bool isDiagonalMoveValid(
    const Eigen::Vector2i & current, int dx, int dy) const;

  GridNodePtr nodeAt(const Eigen::Vector2i & index) noexcept;
  const GridNode * nodeAt(const Eigen::Vector2i & index) const noexcept;
  std::vector<GridNodePtr> retrievePath(GridNodePtr current) const;
  void resetSearchState();

  std::shared_ptr<GridMap2D> grid_map_;
  std::vector<GridNode> node_pool_;
  std::vector<GridNodePtr> grid_path_;
  std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenEntryComparator> open_set_;

  double step_size_{0.0};
  double inv_step_size_{0.0};
  Eigen::Vector2d center_{Eigen::Vector2d::Zero()};
  Eigen::Vector2i center_index_{Eigen::Vector2i::Zero()};
  Eigen::Vector2i pool_size_{Eigen::Vector2i::Zero()};
  std::uint64_t push_sequence_{0U};
  int rounds_{0};
  bool initialized_{false};
};
