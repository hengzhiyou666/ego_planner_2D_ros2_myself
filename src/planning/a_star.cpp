// Copyright 2026 hengzhiyou
// SPDX-License-Identifier: GPL-3.0-only

#include "planning/a_star.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

namespace
{
constexpr double kScoreEpsilon = 1e-12;
}  // namespace

bool AStar::OpenEntryComparator::operator()(
  const OpenEntry & lhs, const OpenEntry & rhs) const noexcept
{
  if (lhs.f_score != rhs.f_score) {
    return lhs.f_score > rhs.f_score;
  }
  if (lhs.g_score != rhs.g_score) {
    // With equal f, prefer the node farther from the start.
    return lhs.g_score < rhs.g_score;
  }
  return lhs.sequence > rhs.sequence;
}

void AStar::initGridMap(
  std::shared_ptr<GridMap2D> occ_map, const Eigen::Vector2i pool_size)
{
  initialized_ = false;
  grid_map_.reset();
  node_pool_.clear();
  grid_path_.clear();
  open_set_ = decltype(open_set_) {};
  pool_size_.setZero();
  center_index_.setZero();

  if (!occ_map || pool_size.x() <= 0 || pool_size.y() <= 0) {
    return;
  }

  const std::size_t width = static_cast<std::size_t>(pool_size.x());
  const std::size_t height = static_cast<std::size_t>(pool_size.y());
  if (width > std::numeric_limits<std::size_t>::max() / height) {
    return;
  }

  pool_size_ = pool_size;
  center_index_ = pool_size_ / 2;
  grid_map_ = std::move(occ_map);
  node_pool_.resize(width * height);
  resetSearchState();
  initialized_ = true;
}

bool AStar::AstarSearch(
  double step_size, Eigen::Vector2d start_pt, Eigen::Vector2d end_pt)
{
  grid_path_.clear();
  open_set_ = decltype(open_set_) {};
  push_sequence_ = 0U;

  if (!initialized_ || !grid_map_ || node_pool_.empty() ||
    !std::isfinite(step_size) || step_size <= 0.0 ||
    !std::isfinite(grid_map_->resolution()) || grid_map_->resolution() <= 0.0 ||
    !start_pt.allFinite() || !end_pt.allFinite())
  {
    return false;
  }

  // Never search with a stride larger than one occupancy cell; otherwise a
  // neighbour edge could jump over a thin occupied cell.
  step_size_ = std::min(step_size, grid_map_->resolution());
  inv_step_size_ = 1.0 / step_size_;
  if (!std::isfinite(inv_step_size_)) {
    return false;
  }
  center_ = 0.5 * start_pt + 0.5 * end_pt;
  if (!center_.allFinite()) {
    return false;
  }
  if (rounds_ == std::numeric_limits<int>::max()) {
    rounds_ = 1;
  } else {
    ++rounds_;
  }
  resetSearchState();

  Eigen::Vector2i start_index;
  Eigen::Vector2i end_index;
  if (!ConvertToIndexAndAdjustStartEndPoints(
      start_pt, end_pt, start_index, end_index))
  {
    return false;
  }

  GridNodePtr start = nodeAt(start_index);
  GridNodePtr goal = nodeAt(end_index);
  if (start == nullptr || goal == nullptr) {
    return false;
  }

  start->gScore = 0.0;
  start->fScore = getHeu(start, goal);
  start->state = GridNode::OPENSET;
  start->cameFrom = nullptr;
  open_set_.push(OpenEntry{start->fScore, start->gScore, push_sequence_++, start});

  while (!open_set_.empty()) {
    const OpenEntry entry = open_set_.top();
    open_set_.pop();
    GridNodePtr current = entry.node;

    if (current == nullptr || current->state == GridNode::CLOSEDSET ||
      entry.g_score > current->gScore + kScoreEpsilon)
    {
      continue;
    }

    if (current->index == end_index) {
      grid_path_ = retrievePath(current);
      return !grid_path_.empty();
    }

    current->state = GridNode::CLOSEDSET;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) {
          continue;
        }

        const Eigen::Vector2i neighbour_index =
          current->index + Eigen::Vector2i(dx, dy);
        if (!isPoolIndexValid(neighbour_index) ||
          checkOccupancy(Index2Coord(neighbour_index)) ||
          !isDiagonalMoveValid(current->index, dx, dy))
        {
          continue;
        }

        GridNodePtr neighbour = nodeAt(neighbour_index);
        if (neighbour == nullptr || neighbour->state == GridNode::CLOSEDSET) {
          continue;
        }

        const double move_cost = (dx != 0 && dy != 0) ? std::sqrt(2.0) : 1.0;
        const double tentative_g = current->gScore + move_cost;
        if (tentative_g + kScoreEpsilon >= neighbour->gScore) {
          continue;
        }

        neighbour->cameFrom = current;
        neighbour->gScore = tentative_g;
        neighbour->fScore = tentative_g + getHeu(neighbour, goal);
        neighbour->state = GridNode::OPENSET;

        // Each improved score gets a new immutable queue entry. Entries carrying
        // an older g-score are discarded when popped, so heap ordering remains valid.
        open_set_.push(
          OpenEntry{
          neighbour->fScore, neighbour->gScore, push_sequence_++, neighbour});
      }
    }
  }

  grid_path_.clear();
  return false;
}

std::vector<Eigen::Vector2d> AStar::getPath()
{
  std::vector<Eigen::Vector2d> path;
  path.reserve(grid_path_.size());
  for (auto iterator = grid_path_.rbegin(); iterator != grid_path_.rend(); ++iterator) {
    if (*iterator != nullptr) {
      path.push_back(Index2Coord((*iterator)->index));
    }
  }
  return path;
}

std::vector<Eigen::Vector3d> AStar::get3DPath()
{
  std::vector<Eigen::Vector3d> path;
  path.reserve(grid_path_.size());
  for (auto iterator = grid_path_.rbegin(); iterator != grid_path_.rend(); ++iterator) {
    if (*iterator == nullptr) {
      continue;
    }
    const Eigen::Vector2d position = Index2Coord((*iterator)->index);
    path.emplace_back(position.x(), position.y(), 0.0);
  }
  return path;
}

double AStar::getDiagHeu(
  const GridNode * node1, const GridNode * node2) const noexcept
{
  const int dx = std::abs(node1->index.x() - node2->index.x());
  const int dy = std::abs(node1->index.y() - node2->index.y());
  const int diagonal = std::min(dx, dy);
  const int straight = std::max(dx, dy) - diagonal;
  return static_cast<double>(diagonal) * std::sqrt(2.0) +
         static_cast<double>(straight);
}

bool AStar::ConvertToIndexAndAdjustStartEndPoints(
  Eigen::Vector2d start_pt, Eigen::Vector2d end_pt,
  Eigen::Vector2i & start_idx, Eigen::Vector2i & end_idx) const
{
  const auto find_first_free = [this](
    const Eigen::Vector2d & from, const Eigen::Vector2d & toward,
    Eigen::Vector2i & result) -> bool
    {
      Eigen::Vector2d candidate = from;
      if (!Coord2Index(candidate, result)) {
        return false;
      }
      if (!checkOccupancy(Index2Coord(result))) {
        return true;
      }

      const Eigen::Vector2d delta = toward - from;
      if (delta.norm() <= kScoreEpsilon) {
        return false;
      }
      const Eigen::Vector2d increment = delta.normalized() * step_size_;
      const int max_steps = 2 * (pool_size_.x() + pool_size_.y());
      Eigen::Vector2i previous = result;
      for (int step = 0; step < max_steps; ++step) {
        candidate += increment;
        if (!Coord2Index(candidate, result)) {
          return false;
        }
        if (result == previous) {
          continue;
        }
        previous = result;
        if (!checkOccupancy(Index2Coord(result))) {
          return true;
        }
      }
      return false;
    };

  if (!find_first_free(start_pt, end_pt, start_idx)) {
    return false;
  }
  return find_first_free(end_pt, Index2Coord(start_idx), end_idx);
}

double AStar::getHeu(
  const GridNode * node1, const GridNode * node2) const noexcept
{
  return getDiagHeu(node1, node2);
}

Eigen::Vector2d AStar::Index2Coord(const Eigen::Vector2i & index) const noexcept
{
  return center_ + (index - center_index_).cast<double>() * step_size_;
}

bool AStar::Coord2Index(
  const Eigen::Vector2d & pt, Eigen::Vector2i & idx) const noexcept
{
  if (!pt.allFinite() || inv_step_size_ <= 0.0) {
    return false;
  }

  const Eigen::Vector2d relative = (pt - center_) * inv_step_size_;
  const double min_x = static_cast<double>(-center_index_.x()) - 0.5;
  const double max_x =
    static_cast<double>(pool_size_.x() - 1 - center_index_.x()) + 0.5;
  const double min_y = static_cast<double>(-center_index_.y()) - 0.5;
  const double max_y =
    static_cast<double>(pool_size_.y() - 1 - center_index_.y()) + 0.5;
  if (relative.x() < min_x || relative.x() > max_x ||
    relative.y() < min_y || relative.y() > max_y)
  {
    return false;
  }

  idx = center_index_ + Eigen::Vector2i(
    static_cast<int>(std::lround(relative.x())),
    static_cast<int>(std::lround(relative.y())));
  return isPoolIndexValid(idx);
}

bool AStar::checkOccupancy(const Eigen::Vector2d & pos) const
{
  return !grid_map_ || grid_map_->getInflateOccupancy(pos);
}

bool AStar::isPoolIndexValid(const Eigen::Vector2i & index) const noexcept
{
  return index.x() >= 0 && index.x() < pool_size_.x() &&
         index.y() >= 0 && index.y() < pool_size_.y();
}

bool AStar::isDiagonalMoveValid(
  const Eigen::Vector2i & current, int dx, int dy) const
{
  if (dx == 0 || dy == 0) {
    return true;
  }

  const Eigen::Vector2i horizontal = current + Eigen::Vector2i(dx, 0);
  const Eigen::Vector2i vertical = current + Eigen::Vector2i(0, dy);
  return isPoolIndexValid(horizontal) && isPoolIndexValid(vertical) &&
         !checkOccupancy(Index2Coord(horizontal)) &&
         !checkOccupancy(Index2Coord(vertical));
}

GridNodePtr AStar::nodeAt(const Eigen::Vector2i & index) noexcept
{
  if (!isPoolIndexValid(index)) {
    return nullptr;
  }
  const std::size_t flat =
    static_cast<std::size_t>(index.y()) * static_cast<std::size_t>(pool_size_.x()) +
    static_cast<std::size_t>(index.x());
  return &node_pool_[flat];
}

const GridNode * AStar::nodeAt(const Eigen::Vector2i & index) const noexcept
{
  if (!isPoolIndexValid(index)) {
    return nullptr;
  }
  const std::size_t flat =
    static_cast<std::size_t>(index.y()) * static_cast<std::size_t>(pool_size_.x()) +
    static_cast<std::size_t>(index.x());
  return &node_pool_[flat];
}

std::vector<GridNodePtr> AStar::retrievePath(GridNodePtr current) const
{
  std::vector<GridNodePtr> path;
  path.reserve(node_pool_.size());
  while (current != nullptr && path.size() <= node_pool_.size()) {
    path.push_back(current);
    current = current->cameFrom;
  }
  if (current != nullptr) {
    path.clear();
  }
  return path;
}

void AStar::resetSearchState()
{
  for (int y = 0; y < pool_size_.y(); ++y) {
    for (int x = 0; x < pool_size_.x(); ++x) {
      GridNode & node = node_pool_[
        static_cast<std::size_t>(y) * static_cast<std::size_t>(pool_size_.x()) +
        static_cast<std::size_t>(x)];
      node = GridNode{};
      node.index = Eigen::Vector2i(x, y);
      node.rounds = rounds_;
    }
  }
}
