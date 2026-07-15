// Copyright 2026 hengzhiyou
// SPDX-License-Identifier: GPL-3.0-only

#include "mapping/grid_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
constexpr double kDistanceEpsilon = 1e-12;

int floorToInt(double value) noexcept
{
  if (!std::isfinite(value)) {
    return std::numeric_limits<int>::min();
  }
  if (value <= static_cast<double>(std::numeric_limits<int>::min())) {
    return std::numeric_limits<int>::min();
  }
  if (value >= static_cast<double>(std::numeric_limits<int>::max())) {
    return std::numeric_limits<int>::max();
  }
  return static_cast<int>(std::floor(value));
}
}  // namespace

GridMap2D::GridMap2D(double resolution, const Eigen::Vector2i & map_size_m)
: resolution_(resolution), requested_size_m_(map_size_m.cast<double>())
{
  if (!std::isfinite(resolution_) || resolution_ <= 0.0) {
    throw std::invalid_argument("GridMap2D resolution must be finite and positive");
  }
  if (map_size_m.x() <= 0 || map_size_m.y() <= 0) {
    throw std::invalid_argument("GridMap2D metric map size must be positive");
  }

  const double width_cells = std::ceil(requested_size_m_.x() / resolution_);
  const double height_cells = std::ceil(requested_size_m_.y() / resolution_);
  if (!std::isfinite(width_cells) || !std::isfinite(height_cells) ||
    width_cells > static_cast<double>(std::numeric_limits<int>::max()) ||
    height_cells > static_cast<double>(std::numeric_limits<int>::max()))
  {
    throw std::length_error("GridMap2D dimensions exceed supported integer range");
  }

  map_size_ = Eigen::Vector2i(
    static_cast<int>(width_cells), static_cast<int>(height_cells));

  const auto width = static_cast<std::size_t>(map_size_.x());
  const auto height = static_cast<std::size_t>(map_size_.y());
  if (width > std::numeric_limits<std::size_t>::max() / height) {
    throw std::length_error("GridMap2D cell count overflows size_t");
  }

  const std::size_t cell_count = width * height;
  original_grid_.assign(cell_count, kFree);
  inflated_grid_.assign(cell_count, kFree);
  observed_grid_.assign(cell_count, kFree);

  // The initial rolling window is centred exactly at the world origin. Subsequent
  // moves are quantised to whole cells so stored obstacles remain world-fixed.
  const Eigen::Vector2d actual_extent = map_size_.cast<double>() * resolution_;
  origin_ = -0.5 * actual_extent;
}

void GridMap2D::setCurPose(double x, double y)
{
  if (!std::isfinite(x) || !std::isfinite(y)) {
    throw std::invalid_argument("GridMap2D rolling-window centre must be finite");
  }
  rollToOrigin(alignedOrigin(x, y));
}

void GridMap2D::resetMap()
{
  resetGrids();
}

void GridMap2D::resetGrids()
{
  std::fill(original_grid_.begin(), original_grid_.end(), kFree);
  std::fill(inflated_grid_.begin(), inflated_grid_.end(), kFree);
  std::fill(observed_grid_.begin(), observed_grid_.end(), kFree);
  inflation_valid_ = true;
}

void GridMap2D::markFreeRay(
  const Eigen::Vector2d & start, const Eigen::Vector2d & end)
{
  Eigen::Vector2i current = worldToGrid(start);
  const Eigen::Vector2i target = worldToGrid(end);
  if (!isIndexValid(current) || !isIndexValid(target)) {
    return;
  }

  const int delta_x = std::abs(target.x() - current.x());
  const int delta_y = std::abs(target.y() - current.y());
  const int step_x = current.x() < target.x() ? 1 : -1;
  const int step_y = current.y() < target.y() ? 1 : -1;
  int error = delta_x - delta_y;
  const std::size_t maximum_steps =
    static_cast<std::size_t>(map_size_.x()) + static_cast<std::size_t>(map_size_.y()) + 1U;

  for (std::size_t step = 0; current != target && step < maximum_steps; ++step) {
    observed_grid_[flatIndex(current)] = kOccupied;
    const int doubled_error = 2 * error;
    if (doubled_error > -delta_y) {
      error -= delta_y;
      current.x() += step_x;
    }
    if (doubled_error < delta_x) {
      error += delta_x;
      current.y() += step_y;
    }
  }
}

void GridMap2D::setObstacle(const Eigen::Vector2i & index, bool is_obstacle)
{
  if (!isIndexValid(index)) {
    return;
  }

  original_grid_[flatIndex(index)] = is_obstacle ? kOccupied : kFree;
  observed_grid_[flatIndex(index)] = kOccupied;
  inflation_valid_ = false;
}

void GridMap2D::setPrintfOpenOrNot(bool enabled)
{
  // Retained as a compatibility hook. Core mapping code intentionally does not
  // write to stdout/stderr; the ROS adapter owns logging policy.
  (void)enabled;
}

void GridMap2D::inflateObstacles(double radius)
{
  setInflateRadius(radius);
  inflate();
}

void GridMap2D::inflate()
{
  rebuildInflated(inflate_radius_);
}

void GridMap2D::setInflateRadius(double radius)
{
  if (!std::isfinite(radius) || radius < 0.0) {
    throw std::invalid_argument("GridMap2D inflation radius must be finite and non-negative");
  }
  if (radius != inflate_radius_) {
    inflate_radius_ = radius;
    inflation_valid_ = false;
  }
}

bool GridMap2D::isObstacle(const Eigen::Vector2d & pos) const
{
  return isOccupied(worldToGrid(pos), false);
}

bool GridMap2D::getInflateOccupancy(const Eigen::Vector2d & world_pos) const
{
  return isOccupied(worldToGrid(world_pos), true);
}

bool GridMap2D::isOccupied(const Eigen::Vector2i & index, bool inflated) const
{
  if (!isIndexValid(index)) {
    return true;
  }
  if (inflated) {
    ensureInflated();
    return inflated_grid_[flatIndex(index)] == kOccupied;
  }
  return original_grid_[flatIndex(index)] == kOccupied;
}

Eigen::Vector2i GridMap2D::worldToGrid(const Eigen::Vector2d & coord) const
{
  if (!coord.allFinite()) {
    return Eigen::Vector2i(-1, -1);
  }

  const Eigen::Array2d continuous = (coord - origin_).array() / resolution_;
  return Eigen::Vector2i(
    floorToInt(continuous.x()), floorToInt(continuous.y()));
}

Eigen::Vector2d GridMap2D::gridToWorld(const Eigen::Vector2i & index) const
{
  return origin_ + (index.cast<double>().array() + 0.5).matrix() * resolution_;
}

bool GridMap2D::isIndexValid(const Eigen::Vector2i & index) const
{
  return index.x() >= 0 && index.x() < map_size_.x() &&
         index.y() >= 0 && index.y() < map_size_.y();
}

std::vector<Eigen::Vector2d> GridMap2D::getObstaclePointCloud(
  bool return_inflated_map) const
{
  if (return_inflated_map) {
    ensureInflated();
  }
  const std::vector<Cell> & source =
    return_inflated_map ? inflated_grid_ : original_grid_;

  std::vector<Eigen::Vector2d> points;
  points.reserve(
    static_cast<std::size_t>(std::count(
      source.begin(), source.end(), kOccupied)));
  for (int y = 0; y < map_size_.y(); ++y) {
    for (int x = 0; x < map_size_.x(); ++x) {
      const Eigen::Vector2i index(x, y);
      if (source[flatIndex(index)] == kOccupied) {
        points.push_back(gridToWorld(index));
      }
    }
  }
  return points;
}

void GridMap2D::getOccupancyGridData(
  std::vector<int8_t> & data, bool return_inflated_map) const
{
  if (return_inflated_map) {
    ensureInflated();
  }
  const std::vector<Cell> & source =
    return_inflated_map ? inflated_grid_ : original_grid_;

  data.resize(source.size());
  for (std::size_t index = 0; index < source.size(); ++index) {
    if (source[index] == kOccupied) {
      data[index] = 100;
    } else {
      data[index] = observed_grid_[index] == kOccupied ? 0 : -1;
    }
  }
}

double GridMap2D::resolution() const noexcept
{
  return resolution_;
}

double GridMap2D::getResolution() const noexcept
{
  return resolution_;
}

const Eigen::Vector2i & GridMap2D::mapSize() const noexcept
{
  return map_size_;
}

const Eigen::Vector2d & GridMap2D::origin() const noexcept
{
  return origin_;
}

std::size_t GridMap2D::flatIndex(const Eigen::Vector2i & index) const noexcept
{
  return static_cast<std::size_t>(index.y()) * static_cast<std::size_t>(map_size_.x()) +
         static_cast<std::size_t>(index.x());
}

Eigen::Vector2d GridMap2D::alignedOrigin(double center_x, double center_y) const
{
  const Eigen::Vector2d actual_extent = map_size_.cast<double>() * resolution_;
  const Eigen::Vector2d desired =
    Eigen::Vector2d(center_x, center_y) - 0.5 * actual_extent;

  Eigen::Vector2d result;
  for (int axis = 0; axis < 2; ++axis) {
    const double shift = (desired[axis] - origin_[axis]) / resolution_;
    if (!std::isfinite(shift) ||
      shift < static_cast<double>(std::numeric_limits<std::int64_t>::min()) ||
      shift > static_cast<double>(std::numeric_limits<std::int64_t>::max()))
    {
      throw std::overflow_error("GridMap2D rolling-window shift exceeds supported range");
    }
    const std::int64_t cell_shift = std::llround(shift);
    result[axis] = origin_[axis] + static_cast<double>(cell_shift) * resolution_;
  }
  return result;
}

void GridMap2D::rollToOrigin(const Eigen::Vector2d & new_origin)
{
  const Eigen::Array2d shift_cells_real = (new_origin - origin_).array() / resolution_;
  const std::int64_t shift_x = std::llround(shift_cells_real.x());
  const std::int64_t shift_y = std::llround(shift_cells_real.y());
  if (shift_x == 0LL && shift_y == 0LL) {
    return;
  }

  std::vector<Cell> rolled(original_grid_.size(), kFree);
  std::vector<Cell> rolled_observed(observed_grid_.size(), kFree);
  if (shift_x > -static_cast<std::int64_t>(map_size_.x()) &&
    shift_x < static_cast<std::int64_t>(map_size_.x()) &&
    shift_y > -static_cast<std::int64_t>(map_size_.y()) &&
    shift_y < static_cast<std::int64_t>(map_size_.y()))
  {
    const int shift_x_cells = static_cast<int>(shift_x);
    const int shift_y_cells = static_cast<int>(shift_y);
    for (int old_y = 0; old_y < map_size_.y(); ++old_y) {
      const int new_y = old_y - shift_y_cells;
      if (new_y < 0 || new_y >= map_size_.y()) {
        continue;
      }
      for (int old_x = 0; old_x < map_size_.x(); ++old_x) {
        const int new_x = old_x - shift_x_cells;
        if (new_x < 0 || new_x >= map_size_.x()) {
          continue;
        }
        const Eigen::Vector2i old_index(old_x, old_y);
        const std::size_t old_flat = flatIndex(old_index);
        const std::size_t new_flat =
          static_cast<std::size_t>(new_y) * static_cast<std::size_t>(map_size_.x()) +
          static_cast<std::size_t>(new_x);
        if (original_grid_[old_flat] == kOccupied) {
          rolled[new_flat] = kOccupied;
        }
        if (observed_grid_[old_flat] == kOccupied) {
          rolled_observed[new_flat] = kOccupied;
        }
      }
    }
  }

  original_grid_.swap(rolled);
  observed_grid_.swap(rolled_observed);
  origin_ = new_origin;
  std::fill(inflated_grid_.begin(), inflated_grid_.end(), kFree);
  inflation_valid_ = false;
}

void GridMap2D::rebuildInflated(double radius) const
{
  inflated_grid_ = original_grid_;
  if (radius <= kDistanceEpsilon) {
    inflation_valid_ = true;
    return;
  }

  const double requested_cell_radius = std::ceil(radius / resolution_);
  const int largest_useful_offset = std::max(map_size_.x() - 1, map_size_.y() - 1);
  const int cell_radius = requested_cell_radius >= static_cast<double>(largest_useful_offset) ?
    largest_useful_offset : static_cast<int>(requested_cell_radius);
  const int x_radius = std::min(cell_radius, map_size_.x() - 1);
  const int y_radius = std::min(cell_radius, map_size_.y() - 1);
  const double radius_squared = radius * radius;
  std::vector<Eigen::Vector2i> offsets;
  const std::size_t offset_width = 2U * static_cast<std::size_t>(x_radius) + 1U;
  const std::size_t offset_height = 2U * static_cast<std::size_t>(y_radius) + 1U;
  if (offset_width <= std::numeric_limits<std::size_t>::max() / offset_height) {
    offsets.reserve(offset_width * offset_height);
  }
  for (int dy = -y_radius; dy <= y_radius; ++dy) {
    for (int dx = -x_radius; dx <= x_radius; ++dx) {
      const double dx_m = static_cast<double>(dx) * resolution_;
      const double dy_m = static_cast<double>(dy) * resolution_;
      const double distance_squared =
        dx_m * dx_m + dy_m * dy_m;
      if (distance_squared <= radius_squared + kDistanceEpsilon) {
        offsets.emplace_back(dx, dy);
      }
    }
  }

  for (int y = 0; y < map_size_.y(); ++y) {
    for (int x = 0; x < map_size_.x(); ++x) {
      const Eigen::Vector2i obstacle(x, y);
      if (original_grid_[flatIndex(obstacle)] != kOccupied) {
        continue;
      }
      for (const Eigen::Vector2i & offset : offsets) {
        const Eigen::Vector2i inflated = obstacle + offset;
        if (isIndexValid(inflated)) {
          inflated_grid_[flatIndex(inflated)] = kOccupied;
        }
      }
    }
  }
  inflation_valid_ = true;
}

void GridMap2D::ensureInflated() const
{
  if (!inflation_valid_) {
    rebuildInflated(inflate_radius_);
  }
}
