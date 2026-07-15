// Copyright (c) ZJU FAST Lab contributors
// Copyright 2026 hengzhiyou
// SPDX-License-Identifier: GPL-3.0-only

#include "planning/planner_interface_dog.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "planning/a_star.hpp"

namespace dog_ego_planner
{

PlannerInterfaceDog::PlannerInterfaceDog() = default;
PlannerInterfaceDog::~PlannerInterfaceDog() = default;

void PlannerInterfaceDog::initParam(
  double max_vel, double max_acc, double max_jerk,
  double control_point_interval)
{
  pp_.max_vel_ = std::max(max_vel, 1e-6);   // 防呆：避免除零
  pp_.max_acc_ = std::max(max_acc, 1e-6);
  pp_.max_jerk_ = std::max(max_jerk, 1e-6);
  pp_.feasibility_tolerance_ = 0.05;

  // ctrl_pt_dist：相邻 B 样条控制点期望弧长间距，影响初始 knot 间隔 ts（见 reboundReplan）
  pp_.ctrl_pt_dist = std::max(control_point_interval, 1e-3);
}

void PlannerInterfaceDog::initGridMap(
  double x_size, double y_size, double resolution,
  const Eigen::Vector2d & origin, double inflate_radius,
  int astar_pool_size)
{
  (void)origin;  // GridMap2D in demo internally defines its origin behavior; keep signature for future.
  if (!std::isfinite(x_size) || !std::isfinite(y_size) || x_size <= 0.0 || y_size <= 0.0 ||
    x_size > static_cast<double>(std::numeric_limits<int>::max()) ||
    y_size > static_cast<double>(std::numeric_limits<int>::max()))
  {
    throw std::invalid_argument("planner map dimensions must be finite positive integers");
  }
  const Eigen::Vector2i map_size(
    static_cast<int>(std::ceil(x_size)), static_cast<int>(std::ceil(y_size)));
  grid_map_ = std::make_shared<GridMap2D>(resolution, map_size);
  grid_map_->setPrintfOpenOrNot(false);
  grid_map_->setInflateRadius(inflate_radius);

  optimizer_ = std::make_unique<ego_planner::BsplineOptimizer>();
  optimizer_->setParam(pp_.max_vel_, pp_.max_acc_, inflate_radius);
  optimizer_->setPrintfOpenOrNot(false);
  optimizer_->setMaxReboundRetries(max_rebound_retries_);
  optimizer_->setEnvironment(grid_map_);
  optimizer_->a_star_ = std::make_shared<AStar>();
  const int pool = std::max(20, std::min(astar_pool_size, 200));  // 防呆：限制范围
  optimizer_->a_star_->initGridMap(grid_map_, Eigen::Vector2i(pool, pool));
}

void PlannerInterfaceDog::setPrintfOpenOrNot(bool enabled)
{
  (void)enabled;
  if (grid_map_) {
    grid_map_->setPrintfOpenOrNot(false);
  }
  if (optimizer_) {
    optimizer_->setPrintfOpenOrNot(false);
  }
}

void PlannerInterfaceDog::setMaxReboundRetries(int max_retries)
{
  max_rebound_retries_ = (max_retries < 0) ? 0 : max_retries;
  if (optimizer_) {
    optimizer_->setMaxReboundRetries(max_rebound_retries_);
  }
}

void PlannerInterfaceDog::setCurrentPose(const PathPoint2D & cur_pose)
{
  cur_pose_ = cur_pose;
  if (grid_map_) {
    grid_map_->setCurPose(cur_pose_.x, cur_pose_.y);
  }
}

void PlannerInterfaceDog::setReferencePath(const std::vector<PathPoint2D> & path_pts)
{
  ref_path_ = path_pts;
}

void PlannerInterfaceDog::setObstacles(const std::vector<Obstacle2D> & obstacles)
{
  if (!grid_map_) {return;}

  grid_map_->resetMap();
  const Eigen::Vector2d current(cur_pose_.x, cur_pose_.y);
  grid_map_->setObstacle(grid_map_->worldToGrid(current), false);

  for (std::size_t index = 0; index < obstacles.size(); ++index) {
    const auto & obstacle = obstacles[index];
    const Eigen::Vector2d coordinate(obstacle.x, obstacle.y);
    if (!coordinate.allFinite()) {
      continue;
    }
    grid_map_->markFreeRay(current, coordinate);
    grid_map_->setObstacle(grid_map_->worldToGrid(coordinate), true);
  }

  grid_map_->inflate();
}

bool PlannerInterfaceDog::getInflatedOccupancyGrid(
  std::vector<int8_t> & data, int & width, int & height,
  double & resolution, Eigen::Vector2d & origin) const
{
  if (!grid_map_) {return false;}
  const Eigen::Vector2i size = grid_map_->mapSize();
  width = size.x();
  height = size.y();
  resolution = grid_map_->resolution();
  origin = grid_map_->origin();
  grid_map_->getOccupancyGridData(data, true);
  return width > 0 && height > 0 && data.size() == static_cast<size_t>(width * height);
}

bool PlannerInterfaceDog::makePlan(
  const Eigen::Vector2d & start_vel_2d,
  const Eigen::Vector2d & start_acc_2d,
  const Eigen::Vector2d & local_target_vel_2d)
{
  if (!grid_map_ || !optimizer_) {
    return false;
  }
  if (ref_path_.size() < 4) {
    return false;
  }

  std::vector<Eigen::Vector3d> point_set;
  point_set.reserve(ref_path_.size());
  for (const auto & p : ref_path_) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      return false;
    }
    point_set.emplace_back(p.x, p.y, 0.0);
  }

  Eigen::Vector3d start_pt(ref_path_.front().x, ref_path_.front().y, 0.0);
  Eigen::Vector3d local_target_pt(ref_path_.back().x, ref_path_.back().y, 0.0);

  Eigen::Vector3d start_vel(start_vel_2d.x(), start_vel_2d.y(), 0.0);
  Eigen::Vector3d start_acc(start_acc_2d.x(), start_acc_2d.y(), 0.0);
  Eigen::Vector3d local_target_vel(local_target_vel_2d.x(), local_target_vel_2d.y(), 0.0);

  return reboundReplan(
    start_pt, start_vel, start_acc, local_target_pt, local_target_vel,
    point_set);
}

bool PlannerInterfaceDog::reboundReplan(
  const Eigen::Vector3d & start_pt, const Eigen::Vector3d & start_vel,
  const Eigen::Vector3d & start_acc, const Eigen::Vector3d & local_target_pt,
  const Eigen::Vector3d & local_target_vel,
  const std::vector<Eigen::Vector3d> & point_set)
{
  if (point_set.size() < 4) {
    return false;
  }

  std::vector<Eigen::Vector3d> start_end_derivatives;
  start_end_derivatives.push_back(start_vel);
  start_end_derivatives.push_back(local_target_vel);
  start_end_derivatives.push_back(start_acc);
  start_end_derivatives.push_back(Eigen::Vector3d::Zero());

  const double dist = (start_pt - local_target_pt).norm();
  const double max_vel_safe = std::max(pp_.max_vel_, 1e-6);  // 防呆：避免除零
  double ts = dist >
    0.1 ? pp_.ctrl_pt_dist / max_vel_safe * 1.2 : pp_.ctrl_pt_dist / max_vel_safe * 5.0;

  Eigen::MatrixXd ctrl_pts_3d;
  ego_planner::UniformBspline::parameterizeToBspline(
    ts, point_set, start_end_derivatives,
    ctrl_pts_3d);
  if (ctrl_pts_3d.rows() < 3 ||
    ctrl_pts_3d.cols() <= 2 * optimizer_->getOrder())
  {
    return false;
  }
  if (!ctrl_pts_3d.allFinite()) {
    return false;
  }
  Eigen::MatrixXd ctrl_pts_2d = ctrl_pts_3d.topRows(2);

  optimizer_->initControlPoints(ctrl_pts_2d, true);

  const bool step1_ok = optimizer_->BsplineOptimizeTrajRebound(ctrl_pts_2d, ts);
  if (!step1_ok) {
    return false;
  }

  last_ctrl_pts_2d_ = ctrl_pts_2d;

  ctrl_pts_3d = optimizer_->convert2DTo3D(ctrl_pts_2d);
  ego_planner::UniformBspline pos(ctrl_pts_3d, 3, ts);
  pos.setPhysicalLimits(
    pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_, pp_.max_jerk_);

  double ratio = 1.0;
  bool step2_ok = true;
  if (!pos.checkFeasibility(ratio, false)) {
    Eigen::MatrixXd optimal_control_points;
    step2_ok = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
    if (step2_ok) {
      pos = ego_planner::UniformBspline(optimal_control_points, 3, ts);
      pos.setPhysicalLimits(
        pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_, pp_.max_jerk_);
    }
  }

  if (!step2_ok) {
    return false;
  }

  double final_ratio = 1.0;
  if (!pos.checkFeasibility(final_ratio, false)) {
    return false;
  }
  if (!trajectoryIsCollisionFree(pos)) {
    return false;
  }

  last_ctrl_pts_2d_ = pos.getControlPoint().topRows(2);
  updateTrajInfo(pos);
  return true;
}

bool PlannerInterfaceDog::refineTrajAlgo(
  ego_planner::UniformBspline & traj,
  std::vector<Eigen::Vector3d> & start_end_derivative, double ratio, double & ts,
  Eigen::MatrixXd & optimal_control_points)
{
  Eigen::MatrixXd ctrl_pts;
  double t_inc = 0.0;
  reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);
  if (ctrl_pts.rows() < 3 || ctrl_pts.cols() < 4) {
    return false;
  }
  if (!ctrl_pts.allFinite()) {
    return false;
  }

  Eigen::MatrixXd ctrl_pts_2d = ctrl_pts.topRows(2);
  traj = ego_planner::UniformBspline(ctrl_pts, 3, ts);

  const int denom = static_cast<int>(ctrl_pts.cols()) - 3;
  if (denom <= 0) {
    return false;
  }
  const double time_sum = traj.getTimeSum();
  const double t_step =
    (denom > 0 && time_sum > 1e-12) ?
    (time_sum / static_cast<double>(denom)) : 0.01;  // 防呆：避免除零
  const double step_safe = std::max(t_step, 1e-9);  // 防呆：保证步长 > 0
  optimizer_->ref_pts_.clear();
  constexpr int kMaxRefPtsIter = 100000;
  int ref_iter = 0;
  for (double t = 0.0; t < time_sum + 1e-4 && ref_iter < kMaxRefPtsIter;
    t += step_safe, ++ref_iter)
  {
    optimizer_->ref_pts_.push_back(traj.evaluateDeBoorT(t).topRows(2));
  }

  Eigen::MatrixXd optimal_control_point_temp;
  const bool success = optimizer_->BsplineOptimizeTrajRefine(
    ctrl_pts_2d, ts, optimal_control_point_temp);
  if (!success || optimal_control_point_temp.rows() != 2 ||
    optimal_control_point_temp.cols() < 4 || !optimal_control_point_temp.allFinite())
  {
    optimal_control_points.resize(0, 0);
    return false;
  }
  optimal_control_points = optimizer_->convert2DTo3D(optimal_control_point_temp);
  return success;
}

void PlannerInterfaceDog::updateTrajInfo(const ego_planner::UniformBspline & position_traj)
{
  local_data_.position_traj_ = position_traj;
  local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
  local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
  local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
  local_data_.duration_ = local_data_.position_traj_.getTimeSum();
  local_data_.traj_id_ += 1;
}

void PlannerInterfaceDog::reparamBspline(
  ego_planner::UniformBspline & bspline,
  std::vector<Eigen::Vector3d> & start_end_derivative, double ratio,
  Eigen::MatrixXd & ctrl_pts, double & dt, double & time_inc)
{
  const double time_origin = bspline.getTimeSum();
  const int seg_num = static_cast<int>(bspline.getControlPoint().cols()) - 3;
  if (seg_num <= 0) {
    ctrl_pts.resize(0, 0);
    dt = 0.0;
    time_inc = 0.0;
    return;
  }

  bspline.lengthenTime(ratio);
  const double duration = bspline.getTimeSum();
  dt = duration / static_cast<double>(seg_num);
  time_inc = duration - time_origin;

  std::vector<Eigen::Vector3d> point_set;
  const double dt_safe = std::max(dt, 1e-9);  // 防呆：dt 为 0 会导致死循环
  constexpr int kMaxReparamIter = 100000;
  int reparam_iter = 0;
  for (double time = 0.0; time <= duration + 1e-4 && reparam_iter < kMaxReparamIter;
    time += dt_safe, ++reparam_iter)
  {
    point_set.push_back(bspline.evaluateDeBoorT(time));
  }

  ego_planner::UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
}

void PlannerInterfaceDog::getPlannedTraj(
  std::vector<PathPoint2D> & out_traj_points,
  double sample_dt) const
{
  out_traj_points.clear();
  if (local_data_.duration_ <= 0.0) {return;}

  // 防呆：sample_dt 必须为正且有限，避免死循环
  if (sample_dt <= 0.0 || !std::isfinite(sample_dt)) {sample_dt = 0.1;}

  const Eigen::MatrixXd pos_pts = local_data_.position_traj_.getControlPoint();
  const Eigen::VectorXd knots = local_data_.position_traj_.getKnot();

  ego_planner::UniformBspline pos_traj(
    pos_pts, 3, local_data_.position_traj_.getInterval());
  pos_traj.setKnot(knots);

  const double T = pos_traj.getTimeSum();
  constexpr std::size_t kMaxSampleIntervals = 100000U;
  const double interval_count_real = std::ceil(T / sample_dt);
  if (!std::isfinite(interval_count_real) || interval_count_real < 1.0 ||
    interval_count_real > static_cast<double>(kMaxSampleIntervals))
  {
    return;
  }
  const std::size_t interval_count = static_cast<std::size_t>(interval_count_real);
  out_traj_points.reserve(interval_count + 1U);
  for (std::size_t index = 0; index < interval_count; ++index) {
    const Eigen::Vector3d point = pos_traj.evaluateDeBoorT(
      static_cast<double>(index) * sample_dt);
    if (!point.allFinite()) {
      out_traj_points.clear();
      return;
    }
    out_traj_points.push_back(PathPoint2D{point.x(), point.y()});
  }
  const Eigen::Vector3d endpoint = pos_traj.evaluateDeBoorT(T);
  if (!endpoint.allFinite()) {
    out_traj_points.clear();
    return;
  }
  out_traj_points.push_back(PathPoint2D{endpoint.x(), endpoint.y()});
}

bool PlannerInterfaceDog::trajectoryIsCollisionFree(
  const ego_planner::UniformBspline & trajectory) const
{
  if (!grid_map_ || !trajectory.isValid()) {
    return false;
  }
  const double duration = trajectory.getTimeSum();
  const double resolution = grid_map_->getResolution();
  const double speed_bound = pp_.max_vel_ * std::sqrt(2.0) *
    (1.0 + std::max(0.0, pp_.feasibility_tolerance_));
  if (!std::isfinite(duration) || duration <= 0.0 ||
    !std::isfinite(resolution) || resolution <= 0.0 ||
    !std::isfinite(speed_bound) || speed_bound <= 0.0)
  {
    return false;
  }

  const double requested_step = std::min(
    trajectory.getInterval() / 10.0, resolution / (4.0 * speed_bound));
  if (!std::isfinite(requested_step) || requested_step <= 0.0) {
    return false;
  }
  constexpr std::size_t kMaximumCollisionSamples = 100000U;
  const double interval_count_real = std::ceil(duration / requested_step);
  if (!std::isfinite(interval_count_real) || interval_count_real < 1.0 ||
    interval_count_real >= static_cast<double>(kMaximumCollisionSamples))
  {
    return false;
  }
  const std::size_t interval_count = static_cast<std::size_t>(interval_count_real);
  for (std::size_t index = 0; index <= interval_count; ++index) {
    const double time = duration * static_cast<double>(index) /
      static_cast<double>(interval_count);
    const Eigen::VectorXd point = trajectory.evaluateDeBoorT(time);
    if (point.rows() < 2 || !point.allFinite() ||
      grid_map_->getInflateOccupancy(point.head<2>()))
    {
      return false;
    }
  }
  return true;
}

void PlannerInterfaceDog::getLastControlPoints(Eigen::MatrixXd & out_ctrl_pts_2d) const
{
  out_ctrl_pts_2d = last_ctrl_pts_2d_;
}

double PlannerInterfaceDog::getLastReboundOptimizeWallMs() const
{
  if (!optimizer_) {return 0.0;}
  return optimizer_->getLastReboundOptimizeWallMs();
}

}  // namespace dog_ego_planner
