#include "dog_ego_planner/planner_interface_dog.h"

#include <cmath>
#include <cstdio>
#include <iostream>

#include <path_searching/dyn_a_star.h>

namespace dog_ego_planner
{

PlannerInterfaceDog::PlannerInterfaceDog() = default;
PlannerInterfaceDog::~PlannerInterfaceDog() = default;

void PlannerInterfaceDog::initParam(double max_vel, double max_acc, double max_jerk, double control_point_interval)
{
  pp_.max_vel_ = std::max(max_vel, 1e-6);   // 防呆：避免除零
  pp_.max_acc_ = std::max(max_acc, 1e-6);
  pp_.max_jerk_ = std::max(max_jerk, 1e-6);
  pp_.feasibility_tolerance_ = 0.05;

  // ctrl_pt_dist：相邻 B 样条控制点期望弧长间距，影响初始 knot 间隔 ts（见 reboundReplan）
  pp_.ctrl_pt_dist = std::max(control_point_interval, 1e-3);
  pp_.planning_horizen_ = 7.0;   // 与 dog_planner_node 中 planning_horizon 用途独立；保留默认
}

void PlannerInterfaceDog::initGridMap(double x_size, double y_size, double resolution,
                                      const Eigen::Vector2d& origin, double inflate_radius,
                                      int astar_pool_size)
{
  (void)origin;  // GridMap2D in demo internally defines its origin behavior; keep signature for future.
  Eigen::Vector2i map_size(static_cast<int>(x_size), static_cast<int>(y_size));
  grid_map_ = std::make_shared<GridMap2D>(resolution, map_size);
  grid_map_->setPrintfOpenOrNot(printf_open_or_not_);
  grid_map_->setInflateRadius(inflate_radius);

  // 板端调试：用 cerr 无缓冲输出，精确定位卡顿位置
  std::cerr << "[initGridMap] 1/5 创建 BsplineOptimizer..." << std::endl;
  optimizer_.reset(new ego_planner::BsplineOptimizer);
  std::cerr << "[initGridMap] 2/5 setParam/setEnv..." << std::endl;
  optimizer_->setParam();
  optimizer_->setPrintfOpenOrNot(printf_open_or_not_);
  optimizer_->setMaxReboundRetries(max_rebound_retries_);
  optimizer_->setEnvironment(grid_map_);
  std::cerr << "[initGridMap] 3/5 创建 A* 对象..." << std::endl;
  optimizer_->a_star_.reset(new AStar);
  const int pool = std::max(20, std::min(astar_pool_size, 200));  // 防呆：限制范围
  std::cerr << "[initGridMap] 4/5 分配 A* 栅格池(" << pool << "x" << pool << ")..." << std::endl;
  optimizer_->a_star_->initGridMap(grid_map_, Eigen::Vector2i(pool, pool));
  std::cerr << "[initGridMap] 5/5 规划器初始化完成。" << std::endl;
}

void PlannerInterfaceDog::setPrintfOpenOrNot(bool enabled)
{
  printf_open_or_not_ = enabled;
  if (grid_map_) {
    grid_map_->setPrintfOpenOrNot(enabled);
  }
  if (optimizer_) {
    optimizer_->setPrintfOpenOrNot(enabled);
  }
}

void PlannerInterfaceDog::setMaxReboundRetries(int max_retries)
{
  max_rebound_retries_ = (max_retries < 0) ? 0 : max_retries;
  if (optimizer_) {
    optimizer_->setMaxReboundRetries(max_rebound_retries_);
  }
}

void PlannerInterfaceDog::setCurrentPose(const PathPoint2D& cur_pose)
{
  cur_pose_ = cur_pose;
  if (grid_map_) {
    grid_map_->setCurPose(cur_pose_.x, cur_pose_.y);
  }
}

void PlannerInterfaceDog::setReferencePath(const std::vector<PathPoint2D>& path_pts)
{
  ref_path_ = path_pts;
}

void PlannerInterfaceDog::setObstacles(const std::vector<Obstacle2D>& obstacles)
{
  if (!grid_map_) return;

  grid_map_->resetMap();

  constexpr size_t kMaxObstacles = 500000;  // 防呆：限制单次障碍数量，避免内存爆炸
  const size_t n = std::min(obstacles.size(), kMaxObstacles);
  for (size_t i = 0; i < n; ++i) {
    const auto& ob = obstacles[i];
    Eigen::Vector2d coord(ob.x, ob.y);
    Eigen::Vector2i idx = grid_map_->worldToGrid(coord);
    grid_map_->setObstacle(idx, true);
  }

  grid_map_->inflate();
}

bool PlannerInterfaceDog::getInflatedOccupancyGrid(std::vector<int8_t>& data, int& width, int& height,
                                                   double& resolution, Eigen::Vector2d& origin) const
{
  if (!grid_map_) return false;
  const Eigen::Vector2i size = grid_map_->mapSize();
  width = size.x();
  height = size.y();
  resolution = grid_map_->resolution();
  origin = grid_map_->origin();
  grid_map_->getOccupancyGridData(data, true);
  return (width > 0 && height > 0 && data.size() == static_cast<size_t>(width * height));
}

bool PlannerInterfaceDog::makePlan(const Eigen::Vector2d& start_vel_2d,
                                   const Eigen::Vector2d& start_acc_2d,
                                   const Eigen::Vector2d& local_target_vel_2d)
{
  if (!grid_map_ || !optimizer_) {
    std::cerr << "[PlannerInterfaceDog] grid map not initialized.\n";
    return false;
  }
  if (ref_path_.size() < 4) {
    std::cerr << "[PlannerInterfaceDog] reference path too short.\n";
    return false;
  }

  std::vector<Eigen::Vector3d> point_set;
  point_set.reserve(ref_path_.size());
  for (const auto& p : ref_path_) {
    point_set.emplace_back(p.x, p.y, 0.0);
  }

  Eigen::Vector3d start_pt(ref_path_.front().x, ref_path_.front().y, 0.0);
  Eigen::Vector3d local_target_pt(ref_path_.back().x, ref_path_.back().y, 0.0);

  Eigen::Vector3d start_vel(start_vel_2d.x(), start_vel_2d.y(), 0.0);
  Eigen::Vector3d start_acc(start_acc_2d.x(), start_acc_2d.y(), 0.0);
  Eigen::Vector3d local_target_vel(local_target_vel_2d.x(), local_target_vel_2d.y(), 0.0);

  return reboundReplan(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, point_set);
}

bool PlannerInterfaceDog::reboundReplan(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& start_vel,
                                       const Eigen::Vector3d& start_acc, const Eigen::Vector3d& local_target_pt,
                                       const Eigen::Vector3d& local_target_vel,
                                       const std::vector<Eigen::Vector3d>& point_set)
{
  if (point_set.size() < 4) {
    std::cerr << "[PlannerInterfaceDog] point_set too short for bspline parameterization.\n";
    return false;
  }

  std::vector<Eigen::Vector3d> start_end_derivatives;
  start_end_derivatives.push_back(start_vel);
  start_end_derivatives.push_back(local_target_vel);
  start_end_derivatives.push_back(start_acc);
  start_end_derivatives.push_back(start_acc);

  const double dist = (start_pt - local_target_pt).norm();
  const double max_vel_safe = std::max(pp_.max_vel_, 1e-6);  // 防呆：避免除零
  double ts = dist > 0.1 ? pp_.ctrl_pt_dist / max_vel_safe * 1.2 : pp_.ctrl_pt_dist / max_vel_safe * 5.0;

  Eigen::MatrixXd ctrl_pts_3d;
  ego_planner::UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts_3d);
  if (ctrl_pts_3d.rows() < 3 || ctrl_pts_3d.cols() < 4) {
    std::cerr << "[PlannerInterfaceDog] invalid control points from parameterizeToBspline: "
              << ctrl_pts_3d.rows() << "x" << ctrl_pts_3d.cols() << "\n";
    return false;
  }
  if (!ctrl_pts_3d.allFinite()) {
    std::cerr << "[PlannerInterfaceDog] non-finite control points from parameterizeToBspline.\n";
    return false;
  }
  Eigen::MatrixXd ctrl_pts_2d = ctrl_pts_3d.topRows(2);

  optimizer_->initControlPoints(ctrl_pts_2d, true);

  const bool step1_ok = optimizer_->BsplineOptimizeTrajRebound(ctrl_pts_2d, ts);
  if (!step1_ok) {
    std::cerr << "[PlannerInterfaceDog] Bspline rebound optimization failed.\n";
    return false;
  }

  last_ctrl_pts_2d_ = ctrl_pts_2d;

  ctrl_pts_3d = optimizer_->convert2DTo3D(ctrl_pts_2d);
  ego_planner::UniformBspline pos(ctrl_pts_3d, 3, ts);
  pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

  double ratio = 1.0;
  bool step2_ok = true;
  if (!pos.checkFeasibility(ratio, false)) {
    Eigen::MatrixXd optimal_control_points;
    step2_ok = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
    if (step2_ok) {
      pos = ego_planner::UniformBspline(optimal_control_points, 3, ts);
    }
  }

  if (!step2_ok) {
    std::cerr << "[PlannerInterfaceDog] Time reallocation refine failed (keep step1 result).\n";
    // Degrade gracefully: keep step1 result instead of failing hard.
  }

  updateTrajInfo(pos);
  return true;
}

bool PlannerInterfaceDog::refineTrajAlgo(ego_planner::UniformBspline& traj,
                                        std::vector<Eigen::Vector3d>& start_end_derivative, double ratio, double& ts,
                                        Eigen::MatrixXd& optimal_control_points)
{
  Eigen::MatrixXd ctrl_pts;
  double t_inc = 0.0;
  reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);
  if (ctrl_pts.rows() < 3 || ctrl_pts.cols() < 4) {
    std::cerr << "[PlannerInterfaceDog] invalid control points after reparamBspline: "
              << ctrl_pts.rows() << "x" << ctrl_pts.cols() << "\n";
    return false;
  }
  if (!ctrl_pts.allFinite()) {
    std::cerr << "[PlannerInterfaceDog] non-finite control points after reparamBspline.\n";
    return false;
  }

  Eigen::MatrixXd ctrl_pts_2d = ctrl_pts.topRows(2);
  traj = ego_planner::UniformBspline(ctrl_pts, 3, ts);

  const int denom = static_cast<int>(ctrl_pts.cols()) - 3;
  if (denom <= 0) {
    std::cerr << "[PlannerInterfaceDog] invalid segment number for refine: cols=" << ctrl_pts.cols() << "\n";
    return false;
  }
  const double time_sum = traj.getTimeSum();
  const double t_step = (denom > 0 && time_sum > 1e-12) ? (time_sum / double(denom)) : 0.01;  // 防呆：避免除零
  const double step_safe = std::max(t_step, 1e-9);  // 防呆：保证步长 > 0
  optimizer_->ref_pts_.clear();
  constexpr int kMaxRefPtsIter = 100000;
  int ref_iter = 0;
  for (double t = 0.0; t < time_sum + 1e-4 && ref_iter < kMaxRefPtsIter; t += step_safe, ++ref_iter) {
    optimizer_->ref_pts_.push_back(traj.evaluateDeBoorT(t).topRows(2));
  }

  Eigen::MatrixXd optimal_control_point_temp;
  const bool success = optimizer_->BsplineOptimizeTrajRefine(ctrl_pts_2d, ts, optimal_control_point_temp);
  optimal_control_points = optimizer_->convert2DTo3D(ctrl_pts_2d);
  return success;
}

void PlannerInterfaceDog::updateTrajInfo(const ego_planner::UniformBspline& position_traj)
{
  local_data_.position_traj_ = position_traj;
  local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
  local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
  local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
  local_data_.duration_ = local_data_.position_traj_.getTimeSum();
  local_data_.traj_id_ += 1;
}

void PlannerInterfaceDog::reparamBspline(ego_planner::UniformBspline& bspline,
                                        std::vector<Eigen::Vector3d>& start_end_derivative, double ratio,
                                        Eigen::MatrixXd& ctrl_pts, double& dt, double& time_inc)
{
  const double time_origin = bspline.getTimeSum();
  const int seg_num = static_cast<int>(bspline.getControlPoint().cols()) - 3;
  if (seg_num <= 0) {
    ctrl_pts.resize(0, 0);
    dt = 0.0;
    time_inc = 0.0;
    std::cerr << "[PlannerInterfaceDog] invalid seg_num in reparamBspline: cols="
              << bspline.getControlPoint().cols() << "\n";
    return;
  }

  bspline.lengthenTime(ratio);
  const double duration = bspline.getTimeSum();
  dt = duration / double(seg_num);
  time_inc = duration - time_origin;

  std::vector<Eigen::Vector3d> point_set;
  const double dt_safe = std::max(dt, 1e-9);  // 防呆：dt 为 0 会导致死循环
  constexpr int kMaxReparamIter = 100000;
  int reparam_iter = 0;
  for (double time = 0.0; time <= duration + 1e-4 && reparam_iter < kMaxReparamIter; time += dt_safe, ++reparam_iter) {
    point_set.push_back(bspline.evaluateDeBoorT(time));
  }

  ego_planner::UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
}

void PlannerInterfaceDog::getPlannedTraj(std::vector<PathPoint2D>& out_traj_points, double sample_dt) const
{
  out_traj_points.clear();
  if (local_data_.duration_ <= 0.0) return;

  // 防呆：sample_dt 必须为正且有限，避免死循环
  if (sample_dt <= 0.0 || !std::isfinite(sample_dt)) sample_dt = 0.1;

  // Demo UniformBspline getters are non-const; make a local copy.
  ego_planner::UniformBspline pos_copy = local_data_.position_traj_;
  const Eigen::MatrixXd pos_pts = pos_copy.getControlPoint();
  const Eigen::VectorXd knots = pos_copy.getKnot();

  ego_planner::UniformBspline pos_traj(pos_pts, 3, pos_copy.getInterval());
  pos_traj.setKnot(knots);

  const double T = pos_traj.getTimeSum();
  constexpr int kMaxSampleIter = 100000;
  int sample_iter = 0;
  for (double t = 0.0; t <= T + 1e-6 && sample_iter < kMaxSampleIter; t += sample_dt, ++sample_iter) {
    const Eigen::Vector3d p = pos_traj.evaluateDeBoorT(t);
    out_traj_points.push_back(PathPoint2D{p.x(), p.y()});
  }
}

void PlannerInterfaceDog::getLastControlPoints(Eigen::MatrixXd& out_ctrl_pts_2d) const
{
  out_ctrl_pts_2d = last_ctrl_pts_2d_;
}

double PlannerInterfaceDog::getLastReboundOptimizeWallMs() const
{
  if (!optimizer_) return 0.0;
  return optimizer_->getLastReboundOptimizeWallMs();
}

}  // namespace dog_ego_planner

