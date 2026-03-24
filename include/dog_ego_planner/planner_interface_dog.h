/*
 * 2D EGO-Planner style interface (dog version).
 * Reuses demo bspline_opt/path_searching/GridMap2D implementation.
 */
#pragma once

#include <chrono>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <plan_manage/plan_container.hpp>

namespace dog_ego_planner
{

struct PathPoint2D
{
  double x{0.0};
  double y{0.0};
};

struct Obstacle2D
{
  double x{0.0};
  double y{0.0};
};

class PlannerInterfaceDog
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PlannerInterfaceDog();
  ~PlannerInterfaceDog();

  void initParam(double max_vel, double max_acc, double max_jerk, double control_point_interval);
  void initGridMap(double x_size, double y_size, double resolution, const Eigen::Vector2d& origin,
                   double inflate_radius, int astar_pool_size = 100);
  void setPrintfOpenOrNot(bool enabled);
  void setMaxReboundRetries(int max_retries);

  void setCurrentPose(const PathPoint2D& cur_pose);
  void setReferencePath(const std::vector<PathPoint2D>& path_pts);   // curve1 dense points
  void setObstacles(const std::vector<Obstacle2D>& obstacles);       // 2D obstacle points
  bool getInflatedOccupancyGrid(std::vector<int8_t>& data, int& width, int& height,
                                double& resolution, Eigen::Vector2d& origin) const;

  bool makePlan(const Eigen::Vector2d& start_vel, const Eigen::Vector2d& start_acc,
                const Eigen::Vector2d& local_target_vel);

  void getPlannedTraj(std::vector<PathPoint2D>& out_traj_points, double sample_dt = 0.1) const;
  void getLastControlPoints(Eigen::MatrixXd& out_ctrl_pts_2d) const;

  /** 最近一次 rebound 优化墙钟耗时（毫秒），与 BsplineOptimizer 一致。 */
  double getLastReboundOptimizeWallMs() const;

private:
  bool reboundReplan(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& start_vel,
                     const Eigen::Vector3d& start_acc, const Eigen::Vector3d& local_target_pt,
                     const Eigen::Vector3d& local_target_vel, const std::vector<Eigen::Vector3d>& point_set);

  bool refineTrajAlgo(ego_planner::UniformBspline& traj, std::vector<Eigen::Vector3d>& start_end_derivative,
                      double ratio, double& ts, Eigen::MatrixXd& optimal_control_points);

  void updateTrajInfo(const ego_planner::UniformBspline& position_traj);
  void reparamBspline(ego_planner::UniformBspline& bspline, std::vector<Eigen::Vector3d>& start_end_derivative,
                      double ratio, Eigen::MatrixXd& ctrl_pts, double& dt, double& time_inc);

private:
  ego_planner::PlanParameters pp_;
  ego_planner::LocalTrajData local_data_;
  std::shared_ptr<GridMap2D> grid_map_;
  ego_planner::BsplineOptimizer::Ptr optimizer_;
  bool printf_open_or_not_{true};
  int max_rebound_retries_{5};

  PathPoint2D cur_pose_{};
  std::vector<PathPoint2D> ref_path_{};
  mutable Eigen::MatrixXd last_ctrl_pts_2d_;
};

}  // namespace dog_ego_planner

