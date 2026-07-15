// Copyright (c) ZJU FAST Lab contributors
// Copyright 2026 hengzhiyou
// SPDX-License-Identifier: GPL-3.0-only

#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

#include <Eigen/Eigen>
#include <bspline_opt/uniform_bspline.h>
#include <lbfgs.hpp>

#include "mapping/grid_map.hpp"
#include "planning/a_star.hpp"

// Gradient and elasitc band optimization

// Input: a signed distance field and a sequence of points
// Output: the optimized sequence of points
// The format of points: N x 3 matrix, each row is a point
namespace ego_planner
{

  class ControlPoints
  {
  public:
    double clearance{0.0};
    int size{0};
    Eigen::MatrixXd points;
    std::vector<std::vector<Eigen::Vector2d>> base_point; // The point at the statrt of the direction vector (collision point)
    std::vector<std::vector<Eigen::Vector2d>> direction;  // Direction vector, must be normalized.
    std::vector<bool> flag_temp;                          // A flag that used in many places. Initialize it everytime before using it.
    // std::vector<bool> occupancy;

    void resize(const int size_set)
    {
      size = std::max(0, size_set);

      base_point.clear();
      direction.clear();
      flag_temp.clear();
      // occupancy.clear();

      points.resize(2, size);
      base_point.resize(size);
      direction.resize(size);
      flag_temp.assign(static_cast<std::size_t>(size), false);
      // occupancy.resize(size);
    }
  };

  class BsplineOptimizer
  {

  public:
    BsplineOptimizer() = default;
    ~BsplineOptimizer() = default;

    /* main API */
    void setEnvironment(const std::shared_ptr<GridMap2D> &env);
    void setParam(double max_vel = 1.0, double max_acc = 2.0, double dist0 = 0.3);
    void setPrintfOpenOrNot(bool enabled);
    void setMaxReboundRetries(int max_retries);
    /* helper function */
    Eigen::MatrixXd convert2DTo3D(const Eigen::MatrixXd& points_2d) const;

    // required inputs
    void setControlPoints(const Eigen::MatrixXd &points);
    void setBsplineInterval(const double &ts);
    AStar::Ptr a_star_;
    std::vector<Eigen::Vector2d> ref_pts_;

    std::vector<std::vector<Eigen::Vector2d>> initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init = true);
    bool BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts); // must be called after initControlPoints()
    bool BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points);

    int getOrder(void) const noexcept { return order_; }

    /** 最近一次 rebound_optimize() 墙钟耗时（毫秒），供上层统计/打印。 */
    double getLastReboundOptimizeWallMs() const { return last_rebound_optimize_ms_; }

  private:
    std::shared_ptr<GridMap2D> grid_map_;


    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_{DONT_STOP};

    // main input
    // Eigen::MatrixXd control_points_;     // B-spline control points, N x dim
    double bspline_interval_{0.0}; // B-spline knot span

    /* optimization parameters */
    int order_{3};                  // bspline degree
    double lambda1_{10.0};          // jerk smoothness weight
    double lambda2_{5.0};           // distance weight
    double new_lambda2_{5.0};
    double lambda3_{3.0};           // feasibility weight
    double lambda4_{1.0};           // curve fitting
    int max_rebound_retries_{5};

    double last_rebound_optimize_ms_{0.0};
    bool rebound_control_points_ready_{false};

    double dist0_{0.3};             // safe distance
    double max_vel_{1.0}, max_acc_{2.0}; // dynamic limits

    int variable_num_{0}; // optimization variables
    int iter_num_{0};     // iteration of the solver

    ControlPoints cps_;

    /* cost function */
    /* calculate each part of cost function with control points q as input */

    // q contains all control points
    void calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                            Eigen::MatrixXd &gradient, bool use_jerk = true);
    void calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                             Eigen::MatrixXd &gradient);
    void calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost);
    void calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    bool check_collision_and_rebound(void);

    static int earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls);
    static double costFunctionRebound(void *func_data, const double *x, double *grad, const int n);
    static double costFunctionRefine(void *func_data, const double *x, double *grad, const int n);

    bool rebound_optimize();
    bool refine_optimize();
    void combineCostRebound(const double *x, double *grad, double &f_combine, const int n);
    void combineCostRefine(const double *x, double *grad, double &f_combine, const int n);

    /* for benckmark evaluation only */
  public:
    using Ptr = std::unique_ptr<BsplineOptimizer>;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace ego_planner
#endif
