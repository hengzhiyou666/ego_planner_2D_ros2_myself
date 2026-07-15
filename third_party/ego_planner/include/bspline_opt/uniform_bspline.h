// Copyright (c) ZJU FAST Lab contributors
// Copyright 2026 hengzhiyou
// SPDX-License-Identifier: GPL-3.0-only

#ifndef _UNIFORM_BSPLINE_H_
#define _UNIFORM_BSPLINE_H_

#include <algorithm>
#include <limits>
#include <vector>

#include <Eigen/Eigen>

namespace ego_planner
{
  // An implementation of non-uniform B-spline with different dimensions
  // It also represents uniform B-spline which is a special case of non-uniform
  class UniformBspline
  {
  private:
    // control points for B-spline with different dimensions.
    // Each column represents one control point.
    // The dimension is determined by the row count.
    // e.g. B-spline with N points in 3D space -> 3xN matrix
    Eigen::MatrixXd control_points_;

    int p_{-1}, n_{-1}, m_{-1}; // p degree, n+1 control points, m = n+p+1
    Eigen::VectorXd u_; // knots vector
    double interval_{0.0}; // knot span \delta t

    Eigen::MatrixXd getDerivativeControlPoints() const;

    double limit_vel_{std::numeric_limits<double>::infinity()};
    double limit_acc_{std::numeric_limits<double>::infinity()};
    double limit_jerk_{std::numeric_limits<double>::infinity()};
    double limit_ratio_{1.1};
    double feasibility_tolerance_{0.0};

  public:
    UniformBspline() = default;
    UniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);
    ~UniformBspline() = default;

    Eigen::MatrixXd get_control_points(void) const { return control_points_; }

    // initialize as an uniform B-spline
    void setUniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);

    // get / set basic bspline info

    void setKnot(const Eigen::VectorXd &knot);
    Eigen::VectorXd getKnot() const;
    Eigen::MatrixXd getControlPoint() const;
    double getInterval() const;
    bool getTimeSpan(double &um, double &um_p) const;
    bool isValid() const;

    // compute position / derivative

    Eigen::VectorXd evaluateDeBoor(const double &u) const; // use u \in [up, u_mp]
    Eigen::VectorXd evaluateDeBoorT(const double &t) const; // use t \in [0, duration]
    UniformBspline getDerivative() const;

    // 3D B-spline interpolation of points in point_set, with boundary vel&acc
    // constraints
    // input : (K+2) points with boundary vel/acc; ts
    // output: (K+6) control_pts
    static void parameterizeToBspline(const double &ts, const std::vector<Eigen::Vector3d> &point_set,
                                      const std::vector<Eigen::Vector3d> &start_end_derivative,
                                      Eigen::MatrixXd &ctrl_pts);

    /* check feasibility, adjust time */

    void setPhysicalLimits(const double &vel, const double &acc, const double &tolerance,
                           const double &jerk = std::numeric_limits<double>::infinity());
    bool checkFeasibility(double &ratio, bool show = false) const;
    void lengthenTime(const double &ratio);

    /* for performance evaluation */

    double getTimeSum() const;
    double getLength(const double &res = 0.01) const;
    double getJerk() const;
    void getMeanAndMaxVel(double &mean_v, double &max_v) const;
    void getMeanAndMaxAcc(double &mean_a, double &max_a) const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace ego_planner
#endif
