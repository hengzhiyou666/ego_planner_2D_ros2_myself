// Copyright (c) ZJU FAST Lab contributors
// Copyright 2026 hengzhiyou
// SPDX-License-Identifier: GPL-3.0-only

#include "bspline_opt/uniform_bspline.h"

#include <cmath>
#include <stdexcept>

namespace ego_planner
{

  UniformBspline::UniformBspline(const Eigen::MatrixXd &points, const int &order,
                                 const double &interval)
  {
    setUniformBspline(points, order, interval);
  }

  void UniformBspline::setUniformBspline(const Eigen::MatrixXd &points, const int &order,
                                         const double &interval)
  {
    if (points.rows() <= 0 || points.cols() <= order || !points.allFinite())
    {
      throw std::invalid_argument("UniformBspline requires finite control points and cols > order");
    }
    if (order < 0)
    {
      throw std::invalid_argument("UniformBspline order must be non-negative");
    }
    if (!std::isfinite(interval) || interval <= 0.0)
    {
      throw std::invalid_argument("UniformBspline interval must be finite and positive");
    }

    control_points_ = points;
    p_ = order;
    interval_ = interval;

    n_ = points.cols() - 1;
    m_ = n_ + p_ + 1;

    u_ = Eigen::VectorXd::Zero(m_ + 1);
    for (int i = 0; i <= m_; ++i)
    {
      u_(i) = static_cast<double>(i - p_) * interval_;
    }
  }

  void UniformBspline::setKnot(const Eigen::VectorXd &knot)
  {
    if (!isValid() || knot.rows() != m_ + 1 || !knot.allFinite())
    {
      throw std::invalid_argument("UniformBspline knot vector has invalid size or values");
    }
    for (Eigen::Index i = 1; i < knot.rows(); ++i)
    {
      if (knot(i) < knot(i - 1))
      {
        throw std::invalid_argument("UniformBspline knots must be non-decreasing");
      }
    }
    if (knot(m_ - p_) <= knot(p_))
    {
      throw std::invalid_argument("UniformBspline active knot span must be positive");
    }
    u_ = knot;
    interval_ = (u_(m_ - p_) - u_(p_)) / static_cast<double>(m_ - 2 * p_);
  }

  Eigen::VectorXd UniformBspline::getKnot() const { return u_; }

  bool UniformBspline::isValid() const
  {
    return p_ >= 0 && n_ >= p_ && m_ == n_ + p_ + 1 && control_points_.rows() > 0 &&
           control_points_.cols() == n_ + 1 && control_points_.allFinite() &&
           u_.rows() == m_ + 1 && u_.allFinite() && std::isfinite(interval_) && interval_ > 0.0;
  }

  bool UniformBspline::getTimeSpan(double &um, double &um_p) const
  {
    if (!isValid() || p_ >= u_.rows() || m_ - p_ < 0 || m_ - p_ >= u_.rows())
    {
      um = 0.0;
      um_p = 0.0;
      return false;
    }

    um = u_(p_);
    um_p = u_(m_ - p_);

    return std::isfinite(um) && std::isfinite(um_p) && um_p > um;
  }

  Eigen::MatrixXd UniformBspline::getControlPoint() const { return control_points_; }

  Eigen::VectorXd UniformBspline::evaluateDeBoor(const double &u) const
  {
    if (!isValid())
    {
      throw std::logic_error("cannot evaluate an uninitialized UniformBspline");
    }
    if (!std::isfinite(u))
    {
      throw std::invalid_argument("UniformBspline evaluation time must be finite");
    }

    const double ub = std::min(std::max(u_(p_), u), u_(m_ - p_));

    // determine which [ui,ui+1] lay in
    int k = p_;
    while (k < m_ - p_ - 1 && u_(k + 1) < ub)
    {
      ++k;
    }

    /* deBoor's alg */
    std::vector<Eigen::VectorXd> d;
    d.reserve(static_cast<std::size_t>(p_ + 1));
    for (int i = 0; i <= p_; ++i)
    {
      d.push_back(control_points_.col(k - p_ + i));
    }

    for (int r = 1; r <= p_; ++r)
    {
      for (int i = p_; i >= r; --i)
      {
        const double denominator = u_(i + 1 + k - r) - u_(i + k - p_);
        const double alpha = std::abs(denominator) > 1e-12
                               ? (ub - u_(i + k - p_)) / denominator
                               : 0.0;
        d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];
      }
    }

    return d[p_];
  }

  Eigen::VectorXd UniformBspline::evaluateDeBoorT(const double &t) const
  {
    if (!isValid())
    {
      throw std::logic_error("cannot evaluate an uninitialized UniformBspline");
    }
    return evaluateDeBoor(t + u_(p_));
  }

  Eigen::MatrixXd UniformBspline::getDerivativeControlPoints() const
  {
    if (!isValid() || p_ <= 0 || control_points_.cols() < 2)
    {
      return Eigen::MatrixXd();
    }
    // The derivative of a b-spline is also a b-spline, its order become p_-1
    // control point Qi = p_*(Pi+1-Pi)/(ui+p_+1-ui+1)
    Eigen::MatrixXd ctp(control_points_.rows(), control_points_.cols() - 1);
    for (int i = 0; i < ctp.cols(); ++i)
    {
      const double denom = u_(i + p_ + 1) - u_(i + 1);
      if (std::abs(denom) > 1e-12) {
        ctp.col(i) = p_ * (control_points_.col(i + 1) - control_points_.col(i)) / denom;
      } else {
        ctp.col(i) = Eigen::VectorXd::Zero(control_points_.rows());  // 防呆：避免除零
      }
    }
    return ctp;
  }

  UniformBspline UniformBspline::getDerivative() const
  {
    if (!isValid() || p_ <= 0)
    {
      return UniformBspline();
    }
    Eigen::MatrixXd ctp = getDerivativeControlPoints();
    UniformBspline derivative(ctp, p_ - 1, interval_);

    /* cut the first and last knot */
    Eigen::VectorXd knot(u_.rows() - 2);
    knot = u_.segment(1, u_.rows() - 2);
    derivative.setKnot(knot);

    return derivative;
  }

  double UniformBspline::getInterval() const { return interval_; }

  void UniformBspline::setPhysicalLimits(const double &vel, const double &acc, const double &tolerance,
                                         const double &jerk)
  {
    if (!std::isfinite(vel) || vel <= 0.0 || !std::isfinite(acc) || acc <= 0.0 ||
        !std::isfinite(tolerance) || tolerance < 0.0 ||
        ((!std::isfinite(jerk) && !std::isinf(jerk)) || jerk <= 0.0))
    {
      throw std::invalid_argument("B-spline physical limits must be positive and tolerance non-negative");
    }
    limit_vel_ = vel;
    limit_acc_ = acc;
    limit_jerk_ = jerk;
    limit_ratio_ = 1.1;
    feasibility_tolerance_ = tolerance;
  }

  bool UniformBspline::checkFeasibility(double &ratio, bool show) const
  {
    (void)show;
    ratio = std::numeric_limits<double>::infinity();
    if (!isValid() || !std::isfinite(limit_vel_) || !std::isfinite(limit_acc_) ||
        limit_vel_ <= 0.0 || limit_acc_ <= 0.0)
    {
      return false;
    }

    const auto max_vector_norm = [](const Eigen::MatrixXd &values) {
      double maximum = 0.0;
      for (Eigen::Index column = 0; column < values.cols(); ++column)
      {
        maximum = std::max(maximum, values.col(column).norm());
      }
      return maximum;
    };

    const Eigen::MatrixXd velocity_points = getDerivativeControlPoints();
    if (velocity_points.size() == 0 || !velocity_points.allFinite())
    {
      return false;
    }
    const double max_vel = max_vector_norm(velocity_points);

    double max_acc = 0.0;
    double max_jerk = 0.0;
    const UniformBspline velocity = getDerivative();
    if (p_ >= 2)
    {
      const Eigen::MatrixXd acceleration_points = velocity.getDerivativeControlPoints();
      if (!acceleration_points.allFinite())
      {
        return false;
      }
      max_acc = max_vector_norm(acceleration_points);
    }
    if (p_ >= 3)
    {
      const UniformBspline acceleration = velocity.getDerivative();
      const Eigen::MatrixXd jerk_points = acceleration.getDerivativeControlPoints();
      if (!jerk_points.allFinite())
      {
        return false;
      }
      max_jerk = max_vector_norm(jerk_points);
    }

    const double tolerance_scale = 1.0 + feasibility_tolerance_;
    const bool velocity_feasible = max_vel <= limit_vel_ * tolerance_scale + 1e-9;
    const bool acceleration_feasible = max_acc <= limit_acc_ * tolerance_scale + 1e-9;
    const bool jerk_limit_enabled = std::isfinite(limit_jerk_);
    const bool jerk_feasible = !jerk_limit_enabled || max_jerk <= limit_jerk_ * tolerance_scale + 1e-9;
    const bool feasible = velocity_feasible && acceleration_feasible && jerk_feasible;

    ratio = std::max(1.0, max_vel / limit_vel_);
    ratio = std::max(ratio, std::sqrt(max_acc / limit_acc_));
    if (jerk_limit_enabled)
    {
      ratio = std::max(ratio, std::cbrt(max_jerk / limit_jerk_));
    }
    if (!feasible)
    {
      ratio *= limit_ratio_;
    }

    return feasible;
  }

  void UniformBspline::lengthenTime(const double &ratio)
  {
    if (!isValid() || !std::isfinite(ratio) || ratio <= 1.0)
    {
      return;
    }

    const double anchor = u_(p_);
    u_ = (u_.array() - anchor) * ratio + anchor;
    interval_ *= ratio;
  }

  void UniformBspline::parameterizeToBspline(const double &ts,
                                             const std::vector<Eigen::Vector3d> &point_set,
                                             const std::vector<Eigen::Vector3d> &start_end_derivative,
                                             Eigen::MatrixXd &ctrl_pts)
  {
    ctrl_pts.resize(0, 0);
    if (!std::isfinite(ts) || ts <= 0.0 || point_set.size() <= 3 ||
        start_end_derivative.size() != 4)
    {
      return;
    }
    const auto finite_vector = [](const Eigen::Vector3d &value) { return value.allFinite(); };
    if (!std::all_of(point_set.begin(), point_set.end(), finite_vector) ||
        !std::all_of(start_end_derivative.begin(), start_end_derivative.end(), finite_vector))
    {
      return;
    }

    const int K = static_cast<int>(point_set.size());

    // write A
    Eigen::Vector3d prow(3), vrow(3), arow(3);
    prow << 1, 4, 1;
    vrow << -1, 0, 1;
    arow << 1, -2, 1;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4, K + 2);

    for (int i = 0; i < K; ++i)
      A.block(i, i, 1, 3) = (1 / 6.0) * prow.transpose();

    A.block(K, 0, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();
    A.block(K + 1, K - 1, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();

    A.block(K + 2, 0, 1, 3) = (1 / ts / ts) * arow.transpose();
    A.block(K + 3, K - 1, 1, 3) = (1 / ts / ts) * arow.transpose();

    // write b
    Eigen::MatrixXd b(K + 4, 3);
    for (int i = 0; i < K; ++i)
    {
      b.row(i) = point_set[static_cast<std::size_t>(i)].transpose();
    }

    for (int i = 0; i < 4; ++i)
    {
      b.row(K + i) = start_end_derivative[static_cast<std::size_t>(i)].transpose();
    }

    // solve Ax = b
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> solver(A);
    if (solver.rank() < A.cols())
    {
      return;
    }
    const Eigen::MatrixXd solution = solver.solve(b);
    if (!solution.allFinite())
    {
      return;
    }

    // convert to control pts
    ctrl_pts = solution.transpose();
  }

  double UniformBspline::getTimeSum() const
  {
    double tm, tmp;
    if (getTimeSpan(tm, tmp))
      return tmp - tm;
    else
      return -1.0;
  }

  double UniformBspline::getLength(const double &res) const
  {
    double length = 0.0;
    double dur = getTimeSum();
    if (dur <= 0.0 || res <= 0.0 || !std::isfinite(res)) return 0.0;  // 防呆：避免死循环
    const double step = std::max(res, 1e-9);  // 防呆：保证步长 > 0
    constexpr std::size_t kMaxLengthIntervals = 1000000U;
    const double interval_count_real = std::ceil(dur / step);
    if (!std::isfinite(interval_count_real) || interval_count_real < 1.0 ||
        interval_count_real > static_cast<double>(kMaxLengthIntervals))
    {
      return std::numeric_limits<double>::infinity();
    }
    const std::size_t interval_count = static_cast<std::size_t>(interval_count_real);
    Eigen::VectorXd p_l = evaluateDeBoorT(0.0), p_n;
    for (std::size_t index = 1; index < interval_count; ++index)
    {
      p_n = evaluateDeBoorT(static_cast<double>(index) * step);
      length += (p_n - p_l).norm();
      p_l = p_n;
    }
    p_n = evaluateDeBoorT(dur);
    length += (p_n - p_l).norm();
    return length;
  }

  double UniformBspline::getJerk() const
  {
    UniformBspline jerk_traj = getDerivative().getDerivative().getDerivative();

    Eigen::VectorXd times = jerk_traj.getKnot();
    Eigen::MatrixXd ctrl_pts = jerk_traj.getControlPoint();
    int dimension = ctrl_pts.rows();

    double jerk = 0.0;
    for (int i = 0; i < ctrl_pts.cols(); ++i)
    {
      for (int j = 0; j < dimension; ++j)
      {
        jerk += (times(i + 1) - times(i)) * ctrl_pts(j, i) * ctrl_pts(j, i);
      }
    }

    return jerk;
  }

  void UniformBspline::getMeanAndMaxVel(double &mean_v, double &max_v) const
  {
    UniformBspline vel = getDerivative();
    double tm, tmp;
    if (!vel.getTimeSpan(tm, tmp)) { mean_v = 0.0; max_v = 0.0; return; }  // 防呆
    constexpr double kStep = 0.01;
    constexpr int kMaxVelIter = 100000;
    double max_vel = -1.0, mean_vel = 0.0;
    int num = 0;
    for (double t = tm; t <= tmp + 1e-4 && num < kMaxVelIter; t += kStep, ++num)
    {
      Eigen::VectorXd vxd = vel.evaluateDeBoor(t);
      double vn = vxd.norm();

      mean_vel += vn;
      if (vn > max_vel)
      {
        max_vel = vn;
      }
    }

    mean_vel = (num > 0) ? (mean_vel / double(num)) : 0.0;  // 防呆：避免除零
    mean_v = mean_vel;
    max_v = max_vel;
  }

  void UniformBspline::getMeanAndMaxAcc(double &mean_a, double &max_a) const
  {
    UniformBspline acc = getDerivative().getDerivative();
    double tm, tmp;
    if (!acc.getTimeSpan(tm, tmp)) { mean_a = 0.0; max_a = 0.0; return; }  // 防呆
    constexpr double kStep = 0.01;
    constexpr int kMaxAccIter = 100000;
    double max_acc = -1.0, mean_acc = 0.0;
    int num = 0;
    for (double t = tm; t <= tmp + 1e-4 && num < kMaxAccIter; t += kStep, ++num)
    {
      Eigen::VectorXd axd = acc.evaluateDeBoor(t);
      double an = axd.norm();

      mean_acc += an;
      if (an > max_acc)
      {
        max_acc = an;
      }
    }

    mean_acc = (num > 0) ? (mean_acc / double(num)) : 0.0;  // 防呆：避免除零
    mean_a = mean_acc;
    max_a = max_acc;
  }
} // namespace ego_planner
