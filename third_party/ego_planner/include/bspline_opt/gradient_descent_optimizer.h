// Copyright (c) ZJU FAST Lab contributors
// Copyright 2026 hengzhiyou
// SPDX-License-Identifier: GPL-3.0-only

#ifndef _GRADIENT_DESCENT_OPT_H_
#define _GRADIENT_DESCENT_OPT_H_

#include <limits>

#include <Eigen/Eigen>

class GradientDescentOptimizer
{

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef double (*objfunDef)(const Eigen::VectorXd &x, Eigen::VectorXd &grad, bool &force_return, void *data);
  enum RESULT
  {
    FIND_MIN,
    FAILED,
    RETURN_BY_ORDER,
    REACH_MAX_ITERATION
  };

  GradientDescentOptimizer(int v_num, objfunDef objf, void *f_data)
  : variable_num_(v_num > 0 ? v_num : 0), f_data_(f_data), objfun_(objf)
  {
  };

  void set_maxiter(int limit) { iter_limit_ = limit; }
  void set_maxeval(int limit) { invoke_limit_ = limit; }
  void set_xtol_rel(double xtol_rel) { xtol_rel_ = xtol_rel; }
  void set_xtol_abs(double xtol_abs) { xtol_abs_ = xtol_abs; }
  void set_min_grad(double min_grad) { min_grad_ = min_grad; }

  RESULT optimize(Eigen::VectorXd &x_init_optimal, double &opt_f);

private:
  int variable_num_{0};
  int iter_limit_{std::numeric_limits<int>::max()};
  int invoke_limit_{std::numeric_limits<int>::max()};
  double xtol_rel_{1e-6};
  double xtol_abs_{1e-8};
  double min_grad_{1e-6};
  void *f_data_{nullptr};
  objfunDef objfun_{nullptr};
};

#endif
