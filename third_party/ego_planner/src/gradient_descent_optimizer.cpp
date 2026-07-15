// Copyright (c) ZJU FAST Lab contributors
// Copyright 2026 hengzhiyou
// SPDX-License-Identifier: GPL-3.0-only

#include <bspline_opt/gradient_descent_optimizer.h>

#include <algorithm>
#include <cmath>
#include <limits>

GradientDescentOptimizer::RESULT
GradientDescentOptimizer::optimize(Eigen::VectorXd &x_init_optimal, double &opt_f)
{
    opt_f = std::numeric_limits<double>::infinity();
    if (objfun_ == nullptr || variable_num_ <= 0 || x_init_optimal.rows() != variable_num_ ||
        !x_init_optimal.allFinite())
    {
        return FAILED;
    }
    if (!std::isfinite(min_grad_) || min_grad_ < 1e-10)
    {
        return FAILED;
    }
    if (iter_limit_ <= 2 || invoke_limit_ <= 2)
    {
        return FAILED;
    }

    void *f_data = f_data_;
    int iter = 2;
    int invoke_count = 2;
    bool force_return = false;
    Eigen::VectorXd x_k(x_init_optimal), x_kp1(x_init_optimal.rows());
    double cost_k, cost_kp1;
    Eigen::VectorXd grad_k(x_init_optimal.rows()), grad_kp1(x_init_optimal.rows());

    cost_k = objfun_(x_k, grad_k, force_return, f_data);
    if (force_return)
        return RETURN_BY_ORDER;
    if (!std::isfinite(cost_k) || !grad_k.allFinite())
        return FAILED;
    double max_grad = std::max(std::abs(grad_k.maxCoeff()), std::abs(grad_k.minCoeff()));
    if (max_grad < min_grad_)
    {
        opt_f = cost_k;
        return FIND_MIN;
    }
    constexpr double MAX_MOVEMENT_AT_FIRST_ITERATION = 0.1; // meter
    double alpha0 = max_grad < MAX_MOVEMENT_AT_FIRST_ITERATION ? 1.0 : (MAX_MOVEMENT_AT_FIRST_ITERATION / max_grad);
    x_kp1 = x_k - alpha0 * grad_k;
    force_return = false;
    cost_kp1 = objfun_(x_kp1, grad_kp1, force_return, f_data);
    if (force_return)
        return RETURN_BY_ORDER;
    if (!std::isfinite(cost_kp1) || !grad_kp1.allFinite())
        return FAILED;

    /*** start iteration ***/
    while (++iter <= iter_limit_ && invoke_count <= invoke_limit_)
    {
        Eigen::VectorXd s = x_kp1 - x_k;
        Eigen::VectorXd y = grad_kp1 - grad_k;
        const double y_dot_y = y.dot(y);
        double alpha = (y_dot_y > 1e-20) ? (s.dot(y) / y_dot_y) : 0.01;  // 防呆：y 为零向量时避免除零
        if (!std::isfinite(alpha) || alpha <= 0.0)
        {
            return FAILED;
        }

        if (iter % 2) // to aviod copying operations
        {
            constexpr int kMaxArmijoIter = 100;  // 防呆：防止 Armijo 线搜索死循环
            int armijo_iter = 0;
            do
            {
                if (++armijo_iter > kMaxArmijoIter) {
                    return REACH_MAX_ITERATION;
                }
                x_k = x_kp1 - alpha * grad_kp1;
                force_return = false;
                cost_k = objfun_(x_k, grad_k, force_return, f_data);
                invoke_count++;
                if (force_return)
                    return RETURN_BY_ORDER;
                if (!std::isfinite(cost_k) || !grad_k.allFinite())
                    return FAILED;
                alpha *= 0.5;
                if (alpha < 1e-20) break;  // 防呆：alpha 下溢则退出
            } while (cost_k > cost_kp1 - 1e-4 * alpha * grad_kp1.transpose() * grad_kp1); // Armijo condition

            if (grad_k.norm() < min_grad_)
            {
                x_init_optimal = x_k;
                opt_f = cost_k;
                return FIND_MIN;
            }
            if ((x_k - x_kp1).norm() <= xtol_abs_ + xtol_rel_ * x_kp1.norm())
            {
                x_init_optimal = x_k;
                opt_f = cost_k;
                return FIND_MIN;
            }
        }
        else
        {
            constexpr int kMaxArmijoIter2 = 100;  // 防呆
            int armijo_iter2 = 0;
            do
            {
                if (++armijo_iter2 > kMaxArmijoIter2) {
                    return REACH_MAX_ITERATION;
                }
                x_kp1 = x_k - alpha * grad_k;
                force_return = false;
                cost_kp1 = objfun_(x_kp1, grad_kp1, force_return, f_data);
                invoke_count++;
                if (force_return)
                    return RETURN_BY_ORDER;
                if (!std::isfinite(cost_kp1) || !grad_kp1.allFinite())
                    return FAILED;
                alpha *= 0.5;
                if (alpha < 1e-20) break;  // 防呆：alpha 下溢则退出
            } while (cost_kp1 > cost_k - 1e-4 * alpha * grad_k.transpose() * grad_k); // Armijo condition

            if (grad_kp1.norm() < min_grad_)
            {
                x_init_optimal = x_kp1;
                opt_f = cost_kp1;
                return FIND_MIN;
            }
            if ((x_kp1 - x_k).norm() <= xtol_abs_ + xtol_rel_ * x_k.norm())
            {
                x_init_optimal = x_kp1;
                opt_f = cost_kp1;
                return FIND_MIN;
            }
        }
    }

    if (iter_limit_ % 2)
    {
        x_init_optimal = x_k;
        opt_f = cost_k;
    }
    else
    {
        x_init_optimal = x_kp1;
        opt_f = cost_kp1;
    }
    return REACH_MAX_ITERATION;
}
