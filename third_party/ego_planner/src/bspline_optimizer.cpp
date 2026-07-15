// Copyright (c) ZJU FAST Lab contributors
// Copyright 2026 hengzhiyou
// SPDX-License-Identifier: GPL-3.0-only

#include "bspline_opt/bspline_optimizer.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

namespace ego_planner
{

namespace
{

bool validControlPoints(const Eigen::MatrixXd &points, int order)
{
  return points.rows() == 2 && points.cols() > 2 * order && points.allFinite();
}

bool validTimeStep(double time_step)
{
  return std::isfinite(time_step) && time_step > 0.0;
}

bool acceptedLbfgsResult(int result)
{
  return result == lbfgs::LBFGS_CONVERGENCE || result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
         result == lbfgs::LBFGS_ALREADY_MINIMIZED || result == lbfgs::LBFGS_STOP;
}

bool finiteArray(const std::vector<double> &values)
{
  return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
}

} // namespace

void BsplineOptimizer::setParam(double max_vel, double max_acc, double dist0)
{

    lambda1_ = 10.0;    // 平滑
    lambda2_ = 5.0;     // 碰撞（原1.0太低，平滑项完全主导导致避障不足）
    lambda3_ = 3.0;     // feasibility
    lambda4_ = 1.0;
    dist0_   = std::max(dist0, 0.05);

    max_vel_ = std::max(max_vel, 1e-6);
    max_acc_ = std::max(max_acc, 1e-6);

    order_   = 3;

}

void BsplineOptimizer::setEnvironment(const std::shared_ptr<GridMap2D> &env)
{
  this->grid_map_ = env;
  rebound_control_points_ready_ = false;
}

void BsplineOptimizer::setPrintfOpenOrNot(bool enabled)
{
  (void)enabled;
}

void BsplineOptimizer::setMaxReboundRetries(int max_retries)
{
  max_rebound_retries_ = (max_retries < 0) ? 0 : max_retries;
}

void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd &points)
{
  rebound_control_points_ready_ = false;
  if (points.rows() != 2 || points.cols() <= 0 || !points.allFinite())
  {
    cps_.resize(0);
    return;
  }
  cps_.resize(static_cast<int>(points.cols()));
  cps_.points = points;
}

void BsplineOptimizer::setBsplineInterval(const double &ts)
{
  bspline_interval_ = validTimeStep(ts) ? ts : 0.0;
}

std::vector<std::vector<Eigen::Vector2d>> BsplineOptimizer::initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init /*= true*/)
{
    rebound_control_points_ready_ = false;
    if (!grid_map_ || !a_star_ || !validControlPoints(init_points, order_) ||
        !std::isfinite(grid_map_->getResolution()) || grid_map_->getResolution() <= 0.0)
    {
      cps_.resize(0);
      return {};
    }
    const bool need_resize = flag_first_init || static_cast<int>(cps_.base_point.size()) != init_points.cols();
    if (need_resize)
    {
      cps_.clearance = dist0_;
      cps_.resize(init_points.cols());
    }
    else
    {
      // Re-initialization path: clear cached collision guides to avoid unbounded growth.
      for (int i = 0; i < init_points.cols(); ++i)
      {
        cps_.base_point[i].clear();
        cps_.direction[i].clear();
        cps_.flag_temp[i] = false;
      }
    }
    cps_.points = init_points;

    /*** Segment the initial trajectory according to obstacles ***/
    constexpr int ENOUGH_INTERVAL = 2;
    const double seg_norm = (init_points.col(0) - init_points.col(init_points.cols() - 1)).norm();
    const double denom = (seg_norm / static_cast<double>(init_points.cols() - 1)) / 2.0;
    double step_size = denom > 1e-12 ? grid_map_->getResolution() / denom : 0.05;
    step_size = std::clamp(step_size, 1e-6, 1.0);
    int in_id = -1, out_id = -1;
    std::vector<std::pair<int, int>> segment_ids;
    int same_occ_state_times = ENOUGH_INTERVAL + 1;
    bool occ = false, last_occ = false;
    bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;
    const int i_end = static_cast<int>(init_points.cols()) - order_ -
                      (static_cast<int>(init_points.cols()) - 2 * order_) / 3;
    constexpr int kMaxSegmentAlphaIter = 100000;  // 防呆：防止 for(a-=step) 死循环
    for (int i = order_; i <= i_end; ++i)
    {
      int alpha_iter = 0;
      for (double a = 1.0; a >= 0.0 && alpha_iter < kMaxSegmentAlphaIter; a -= step_size, ++alpha_iter)
      {
        occ = grid_map_->getInflateOccupancy(a * init_points.col(i - 1) + (1 - a) * init_points.col(i));
        if (occ && !last_occ)
        {
          if (same_occ_state_times > ENOUGH_INTERVAL || i == order_)
          {
            in_id = i - 1;
            flag_got_start = true;
          }
          same_occ_state_times = 0;
          flag_got_end_maybe = false; // terminate in advance
        }
        else if (!occ && last_occ)
        {
          out_id = i;
          flag_got_end_maybe = true;
          same_occ_state_times = 0;
        }
        else
        {
          ++same_occ_state_times;
        }

        if (flag_got_end_maybe && (same_occ_state_times > ENOUGH_INTERVAL || (i == (int)init_points.cols() - order_)))
        {
          flag_got_end_maybe = false;
          flag_got_end = true;
        }

        last_occ = occ;

        if (flag_got_start && flag_got_end)
        {
          flag_got_start = false;
          flag_got_end = false;
          if (in_id >= 0 && out_id >= 0 && in_id < out_id)
            segment_ids.push_back(std::pair<int, int>(in_id, out_id));
        }
      }
    }

    /*** a star search ***/
    std::vector<std::vector<Eigen::Vector2d>> a_star_pathes;
    for (size_t i = 0; i < segment_ids.size(); ++i)
    {
      Eigen::Vector2d in(init_points.col(segment_ids[i].first)), out(init_points.col(segment_ids[i].second));
      if (a_star_->AstarSearch(/*(in-out).norm()/10+0.05*/ 0.1, in, out))
      {
        const std::vector<Eigen::Vector2d> path = a_star_->getPath();
        const bool path_is_finite = std::all_of(path.begin(), path.end(),
          [](const Eigen::Vector2d &point) { return point.allFinite(); });
        if (path.size() < 2 || !path_is_finite)
        {
          cps_.resize(0);
          return {};
        }
        a_star_pathes.push_back(path);
      }
      else
      {
        cps_.resize(0);
        return {};
      }
    }

    /*** calculate bounds ***/
    int id_low_bound, id_up_bound;
    std::vector<std::pair<int, int>> bounds(segment_ids.size());
    for (size_t i = 0; i < segment_ids.size(); i++)
    {

      if (i == 0) // first segment
      {
        id_low_bound = order_;
        if (segment_ids.size() > 1)
        {
          id_up_bound = (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0f) / 2); // id_up_bound : -1.0f fix()
        }
        else
        {
          id_up_bound = init_points.cols() - order_ - 1;
        }
      }
      else if (i == segment_ids.size() - 1) // last segment, i != 0 here
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = init_points.cols() - order_ - 1;
      }
      else
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0f) / 2);  // id_up_bound : -1.0f fix()
      }

      bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound);
    }

    /*** Adjust segment length ***/
    std::vector<std::pair<int, int>> final_segment_ids(segment_ids.size());
    constexpr double MINIMUM_PERCENT = 0.0; // Each segment is guaranteed to have sufficient points to generate sufficient thrust
    const int minimum_points = static_cast<int>(std::round(init_points.cols() * MINIMUM_PERCENT));
    int num_points = 0;
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      /*** Adjust segment length ***/
      num_points = segment_ids[i].second - segment_ids[i].first + 1;
      if (num_points < minimum_points)
      {
        double add_points_each_side = (int)(((minimum_points - num_points) + 1.0f) / 2);

        final_segment_ids[i].first = segment_ids[i].first - add_points_each_side >= bounds[i].first ? segment_ids[i].first - add_points_each_side : bounds[i].first;

        final_segment_ids[i].second = segment_ids[i].second + add_points_each_side <= bounds[i].second ? segment_ids[i].second + add_points_each_side : bounds[i].second;
      }
      else
      {
        final_segment_ids[i].first = segment_ids[i].first;
        final_segment_ids[i].second = segment_ids[i].second;
      }
    }

    /*** Assign data to each segment ***/
    constexpr int kMaxAstarBisect = 10000;  // 防呆：防止二分查找死循环（全段共用）
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      // step 1
      for (int j = final_segment_ids[i].first; j <= final_segment_ids[i].second; ++j)
        cps_.flag_temp[j] = false;

      // step 2
      int got_intersection_id = -1;
      for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
      {
        Eigen::Vector2d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
        if (ctrl_pts_law.squaredNorm() <= 1e-12)
        {
          continue;
        }
        const int path_size = static_cast<int>(a_star_pathes[i].size());
        int Astar_id = path_size / 2;
        int last_Astar_id = Astar_id;
        double val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), last_val = val;
        bool found_intersection = false;
        int astar_iter = 0;
        while (Astar_id >= 0 && Astar_id < path_size)
        {
          if (++astar_iter > kMaxAstarBisect) break;  // 防呆
          last_Astar_id = Astar_id;
          last_val = val;

          if (val >= 0)
            --Astar_id;
          else
            ++Astar_id;

          if (Astar_id < 0 || Astar_id >= path_size)
            break;

          val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

          if (val * last_val <= 0 && (std::abs(val) > 0 || std::abs(last_val) > 0))
          {
            const double denom_t = ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]);
            if (std::abs(denom_t) < 1e-12) break;  // 防呆：避免除零
            intersection_point =
                a_star_pathes[i][Astar_id] +
                ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(cps_.points.col(j) - a_star_pathes[i][Astar_id]) / denom_t));

            found_intersection = intersection_point.allFinite();
            break;
          }
        }

        if (found_intersection)
        {
          const double length = (intersection_point - cps_.points.col(j)).norm();
          if (length > 1e-5)
          {
            const double res = std::max(grid_map_->getResolution(), 1e-6);  // 防呆：避免除零或步长为0导致死循环
            constexpr int kMaxInflateIter = 100000;
            int inflate_iter = 0;
            for (double a = length; a >= 0.0 && inflate_iter < kMaxInflateIter; a -= res, ++inflate_iter)
            {
              occ = grid_map_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));

              if (occ || a < res)
              {
                if (occ)
                  a += res;
                cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
                cps_.flag_temp[j] = true;
                got_intersection_id = j;
                break;
              }
            }
          }
        }
      }

      /* Corner case: the segment length is too short. Here the control points may outside the A* path, leading to opposite gradient direction. So I have to take special care of it */
      if (segment_ids[i].second - segment_ids[i].first == 1)
      {
        Eigen::Vector2d ctrl_pts_law(cps_.points.col(segment_ids[i].second) - cps_.points.col(segment_ids[i].first)), intersection_point;
        Eigen::Vector2d middle_point = (cps_.points.col(segment_ids[i].second) + cps_.points.col(segment_ids[i].first)) / 2;
        if (ctrl_pts_law.squaredNorm() <= 1e-12)
        {
          continue;
        }
        const int path_size = static_cast<int>(a_star_pathes[i].size());
        int Astar_id = path_size / 2;
        int last_Astar_id = Astar_id;
        double val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law), last_val = val;
        int astar_iter2 = 0;
        while (Astar_id >= 0 && Astar_id < path_size)
        {
          if (++astar_iter2 > kMaxAstarBisect) break;  // 防呆
          last_Astar_id = Astar_id;
          last_val = val;

          if (val >= 0)
            --Astar_id;
          else
            ++Astar_id;

          if (Astar_id < 0 || Astar_id >= path_size)
            break;

          val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law);

          if (val * last_val <= 0 && (std::abs(val) > 0 || std::abs(last_val) > 0))
          {
            const double denom_t2 = ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]);
            if (std::abs(denom_t2) < 1e-12) break;  // 防呆：避免除零
            intersection_point =
                a_star_pathes[i][Astar_id] +
                ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(middle_point - a_star_pathes[i][Astar_id]) / denom_t2));

            if ((intersection_point - middle_point).norm() > 0.01) // 1cm.
            {
              cps_.flag_temp[segment_ids[i].first] = true;
              cps_.base_point[segment_ids[i].first].push_back(cps_.points.col(segment_ids[i].first));
              cps_.direction[segment_ids[i].first].push_back((intersection_point - middle_point).normalized());

              got_intersection_id = segment_ids[i].first;
            }
            break;
          }
        }
      }

      //step 3
      if (got_intersection_id >= 0)
      {
        for (int j = got_intersection_id + 1; j <= final_segment_ids[i].second; ++j)
          if (!cps_.flag_temp[j])
          {
            if (cps_.base_point[j - 1].empty() || cps_.direction[j - 1].empty())
            {
              continue;
            }
            cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
            cps_.direction[j].push_back(cps_.direction[j - 1].back());
          }

        for (int j = got_intersection_id - 1; j >= final_segment_ids[i].first; --j)
          if (!cps_.flag_temp[j])
          {
            if (cps_.base_point[j + 1].empty() || cps_.direction[j + 1].empty())
            {
              continue;
            }
            cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
            cps_.direction[j].push_back(cps_.direction[j + 1].back());
          }
      }
    }

    rebound_control_points_ready_ = true;
    return a_star_pathes;
}
int BsplineOptimizer::earlyExit(void *func_data, const double *, const double *, const double,
                                const double, const double, const double, int, int, int)
{
  BsplineOptimizer *opt = static_cast<BsplineOptimizer *>(func_data);
  if (opt == nullptr)
  {
    return 1;
  }
  return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
}

double BsplineOptimizer::costFunctionRebound(void *func_data, const double *x, double *grad, const int n)
{
  BsplineOptimizer *opt = static_cast<BsplineOptimizer *>(func_data);
  if (opt == nullptr)
  {
    return std::numeric_limits<double>::max() / 1024.0;
  }
  double cost = std::numeric_limits<double>::max() / 1024.0;
  opt->combineCostRebound(x, grad, cost, n);
  opt->iter_num_ += 1;
  return cost;
}

double BsplineOptimizer::costFunctionRefine(void *func_data, const double *x, double *grad, const int n)
{
  BsplineOptimizer *opt = static_cast<BsplineOptimizer *>(func_data);
  if (opt == nullptr)
  {
    return std::numeric_limits<double>::max() / 1024.0;
  }
  double cost = std::numeric_limits<double>::max() / 1024.0;
  opt->combineCostRefine(x, grad, cost, n);
  opt->iter_num_ += 1;
  return cost;
}

void BsplineOptimizer::calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost,
                                                 Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost)
{
    cost = 0.0;
    gradient.setZero(2, q.cols());
    if (q.rows() != 2 || !q.allFinite() || cps_.size != q.cols() ||
        cps_.base_point.size() != static_cast<std::size_t>(cps_.size) ||
        cps_.direction.size() != static_cast<std::size_t>(cps_.size))
    {
      cost = std::numeric_limits<double>::max() / 1024.0;
      return;
    }
    const int end_idx = static_cast<int>(q.cols()) - order_;
    const double demarcation = cps_.clearance;
    const double a = 3.0 * demarcation;
    const double b = -3.0 * demarcation * demarcation;
    const double c = demarcation * demarcation * demarcation;

    force_stop_type_ = DONT_STOP;
    const int optimized_points = cps_.size - 2 * order_;
    if (iter_num > 3 && optimized_points > 0 && smoothness_cost / optimized_points < 0.1)
    {
      check_collision_and_rebound();
    }

    /*** calculate distance cost and gradient ***/
    for (auto i = order_; i < end_idx; ++i)
    {
      const std::size_t guide_count = std::min(cps_.direction[i].size(), cps_.base_point[i].size());
      for (std::size_t j = 0; j < guide_count; ++j)
      {
        double dist = (cps_.points.col(i) - cps_.base_point[i][j]).dot(cps_.direction[i][j]);
        double dist_err = cps_.clearance - dist;
        Eigen::Vector2d dist_grad = cps_.direction[i][j];

        if (dist_err < 0)
        {
          /* do nothing */
        }
        else if (dist_err < demarcation)
        {
          cost += dist_err * dist_err * dist_err;
          gradient.col(i) += -3.0 * dist_err * dist_err * dist_grad;
        }
        else
        {
          cost += a * dist_err * dist_err + b * dist_err + c;
          gradient.col(i) += -(2.0 * a * dist_err + b) * dist_grad;
        }
      }
    }
}

void BsplineOptimizer::calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
{
  cost = 0.0;
  gradient.setZero(2, q.cols());
  const int end_idx = static_cast<int>(q.cols()) - order_;
  if (q.rows() != 2 || end_idx < order_ || static_cast<int>(ref_pts_.size()) <= end_idx)
  {
    cost = std::numeric_limits<double>::max() / 1024.0;
    return;
  }
  constexpr double a2 = 25.0;
  constexpr double b2 = 1.0;

  for (auto i = order_ - 1; i < end_idx + 1; ++i)
  {
    // 2D 控制点加权平均
    Eigen::Vector2d x = (q.col(i - 1) + 4 * q.col(i) + q.col(i + 1)) / 6.0 - ref_pts_[i - 1];
    Eigen::Vector2d dv = ref_pts_[i] - ref_pts_[i - 2];
    Eigen::Vector2d v = (dv.norm() > 1e-12) ? dv.normalized() : Eigen::Vector2d(1, 0);  // 防呆：零向量 normalized() 会 NaN

    double xdotv = x.dot(v);
    double xcrossv_mag = x(0)*v(1) - x(1)*v(0); // 2D 叉乘（大小）
    double f = xdotv*xdotv / a2 + xcrossv_mag*xcrossv_mag / b2;
    cost += f;

    // 2D 梯度计算
    Eigen::Vector2d df_dx;
    df_dx(0) = 2 * xdotv * v(0) / a2 + 2 * xcrossv_mag * v(1) / b2;
    df_dx(1) = 2 * xdotv * v(1) / a2 - 2 * xcrossv_mag * v(0) / b2;

    // 仅更新 x、y 梯度
    gradient.col(i - 1) += df_dx / 6;
    gradient.col(i) += 4 * df_dx / 6;
    gradient.col(i + 1)+= df_dx / 6;
  }
}

void BsplineOptimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                          Eigen::MatrixXd &gradient, bool use_jerk)
{
  cost = 0.0;
  gradient.setZero(2, q.cols()); // 梯度矩阵改为 2 行（x、y）
  if (q.rows() != 2 || !q.allFinite())
  {
    cost = std::numeric_limits<double>::max() / 1024.0;
    return;
  }

  if (use_jerk)
  {
    Eigen::Vector2d jerk, temp_j;
    for (int i = 0; i < q.cols() - 3; i++)
    {
      // 2D jerk 计算
      jerk = q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i);
      cost += jerk.squaredNorm();
      temp_j = 2.0 * jerk;

      // 仅更新 x、y 梯度
      gradient.col(i + 0) += -temp_j;
      gradient.col(i + 1) += 3.0 * temp_j;
      gradient.col(i + 2) += -3.0 * temp_j;
      gradient.col(i + 3) += temp_j;
    }
  }
  else
  {
    Eigen::Vector2d acc, temp_acc;
    for (int i = 0; i < q.cols() - 2; i++)
    {
      // 2D acc 计算
      acc = q.col(i + 2) - 2 * q.col(i + 1) + q.col(i);
      cost += acc.squaredNorm();
      temp_acc = 2.0 * acc;

      // 仅更新 x、y 梯度
      gradient.col(i + 0) += temp_acc;
      gradient.col(i + 1) += -2.0 * temp_acc;
      gradient.col(i + 2) += temp_acc;
    }
  }
}


void BsplineOptimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                           Eigen::MatrixXd &gradient)
{
  cost = 0.0;
  gradient.setZero(2, q.cols());
  if (q.rows() != 2 || !q.allFinite() || !validTimeStep(bspline_interval_))
  {
    cost = std::numeric_limits<double>::max() / 1024.0;
    return;
  }

  const double inverse_time = 1.0 / bspline_interval_;
  const double inverse_time_squared = inverse_time * inverse_time;

  for (int i = 0; i < q.cols() - 1; ++i)
  {
    const Eigen::Vector2d velocity = (q.col(i + 1) - q.col(i)) * inverse_time;
    for (int axis = 0; axis < 2; ++axis)
    {
      const double excess = std::abs(velocity(axis)) - max_vel_;
      if (excess <= 0.0)
      {
        continue;
      }
      cost += excess * excess * inverse_time_squared;
      const double signed_gradient = std::copysign(2.0 * excess * inverse_time * inverse_time_squared,
                                                   velocity(axis));
      gradient(axis, i) -= signed_gradient;
      gradient(axis, i + 1) += signed_gradient;
    }
  }

  for (int i = 0; i < q.cols() - 2; ++i)
  {
    const Eigen::Vector2d acceleration =
      (q.col(i + 2) - 2.0 * q.col(i + 1) + q.col(i)) * inverse_time_squared;
    for (int axis = 0; axis < 2; ++axis)
    {
      const double excess = std::abs(acceleration(axis)) - max_acc_;
      if (excess <= 0.0)
      {
        continue;
      }
      cost += excess * excess;
      const double signed_gradient = std::copysign(2.0 * excess * inverse_time_squared,
                                                   acceleration(axis));
      gradient(axis, i) += signed_gradient;
      gradient(axis, i + 1) -= 2.0 * signed_gradient;
      gradient(axis, i + 2) += signed_gradient;
    }
  }
}


bool BsplineOptimizer::check_collision_and_rebound(void)
{
  if (!grid_map_ || !a_star_ || !validControlPoints(cps_.points, order_) ||
      cps_.size != cps_.points.cols())
  {
    force_stop_type_ = STOP_FOR_ERROR;
    return false;
  }

  const int end_idx = cps_.size - order_;
  const int check_end = end_idx - (end_idx - order_) / 3;
  bool new_collision = false;
  for (int i = order_ - 1; i <= check_end; ++i)
  {
    bool occupied = grid_map_->getInflateOccupancy(cps_.points.col(i));
    if (occupied)
    {
      for (std::size_t guide = 0; guide < cps_.direction[i].size(); ++guide)
      {
        if ((cps_.points.col(i) - cps_.base_point[i][guide]).dot(cps_.direction[i][guide]) <
            grid_map_->getResolution())
        {
          occupied = false;
          break;
        }
      }
    }
    if (occupied)
    {
      new_collision = true;
      break;
    }
  }

  if (!new_collision)
  {
    return false;
  }

  (void)initControlPoints(cps_.points, false);
  if (!rebound_control_points_ready_)
  {
    force_stop_type_ = STOP_FOR_ERROR;
    return false;
  }

  force_stop_type_ = STOP_FOR_REBOUND;
  return true;
}

bool BsplineOptimizer::BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts)
{
  optimal_points.resize(0, 0);
  setBsplineInterval(ts);
  if (!rebound_control_points_ready_ || !grid_map_ || !a_star_ ||
      !validControlPoints(cps_.points, order_) ||
      cps_.size != cps_.points.cols() || !validTimeStep(bspline_interval_))
  {
    return false;
  }
  const bool success = rebound_optimize();
  if (!cps_.points.allFinite())
  {
    return false;
  }
  optimal_points = cps_.points;
  return success;
}

bool BsplineOptimizer::BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points)
{
  optimal_points = init_points;
  if (!grid_map_ || !validControlPoints(init_points, order_) || !validTimeStep(ts))
  {
    return false;
  }
  setControlPoints(init_points);
  setBsplineInterval(ts);
  const bool success = refine_optimize();
  if (!success || cps_.points.rows() != init_points.rows() || cps_.points.cols() != init_points.cols() ||
      !cps_.points.allFinite())
  {
    cps_.points = init_points;
    return false;
  }
  optimal_points = cps_.points;
  return true;
}

Eigen::MatrixXd BsplineOptimizer::convert2DTo3D(const Eigen::MatrixXd& points_2d) const 
{
  if (points_2d.rows() != 2 || points_2d.cols() <= 0 || !points_2d.allFinite()) {
    return Eigen::MatrixXd();
  }

  const int num_cols = static_cast<int>(points_2d.cols());

  Eigen::MatrixXd points_3d(3, num_cols);

  points_3d.topRows(2) = points_2d;

  points_3d.bottomRows(1).setZero();

  return points_3d;
}

bool BsplineOptimizer::rebound_optimize()
{
  const auto optimization_start = std::chrono::steady_clock::now();
  const auto record_elapsed = [this, &optimization_start]() {
    last_rebound_optimize_ms_ = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - optimization_start).count();
  };

  if (!grid_map_ || !a_star_ || !validControlPoints(cps_.points, order_) ||
      cps_.size != cps_.points.cols() || !validTimeStep(bspline_interval_))
  {
    record_elapsed();
    return false;
  }

  iter_num_ = 0;
  const int start_id = order_;
  const int end_id = static_cast<int>(cps_.points.cols()) - order_;
  variable_num_ = 2 * (end_id - start_id);
  if (variable_num_ <= 0 || 2 * start_id + variable_num_ > cps_.points.size())
  {
    record_elapsed();
    return false;
  }

  int restart_nums = 0, rebound_times = 0;
  bool flag_force_return = false;
  bool flag_occ = false;
  bool success = false;
  new_lambda2_ = lambda2_;
  constexpr int kMaxRestartCount = 5;
  const int max_rebound_times = std::max(0, max_rebound_retries_);

  do
  {
    iter_num_ = 0;
    force_stop_type_ = DONT_STOP;
    flag_force_return = false;
    flag_occ = false;
    success = false;
    std::vector<double> q(static_cast<std::size_t>(variable_num_));
    std::memcpy(q.data(), cps_.points.data() + 2 * start_id,
                static_cast<std::size_t>(variable_num_) * sizeof(double));
    if (!finiteArray(q))
    {
      record_elapsed();
      return false;
    }

    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 16;
    lbfgs_params.max_iterations = 200;
    lbfgs_params.g_epsilon = 0.001;

    double final_cost = std::numeric_limits<double>::infinity();
    const int result = lbfgs::lbfgs_optimize(
      variable_num_, q.data(), &final_cost, BsplineOptimizer::costFunctionRebound, nullptr,
      BsplineOptimizer::earlyExit, this, &lbfgs_params);
    if (!finiteArray(q) || !std::isfinite(final_cost))
    {
      record_elapsed();
      return false;
    }
    std::memcpy(cps_.points.data() + 2 * start_id, q.data(),
                static_cast<std::size_t>(variable_num_) * sizeof(double));

    if (acceptedLbfgsResult(result))
    {
      const UniformBspline trajectory(cps_.points, order_, bspline_interval_);
      const double duration = trajectory.getTimeSum();
      const double resolution = std::max(grid_map_->getResolution(), 1e-6);
      const double time_step = std::max(1e-6, std::min(bspline_interval_ / 5.0,
                                                       resolution / std::max(max_vel_, 1e-6)));
      for (double time = 0.0; time < duration * 2.0 / 3.0; time += time_step)
      {
        const Eigen::Vector2d point = trajectory.evaluateDeBoorT(time);
        flag_occ = grid_map_->getInflateOccupancy(point);
        if (flag_occ)
        {
          if (time <= bspline_interval_)
          {
            record_elapsed();
            return false;
          }
          break;
        }
      }

      if (!flag_occ)
      {
        success = true;
      }
      else
      {
        restart_nums++;
        (void)initControlPoints(cps_.points, false);
        if (!rebound_control_points_ready_)
        {
          record_elapsed();
          return false;
        }
        new_lambda2_ = std::min(new_lambda2_ * 2.0, 1e6);
      }
    }
    else if (result == lbfgs::LBFGSERR_CANCELED && force_stop_type_ == STOP_FOR_REBOUND)
    {
      flag_force_return = true;
      rebound_times++;
      if (rebound_times >= max_rebound_times)
      {
        flag_force_return = false;
      }
    }
    else
    {
      record_elapsed();
      return false;
    }

  } while ((flag_occ && restart_nums < kMaxRestartCount) ||
           (flag_force_return && force_stop_type_ == STOP_FOR_REBOUND && rebound_times < max_rebound_times));

  record_elapsed();
  return success;
}

bool BsplineOptimizer::refine_optimize()
{
  if (!grid_map_ || !validControlPoints(cps_.points, order_) || !validTimeStep(bspline_interval_))
  {
    return false;
  }

  iter_num_ = 0;
  const int start_id = order_;
  const int end_id = static_cast<int>(cps_.points.cols()) - order_;
  variable_num_ = 2 * (end_id - start_id);
  const bool reference_points_finite = std::all_of(
    ref_pts_.begin(), ref_pts_.end(), [](const Eigen::Vector2d &point) { return point.allFinite(); });
  if (variable_num_ <= 0 || 2 * start_id + variable_num_ > cps_.points.size() ||
      static_cast<int>(ref_pts_.size()) <= end_id || !reference_points_finite)
  {
    return false;
  }

  std::vector<double> q(static_cast<std::size_t>(variable_num_));
  std::memcpy(q.data(), cps_.points.data() + 2 * start_id,
              static_cast<std::size_t>(variable_num_) * sizeof(double));
  if (!finiteArray(q))
  {
    return false;
  }

  const double origin_lambda4 = lambda4_;
  constexpr int kMaxRefineAttempts = 2;

  for (int attempt = 0; attempt < kMaxRefineAttempts; ++attempt)
  {
    iter_num_ = 0;
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 16;
    lbfgs_params.max_iterations = 200;
    lbfgs_params.g_epsilon = 0.001;

    double final_cost = std::numeric_limits<double>::infinity();
    const int result = lbfgs::lbfgs_optimize(
      variable_num_, q.data(), &final_cost, BsplineOptimizer::costFunctionRefine, nullptr, nullptr,
      this, &lbfgs_params);
    if (!acceptedLbfgsResult(result) || !finiteArray(q) || !std::isfinite(final_cost))
    {
      lambda4_ = origin_lambda4;
      return false;
    }
    std::memcpy(cps_.points.data() + 2 * start_id, q.data(),
                static_cast<std::size_t>(variable_num_) * sizeof(double));

    const UniformBspline trajectory(cps_.points, order_, bspline_interval_);
    const double duration = trajectory.getTimeSum();
    const double resolution = std::max(grid_map_->getResolution(), 1e-6);
    const double time_step = std::max(1e-6, std::min(bspline_interval_ / 5.0,
                                                     resolution / std::max(max_vel_, 1e-6)));
    bool safe = true;
    for (double time = 0.0; time < duration * 2.0 / 3.0; time += time_step)
    {
      const Eigen::Vector2d point = trajectory.evaluateDeBoorT(time);
      if (grid_map_->getInflateOccupancy(point))
      {
        safe = false;
        break;
      }
    }

    if (safe)
    {
      lambda4_ = origin_lambda4;
      return true;
    }
    lambda4_ = std::min(lambda4_ * 2.0, 1e6);
  }

  lambda4_ = origin_lambda4;
  return false;
}

void BsplineOptimizer::combineCostRebound(const double *x, double *grad, double &f_combine, const int n)
{
  f_combine = std::numeric_limits<double>::max() / 1024.0;
  if (grad != nullptr && n > 0)
  {
    std::fill_n(grad, n, 0.0);
  }
  if (x == nullptr || grad == nullptr || n <= 0 || n != variable_num_ ||
      2 * order_ + n > cps_.points.size() || !validControlPoints(cps_.points, order_))
  {
    return;
  }
  std::memcpy(cps_.points.data() + 2 * order_, x, static_cast<std::size_t>(n) * sizeof(double));
  if (!cps_.points.allFinite())
  {
    return;
  }

  /* 计算各代价（均为 2D）*/
  double f_smoothness = 0.0, f_distance = 0.0, f_feasibility = 0.0;
  Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(2, cps_.points.cols()); // 2D 梯度
  Eigen::MatrixXd g_distance = Eigen::MatrixXd::Zero(2, cps_.points.cols());
  Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(2, cps_.points.cols());

  calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
  calcDistanceCostRebound(cps_.points, f_distance, g_distance, iter_num_, f_smoothness);
  calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);

  f_combine = lambda1_ * f_smoothness + new_lambda2_ * f_distance + lambda3_ * f_feasibility;
  const Eigen::MatrixXd gradient_2d =
    lambda1_ * g_smoothness + new_lambda2_ * g_distance + lambda3_ * g_feasibility;
  if (!std::isfinite(f_combine) || !gradient_2d.allFinite())
  {
    f_combine = std::numeric_limits<double>::max() / 1024.0;
    return;
  }
  std::memcpy(grad, gradient_2d.data() + 2 * order_, static_cast<std::size_t>(n) * sizeof(double));
}

void BsplineOptimizer::combineCostRefine(const double *x, double *grad, double &f_combine, const int n)
{
  f_combine = std::numeric_limits<double>::max() / 1024.0;
  if (grad != nullptr && n > 0)
  {
    std::fill_n(grad, n, 0.0);
  }
  if (x == nullptr || grad == nullptr || n <= 0 || n != variable_num_ ||
      2 * order_ + n > cps_.points.size() || !validControlPoints(cps_.points, order_))
  {
    return;
  }
  std::memcpy(cps_.points.data() + 2 * order_, x, static_cast<std::size_t>(n) * sizeof(double));
  if (!cps_.points.allFinite())
  {
    return;
  }
  /* 计算各代价*/
  double f_smoothness = 0.0, f_fitness = 0.0, f_feasibility = 0.0;
  Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(2, cps_.points.cols()); // 2D 梯度矩阵
  Eigen::MatrixXd g_fitness = Eigen::MatrixXd::Zero(2, cps_.points.cols());
  Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(2, cps_.points.cols());

  calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
  calcFitnessCost(cps_.points, f_fitness, g_fitness);
  calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);

  f_combine = lambda1_ * f_smoothness + lambda4_ * f_fitness + lambda3_ * f_feasibility;
  const Eigen::MatrixXd gradient_2d =
    lambda1_ * g_smoothness + lambda4_ * g_fitness + lambda3_ * g_feasibility;
  if (!std::isfinite(f_combine) || !gradient_2d.allFinite())
  {
    f_combine = std::numeric_limits<double>::max() / 1024.0;
    return;
  }
  std::memcpy(grad, gradient_2d.data() + 2 * order_, static_cast<std::size_t>(n) * sizeof(double));
}

} // namespace ego_planner
