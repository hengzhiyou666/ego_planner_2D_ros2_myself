// Copyright (c) ZJU FAST Lab contributors
// Copyright 2026 hengzhiyou
// SPDX-License-Identifier: GPL-3.0-only

#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <bspline_opt/uniform_bspline.h>

namespace ego_planner
{


  struct PlanParameters
  {
    /* planning algorithm parameters */
    double max_vel_{0.0}, max_acc_{0.0}, max_jerk_{0.0}; // physical limits
    double ctrl_pt_dist{0.0};              // distance between adjacent B-spline control points
    double feasibility_tolerance_{0.0};    // permitted ratio of derivative limits

    /* processing time */
    double time_search_ = 0.0;
    double time_optimize_ = 0.0;
    double time_adjust_ = 0.0;
  };

  struct LocalTrajData
  {
    /* info of generated traj */

    int traj_id_{0};
    double duration_{0.0};
    double global_time_offset{0.0}; // Offset between local and global trajectory clocks.
    Eigen::Vector3d start_pos_{Eigen::Vector3d::Zero()};
    UniformBspline position_traj_, velocity_traj_, acceleration_traj_;
  };

} // namespace ego_planner

#endif
