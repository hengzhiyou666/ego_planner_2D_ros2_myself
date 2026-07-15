// Copyright 2026 hengzhiyou
// SPDX-License-Identifier: GPL-3.0-only

#include <cmath>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_manage/plan_container.hpp>

#include "mapping/grid_map.hpp"
#include "planning/a_star.hpp"
#include "planning/planner_interface_dog.hpp"

TEST(UniformBspline, DefaultConstructedObjectsHaveSafeState)
{
  const ego_planner::UniformBspline spline;
  EXPECT_FALSE(spline.isValid());
  EXPECT_DOUBLE_EQ(spline.getTimeSum(), -1.0);
  EXPECT_FALSE(spline.getDerivative().isValid());
  EXPECT_THROW(spline.evaluateDeBoorT(0.0), std::logic_error);

  const ego_planner::PlanParameters parameters;
  EXPECT_DOUBLE_EQ(parameters.max_vel_, 0.0);
  EXPECT_DOUBLE_EQ(parameters.max_jerk_, 0.0);

  const ego_planner::LocalTrajData trajectory;
  EXPECT_EQ(trajectory.traj_id_, 0);
  EXPECT_DOUBLE_EQ(trajectory.duration_, 0.0);
  EXPECT_TRUE(trajectory.start_pos_.isZero());
}

TEST(UniformBspline, TwoDimensionalFeasibilityCheckIsDimensionSafe)
{
  Eigen::MatrixXd control_points(2, 7);
  for (int index = 0; index < control_points.cols(); ++index) {
    control_points.col(index) << 0.1 * index, 0.0;
  }

  ego_planner::UniformBspline spline(control_points, 3, 1.0);
  spline.setPhysicalLimits(1.0, 1.0, 0.0, 1.0);

  double ratio = 0.0;
  EXPECT_TRUE(spline.checkFeasibility(ratio));
  EXPECT_DOUBLE_EQ(ratio, 1.0);
}

TEST(UniformBspline, JerkLimitProducesAUsableTimeScalingRatio)
{
  Eigen::MatrixXd control_points = Eigen::MatrixXd::Zero(2, 7);
  control_points.row(0) << 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0;

  ego_planner::UniformBspline spline(control_points, 3, 0.1);
  spline.setPhysicalLimits(1000.0, 1000.0, 0.0, 1.0);

  double ratio = 1.0;
  ASSERT_FALSE(spline.checkFeasibility(ratio));
  ASSERT_TRUE(std::isfinite(ratio));
  ASSERT_GT(ratio, 1.0);

  const double original_duration = spline.getTimeSum();
  spline.lengthenTime(ratio);
  EXPECT_NEAR(spline.getTimeSum(), original_duration * ratio, 1e-9);

  double final_ratio = 0.0;
  EXPECT_TRUE(spline.checkFeasibility(final_ratio));
  EXPECT_DOUBLE_EQ(final_ratio, 1.0);
}

TEST(UniformBspline, PlanarVelocityLimitUsesVectorMagnitude)
{
  Eigen::MatrixXd control_points(2, 7);
  for (int index = 0; index < control_points.cols(); ++index) {
    control_points.col(index) << 0.8 * index, 0.8 * index;
  }
  ego_planner::UniformBspline spline(control_points, 3, 1.0);
  spline.setPhysicalLimits(1.0, 100.0, 0.0, 100.0);

  double ratio = 1.0;
  EXPECT_FALSE(spline.checkFeasibility(ratio));
  EXPECT_TRUE(std::isfinite(ratio));
  EXPECT_GT(ratio, 1.0);
}

TEST(UniformBspline, LengthSamplingIncludesFinalPartialInterval)
{
  Eigen::MatrixXd control_points(2, 7);
  for (int index = 0; index < control_points.cols(); ++index) {
    control_points.col(index) << 0.2 * index, 0.0;
  }
  const ego_planner::UniformBspline spline(control_points, 3, 0.2);
  const double duration = spline.getTimeSum();
  const double endpoint_distance =
    (spline.evaluateDeBoorT(duration) - spline.evaluateDeBoorT(0.0)).norm();

  EXPECT_NEAR(spline.getLength(duration * 2.0), endpoint_distance, 1e-9);
}

TEST(UniformBspline, InvalidParameterizationClearsPreviousOutput)
{
  Eigen::MatrixXd control_points = Eigen::MatrixXd::Ones(3, 4);
  const std::vector<Eigen::Vector3d> too_few_points{
    Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX()};
  const std::vector<Eigen::Vector3d> derivatives(4, Eigen::Vector3d::Zero());

  ego_planner::UniformBspline::parameterizeToBspline(
    0.1, too_few_points, derivatives, control_points);
  EXPECT_EQ(control_points.size(), 0);
}

TEST(UniformBspline, ParameterizationReturnsFiniteControlPoints)
{
  const std::vector<Eigen::Vector3d> points{
    {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.5, 0.0}, {3.0, 1.0, 0.0}};
  const std::vector<Eigen::Vector3d> derivatives(4, Eigen::Vector3d::Zero());
  Eigen::MatrixXd control_points;

  ego_planner::UniformBspline::parameterizeToBspline(
    0.5, points, derivatives, control_points);

  EXPECT_EQ(control_points.rows(), 3);
  EXPECT_EQ(control_points.cols(), 6);
  EXPECT_TRUE(control_points.allFinite());
}

TEST(BsplineOptimizer, FreeSpaceReboundAndRefineReturnFiniteControlPoints)
{
  auto map = std::make_shared<GridMap2D>(0.1, Eigen::Vector2i(4, 4));
  map->setInflateRadius(0.0);

  ego_planner::BsplineOptimizer optimizer;
  optimizer.setEnvironment(map);
  optimizer.setParam(1.0, 2.0, 0.2);
  optimizer.a_star_ = std::make_shared<AStar>();
  optimizer.a_star_->initGridMap(map, Eigen::Vector2i(50, 50));

  Eigen::MatrixXd control_points(2, 8);
  for (int index = 0; index < control_points.cols(); ++index) {
    control_points.col(index) << -0.7 + 0.2 * index, 0.0;
  }

  optimizer.initControlPoints(control_points, true);
  Eigen::MatrixXd rebound_output;
  ASSERT_TRUE(optimizer.BsplineOptimizeTrajRebound(rebound_output, 0.5));
  ASSERT_EQ(rebound_output.rows(), 2);
  ASSERT_EQ(rebound_output.cols(), control_points.cols());
  ASSERT_TRUE(rebound_output.allFinite());

  const ego_planner::UniformBspline trajectory(rebound_output, 3, 0.5);
  optimizer.ref_pts_.clear();
  const int segment_count = static_cast<int>(rebound_output.cols()) - 3;
  for (int sample = 0; sample <= segment_count; ++sample) {
    const double time = trajectory.getTimeSum() * sample / segment_count;
    optimizer.ref_pts_.push_back(trajectory.evaluateDeBoorT(time));
  }

  Eigen::MatrixXd refine_output;
  ASSERT_TRUE(optimizer.BsplineOptimizeTrajRefine(rebound_output, 0.5, refine_output));
  EXPECT_EQ(refine_output.rows(), 2);
  EXPECT_EQ(refine_output.cols(), rebound_output.cols());
  EXPECT_TRUE(refine_output.allFinite());
}

TEST(BsplineOptimizer, InvalidInitializationCannotReusePreviousControlPoints)
{
  auto map = std::make_shared<GridMap2D>(0.1, Eigen::Vector2i(4, 4));
  map->setInflateRadius(0.0);

  ego_planner::BsplineOptimizer optimizer;
  optimizer.setEnvironment(map);
  optimizer.setParam(1.0, 2.0, 0.2);
  optimizer.a_star_ = std::make_shared<AStar>();
  optimizer.a_star_->initGridMap(map, Eigen::Vector2i(50, 50));

  Eigen::MatrixXd valid_points(2, 8);
  for (int index = 0; index < valid_points.cols(); ++index) {
    valid_points.col(index) << -0.7 + 0.2 * index, 0.0;
  }
  optimizer.initControlPoints(valid_points, true);
  Eigen::MatrixXd first_output;
  ASSERT_TRUE(optimizer.BsplineOptimizeTrajRebound(first_output, 0.5));

  Eigen::MatrixXd too_few_points = Eigen::MatrixXd::Zero(2, 6);
  optimizer.initControlPoints(too_few_points, true);
  Eigen::MatrixXd stale_output = Eigen::MatrixXd::Ones(2, 8);
  EXPECT_FALSE(optimizer.BsplineOptimizeTrajRebound(stale_output, 0.5));
  EXPECT_EQ(stale_output.size(), 0);
}

TEST(PlannerInterfaceDog, SampledTrajectoryIncludesEndpoint)
{
  dog_ego_planner::PlannerInterfaceDog planner;
  planner.initParam(2.0, 5.0, 20.0, 0.3);
  planner.initGridMap(10.0, 10.0, 0.1, Eigen::Vector2d(-5.0, -5.0), 0.2, 100);
  planner.setCurrentPose({0.0, 0.0});
  planner.setObstacles({});
  const std::vector<dog_ego_planner::PathPoint2D> reference{
    {0.0, 0.0}, {0.3, 0.0}, {0.6, 0.0}, {0.9, 0.0},
    {1.2, 0.0}, {1.5, 0.0}, {1.8, 0.0}, {2.1, 0.0}};
  planner.setReferencePath(reference);

  ASSERT_TRUE(
    planner.makePlan(
      Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()));
  std::vector<dog_ego_planner::PathPoint2D> trajectory_17;
  std::vector<dog_ego_planner::PathPoint2D> trajectory_19;
  planner.getPlannedTraj(trajectory_17, 0.17);
  planner.getPlannedTraj(trajectory_19, 0.19);

  ASSERT_GE(trajectory_17.size(), 2U);
  ASSERT_GE(trajectory_19.size(), 2U);
  EXPECT_NEAR(trajectory_17.back().x, trajectory_19.back().x, 1e-9);
  EXPECT_NEAR(trajectory_17.back().y, trajectory_19.back().y, 1e-9);
}

TEST(PlannerInterfaceDog, RejectsCollisionAtTrajectoryEndpoint)
{
  dog_ego_planner::PlannerInterfaceDog planner;
  planner.initParam(2.0, 5.0, 20.0, 0.3);
  planner.initGridMap(10.0, 10.0, 0.1, Eigen::Vector2d(-5.0, -5.0), 0.2, 100);
  planner.setCurrentPose({0.0, 0.0});
  planner.setObstacles({{2.1, 0.0}});
  planner.setReferencePath(
    {{0.0, 0.0}, {0.3, 0.0}, {0.6, 0.0}, {0.9, 0.0},
      {1.2, 0.0}, {1.5, 0.0}, {1.8, 0.0}, {2.1, 0.0}});

  EXPECT_FALSE(
    planner.makePlan(
      Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()));
}
