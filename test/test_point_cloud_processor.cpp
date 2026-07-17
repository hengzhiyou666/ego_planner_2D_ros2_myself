// Copyright 2026 hengzhiyou
// SPDX-License-Identifier: GPL-3.0-only

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "node/point_cloud_processor.hpp"

namespace
{

sensor_msgs::msg::PointCloud2 makeCloud(
  const std::vector<Eigen::Vector3f> & points)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "lidar";
  cloud.height = 1U;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  for (const Eigen::Vector3f & point : points) {
    *x = point.x();
    *y = point.y();
    *z = point.z();
    ++x;
    ++y;
    ++z;
  }
  return cloud;
}

geometry_msgs::msg::TransformStamped makeTransform(double x, double y, double z)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "local_map_lidar_init_xyz";
  transform.child_frame_id = "lidar";
  transform.transform.translation.x = x;
  transform.transform.translation.y = y;
  transform.transform.translation.z = z;
  transform.transform.rotation.w = 1.0;
  return transform;
}

}  // namespace

TEST(PointCloudProcessor, TransformsAndAppliesHeightRangeAndFootprintFilters)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const sensor_msgs::msg::PointCloud2 cloud = makeCloud(
    {{0.0F, 0.0F, 0.5F}, {1.0F, 0.0F, 0.5F}, {1.0F, 0.0F, 2.0F},
      {20.0F, 0.0F, 0.5F}, {nan, 0.0F, 0.5F}});
  dog_ego_planner::PointCloudFilterConfig config;
  config.planning_frame = "local_map_lidar_init_xyz";
  config.maximum_x_distance_m = 10.0;
  config.maximum_y_distance_m = 10.0;

  const dog_ego_planner::PointCloudFilterResult result =
    dog_ego_planner::filterPointCloud(
    cloud, makeTransform(2.0, 3.0, 0.0), Eigen::Vector3d(2.0, 3.0, 0.0),
    0.0, config);

  ASSERT_EQ(result.input_points, 5U);
  ASSERT_EQ(result.valid_input_points, 4U);
  ASSERT_EQ(result.obstacles.size(), 1U);
  EXPECT_TRUE(result.obstacles.front().isApprox(Eigen::Vector2d(3.0, 3.0)));
  EXPECT_EQ(result.rejected_self, 1U);
  EXPECT_EQ(result.rejected_height, 1U);
  EXPECT_EQ(result.rejected_range, 1U);
  EXPECT_EQ(result.rejected_non_finite, 1U);
  EXPECT_EQ(result.filtered_cloud.header.frame_id, "local_map_lidar_init_xyz");
  EXPECT_EQ(result.filtered_cloud.width, 1U);
}

TEST(PointCloudProcessor, SelfFilterUsesRobotYawAndRectangularFootprint)
{
  const sensor_msgs::msg::PointCloud2 cloud = makeCloud(
    {{0.0F, 0.30F, 0.5F}, {0.30F, 0.0F, 0.5F}});
  dog_ego_planner::PointCloudFilterConfig config;
  const double half_pi = std::acos(-1.0) * 0.5;

  const dog_ego_planner::PointCloudFilterResult result =
    dog_ego_planner::filterPointCloud(
    cloud, makeTransform(0.0, 0.0, 0.0), Eigen::Vector3d::Zero(), half_pi, config);

  ASSERT_EQ(result.rejected_self, 1U);
  ASSERT_EQ(result.valid_input_points, 2U);
  ASSERT_EQ(result.obstacles.size(), 1U);
  EXPECT_TRUE(result.obstacles.front().isApprox(Eigen::Vector2d(0.30, 0.0), 1e-6));
}

TEST(PointCloudProcessor, RejectsNonFiniteTransform)
{
  const sensor_msgs::msg::PointCloud2 cloud = makeCloud({{1.0F, 0.0F, 0.5F}});
  geometry_msgs::msg::TransformStamped transform = makeTransform(0.0, 0.0, 0.0);
  transform.transform.translation.x = std::numeric_limits<double>::infinity();
  const dog_ego_planner::PointCloudFilterConfig config;

  EXPECT_THROW(
    dog_ego_planner::filterPointCloud(
      cloud, transform, Eigen::Vector3d::Zero(), 0.0, config),
    std::runtime_error);
}

TEST(PointCloudProcessor, EnforcesConfiguredPointLimit)
{
  const sensor_msgs::msg::PointCloud2 cloud = makeCloud(
    {{1.0F, 0.0F, 0.5F}, {2.0F, 0.0F, 0.5F}});
  dog_ego_planner::PointCloudFilterConfig config;
  config.maximum_input_points = 1U;

  EXPECT_THROW(
    dog_ego_planner::filterPointCloud(
      cloud, makeTransform(0.0, 0.0, 0.0), Eigen::Vector3d::Zero(), 0.0, config),
    std::length_error);
}
