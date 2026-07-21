// Copyright 2026 hengzhiyou
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace dog_ego_planner
{

struct PointCloudFilterConfig
{
  std::string planning_frame{"map_frame"};
  double maximum_x_distance_m{20.0};
  double maximum_y_distance_m{20.0};
  double minimum_z_offset_m{0.01};
  double maximum_z_offset_m{1.0};
  double map_size_m{20.0};
  double robot_length_m{0.70};
  double robot_width_m{0.40};
  double self_filter_inset_m{0.02};
  std::size_t maximum_input_points{1000000U};
};

struct PointCloudFilterResult
{
  sensor_msgs::msg::PointCloud2 filtered_cloud;
  std::vector<Eigen::Vector2d> obstacles;
  std::size_t input_points{0};
  std::size_t valid_input_points{0};
  std::size_t rejected_non_finite{0};
  std::size_t rejected_height{0};
  std::size_t rejected_range{0};
  std::size_t rejected_self{0};
};

PointCloudFilterResult filterPointCloud(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const geometry_msgs::msg::TransformStamped & cloud_to_planning,
  const Eigen::Vector3d & robot_position, double robot_yaw,
  const PointCloudFilterConfig & config);

}  // namespace dog_ego_planner
