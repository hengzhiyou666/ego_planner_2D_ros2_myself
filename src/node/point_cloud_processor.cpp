// Copyright 2026 hengzhiyou
// SPDX-License-Identifier: GPL-3.0-only

#include "node/point_cloud_processor.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>

#include <Eigen/Geometry>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace dog_ego_planner
{

PointCloudFilterResult filterPointCloud(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const geometry_msgs::msg::TransformStamped & cloud_to_planning,
  const Eigen::Vector3d & robot_position, double robot_yaw,
  const PointCloudFilterConfig & config)
{
  PointCloudFilterResult result;
  result.filtered_cloud.header = cloud.header;
  result.filtered_cloud.header.frame_id = config.planning_frame;

  const auto & translation = cloud_to_planning.transform.translation;
  const auto & rotation = cloud_to_planning.transform.rotation;
  if (!std::isfinite(translation.x) || !std::isfinite(translation.y) ||
    !std::isfinite(translation.z))
  {
    throw std::runtime_error("point-cloud transform contains a non-finite translation");
  }
  Eigen::Quaterniond quaternion(rotation.w, rotation.x, rotation.y, rotation.z);
  if (!quaternion.coeffs().allFinite() || quaternion.norm() < 1e-9) {
    throw std::runtime_error("point-cloud transform contains an invalid quaternion");
  }
  quaternion.normalize();
  const Eigen::Vector3d offset(translation.x, translation.y, translation.z);

  const double x_limit = std::min(
    std::max(config.maximum_x_distance_m, 0.0), config.map_size_m * 0.5);
  const double y_limit = std::min(
    std::max(config.maximum_y_distance_m, 0.0), config.map_size_m * 0.5);
  const double z_minimum = robot_position.z() + config.minimum_z_offset_m;
  const double z_maximum = robot_position.z() + config.maximum_z_offset_m;
  const double self_half_length = std::max(
    0.0, config.robot_length_m * 0.5 - config.self_filter_inset_m);
  const double self_half_width = std::max(
    0.0, config.robot_width_m * 0.5 - config.self_filter_inset_m);
  const double cosine = std::cos(robot_yaw);
  const double sine = std::sin(robot_yaw);

  std::vector<std::array<float, 3>> kept;
  const std::size_t declared_points =
    static_cast<std::size_t>(cloud.width) * static_cast<std::size_t>(cloud.height);
  if (declared_points > config.maximum_input_points) {
    throw std::length_error("point cloud exceeds cloud_max_input_points");
  }
  kept.reserve(declared_points);
  result.obstacles.reserve(kept.capacity());

  sensor_msgs::PointCloud2ConstIterator<float> x_iterator(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_iterator(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z_iterator(cloud, "z");
  for (; x_iterator != x_iterator.end();
    ++x_iterator, ++y_iterator, ++z_iterator, ++result.input_points)
  {
    const Eigen::Vector3d sensor_point(*x_iterator, *y_iterator, *z_iterator);
    if (!sensor_point.allFinite()) {
      ++result.rejected_non_finite;
      continue;
    }
    const Eigen::Vector3d point = quaternion * sensor_point + offset;
    if (!point.allFinite()) {
      ++result.rejected_non_finite;
      continue;
    }
    ++result.valid_input_points;
    if (point.z() < z_minimum || point.z() > z_maximum) {
      ++result.rejected_height;
      continue;
    }
    const double relative_x = point.x() - robot_position.x();
    const double relative_y = point.y() - robot_position.y();
    if (std::abs(relative_x) > x_limit || std::abs(relative_y) > y_limit) {
      ++result.rejected_range;
      continue;
    }

    const double robot_x = cosine * relative_x + sine * relative_y;
    const double robot_y = -sine * relative_x + cosine * relative_y;
    if (std::abs(robot_x) < self_half_length && std::abs(robot_y) < self_half_width) {
      ++result.rejected_self;
      continue;
    }

    kept.push_back(
      {
        static_cast<float>(point.x()), static_cast<float>(point.y()),
        static_cast<float>(point.z())});
    result.obstacles.emplace_back(point.x(), point.y());
  }

  result.filtered_cloud.height = 1;
  result.filtered_cloud.width = static_cast<std::uint32_t>(kept.size());
  result.filtered_cloud.is_bigendian = false;
  result.filtered_cloud.is_dense = false;
  sensor_msgs::PointCloud2Modifier modifier(result.filtered_cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(kept.size());
  sensor_msgs::PointCloud2Iterator<float> output_x(result.filtered_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> output_y(result.filtered_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> output_z(result.filtered_cloud, "z");
  for (const auto & point : kept) {
    *output_x = point[0];
    *output_y = point[1];
    *output_z = point[2];
    ++output_x;
    ++output_y;
    ++output_z;
  }
  return result;
}

}  // namespace dog_ego_planner
