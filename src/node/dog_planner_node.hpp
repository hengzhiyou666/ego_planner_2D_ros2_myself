// Copyright 2026 hengzhiyou
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <memory>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

namespace dog_ego_planner
{

std::shared_ptr<rclcpp::Node> makeDogPlannerNode(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

}  // namespace dog_ego_planner
