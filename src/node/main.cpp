// Copyright 2026 hengzhiyou
// SPDX-License-Identifier: GPL-3.0-only

#include "node/dog_planner_node.hpp"

#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    const std::shared_ptr<rclcpp::Node> node = dog_ego_planner::makeDogPlannerNode();
    if (rclcpp::ok()) {
      rclcpp::spin(node);
    }
  } catch (const std::exception & error) {
    if (rclcpp::ok()) {
      RCLCPP_FATAL(rclcpp::get_logger("dog_ego_planner"), "Node startup failed: %s", error.what());
      exit_code = 1;
    }
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
