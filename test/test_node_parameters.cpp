// Copyright 2026 hengzhiyou
// SPDX-License-Identifier: GPL-3.0-only

#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "node/dog_planner_node.hpp"

class NodeParameterTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  static rclcpp::NodeOptions optionsWith(
    std::vector<rclcpp::Parameter> parameters)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(std::move(parameters));
    return options;
  }
};

TEST_F(NodeParameterTest, RejectsUnsafeFootprintFilterInset)
{
  const rclcpp::NodeOptions options = optionsWith(
    {rclcpp::Parameter("robot_footprint.self_filter_inset_m", -0.01)});
  EXPECT_THROW(dog_ego_planner::makeDogPlannerNode(options), std::invalid_argument);
}

TEST_F(NodeParameterTest, UsesPortableDefaultRosInterface)
{
  const std::shared_ptr<rclcpp::Node> node =
    dog_ego_planner::makeDogPlannerNode(rclcpp::NodeOptions{});

  EXPECT_EQ(node->get_parameter("topics.odom").as_string(), "lidar_location_now");
  EXPECT_EQ(node->get_parameter("topics.point_cloud").as_string(), "lidar_points_copy");
  EXPECT_EQ(node->get_parameter("planning_frame").as_string(), "local_map_lidar_init_xyz");
  EXPECT_FALSE(node->get_parameter("odometry_use_best_effort_qos").as_bool());
  EXPECT_EQ(
    node->get_node_topics_interface()->resolve_topic_name(
      node->get_parameter("topics.odom").as_string()),
    "/lidar_location_now");
  EXPECT_EQ(
    node->get_node_topics_interface()->resolve_topic_name(
      node->get_parameter("topics.point_cloud").as_string()),
    "/lidar_points_copy");
}

TEST_F(NodeParameterTest, RejectsExcessiveMapAllocation)
{
  const rclcpp::NodeOptions options = optionsWith(
    {rclcpp::Parameter("grid_map_size_m", 1000.0),
      rclcpp::Parameter("grid_map_resolution", 0.01)});
  EXPECT_THROW(dog_ego_planner::makeDogPlannerNode(options), std::invalid_argument);
}

TEST_F(NodeParameterTest, RejectsEmptyPublicTopic)
{
  const rclcpp::NodeOptions options = optionsWith(
    {rclcpp::Parameter("topics.local_path", "")});
  EXPECT_THROW(dog_ego_planner::makeDogPlannerNode(options), std::invalid_argument);
}

TEST_F(NodeParameterTest, RejectsInputOutputTopicFeedbackLoop)
{
  const rclcpp::NodeOptions options = optionsWith(
    {rclcpp::Parameter("topics.global_path", "shared_path"),
      rclcpp::Parameter("topics.local_path", "shared_path")});
  EXPECT_THROW(dog_ego_planner::makeDogPlannerNode(options), std::invalid_argument);
}

TEST_F(NodeParameterTest, RejectsInvalidPointCloudHealthParameters)
{
  const rclcpp::NodeOptions negative_minimum = optionsWith(
    {rclcpp::Parameter("safety.minimum_point_cloud_points", -1)});
  EXPECT_THROW(
    dog_ego_planner::makeDogPlannerNode(negative_minimum), std::invalid_argument);

  const rclcpp::NodeOptions negative_skew = optionsWith(
    {rclcpp::Parameter("safety.max_cloud_odometry_skew_s", -0.1)});
  EXPECT_THROW(dog_ego_planner::makeDogPlannerNode(negative_skew), std::invalid_argument);
}

TEST_F(NodeParameterTest, LegacyTopicParameterFeedsCanonicalParameter)
{
  const rclcpp::NodeOptions options = optionsWith(
    {rclcpp::Parameter("topic_odom", "legacy_odometry")});
  const std::shared_ptr<rclcpp::Node> node = dog_ego_planner::makeDogPlannerNode(options);

  EXPECT_EQ(node->get_parameter("topics.odom").as_string(), "legacy_odometry");
  EXPECT_EQ(
    node->get_node_topics_interface()->resolve_topic_name("legacy_odometry"),
    "/legacy_odometry");
}

TEST_F(NodeParameterTest, CanonicalParameterTakesPrecedenceOverLegacyParameter)
{
  const rclcpp::NodeOptions options = optionsWith(
    {rclcpp::Parameter("topic_odom", "legacy_odometry"),
      rclcpp::Parameter("topics.odom", "canonical_odometry")});
  const std::shared_ptr<rclcpp::Node> node = dog_ego_planner::makeDogPlannerNode(options);

  EXPECT_EQ(node->get_parameter("topics.odom").as_string(), "canonical_odometry");
}
