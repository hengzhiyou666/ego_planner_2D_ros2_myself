// Copyright 2026 hengzhiyou
// SPDX-License-Identifier: GPL-3.0-only

#include "node/dog_planner_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "node/point_cloud_processor.hpp"
#include "path_processing/path_utils.hpp"
#include "planning/planning_state.hpp"
#include "planning/planner_interface_dog.hpp"

namespace dog_ego_planner
{
namespace
{

using SteadyClock = std::chrono::steady_clock;
using path_processing::distance2D;

constexpr double kMinimumPositive = 1e-6;

bool stampIsZero(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec == 0 && stamp.nanosec == 0;
}

Eigen::Quaterniond quaternionFromMessage(const geometry_msgs::msg::Quaternion & message)
{
  Eigen::Quaterniond quaternion(message.w, message.x, message.y, message.z);
  if (!quaternion.coeffs().allFinite() || quaternion.norm() < kMinimumPositive) {
    throw std::invalid_argument("quaternion is non-finite or has near-zero norm");
  }
  quaternion.normalize();
  return quaternion;
}

geometry_msgs::msg::Quaternion quaternionToMessage(const Eigen::Quaterniond & quaternion)
{
  geometry_msgs::msg::Quaternion message;
  message.x = quaternion.x();
  message.y = quaternion.y();
  message.z = quaternion.z();
  message.w = quaternion.w();
  return message;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & message)
{
  const Eigen::Quaterniond quaternion = quaternionFromMessage(message);
  const Eigen::Matrix3d rotation = quaternion.toRotationMatrix();
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

geometry_msgs::msg::TransformStamped identityTransform(
  const std::string & frame, const builtin_interfaces::msg::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = frame;
  transform.header.stamp = stamp;
  transform.child_frame_id = frame;
  transform.transform.rotation.w = 1.0;
  return transform;
}

}  // namespace

class DogPlannerNode final : public rclcpp::Node
{
public:
  explicit DogPlannerNode(const rclcpp::NodeOptions & options)
  : Node("dog_ego_planner", options)
  {
    declareAndValidateParameters();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS().keep_last(10);
    rclcpp::QoS odometry_qos(rclcpp::KeepLast(10));
    if (odometry_best_effort_) {
      odometry_qos.best_effort().durability_volatile();
    } else {
      odometry_qos.reliable().durability_volatile();
    }
    rclcpp::QoS global_path_qos(rclcpp::KeepLast(global_path_transient_local_ ? 1 : 10));
    global_path_qos.reliable();
    if (global_path_transient_local_) {
      global_path_qos.transient_local();
    } else {
      global_path_qos.durability_volatile();
    }

    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic_, odometry_qos,
      [this](nav_msgs::msg::Odometry::SharedPtr message) {onOdometry(*message);});
    global_path_subscription_ = create_subscription<nav_msgs::msg::Path>(
      global_path_topic_, global_path_qos,
      [this](nav_msgs::msg::Path::SharedPtr message) {onGlobalPath(*message);});
    point_cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      point_cloud_topic_, sensor_qos,
      [this](sensor_msgs::msg::PointCloud2::SharedPtr message) {onPointCloud(*message);});

    filtered_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      filtered_cloud_topic_, sensor_qos);
    unfinished_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      unfinished_path_topic_, rclcpp::QoS(1).reliable().transient_local());
    local_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      local_path_topic_, rclcpp::QoS(1).reliable().transient_local());
    occupancy_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      occupancy_topic_, rclcpp::QoS(1).reliable().transient_local());
    legacy_goal_publisher_ = create_publisher<std_msgs::msg::Int32>(
      legacy_goal_topic_, rclcpp::QoS(1).reliable().transient_local());
    goal_publisher_ = create_publisher<std_msgs::msg::Bool>(
      goal_topic_, rclcpp::QoS(1).reliable().transient_local());

    planner_.initParam(
      maximum_velocity_, maximum_acceleration_, maximum_jerk_,
      control_point_interval_);
    planner_.initGridMap(
      grid_map_size_, grid_map_size_, grid_map_resolution_,
      Eigen::Vector2d(-grid_map_size_ * 0.5, -grid_map_size_ * 0.5),
      effective_inflation_radius_, astar_pool_size_);
    planner_.setPrintfOpenOrNot(optimizer_logging_);
    planner_.setMaxReboundRetries(maximum_rebound_retries_);

    const auto replan_period = std::max(
      std::chrono::milliseconds(1),
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / replanning_frequency_)));
    replan_timer_ = create_wall_timer(replan_period, [this]() {replan();});
    status_timer_ = create_wall_timer(
      std::chrono::milliseconds(200), [this]() {publishGoalStatus();});
    if (unfinished_update_interval_ > 0.0) {
      const auto unfinished_period = std::max(
        std::chrono::milliseconds(1),
        std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::duration<double>(unfinished_update_interval_)));
      unfinished_timer_ = create_wall_timer(
        unfinished_period,
        [this]() {publishCurrentUnfinishedPath();});
    }

    RCLCPP_INFO(
      get_logger(),
      "dog_ego_planner ready: frame=%s, odometry=%s, global_path=%s, point_cloud=%s",
      planning_frame_.c_str(), odometry_topic_.c_str(), global_path_topic_.c_str(),
      point_cloud_topic_.c_str());
  }

private:
  bool parameterWasOverridden(const std::string & name) const
  {
    const auto & overrides = get_node_options().parameter_overrides();
    return std::any_of(
      overrides.begin(), overrides.end(), [&name](const rclcpp::Parameter & parameter) {
        return parameter.get_name() == name;
      });
  }

  template<typename Type>
  Type declareCompatibleParameter(
    const std::string & canonical_name, const std::string & legacy_name,
    const Type & default_value)
  {
    const Type legacy_value = declare_parameter<Type>(legacy_name, default_value);
    const Type canonical_value = declare_parameter<Type>(canonical_name, legacy_value);
    if (parameterWasOverridden(legacy_name)) {
      RCLCPP_WARN(
        get_logger(), "Parameter '%s' is deprecated; use '%s'.",
        legacy_name.c_str(), canonical_name.c_str());
    }
    return canonical_value;
  }

  template<typename Type>
  void declareIgnoredDeprecatedParameter(
    const std::string & name, const Type & default_value,
    const std::string & replacement)
  {
    (void)declare_parameter<Type>(name, default_value);
    if (parameterWasOverridden(name)) {
      RCLCPP_WARN(
        get_logger(), "Parameter '%s' is deprecated and ignored; %s.",
        name.c_str(), replacement.c_str());
    }
  }

  void declareAndValidateParameters()
  {
    debug_ = declare_parameter<bool>("debug", false);
    replan_on_change_only_ = declareCompatibleParameter<bool>(
      "planning.replan_on_change_only", "control_if_test_environment_changing", true);
    goal_threshold_ = declare_parameter<double>("goal_threshold", 0.7);
    pose_change_threshold_ = declare_parameter<double>("pose_change_thresh", 0.05);
    obstacle_change_resolution_ = declare_parameter<double>("obs_change_thresh", 0.1);

    maximum_horizon_ = declareCompatibleParameter<double>(
      "local_path_max_m", "planning_horizon", 7.0);
    minimum_horizon_ = declare_parameter<double>("local_path_min_m", 2.0);
    horizon_step_ = std::abs(
      declareCompatibleParameter<double>(
        "planning.horizon_step_m", "step", -1.0));
    control_point_interval_ = declare_parameter<double>("control_point_interval", 0.3);
    replanning_frequency_ = declare_parameter<double>("replan_freq", 10.0);
    remaining_path_trigger_ = declare_parameter<double>("planning.remaining_path_trigger_m", 4.0);
    progress_search_margin_ =
      declare_parameter<double>("global_path.progress_search_margin_m", 1.0);
    initial_progress_search_length_ =
      declare_parameter<double>("global_path.initial_search_length_m", 5.0);
    path_reacquire_distance_ =
      declare_parameter<double>("global_path.reacquire_distance_m", 2.0);

    maximum_velocity_ = declare_parameter<double>("max_vel", 1.0);
    maximum_acceleration_ = declare_parameter<double>("max_acc", 1.5);
    maximum_jerk_ = declare_parameter<double>("max_jerk", 5.0);
    optimizer_logging_ = declareCompatibleParameter<bool>(
      "debug.optimizer_logging", "printfOpenOrNot", false);
    maximum_rebound_retries_ = declare_parameter<int>("max_rebound_retries", 3);

    planning_frame_ = declare_parameter<std::string>(
      "planning_frame", "local_map_lidar_init_xyz");
    transform_timeout_ms_ = declare_parameter<int>("cloud_tf_timeout_ms", 100);
    transform_fallback_maximum_age_ms_ =
      declare_parameter<int>("cloud_tf_fallback_max_age_ms", 100);
    allow_latest_transform_fallback_ =
      declare_parameter<bool>("cloud_tf_allow_latest_fallback", true);

    cloud_filter_config_.planning_frame = planning_frame_;
    cloud_filter_config_.maximum_x_distance_m =
      declare_parameter<double>("cloud_valid_x_abs_max", 10.0);
    cloud_filter_config_.maximum_y_distance_m =
      declare_parameter<double>("cloud_valid_y_abs_max", 10.0);
    cloud_filter_config_.minimum_z_offset_m =
      declare_parameter<double>("cloud_valid_z_min", 0.01);
    cloud_filter_config_.maximum_z_offset_m =
      declare_parameter<double>("cloud_valid_z_max", 1.0);
    cloud_filter_config_.robot_length_m =
      declare_parameter<double>("robot_footprint.length_m", 0.70);
    cloud_filter_config_.robot_width_m =
      declare_parameter<double>("robot_footprint.width_m", 0.40);
    cloud_filter_config_.self_filter_inset_m =
      declare_parameter<double>("robot_footprint.self_filter_inset_m", 0.02);
    const int maximum_cloud_points =
      declare_parameter<int>("cloud_max_input_points", 1000000);
    cloud_filter_config_.maximum_input_points = maximum_cloud_points > 0 ?
      static_cast<std::size_t>(maximum_cloud_points) : 0U;
    minimum_point_cloud_points_ =
      declare_parameter<int>("safety.minimum_point_cloud_points", 10);
    footprint_safety_margin_ =
      declare_parameter<double>("robot_footprint.safety_margin_m", 0.05);

    grid_map_size_ = declare_parameter<double>("grid_map_size_m", 20.0);
    grid_map_resolution_ = declare_parameter<double>("grid_map_resolution", 0.1);
    configured_inflation_radius_ = declare_parameter<double>("grid_inflate_radius", 0.46);
    astar_pool_size_ = declare_parameter<int>("astar_pool_size", 100);
    occupancy_publish_frequency_ = declare_parameter<double>("occupancy_publish_freq", 2.0);
    cloud_filter_config_.map_size_m = grid_map_size_;

    declareIgnoredDeprecatedParameter<double>(
      "self_obstacle_clear_radius", -1.0, "use robot_footprint.*");

    unfinished_update_interval_ =
      declare_parameter<double>("global_path_update_interval_s", 1.0);
    direct_follow_enabled_ =
      declare_parameter<bool>("enable_direct_follow_when_corridor_clear", true);
    direct_path_step_ = declare_parameter<double>("corridor_dense_step_m", 0.1);
    direct_follow_requires_lidar_ =
      declare_parameter<bool>("direct_follow_require_lidar", true);
    declareIgnoredDeprecatedParameter<double>(
      "corridor_half_width_m", -1.0,
      "clearance is derived from robot_footprint.*");
    direct_smoothing_enabled_ = declareCompatibleParameter<bool>(
      "direct_path.smoothing_enabled", "if_filter_6m", true);
    direct_smoothing_iterations_ = declareCompatibleParameter<int>(
      "direct_path.smoothing_iterations", "filter_6m_rounds", 2);
    skip_duplicate_paths_ = declareCompatibleParameter<bool>(
      "global_path.skip_duplicate_updates", "if_test_pct_path_update", true);
    duplicate_path_tolerance_ =
      declare_parameter<double>("if_test_pct_path_update_tolerance", 0.01);

    failure_cooldown_ms_ = declare_parameter<int>("replan_failure_cooldown_ms", 200);
    failures_before_cooldown_ =
      declare_parameter<int>("max_consecutive_failures_before_cooldown", 3);
    failure_stop_timeout_ = declareCompatibleParameter<double>(
      "safety.failure_stop_timeout_s", "if_continuously_plan_failed_time", 0.5);
    odometry_timeout_ = declare_parameter<double>("safety.odometry_timeout_s", 0.5);
    point_cloud_timeout_ = declare_parameter<double>("safety.point_cloud_timeout_s", 0.5);
    maximum_odometry_message_age_ =
      declare_parameter<double>("safety.max_odometry_message_age_s", 0.5);
    maximum_point_cloud_message_age_ =
      declare_parameter<double>("safety.max_point_cloud_message_age_s", 0.5);
    maximum_future_stamp_tolerance_ =
      declare_parameter<double>("safety.max_future_stamp_tolerance_s", 0.1);
    maximum_cloud_odometry_skew_ =
      declare_parameter<double>("safety.max_cloud_odometry_skew_s", 0.1);
    path_reuse_maximum_distance_ =
      declare_parameter<double>("safety.path_reuse_max_distance_m", 0.5);
    require_fresh_obstacle_data_ =
      declare_parameter<bool>("safety.require_fresh_obstacle_data", true);

    odometry_topic_ = declareCompatibleParameter<std::string>(
      "topics.odom", "topic_odom", "lidar_location_now");
    global_path_topic_ = declareCompatibleParameter<std::string>(
      "topics.global_path", "topic_pct_path", "pct_path_copy");
    point_cloud_topic_ = declareCompatibleParameter<std::string>(
      "topics.point_cloud", "topic_lidar", "lidar_points_copy");
    filtered_cloud_topic_ =
      declare_parameter<std::string>(
      "topics.filtered_point_cloud", "lidar_points_filtered_copy");
    unfinished_path_topic_ = declare_parameter<std::string>(
      "topics.unfinished_global_path", "dog_output_global_path_unfinished_copy");
    local_path_topic_ =
      declare_parameter<std::string>(
      "topics.local_path", "dog_output_local_path_copy");
    occupancy_topic_ =
      declare_parameter<std::string>(
      "topics.occupancy_grid", "dog_2Dmap_occupancy_copy");
    legacy_goal_topic_ =
      declare_parameter<std::string>(
      "topics.goal_reached_legacy", "if_reach_the_goal_copy");
    goal_topic_ =
      declare_parameter<std::string>("topics.goal_reached", "goal_reached_copy");
    legacy_no_path_is_reached_ = declare_parameter<bool>(
      "compatibility.legacy_no_path_is_reached", false);

    odometry_best_effort_ =
      declare_parameter<bool>("odometry_use_best_effort_qos", false);
    global_path_transient_local_ =
      declare_parameter<bool>("pct_path_match_nav2_qos", true);

    declareIgnoredDeprecatedParameter<bool>(
      "print_callback_msg", false, "use ROS logger levels instead");
    declareIgnoredDeprecatedParameter<bool>(
      "print_replan_return_reason", false, "use ROS logger levels instead");
    declareIgnoredDeprecatedParameter<double>(
      "local_traj_duration", 1.0, "the planner now derives trajectory duration from constraints");

    requirePositive("goal_threshold", goal_threshold_);
    requirePositive("local_path_max_m", maximum_horizon_);
    requirePositive("local_path_min_m", minimum_horizon_);
    requirePositive("planning.horizon_step_m", horizon_step_);
    requirePositive("control_point_interval", control_point_interval_);
    requirePositive("replan_freq", replanning_frequency_);
    requirePositive("max_vel", maximum_velocity_);
    requirePositive("max_acc", maximum_acceleration_);
    requirePositive("max_jerk", maximum_jerk_);
    requirePositive("grid_map_size_m", grid_map_size_);
    requirePositive("grid_map_resolution", grid_map_resolution_);
    requirePositive("cloud_valid_x_abs_max", cloud_filter_config_.maximum_x_distance_m);
    requirePositive("cloud_valid_y_abs_max", cloud_filter_config_.maximum_y_distance_m);
    requirePositive("robot_footprint.length_m", cloud_filter_config_.robot_length_m);
    requirePositive("robot_footprint.width_m", cloud_filter_config_.robot_width_m);
    requirePositive("cloud_max_input_points", static_cast<double>(maximum_cloud_points));
    requirePositive("corridor_dense_step_m", direct_path_step_);
    requirePositive("global_path.progress_search_margin_m", progress_search_margin_);
    requirePositive("global_path.initial_search_length_m", initial_progress_search_length_);
    requirePositive("global_path.reacquire_distance_m", path_reacquire_distance_);
    requirePositive("safety.path_reuse_max_distance_m", path_reuse_maximum_distance_);
    requireNonNegative("pose_change_thresh", pose_change_threshold_);
    requirePositive("obs_change_thresh", obstacle_change_resolution_);
    requireNonNegative("planning.remaining_path_trigger_m", remaining_path_trigger_);
    requireNonNegative("robot_footprint.safety_margin_m", footprint_safety_margin_);
    requireNonNegative(
      "robot_footprint.self_filter_inset_m", cloud_filter_config_.self_filter_inset_m);
    requireNonNegative("grid_inflate_radius", configured_inflation_radius_);
    requireNonNegative("occupancy_publish_freq", occupancy_publish_frequency_);
    requireNonNegative("global_path_update_interval_s", unfinished_update_interval_);
    requireNonNegative("if_test_pct_path_update_tolerance", duplicate_path_tolerance_);
    requireNonNegative("safety.failure_stop_timeout_s", failure_stop_timeout_);
    requireNonNegative("safety.odometry_timeout_s", odometry_timeout_);
    requireNonNegative("safety.point_cloud_timeout_s", point_cloud_timeout_);
    requireNonNegative("safety.max_odometry_message_age_s", maximum_odometry_message_age_);
    requireNonNegative("safety.max_point_cloud_message_age_s", maximum_point_cloud_message_age_);
    requireNonNegative("safety.max_future_stamp_tolerance_s", maximum_future_stamp_tolerance_);
    requireNonNegative("safety.max_cloud_odometry_skew_s", maximum_cloud_odometry_skew_);
    if (minimum_horizon_ > maximum_horizon_) {
      throw std::invalid_argument("local_path_min_m must not exceed local_path_max_m");
    }
    if (!std::isfinite(cloud_filter_config_.minimum_z_offset_m) ||
      !std::isfinite(cloud_filter_config_.maximum_z_offset_m) ||
      cloud_filter_config_.minimum_z_offset_m >= cloud_filter_config_.maximum_z_offset_m)
    {
      throw std::invalid_argument("cloud_valid_z_min must be less than cloud_valid_z_max");
    }
    if (planning_frame_.empty()) {
      throw std::invalid_argument("planning_frame must not be empty");
    }
    const double maximum_self_filter_inset = 0.5 * std::min(
      cloud_filter_config_.robot_length_m, cloud_filter_config_.robot_width_m);
    if (cloud_filter_config_.self_filter_inset_m >= maximum_self_filter_inset) {
      throw std::invalid_argument(
              "robot_footprint.self_filter_inset_m must be smaller than half the footprint width");
    }
    if (maximum_rebound_retries_ < 0 || direct_smoothing_iterations_ < 0 ||
      failure_cooldown_ms_ < 0 || failures_before_cooldown_ < 1 ||
      transform_timeout_ms_ < 0 || transform_fallback_maximum_age_ms_ < 0 ||
      astar_pool_size_ < 20 || astar_pool_size_ > 200 ||
      minimum_point_cloud_points_ < 0 || minimum_point_cloud_points_ > maximum_cloud_points)
    {
      throw std::invalid_argument("integer planner parameters are outside their supported range");
    }
    const double cells_per_axis = std::ceil(grid_map_size_ / grid_map_resolution_);
    constexpr double kMaximumGridCells = 10000000.0;
    if (!std::isfinite(cells_per_axis) || cells_per_axis * cells_per_axis > kMaximumGridCells) {
      throw std::invalid_argument("grid_map_size_m/grid_map_resolution exceeds the map memory limit");
    }
    requireNonEmpty("topics.odom", odometry_topic_);
    requireNonEmpty("topics.global_path", global_path_topic_);
    requireNonEmpty("topics.point_cloud", point_cloud_topic_);
    requireNonEmpty("topics.filtered_point_cloud", filtered_cloud_topic_);
    requireNonEmpty("topics.unfinished_global_path", unfinished_path_topic_);
    requireNonEmpty("topics.local_path", local_path_topic_);
    requireNonEmpty("topics.occupancy_grid", occupancy_topic_);
    requireNonEmpty("topics.goal_reached_legacy", legacy_goal_topic_);
    requireNonEmpty("topics.goal_reached", goal_topic_);
    validateTopicUniqueness();

    footprint_clearance_ = std::hypot(
      cloud_filter_config_.robot_length_m * 0.5,
      cloud_filter_config_.robot_width_m * 0.5) + std::max(0.0, footprint_safety_margin_);
    effective_inflation_radius_ = std::max(configured_inflation_radius_, footprint_clearance_);
    if (configured_inflation_radius_ + 1e-6 < footprint_clearance_) {
      RCLCPP_WARN(
        get_logger(),
        "grid_inflate_radius %.3f m is smaller than the footprint clearance %.3f m; using %.3f m.",
        configured_inflation_radius_, footprint_clearance_, effective_inflation_radius_);
    }
  }

  void requirePositive(const std::string & name, double value) const
  {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(name + " must be positive and finite");
    }
  }

  void requireNonNegative(const std::string & name, double value) const
  {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument(name + " must be non-negative and finite");
    }
  }

  void requireNonEmpty(const std::string & name, const std::string & value) const
  {
    if (value.empty()) {
      throw std::invalid_argument(name + " must not be empty");
    }
  }

  void validateTopicUniqueness()
  {
    const std::vector<std::pair<std::string, std::string>> topics{
      {"topics.odom", odometry_topic_},
      {"topics.global_path", global_path_topic_},
      {"topics.point_cloud", point_cloud_topic_},
      {"topics.filtered_point_cloud", filtered_cloud_topic_},
      {"topics.unfinished_global_path", unfinished_path_topic_},
      {"topics.local_path", local_path_topic_},
      {"topics.occupancy_grid", occupancy_topic_},
      {"topics.goal_reached_legacy", legacy_goal_topic_},
      {"topics.goal_reached", goal_topic_}};
    std::unordered_set<std::string> resolved_topics;
    for (const auto & [parameter_name, topic] : topics) {
      const std::string resolved = get_node_topics_interface()->resolve_topic_name(topic);
      if (!resolved_topics.insert(resolved).second) {
        throw std::invalid_argument(
                parameter_name + " resolves to a topic name already used by this node: " +
                resolved);
      }
    }
  }

  bool transformAgeIsAcceptable(
    const geometry_msgs::msg::TransformStamped & transform,
    const builtin_interfaces::msg::Time & message_stamp) const
  {
    if (stampIsZero(message_stamp) || stampIsZero(transform.header.stamp)) {
      return true;
    }
    const rclcpp::Time message_time(message_stamp, get_clock()->get_clock_type());
    const rclcpp::Time transform_time(
      transform.header.stamp, get_clock()->get_clock_type());
    const double age_ms = static_cast<double>((message_time - transform_time).nanoseconds()) / 1e6;
    return age_ms >= 0.0 &&
           age_ms <= static_cast<double>(transform_fallback_maximum_age_ms_);
  }

  bool validateSensorStamp(
    const builtin_interfaces::msg::Time & stamp,
    std::optional<rclcpp::Time> & previous_stamp,
    double maximum_age_seconds, const char * source_name)
  {
    if (stampIsZero(stamp)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "%s has a zero timestamp; rejecting it for safety.", source_name);
      return false;
    }
    const rclcpp::Time message_time(stamp, get_clock()->get_clock_type());
    const rclcpp::Time current_time = now();
    const double age_seconds = (current_time - message_time).seconds();
    if ((maximum_age_seconds > 0.0 && age_seconds > maximum_age_seconds) ||
      age_seconds < -maximum_future_stamp_tolerance_)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "%s timestamp is outside the accepted age window (age=%.3f s).",
        source_name, age_seconds);
      return false;
    }
    if (previous_stamp && message_time < *previous_stamp) {
      RCLCPP_ERROR(
        get_logger(), "%s timestamp moved backwards; clearing local planning history.",
        source_name);
      previous_stamp.reset();
      last_valid_cloud_transform_.reset();
      last_valid_cloud_source_frame_.clear();
      progress_initialized_ = false;
      progress_segment_ = 0U;
      progress_arc_length_ = 0.0;
      have_local_plan_ = false;
      last_plan_position_.reset();
      last_plan_obstacles_.reset();
      planning_failure_active_ = false;
      return false;
    }
    previous_stamp = message_time;
    return true;
  }

  bool lookupTransform(
    const std::string & source_frame, const builtin_interfaces::msg::Time & stamp,
    geometry_msgs::msg::TransformStamped & transform, bool allow_latest)
  {
    if (source_frame.empty()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Input message has an empty frame_id; refusing to assume planning_frame.");
      return false;
    }
    const std::string & source = source_frame;
    if (source == planning_frame_) {
      transform = identityTransform(planning_frame_, stamp);
      return true;
    }
    try {
      const auto timeout = tf2::durationFromSec(
        static_cast<double>(std::max(0, transform_timeout_ms_)) / 1000.0);
      if (stampIsZero(stamp)) {
        transform = tf_buffer_->lookupTransform(
          planning_frame_, source, tf2::TimePointZero, timeout);
      } else {
        transform = tf_buffer_->lookupTransform(
          planning_frame_, source, rclcpp::Time(stamp), timeout);
      }
      return true;
    } catch (const tf2::TransformException & error) {
      if (allow_latest && !stampIsZero(stamp)) {
        try {
          const auto timeout = tf2::durationFromSec(
            static_cast<double>(std::max(0, transform_timeout_ms_)) / 1000.0);
          transform = tf_buffer_->lookupTransform(
            planning_frame_, source, tf2::TimePointZero, timeout);
          if (!transformAgeIsAcceptable(transform, stamp)) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "Latest transform for %s -> %s exceeds cloud_tf_fallback_max_age_ms.",
              source.c_str(), planning_frame_.c_str());
            return false;
          }
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Using latest transform for %s -> %s after timestamp lookup failed: %s",
            source.c_str(), planning_frame_.c_str(), error.what());
          return true;
        } catch (const tf2::TransformException &) {
        }
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Transform %s -> %s unavailable: %s",
        source.c_str(), planning_frame_.c_str(), error.what());
      return false;
    }
  }

  geometry_msgs::msg::Pose transformPose(
    const geometry_msgs::msg::Pose & pose,
    const geometry_msgs::msg::TransformStamped & transform) const
  {
    const auto & translation = transform.transform.translation;
    if (!std::isfinite(translation.x) || !std::isfinite(translation.y) ||
      !std::isfinite(translation.z))
    {
      throw std::invalid_argument("transform translation is non-finite");
    }
    const Eigen::Quaterniond frame_rotation = quaternionFromMessage(transform.transform.rotation);
    const Eigen::Quaterniond pose_rotation = quaternionFromMessage(pose.orientation);
    const Eigen::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
    const Eigen::Vector3d transformed_position = frame_rotation * position + Eigen::Vector3d(
      translation.x, translation.y, translation.z);
    geometry_msgs::msg::Pose result;
    result.position.x = transformed_position.x();
    result.position.y = transformed_position.y();
    result.position.z = transformed_position.z();
    result.orientation = quaternionToMessage((frame_rotation * pose_rotation).normalized());
    return result;
  }

  bool normalizeOdometry(
    const nav_msgs::msg::Odometry & input, nav_msgs::msg::Odometry & output)
  {
    geometry_msgs::msg::TransformStamped pose_transform;
    if (!lookupTransform(input.header.frame_id, input.header.stamp, pose_transform, false)) {
      return false;
    }
    output = input;
    output.header.frame_id = planning_frame_;
    try {
      output.pose.pose = transformPose(input.pose.pose, pose_transform);
    } catch (const std::exception &) {
      return false;
    }
    if (!std::isfinite(output.pose.pose.position.x) ||
      !std::isfinite(output.pose.pose.position.y) ||
      !std::isfinite(output.pose.pose.position.z))
    {
      return false;
    }

    geometry_msgs::msg::TransformStamped velocity_transform;
    const std::string velocity_frame = input.child_frame_id.empty() ?
      input.header.frame_id : input.child_frame_id;
    if (lookupTransform(velocity_frame, input.header.stamp, velocity_transform, false)) {
      Eigen::Quaterniond rotation;
      try {
        rotation = quaternionFromMessage(velocity_transform.transform.rotation);
      } catch (const std::exception &) {
        return false;
      }
      const Eigen::Vector3d linear(
        input.twist.twist.linear.x, input.twist.twist.linear.y,
        input.twist.twist.linear.z);
      const Eigen::Vector3d angular(
        input.twist.twist.angular.x, input.twist.twist.angular.y,
        input.twist.twist.angular.z);
      const Eigen::Vector3d transformed_linear = rotation * linear;
      const Eigen::Vector3d transformed_angular = rotation * angular;
      output.twist.twist.linear.x = transformed_linear.x();
      output.twist.twist.linear.y = transformed_linear.y();
      output.twist.twist.linear.z = transformed_linear.z();
      output.twist.twist.angular.x = transformed_angular.x();
      output.twist.twist.angular.y = transformed_angular.y();
      output.twist.twist.angular.z = transformed_angular.z();
      output.child_frame_id = planning_frame_;
    } else {
      output.twist.twist = geometry_msgs::msg::Twist{};
      output.child_frame_id = planning_frame_;
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(), 2000,
        "Odometry pose is usable but twist frame could not be transformed; using zero start velocity.");
    }
    return true;
  }

  bool normalizePath(const nav_msgs::msg::Path & input, nav_msgs::msg::Path & output)
  {
    if (input.poses.empty()) {
      return false;
    }
    std::string source_frame = input.header.frame_id;
    if (source_frame.empty()) {
      source_frame = input.poses.front().header.frame_id;
    }
    if (source_frame.empty()) {
      RCLCPP_ERROR(get_logger(), "Global path has an empty frame_id; rejecting it.");
      return false;
    }
    for (const auto & pose : input.poses) {
      if (!pose.header.frame_id.empty() && pose.header.frame_id != source_frame) {
        RCLCPP_ERROR(get_logger(), "Global path contains mixed frame IDs; rejecting it.");
        return false;
      }
    }
    geometry_msgs::msg::TransformStamped transform;
    if (!lookupTransform(source_frame, input.header.stamp, transform, false)) {
      return false;
    }
    output.header = input.header;
    output.header.frame_id = planning_frame_;
    output.poses.clear();
    output.poses.reserve(input.poses.size());
    for (const auto & pose : input.poses) {
      geometry_msgs::msg::PoseStamped transformed;
      transformed.header = output.header;
      try {
        transformed.pose = transformPose(pose.pose, transform);
      } catch (const std::exception &) {
        return false;
      }
      if (!std::isfinite(transformed.pose.position.x) ||
        !std::isfinite(transformed.pose.position.y))
      {
        return false;
      }
      output.poses.push_back(std::move(transformed));
    }
    return true;
  }

  void invalidateOdometry(const std::string & reason)
  {
    have_odometry_ = false;
    last_odometry_received_ = SteadyClock::time_point::min();
    publishSafetyStop(reason);
  }

  void invalidatePointCloud(const std::string & reason)
  {
    have_point_cloud_ = false;
    point_cloud_fault_active_ = true;
    last_point_cloud_received_ = SteadyClock::time_point::min();
    obstacle_points_.clear();
    publishSafetyStop(reason);
  }

  void onOdometry(const nav_msgs::msg::Odometry & message)
  {
    if (!validateSensorStamp(
        message.header.stamp, last_odometry_message_stamp_,
        maximum_odometry_message_age_, "odometry"))
    {
      invalidateOdometry("odometry timestamp is invalid");
      return;
    }
    nav_msgs::msg::Odometry normalized;
    if (!normalizeOdometry(message, normalized)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Rejected odometry that could not be normalized.");
      invalidateOdometry("odometry frame or pose is invalid");
      return;
    }
    last_odometry_ = std::move(normalized);
    have_odometry_ = true;
    last_odometry_received_ = SteadyClock::now();
  }

  void onGlobalPath(const nav_msgs::msg::Path & message)
  {
    nav_msgs::msg::Path normalized;
    if (!normalizePath(message, normalized) || normalized.poses.size() < 2) {
      have_global_path_ = false;
      last_global_path_ = nav_msgs::msg::Path{};
      global_path_points_.clear();
      last_unfinished_path_.clear();
      global_path_length_ = 0.0;
      progress_initialized_ = false;
      progress_segment_ = 0U;
      progress_arc_length_ = 0.0;
      ++global_path_generation_;
      have_local_plan_ = false;
      planning_failure_active_ = false;
      last_plan_position_.reset();
      last_plan_obstacles_.reset();
      failure_streak_start_.reset();
      consecutive_failures_ = 0;
      cooldown_until_ = SteadyClock::time_point::min();
      publishSafetyStop("global path is empty or cannot be transformed", true);
      publishGoalStatus();
      return;
    }
    if (skip_duplicate_paths_ && have_global_path_ &&
      pathsApproximatelyEqual(normalized, last_global_path_))
    {
      return;
    }
    ++global_path_generation_;
    last_global_path_ = std::move(normalized);
    global_path_points_.clear();
    global_path_points_.reserve(last_global_path_.poses.size());
    for (const auto & pose : last_global_path_.poses) {
      global_path_points_.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }
    have_global_path_ = true;
    global_path_length_ = path_processing::polylineLength(global_path_points_);
    planning_finished_ = false;
    progress_initialized_ = false;
    progress_segment_ = 0U;
    progress_arc_length_ = 0.0;
    last_progress_update_ = SteadyClock::now();
    have_local_plan_ = false;
    last_local_path_.clear();
    last_plan_position_.reset();
    last_plan_obstacles_.reset();
    planning_failure_active_ = false;
    consecutive_failures_ = 0;
    failure_streak_start_.reset();
    cooldown_until_ = SteadyClock::time_point::min();
    publishSafetyStop(
      "new global path accepted; waiting for a validated local plan", true,
      PlanningState::kWaitingForData);
    publishCurrentUnfinishedPath();
    publishGoalStatus();
  }

  bool resolvePointCloudTransform(
    const sensor_msgs::msg::PointCloud2 & message,
    geometry_msgs::msg::TransformStamped & transform)
  {
    if (message.header.frame_id.empty()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Point cloud has an empty frame_id; rejecting it.");
      return false;
    }
    const std::string & source = message.header.frame_id;
    if (lookupTransform(
        source, message.header.stamp, transform, allow_latest_transform_fallback_))
    {
      last_valid_cloud_transform_ = transform;
      last_valid_cloud_source_frame_ = source;
      return true;
    }
    if (!last_valid_cloud_transform_ || last_valid_cloud_source_frame_ != source ||
      stampIsZero(message.header.stamp))
    {
      return false;
    }
    if (!transformAgeIsAcceptable(*last_valid_cloud_transform_, message.header.stamp)) {
      last_valid_cloud_transform_.reset();
      last_valid_cloud_source_frame_.clear();
      return false;
    }
    transform = *last_valid_cloud_transform_;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Using the last validated point-cloud transform after lookup failure.");
    return true;
  }

  void onPointCloud(const sensor_msgs::msg::PointCloud2 & message)
  {
    if (!odometryIsFresh()) {
      invalidatePointCloud("point cloud arrived without fresh odometry");
      return;
    }
    if (!validateSensorStamp(
        message.header.stamp, last_point_cloud_message_stamp_,
        maximum_point_cloud_message_age_, "point cloud"))
    {
      invalidatePointCloud("point cloud timestamp is invalid");
      return;
    }
    if (!pointCloudMatchesOdometryStamp(message.header.stamp)) {
      invalidatePointCloud("point cloud and odometry timestamps are too far apart");
      return;
    }
    geometry_msgs::msg::TransformStamped transform;
    if (!resolvePointCloudTransform(message, transform)) {
      invalidatePointCloud("point cloud TF is unavailable or too old");
      return;
    }
    const std::uint64_t declared_point_count =
      static_cast<std::uint64_t>(message.width) * static_cast<std::uint64_t>(message.height);
    if (declared_point_count < static_cast<std::uint64_t>(minimum_point_cloud_points_)) {
      invalidatePointCloud("point cloud does not contain enough points to prove sensor health");
      return;
    }
    if (message.width == 0U || message.height == 0U) {
      if (!message.data.empty()) {
        invalidatePointCloud("point cloud dimensions are inconsistent with its payload");
        return;
      }
      obstacle_points_.clear();
      have_point_cloud_ = true;
      point_cloud_fault_active_ = false;
      last_point_cloud_received_ = SteadyClock::now();
      sensor_msgs::msg::PointCloud2 empty_cloud = message;
      empty_cloud.header.frame_id = planning_frame_;
      if (stampIsZero(empty_cloud.header.stamp)) {
        empty_cloud.header.stamp = now();
      }
      empty_cloud.height = 1U;
      empty_cloud.width = 0U;
      empty_cloud.row_step = 0U;
      filtered_cloud_publisher_->publish(empty_cloud);
      return;
    }
    if (message.data.empty()) {
      invalidatePointCloud("point cloud payload is empty for non-zero dimensions");
      return;
    }
    try {
      PointCloudFilterResult filtered = filterPointCloud(
        message, transform,
        Eigen::Vector3d(
          last_odometry_.pose.pose.position.x, last_odometry_.pose.pose.position.y,
          last_odometry_.pose.pose.position.z),
        yawFromQuaternion(last_odometry_.pose.pose.orientation), cloud_filter_config_);
      if (filtered.valid_input_points <
        static_cast<std::size_t>(minimum_point_cloud_points_))
      {
        invalidatePointCloud("point cloud does not contain enough finite transformed points");
        return;
      }
      if (stampIsZero(filtered.filtered_cloud.header.stamp)) {
        filtered.filtered_cloud.header.stamp = now();
      }
      obstacle_points_ = std::move(filtered.obstacles);
      have_point_cloud_ = true;
      point_cloud_fault_active_ = false;
      last_point_cloud_received_ = SteadyClock::now();
      filtered_cloud_publisher_->publish(filtered.filtered_cloud);
      if (have_local_plan_ && !path_processing::corridorIsClear(
          last_local_path_, obstacle_points_, footprint_clearance_))
      {
        publishSafetyStop("new obstacle blocks the current local path");
      }
      if (debug_) {
        RCLCPP_DEBUG(
          get_logger(),
          "Point cloud: input=%zu valid=%zu obstacles=%zu "
          "rejected(nonfinite=%zu height=%zu range=%zu self=%zu)",
          filtered.input_points, filtered.valid_input_points, obstacle_points_.size(),
          filtered.rejected_non_finite, filtered.rejected_height,
          filtered.rejected_range, filtered.rejected_self);
      }
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Rejected malformed PointCloud2: %s", error.what());
      invalidatePointCloud("point cloud is malformed");
    }
  }

  bool pathsApproximatelyEqual(
    const nav_msgs::msg::Path & first, const nav_msgs::msg::Path & second) const
  {
    if (first.poses.size() != second.poses.size() ||
      first.header.frame_id != second.header.frame_id)
    {
      return false;
    }
    const double tolerance_squared = duplicate_path_tolerance_ * duplicate_path_tolerance_;
    for (std::size_t index = 0; index < first.poses.size(); ++index) {
      const double x_difference =
        first.poses[index].pose.position.x - second.poses[index].pose.position.x;
      const double y_difference =
        first.poses[index].pose.position.y - second.poses[index].pose.position.y;
      if (x_difference * x_difference + y_difference * y_difference > tolerance_squared) {
        return false;
      }
    }
    return true;
  }

  Eigen::Vector2d currentPosition() const
  {
    return Eigen::Vector2d(
      last_odometry_.pose.pose.position.x, last_odometry_.pose.pose.position.y);
  }

  Eigen::Vector2d currentVelocity() const
  {
    return Eigen::Vector2d(
      last_odometry_.twist.twist.linear.x, last_odometry_.twist.twist.linear.y);
  }

  bool dataIsFresh(SteadyClock::time_point stamp, double timeout_seconds) const
  {
    if (stamp == SteadyClock::time_point::min()) {
      return false;
    }
    if (timeout_seconds <= 0.0) {
      return true;
    }
    return std::chrono::duration<double>(SteadyClock::now() - stamp).count() <= timeout_seconds;
  }

  bool messageStampIsFresh(
    const std::optional<rclcpp::Time> & stamp, double maximum_age_seconds) const
  {
    if (!stamp) {
      return false;
    }
    const double age_seconds = (now() - *stamp).seconds();
    return age_seconds >= -maximum_future_stamp_tolerance_ &&
           (maximum_age_seconds <= 0.0 || age_seconds <= maximum_age_seconds);
  }

  bool odometryIsFresh() const
  {
    return have_odometry_ && dataIsFresh(last_odometry_received_, odometry_timeout_) &&
           messageStampIsFresh(last_odometry_message_stamp_, maximum_odometry_message_age_);
  }

  bool pointCloudIsFresh() const
  {
    return have_point_cloud_ && dataIsFresh(last_point_cloud_received_, point_cloud_timeout_) &&
           messageStampIsFresh(
      last_point_cloud_message_stamp_, maximum_point_cloud_message_age_);
  }

  bool pointCloudMatchesOdometryStamp(const builtin_interfaces::msg::Time & cloud_stamp) const
  {
    if (!last_odometry_message_stamp_) {
      return false;
    }
    if (maximum_cloud_odometry_skew_ <= 0.0) {
      return true;
    }
    const rclcpp::Time cloud_time(cloud_stamp, get_clock()->get_clock_type());
    return std::abs((cloud_time - *last_odometry_message_stamp_).seconds()) <=
           maximum_cloud_odometry_skew_;
  }

  std::vector<Eigen::Vector2d> buildCurrentUnfinishedPath()
  {
    if (!have_odometry_ || global_path_points_.size() < 2) {
      return {};
    }
    const Eigen::Vector2d current = currentPosition();
    const auto now_steady = SteadyClock::now();
    const double elapsed = progress_initialized_ ?
      std::chrono::duration<double>(now_steady - last_progress_update_).count() : 0.0;
    const double search_length = progress_initialized_ ?
      std::max(
      progress_search_margin_, maximum_velocity_ * std::max(
        0.0,
        elapsed) +
      progress_search_margin_) :
      initial_progress_search_length_;
    path_processing::PathProjection projection = path_processing::projectForward(
      global_path_points_, current, progress_initialized_ ? progress_segment_ : 0U,
      search_length, progress_initialized_ ? progress_arc_length_ : 0.0);
    if (!projection.valid || projection.squared_distance >
      path_reacquire_distance_ * path_reacquire_distance_)
    {
      return {};
    }
    progress_arc_length_ = std::max(progress_arc_length_, projection.arc_length_m);
    progress_segment_ = projection.segment_index;
    progress_initialized_ = true;
    last_progress_update_ = now_steady;
    return path_processing::unfinishedFromProjection(global_path_points_, current, projection);
  }

  nav_msgs::msg::Path toPathMessage(
    const std::vector<Eigen::Vector2d> & points,
    std::optional<double> fallback_yaw = std::nullopt) const
  {
    nav_msgs::msg::Path message;
    message.header.stamp = now();
    message.header.frame_id = planning_frame_;
    message.poses.reserve(points.size());
    for (std::size_t index = 0; index < points.size(); ++index) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = message.header;
      pose.pose.position.x = points[index].x();
      pose.pose.position.y = points[index].y();
      const double yaw = path_processing::tangentYaw(
        points, index, fallback_yaw.value_or(0.0));
      pose.pose.orientation.z = std::sin(yaw * 0.5);
      pose.pose.orientation.w = std::cos(yaw * 0.5);
      message.poses.push_back(std::move(pose));
    }
    return message;
  }

  void publishCurrentUnfinishedPath()
  {
    if (!have_global_path_) {
      return;
    }
    if (!odometryIsFresh()) {
      last_unfinished_path_.clear();
      unfinished_path_publisher_->publish(toPathMessage({}));
      return;
    }
    std::vector<Eigen::Vector2d> unfinished = buildCurrentUnfinishedPath();
    if (unfinished.size() < 2) {
      last_unfinished_path_.clear();
      unfinished_path_publisher_->publish(toPathMessage({}));
      return;
    }
    last_unfinished_path_ = unfinished;
    unfinished_path_publisher_->publish(toPathMessage(last_unfinished_path_));
  }

  void publishGoalStatus()
  {
    const bool reached = goalIsReached();
    std_msgs::msg::Bool goal_message;
    goal_message.data = reached;
    goal_publisher_->publish(goal_message);
    std_msgs::msg::Int32 legacy_message;
    legacy_message.data = (reached || (!have_global_path_ && legacy_no_path_is_reached_)) ? 1 : 0;
    legacy_goal_publisher_->publish(legacy_message);
    if (reached && !planning_finished_) {
      planning_finished_ = true;
      publishSafetyStop("goal reached", true, PlanningState::kGoalReached);
      RCLCPP_INFO(get_logger(), "Goal reached; local planning stopped.");
    } else if (!reached && have_global_path_) {
      planning_finished_ = false;
    }
  }

  bool goalIsReached() const
  {
    if (!have_global_path_ || !odometryIsFresh() || global_path_points_.empty() ||
      !progress_initialized_)
    {
      return false;
    }
    const double remaining_arc = std::max(0.0, global_path_length_ - progress_arc_length_);
    return remaining_arc <= goal_threshold_ &&
           distance2D(currentPosition(), global_path_points_.back()) <= goal_threshold_;
  }

  void transitionTo(PlanningState next_state, const std::string & reason)
  {
    if (planning_state_ == next_state) {
      return;
    }
    RCLCPP_INFO(
      get_logger(), "Planning state: %s -> %s (%s)",
      planningStateName(planning_state_), planningStateName(next_state), reason.c_str());
    planning_state_ = next_state;
  }

  void publishSafetyStop(
    const std::string & reason, bool clear_unfinished = false,
    PlanningState stop_state = PlanningState::kDegradedStop)
  {
    std::vector<Eigen::Vector2d> stop_path;
    if (odometryIsFresh()) {
      const Eigen::Vector2d current = currentPosition();
      stop_path = {current, current};
      double yaw = 0.0;
      try {
        yaw = yawFromQuaternion(last_odometry_.pose.pose.orientation);
      } catch (const std::exception &) {
      }
      local_path_publisher_->publish(toPathMessage(stop_path, yaw));
    } else {
      local_path_publisher_->publish(toPathMessage({}));
    }
    last_local_path_ = std::move(stop_path);
    have_local_plan_ = false;
    if (clear_unfinished) {
      unfinished_path_publisher_->publish(toPathMessage({}));
    }
    if (reason != last_stop_reason_) {
      RCLCPP_WARN(get_logger(), "Publishing safe stop: %s", reason.c_str());
      last_stop_reason_ = reason;
    }
    transitionTo(stop_state, reason);
  }

  bool poseChanged() const
  {
    return !last_plan_position_ ||
           distance2D(currentPosition(), *last_plan_position_) > pose_change_threshold_;
  }

  bool obstaclesChanged() const
  {
    if (!last_plan_obstacles_) {
      return true;
    }
    const auto previous = path_processing::quantizedObstacleSet(
      *last_plan_obstacles_, obstacle_change_resolution_);
    const auto current = path_processing::quantizedObstacleSet(
      obstacle_points_, obstacle_change_resolution_);
    std::size_t difference = 0;
    for (const auto key : previous) {
      difference += current.count(key) == 0 ? 1U : 0U;
    }
    for (const auto key : current) {
      difference += previous.count(key) == 0 ? 1U : 0U;
    }
    const std::size_t baseline =
      std::max<std::size_t>(1, std::max(previous.size(), current.size()));
    return static_cast<double>(difference) / static_cast<double>(baseline) > 0.05;
  }

  bool shouldPublishOccupancy()
  {
    if (occupancy_publish_frequency_ <= 0.0) {
      return true;
    }
    const auto now_steady = SteadyClock::now();
    if (last_occupancy_publish_ == SteadyClock::time_point::min()) {
      last_occupancy_publish_ = now_steady;
      return true;
    }
    const double elapsed = std::chrono::duration<double>(
      now_steady - last_occupancy_publish_).count();
    if (elapsed < 1.0 / occupancy_publish_frequency_) {
      return false;
    }
    last_occupancy_publish_ = now_steady;
    return true;
  }

  void publishOccupancyGrid()
  {
    std::vector<std::int8_t> data;
    int width = 0;
    int height = 0;
    double resolution = 0.0;
    Eigen::Vector2d origin = Eigen::Vector2d::Zero();
    if (!planner_.getInflatedOccupancyGrid(data, width, height, resolution, origin)) {
      return;
    }
    nav_msgs::msg::OccupancyGrid message;
    message.header.stamp = now();
    message.header.frame_id = planning_frame_;
    message.info.resolution = static_cast<float>(resolution);
    message.info.width = static_cast<std::uint32_t>(width);
    message.info.height = static_cast<std::uint32_t>(height);
    message.info.origin.position.x = origin.x();
    message.info.origin.position.y = origin.y();
    message.info.origin.orientation.w = 1.0;
    message.data = std::move(data);
    occupancy_publisher_->publish(message);
  }

  std::optional<std::vector<Eigen::Vector2d>> safeReusablePath() const
  {
    if (!have_local_plan_ || local_plan_generation_ != global_path_generation_ ||
      !pointCloudIsFresh() || last_local_path_.size() < 2)
    {
      return std::nullopt;
    }
    const path_processing::PathProjection projection = path_processing::projectForward(
      last_local_path_, currentPosition(), 0U, std::numeric_limits<double>::infinity());
    if (!projection.valid || projection.squared_distance >
      path_reuse_maximum_distance_ * path_reuse_maximum_distance_)
    {
      return std::nullopt;
    }
    std::vector<Eigen::Vector2d> suffix = path_processing::unfinishedFromProjection(
      last_local_path_, currentPosition(), projection);
    if (suffix.size() < 2 || !path_processing::corridorIsClear(
        suffix, obstacle_points_, footprint_clearance_))
    {
      return std::nullopt;
    }
    return suffix;
  }

  void recordPlanSnapshot()
  {
    last_plan_position_ = currentPosition();
    last_plan_obstacles_ = obstacle_points_;
  }

  void handlePlanningFailure(const std::string & reason)
  {
    const auto now_steady = SteadyClock::now();
    if (!failure_streak_start_) {
      failure_streak_start_ = now_steady;
    }
    planning_failure_active_ = true;
    ++consecutive_failures_;
    const double failure_duration = std::chrono::duration<double>(
      now_steady - *failure_streak_start_).count();
    std::optional<std::vector<Eigen::Vector2d>> reusable_path = safeReusablePath();
    const bool may_reuse = reusable_path && failure_stop_timeout_ > 0.0 &&
      failure_duration < failure_stop_timeout_;
    if (may_reuse) {
      last_local_path_ = std::move(*reusable_path);
      local_path_publisher_->publish(toPathMessage(last_local_path_));
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Planning failed but the previous local path is still collision-free: %s", reason.c_str());
    } else {
      publishSafetyStop("planning failure: " + reason);
    }
    if (consecutive_failures_ >= std::max(1, failures_before_cooldown_)) {
      cooldown_until_ = now_steady + std::chrono::milliseconds(std::max(0, failure_cooldown_ms_));
      consecutive_failures_ = 0;
    }
    recordPlanSnapshot();
  }

  void replan()
  {
    if (planning_finished_) {
      return;
    }
    if (!have_global_path_ || global_path_points_.size() < 2) {
      publishSafetyStop(
        "waiting for a valid global path", false, PlanningState::kWaitingForData);
      return;
    }
    if (!odometryIsFresh()) {
      publishSafetyStop(
        "odometry is missing or stale", false,
        have_odometry_ ? PlanningState::kDegradedStop : PlanningState::kWaitingForData);
      return;
    }
    if (require_fresh_obstacle_data_ && !pointCloudIsFresh()) {
      publishSafetyStop(
        "point cloud is missing or stale", false,
        have_point_cloud_ ? PlanningState::kDegradedStop : PlanningState::kWaitingForData);
      return;
    }
    if (point_cloud_fault_active_) {
      publishSafetyStop("point cloud input is in a faulted state");
      return;
    }
    if (goalIsReached()) {
      publishGoalStatus();
      return;
    }
    if (SteadyClock::now() < cooldown_until_) {
      const double failure_duration = failure_streak_start_ ?
        std::chrono::duration<double>(SteadyClock::now() - *failure_streak_start_).count() : 0.0;
      const bool reuse_window_open = failure_stop_timeout_ > 0.0 &&
        failure_duration < failure_stop_timeout_;
      std::optional<std::vector<Eigen::Vector2d>> reusable_path = safeReusablePath();
      if (!reusable_path || (planning_failure_active_ && !reuse_window_open)) {
        publishSafetyStop("planner cooldown and previous path is unsafe");
      } else {
        last_local_path_ = std::move(*reusable_path);
        local_path_publisher_->publish(toPathMessage(last_local_path_));
      }
      return;
    }

    std::vector<Eigen::Vector2d> unfinished = buildCurrentUnfinishedPath();
    if (unfinished.size() < 2) {
      publishSafetyStop("global path progress could not be projected", true);
      return;
    }
    last_unfinished_path_ = unfinished;
    unfinished_path_publisher_->publish(toPathMessage(last_unfinished_path_));

    const bool pose_changed = poseChanged();
    const bool obstacles_changed = obstaclesChanged();
    const bool remaining_short =
      path_processing::remainingLength(
      last_local_path_,
      currentPosition()) < remaining_path_trigger_;
    Eigen::Vector2d temporary_goal = unfinished.back();
    std::vector<Eigen::Vector2d> horizon = path_processing::extractHorizon(
      unfinished, maximum_horizon_, &temporary_goal);
    if (horizon.size() < 2) {
      publishSafetyStop("local horizon is too short");
      return;
    }
    const bool cloud_available = pointCloudIsFresh() || !direct_follow_requires_lidar_;
    const bool corridor_clear = cloud_available && path_processing::corridorIsClear(
      horizon, obstacle_points_, footprint_clearance_);

    planner_.setCurrentPose(PathPoint2D{currentPosition().x(), currentPosition().y()});
    std::vector<Obstacle2D> planner_obstacles;
    planner_obstacles.reserve(obstacle_points_.size());
    for (const Eigen::Vector2d & obstacle : obstacle_points_) {
      planner_obstacles.push_back(Obstacle2D{obstacle.x(), obstacle.y()});
    }
    // Rebuild after every rolling-map center update so occupancy never retains an old origin.
    planner_.setObstacles(planner_obstacles);
    if (shouldPublishOccupancy()) {
      publishOccupancyGrid();
    }

    if (replan_on_change_only_ && have_local_plan_ && !planning_failure_active_ &&
      !pose_changed && !obstacles_changed && !remaining_short && corridor_clear &&
      safeReusablePath().has_value())
    {
      return;
    }

    if (direct_follow_enabled_ && corridor_clear) {
      std::vector<Eigen::Vector2d> direct_path = path_processing::densify(
        horizon, direct_path_step_);
      if (direct_smoothing_enabled_) {
        const std::vector<Eigen::Vector2d> smoothed = path_processing::smooth(
          direct_path, direct_smoothing_iterations_);
        if (path_processing::corridorIsClear(
            smoothed, obstacle_points_, footprint_clearance_))
        {
          direct_path = smoothed;
        } else {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Rejected a smoothed direct path because it entered the footprint clearance corridor.");
        }
      }
      if (direct_path.size() >= 2 && path_processing::corridorIsClear(
          direct_path, obstacle_points_, footprint_clearance_))
      {
        last_local_path_ = std::move(direct_path);
        have_local_plan_ = true;
        local_plan_generation_ = global_path_generation_;
        local_path_publisher_->publish(toPathMessage(last_local_path_));
        consecutive_failures_ = 0;
        failure_streak_start_.reset();
        planning_failure_active_ = false;
        last_stop_reason_.clear();
        transitionTo(PlanningState::kActive, "validated direct local path published");
        recordPlanSnapshot();
        return;
      }
    }

    const std::vector<Eigen::Vector2d> reusable_local_path =
      have_local_plan_ && local_plan_generation_ == global_path_generation_ ?
      last_local_path_ : std::vector<Eigen::Vector2d>{};
    const std::vector<Eigen::Vector2d> guide = path_processing::inheritedGuide(
      currentPosition(), reusable_local_path, unfinished, maximum_horizon_, minimum_horizon_);
    bool planned = false;
    const std::vector<double> horizon_lengths = path_processing::descendingHorizonLengths(
      maximum_horizon_, minimum_horizon_, horizon_step_);
    for (const double horizon_length : horizon_lengths) {
      Eigen::Vector2d local_goal = guide.back();
      std::vector<Eigen::Vector2d> reference = path_processing::extractHorizon(
        guide, horizon_length, &local_goal);
      const std::vector<Eigen::Vector2d> straight = path_processing::straightReference(
        currentPosition(), local_goal);
      if (!straight.empty() && path_processing::polylineLength(straight) + 1e-3 < horizon_length &&
        path_processing::corridorIsClear(straight, obstacle_points_, footprint_clearance_))
      {
        reference = straight;
      }
      if (reference.size() < 4) {
        reference = path_processing::densify(reference, control_point_interval_ * 0.5);
      }
      const auto control_indices = path_processing::sampleControlPointIndices(reference);
      std::vector<PathPoint2D> control_points;
      control_points.reserve(control_indices.size());
      for (const std::size_t index : control_indices) {
        if (index < reference.size()) {
          control_points.push_back(PathPoint2D{reference[index].x(), reference[index].y()});
        }
      }
      if (control_points.size() < 4) {
        continue;
      }
      planner_.setReferencePath(control_points);
      const bool attempt_succeeded = planner_.makePlan(
        currentVelocity(), Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero());
      if (optimizer_logging_) {
        RCLCPP_DEBUG(
          get_logger(), "B-spline horizon %.2f m: success=%s wall_time=%.3f ms",
          horizon_length, attempt_succeeded ? "true" : "false",
          planner_.getLastReboundOptimizeWallMs());
      }
      if (attempt_succeeded) {
        planned = true;
        break;
      }
    }
    if (!planned) {
      handlePlanningFailure("all configured horizon lengths failed");
      return;
    }

    std::vector<PathPoint2D> trajectory;
    planner_.getPlannedTraj(trajectory, 0.1);
    std::vector<Eigen::Vector2d> planned_path;
    planned_path.reserve(trajectory.size());
    bool trajectory_is_finite = true;
    for (const PathPoint2D & point : trajectory) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        trajectory_is_finite = false;
        break;
      }
      planned_path.emplace_back(point.x, point.y);
    }
    if (!trajectory_is_finite || planned_path.size() < 2 || !path_processing::corridorIsClear(
        planned_path, obstacle_points_, footprint_clearance_))
    {
      handlePlanningFailure("optimizer output failed final collision validation");
      return;
    }

    last_local_path_ = std::move(planned_path);
    have_local_plan_ = true;
    local_plan_generation_ = global_path_generation_;
    local_path_publisher_->publish(toPathMessage(last_local_path_));
    consecutive_failures_ = 0;
    failure_streak_start_.reset();
    planning_failure_active_ = false;
    cooldown_until_ = SteadyClock::time_point::min();
    last_stop_reason_.clear();
    transitionTo(PlanningState::kActive, "validated optimized local path published");
    recordPlanSnapshot();
  }

  bool debug_{false};
  bool replan_on_change_only_{true};
  double goal_threshold_{0.7};
  double pose_change_threshold_{0.05};
  double obstacle_change_resolution_{0.1};
  double maximum_horizon_{7.0};
  double minimum_horizon_{2.0};
  double horizon_step_{1.0};
  double control_point_interval_{0.3};
  double replanning_frequency_{10.0};
  double remaining_path_trigger_{4.0};
  double progress_search_margin_{1.0};
  double initial_progress_search_length_{5.0};
  double path_reacquire_distance_{2.0};
  double maximum_velocity_{1.0};
  double maximum_acceleration_{1.5};
  double maximum_jerk_{5.0};
  bool optimizer_logging_{false};
  int maximum_rebound_retries_{3};
  std::string planning_frame_{"local_map_lidar_init_xyz"};
  int transform_timeout_ms_{100};
  int transform_fallback_maximum_age_ms_{100};
  bool allow_latest_transform_fallback_{true};
  PointCloudFilterConfig cloud_filter_config_;
  int minimum_point_cloud_points_{10};
  double footprint_safety_margin_{0.05};
  double footprint_clearance_{0.46};
  double grid_map_size_{20.0};
  double grid_map_resolution_{0.1};
  double configured_inflation_radius_{0.46};
  double effective_inflation_radius_{0.46};
  int astar_pool_size_{100};
  double occupancy_publish_frequency_{2.0};
  double unfinished_update_interval_{1.0};
  bool direct_follow_enabled_{true};
  double direct_path_step_{0.1};
  bool direct_follow_requires_lidar_{true};
  bool direct_smoothing_enabled_{true};
  int direct_smoothing_iterations_{2};
  bool skip_duplicate_paths_{true};
  double duplicate_path_tolerance_{0.01};
  int failure_cooldown_ms_{200};
  int failures_before_cooldown_{3};
  double failure_stop_timeout_{0.5};
  double odometry_timeout_{0.5};
  double point_cloud_timeout_{0.5};
  double maximum_odometry_message_age_{0.5};
  double maximum_point_cloud_message_age_{0.5};
  double maximum_future_stamp_tolerance_{0.1};
  double maximum_cloud_odometry_skew_{0.1};
  double path_reuse_maximum_distance_{0.5};
  bool require_fresh_obstacle_data_{true};
  std::string odometry_topic_{"lidar_location_now"};
  std::string global_path_topic_{"pct_path_copy"};
  std::string point_cloud_topic_{"lidar_points_copy"};
  std::string filtered_cloud_topic_{"lidar_points_filtered_copy"};
  std::string unfinished_path_topic_{"dog_output_global_path_unfinished_copy"};
  std::string local_path_topic_{"dog_output_local_path_copy"};
  std::string occupancy_topic_{"dog_2Dmap_occupancy_copy"};
  std::string legacy_goal_topic_{"if_reach_the_goal_copy"};
  std::string goal_topic_{"goal_reached_copy"};
  bool legacy_no_path_is_reached_{false};
  bool odometry_best_effort_{false};
  bool global_path_transient_local_{true};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr unfinished_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr legacy_goal_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_publisher_;
  rclcpp::TimerBase::SharedPtr replan_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr unfinished_timer_;

  nav_msgs::msg::Odometry last_odometry_;
  nav_msgs::msg::Path last_global_path_;
  std::vector<Eigen::Vector2d> global_path_points_;
  std::vector<Eigen::Vector2d> obstacle_points_;
  std::vector<Eigen::Vector2d> last_unfinished_path_;
  std::vector<Eigen::Vector2d> last_local_path_;
  bool have_odometry_{false};
  bool have_global_path_{false};
  bool have_point_cloud_{false};
  bool point_cloud_fault_active_{false};
  bool have_local_plan_{false};
  bool planning_finished_{false};
  bool planning_failure_active_{false};
  bool progress_initialized_{false};
  std::size_t progress_segment_{0};
  double progress_arc_length_{0.0};
  double global_path_length_{0.0};
  std::uint64_t global_path_generation_{0U};
  std::uint64_t local_plan_generation_{0U};
  PlanningState planning_state_{PlanningState::kWaitingForData};
  SteadyClock::time_point last_odometry_received_{SteadyClock::time_point::min()};
  SteadyClock::time_point last_point_cloud_received_{SteadyClock::time_point::min()};
  SteadyClock::time_point last_progress_update_{SteadyClock::time_point::min()};
  SteadyClock::time_point last_occupancy_publish_{SteadyClock::time_point::min()};
  SteadyClock::time_point cooldown_until_{SteadyClock::time_point::min()};
  std::optional<SteadyClock::time_point> failure_streak_start_;
  std::optional<rclcpp::Time> last_odometry_message_stamp_;
  std::optional<rclcpp::Time> last_point_cloud_message_stamp_;
  std::optional<Eigen::Vector2d> last_plan_position_;
  std::optional<std::vector<Eigen::Vector2d>> last_plan_obstacles_;
  std::optional<geometry_msgs::msg::TransformStamped> last_valid_cloud_transform_;
  std::string last_valid_cloud_source_frame_;
  int consecutive_failures_{0};
  std::string last_stop_reason_;

  PlannerInterfaceDog planner_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

std::shared_ptr<rclcpp::Node> makeDogPlannerNode(const rclcpp::NodeOptions & options)
{
  return std::make_shared<DogPlannerNode>(options);
}

}  // namespace dog_ego_planner
