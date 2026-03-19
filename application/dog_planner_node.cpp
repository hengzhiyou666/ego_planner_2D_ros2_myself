#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/header.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include <tf2/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "dog_ego_planner/planner_interface_dog.h"

namespace
{

constexpr const char* kFrameId = "head_init";

double dist2d(const Eigen::Vector2d& a, const Eigen::Vector2d& b) { return (a - b).norm(); }

// Find index of closest point.
size_t closestIndex(const std::vector<Eigen::Vector2d>& pts, const Eigen::Vector2d& q)
{
  if (pts.empty()) return 0;  // 防呆：空容器时返回 0，调用方需确保非空
  size_t best = 0;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < pts.size(); ++i) {
    const double d2 = (pts[i] - q).squaredNorm();
    if (d2 < best_d2) {
      best_d2 = d2;
      best = i;
    }
  }
  return best;
}

// Interpolate point at "target_len" from start index along pts.
// Returns (point, index_before, alpha in [0,1] on segment [i,i+1]).
struct InterpOnPath
{
  Eigen::Vector2d p;
  size_t i0{0};
  double alpha{0.0};
};

InterpOnPath pointAlongPath(const std::vector<Eigen::Vector2d>& pts, size_t start_idx, double target_len)
{
  InterpOnPath out;
  out.p = pts[start_idx];
  out.i0 = start_idx;
  out.alpha = 0.0;
  if (pts.empty()) return out;
  if (start_idx >= pts.size() - 1) return out;
  if (target_len <= 0.0) return out;

  double acc = 0.0;
  constexpr size_t kMaxPathSegments = 100000;  // 防呆：防止异常长路径死循环
  for (size_t i = start_idx; i + 1 < pts.size() && (i - start_idx) < kMaxPathSegments; ++i) {
    const double seg = (pts[i + 1] - pts[i]).norm();
    if (acc + seg >= target_len && seg > 1e-9) {
      const double rem = target_len - acc;
      out.i0 = i;
      out.alpha = rem / seg;
      out.p = pts[i] + out.alpha * (pts[i + 1] - pts[i]);
      return out;
    }
    acc += seg;
  }
  // beyond end: clamp
  out.p = pts.back();
  out.i0 = (pts.size() >= 2) ? (pts.size() - 2) : 0;  // 防呆：pts.size()==1 时避免 size_t 下溢
  out.alpha = 1.0;
  return out;
}

// Build a simple hash-set from points for obstacle-change detection.
int64_t hashKey(double x, double y, double res)
{
  const double res_safe = std::max(std::abs(res), 1e-9);  // 防呆：避免除零
  const int64_t ix = static_cast<int64_t>(std::llround(x / res_safe));
  const int64_t iy = static_cast<int64_t>(std::llround(y / res_safe));
  return (ix << 32) ^ (iy & 0xffffffff);
}

std::unordered_set<int64_t> quantizeSet(const std::vector<dog_ego_planner::Obstacle2D>& obs, double res)
{
  std::unordered_set<int64_t> s;
  if (std::abs(res) < 1e-12) return s;  // 防呆：res 过小或为 0 时返回空集
  s.reserve(obs.size());
  for (const auto& o : obs) s.insert(hashKey(o.x, o.y, res));
  return s;
}

double angleDeg(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2)
{
  const double n1 = v1.norm();
  const double n2 = v2.norm();
  if (n1 < 1e-6 || n2 < 1e-6) return 0.0;
  double c = v1.dot(v2) / (n1 * n2);
  c = std::max(-1.0, std::min(1.0, c));
  return std::acos(c) * 180.0 / M_PI;
}

}  // namespace

class DogPlannerNode : public rclcpp::Node
{
public:
  DogPlannerNode() : Node("dog_ego_planner")
  {
    debug_ = declare_parameter<bool>("debug", true);
    goal_threshold_ = declare_parameter<double>("goal_threshold", 0.1);
    pose_change_thresh_ = declare_parameter<double>("pose_change_thresh", 0.05);
    obs_change_thresh_ = declare_parameter<double>("obs_change_thresh", 0.1);  // used as quantize res baseline

    planning_horizon_ = declare_parameter<double>("planning_horizon", 7.0);
    control_point_interval_ = declare_parameter<double>("control_point_interval", 0.3);
    replan_freq_ = declare_parameter<double>("replan_freq", 50.0);
    local_traj_duration_ = declare_parameter<double>("local_traj_duration", 1.0);

    // Core constraints (2D)
    max_vel_ = declare_parameter<double>("max_vel", 1.0);
    max_acc_ = declare_parameter<double>("max_acc", 2.0);
    max_jerk_ = declare_parameter<double>("max_jerk", 5.0);
    printf_open_or_not_ = declare_parameter<bool>("printfOpenOrNot", true);
    max_rebound_retries_ = declare_parameter<int>("max_rebound_retries", 5);
    cloud_tf_timeout_ms_ = declare_parameter<int>("cloud_tf_timeout_ms", 100);
    cloud_valid_x_abs_max_ = declare_parameter<double>("cloud_valid_x_abs_max", 200.0);
    cloud_valid_y_abs_max_ = declare_parameter<double>("cloud_valid_y_abs_max", 200.0);
    cloud_valid_z_min_ = declare_parameter<double>("cloud_valid_z_min", -3.0);
    cloud_valid_z_max_ = declare_parameter<double>("cloud_valid_z_max", 5.0);
    grid_map_size_m_ = declare_parameter<double>("grid_map_size_m", 40.0);
    grid_map_resolution_ = declare_parameter<double>("grid_map_resolution", 0.1);
    grid_inflate_radius_ = declare_parameter<double>("grid_inflate_radius", 0.20);
    astar_pool_size_ = declare_parameter<int>("astar_pool_size", 100);
    occupancy_publish_freq_ = declare_parameter<double>("occupancy_publish_freq", -1.0);
    self_obstacle_clear_radius_ = declare_parameter<double>("self_obstacle_clear_radius", 0.45);
    cloud_tf_fallback_max_age_ms_ = declare_parameter<int>("cloud_tf_fallback_max_age_ms", 500);
    global_path_update_interval_s_ = declare_parameter<double>("global_path_update_interval_s", 1.0);
    if_test_pct_path_update_ = declare_parameter<bool>("if_test_pct_path_update", true);
    if_test_pct_path_update_tolerance_ = declare_parameter<double>("if_test_pct_path_update_tolerance", 0.01);
    replan_failure_cooldown_ms_ = declare_parameter<int>("replan_failure_cooldown_ms", 500);
    max_consecutive_failures_before_cooldown_ =
      declare_parameter<int>("max_consecutive_failures_before_cooldown", 3);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry", rclcpp::QoS(10),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { onOdom(*msg); });

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/pct_path", rclcpp::QoS(10),
      [this](nav_msgs::msg::Path::SharedPtr msg) { autofunction_dealwith_pct_path(*msg); });

    // 订阅激光点云话题：队列深度为 10，收到每帧点云后交给 autofunction_dealwith_lidar_points 进行预处理与建图更新。
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar_points", rclcpp::QoS(10),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { autofunction_dealwith_lidar_points(*msg); });

    pub_cloud_filtered_ = create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_points_filtered", 10);
    pub_global_unfinished_ = create_publisher<nav_msgs::msg::Path>("/dog_output_global_path_unfinished", 10);
    pub_local_path_ = create_publisher<nav_msgs::msg::Path>("/dog_output_local_path", 10);
    pub_occupancy_map_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/dog_2Dmap_occupancy", 1);

    // TF 监听器：用于将激光点云从 vita_lidar 等传感器坐标系转换到 head_init。
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Init planner core
    planner_.initParam(max_vel_, max_acc_, max_jerk_);
    planner_.initGridMap(
      grid_map_size_m_, grid_map_size_m_, grid_map_resolution_,
      Eigen::Vector2d(-grid_map_size_m_ / 2.0, -grid_map_size_m_ / 2.0),
      grid_inflate_radius_, astar_pool_size_);
    planner_.setPrintfOpenOrNot(printf_open_or_not_);
    planner_.setMaxReboundRetries(max_rebound_retries_);

    const int replan_ms = static_cast<int>(std::round(1000.0 / std::max(1.0, replan_freq_)));
    replan_timer_ = create_wall_timer(std::chrono::milliseconds(replan_ms), [this] { replanTick(); });

    data_wait_start_time_ = std::chrono::steady_clock::now();
    data_wait_timer_ = create_wall_timer(std::chrono::seconds(1), [this] { dataWaitStatusTick(); });

    RCLCPP_INFO(get_logger(), "dog_ego_planner 节点初始化完成，等待 /odometry 和 /pct_path 数据...");

    // 定时器：每隔 global_path_update_interval_s_ 秒根据 /pct_path 和当前 /odometry 重新计算全局未完成路径
    if (global_path_update_interval_s_ > 0.0) {
      const int global_path_ms = static_cast<int>(std::round(global_path_update_interval_s_ * 1000.0));
      global_path_update_timer_ =
        create_wall_timer(std::chrono::milliseconds(global_path_ms), [this] { updateGlobalUnfinished(); });
    }
  }

private:
  void onOdom(const nav_msgs::msg::Odometry& msg)
  {
    last_odom_ = msg;
    have_odom_ = true;
  }

  void autofunction_dealwith_pct_path(const nav_msgs::msg::Path& msg)
  {
    if (msg.poses.empty()) {
      have_pct_path_ = false;
      return;
    }
    if (if_test_pct_path_update_ && isSamePath(msg, last_pct_path_)) {
      return;
    }
    last_pct_path_ = msg;
    have_pct_path_ = true;
    updateGlobalUnfinished();
  }

  bool isSamePath(const nav_msgs::msg::Path& a, const nav_msgs::msg::Path& b) const
  {
    if (a.poses.size() != b.poses.size()) return false;
    if (a.poses.empty()) return true;
    const double tol2 = if_test_pct_path_update_tolerance_ * if_test_pct_path_update_tolerance_;
    for (size_t i = 0; i < a.poses.size(); ++i) {
      const double dx = a.poses[i].pose.position.x - b.poses[i].pose.position.x;
      const double dy = a.poses[i].pose.position.y - b.poses[i].pose.position.y;
      if (dx * dx + dy * dy > tol2) return false;
    }
    return true;
  }

  // 点云回调主入口：
  // - 目的：接收 /lidar_points 点云，统一转换到 head_init 坐标系，并提取可用于2D避障的障碍点。
  // - 输入：msg（原始 PointCloud2，通常来自 vita_lidar 坐标系，含 x/y/z 字段）。
  // - 输出：无直接 return；通过成员变量与话题发布产生结果：
  //   1) 更新 last_obs2d_（供规划器 setObstacles 使用）
  //   2) 更新 filtered_cloud_ 并发布 /lidar_points_filtered（frame_id=head_init）
  //   3) 更新 have_cloud_ / planner_obstacles_dirty_ 状态，触发后续重规划流程
  void autofunction_dealwith_lidar_points(const sensor_msgs::msg::PointCloud2& msg)
  {
    // ============[模块1] 本帧初始化：===========================================================
    // - 记录最新点云 header（保留时间戳/坐标系等元信息，便于后续调试与关联）。
    // - 清空上一帧障碍缓存，确保本次规划只基于当前有效点云结果。
    // - 重置过滤后点云缓存，避免旧数据被误用。
    // 注意：不要在入口处直接清空 last_obs2d_，否则 TF 失败时会把上一帧有效障碍误清空，
    // 触发不必要重规划并加剧规划抖动。改为本地构建成功后再一次性提交。

    // ============[模块2] 输入有效性快速检查：===========================================================
    // 若消息无点数据，直接返回，避免后续 TF 查询与遍历开销。
    if (msg.data.empty()) return;

    // ============[模块3] 坐标系对齐（传感器系 -> head_init）：==========================================
    // - 规划和障碍栅格都工作在 head_init 坐标系，必须先完成统一变换。
    // - resolveCloudTransformToHeadInit 内部会处理变换不可用场景（含最近一次有效变换降级逻辑）。
    // - 若本帧无法获得可用变换，直接丢弃本帧，防止障碍投影到错误位置。
    geometry_msgs::msg::TransformStamped cloud_to_head_tf;
    if (!resolveCloudTransformToHeadInit(msg, cloud_to_head_tf)) {
      return;
    }

    // ============[模块4] 点云遍历与筛选：===========================================================
    // 目标：从原始点云中提取“对 2D 规划有意义”的障碍点，并保留可视化点云。
    //
    // 处理顺序：
    // 1) 读取 x/y/z 字段；
    // 2) 剔除 NaN/Inf 非法点；
    // 3) 将点从传感器坐标系转换到 head_init；
    // 4) 执行全局有效范围过滤（x/y 绝对值范围、z 范围）；
    // 5) 在有效点中再按障碍高度窗 [0.1, 2.0] 提取用于避障的点。
    const double cur_x = have_odom_ ? last_odom_.pose.pose.position.x : 0.0;
    const double cur_y = have_odom_ ? last_odom_.pose.pose.position.y : 0.0;
    std::vector<std::array<float, 3>> kept;
    std::vector<dog_ego_planner::Obstacle2D> new_obs2d;
    // 预留约一半容量，降低动态扩容开销（经验值，避免过度保守）。
    kept.reserve(msg.width * msg.height / 2);
    new_obs2d.reserve(msg.width * msg.height / 2);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> it_x(msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> it_y(msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> it_z(msg, "z");
      const size_t max_points = static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
      constexpr size_t kMaxPointCloudPoints = 10000000;  // 防呆：防止异常大点云导致长时间循环
      const size_t iter_limit = std::min(max_points, kMaxPointCloudPoints);
      size_t point_count = 0;
      for (; it_x != it_x.end() && point_count < iter_limit; ++it_x, ++it_y, ++it_z, ++point_count) {
        Eigen::Vector3d p_sensor(*it_x, *it_y, *it_z);
        if (!std::isfinite(p_sensor.x()) || !std::isfinite(p_sensor.y()) || !std::isfinite(p_sensor.z())) {
          continue;
        }

        // 先做坐标转换，再执行范围与高度过滤，确保后续障碍栅格与规划坐标系一致。
        const Eigen::Vector3d p_head = transformPointCloudToHeadInit(p_sensor, cloud_to_head_tf);
        if (!std::isfinite(p_head.x()) || !std::isfinite(p_head.y()) || !std::isfinite(p_head.z())) continue;
        if (p_head.z() < cloud_valid_z_min_ || p_head.z() > cloud_valid_z_max_) continue;
        const double map_half = grid_map_size_m_ / 2.0;
        const bool in_map = have_odom_
          ? (std::abs(p_head.x() - cur_x) < map_half && std::abs(p_head.y() - cur_y) < map_half)
          : (std::abs(p_head.x()) < map_half && std::abs(p_head.y()) < map_half);
        if (!in_map) continue;

        if (p_head.z() >= 0.1 && p_head.z() <= 2.0) {
          // kept: 用于发布过滤后的 debug 点云（保留 xyz）。
          kept.push_back({static_cast<float>(p_head.x()), static_cast<float>(p_head.y()), static_cast<float>(p_head.z())});
          // last_obs2d_: 规划器输入，仅保留二维障碍位置（x,y）。
          new_obs2d.push_back(dog_ego_planner::Obstacle2D{p_head.x(), p_head.y()});
        }
      }
    } catch (const std::runtime_error& e) {
      // 点云字段异常（如缺少 x/y/z）时节流告警并丢弃本帧，避免刷屏。
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "PointCloud2 iterator error: %s", e.what());
      return;
    }

    // ============[模块5] 构建并发布过滤后点云（调试可视化）：===========================================================
    // - 输出 frame 固定为 head_init（kFrameId），与规划坐标系一致。
    // - 数据来源于 kept，反映“通过筛选后的障碍相关点”。
    sensor_msgs::msg::PointCloud2 out;
    out.header.stamp = msg.header.stamp;
    if (out.header.stamp.sec == 0 && out.header.stamp.nanosec == 0) {
      out.header.stamp = this->now();
    }
    out.header.frame_id = kFrameId;
    out.height = 1;
    out.width = static_cast<uint32_t>(kept.size());
    out.is_bigendian = false;
    out.is_dense = false;
    sensor_msgs::PointCloud2Modifier mod(out);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(kept.size());

    sensor_msgs::PointCloud2Iterator<float> o_x(out, "x");
    sensor_msgs::PointCloud2Iterator<float> o_y(out, "y");
    sensor_msgs::PointCloud2Iterator<float> o_z(out, "z");
    for (size_t i = 0; i < kept.size(); ++i, ++o_x, ++o_y, ++o_z) {
      *o_x = kept[i][0];
      *o_y = kept[i][1];
      *o_z = kept[i][2];
    }

    // ============[模块6] 输出状态回写：===========================================================  
    // - 保存并发布本帧过滤后点云；
    // - 标记已有有效点云数据；
    // - 置位障碍脏标记，驱动下游规划流程在下一周期使用新障碍重算。
    last_cloud_header_ = msg.header;
    last_obs2d_ = std::move(new_obs2d);
    filtered_cloud_ = out;
    pub_cloud_filtered_->publish(out);
    have_cloud_ = true;
    planner_obstacles_dirty_ = true;
  }

  bool resolveCloudTransformToHeadInit(const sensor_msgs::msg::PointCloud2& msg,
                                       geometry_msgs::msg::TransformStamped& tf_out)
  {
    if (!tf_buffer_) return false;

    const std::string source_frame = msg.header.frame_id.empty() ? std::string(kFrameId) : msg.header.frame_id;
    if (source_frame == kFrameId) {
      geometry_msgs::msg::TransformStamped identity;
      identity.header.stamp = msg.header.stamp;
      identity.header.frame_id = kFrameId;
      identity.child_frame_id = kFrameId;
      identity.transform.translation.x = 0.0;
      identity.transform.translation.y = 0.0;
      identity.transform.translation.z = 0.0;
      identity.transform.rotation.x = 0.0;
      identity.transform.rotation.y = 0.0;
      identity.transform.rotation.z = 0.0;
      identity.transform.rotation.w = 1.0;
      tf_out = identity;
      return true;
    }

    const bool msg_stamp_zero = (msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0);
    const rclcpp::Time query_stamp = msg_stamp_zero ? now() : rclcpp::Time(msg.header.stamp);

    try {
      const tf2::Duration timeout = tf2::durationFromSec(static_cast<double>(cloud_tf_timeout_ms_) / 1000.0);
      tf_out = tf_buffer_->lookupTransform(kFrameId, source_frame, query_stamp, timeout);
      last_valid_cloud_tf_ = tf_out;
      return true;
    } catch (const tf2::TransformException& ex) {
      const std::string ex_msg = ex.what();
      const bool disconnected_tree = (ex_msg.find("not part of the same tree") != std::string::npos);
      if (disconnected_tree) {
        // 坐标树断开时，复用旧TF风险极高，直接丢帧更安全。
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "点云TF查找失败（%s -> %s）：%s，检测到TF树断开，禁用历史TF降级并跳过本帧。",
          source_frame.c_str(), kFrameId, ex.what());
        return false;
      }
      if (last_valid_cloud_tf_) {
        const rclcpp::Time last_tf_stamp(last_valid_cloud_tf_->header.stamp);
        const double dt_ms = static_cast<double>((query_stamp - last_tf_stamp).nanoseconds()) / 1e6;
        if (dt_ms < 0.0) {
          // 出现时间回拨（常见于仿真/录包重启）时，旧TF不可复用，避免触发 TF_OLD_DATA 链式问题。
          last_valid_cloud_tf_.reset();
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "检测到时间回拨(%.1fms)，清空历史点云TF并跳过本帧。",
            dt_ms);
          return false;
        }
        const double fallback_age_ms = dt_ms;
        if (cloud_tf_fallback_max_age_ms_ >= 0 &&
            fallback_age_ms > static_cast<double>(cloud_tf_fallback_max_age_ms_)) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "点云TF查找失败（%s -> %s）：%s，且最近有效变换已过期(%.1fms > %dms)，跳过本帧点云。",
            source_frame.c_str(), kFrameId, ex.what(), fallback_age_ms, cloud_tf_fallback_max_age_ms_);
          return false;
        }
        tf_out = *last_valid_cloud_tf_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "点云TF查找失败（%s -> %s）：%s，降级使用最近一次有效变换。",
          source_frame.c_str(), kFrameId, ex.what());
        return true;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "点云TF查找失败（%s -> %s）：%s，且无历史有效变换，跳过本帧点云。",
        source_frame.c_str(), kFrameId, ex.what());
      return false;
    }
  }

  Eigen::Vector3d transformPointCloudToHeadInit(
    const Eigen::Vector3d& p_sensor, const geometry_msgs::msg::TransformStamped& tf_sensor_to_head_init) const
  {
    const auto& t = tf_sensor_to_head_init.transform.translation;
    const auto& q = tf_sensor_to_head_init.transform.rotation;
    const Eigen::Quaterniond q_rot(q.w, q.x, q.y, q.z);
    const Eigen::Vector3d trans(t.x, t.y, t.z);
    return q_rot * p_sensor + trans;
  }

  Eigen::Vector2d currentPoseXY() const
  {
    const auto& p = last_odom_.pose.pose.position;
    return Eigen::Vector2d(p.x, p.y);
  }

  Eigen::Vector2d currentVelXY() const
  {
    const auto& v = last_odom_.twist.twist.linear;
    return Eigen::Vector2d(v.x, v.y);
  }

  void publishEmptyAndStop()
  {
    nav_msgs::msg::Path empty;
    empty.header.stamp = now();
    empty.header.frame_id = kFrameId;
    pub_local_path_->publish(empty);
    pub_global_unfinished_->publish(empty);
  }

  void updateGlobalUnfinished()
  {
    if (!have_odom_ || !have_pct_path_) return;

    // Convert pct_path poses into 2D points (keep original density).
    std::vector<Eigen::Vector2d> pct_pts;
    pct_pts.reserve(last_pct_path_.poses.size());
    for (const auto& ps : last_pct_path_.poses) {
      pct_pts.emplace_back(ps.pose.position.x, ps.pose.position.y);
    }
    if (pct_pts.size() < 2) return;

    const Eigen::Vector2d A = currentPoseXY();
    const size_t idxB = closestIndex(pct_pts, A);
    const double AB = dist2d(A, pct_pts[idxB]);

    // Find C by moving forward from B with length AB along pct_path.
    const InterpOnPath Cinfo = pointAlongPath(pct_pts, idxB, AB);
    const Eigen::Vector2d C = Cinfo.p;

    // Build A->C dense (10cm) if needed.
    std::vector<Eigen::Vector2d> out_pts;
    out_pts.reserve(200);
    out_pts.push_back(A);
    const double AC = dist2d(A, C);
    if (AC >= 0.1 && std::isfinite(AC)) {  // 防呆：AC 非有限则跳过
      const Eigen::Vector2d dir = (C - A) / AC;
      constexpr int kMaxInterpIter = 10000;
      int interp_iter = 0;
      for (double d = 0.1; d < AC && interp_iter < kMaxInterpIter; d += 0.1, ++interp_iter) {
        out_pts.push_back(A + dir * d);
      }
    }
    out_pts.push_back(C);

    // Append unfinished segment after C (including the remaining path points).
    // C lies on segment [i0, i0+1], so append i0+1 ... end
    const size_t start_append = std::min(Cinfo.i0 + 1, pct_pts.size() - 1);
    for (size_t i = start_append; i < pct_pts.size(); ++i) out_pts.push_back(pct_pts[i]);

    // Publish unfinished global path.
    nav_msgs::msg::Path msg;
    msg.header.stamp = now();
    msg.header.frame_id = kFrameId;
    msg.poses.reserve(out_pts.size());
    for (const auto& p : out_pts) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = msg.header;
      ps.pose.position.x = p.x();
      ps.pose.position.y = p.y();
      ps.pose.position.z = 0.0;
      ps.pose.orientation.w = 1.0;
      msg.poses.push_back(ps);
    }
    last_global_unfinished_pts_ = std::move(out_pts);
    pub_global_unfinished_->publish(msg);

    // Goal check (global end).
    const Eigen::Vector2d G = pct_pts.back();
    if (dist2d(A, G) < goal_threshold_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Goal reached, stop planning.");
      planning_finished_ = true;
      publishEmptyAndStop();
    }
  }

  std::vector<Eigen::Vector2d> takeHorizonCurve1(const std::vector<Eigen::Vector2d>& global_unfinished,
                                                 double horizon_m, Eigen::Vector2d& out_temp_goal) const
  {
    std::vector<Eigen::Vector2d> curve;
    if (global_unfinished.size() < 2 || horizon_m <= 0.0 || !std::isfinite(horizon_m)) return curve;  // 防呆
    curve.reserve(100);
    curve.push_back(global_unfinished.front());
    double acc = 0.0;
    constexpr size_t kMaxHorizonIter = 100000;  // 防呆：防止异常长路径死循环
    for (size_t i = 1; i < global_unfinished.size() && (i - 1) < kMaxHorizonIter; ++i) {
      const double seg = (global_unfinished[i] - global_unfinished[i - 1]).norm();
      if (acc + seg >= horizon_m && seg > 1e-9) {
        const double rem = horizon_m - acc;
        const double a = rem / seg;
        const Eigen::Vector2d p = global_unfinished[i - 1] + a * (global_unfinished[i] - global_unfinished[i - 1]);
        curve.push_back(p);
        out_temp_goal = p;
        return curve;
      }
      curve.push_back(global_unfinished[i]);
      acc += seg;
    }
    out_temp_goal = global_unfinished.back();
    return curve;
  }

  std::vector<size_t> sampleControlPointIndices(const std::vector<Eigen::Vector2d>& dense_curve) const
  {
    const size_t n = dense_curve.size();
    std::unordered_set<size_t> keep;
    if (n == 0) return {};
    keep.insert(0);
    keep.insert(n - 1);

    // Sample each 3 dense points (10cm) => ~0.3m.
    for (size_t i = 0; i < n; i += 3) keep.insert(i);
    keep.insert(n - 1);

    // Turn keypoints (>15 deg)
    for (size_t i = 1; i + 1 < n; ++i) {
      const Eigen::Vector2d v1 = dense_curve[i] - dense_curve[i - 1];
      const Eigen::Vector2d v2 = dense_curve[i + 1] - dense_curve[i];
      if (angleDeg(v1, v2) > 15.0) keep.insert(i);
    }

    std::vector<size_t> idx(keep.begin(), keep.end());
    std::sort(idx.begin(), idx.end());

    // Enforce count 20~25 for 7m horizon.
    const auto is_critical = [&](size_t i) -> bool {
      if (i == 0 || i == n - 1) return true;
      if (i >= 1 && i + 1 < n) {
        const Eigen::Vector2d v1 = dense_curve[i] - dense_curve[i - 1];
        const Eigen::Vector2d v2 = dense_curve[i + 1] - dense_curve[i];
        return angleDeg(v1, v2) > 15.0;
      }
      return false;
    };

    if (idx.size() > 25) {
      // Keep all critical points, then fill remaining uniformly.
      std::vector<size_t> critical;
      std::vector<size_t> others;
      for (auto i : idx) (is_critical(i) ? critical : others).push_back(i);
      std::sort(critical.begin(), critical.end());
      std::sort(others.begin(), others.end());
      std::unordered_set<size_t> chosen(critical.begin(), critical.end());
      const size_t remain = (critical.size() >= 25) ? 0 : (25 - critical.size());
      if (remain > 0 && !others.empty()) {
        for (size_t k = 0; k < remain; ++k) {
          const double t = (remain == 1) ? 0.5 : double(k) / double(remain - 1);
          const size_t j = std::min<size_t>(others.size() - 1, static_cast<size_t>(std::llround(t * (others.size() - 1))));
          chosen.insert(others[j]);
        }
      }
      idx.assign(chosen.begin(), chosen.end());
      std::sort(idx.begin(), idx.end());
      if (idx.size() > 25) idx.resize(25);
    } else if (idx.size() < 20) {
      // Add more by denser sampling.
      size_t step = 2;
      constexpr int kMaxSampleRetry = 10;  // 防呆：防止异常情况死循环
      int retry = 0;
      while (idx.size() < 20 && step >= 1 && retry < kMaxSampleRetry) {
        for (size_t i = 0; i < n && idx.size() < 20; i += step) keep.insert(i);
        idx.assign(keep.begin(), keep.end());
        std::sort(idx.begin(), idx.end());
        if (step == 1) break;
        step = 1;
        ++retry;
      }
    }
    return idx;
  }

  nav_msgs::msg::Path toPathMsg(const std::vector<Eigen::Vector2d>& pts) const
  {
    nav_msgs::msg::Path msg;
    msg.header.stamp = now();
    msg.header.frame_id = kFrameId;
    msg.poses.reserve(pts.size());
    for (const auto& p : pts) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = msg.header;
      ps.pose.position.x = p.x();
      ps.pose.position.y = p.y();
      ps.pose.position.z = 0.0;
      ps.pose.orientation.w = 1.0;
      msg.poses.push_back(ps);
    }
    return msg;
  }

  double remainingLocalLength(const std::vector<Eigen::Vector2d>& local_pts, const Eigen::Vector2d& cur) const
  {
    if (local_pts.size() < 2) return 0.0;
    const size_t idx = closestIndex(local_pts, cur);
    double L = 0.0;
    for (size_t i = std::max<size_t>(1, idx + 1); i < local_pts.size(); ++i) {
      L += (local_pts[i] - local_pts[i - 1]).norm();
    }
    return L;
  }

  void publishOccupancyMap()
  {
    std::vector<int8_t> data;
    int width = 0, height = 0;
    double resolution = 0.0;
    Eigen::Vector2d origin = Eigen::Vector2d::Zero();
    if (!planner_.getInflatedOccupancyGrid(data, width, height, resolution, origin)) return;

    nav_msgs::msg::OccupancyGrid map_msg;
    map_msg.header.stamp = now();
    map_msg.header.frame_id = kFrameId;
    map_msg.info.resolution = static_cast<float>(resolution);
    map_msg.info.width = static_cast<uint32_t>(width);
    map_msg.info.height = static_cast<uint32_t>(height);
    map_msg.info.origin.position.x = origin.x();
    map_msg.info.origin.position.y = origin.y();
    map_msg.info.origin.position.z = 0.0;
    map_msg.info.origin.orientation.w = 1.0;
    map_msg.data = std::move(data);
    pub_occupancy_map_->publish(map_msg);
  }

  bool poseChanged() const
  {
    if (!last_plan_pose_) return true;
    return dist2d(currentPoseXY(), *last_plan_pose_) > pose_change_thresh_;
  }

  bool obstaclesChanged() const
  {
    if (!last_plan_obstacles_) return true;
    if (!have_cloud_) return false;

    // Change metric:
    // 1) symmetric difference on quantized cells > 5%
    // 2) centroid shift > obs_change_thresh_
    const double res = std::max(0.05, obs_change_thresh_);
    const auto s1 = quantizeSet(*last_plan_obstacles_, res);
    const auto s2 = quantizeSet(last_obs2d_, res);
    size_t diff = 0;
    for (const auto& k : s1) if (!s2.count(k)) diff++;
    for (const auto& k : s2) if (!s1.count(k)) diff++;
    const size_t base = std::max<size_t>(1, std::max(s1.size(), s2.size()));
    const double diff_ratio = double(diff) / double(base);
    if (diff_ratio > 0.05) return true;

    auto centroid = [](const std::vector<dog_ego_planner::Obstacle2D>& obs) -> Eigen::Vector2d {
      if (obs.empty()) return Eigen::Vector2d::Zero();
      Eigen::Vector2d c(0, 0);
      for (const auto& o : obs) c += Eigen::Vector2d(o.x, o.y);
      return c / double(obs.size());
    };
    const Eigen::Vector2d c1 = centroid(*last_plan_obstacles_);
    const Eigen::Vector2d c2 = centroid(last_obs2d_);
    return (c2 - c1).norm() > obs_change_thresh_;
  }

  std::vector<dog_ego_planner::Obstacle2D> filterObstaclesNearRobot(
    const std::vector<dog_ego_planner::Obstacle2D>& obstacles, const Eigen::Vector2d& cur) const
  {
    if (self_obstacle_clear_radius_ <= 0.0 || obstacles.empty()) return obstacles;
    if (!std::isfinite(cur.x()) || !std::isfinite(cur.y())) return obstacles;  // 防呆：非法位姿
    const double r2 = self_obstacle_clear_radius_ * self_obstacle_clear_radius_;
    std::vector<dog_ego_planner::Obstacle2D> filtered;
    filtered.reserve(obstacles.size());
    for (const auto& o : obstacles) {
      const double dx = o.x - cur.x();
      const double dy = o.y - cur.y();
      if (dx * dx + dy * dy > r2) filtered.push_back(o);
    }
    return filtered;
  }

  std::vector<Eigen::Vector2d> buildCurrentUnfinished(const Eigen::Vector2d& cur) const
  {
    if (last_global_unfinished_pts_.size() < 2) return {};
    const size_t idx = closestIndex(last_global_unfinished_pts_, cur);
    std::vector<Eigen::Vector2d> result;
    result.reserve(1 + (last_global_unfinished_pts_.size() - idx));
    result.push_back(cur);
    for (size_t i = idx; i < last_global_unfinished_pts_.size(); ++i) {
      result.push_back(last_global_unfinished_pts_[i]);
    }
    return result;
  }

  // 定时器回调：判断重规划触发条件，并执行一次局部重规划。
  void dataWaitStatusTick()
  {
    if (have_odom_ && have_pct_path_) return;
    const auto elapsed_s =
      std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - data_wait_start_time_).count();
    if (!have_odom_ && !have_pct_path_) {
      std::cout << "等待 /odometry 和 /pct_path 数据，已等待 " << elapsed_s << "s" << std::endl;
    } else if (have_odom_ && !have_pct_path_) {
      std::cout << "已经收到 /odometry，等待 /pct_path 数据，已等待 " << elapsed_s << "s" << std::endl;
    } else {
      std::cout << "已经收到 /pct_path，等待 /odometry 数据，已等待 " << elapsed_s << "s" << std::endl;
    }
  }

  void replanTick()
  {
    if (planning_finished_) return;
    if (!have_odom_ || !have_pct_path_) return;
    if (last_global_unfinished_pts_.size() < 2) return;
    const auto steady_now = std::chrono::steady_clock::now();
    if (steady_now < replan_cooldown_until_) return;

    const Eigen::Vector2d cur = currentPoseXY();

    const bool trigger_by_pose = poseChanged();
    const bool trigger_by_obs = obstaclesChanged();
    const bool trigger_by_remaining = (remainingLocalLength(last_local_path_pts_, cur) < 4.0);

    if (!trigger_by_pose && !trigger_by_obs && !trigger_by_remaining && have_local_plan_) return;

    // 显眼提示：当前定时周期触发并正式进入一次重规划流程。
    const double replan_period_ms = 1000.0 / std::max(1.0, replan_freq_);
    ++replan_count_;
    std::cout << "================ [重规划开始#" << replan_count_ << "] 周期=" << static_cast<int>(std::round(replan_period_ms))
              << "ms | 位姿触发=" << (trigger_by_pose ? "是" : "否")
              << " 障碍触发=" << (trigger_by_obs ? "是" : "否")
              << " 剩余距离触发=" << (trigger_by_remaining ? "是" : "否")
              << " ================" << std::endl;

    // 用当前 odom 裁剪全局未完成路径：从最近点开始，前面补上当前位置，
    // 保证局部规划起点始终跟随 /odometry，而不依赖 updateGlobalUnfinished 的调用时机。
    const std::vector<Eigen::Vector2d> current_unfinished = buildCurrentUnfinished(cur);
    if (current_unfinished.size() < 2) return;

    // Build curve1: 7m horizon from current A along unfinished path.
    Eigen::Vector2d temp_goal = current_unfinished.back();
    const std::vector<Eigen::Vector2d> curve1 = takeHorizonCurve1(current_unfinished, planning_horizon_, temp_goal);
    if (curve1.size() < 4) return;

    // Control points sampling (paper rule): every 3 dense points, keep key turns.
    const std::vector<size_t> ctrl_idx = sampleControlPointIndices(curve1);
    std::vector<dog_ego_planner::PathPoint2D> ctrl_pts;
    ctrl_pts.reserve(ctrl_idx.size());
    const size_t curve1_size = curve1.size();
    for (auto i : ctrl_idx) {
      if (i < curve1_size) ctrl_pts.push_back({curve1[i].x(), curve1[i].y()});  // 防呆：避免越界
    }

    // Feed planner.
    planner_.setCurrentPose(dog_ego_planner::PathPoint2D{cur.x(), cur.y()});
    // 仅在点云更新后重建障碍地图，避免位姿触发时重复做重活导致CPU飙高。
    if (planner_obstacles_dirty_) {
      // 保护：剔除机器人自身附近小半径障碍，避免“起点在障碍物内”导致重规划风暴。
      const auto planning_obs = filterObstaclesNearRobot(last_obs2d_, cur);
      planner_.setObstacles(planning_obs);
      bool should_publish_occ = true;
      if (occupancy_publish_freq_ > 0.0) {
        const auto now_steady = std::chrono::steady_clock::now();
        const int interval_ms = static_cast<int>(std::round(1000.0 / occupancy_publish_freq_));
        const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          now_steady - last_occupancy_publish_time_).count();
        should_publish_occ = (elapsed_ms >= interval_ms);
        if (should_publish_occ) last_occupancy_publish_time_ = now_steady;
      }
      if (should_publish_occ) publishOccupancyMap();
      planner_obstacles_dirty_ = false;
    }
    planner_.setReferencePath(ctrl_pts);

    const Eigen::Vector2d start_vel = currentVelXY();
    const Eigen::Vector2d start_acc(0.0, 0.0);
    const Eigen::Vector2d goal_vel(0.0, 0.0);

    const bool ok = planner_.makePlan(start_vel, start_acc, goal_vel);
    if (!ok) {
      ++consecutive_plan_failures_;
      if (consecutive_plan_failures_ >= std::max(1, max_consecutive_failures_before_cooldown_)) {
        replan_cooldown_until_ =
          steady_now + std::chrono::milliseconds(std::max(0, replan_failure_cooldown_ms_));
        RCLCPP_WARN(
          get_logger(),
          "连续规划失败达到阈值(%d次)，进入冷却%dms以避免重规划风暴。",
          max_consecutive_failures_before_cooldown_, replan_failure_cooldown_ms_);
        consecutive_plan_failures_ = 0;
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Local planning failed, reuse last local path.");
      // 失败也记录一次当前状态快照，避免同一状态在高频定时器下被无意义重复触发。
      last_plan_pose_ = cur;
      last_plan_obstacles_ = last_obs2d_;
      // Degrade: publish last local path again.
      pub_local_path_->publish(toPathMsg(last_local_path_pts_));
      return;
    }
    consecutive_plan_failures_ = 0;
    replan_cooldown_until_ = std::chrono::steady_clock::time_point::min();

    std::vector<dog_ego_planner::PathPoint2D> traj2d;
    planner_.getPlannedTraj(traj2d, 0.1);
    std::vector<Eigen::Vector2d> curve2;
    curve2.reserve(traj2d.size());
    for (const auto& p : traj2d) curve2.emplace_back(p.x, p.y);

    if (curve2.size() < 2) return;

    // Publish local path.
    last_local_path_pts_ = curve2;
    have_local_plan_ = true;
    pub_local_path_->publish(toPathMsg(curve2));

    // Update unfinished global by replacing the first horizon segment with curve2 (simple splice).
    // Find where to splice: the end of curve1 corresponds to temp_goal; we splice at closest point in global_unfinished.
    size_t splice_idx = closestIndex(last_global_unfinished_pts_, temp_goal);
    std::vector<Eigen::Vector2d> new_global;
    new_global.reserve(curve2.size() + (last_global_unfinished_pts_.size() - splice_idx));
    new_global.insert(new_global.end(), curve2.begin(), curve2.end());
    for (size_t i = splice_idx; i < last_global_unfinished_pts_.size(); ++i) new_global.push_back(last_global_unfinished_pts_[i]);
    last_global_unfinished_pts_ = new_global;
    pub_global_unfinished_->publish(toPathMsg(last_global_unfinished_pts_));

    // Update planning state snapshot.
    last_plan_pose_ = cur;
    last_plan_obstacles_ = last_obs2d_;
  }

private:
  // Params
  bool debug_{true};
  double goal_threshold_{0.1};
  double pose_change_thresh_{0.05};
  double obs_change_thresh_{0.1};
  double planning_horizon_{7.0};
  double control_point_interval_{0.3};
  double replan_freq_{50.0};
  double local_traj_duration_{1.0};
  double max_vel_{1.0};
  double max_acc_{2.0};
  double max_jerk_{5.0};
  bool printf_open_or_not_{true};
  int max_rebound_retries_{5};
  int cloud_tf_timeout_ms_{100};
  double cloud_valid_x_abs_max_{200.0};
  double cloud_valid_y_abs_max_{200.0};
  double cloud_valid_z_min_{-3.0};
  double cloud_valid_z_max_{5.0};
  double grid_map_size_m_{40.0};
  double grid_map_resolution_{0.1};
  double grid_inflate_radius_{0.20};
  int astar_pool_size_{100};
  double occupancy_publish_freq_{-1.0};
  double self_obstacle_clear_radius_{0.45};
  int cloud_tf_fallback_max_age_ms_{500};
  double global_path_update_interval_s_{1.0};
  bool if_test_pct_path_update_{true};
  double if_test_pct_path_update_tolerance_{0.01};
  int replan_failure_cooldown_ms_{500};
  int max_consecutive_failures_before_cooldown_{3};

  // Subs & pubs
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_filtered_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_global_unfinished_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_local_path_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_occupancy_map_;

  rclcpp::TimerBase::SharedPtr replan_timer_;
  rclcpp::TimerBase::SharedPtr global_path_update_timer_;
  rclcpp::TimerBase::SharedPtr data_wait_timer_;
  std::chrono::steady_clock::time_point data_wait_start_time_;

  // Latest inputs
  nav_msgs::msg::Odometry last_odom_{};
  nav_msgs::msg::Path last_pct_path_{};
  std_msgs::msg::Header last_cloud_header_{};
  std::optional<sensor_msgs::msg::PointCloud2> filtered_cloud_;
  bool have_odom_{false};
  bool have_pct_path_{false};
  bool have_cloud_{false};
  bool planner_obstacles_dirty_{true};
  std::optional<geometry_msgs::msg::TransformStamped> last_valid_cloud_tf_;

  std::vector<dog_ego_planner::Obstacle2D> last_obs2d_;

  // State
  bool planning_finished_{false};
  bool have_local_plan_{false};
  uint64_t replan_count_{0};
  std::vector<Eigen::Vector2d> last_global_unfinished_pts_;
  std::vector<Eigen::Vector2d> last_local_path_pts_;
  std::optional<Eigen::Vector2d> last_plan_pose_;
  std::optional<std::vector<dog_ego_planner::Obstacle2D>> last_plan_obstacles_;
  int consecutive_plan_failures_{0};
  std::chrono::steady_clock::time_point replan_cooldown_until_{
    std::chrono::steady_clock::time_point::min()};
  std::chrono::steady_clock::time_point last_occupancy_publish_time_{
    std::chrono::steady_clock::time_point::min()};

  dog_ego_planner::PlannerInterfaceDog planner_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DogPlannerNode>());
  rclcpp::shutdown();
  return 0;
}

