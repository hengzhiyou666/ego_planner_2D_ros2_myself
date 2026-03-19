#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <optional>
#include <unordered_set>
#include <vector>

#include "dog_ego_planner/planner_interface_dog.h"

namespace
{

constexpr const char* kFrameId = "head_init";

double dist2d(const Eigen::Vector2d& a, const Eigen::Vector2d& b) { return (a - b).norm(); }

// Find index of closest point.
size_t closestIndex(const std::vector<Eigen::Vector2d>& pts, const Eigen::Vector2d& q)
{
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
  for (size_t i = start_idx; i + 1 < pts.size(); ++i) {
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
  out.i0 = pts.size() - 2;
  out.alpha = 1.0;
  return out;
}

// Build a simple hash-set from points for obstacle-change detection.
int64_t hashKey(double x, double y, double res)
{
  const int64_t ix = static_cast<int64_t>(std::llround(x / res));
  const int64_t iy = static_cast<int64_t>(std::llround(y / res));
  return (ix << 32) ^ (iy & 0xffffffff);
}

std::unordered_set<int64_t> quantizeSet(const std::vector<dog_ego_planner::Obstacle2D>& obs, double res)
{
  std::unordered_set<int64_t> s;
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

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry", rclcpp::QoS(10),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { onOdom(*msg); });

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/pct_path", rclcpp::QoS(10),
      [this](nav_msgs::msg::Path::SharedPtr msg) { onPctPath(*msg); });

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar_points", rclcpp::QoS(10),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { onCloud(*msg); });

    pub_cloud_filtered_ = create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_points_filtered", 10);
    pub_global_unfinished_ = create_publisher<nav_msgs::msg::Path>("/dog_output_global_path_unfinished", 10);
    pub_local_path_ = create_publisher<nav_msgs::msg::Path>("/dog_output_local_path", 10);

    // Init planner core
    planner_.initParam(max_vel_, max_acc_, max_jerk_);
    planner_.initGridMap(
      200.0, 200.0, 0.1, Eigen::Vector2d(-100.0, -100.0),
      0.20 /*inflate_radius*/);
    planner_.setPrintfOpenOrNot(printf_open_or_not_);
    planner_.setMaxReboundRetries(max_rebound_retries_);

    global_timer_ = create_wall_timer(std::chrono::milliseconds(200), [this] { updateGlobalUnfinished(); });

    const int replan_ms = static_cast<int>(std::round(1000.0 / std::max(1.0, replan_freq_)));
    replan_timer_ = create_wall_timer(std::chrono::milliseconds(replan_ms), [this] { replanTick(); });
  }

private:
  void onOdom(const nav_msgs::msg::Odometry& msg)
  {
    last_odom_ = msg;
    have_odom_ = true;
  }

  void onPctPath(const nav_msgs::msg::Path& msg)
  {
    last_pct_path_ = msg;
    have_pct_path_ = !msg.poses.empty();
  }

  void onCloud(const sensor_msgs::msg::PointCloud2& msg)
  {
    last_cloud_header_ = msg.header;
    last_obs2d_.clear();
    filtered_cloud_.reset();

    // Filter: keep 0.1m <= z <= 2.0m; store x/y only.
    if (msg.data.empty()) return;

    // Iterator-based parsing/building (assume x/y/z exist).
    std::vector<std::array<float, 3>> kept;
    kept.reserve(msg.width * msg.height / 2);
    try {
      sensor_msgs::PointCloud2ConstIterator<float> it_x(msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> it_y(msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> it_z(msg, "z");
      for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
        const float x = *it_x;
        const float y = *it_y;
        const float z = *it_z;
        if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && z >= 0.1f && z <= 2.0f) {
          kept.push_back({x, y, z});
          last_obs2d_.push_back(dog_ego_planner::Obstacle2D{static_cast<double>(x), static_cast<double>(y)});
        }
      }
    } catch (const std::runtime_error& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "PointCloud2 iterator error: %s", e.what());
      return;
    }

    // Publish filtered cloud for debug visualization.
    sensor_msgs::msg::PointCloud2 out;
    out.header.stamp = now();
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
    filtered_cloud_ = out;
    pub_cloud_filtered_->publish(out);
    have_cloud_ = true;
    planner_obstacles_dirty_ = true;
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
    if (AC >= 0.1) {
      const Eigen::Vector2d dir = (C - A) / AC;
      for (double d = 0.1; d < AC; d += 0.1) {
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
    if (global_unfinished.size() < 2) return curve;
    curve.reserve(100);
    curve.push_back(global_unfinished.front());
    double acc = 0.0;
    for (size_t i = 1; i < global_unfinished.size(); ++i) {
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
      while (idx.size() < 20 && step >= 1) {
        for (size_t i = 0; i < n && idx.size() < 20; i += step) keep.insert(i);
        idx.assign(keep.begin(), keep.end());
        std::sort(idx.begin(), idx.end());
        if (step == 1) break;
        step = 1;
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

  // 定时器回调：判断重规划触发条件，并执行一次局部重规划。
  void replanTick()
  {
    if (planning_finished_) return;
    if (!have_odom_ || !have_pct_path_) return;
    if (last_global_unfinished_pts_.size() < 2) return;

    const Eigen::Vector2d cur = currentPoseXY();

    const bool trigger_by_pose = poseChanged();
    const bool trigger_by_obs = obstaclesChanged();
    const bool trigger_by_remaining = (remainingLocalLength(last_local_path_pts_, cur) < 4.0);

    if (!trigger_by_pose && !trigger_by_obs && !trigger_by_remaining && have_local_plan_) return;

    // 显眼提示：当前定时周期触发并正式进入一次重规划流程。
    const double replan_period_ms = 1000.0 / std::max(1.0, replan_freq_);
    std::cout << "================ [重规划开始] 周期=" << static_cast<int>(std::round(replan_period_ms))
              << "ms | 位姿触发=" << (trigger_by_pose ? "是" : "否")
              << " 障碍触发=" << (trigger_by_obs ? "是" : "否")
              << " 剩余距离触发=" << (trigger_by_remaining ? "是" : "否")
              << " ================" << std::endl;

    // Build curve1: 7m horizon from current A along unfinished path.
    Eigen::Vector2d temp_goal = last_global_unfinished_pts_.back();
    const std::vector<Eigen::Vector2d> curve1 = takeHorizonCurve1(last_global_unfinished_pts_, planning_horizon_, temp_goal);
    if (curve1.size() < 4) return;

    // Control points sampling (paper rule): every 3 dense points, keep key turns.
    const std::vector<size_t> ctrl_idx = sampleControlPointIndices(curve1);
    std::vector<dog_ego_planner::PathPoint2D> ctrl_pts;
    ctrl_pts.reserve(ctrl_idx.size());
    for (auto i : ctrl_idx) ctrl_pts.push_back({curve1[i].x(), curve1[i].y()});

    // Feed planner.
    planner_.setCurrentPose(dog_ego_planner::PathPoint2D{cur.x(), cur.y()});
    // 仅在点云更新后重建障碍地图，避免位姿触发时重复做重活导致CPU飙高。
    if (planner_obstacles_dirty_) {
      planner_.setObstacles(last_obs2d_);
      planner_obstacles_dirty_ = false;
    }
    planner_.setReferencePath(ctrl_pts);

    const Eigen::Vector2d start_vel = currentVelXY();
    const Eigen::Vector2d start_acc(0.0, 0.0);
    const Eigen::Vector2d goal_vel(0.0, 0.0);

    const bool ok = planner_.makePlan(start_vel, start_acc, goal_vel);
    if (!ok) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Local planning failed, reuse last local path.");
      // Degrade: publish last local path again.
      pub_local_path_->publish(toPathMsg(last_local_path_pts_));
      return;
    }

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

  // Subs & pubs
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_filtered_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_global_unfinished_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_local_path_;

  rclcpp::TimerBase::SharedPtr global_timer_;
  rclcpp::TimerBase::SharedPtr replan_timer_;

  // Latest inputs
  nav_msgs::msg::Odometry last_odom_{};
  nav_msgs::msg::Path last_pct_path_{};
  std_msgs::msg::Header last_cloud_header_{};
  std::optional<sensor_msgs::msg::PointCloud2> filtered_cloud_;
  bool have_odom_{false};
  bool have_pct_path_{false};
  bool have_cloud_{false};
  bool planner_obstacles_dirty_{true};

  std::vector<dog_ego_planner::Obstacle2D> last_obs2d_;

  // State
  bool planning_finished_{false};
  bool have_local_plan_{false};
  std::vector<Eigen::Vector2d> last_global_unfinished_pts_;
  std::vector<Eigen::Vector2d> last_local_path_pts_;
  std::optional<Eigen::Vector2d> last_plan_pose_;
  std::optional<std::vector<dog_ego_planner::Obstacle2D>> last_plan_obstacles_;

  dog_ego_planner::PlannerInterfaceDog planner_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DogPlannerNode>());
  rclcpp::shutdown();
  return 0;
}

