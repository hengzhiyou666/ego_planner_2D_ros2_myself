# dog_ego_planner

`dog_ego_planner` 是面向四足机器人/机器狗的 ROS 2 Humble 二维局部路径规划包。节点接收里程计、全局参考路径和点云，在二维占用栅格上进行局部重规划，并发布可供下游控制器跟踪的局部 `nav_msgs/Path`。

> [!WARNING]
> 规划节点不是完整导航栈，也不发布速度指令。仓库同时安装一个默认双重锁定的配套路径跟踪控制器，但它不负责执行器制动、物理急停、动态障碍预测或整机功能安全。`nav_msgs/Path` 不包含可直接执行的速度/时间约束。真机使用前必须完成仿真、rosbag 回放、架空低速和带物理急停的封闭场地测试。

## 功能边界

主要能力：

- 将障碍点云转换到规划坐标系，过滤后构建滚动二维占用栅格。
- 沿全局路径维护单调进度并截取局部规划区间。
- 在安全走廊无障碍时生成直通路径；否则使用 A* 与 B-spline/EGO 风格优化重规划。
- 在输入无效、数据过期、TF 不可用或规划失败时发布停车/无效路径，避免继续输出未经验证的旧路径。
- 发布局部路径、未完成全局路径、过滤点云、占用栅格和目标到达状态。

不在本包范围内：

- 全局路径生成与任务调度。
- 路径跟踪、`cmd_vel`/`vel_cmd` 生成和底盘控制。
- 符合功能安全等级的急停、避碰认证或动态目标行为预测。
- Nav2 行为树、恢复行为和完整生命周期管理。

## 环境

- Ubuntu 22.04
- ROS 2 Humble
- C++17 编译器
- CMake 3.21 或更高版本
- Eigen3

## 构建

在本仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src --rosdistro humble -r -y
./scripts/build.sh
source install/setup.bash
```

`scripts/build.sh` 默认执行增量 `RelWithDebInfo` 构建。可使用 `--debug` 或 `--release` 选择构建类型；只有显式传入 `--clean` 时才会删除当前仓库内的 `build/`、`install/` 和 `log/`。

## 启动

无需手动 `source` 的一键启动方式：

```bash
./1_start_planner.sh
```

本地同时启动 RViz：

```bash
./1_start_planner.sh launch_rviz:=true
```

使用 rosbag 的 `/clock` 时：

```bash
./1_start_planner.sh use_sim_time:=true launch_rviz:=true
```

脚本会自动识别本地和 dog3 的 ROS 2 Humble 安装目录，并加载当前工程的
`install/setup.bash`。dog3上会优先复用`/userdata/1_slam/setup_dog3_env.sh`，确保
SLAM、规划器和控制器使用同一Zenoh环境。它只启动规划器，不启动
`dog_move_forward.py`，也不发布速度命令。可以先执行`./1_start_planner.sh --check`，
只检查环境而不启动节点。

推荐入口：

```bash
ros2 launch dog_ego_planner dog_ego_planner.launch.py \
  use_sim_time:=false \
  launch_rviz:=false
```

使用自定义参数文件：

```bash
ros2 launch dog_ego_planner dog_ego_planner.launch.py \
  params_file:=/absolute/path/to/planner_params.yaml
```

Launch 参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `params_file` | 包内 `config/planner_params.yaml` | 节点参数文件 |
| `use_sim_time` | `false` | 是否使用 `/clock`；真机应保持 `false` |
| `launch_rviz` | `false` | 是否加载包内 RViz 配置 |
| `namespace` | 空 | 可选 ROS namespace，用于多机器人隔离 |
| `node_name` | `dog_ego_planner` | 节点名；可执行程序仍为 `dog_planner_node` |

`robot_launch.py` 作为一个主版本周期内的兼容入口保留，新部署应改用 `dog_ego_planner.launch.py`。Launch 文件不解析 YAML；`launch_rviz` 和 `use_sim_time` 均由 launch 参数显式控制。

## ROS 接口

默认话题名使用相对名称，因此在根命名空间下仍解析为下表中的 `/...`，同时支持通过 ROS namespace 隔离多机器人实例。

当前 dog3 自研链路使用根命名空间，启动时不要单独添加 `namespace:=dog3`。只有
SLAM、路径发布器、规划器和控制器全部采用同一 namespace 或完成显式 remap 后，
才能启用 namespace 隔离。

### 订阅

| 参数 | 默认话题 | 类型 | 用途 |
|---|---|---|---|
| `topics.odom` | `/location_now` | `nav_msgs/msg/Odometry` | 自研 SLAM 输出的当前位姿和速度 |
| `topics.global_path` | `/pct_path_copy` | `nav_msgs/msg/Path` | 自研全局参考路径 |
| `topics.point_cloud` | `/lidar_points_copy` | `sensor_msgs/msg/PointCloud2` | 转入自研坐标体系的障碍点云 |

### 发布

| 参数 | 默认话题 | 类型 | 用途 |
|---|---|---|---|
| `topics.local_path` | `/dog_output_local_path_copy` | `nav_msgs/msg/Path` | 下游自研控制器使用的局部几何路径 |
| `topics.unfinished_global_path` | `/dog_output_global_path_unfinished_copy` | `nav_msgs/msg/Path` | 尚未完成的全局路径段 |
| `topics.filtered_point_cloud` | `/lidar_points_filtered_copy` | `sensor_msgs/msg/PointCloud2` | 转换、过滤后的点云 |
| `topics.occupancy_grid` | `/dog_2Dmap_occupancy_copy` | `nav_msgs/msg/OccupancyGrid` | 二维规划栅格，编码为自由 `0`、占用 `100`、未知 `-1` |
| `topics.goal_reached` | `/goal_reached_copy` | `std_msgs/msg/Bool` | 规范的目标到达状态 |
| `topics.goal_reached_legacy` | `/if_reach_the_goal_copy` | `std_msgs/msg/Int32` | 旧接口兼容输出；新系统请使用 `/goal_reached_copy` |

节点默认在 `planning_frame=local_map_lidar_init_xyz` 中规划。输入消息若不在该坐标系，必须能够通过 TF 转换；空 frame、无效四元数、过期/倒退的传感器时间戳以及超龄 TF 回退都会使节点进入停车状态。`/dog_output_local_path_copy`、未完成全局路径、过滤点云和占据栅格等带坐标信息的规划输出都会使用该 `planning_frame`。生产部署应确认时间源、TF 树和消息时间戳一致。

默认至少需要 10 个可正常变换的点云输入点，并要求点云与最近里程计时间差不超过 0.1 秒。空点云、全非有限点或时间不同步都会使规划器停止。该检查只能判断消息基本健康，不等同于通过功能安全认证的自由空间检测。

里程计仍新鲜时，停车输出是当前位置的两个重合 Pose；里程计已经过期或无效时发布空路径，避免下游跟踪器把旧坐标当成新目标。无论哪种情况，实际制动都必须由下游控制器和独立 watchdog 保证。

旧 `Int32` 目标接口在“没有有效全局路径”时默认输出 `0`。如旧系统确实依赖原先的 `1`，可显式设置 `compatibility.legacy_no_path_is_reached=true`；生产使用前必须确认任务管理器不会把无效任务误判为完成。

自研定位话题`/location_now`默认使用 reliable QoS，全局路径`/pct_path_copy`默认使用 reliable + transient-local QoS；如上游发布者策略不同，可分别通过 `odometry_use_best_effort_qos` 和 `pct_path_match_nav2_qos` 调整。

规划输入输出统一追加`_copy`，避免厂家SLAM、导航或App节点使用同名话题时混入自研链路。

## 参数兼容

参数定义和推荐值见 [`config/planner_params.yaml`](config/planner_params.yaml)。新参数采用分组命名；以下旧参数在一个主版本周期内保持兼容并产生弃用提示：

| 旧参数 | 新参数 |
|---|---|
| `topic_odom` | `topics.odom` |
| `topic_pct_path` | `topics.global_path` |
| `topic_lidar` | `topics.point_cloud` |
| `step` | `planning.horizon_step_m` |
| `printfOpenOrNot` | `debug.optimizer_logging` |
| `if_filter_6m` | `direct_path.smoothing_enabled` |
| `filter_6m_rounds` | `direct_path.smoothing_iterations` |
| `if_test_pct_path_update` | `global_path.skip_duplicate_updates` |
| `if_continuously_plan_failed_time` | `safety.failure_stop_timeout_s` |

输出话题同样可通过 `topics.*` 参数配置，不应在下游系统中依赖硬编码名称。

`max_vel`、`max_acc` 和 `max_jerk` 按二维向量模长检查。规划器会验证完整 B-spline 时域、终点和最终发布路径，不允许无效的新任务复用上一次优化状态。

## 验证

```bash
colcon test --packages-select dog_ego_planner --event-handlers console_direct+
colcon test-result --verbose
```

每次真机部署至少验证：空路径、点云断流、里程计超时、TF 失败、窄通道、突然出现障碍、规划失败停车和目标到达。实际速度、角速度与制动限制必须由下游控制器独立执行。

在已运行控制系统的机器狗上做软件验收时执行 `scripts/test_isolated.sh`。脚本会在加载工作区后强制覆盖调用者环境，固定使用 `ROS_DOMAIN_ID=232`、`ROS_LOCALHOST_ONLY=1`、Fast DDS 和测试 namespace；CTest 中的 ROS 测试也带有相同隔离环境。详细流程见 [`docs/DEPLOYMENT_DOG3.md`](docs/DEPLOYMENT_DOG3.md)。

## 参与贡献

开发约定、测试要求和安全检查清单见 [`CONTRIBUTING.md`](CONTRIBUTING.md)。版本变化见 [`CHANGELOG.rst`](CHANGELOG.rst)。

## 许可证与上游来源

本组合工程以 `GPL-3.0-only` 发布，完整条款见 [`LICENSES/GPL-3.0-only.txt`](LICENSES/GPL-3.0-only.txt)。部分规划实现源自 ZJU FAST Lab 的 EGO-Planner；`lbfgs.hpp` 源自 LBFGS-Lite，并单独遵循 MIT 许可证。详细归属和修改说明见 [`LICENSES/THIRD_PARTY_NOTICES.md`](LICENSES/THIRD_PARTY_NOTICES.md)。
