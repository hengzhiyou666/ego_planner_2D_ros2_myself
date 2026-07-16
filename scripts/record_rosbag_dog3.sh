#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-only

# dog3 的建图/导航数据录制脚本。
# 默认只订阅数据，不发布控制命令，不会控制机器狗移动。

set -eo pipefail

readonly RECORD_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly SLAM_DIR="${SLAM_DIR:-/userdata/1_slam}"

usage() {
  cat <<'EOF'
用法：
  ./record_rosbag.sh           录制建图和导航所需的标准话题
  ./record_rosbag.sh --check   只检查话题，不开始录制
  ./record_rosbag.sh --all     录制当前 ROS 系统的全部非隐藏话题（数据量很大）

开始录制后，在当前终端按 Ctrl+C 正常停止并保存 rosbag。
EOF
}

mode="standard"
case "${1:-}" in
  "")
    ;;
  --check)
    mode="check"
    ;;
  --all)
    mode="all"
    ;;
  -h|--help)
    usage
    exit 0
    ;;
  *)
    echo "未知参数：$1" >&2
    usage >&2
    exit 2
    ;;
esac

# 复用建图工程的环境脚本，确保 rosbag 使用与 Fast-LIO 相同的 ROS 2、
# 自定义消息和 Zenoh 通信配置。
if [[ ! -f "${SLAM_DIR}/setup_dog3_env.sh" ]]; then
  echo "错误：未找到 ${SLAM_DIR}/setup_dog3_env.sh" >&2
  exit 2
fi

set +u
source "${SLAM_DIR}/setup_dog3_env.sh"
if [[ -f "${SLAM_DIR}/install/local_setup.bash" ]]; then
  source "${SLAM_DIR}/install/local_setup.bash"
fi
set -u

export ROS2CLI_NO_DAEMON=1

# Zenoh 可能短时间保留已经停止的话题名称，所以不能只判断“名字是否存在”。
# 这里进一步检查 Publisher count，确认真的有节点正在发布数据。
publisher_count() {
  local information
  information="$(
    ros2 topic info "$1" --no-daemon --spin-time 3 2>/dev/null || true
  )"
  awk '/Publisher count:/ {print $3; exit}' <<<"${information}"
}

echo "正在检查自研SLAM核心话题是否有真实发布者，请稍候……"
location_publishers="$(publisher_count /location_now)"
mapping_publishers="$(publisher_count /state_estimation_copy)"
imu_publishers="$(publisher_count /lidar_imu_copy)"
lidar_publishers="$(publisher_count /lidar_points_copy)"
location_publishers="${location_publishers:-0}"
mapping_publishers="${mapping_publishers:-0}"
imu_publishers="${imu_publishers:-0}"
lidar_publishers="${lidar_publishers:-0}"

echo "定位全局位姿 /location_now：${location_publishers} 个发布者"
echo "建图位姿 /state_estimation_copy：${mapping_publishers} 个发布者"
echo "适配雷达 IMU /lidar_imu_copy：${imu_publishers} 个发布者"
echo "规划点云      /lidar_points_copy：${lidar_publishers} 个发布者"

if (((location_publishers + mapping_publishers) < 1 || imu_publishers < 1 || lidar_publishers < 1)); then
  echo "错误：定位/建图位姿、适配IMU、适配雷达没有全部启动。" >&2
  echo "请先启动自研建图或定位，再重新运行本脚本。" >&2
  exit 3
fi

if [[ "${mode}" == "check" ]]; then
  echo "录包前检查通过；没有开始录制。"
  exit 0
fi

# 标准模式录制行业中常用于激光 SLAM、定位和局部规划回放的数据：
# 1. 原始传感器：雷达点云/数据包、雷达 IMU、机身 IMU；
# 2. 位姿结果：里程计、Fast-LIO 状态估计、路径；
# 3. 坐标关系：/tf 和 /tf_static；
# 4. 调试结果：配准点云、地图、SLAM/导航/底盘状态；
# 5. 操作输入：速度命令、遥控器、目标点和初始位姿；
# 6. 当前规划工程的输入输出及 ROS 日志/参数事件。
# 使用正则表达式后，即使某个节点稍晚启动，其匹配的话题也会自动加入录制。
readonly STANDARD_TOPIC_REGEX='^/(odometry|location_now|state_estimation|state_estimation_copy|odometry_copy|lidar_imu|lidar_imu_copy|imu_raw|imu_raw_x5|lidar_points|lidar_points_copy|lidar_packets|lidar_device_ctrl_state|tf|tf_static|path|path_copy|cloud_registered|cloud_registered_copy|cloud_registered_body|cloud_registered_body_copy|cloud_effected|cloud_effected_copy|Laser_map|Laser_map_copy|ikd_tree_copy|map|map_metadata|scan|slam/status|navigation_status|locomotion/status|vel_cmd|vel_cmd_copy|joy|gnss/fix|gnss/fusion|initialpose|initialpose_copy|goal_pose|clicked_point|icp_result|icp_result_copy|prior_map_copy|transformed_cloud_copy|relocalization_result|pct_path|pct_path_copy|dog_output_local_path|dog_output_local_path_copy|dog_output_global_path_unfinished|dog_output_global_path_unfinished_copy|goal_reached|goal_reached_copy|if_reach_the_goal|if_reach_the_goal_copy|dog_2Dmap_occupancy|dog_2Dmap_occupancy_copy|lidar_points_filtered|lidar_points_filtered_copy|diagnostics|diagnostics_agg|parameter_events|rosout)$'

timestamp="$(date '+%Y%m%d_%H%M%S')"
bag_prefix="${BAG_PREFIX:-slam_nav}"
if [[ ! "${bag_prefix}" =~ ^[A-Za-z0-9_-]+$ ]]; then
  echo "错误：BAG_PREFIX 只能包含字母、数字、下划线和短横线。" >&2
  exit 2
fi

# 无论从哪里运行脚本，包都固定保存在脚本所在目录。
output_path="${RECORD_DIR}/${bag_prefix}_${timestamp}"
available_kib="$(df -Pk "${RECORD_DIR}" | awk 'NR == 2 {print $4}')"
if ((available_kib < 2 * 1024 * 1024)); then
  echo "错误：当前目录可用空间不足 2 GiB，拒绝开始录制。" >&2
  exit 4
fi

echo
echo "开始录制 ROS 2 数据"
echo "保存目录：${output_path}"
echo "可用空间：$((available_kib / 1024 / 1024)) GiB"
echo "停止方法：在这个终端按 Ctrl+C，然后等待 rosbag 完成收尾。"
echo

# 每个数据库文件最大 4 GiB。长时间录制时会自动分成多个文件，仍属于同一个包。
if [[ "${mode}" == "all" ]]; then
  echo "警告：--all 会包含相机、音频等高带宽话题，磁盘占用和系统负载会明显增加。"
  exec ros2 bag record \
    --storage sqlite3 \
    --max-bag-size 4294967296 \
    --output "${output_path}" \
    --all
fi

exec ros2 bag record \
  --storage sqlite3 \
  --max-bag-size 4294967296 \
  --output "${output_path}" \
  --regex "${STANDARD_TOPIC_REGEX}"
