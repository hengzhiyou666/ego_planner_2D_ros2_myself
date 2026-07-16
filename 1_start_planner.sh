#!/usr/bin/env bash
# 当直接使用 ./1_start_planner.sh 执行脚本时，上面这一行告诉电脑：请使用 Bash 来执行这个文件。

# 这是本文件使用的软件许可证标记，不影响脚本运行。
# SPDX-License-Identifier: GPL-3.0-only

# 这个脚本只做四件事：
# 1. 找到 ROS 2；
# 2. 找到当前规划工程；
# 3. 自动加载运行环境，代替手动输入 source；
# 4. 启动局部路径规划节点。

# 只要某一步发生错误，脚本就立刻停止，避免带着错误继续运行。
set -eo pipefail

# 这个脚本现在就放在工程根目录，所以它所在的文件夹就是工程根目录。
# 不管你从哪个目录运行脚本，都能正确找到工程。
# 本地示例：/home/hzy/.../ego_planner_2D_ros2_myself
# dog3 示例：/userdata/5_egoplanner
readonly REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# 当用户输入 --help 时，这个函数负责显示帮助文字。
usage() {
  cat <<'EOF'
用法：
  ./1_start_planner.sh [ROS 2 launch 参数]
  ./1_start_planner.sh --check

示例：
  ./1_start_planner.sh
  ./1_start_planner.sh launch_rviz:=true
  ./1_start_planner.sh use_sim_time:=true launch_rviz:=true

脚本只启动 dog_ego_planner，不启动 dog_move_forward.py，也不发布 /vel_cmd。
EOF
}

# 检查用户输入的第一个参数。
case "${1:-}" in
  -h|--help)
    # 输入 --help 或 -h：只显示帮助，不启动规划器。
    usage
    exit 0
    ;;
  --check)
    # 输入 --check：只检查环境，不启动规划器。
    check_only=true
    # shift 表示已经处理完 --check，把它从参数列表中拿走。
    shift
    ;;
  *)
    # 没有输入上面的特殊参数：准备正常启动规划器。
    check_only=false
    ;;
esac

# runtime_setup 用来保存统一运行环境文件的位置。
runtime_setup=""

# dog3 和本地电脑的 ROS 2 安装位置不同：
# dog3：/app/opt/ros/humble/setup.bash
# 本地：/opt/ros/humble/setup.bash
# 下面的循环会自动寻找，找到第一个存在的文件就停止寻找。
for candidate in \
  /userdata/1_slam/setup_dog3_env.sh \
  /app/opt/ros/humble/setup.bash \
  /opt/ros/humble/setup.bash; do
  if [[ -f "${candidate}" ]]; then
    runtime_setup="${candidate}"
    break
  fi
done

# 如果两个位置都没有找到 ROS 2，就显示错误并结束。
if [[ -z "${runtime_setup}" ]]; then
  echo "未找到 dog3 统一环境脚本或 ROS 2 Humble setup.bash。" >&2
  exit 1
fi

# install/setup.bash 是工程编译后生成的环境文件。
# 如果它不存在，通常表示工程还没有编译，需要先运行 build.sh。
if [[ ! -f "${REPO_ROOT}/install/setup.bash" ]]; then
  echo "未找到 ${REPO_ROOT}/install/setup.bash，请先运行 ./scripts/build.sh。" >&2
  exit 1
fi

# source 可以理解为“把工具箱打开并放到当前终端里”。
# 第一条 source 加载 ROS 2，第二条 source 加载当前规划工程。
# ROS 2 环境文件可能使用尚未定义的变量，所以 source 时暂时关闭变量严格检查。
set +u
source "${runtime_setup}"
source "${REPO_ROOT}/install/setup.bash"
set -u

# 询问 ROS 2：你找到的 dog_ego_planner 包安装在哪里？
# 保存这个结果，既能检查环境，也方便启动前显示给用户。
package_prefix="$(ros2 pkg prefix dog_ego_planner)"

# 如果用户输入了 --check，到这里说明环境没有问题。
# 只显示检查结果，然后退出，不会启动任何 ROS 节点。
if [[ "${check_only}" == true ]]; then
  echo "运行环境：${runtime_setup}"
  echo "工程目录：${REPO_ROOT}"
  echo "包安装目录：${package_prefix}"
  echo "启动环境检查通过；没有启动节点。"
  exit 0
fi

# 环境已经准备好，现在开始启动局部路径规划器。
echo "启动 dog_ego_planner（包目录：${package_prefix}）"

# "$@" 表示把用户输入的其他参数继续交给 ros2 launch。
# 例如输入 launch_rviz:=true，就会继续传给 launch 文件。
# exec 表示让当前脚本进程直接变成规划器启动进程，按 Ctrl+C 时也更容易正常退出。
exec ros2 launch dog_ego_planner dog_ego_planner.launch.py "$@"
