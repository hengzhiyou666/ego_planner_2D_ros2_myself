#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-only

set -euo pipefail

readonly REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly DOG_ENV="/userdata/1_slam/setup_dog3_env.sh"

if [[ ! -f "${DOG_ENV}" ]]; then
  echo "未找到 ${DOG_ENV}；请先部署并编译 /userdata/1_slam。" >&2
  exit 2
fi

# dog3的ROS依赖、colcon、Eigen和默认rmw_zenoh_cpp由SLAM工作区统一提供。
# shellcheck disable=SC1091
source "${DOG_ENV}"
export Eigen3_DIR="/userdata/1_slam/.deps/root/usr/share/eigen3/cmake"

cd -- "${REPO_ROOT}"
rm -rf -- build install log

colcon build \
  --base-paths "${REPO_ROOT}" \
  --packages-select dog_ego_planner \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=ON \
    -DDOG_EGO_PLANNER_WARNINGS_AS_ERRORS=ON \
    -DDOG_EGO_PLANNER_ENABLE_LINT=OFF \
    -DDOG_EGO_PLANNER_VENDOR_SYSROOT_PREFIX=/sysroot

echo
echo "dog3原生编译完成。"
echo "隔离测试：cd ${REPO_ROOT} && ./scripts/test_isolated.sh"
echo "环境检查：cd ${REPO_ROOT} && ./1_start_planner.sh --check"
