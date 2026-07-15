#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-only
# Copyright 2026 hengzhiyou

set -eo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly PACKAGE_NAME="dog_ego_planner"

repo_root=""
candidate="${SCRIPT_DIR}"
for _ in {1..8}; do
  if [[ -f "${candidate}/package.xml" && -f "${candidate}/install/setup.bash" ]]; then
    repo_root="${candidate}"
    break
  fi
  candidate="$(dirname -- "${candidate}")"
done
if [[ -z "${repo_root}" ]]; then
  echo "Could not locate a built dog_ego_planner source workspace." >&2
  exit 1
fi
readonly REPO_ROOT="${repo_root}"

cd -- "${REPO_ROOT}"

# shellcheck disable=SC1091
source install/setup.bash
set -u

# Never inherit a caller's production ROS graph settings.  The test suite uses
# Fast DDS on the highest valid ROS domain and restricts discovery to localhost.
export ROS_DOMAIN_ID=232
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS2CLI_NO_DAEMON=1
export AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS="${AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS:-1}"

echo "Testing ${PACKAGE_NAME} in isolated ROS domain ${ROS_DOMAIN_ID} (localhost only)."
colcon test --packages-select "${PACKAGE_NAME}" --event-handlers console_direct+
colcon test-result --verbose
