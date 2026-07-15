#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-only
# Copyright 2026 hengzhiyou

set -euo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly PACKAGE_NAME="dog_ego_planner"

build_type="RelWithDebInfo"
clean=false
extra_colcon_args=()

usage() {
  cat <<'EOF'
Usage: scripts/build.sh [--clean] [--debug | --release] [-- COLCON_ARGS...]

  --clean    Remove this repository's build/, install/ and log/ before building.
  --debug    Configure CMake with CMAKE_BUILD_TYPE=Debug.
  --release  Configure CMake with CMAKE_BUILD_TYPE=Release.
  -h, --help Show this help.

The default is an incremental RelWithDebInfo build.
EOF
}

while (($# > 0)); do
  case "$1" in
    --clean)
      clean=true
      shift
      ;;
    --debug)
      build_type="Debug"
      shift
      ;;
    --release)
      build_type="Release"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      extra_colcon_args=("$@")
      break
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if ! command -v colcon >/dev/null 2>&1; then
  echo "colcon was not found. Source ROS 2 Humble before building." >&2
  exit 127
fi

cd -- "${REPO_ROOT}"

if [[ "${clean}" == true ]]; then
  if [[ -z "${REPO_ROOT}" || "${REPO_ROOT}" == "/" ]]; then
    echo "Refusing to clean an unsafe repository path." >&2
    exit 1
  fi
  rm -rf -- "${REPO_ROOT}/build" "${REPO_ROOT}/install" "${REPO_ROOT}/log"
fi

echo "Building ${PACKAGE_NAME} (${build_type}) in ${REPO_ROOT}"
colcon build \
  --base-paths "${REPO_ROOT}" \
  --packages-select "${PACKAGE_NAME}" \
  "${extra_colcon_args[@]}" \
  --cmake-args "-DCMAKE_BUILD_TYPE=${build_type}"
