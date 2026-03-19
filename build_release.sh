#!/usr/bin/env bash
set -euo pipefail

rm -rf build install log
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
