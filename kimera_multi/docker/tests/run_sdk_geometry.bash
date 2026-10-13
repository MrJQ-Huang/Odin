#!/usr/bin/env bash
set -euo pipefail
g++ -std=c++17 /workspace/docker/tests/sdk_geometry.cpp \
  -I/workspace/include -I/opt/ros/noetic/include -I/usr/include/eigen3 \
  -o /tmp/sdk_geometry_test
/tmp/sdk_geometry_test
