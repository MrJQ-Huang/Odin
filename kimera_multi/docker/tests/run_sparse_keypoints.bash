#!/usr/bin/env bash
set -euo pipefail
g++ -std=c++17 /workspace/docker/tests/sparse_keypoints.cpp \
  -I/opt/kimera_ws/src/kimera_vio/include -I/opt/kimera_ws/devel/include \
  -I/usr/include/eigen3 -L/opt/kimera_ws/devel/lib -L/usr/local/lib \
  -lkimera_vio -lgtsam -lglog $(pkg-config --cflags --libs opencv4) \
  -o /tmp/sparse_keypoints_test
/tmp/sparse_keypoints_test "${1:-/workspace/artifacts/odin1/params/LeftCameraParams.yaml}"
