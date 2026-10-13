#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
IMAGE=${ODIN1_IMAGE:-kimera-odin1:noetic}
if ! command -v docker >/dev/null; then
  echo 'Docker 未安装，请安装 docker.io 并启动 docker 服务。' >&2
  exit 1
fi
# Group membership takes effect in a fresh process, without forcing a logout.
if ! docker info >/dev/null 2>&1; then
  if [[ ${ODIN1_GROUP_REEXEC:-0} != 1 && " $(id -nG "$USER") " == *" docker "* ]]; then
    export ODIN1_GROUP_REEXEC=1
    exec sg docker -c "exec $(printf '%q ' "$0" "$@")"
  fi
  docker info >&2
  exit 1
fi
case ${1:-help} in
  build)
    python3 "$ROOT/docker/prepare_odin1.py"
    # The Ubuntu docker.io package does not require the separate buildx plugin.
    build_args=(--build-arg "BUILD_JOBS=${BUILD_JOBS:-2}")
    if [[ -n ${ODIN1_BASE_IMAGE:-} ]]; then
      build_args+=(--build-arg "BASE_IMAGE=$ODIN1_BASE_IMAGE")
    fi
    DOCKER_BUILDKIT=0 docker build --network=host "${build_args[@]}" \
      -f "$ROOT/artifacts/docker/context/docker/Dockerfile.odin1" \
      -t "$IMAGE" "$ROOT/artifacts/docker/context"
    ;;
  shell)
    exec docker run --rm --init -it --user "$(id -u):$(id -g)" \
      -e HOME=/tmp/odin1-home --mount "type=bind,src=$ROOT,dst=/workspace" "$IMAGE" bash
    ;;
  replay)
    shift
    exec docker run --rm --init --user "$(id -u):$(id -g)" \
      -e HOME=/tmp/odin1-home --mount "type=bind,src=$ROOT,dst=/workspace" \
      -e "ODIN1_IMAGE_ID=$(docker image inspect --format '{{.Id}}' "$IMAGE")" \
      "$IMAGE" python3 /workspace/docker/replay_odin1.py "$@"
    ;;
  check)
    exec docker run --rm --init --user "$(id -u):$(id -g)" \
      -e HOME=/tmp/odin1-home --mount "type=bind,src=$ROOT,dst=/workspace,readonly" \
      "$IMAGE" bash -ec 'rosversion -d; rospack find kimera_multi; roslaunch --nodes /workspace/launch/kimera_sdk_odin1.launch params_folder:=/workspace/artifacts/odin1/params'
    ;;
  test-sparse)
    shift
    exec docker run --rm --init --user "$(id -u):$(id -g)" \
      -e HOME=/tmp/odin1-home --mount "type=bind,src=$ROOT,dst=/workspace,readonly" \
      "$IMAGE" bash /workspace/docker/tests/run_sparse_keypoints.bash "$@"
    ;;
  test-sdk)
    exec docker run --rm --init --user "$(id -u):$(id -g)" \
      -e HOME=/tmp/odin1-home --mount "type=bind,src=$ROOT,dst=/workspace,readonly" \
      "$IMAGE" bash /workspace/docker/tests/run_sdk_geometry.bash
    ;;
  *)
    echo 'Usage: docker/odin1.bash {build|check|test-sdk|test-sparse [camera.yaml]|shell|replay [--frontend sdk|vio] [--bag PATH] [--rate 1] [--output PATH]}'
    ;;
esac
