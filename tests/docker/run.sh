#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 3 || $# -gt 4 ]]; then
  echo "Usage: bash tests/docker/run.sh humble|jazzy TRANSPORT_SOURCE HAND_SOURCE [RESULTS_DIR]" >&2
  exit 2
fi
ros_distro=$1
case "$ros_distro" in
  humble|jazzy) ;;
  *) echo "Supported distributions: humble, jazzy" >&2; exit 2 ;;
esac
transport_source=$(realpath "${2:?Provide the Transport main source directory}")
hand_source=$(realpath "${3:?Provide the Hand Python main source directory}")
repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
results_dir=${4:-"${repo_root}/tests/docker/results/${ros_distro}"}
mkdir -p "$results_dir"
results_dir=$(realpath "$results_dir")
test_image="linkerbot-hand-ros-test:${ros_distro}"
base_image=${ROS_TEST_BASE_IMAGE:-ros:${ros_distro}-ros-base}
container_name="linkerbot-hand-${ros_distro}-test-$$"
cleanup() {
  docker rm -f "$container_name" >/dev/null 2>&1 || true
}
trap cleanup EXIT
asset_options=()
if [[ -n "${LINKERBOT_HAND_ASSETS_ROOT:-}" ]]; then
  asset_source=$(realpath "${LINKERBOT_HAND_ASSETS_ROOT}")
  if [[ ! -d "$asset_source" ]]; then
    echo "LINKERBOT_HAND_ASSETS_ROOT must be a directory" >&2
    exit 2
  fi
  asset_options=(
    --mount "type=bind,source=${asset_source},target=/sources/assets,readonly"
    --env LINKERBOT_HAND_ASSETS_ROOT=/sources/assets
  )
fi

{
  for source_dir in "$repo_root" "$transport_source" "$hand_source"; do
    git -C "$source_dir" log -1 --format='%H %s'
    git -C "$source_dir" status --short
  done
} | tee "$results_dir/sources.log"
docker build --network "${ROS_TEST_BUILD_NETWORK:-default}" \
  --build-arg HTTP_PROXY --build-arg HTTPS_PROXY --build-arg NO_PROXY \
  --build-arg "ROS_IMAGE=${base_image}" \
  -f "${repo_root}/tests/docker/Dockerfile" -t "$test_image" "$repo_root" \
  2>&1 | tee "$results_dir/build.log"
docker run --rm --init --name "$container_name" \
  --env "EXPECTED_ROS_DISTRO=${ros_distro}" \
  --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-87}" \
  --mount "type=bind,source=${repo_root},target=/sources/ros,readonly" \
  --mount "type=bind,source=${transport_source},target=/sources/transport,readonly" \
  --mount "type=bind,source=${hand_source},target=/sources/hand,readonly" \
  --mount "type=bind,source=${results_dir},target=/results" \
  "${asset_options[@]}" \
  "$test_image" 2>&1 | tee "$results_dir/tests.log"
