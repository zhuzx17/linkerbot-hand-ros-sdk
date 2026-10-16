#!/usr/bin/env bash
set -eo pipefail

if [[ "${ROS_DISTRO}" != "${EXPECTED_ROS_DISTRO}" ]]; then
  echo "Base image ROS distribution ${ROS_DISTRO} does not match ${EXPECTED_ROS_DISTRO}" >&2
  exit 2
fi
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /venv/bin/activate
python -c 'import os, sys, rclpy; print("ROS:", os.environ["ROS_DISTRO"], "Python:", sys.version, "rclpy:", rclpy.__file__)'
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-87}
export ROS_LOG_DIR=/tmp/ros-logs
python -c 'import os; print("Test domain:", os.environ["ROS_DOMAIN_ID"], "Localhost only:", os.environ["ROS_LOCALHOST_ONLY"])'
if [[ "${ROS_DISTRO}" == jazzy ]]; then
  # Default Fast DDS transport timed out in cross-process lifecycle checks
  # after repeated context creation. Use loopback UDP in this test environment.
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
fi

mkdir -p /workspace/src/linkerbot-hand-ros /dependencies-src/transport /dependencies-src/hand /results
# Work on container-local copies; mounted source trees remain read-only.
tar -C /sources/ros --exclude='__pycache__' --exclude='*.egg-info' -cf - \
  linkerbot_hand_msgs linkerbot_hand_ros linkerbot_hand_gui tests pyproject.toml \
  README.md README_zh.md DEVELOPMENT.md | tar -C /workspace/src/linkerbot-hand-ros -xf -
for dependency in transport hand; do
  # SDK checkouts can also contain nested CI repos and actively written reports.
  # Copy only package inputs so these do not become part of the dependency.
  tar -C "/sources/${dependency}" --exclude='__pycache__' \
    --exclude='*.egg-info' -cf - pyproject.toml README.md LICENSE src \
    | tar -C "/dependencies-src/${dependency}" -xf -
done
python -m pip install -e '/dependencies-src/transport[all]' -e /dependencies-src/hand

cd /workspace
rosdep check --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}"
python -m colcon --log-base /results/colcon-log build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE="$(command -v python)"
source install/setup.bash
python -c 'import linkerbot_transport, linkerbot_hand; print("Python dependencies: OK")'
ros2 pkg prefix linkerbot_hand_msgs
ros2 pkg prefix linkerbot_hand_ros
ros2 pkg prefix linkerbot_hand_gui
ros2 pkg executables linkerbot_hand_gui
ros2 interface show linkerbot_hand_msgs/msg/HandJointCommand
for package in linkerbot_hand_msgs linkerbot_hand_ros linkerbot_hand_gui; do
  test -f "$(ros2 pkg prefix "$package")/share/${package}/LICENSE"
done

cd src/linkerbot-hand-ros
python -m pytest -q -ra -p pytest_timeout -p pytest_cov.plugin \
  linkerbot_hand_ros/tests linkerbot_hand_gui/test linkerbot_hand_msgs/test \
  --timeout=60 --junitxml=/results/pytest.xml \
  --cov=linkerbot_hand_ros --cov=linkerbot_hand_gui \
  --cov-report=term-missing:skip-covered --cov-report=xml:/results/coverage.xml "$@"
