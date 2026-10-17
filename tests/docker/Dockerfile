ARG ROS_IMAGE=ros:humble-ros-base
FROM ${ROS_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

COPY linkerbot_hand_msgs/package.xml /dependencies/linkerbot_hand_msgs/package.xml
COPY linkerbot_hand_ros/package.xml /dependencies/linkerbot_hand_ros/package.xml
COPY linkerbot_hand_gui/package.xml /dependencies/linkerbot_hand_gui/package.xml

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-venv python3-pip python3-setuptools python3-wheel \
      python3-colcon-common-extensions python3-rosdep

RUN rosdep update --rosdistro "${ROS_DISTRO}" \
    && rosdep install --from-paths /dependencies --ignore-src -y \
      --rosdistro "${ROS_DISTRO}" \
    && rm -rf /var/lib/apt/lists/*

COPY requirements/ci-test.txt /tmp/ci-test.txt
RUN python3 -m venv --system-site-packages /venv \
    && /venv/bin/python -m pip install --upgrade pip \
    && /venv/bin/python -m pip install -r /tmp/ci-test.txt

COPY tests/docker/verify.sh /usr/local/bin/verify-hand-ros
ENTRYPOINT ["/bin/bash", "/usr/local/bin/verify-hand-ros"]
