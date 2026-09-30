# Runs `uri interface` with the RobotDriverROS2Control plugin.
#
# Built on the unified_robot_interface image (ROS Jazzy, mc_rtc, zenoh and
# URI already installed in ${URI_PREFIX}=/opt/uri), so only this driver is
# built here. It is installed in the same prefix, where `uri interface` looks
# for driver plugins.
ARG BASE_IMAGE=ghcr.io/isri-aist/unified_robot_interface:latest
FROM ${BASE_IMAGE}

USER root

# ROS dependencies not already in the base image, plus CycloneDDS so either
# RMW_IMPLEMENTATION (Fast DDS is the ROS default) can be used.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-controller-manager-msgs \
      ros-${ROS_DISTRO}-trajectory-msgs \
      ros-${ROS_DISTRO}-sensor-msgs \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
      libfmt-dev \
    && rm -rf /var/lib/apt/lists/*

COPY . /tmp/ros2_control_driver
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cmake -S /tmp/ros2_control_driver -B /tmp/ros2_control_driver/build \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DCMAKE_PREFIX_PATH="${URI_PREFIX};${EXTRA_DEPS_PREFIX}" \
        -DCMAKE_INSTALL_PREFIX=${URI_PREFIX} \
    && cmake --build /tmp/ros2_control_driver/build --parallel $(nproc) \
    && cmake --install /tmp/ros2_control_driver/build \
    && mkdir -p /config \
    && mkdir -p ${URI_PREFIX}/share \
    && cp -r /tmp/ros2_control_driver/etc ${URI_PREFIX}/share/ros2_control_driver \
    && rm -rf /tmp/ros2_control_driver \
    && test -f ${URI_PREFIX}/lib/robot_interface/libRobotDriverROS2Control.so

USER vscode
WORKDIR /config

# Mount a directory holding robot_interface.yaml (whose config_path points to
# a ros2_control.yaml, e.g. /config/ros2_control/ros2_control.yaml) on /config.
# Example configs are in /opt/uri/share/ros2_control_driver.
# The base image's entrypoint sources /opt/ros/${ROS_DISTRO}/setup.bash.
CMD ["uri", "interface", "-c", "/config/robot_interface.yaml"]
