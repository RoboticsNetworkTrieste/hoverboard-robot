# ROS2 Humble environment for the hoverboard_bridge workspace.
FROM osrf/ros:humble-desktop

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000
ARG LDLIDAR_REF=bf668a8

RUN apt-get update && apt-get install -y --no-install-recommends \
        git python3-pytest python3-colcon-common-extensions iproute2 iputils-ping \
    && rm -rf /var/lib/apt/lists/*

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} --create-home --shell /bin/bash ${USERNAME}

# LIDAR driver, built once as an underlay.
RUN mkdir -p /opt/ldlidar_ws/src \
    && git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git /opt/ldlidar_ws/src/ldlidar_stl_ros2 \
    && git -C /opt/ldlidar_ws/src/ldlidar_stl_ros2 checkout ${LDLIDAR_REF} \
    && cd /opt/ldlidar_ws \
    && . /opt/ros/humble/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

# Named volumes inherit this ownership on first use.
RUN mkdir -p /ws/src /ws/build /ws/install /ws/log && chown -R ${USER_UID}:${USER_GID} /ws

COPY ros_env.sh entrypoint.sh /opt/hoverboard/
RUN chmod +x /opt/hoverboard/entrypoint.sh \
    && echo 'source /opt/hoverboard/ros_env.sh' >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /ws
ENTRYPOINT ["/opt/hoverboard/entrypoint.sh"]
CMD ["bash"]
