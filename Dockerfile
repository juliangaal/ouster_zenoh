FROM ros:jazzy-ros-base

ENV ROS_WS=/ros2
WORKDIR $ROS_WS

COPY rmw_zenoh src/rmw_zenoh
COPY meta src/meta

RUN apt-get update &&\
    rosdep update &&\
    rosdep install --from-paths src --ignore-src -y &&\
    rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/$ROS_DISTRO/setup.bash &&\
    colcon build \
        --event-handlers console_direct+ \
        --parallel-workers $(nproc) \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

ARG INSTALL_DESKTOP=false

RUN if [ "$INSTALL_DESKTOP" = "true" ]; then \
        apt-get update &&\
        apt-get install -y ros-${ROS_DISTRO}-desktop &&\
        rm -rf /var/lib/apt/lists/*; \
    fi

COPY ouster_simulator src/ouster_simulator

RUN source /opt/ros/$ROS_DISTRO/setup.bash &&\
    colcon build \
        --event-handlers console_direct+ \
        --parallel-workers $(nproc) \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

ENV ROS_DOMAIN_ID=10
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
COPY entrypoint.sh .
ENTRYPOINT ["./entrypoint.sh"]

