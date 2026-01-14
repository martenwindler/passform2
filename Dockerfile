FROM ros:humble-ros-base

WORKDIR /app

RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

COPY . /app/src/passform2

RUN pip3 install /app/src/passform2

RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select passform2

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source install/setup.bash && exec \"$@\"", "--"]
CMD ["ros2", "run", "passform2", "manager"]
