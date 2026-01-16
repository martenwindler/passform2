#
# ──────────────────────────────────────────────────────────────
#  ROS 2 Rolling + MFRC522 NFC Reader + Raspberry Pi 5
# ──────────────────────────────────────────────────────────────
#

# Basis-Image (offiziell von Open Robotics)
FROM ros:rolling-ros-base

# Allgemeine Umgebungsvariablen
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=rolling
ENV MODULE_TYPE=""
ENV POSITION_X=""
ENV POSITION_Y=""
ENV SKIP_NFC="false"


# Systempakete installieren
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake pkg-config \
    python3-pip python3-dev python3-setuptools \
    python3-rpi.gpio python3-spidev \
    python3-colcon-common-extensions \
    git wget curl \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Python-Abhängigkeiten für MFRC522
RUN pip3 install --no-cache-dir --break-system-packages \
    mfrc522 \
    spidev \
    RPi.GPIO


# ROS 2 Workspace einrichten
WORKDIR /ros_ws
COPY . ./src/passform_agent_ros/

RUN rosdep update && \
    rosdep install --from-paths src/passform_agent_ros --ignore-src --rosdistro=${ROS_DISTRO} -r -y


RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
colcon build --symlink-install"


# Bashrc Setup für ROS
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> ~/.bashrc && \
    echo 'source /ros_ws/install/setup.bash' >> ~/.bashrc


# NFC- und ROS-Einstiegsskripte
COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/nfc_reader.py /nfc_reader.py
RUN chmod +x /entrypoint.sh /nfc_reader.py

ENTRYPOINT ["/entrypoint.sh"]
