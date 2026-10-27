FROM osrf/ros:jazzy-desktop

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions python3-rosdep python3-pip \
      ros-jazzy-turtlesim ros-jazzy-demo-nodes-cpp ros-jazzy-demo-nodes-py \
      build-essential git nano \
      python3-fastapi python3-uvicorn python3-yaml python3-httpx python3-pytest \
      python3-sgp4 python3-numpy libhamlib-utils \
      ros-jazzy-rosbridge-server \
    && rm -rf /var/lib/apt/lists/*

ARG UID=1000
ARG GID=1000
RUN (userdel -r ubuntu 2>/dev/null || true) \
    && groupadd -g ${GID} dev && useradd -m -u ${UID} -g ${GID} -s /bin/bash dev \
    && echo "source /opt/ros/jazzy/setup.bash" >> /home/dev/.bashrc \
    && echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> /home/dev/.bashrc

USER dev
WORKDIR /ws
CMD ["bash"]
