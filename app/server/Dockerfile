FROM wisevision/ros_with_wisevision_msgs:humble

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-vcstool \
    build-essential \
    cmake \
    openssh-client && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /usr/src
COPY app /usr/src/app

ENV PYTHONPATH=/usr/src/app

WORKDIR /usr/src/app/server

RUN pip3 install --no-cache-dir -r requirements.txt
WORKDIR /usr/src
SHELL ["/bin/bash", "-c"]

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && source /root/wisevision_msgs_ws/install/setup.bash && exec python3 -m app.server.run"]

EXPOSE 5000