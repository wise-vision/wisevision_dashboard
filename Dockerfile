FROM czarekk/ros_with_lora_msgs:humble

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-vcstool \
    build-essential \
    cmake \
    openssh-client && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /usr/src/app

COPY requirements.txt ./

RUN pip3 install --no-cache-dir -r requirements.txt

COPY . .

SHELL ["/bin/bash", "-c"]

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && source /root/lora_msgs_ws/install/setup.bash && exec python3 -m app.server.run"]

EXPOSE 5000