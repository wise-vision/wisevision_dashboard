FROM ros:humble

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-vcstool \
    build-essential \
    cmake \
    openssh-client && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-humble-ament-cmake \
    ros-humble-ros-base && \
    rm -rf /var/lib/apt/lists/*

ARG GITHUB_TOKEN

WORKDIR /ros2_ws

COPY msgs.repos .

RUN mkdir -p src && \
    sed -i "s|https://github.com|https://$GITHUB_TOKEN@github.com|g" msgs.repos && \
    cat msgs.repos && \
    vcs import src < msgs.repos

RUN apt-get update && rosdep update && \
    rosdep install --from-paths src -i -y --rosdistro humble

RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

WORKDIR /usr/src/app

COPY requirements.txt ./

RUN pip3 install --no-cache-dir -r requirements.txt

COPY . .

SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash"

EXPOSE 5000

CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && python3 -m app.server.run"]