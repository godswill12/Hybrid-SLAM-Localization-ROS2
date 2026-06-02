FROM osrf/ros:jazzy-desktop-full

SHELL ["/bin/bash", "-c"]

# -----------------------------
# Core ROS 2 + Navigation stack
# -----------------------------
RUN apt-get update && apt-get install -y \
    python3-pip python3-numpy \
    ros-jazzy-turtlebot3* \
    ros-jazzy-gazebo-ros* \
    ros-jazzy-gazebo-msgs \
    ros-jazzy-nav2* ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-vision-opencv \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-rtabmap-ros \
    build-essential cmake git \
    ninja-build \
    python3-opencv \
    libopencv-dev \
    libeigen3-dev \
    wget curl \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# Python perception stack
# -----------------------------
RUN pip3 install --no-cache-dir --break-system-packages \
    ultralytics \
    numpy \
    scipy \
    matplotlib

# -----------------------------
# Create ROS 2 workspace
# -----------------------------
RUN mkdir -p /ros_ws/src
WORKDIR /ros_ws/src

# -----------------------------
# (Optional) Clone your semantic costmap package later
# -----------------------------
# RUN git clone https://github.com/<your_repo>/semantic_costmap_layer.git

# -----------------------------
# Resolve ROS dependencies
# -----------------------------
WORKDIR /ros_ws

RUN apt-get update && \
    rosdep update && \
    rosdep install -r --from-paths src --ignore-src -y --rosdistro jazzy

# -----------------------------
# Build workspace
# -----------------------------
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install"

# -----------------------------
# Environment setup
# -----------------------------
RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc && \
    echo 'source /ros_ws/install/setup.bash' >> ~/.bashrc && \
    echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc

WORKDIR /ros_ws

CMD ["bash"]