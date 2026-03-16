FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    libbz2-dev \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros2_ws/src

# Copy package source
COPY . /ros2_ws/src/nimbro_topic_transport/

# Build
WORKDIR /ros2_ws
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install

# Source workspace on entry
ENTRYPOINT ["/bin/bash", "-c", "source /ros2_ws/install/setup.bash && exec \"$@\"", "--"]
CMD ["bash"]
