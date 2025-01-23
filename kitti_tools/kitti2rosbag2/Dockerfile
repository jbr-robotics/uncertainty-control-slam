# Use ROS2 desktop full image as the base
FROM ros:humble

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
SHELL [ "/bin/bash", "-c" ]

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    ros-humble-cv-bridge \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install OpenCV Python module via pip (for completeness)
RUN pip3 install opencv-python-headless

# Set up the workspace
WORKDIR /root/ros2_ws/src

# Clone the kitti2rosbag2 repository
RUN git clone https://github.com/bharadwajsirigadi/kitti2rosbag2.git

# Use our custom launch file
COPY kitti2rosbag2.launch.py /root/ros2_ws/src/kitti2rosbag2/launch/

# Build the package
WORKDIR /root/ros2_ws
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --packages-select kitti2rosbag2 --symlink-install

# Source ROS2 and workspace setup
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Set the entrypoint to a shell
ENTRYPOINT ["/bin/bash"]
