# Use the official ROS Noetic base image
FROM ros:noetic

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

# Install necessary packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    build-essential \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-tf \
    ros-${ROS_DISTRO}-eigen-conversions \
    ros-${ROS_DISTRO}-tf-conversions \
    libgflags-dev \
    libgoogle-glog-dev \
    libeigen3-dev 

RUN apt-get install -y \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-pcl-ros 

# Set up the workspace
WORKDIR /root/catkin_ws/src

# Clone the kitti_to_rosbag repository
RUN git clone https://github.com/ethz-asl/kitti_to_rosbag.git

# Clone the required dependency repositories
RUN git clone https://github.com/catkin/catkin_simple.git \
    && git clone https://github.com/ethz-asl/gflags_catkin.git \
    && git clone https://github.com/ethz-asl/glog_catkin.git \
    && git clone https://github.com/ethz-asl/eigen_catkin.git \
    && git clone https://github.com/ethz-asl/eigen_checks.git \
    && git clone https://github.com/ethz-asl/minkindr.git \
    && git clone https://github.com/ethz-asl/minkindr_ros.git \
    && git clone https://github.com/ethz-asl/numpy_eigen.git \
    && git clone https://github.com/ethz-asl/catkin_boost_python_buildtool.git

# Build the workspace
WORKDIR /root/catkin_ws

RUN sed -i 's/CV_LOAD_IMAGE_UNCHANGED/cv::IMREAD_UNCHANGED/g' /root/catkin_ws/src/kitti_to_rosbag/kitti_to_rosbag/src/kitti_parser.cpp

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd src && \
    catkin_init_workspace && \
    cd .. && \
    mkdir build && \
    cd build && \
    cmake ../src -DCMAKE_INSTALL_PREFIX=../install -DCATKIN_DEVEL_PREFIX=../devel -DCMAKE_CXX_STANDARD=14 -DCMAKE_CXX_FLAGS="-w" && \
    make -j" 

# Set the entrypoint to a shell
ENTRYPOINT ["/bin/bash"]
