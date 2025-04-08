FROM ros:humble

RUN apt-get update && apt-get install -y \
    wget \
    gnupg2 \
    lsb-release \
    software-properties-common \
    mesa-utils \
    libgl1-mesa-glx 

RUN add-apt-repository ppa:kisak/kisak-mesa \
    && apt-get update \
    && apt-get upgrade -y

RUN apt-get install -y software-properties-common mesa-utils libgl1-mesa-dri libglx-mesa0 libgl1-mesa-glx 

RUN export DEBIAN_FRONTEND=noninteractive \
    && apt-get update \
    && apt-get install -y \
        ninja-build \
        ros-humble-desktop-full \
        libceres-dev \
        lua5.2 \
        liblua5.2-dev \
        mesa-utils \
        libgl1-mesa-glx \
        ros-humble-cartographer \
        ros-humble-cartographer-ros \
        ros-humble-cartographer-rviz \
        ros-humble-nav2-map-server \
        ros-humble-tf2-tools \
        python3-pip 

RUN pip3 install \
    rosbags \
    lupa 

# Install map-metrics
RUN pip3 install Cython==0.29.36      
RUN pip3 install scikit-learn==0.24.2 --no-build-isolation
RUN git clone https://github.com/MobileRoboticsSkoltech/map-metrics.git /tmp/map-metrics && \
    cd /tmp/map-metrics && \
    sed -i 's/open3d = "^0.14.1"/open3d = "^0.19.0"/g' pyproject.toml && \
    pip3 install . && \
    rm -rf /tmp/map-metrics

# # Install ros2bag tools
RUN mkdir -p /root/ros2_ws/src && \
    cd /root/ros2_ws/src && \
    git clone https://github.com/Seva-Vaskin/ros2bag_tools.git && \
    cd /root/ros2_ws && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install 
RUN pip3 install pandas numexpr==2.8.4 bottleneck==1.3.6
RUN pip3 install -U argcomplete

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc 
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc 

# Set environment variable to ensure Qt applications like Gazebo can run
ENV QT_X11_NO_MITSHM=1

# Install cartographer_tuner
COPY ./cartographer_tuner /root/cartographer_tuner
WORKDIR /root/cartographer_tuner
RUN pip3 install -e .

WORKDIR /root

COPY ./launch/* /opt/ros/humble/share/cartographer_ros/launch/
COPY ./configuration_files/* /opt/ros/humble/share/cartographer_ros/configuration_files/

ENTRYPOINT ["/ros_entrypoint.sh"]
WORKDIR /root
CMD ["/bin/bash"]
