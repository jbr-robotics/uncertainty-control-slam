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
        libgl1-mesa-glx 

RUN export DEBIAN_FRONTEND=noninteractive \
    && apt-get install -y \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-cartographer-rviz \
    ros-humble-nav2-map-server

RUN apt-get install -y python3-pip 
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

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc 
# Set environment variable to ensure Qt applications like Gazebo can run
ENV QT_X11_NO_MITSHM=1

COPY ./launch/* /opt/ros/humble/share/cartographer_ros/launch/
COPY ./configuration_files/* /opt/ros/humble/share/cartographer_ros/configuration_files/
COPY ./quality_estimator /root/quality_estimator

ENTRYPOINT ["/ros_entrypoint.sh"]
WORKDIR /root
CMD ["/bin/bash"]
