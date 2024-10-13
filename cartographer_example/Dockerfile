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

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc 
# Set environment variable to ensure Qt applications like Gazebo can run
ENV QT_X11_NO_MITSHM=1

RUN export DEBIAN_FRONTEND=noninteractive \ 
    && apt-get install -y \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-cartographer-rviz

RUN apt-get install -y python3-pip 
RUN pip3 install rosbags

COPY ./launch/* /opt/ros/humble/share/cartographer_ros/launch/
COPY ./configuration_files/* /opt/ros/humble/share/cartographer_ros/configuration_files/

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]
