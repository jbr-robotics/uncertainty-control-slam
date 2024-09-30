FROM ros:noetic-ros-core

# Install ROS
RUN apt-get update
RUN apt-get update && apt-get install -y \
    python3-catkin-pkg \
    python3-catkin-tools \
    ninja-build \
    ros-noetic-desktop-full \
    libceres-dev \
    lua5.2 \
    liblua5.2-dev \
    ros-noetic-map-server

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc 
ENV ROS_PACKAGE_PATH=/catkin_ws/src:/opt/ros/noetic/share

############

# Install cartographer: https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html
RUN sudo apt-get install -y python3-wstool python3-rosdep ninja-build stow

RUN mkdir /root/catkin_ws
WORKDIR /root/catkin_ws
RUN wstool init src
RUN wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
RUN wstool update -t src

RUN sudo rosdep init
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src --rosdistro=noetic -y || true
RUN src/cartographer/scripts/install_abseil.sh

ENV PYTHONPATH="/opt/ros/noetic/lib/python3/dist-packages"
ENV CMAKE_PREFIX_PATH="/opt/ros/noetic"

RUN /opt/ros/noetic/bin/catkin_make_isolated --install --use-ninja

RUN echo "source /root/catkin_ws/install_isolated/setup.bash" >> /root/.bashrc

############

# Prepare configs
COPY ./configuration_files /root/catkin_ws/install_isolated/share/cartographer_ros/configuration_files
COPY ./launch /root/catkin_ws/install_isolated/share/cartographer_ros/launch

ENV QT_X11_NO_MITSHM=1

RUN echo "export QT_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/qt5/plugins" >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]