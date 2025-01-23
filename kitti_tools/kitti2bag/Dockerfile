FROM ros:lunar-ros-base

# Update package lists and install necessary packages
RUN apt-get update \
  && DEBIAN_FRONTEND=noninteractive apt-get -y install \
    ros-lunar-cv-bridge \
    ros-lunar-opencv3 \
    ros-lunar-tf \
    python-pip \
    python-matplotlib \
    wget \
  && rm -rf /var/lib/apt/lists/*

# Manually download and install the latest version of pip
RUN wget https://bootstrap.pypa.io/pip/2.7/get-pip.py \
  && python get-pip.py \
  && rm get-pip.py

# Install kitti2bag
RUN pip install kitti2bag numpy==1.12.1 

RUN pip install opencv-python==3.4.0.12

# Copy the entrypoint script
COPY docker_entrypoint.sh /docker_entrypoint.sh

# Set the working directory
WORKDIR /data

# RUN python -c "import pykitti; print(pykitti.__version__)"

# Define the entrypoint
ENTRYPOINT ["/docker_entrypoint.sh"]
