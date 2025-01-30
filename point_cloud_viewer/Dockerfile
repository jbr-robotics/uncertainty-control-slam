FROM ubuntu:22.04

RUN apt-get update && apt-get install -y \
    libsdl2-dev \
    cmake \
    curl \
    git \
    g++ \
    golang

# Install Rust
RUN curl --proto '=https' --tlsv1.2 -sS https://sh.rustup.rs | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

# Install point_cloud_viewer
WORKDIR /app
RUN git clone https://github.com/cartographer-project/point_cloud_viewer.git


## Install build_octree
WORKDIR /app/point_cloud_viewer
RUN git checkout 189a81d53b3f674c7738a6a16d9674c20fc7c106
RUN git submodule update --init --recursive
RUN cargo build --release || true
COPY ./patch/build_octree_patch.sh ./build_octree_patch.sh
RUN ./build_octree_patch.sh 
RUN cargo build --release

## Install sdl_viewer
WORKDIR /app/point_cloud_viewer/sdl_viewer/
RUN cargo build --release

RUN echo "export PATH=\"/app/point_cloud_viewer/target/release:${PATH}\"" >>~/.bashrc

CMD ["/bin/bash"]
