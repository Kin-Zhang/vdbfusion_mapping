FROM osrf/ros:noetic-desktop-full
LABEL maintainer="Kin Zhang <kin_eng@163.com> Ignacio Vizzo <ivizzo@uni-bonn.de>"

# Just in case we need it
ENV DEBIAN_FRONTEND noninteractive

# install zsh
RUN apt update && apt install -y wget git zsh tmux vim g++
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.2/zsh-in-docker.sh)" -- \
    -t robbyrussell \
    -p git \
    -p ssh-agent \
    -p https://github.com/agkozak/zsh-z \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-completions \
    -p https://github.com/zsh-users/zsh-syntax-highlighting

# Install utilities
RUN apt-get update && apt-get install --no-install-recommends -y \
    git \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python3 utils
RUN python3 -m pip install --no-cache  catkin-tools

# Install extra ROS dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-${ROS_DISTRO}-tf2-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install OpenVDB dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    libblosc-dev \
    libboost-iostreams-dev \
    libboost-system-dev \
    libboost-system-dev \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# Install OpenVDB from source
RUN git clone --depth 1 https://github.com/nachovizzo/openvdb.git -b nacho/vdbfusion \
    && cd openvdb \
    && mkdir build && cd build \
    && cmake  -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DUSE_ZLIB=OFF .. \
    && make -j$(nproc) all install \
    && cd / \
    && rm -rf /openvdb

# Install Open3D dependencies
RUN git clone --depth 1 --branch v0.16.0 https://gitcode.net/mirrors/isl-org/Open3D.git && cd Open3D \
    && util/install_deps_ubuntu.sh && mkdir build && cd build && cmake .. \
    && make -j$(nproc) \
    && make -j$(nproc) all install \
    && cd / \
    && rm -rf /Open3D

# kin self vdbfusion_mapping
RUN echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc
RUN echo "source /opt/ros/noetic/setup.bashrc" >> ~/.bashrc

# needs to be done before we can apply the patches
RUN git config --global user.email "kin_eng@163.com"
RUN git config --global user.name "kin-docker"

RUN mkdir -p /workspace/vdbfusion_mapping_ws /workspace/data
WORKDIR /workspace/vdbfusion_mapping_ws
RUN git clone --recurse-submodules https://gitee.com/kin_zhang/vdbfusion_mapping.git /workspace/vdbfusion_mapping_ws/src

RUN chmod +x /workspace/vdbfusion_mapping_ws/src/assets/scripts/setup_lib.sh
RUN /workspace/vdbfusion_mapping_ws/src/assets/scripts/setup_lib.sh
