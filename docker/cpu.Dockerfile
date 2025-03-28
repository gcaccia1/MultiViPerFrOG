#==
# Foundation
#==
ARG UBUNTU_VERSION=20.04
FROM ubuntu:${UBUNTU_VERSION} as base

# Suppresses interactive calls to APT
ENV DEBIAN_FRONTEND="noninteractive"

# Needed for string substitution
SHELL ["/bin/bash", "-c"]
ENV TERM=xterm-256color

# needed?
ENV ROS_ENVIRONMENT=1

# ----------------------------------------------------------------------------

#==
# some catkin stuff
#==

WORKDIR /home/
# Create Catkin workspace folder
RUN mkdir -p /home/catkin_wp_ceres/src
# Instead of cloning, copy the local folder into the Docker image
COPY . /home/catkin_wp_ceres/src/multiviperfrog

#==
# ROS
#==

# Version
ARG ROS=noetic
RUN chmod +x /home/catkin_wp_ceres/src/multiviperfrog/docker/submodules/ros_1.sh \
    # Fix: Install missing dependencies BEFORE running ros_1.sh
    && apt update && apt install -y \
    lsb-release \
    curl \
    sudo \
    gnupg \
    && rm -rf /var/lib/apt/lists/* \
    # Then
    && /home/catkin_wp_ceres/src/multiviperfrog/docker/submodules/ros_1.sh

#==
# Ceres Solver Setup
#==

# ----------------------------------------------------------------------------
# Install Ceres dependencies (including Abseil and googleTest built from source)
# ----------------------------------------------------------------------------

RUN apt update && apt install -y \
    git \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    cmake \
    ninja-build \
    && rm -rf /var/lib/apt/lists/*

# ----------------------------------------------------------------------------
# Build and install GoogleTest (GTest)
# ----------------------------------------------------------------------------

RUN git clone https://github.com/google/googletest.git && \
    cd googletest && mkdir build && cd build && \
    cmake .. && make -j$(nproc) && make install && \
    cd /home && rm -rf googletest
# ----------------------------------------------------------------------------
# Build and install Abseil
# ----------------------------------------------------------------------------

ENV ABSL_VERSION="lts_2025_01_27"

RUN rm -rf abseil-cpp && \
    git clone https://github.com/abseil/abseil-cpp.git && \
    cd abseil-cpp && \
    git checkout ${ABSL_VERSION} && \
    mkdir build && cd build && \
    cmake .. -G Ninja \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DCMAKE_CXX_STANDARD=17 \
        -DABSL_PROPAGATE_CXX_STD=ON \
        -DABSL_USE_EXTERNAL_GOOGLETEST=ON \
        -DABSL_ENABLE_INSTALL=ON && \
    ninja -j$(nproc) && ninja install \
    # Ensure Abseil is installed
    && ls /usr/local/lib/cmake/absl \
    && cat /usr/local/lib/cmake/absl/abslConfig.cmake

# ----------------------------------------------------------------------------
# Clone and Build Ceres Solver
# ----------------------------------------------------------------------------
WORKDIR /home/catkin_wp_ceres/src
RUN git clone https://ceres-solver.googlesource.com/ceres-solver && \
    cd ceres-solver && \
    mkdir ceres-bin && cd ceres-bin && \
    cmake .. -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_SHARED_LIBS=ON \
        -DEXPORT_BUILD_DIR=ON \
        -DCMAKE_CXX_STANDARD=17 \
        -Dabsl_DIR=/usr/local/lib/cmake/absl \
        -Dabsl_VERSION=20230125.3 && \
    ninja -j$(nproc) && \
    ninja install \
    # Ensure Ceres is installed
    && find /usr/local/lib -name "libceres.so*"

# Ensure the built Ceres library is available in the system
ENV CMAKE_PREFIX_PATH="/home/catkin_wp_ceres/src/ceres-solver/ceres-bin:$CMAKE_PREFIX_PATH"

# ----------------------------------------------------------------------------
# Install Open3D from Source (with CUDA)
# ----------------------------------------------------------------------------
WORKDIR /home

# Install required dependencies including libc++ and libc++abi
RUN apt update && apt install -y \
    libc++-dev libc++abi-dev \
    build-essential \
    cmake \
    libglfw3-dev \
    libglew-dev \
    libomp-dev \
    libxinerama-dev \
    libxcursor-dev \
    xorg-dev \
    libglu1-mesa-dev \
    python3-dev \
    python3-pip \
    ninja-build \
    && rm -rf /var/lib/apt/lists/*

# Install newer CMake version (3.25.2)
WORKDIR /home
RUN apt update && apt install -y wget && \
    wget -qO- "https://github.com/Kitware/CMake/releases/download/v3.25.2/cmake-3.25.2-linux-x86_64.tar.gz" | tar --strip-components=1 -xz -C /usr/local && \
    cmake --version \
    # install fmt
    && apt update && apt install -y libfmt-dev

# Clone Open3D repository and build
RUN git clone --recursive --branch v0.17.0 https://github.com/isl-org/Open3D.git && \
    cd Open3D && mkdir build && cd build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_SHARED_LIBS=ON \
        -DUSE_SYSTEM_OPENMP=ON \
        -DBUILD_PYTHON_MODULE=OFF \
        -DBUILD_PYBIND11=OFF \
        -DBUILD_UNIT_TESTS=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_GUI=OFF \
        -DBUILD_WEBRTC=OFF \
    && make -j$(nproc) && make install \
    && find /usr/local/lib -name "libOpen3D*" \
    # Cleanup
    && rm -rf /home/Open3D

# ----------------------------------------------------------------------------
# Cloning open3d_conversions and Multiviperfrog
# ----------------------------------------------------------------------------

RUN apt update && apt install -y git openssh-client && rm -rf /var/lib/apt/lists/*


# Clone open3d_conversions (from perception_open3d)
WORKDIR /home/catkin_wp_ceres/src
RUN git clone https://github.com/ros-perception/perception_open3d.git


# ----------------------------------------------------------------------------
# Install missing ROS dependencies
# ----------------------------------------------------------------------------
RUN echo 'export Open3D_DIR=/usr/local/lib/cmake/Open3D' >> /opt/ros/noetic/setup.bash \
    # Debug: Check if Open3D_DIR is properly set \
    && cat /opt/ros/noetic/setup.bash | grep Open3D_DIR || echo "No Open3D_DIR in setup.bash" \
    && /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /home/catkin_wp_ceres && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y"

# ----------------------------------------------------------------------------
# Install Nanoflann
# ----------------------------------------------------------------------------

RUN chmod +x /home/catkin_wp_ceres/src/multiviperfrog/docker/submodules/install_nanoflann.sh \
    && /home/catkin_wp_ceres/src/multiviperfrog/docker/submodules/install_nanoflann.sh

# ----------------------------------------------------------------------------
# Setup Catkin Workspace and Build Packages
# ----------------------------------------------------------------------------
WORKDIR /home/catkin_wp_ceres

# Initialize catkin workspace
# TODO: this part is still not working!
#RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin init" \
## Build multiviperfrog using catkin tools \
#    && /bin/bash -c "source /opt/ros/noetic/setup.bash && \
#    export CMAKE_PREFIX_PATH=/usr/local/lib/cmake/absl:$CMAKE_PREFIX_PATH && \
#    cd /home/catkin_wp_ceres && \
#    catkin build multiviperfrog --no-color" && \
#    # Ensure ROS packages are available at runtime
#    echo 'source /home/catkin_wp_ceres/devel/setup.bash' >> /root/.bashrc"
#
#
# ----------------------------------------------------------------------------

#==
# Cleanup
#==
RUN apt update && apt upgrade -y

# ----------------------------------------------------------------------------
# EOF
