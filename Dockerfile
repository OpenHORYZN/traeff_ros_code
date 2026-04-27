ARG TARGETARCH
FROM ros:humble-perception AS base_amd64
FROM arm64v8/ros:humble-perception AS base_arm64
FROM base_${TARGETARCH} AS final

# ── Environment ───────────────────────────────────────────────────────────── #
ENV ROS_WS=/ros_ws
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR $ROS_WS

# Fix broken apt keys / certificates
RUN rm -rf /var/lib/apt/lists/* && \
    apt-get clean && \
    apt-get update -o Acquire::AllowInsecureRepositories=true \
                   -o Acquire::AllowDowngradeToInsecureRepositories=true && \
    apt-get install -y --no-install-recommends \
        ca-certificates \
        gnupg \
        lsb-release && \
    rm -rf /var/lib/apt/lists/*

# ── System & build tools ──────────────────────────────────────────────────── #
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    pkg-config \
    wget \
    unzip \
    python3-colcon-common-extensions \
    python3-dev \
    python3-numpy \
    libgtk-3-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libx264-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    gfortran \
    libopenexr-dev \
    libatlas-base-dev \
    && rm -rf /var/lib/apt/lists/*

# ── OpenCV 4.8 with contrib (includes ArUco in objdetect) ────────────────── #

ARG OPENCV_VERSION=4.8.0
RUN apt-get update && apt-get install -y --no-install-recommends \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/* \
    || true

# If the distro OpenCV is < 4.8, build 4.8 from source
RUN INSTALLED=$(pkg-config --modversion opencv4 2>/dev/null || echo "0.0.0") && \
    MAJOR=$(echo $INSTALLED | cut -d. -f1) && \
    MINOR=$(echo $INSTALLED | cut -d. -f2) && \
    if [ "$MAJOR" -lt 4 ] || { [ "$MAJOR" -eq 4 ] && [ "$MINOR" -lt 8 ]; }; then \
      echo "Building OpenCV ${OPENCV_VERSION} from source..." && \
      apt-get update && apt-get install -y --no-install-recommends ca-certificates && \
      cd /tmp && \
      wget -q https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip -O opencv.zip && \
      wget -q https://github.com/opencv/opencv_contrib/archive/${OPENCV_VERSION}.zip -O opencv_contrib.zip && \
      unzip -q opencv.zip && unzip -q opencv_contrib.zip && \
      mkdir -p opencv-${OPENCV_VERSION}/build && \
      cd opencv-${OPENCV_VERSION}/build && \
      cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DOPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib-${OPENCV_VERSION}/modules \
        -DBUILD_TESTS=OFF \
        -DBUILD_PERF_TESTS=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DWITH_GTK=ON \
        -DWITH_FFMPEG=ON \
        -DWITH_V4L=ON \
        -DPYTHON3_EXECUTABLE=$(which python3) && \
      make -j$(nproc) && \
      make install && \
      ldconfig && \
      cd / && rm -rf /tmp/opencv* ; \
    else \
      echo "OpenCV ${INSTALLED} already >= 4.8, skipping source build." ; \
    fi

# ── ROS 2 Humble packages ─────────────────────────────────────────────────── #
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rclcpp \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    ros-humble-mavros-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-camera-info-manager \
    && rm -rf /var/lib/apt/lists/*

# ── RealSense SDK ───────────────────────────────────────────── #
RUN git clone https://github.com/realsenseai/librealsense.git && \
    cd librealsense && mkdir build && cd build && \
    cmake .. && make -j$(nproc) && make install && ldconfig

# ── Copy source & build ───────────────────────────────────────────────────── #
COPY ros_src/ $ROS_WS/src/

RUN /bin/bash -c "\
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    cd $ROS_WS && \
    colcon build \
      --symlink-install \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    "

# ── Auto-source ───────────────────────────────────────────────────────────── #
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc && \
    echo "source $ROS_WS/install/setup.bash"      >> ~/.bashrc

