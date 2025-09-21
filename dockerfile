FROM ubuntu:22.04

# --- 1) Entorno ---------------------------------------------------------------
ARG ROS_DISTRO=humble
# fortress | harmonic  (fortress recomendado para ROS 2 Humble)
ARG GZ_FLAVOR=fortress

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    SETUPTOOLS_USE_DISTUTILS=stdlib

# --- 2) Locales, Qt6 y básicos -----------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    curl gnupg lsb-release ca-certificates software-properties-common \
    build-essential python3-dev python3-pip python3-distutils \
    libgl1-mesa-glx libglib2.0-0 \
    libqt6gui6 libqt6widgets6 libqt6core6 qt6-base-dev && \
    locale-gen en_US.UTF-8 && \
    add-apt-repository -y universe && \
    rm -rf /var/lib/apt/lists/*

# --- 3) Repos ROS 2 -----------------------------------------------------------
RUN install -d /etc/apt/keyrings && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor -o /etc/apt/keyrings/ros2.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros2.gpg] \
      http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/ros2.list

# --- 4) Repo OSRF (Gazebo Ignition/GZ) ---------------------------------------
# Keyring y repo oficial para Gazebo (Ignition/GZ)
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
      https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list

# --- 5) ROS 2 + deps de sistema (Ignition en vez de Classic) -----------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-desktop-full \
    python3-colcon-common-extensions python3-vcstool python3-rosdep \
    geographiclib-tools libgeographic-dev \
    libopencv-dev libpcl-dev libnanoflann-dev \
    libxcb-cursor0 libxkbcommon-x11-0 libxcb-xinerama0 \
    git cmake ninja-build libglib2.0-dev \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp && \
    if [ "$GZ_FLAVOR" = "harmonic" ]; then \
      apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-ros-gzharmonic gz-harmonic ; \
    else \
      apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-ros-gz ignition-fortress ; \
    fi && \
    rosdep init && rosdep update && \
    rm -rf /var/lib/apt/lists/*

# --- 5.b) LCM 1.5 (si lo necesitas) ------------------------------------------
RUN git clone --depth 1 --branch v1.5.0 https://github.com/lcm-proj/lcm.git /tmp/lcm && \
    cmake -S /tmp/lcm -B /tmp/lcm/build -G Ninja \
          -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=Release && \
    cmake --build  /tmp/lcm/build && \
    cmake --install /tmp/lcm/build && \
    ldconfig && rm -rf /tmp/lcm

# --- 6) Python y paquetes extra ----------------------------------------------
RUN python3 -m pip install --no-cache-dir --upgrade pip setuptools~=69.5 wheel packaging && \
    python3 -m pip install --no-cache-dir \
      "numpy<1.24" "PySide6~=6.5" Cython~=0.29 utm pyproj

# --- 7) Workspace -------------------------------------------------------------
WORKDIR /ros2_ws
COPY src ./src
RUN bash -lc "set -e \
  && apt-get update \
  && source /opt/ros/${ROS_DISTRO}/setup.bash \
  && rosdep fix-permissions || true \
  && rosdep update --rosdistro ${ROS_DISTRO} \
  && rosdep install --rosdistro ${ROS_DISTRO} --from-paths src --ignore-src -r -y \
       --skip-keys='lcm_to_ros_interfaces message_runtime OpenCV PCL GeographicLib' \
  && colcon build --symlink-install"

# --- 8) Entrypoint y entorno --------------------------------------------------
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Fuerza CycloneDDS para alinear con el host/contenedor que ya te funciona
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
