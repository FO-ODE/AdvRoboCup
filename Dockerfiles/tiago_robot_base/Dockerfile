# modified from Tiago official Dockerfile
# adjust the moveit version for TIAGo Robot in reality

FROM osrf/ros:noetic-desktop-full-focal

LABEL maintainer="Yue Erro <yue.erro@pal-robotics.com>"

ARG REPO_WS=/tiago_public_ws
RUN mkdir -p $REPO_WS/src
WORKDIR $REPO_WS

# 安装基础依赖工具
RUN apt-get update && apt-get install -y --no-install-recommends \
    apt-utils \
    libv4l-dev \
    libv4l2rds0 \
    git \
    wget \
    vim \
    locales \
    dpkg \
    ssh \
    curl \
    aptitude \
    g++ \
    gcc \
    openvpn \
    gnupg \
    bash-completion \
    vim-gtk3 \
    nano \
    psmisc \
    ccache \
    gdb \
    qtcreator \
    htop \
    man \
    meld \
    silversearcher-ag \
    terminator \
    tig \
    valgrind \
    iputils-ping \
    ipython3 \
    python-is-python3 \
    python3-scipy \
    python3-wstool \
    python3-networkx \
    python3-pip  \
    python3-vcstool \
    python3-rosinstall \
    python3-catkin-tools \
    ros-noetic-actionlib-tools \
    && rm -rf /var/lib/apt/lists/*

# 下载 TIAGo 工程源码
RUN wget https://raw.githubusercontent.com/pal-robotics/tiago_tutorials/noetic-devel/tiago_public-noetic.rosinstall \
    && vcs import src < tiago_public-noetic.rosinstall

# 安装 srdfdom 所需依赖
RUN apt-get update && apt-get install -y \
    liburdfdom-dev \
    libconsole-bridge-dev \
    libtinyxml2-dev \
    cmake \
    libboost-system-dev \
    libboost-filesystem-dev \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# 编译并安装 srdfdom 0.6.3
RUN git clone -b 0.6.3 https://github.com/ros-planning/srdfdom.git /opt/srdfdom && \
    mkdir -p /opt/srdf_ws/src && \
    mv /opt/srdfdom /opt/srdf_ws/src/srdfdom && \
    cd /opt/srdf_ws && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && \
                  export CMAKE_PREFIX_PATH=/opt/ros/noetic:$CMAKE_PREFIX_PATH && \
                  catkin build && \
                  source devel/setup.bash && \
                  cd src/srdfdom && \
                  mkdir build && cd build && \
                  cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local && \
                  make -j$(nproc) && \
                  make install" && \
    ldconfig

# 设置环境变量
RUN echo "source /opt/srdf_ws/devel/setup.bash" >> ~/.bashrc
ENV LD_LIBRARY_PATH=/usr/local/lib:/opt/srdf_ws/devel/lib:/usr/lib:/lib:$LD_LIBRARY_PATH

# 安装自定义 MoveIt deb 包（tiago_moveit 文件夹应与 Dockerfile 同级）
COPY tiago_moveit /opt/tiago_moveit
WORKDIR /opt/tiago_moveit
RUN apt-get update && \
    apt-get remove -y ros-noetic-moveit-* || true && \
    dpkg -i *.deb || apt-get install -f -y && \
    rm -rf /var/lib/apt/lists/*

# 回到主工作目录
WORKDIR $REPO_WS

# 安装 ROS 依赖（跳过已知缺失）
ARG ROSDEP_IGNORE="urdf_test omni_drive_controller orocos_kdl pal_filters libgazebo9-dev pal_usb_utils speed_limit_node camera_calibration_files pal_moveit_plugins pal_startup_msgs pal_local_joint_control pal_pcl_points_throttle_and_filter current_limit_controller hokuyo_node dynamixel_cpp pal_moveit_capabilities pal_pcl dynamic_footprint gravity_compensation_controller pal-orbbec-openni2 pal_loc_measure pal_map_manager ydlidar_ros_driver"
RUN apt-get update && rosdep install --from-paths src --ignore-src -y --rosdistro noetic --skip-keys="${ROSDEP_IGNORE}" && \
    rm -rf /var/lib/apt/lists/*

# 构建主工作空间
RUN bash -c "source /opt/ros/noetic/setup.bash && \
             source /opt/srdf_ws/devel/setup.bash && \
             catkin build -DCATKIN_ENABLE_TESTING=0 -j $(nproc)"

# 设置默认环境变量
RUN echo "source $REPO_WS/devel/setup.bash" >> ~/.bashrc

ENTRYPOINT ["bash"]